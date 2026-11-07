#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include "ble_phone.h"
#include "ble_quiet_policy.h"

#define OLED_SDA 0
#define OLED_SCL 1
#define OLED_ADDR 0x3C
#define RTC_ADDR 0x68
#define BTN_LEFT 10
#define BTN_SELECT 20
#define BTN_RIGHT 21

static constexpr uint8_t W = 128, H = 64;
static uint8_t fb[W * H / 8];
static uint8_t oldFb[W * H / 8];
static uint8_t newFb[W * H / 8];
static uint8_t sentFb[W * H / 8];
static bool sentFbValid=false;
static bool i2cBusFault=false;
static Preferences prefs;
static uint32_t transitionStart=0;
static uint16_t transitionDuration=213;

#include "original_english.h"
#include "original_resources.h"

struct Glyph { char c; uint8_t d[3]; };
static const Glyph font[] = {
 {'0',{31,17,31}},{'1',{0,31,0}},{'2',{29,21,23}},{'3',{21,21,31}},
 {'4',{7,4,31}},{'5',{23,21,29}},{'6',{31,21,29}},{'7',{1,1,31}},
 {'8',{31,21,31}},{'9',{23,21,31}},{':',{0,10,0}},{'.',{0,16,0}},
 {'-',{4,4,4}},{'+',{4,14,4}},{'/',{24,4,3}},{'=',{10,10,10}},
 {'A',{30,5,30}},{'B',{31,21,10}},{'C',{14,17,17}},{'D',{31,17,14}},
 {'E',{31,21,17}},{'F',{31,5,1}},{'G',{14,17,29}},{'H',{31,4,31}},
 {'I',{17,31,17}},{'J',{8,16,15}},{'K',{31,4,27}},{'L',{31,16,16}},
 {'M',{31,2,31}},{'N',{31,1,30}},{'O',{14,17,14}},{'P',{31,5,7}},
 {'Q',{14,25,30}},{'R',{31,5,26}},{'S',{18,21,9}},{'T',{1,31,1}},
 {'U',{15,16,15}},{'V',{7,24,7}},{'W',{31,8,31}},{'X',{27,4,27}},
 {'Y',{3,28,3}},{'Z',{25,21,19}},{' ',{0,0,0}},{'?',{1,21,3}}
};

static void cmd(uint8_t c){Wire.beginTransmission(OLED_ADDR);Wire.write(0);Wire.write(c);if(Wire.endTransmission()!=0)i2cBusFault=true;}
static void oledInit(){
  Wire.begin(OLED_SDA,OLED_SCL);Wire.setClock(400000);Wire.setTimeOut(20);i2cBusFault=false;delay(40);
  // Start dim. Bright full-screen boot frames were the main current spike on
  // marginal 3V3 rails and could trigger ESP_RST_BROWNOUT before BLE started.
  const uint8_t a[]={0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0,0x40,0xAD,0x8B,0xA1,0xC8,0xDA,0x12,0x81,0x10,0xD9,0x22,0xDB,0x35,0xA4,0xA6,0xAF};
  for(uint8_t c:a) cmd(c);
}
static void recoverI2cAndOled(){
  Wire.end();pinMode(OLED_SDA,INPUT_PULLUP);pinMode(OLED_SCL,OUTPUT_OPEN_DRAIN);digitalWrite(OLED_SCL,HIGH);delayMicroseconds(5);
  for(uint8_t i=0;i<9&&!digitalRead(OLED_SDA);i++){digitalWrite(OLED_SCL,LOW);delayMicroseconds(5);digitalWrite(OLED_SCL,HIGH);delayMicroseconds(5);}
  pinMode(OLED_SDA,INPUT_PULLUP);pinMode(OLED_SCL,INPUT_PULLUP);oledInit();sentFbValid=false;
}
static void flush(){
  if(i2cBusFault){recoverI2cAndOled();if(i2cBusFault)return;}
  for(uint8_t p=0;p<8;p++){
    int first=0,last=127;
    if(sentFbValid){
      while(first<128&&sentFb[p*128+first]==fb[p*128+first])first++;
      if(first==128)continue;
      while(last>first&&sentFb[p*128+last]==fb[p*128+last])last--;
    }
    uint8_t col=first+2;
    Wire.beginTransmission(OLED_ADDR);Wire.write(0x00);Wire.write(0xB0|p);Wire.write(col&0x0F);Wire.write(0x10|(col>>4));if(Wire.endTransmission()!=0){i2cBusFault=true;break;}
    for(int pos=first;pos<=last;pos+=64){
      uint8_t len=min(64,last-pos+1);
      Wire.beginTransmission(OLED_ADDR);Wire.write(0x40);Wire.write(fb+p*128+pos,len);if(Wire.endTransmission()!=0){i2cBusFault=true;break;}
    }
    if(i2cBusFault)break;
    memcpy(sentFb+p*128+first,fb+p*128+first,last-first+1);
  }
  if(i2cBusFault){sentFbValid=false;return;}sentFbValid=true;
}
static void clear(bool on=false){ memset(fb,on?0xFF:0,sizeof(fb)); }
static void px(int x,int y,bool on=true){ if(x<0||x>=W||y<0||y>=H)return; uint8_t&m=fb[x+(y>>3)*W],q=1<<(y&7); if(on)m|=q;else m&=~q; }
static void xorPx(int x,int y){if(x<0||x>=W||y<0||y>=H)return;fb[x+(y>>3)*W]^=1<<(y&7);}
static bool bufferPx(const uint8_t*b,int x,int y){return x>=0&&x<W&&y>=0&&y<H&&(b[x+(y>>3)*W]&(1<<(y&7)));}
static void composeTransition(){
  if(!transitionStart)return;
  uint32_t age=millis()-transitionStart;
  if(age>=transitionDuration){transitionStart=0;transitionDuration=213;return;}
  memcpy(newFb,fb,sizeof(fb));
  int offset=(age*64)/transitionDuration;
  clear();
  for(int y=0;y<H;y++)for(int x=0;x<W;x++){
    if(bufferPx(oldFb,x,y+offset))px(x,y);
    if(bufferPx(newFb,x,y-(64-offset)))px(x,y);
  }
}
static void line(int x0,int y0,int x1,int y1){ int dx=abs(x1-x0),sx=x0<x1?1:-1,dy=-abs(y1-y0),sy=y0<y1?1:-1,e=dx+dy; for(;;){px(x0,y0);if(x0==x1&&y0==y1)break;int e2=2*e;if(e2>=dy){e+=dy;x0+=sx;}if(e2<=dx){e+=dx;y0+=sy;}} }
static void rect(int x,int y,int w,int h,bool fill=false){ if(fill){for(int i=x;i<x+w;i++)for(int j=y;j<y+h;j++)px(i,j);}else{line(x,y,x+w-1,y);line(x,y+h-1,x+w-1,y+h-1);line(x,y,x,y+h-1);line(x+w-1,y,x+w-1,y+h-1);} }
static void circle(int cx,int cy,int r){int x=r,y=0,err=0;while(x>=y){px(cx+x,cy+y);px(cx+y,cy+x);px(cx-y,cy+x);px(cx-x,cy+y);px(cx-x,cy-y);px(cx-y,cy-x);px(cx+y,cy-x);px(cx+x,cy-y);y++;if(err<=0)err+=2*y+1;if(err>0){x--;err-=2*x+1;}}}
static const uint8_t* glyph(char c){ for(auto&g:font)if(g.c==c)return g.d;return font[sizeof(font)/sizeof(font[0])-1].d; }
static void text(const char*s,int x,int y,uint8_t z=1){ while(*s){char c=*s++;if(c>='a'&&c<='z')c-=32;auto g=glyph(c);for(int i=0;i<3;i++)for(int j=0;j<5;j++)if(g[i]&(1<<j))for(int a=0;a<z;a++)for(int b=0;b<z;b++)px(x+i*z+a,y+j*z+b);x+=4*z;} }
static void xorText(const char*s,int x,int y){while(*s){char c=*s++;if(c>='a'&&c<='z')c-=32;auto g=glyph(c);for(int i=0;i<3;i++)for(int j=0;j<5;j++)if(g[i]&(1<<j))xorPx(x+i,y+j);x+=4;}}
static void xorTextScaled(const char*s,int x,int y,uint8_t z){while(*s){char c=*s++;if(c>='a'&&c<='z')c-=32;auto g=glyph(c);for(int i=0;i<3;i++)for(int j=0;j<5;j++)if(g[i]&(1<<j))for(int a=0;a<z;a++)for(int b=0;b<z;b++)xorPx(x+i*z+a,y+j*z+b);x+=4*z;}}
static void textXY(const char*s,int x,int y,uint8_t zx,uint8_t zy){while(*s){char c=*s++;if(c>='a'&&c<='z')c-=32;auto g=glyph(c);for(int i=0;i<3;i++)for(int j=0;j<5;j++)if(g[i]&(1<<j))for(int a=0;a<zx;a++)for(int b=0;b<zy;b++)px(x+i*zx+a,y+j*zy+b);x+=4*zx;}}
static void center(const char*s,int y,uint8_t z=1){ text(s,(128-(int)strlen(s)*4*z)/2,y,z); }
static void arcText(const char*s,int x,int y,int clipCx,int clipCy,int clipR){
  while(*s){
    char c=*s++;if(c>='a'&&c<='z')c-=32;auto g=glyph(c);
    for(int i=0;i<3;i++)for(int j=0;j<5;j++)if(g[i]&(1<<j)){
      int xx=x+i,yy=y+j;
      if((xx-clipCx)*(xx-clipCx)+(yy-clipCy)*(yy-clipCy)<clipR*clipR)px(xx,yy);
    }
    x+=4;
  }
}
static void arcLine(int x0,int y0,int x1,int y1,int clipCx,int clipCy,int clipR){
  int dx=abs(x1-x0),sx=x0<x1?1:-1,dy=-abs(y1-y0),sy=y0<y1?1:-1,e=dx+dy;
  for(;;){
    if((x0-clipCx)*(x0-clipCx)+(y0-clipCy)*(y0-clipCy)<clipR*clipR)px(x0,y0);
    if(x0==x1&&y0==y1)break;int e2=2*e;if(e2>=dy){e+=dy;x0+=sx;}if(e2<=dx){e+=dx;y0+=sy;}
  }
}
static void arcTriangle(int centerX,int centerY,float angle,int clipCx,int clipCy,int clipR){
  static const int8_t vx[3]={-4,-4,5},vy[3]={-5,5,0};int x[3],y[3];float ca=cosf(angle),sa=sinf(angle);
  for(int i=0;i<3;i++){x[i]=centerX+lroundf(vx[i]*ca-vy[i]*sa);y[i]=centerY+lroundf(vx[i]*sa+vy[i]*ca);}
  arcLine(x[0],y[0],x[1],y[1],clipCx,clipCy,clipR);arcLine(x[1],y[1],x[2],y[2],clipCx,clipCy,clipR);arcLine(x[2],y[2],x[0],y[0],clipCx,clipCy,clipR);
}
static void originalBitmap(int x,int y,const uint8_t*data,int w,int h,bool inv=false){
  for(int row=0;row<h/8;row++)for(int col=0;col<w;col++){
    uint8_t v=pgm_read_byte(data+row*w+col);if(inv)v=~v;
    for(int bit=0;bit<8;bit++)if(v&(1<<bit))px(x+col,y+row*8+bit);
  }
}
static void xorOriginalChar(char c,int x,int y){
  uint8_t idx=(c>=' '&&c<=0x7f)?c-' ':0;
  for(int col=0;col<5;col++){uint8_t v=pgm_read_byte(smallFont[idx]+col);for(int bit=0;bit<8;bit++)if(v&(1<<bit))xorPx(x+col,y+bit);}
}
static void originalText(const char*s,int x,int y,bool inv=false){
  while(*s){uint8_t c=(*s>=' '&&*s<=0x7f)?*s-' ':0;originalBitmap(x,y,smallFont[c],5,8,inv);x+=7;s++;}
}
static void originalCenter(const char*s,int y){originalText(s,(128-strlen(s)*7)/2,y);}
static void title(const char*s){ center(s,2); for(int x=0;x<128;x++)for(int y=0;y<10;y++)px(x,y,!((fb[x+(y>>3)*128]>>(y&7))&1)); }
static void footer(const char*l,const char*m,const char*r){ text(l,1,58); center(m,58); text(r,127-strlen(r)*4,58); }

enum Key:uint8_t{NONE,LEFT,SELECT,RIGHT,LONG_SELECT};
struct Button{uint8_t pin;bool last;uint32_t down,debounce;bool longFired;};
static Button buttons[3]={{BTN_LEFT,1,0,0,false},{BTN_SELECT,1,0,0,false},{BTN_RIGHT,1,0,0,false}};
static Key readKey(){
  uint32_t now=millis();
  for(int i=0;i<3;i++){
    bool v=digitalRead(buttons[i].pin);
    if(v!=buttons[i].last&&now-buttons[i].debounce>18){
      buttons[i].debounce=now;buttons[i].last=v;
      if(!v){buttons[i].down=now;buttons[i].longFired=false;}
      else if(buttons[i].down){
        uint32_t held=now-buttons[i].down;buttons[i].down=0;
        bool wasLong=buttons[i].longFired;buttons[i].longFired=false;
        if(!wasLong&&held>30)return i==0?LEFT:i==1?SELECT:RIGHT;
      }
    }
    if(i==1&&!v&&buttons[i].down&&!buttons[i].longFired&&now-buttons[i].down>=400){
      buttons[i].longFired=true;
      return LONG_SELECT;
    }
  }
  return NONE;
}

enum Screen:uint8_t{WATCH,MENU,APPS,GAMES,ALARMS,SETTINGS,DIAG,STOPWATCH,CALC,TORCH,BREAKOUT,CARDODGE,SETTIME,SLEEPSET,DISPSET,ABOUT,CONNECTION};
static Screen screen=WATCH;
static constexpr uint8_t FACE_COUNT=11;
static uint8_t cursor=0,watchface=0,brightness=1;
static uint8_t sleepSetting=5;
static bool invertDisplay=false,alarmOn=false,showFPS=false;
static uint8_t alarmH=7,alarmM=0;
static uint8_t setH=12,setM=0;
static uint8_t dateDay=1,dateMonth=10,dateDow=1;
static uint16_t dateYear=2024;
static uint32_t clockBase=0,lastSave=0;
static uint32_t lastInteraction=0;
static bool torchBrightnessForced=false;
static bool rtcPresent=false,rtcTimeValid=false,rtcInitializedFromFallback=false;
static uint32_t rtcLastSync=0;
static bool displaySleeping=false;
static bool notificationVisible=false;
static uint32_t notificationShownAt=0;
static BlePhoneNotification currentNotification={};
static bool bleEnabled=true;
static BleQuietPolicy bleQuiet;
static bool bleInitAttempted=false;
static bool bleInitSucceeded=false;
static uint32_t bleInitAt=0;
static bool bleResetBonds=false;
static bool bootPowerGuard=true;
static uint32_t bootPowerReleaseAt=0;

// SH1106 contrast above ~0x80 caused the shared 3V3 rail to cross the C3
// brownout threshold on this Super Mini. These levels remain visibly distinct.
static uint8_t configuredContrast(){return brightness==0?0x18:brightness==1?0x30:brightness==2?0x50:0x70;}
static void applyConfiguredBrightness(){cmd(0x81);cmd(bootPowerGuard?min(configuredContrast(),uint8_t(0x18)):configuredContrast());}
static void releaseBootPowerGuard(){
  uint16_t target=configuredContrast(),step=(target>0x18)?(target-0x18)/10:1;if(!step)step=1;
  for(uint16_t value=0x18;value<target;value+=step){cmd(0x81);cmd(uint8_t(value));delay(18);}
  bootPowerGuard=false;applyConfiguredBrightness();
}
static void forceTorchBrightness(){if(torchBrightnessForced)return;cmd(0x81);cmd(0x90);torchBrightnessForced=true;}
static void restoreTorchBrightness(){if(!torchBrightnessForced)return;applyConfiguredBrightness();torchBrightnessForced=false;}

static uint32_t clockSeconds(){ return (uint32_t(setH)*3600+uint32_t(setM)*60+(millis()-clockBase)/1000)%86400; }
static void getTime(uint8_t&h,uint8_t&m,uint8_t&s){uint32_t t=clockSeconds();h=t/3600;m=(t/60)%60;s=t%60;}
static void saveConfig(){
  uint8_t h,m,s;getTime(h,m,s);prefs.putUChar("hour",h);prefs.putUChar("minute",m);prefs.putUChar("second",s);
  prefs.putUChar("face",watchface);prefs.putUChar("facever",55);prefs.putBool("invert",invertDisplay);prefs.putUChar("bright",brightness);
  prefs.putBool("alarm",alarmOn);prefs.putUChar("alarmH",alarmH);prefs.putUChar("alarmM",alarmM);
  prefs.putBool("fps",showFPS);prefs.putUChar("day",dateDay);prefs.putUChar("month",dateMonth);prefs.putUChar("dow",dateDow);prefs.putUShort("year",dateYear);
  prefs.putUChar("sleep",sleepSetting);prefs.putBool("ble",bleEnabled);
  setH=h;setM=m;clockBase=millis()-uint32_t(s)*1000;lastSave=millis();
}
static void loadConfig(){
  prefs.begin("nwatch",false);setH=prefs.getUChar("hour",12);setM=prefs.getUChar("minute",0);uint8_t s=prefs.getUChar("second",0);
  uint8_t storedFace=prefs.getUChar("face",0);
  if(prefs.getUChar("facever",0)<55){const uint8_t migrate[13]={0,1,2,3,4,4,5,6,7,7,8,9,10};storedFace=migrate[min(storedFace,uint8_t(12))];prefs.putUChar("facever",55);prefs.putUChar("face",storedFace);}
  watchface=storedFace%FACE_COUNT;invertDisplay=prefs.getBool("invert",false);brightness=prefs.getUChar("bright",1)%4;
  // Native NimBLE and Bluedroid use different bond stores. Do not erase NVS.
  // First pairing on this stack is performed after forgetting the old iPhone bond.
  bleResetBonds=false;
  alarmOn=prefs.getBool("alarm",false);alarmH=prefs.getUChar("alarmH",7);alarmM=prefs.getUChar("alarmM",0);
  showFPS=prefs.getBool("fps",false);dateDay=constrain(prefs.getUChar("day",1),1,31);dateMonth=constrain(prefs.getUChar("month",10),1,12);dateDow=prefs.getUChar("dow",1)%7;dateYear=constrain((int)prefs.getUShort("year",2024),2024,2099);
  sleepSetting=min(prefs.getUChar("sleep",5),uint8_t(5));bleEnabled=prefs.getBool("ble",true);
  clockBase=millis()-uint32_t(s)*1000;cmd(invertDisplay?0xA7:0xA6);applyConfiguredBrightness();
}

static uint8_t fromBcd(uint8_t value){return (value>>4)*10+(value&0x0F);}
static uint8_t toBcd(uint8_t value){return ((value/10)<<4)|(value%10);}
static bool rtcReadBytes(uint8_t reg,uint8_t*data,uint8_t length){
  Wire.beginTransmission(RTC_ADDR);Wire.write(reg);if(Wire.endTransmission(false)!=0)return false;
  if(Wire.requestFrom(uint8_t(RTC_ADDR),length)!=length)return false;
  for(uint8_t i=0;i<length;i++)data[i]=Wire.read();return true;
}
static bool rtcWriteByte(uint8_t reg,uint8_t value){
  Wire.beginTransmission(RTC_ADDR);Wire.write(reg);Wire.write(value);return Wire.endTransmission()==0;
}
static uint8_t calculateDow(uint16_t year,uint8_t month,uint8_t day){
  static const uint8_t offsets[]={0,3,2,5,0,3,5,1,4,6,2,4};uint16_t y=year-(month<3);
  uint8_t sundayFirst=(y+y/4-y/100+y/400+offsets[month-1]+day)%7;return (sundayFirst+6)%7;
}
static bool rtcReadTime(uint8_t&hour,uint8_t&minute,uint8_t&second,uint8_t&day,uint8_t&month,uint16_t&year,bool&valid){
  uint8_t data[7],status;if(!rtcReadBytes(0x00,data,7)||!rtcReadBytes(0x0F,&status,1))return false;
  second=fromBcd(data[0]&0x7F);minute=fromBcd(data[1]&0x7F);
  if(data[2]&0x40){hour=fromBcd(data[2]&0x1F)%12;if(data[2]&0x20)hour+=12;}else hour=fromBcd(data[2]&0x3F);
  day=fromBcd(data[4]&0x3F);month=fromBcd(data[5]&0x1F);year=2000+fromBcd(data[6])+((data[5]&0x80)?100:0);
  valid=!(status&0x80)&&second<60&&minute<60&&hour<24&&day>=1&&day<=31&&month>=1&&month<=12&&year>=2024&&year<=2099;
  return true;
}
static bool rtcWriteCurrent(){
  uint8_t hour,minute,second;getTime(hour,minute,second);dateDow=calculateDow(dateYear,dateMonth,dateDay);
  uint8_t control;if(rtcReadBytes(0x0E,&control,1))rtcWriteByte(0x0E,control&0x7F);
  Wire.beginTransmission(RTC_ADDR);Wire.write(uint8_t(0x00));Wire.write(toBcd(second));Wire.write(toBcd(minute));Wire.write(toBcd(hour));
  Wire.write(toBcd(dateDow+1));Wire.write(toBcd(dateDay));Wire.write(toBcd(dateMonth));Wire.write(toBcd(dateYear%100));
  if(Wire.endTransmission()!=0){rtcPresent=false;rtcTimeValid=false;return false;}
  uint8_t status;if(rtcReadBytes(0x0F,&status,1))rtcWriteByte(0x0F,status&0x7F);
  rtcPresent=true;rtcTimeValid=true;rtcLastSync=millis();return true;
}
static bool rtcSyncFromHardware(){
  uint8_t hour,minute,second,day,month;uint16_t year;bool valid=false;
  if(!rtcReadTime(hour,minute,second,day,month,year,valid)){rtcPresent=false;rtcTimeValid=false;return false;}
  rtcPresent=true;rtcTimeValid=valid;if(!valid)return false;
  setH=hour;setM=minute;clockBase=millis()-uint32_t(second)*1000;dateDay=day;dateMonth=month;dateYear=year;dateDow=calculateDow(year,month,day);rtcLastSync=millis();return true;
}
static void rtcInitialize(){
  Wire.beginTransmission(RTC_ADDR);rtcPresent=Wire.endTransmission()==0;rtcInitializedFromFallback=false;
  if(!rtcPresent)return;
  if(!rtcSyncFromHardware()){rtcInitializedFromFallback=rtcWriteCurrent();}
}
static void serviceRtc(){
  if(screen==SETTIME||millis()-rtcLastSync<5000)return;rtcLastSync=millis();rtcSyncFromHardware();
}

static char latinFromUnicode(uint32_t cp){
  if(cp==0x0110||cp==0x0111)return 'D';
  if((cp>=0x1EA0&&cp<=0x1EB7)||cp==0x00C0||cp==0x00C1||cp==0x00C2||cp==0x00C3||cp==0x00C4||cp==0x00E0||cp==0x00E1||cp==0x00E2||cp==0x00E3||cp==0x00E4)return 'A';
  if((cp>=0x1EB8&&cp<=0x1EC7)||cp==0x00C8||cp==0x00C9||cp==0x00CA||cp==0x00CB||cp==0x00E8||cp==0x00E9||cp==0x00EA||cp==0x00EB)return 'E';
  if((cp>=0x1EC8&&cp<=0x1ECB)||cp==0x00CC||cp==0x00CD||cp==0x00CE||cp==0x00CF||cp==0x00EC||cp==0x00ED||cp==0x00EE||cp==0x00EF)return 'I';
  if((cp>=0x1ECC&&cp<=0x1EE3)||cp==0x00D2||cp==0x00D3||cp==0x00D4||cp==0x00D5||cp==0x00D6||cp==0x00F2||cp==0x00F3||cp==0x00F4||cp==0x00F5||cp==0x00F6)return 'O';
  if((cp>=0x1EE4&&cp<=0x1EF1)||cp==0x00D9||cp==0x00DA||cp==0x00DB||cp==0x00DC||cp==0x00F9||cp==0x00FA||cp==0x00FB||cp==0x00FC)return 'U';
  if((cp>=0x1EF2&&cp<=0x1EF9)||cp==0x00DD||cp==0x00FD||cp==0x00FF)return 'Y';
  return 0;
}
static void displayAscii(const char*source,char*dest,size_t capacity){
  if(!capacity)return;size_t out=0;const uint8_t*p=(const uint8_t*)source;
  while(*p&&out+1<capacity){
    uint32_t cp;uint8_t used=1;
    if(*p<0x80)cp=*p;
    else if((*p&0xE0)==0xC0&&p[1]){cp=((*p&0x1F)<<6)|(p[1]&0x3F);used=2;}
    else if((*p&0xF0)==0xE0&&p[1]&&p[2]){cp=((*p&0x0F)<<12)|((p[1]&0x3F)<<6)|(p[2]&0x3F);used=3;}
    else{p++;continue;}
    p+=used;char c=cp<128?char(cp):latinFromUnicode(cp);if(!c)continue;
    if(c<' '||c>126)c=' ';if(c>='a'&&c<='z')c-=32;
    if(c==' '&&out&&dest[out-1]==' ')continue;dest[out++]=c;
  }
  while(out&&dest[out-1]==' ')out--;dest[out]=0;
}
static const char*notificationAppName(const char*identifier){
  if(strstr(identifier,"MobileSMS"))return "MESSAGES";
  if(strstr(identifier,"mobilephone")||strstr(identifier,"MobilePhone"))return "PHONE";
  if(strstr(identifier,"youtube")||strstr(identifier,"YouTube"))return "YOUTUBE";
  if(strstr(identifier,"MobileCal")||strstr(identifier,"calendar"))return "CALENDAR";
  if(strstr(identifier,"zalo")||strstr(identifier,"Zalo"))return "ZALO";
  if(strstr(identifier,"Messenger")||strstr(identifier,"messenger"))return "MESSENGER";
  if(strstr(identifier,"WhatsApp")||strstr(identifier,"whatsapp"))return "WHATSAPP";
  if(strstr(identifier,"mobilemail")||strstr(identifier,"MobileMail"))return "MAIL";
  if(strstr(identifier,"facebook")||strstr(identifier,"Facebook"))return "FACEBOOK";
  return identifier;
}
static void wakeDisplay(){
  if(displaySleeping){cmd(0xAF);sentFbValid=false;displaySleeping=false;}
  lastInteraction=millis();
}
static void processBlePhone(){
  if(!bleEnabled||!bleInitSucceeded)return;
  blePhonePoll();
  BlePhoneTime phoneTime;
  if(blePhoneTakeTime(&phoneTime)){
    setH=phoneTime.hour;setM=phoneTime.minute;clockBase=millis()-uint32_t(phoneTime.second)*1000;
    dateDay=phoneTime.day;dateMonth=phoneTime.month;dateYear=phoneTime.year;dateDow=calculateDow(dateYear,dateMonth,dateDay);
    rtcWriteCurrent();saveConfig();
  }
  BlePhoneNotification incoming;
  if(blePhoneTakeNotification(&incoming)){
    currentNotification=incoming;notificationVisible=true;notificationShownAt=millis();wakeDisplay();
    memcpy(oldFb,fb,sizeof(fb));transitionDuration=150;transitionStart=millis();
  }
  if(notificationVisible&&millis()-notificationShownAt>=12000){notificationVisible=false;lastInteraction=millis();}
}

static void updateBleQuietGuard(){
  bool was=bleQuiet.active;
  bool pressed=!digitalRead(BTN_LEFT)||!digitalRead(BTN_SELECT)||!digitalRead(BTN_RIGHT);
  bleQuiet.update(bleEnabled&&bleInitSucceeded,blePhoneConnected(),blePhoneReady(),displaySleeping,pressed,millis());
  if(bleQuiet.active&&!was){cmd(0xAE);Serial.println("[POWER] OLED QUIET - PAIR/DISCOVER (max 20s, any button cancels)");}
  else if(was&&!bleQuiet.active){if(!displaySleeping){cmd(0xAF);sentFbValid=false;}Serial.println("[POWER] OLED QUIET END");}
}

static void drawBluetoothSymbol(int x,int y){
  line(x+7,y,x+7,y+7);line(x+7,y,x+11,y+2);line(x+11,y+2,x+7,y+4);line(x+7,y+4,x+11,y+6);line(x+11,y+6,x+7,y+7);line(x+3,y+2,x+10,y+6);line(x+3,y+6,x+10,y+2);
}
static uint8_t rollPrev[6]={1,2,0,0,0,0},rollNow[6]={1,2,0,0,0,0},rollY=0,lastRollSecond=255;
static uint8_t lastScanMinute=255;
static uint32_t scanAnimationStart=0;
static uint32_t faceSelectorStart=0;
static uint32_t triangleSpinStart=0;
static int8_t faceScrollDir=1;
static bool faceSavePending=false;
static uint32_t faceSaveAt=0;
static const char*faceNames[]={"NORMAL","SCAN","ANALOG","PIXEL","MINIMAL","DATA","RAIN","MATRIX","CLOCK","BIGTIME","CUBE"};
static uint32_t sleepTimeoutMs();
static void sleepCountdownLabel(char*out,size_t capacity){
  uint32_t timeout=sleepTimeoutMs();
  if(!timeout){snprintf(out,capacity,"SLP ON");return;}
  uint32_t age=millis()-lastInteraction;
  uint32_t seconds=age>=timeout?0:(timeout-age+999)/1000;
  if(seconds>=60)snprintf(out,capacity,"SLP%luM",(unsigned long)((seconds+59)/60));
  else snprintf(out,capacity,"SLP%lu",(unsigned long)seconds);
}
static void originalTickerDigit(int x,int y,const uint8_t*base,int w,int h,uint8_t now,uint8_t prev,bool moving){
  int size=w*h/8;if(!moving){originalBitmap(x,y,base+now*size,w,h);return;}
  originalBitmap(x,y-rollY,base+prev*size,w,h);originalBitmap(x,y+h+4-rollY,base+now*size,w,h);
}
static void drawOriginalWatch(uint8_t h,uint8_t m,uint8_t s){
  uint8_t d[6]={uint8_t(h/10),uint8_t(h%10),uint8_t(m/10),uint8_t(m%10),uint8_t(s/10),uint8_t(s%10)};
  if(s!=lastRollSecond){memcpy(rollPrev,rollNow,6);memcpy(rollNow,d,6);rollY=1;lastRollSecond=s;}
  if(rollY){if(rollY<4)rollY+=2;else if(rollY<10)rollY+=4;else if(rollY<23)rollY+=7;else rollY+=5;if(rollY>=28)rollY=0;}
  static const char*dows[]={"MON","TUE","WED","THU","FRI","SAT","SUN"};
  static const char*months[]={"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
  char date[24];snprintf(date,sizeof(date),"%s %02u %s %04u",dows[dateDow],dateDay,months[dateMonth-1],dateYear);originalCenter(date,0);
  originalTickerDigit(1,20,(const uint8_t*)midFont,19,24,d[0],rollPrev[0],rollY&&d[0]!=rollPrev[0]);
  originalTickerDigit(24,20,(const uint8_t*)midFont,19,24,d[1],rollPrev[1],rollY&&d[1]!=rollPrev[1]);
  originalBitmap(49,20,colon,6,24);
  originalTickerDigit(60,20,(const uint8_t*)midFont,19,24,d[2],rollPrev[2],rollY&&d[2]!=rollPrev[2]);
  originalTickerDigit(83,20,(const uint8_t*)midFont,19,24,d[3],rollPrev[3],rollY&&d[3]!=rollPrev[3]);
  originalTickerDigit(104,28,(const uint8_t*)small2Font,11,16,d[4],rollPrev[4],rollY&&d[4]!=rollPrev[4]);
  originalTickerDigit(116,28,(const uint8_t*)small2Font,11,16,d[5],rollPrev[5],rollY!=0);
  drawBluetoothSymbol(1,56);originalText(!bleEnabled?"OFF":blePhoneConnected()?"LINK":"PAIR",17,56);
  char sleepLabel[9];sleepCountdownLabel(sleepLabel,sizeof(sleepLabel));originalText(sleepLabel,128-int(strlen(sleepLabel))*7,56);
}
static void drawMidTime(uint8_t h,uint8_t m,int y=20){
  uint8_t d[4]={uint8_t(h/10),uint8_t(h%10),uint8_t(m/10),uint8_t(m%10)};
  originalBitmap(8,y,(const uint8_t*)midFont+d[0]*57,19,24);originalBitmap(31,y,(const uint8_t*)midFont+d[1]*57,19,24);
  originalBitmap(55,y,colon,6,24);originalBitmap(67,y,(const uint8_t*)midFont+d[2]*57,19,24);originalBitmap(90,y,(const uint8_t*)midFont+d[3]*57,19,24);
}
static void drawFaceSelector(){
  if(!faceSelectorStart)return;
  const uint32_t slideMs=173,holdMs=600,endMs=900;
  uint32_t age=millis()-faceSelectorStart;
  if(age>=endMs){faceSelectorStart=0;return;}
  memcpy(newFb,fb,sizeof(fb));
  const int selectorCx=135,selectorCy=32,selectorRadius=62;
  const int splitX=selectorCx-selectorRadius;

  // The left watchface follows the selector text: next scrolls upward and the
  // previous face scrolls downward. The right half remains frozen behind the
  // stationary semicircle until the hold finishes.
  float slideT=min(1.0f,float(age)/slideMs);slideT=slideT*slideT*(3.0f-2.0f*slideT);
  int shift=lroundf(slideT*64.0f);
  for(int y=0;y<H;y++)for(int x=0;x<splitX;x++){
    bool v;
    if(shift>=64)v=bufferPx(newFb,x,y);
    else if(faceScrollDir>0)v=bufferPx(oldFb,x,y+shift)||bufferPx(newFb,x,y-(64-shift));
    else v=bufferPx(oldFb,x,y-shift)||bufferPx(newFb,x,y+(64-shift));
    px(x,y,v);
  }
  if(age<holdMs)for(int y=0;y<H;y++)for(int x=splitX;x<W;x++)px(x,y,bufferPx(oldFb,x,y));

  int r=selectorRadius,cx=selectorCx,cy=selectorCy;
  if(age>holdMs)r=selectorRadius-int(((age-holdMs)*selectorRadius)/(endMs-holdMs));
  r=max(1,r);
  for(int y=max(0,cy-r+1);y<min(64,cy+r);y++)for(int x=max(splitX+1,cx-r+1);x<128;x++)if((x-cx)*(x-cx)+(y-cy)*(y-cy)<(r-1)*(r-1))px(x,y,false);
  circle(cx,cy,r);

  if(r>18){
    int scroll=lroundf(faceScrollDir*13.0f*(1.0f-slideT));
    for(int rel=-2;rel<=2;rel++){
      int y=30+rel*13+scroll,midY=y+2,dy=midY-cy;
      int inside=(r-3)*(r-3)-dy*dy;if(inside<=0)continue;
      int x=cx-int(sqrtf(inside))+14;
      arcText(faceNames[(watchface+rel+FACE_COUNT)%FACE_COUNT],x,y,cx,cy,r-2);
    }
    uint32_t spinAge=millis()-triangleSpinStart;
    float angle=spinAge<127?spinAge*0.137f:0.0f;
    arcTriangle(82,32,angle,cx,cy,r-2);
  }
}
static void drawMatrixFace(uint8_t h,uint8_t m,uint8_t s){
  uint8_t d[6]={uint8_t(h/10),uint8_t(h%10),uint8_t(m/10),uint8_t(m%10),uint8_t(s/10),uint8_t(s%10)};
  originalBitmap(1,20,(const uint8_t*)midFont[d[0]],19,24);originalBitmap(24,20,(const uint8_t*)midFont[d[1]],19,24);
  originalBitmap(49,20,colon,6,24);originalBitmap(60,20,(const uint8_t*)midFont[d[2]],19,24);originalBitmap(83,20,(const uint8_t*)midFont[d[3]],19,24);
  originalBitmap(104,28,(const uint8_t*)small2Font[d[4]],11,16);originalBitmap(116,28,(const uint8_t*)small2Font[d[5]],11,16);

  static const char chars[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  static const int8_t laneX[10]={0,10,20,41,52,72,92,104,116,123};
  static const uint8_t laneSpeed[10]={29,53,37,71,43,31,67,47,83,59};
  static const uint8_t laneLength[10]={7,5,9,6,8,5,9,7,6,8};
  static const uint8_t lanePhase[10]={8,47,19,63,31,74,12,55,27,68};
  uint32_t now=millis();
  for(int lane=0;lane<10;lane++){
    bool large=lane==2||lane==7;int gap=large?12:8,len=laneLength[lane],cycle=72+len*gap;
    uint32_t tick=now/laneSpeed[lane];int head=(tick+lanePhase[lane])%cycle-gap;
    for(int n=0;n<len;n++){
      int y=head-n*gap;if(y<-(large?10:8)||y>=64)continue;
      char c=chars[(lane*11+n*7+tick/(3+(lane%4)))%36];
      if(large){char one[2]={c,0};xorTextScaled(one,laneX[lane],y,2);}else xorOriginalChar(c,laneX[lane],y);
    }
  }
}
static void drawScanFace(uint8_t h,uint8_t m,uint8_t s){
  if(lastScanMinute==255)lastScanMinute=m;
  else if(m!=lastScanMinute){lastScanMinute=m;scanAnimationStart=millis();}
  int offset=0;
  if(scanAnimationStart){
    uint32_t age=millis()-scanAnimationStart;
    if(age>=320)scanAnimationStart=0;
    else{float t=age<160?float(age)/160.0f:float(age-160)/160.0f;t=t*t*(3.0f-2.0f*t);offset=age<160?-lroundf(36.0f*t):-lroundf(36.0f*(1.0f-t));}
  }
  drawMidTime(h,m,14);for(int i=0;i<6;i++)line(0,46+i*3+offset,127,46+i*3+offset);char b[4];snprintf(b,sizeof(b),"%02u",s);originalText(b,112,0);
}
static void drawPixelFace(uint8_t h,uint8_t m,uint8_t s){
  char b[4];snprintf(b,sizeof(b),"%02u",h);textXY(b,0,4,8,11);
  snprintf(b,sizeof(b),"%02u",m);textXY(b,64,3,8,5);
  snprintf(b,sizeof(b),"%02u",s);textXY(b,64,36,8,5);
}
static void drawNumberedAnalog(uint8_t h,uint8_t m,uint8_t s){
  const int cx=64,cy=32,r=30;circle(cx,cy,r);text("12",60,2);text("3",88,30);text("6",62,56);text("9",38,30);
  for(int mark=1;mark<12;mark++)if(mark!=3&&mark!=6&&mark!=9){float q=mark*6.2831853f/12-1.5708f;px(cx+lroundf(cosf(q)*26),cy+lroundf(sinf(q)*26));}
  float sec=s*6.2831853f/60-1.5708f,minu=(m+s/60.0f)*6.2831853f/60-1.5708f,hour=((h%12)+m/60.0f)*6.2831853f/12-1.5708f;
  line(cx,cy,cx+lroundf(cosf(hour)*14),cy+lroundf(sinf(hour)*14));line(cx,cy,cx+lroundf(cosf(minu)*21),cy+lroundf(sinf(minu)*21));line(cx,cy,cx+lroundf(cosf(sec)*24),cy+lroundf(sinf(sec)*24));rect(cx-1,cy-1,3,3,true);
}
static void drawBigPixelTime(uint8_t h,uint8_t m){char b[8];snprintf(b,sizeof(b),"%02u:%02u",h,m);center(b,17,6);}
static void drawCubeFace(uint8_t h,uint8_t m){
  float ax=millis()*0.00085f,ay=millis()*0.00105f,az=millis()*0.00065f;
  float cax=cosf(ax),sax=sinf(ax),cay=cosf(ay),say=sinf(ay),caz=cosf(az),saz=sinf(az);int sx[8],sy[8];
  for(int i=0;i<8;i++){
    float x=(i&1)?1.0f:-1.0f,y=(i&2)?1.0f:-1.0f,z=(i&4)?1.0f:-1.0f;
    float y1=y*cax-z*sax,z1=y*sax+z*cax,x2=x*cay+z1*say,z2=-x*say+z1*cay,x3=x2*caz-y1*saz,y3=x2*saz+y1*caz;
    float scale=58.0f/(z2+4.2f);sx[i]=32+lroundf(x3*scale);sy[i]=32+lroundf(y3*scale);
  }
  for(int i=0;i<8;i++)for(int bit=0;bit<3;bit++){int j=i^(1<<bit);if(i<j)line(sx[i],sy[i],sx[j],sy[j]);}
  char b[4];snprintf(b,sizeof(b),"%02u",h);text(b,83,2,5);snprintf(b,sizeof(b),"%02u",m);text(b,83,35,5);
}
static void drawWatch(){
  uint8_t h,m,s;getTime(h,m,s);char b[24];
  if(watchface==0)drawOriginalWatch(h,m,s);
  else if(watchface==1)drawScanFace(h,m,s);
  else if(watchface==2){circle(32,33,26);for(int a=0;a<12;a++){float q=a*6.2831853f/12;px(32+int(cosf(q)*23),33+int(sinf(q)*23));}float mh=(m/60.0f)*6.2831853f-1.5708f,hh=((h%12+m/60.0f)/12)*6.2831853f-1.5708f;line(32,33,32+int(cosf(mh)*20),33+int(sinf(mh)*20));line(32,33,32+int(cosf(hh)*13),33+int(sinf(hh)*13));snprintf(b,sizeof(b),"%02u:%02u",h,m);text(b,67,18,2);snprintf(b,sizeof(b),"SEC %02u",s);originalText(b,72,42);}
  else if(watchface==3)drawPixelFace(h,m,s);
  else if(watchface==4){drawMidTime(h,m,20);snprintf(b,sizeof(b),"%02u",s);originalText(b,112,52);}
  else if(watchface==5){originalText("ESP32-C3",0,0);snprintf(b,sizeof(b),"TIME  %02u:%02u:%02u",h,m,s);originalText(b,0,12);snprintf(b,sizeof(b),"DATE  %02u/%02u/%04u",dateDay,dateMonth,dateYear);originalText(b,0,24);originalText("OLED  SH1106",0,36);char sleepLabel[9];sleepCountdownLabel(sleepLabel,sizeof(sleepLabel));snprintf(b,sizeof(b),"SLEEP %s",sleepLabel+3);originalText(b,0,48);originalBitmap(112,0,battIconFull,16,8);}
  else if(watchface==6){for(int x=0;x<128;x+=6){uint32_t seed=(x*37+millis()/67);int len=8+(seed%40);for(int y=seed%16;y<min(64,int(seed%16+len));y+=4)px(x,y);}snprintf(b,sizeof(b),"%02u:%02u",h,m);text(b,32,23,2);}
  else if(watchface==7)drawMatrixFace(h,m,s);
  else if(watchface==8)drawNumberedAnalog(h,m,s);
  else if(watchface==9)drawBigPixelTime(h,m);
  else drawCubeFace(h,m);
  drawFaceSelector();
}
static const char*mainItems[]={"SETTINGS","STOPWATCH","GAMES","ALARMS","CONNECT","APPS","EXIT"};
static const char*appItems[]={"STOPWATCH","CALCULATOR","TORCH"};
static const char*gameItems[]={"BREAKOUT","CAR DODGE"};
static const char*settingItems[]={"TIME & DATE","SLEEP","DIAGNOSTICS","DISPLAY","BACK"};
static const char*displayItems[]={"BRIGHTNESS","INVERT","FPS","BACK"};
static void drawList(const char*name,const char**items,uint8_t count){
  title(name);uint8_t scroll=(cursor/5)*5;
  for(int row=0;row<5;row++){
    int idx=scroll+row;if(idx>=count)break;int y=12+row*9;
    if(idx==cursor){rect(1,y-1,126,8,true);for(size_t i=0;i<strlen(items[idx]);i++){char one[2]={items[idx][i],0};text(one,5+i*4,y,1);for(int yy=y;yy<y+5;yy++)for(int xx=5+i*4;xx<8+i*4;xx++)px(xx,yy,!((fb[xx+(yy>>3)*128]>>(yy&7))&1));}}
    else text(items[idx],5,y);
  }
  footer("UP","OPEN","DOWN");
}

// Original NWatch-style horizontal icon carousel. The icon order is mapped to
// the Stage 1 menu targets so no working application becomes unreachable.
static uint32_t iconMenuIntroStart=0;
static Screen iconMenuIntroScreen=WATCH;
static bool isIconMenu(Screen value){return value==MENU||value==APPS||value==SETTINGS||value==DISPSET;}
static uint8_t iconMenuCount(Screen value){
  if(value==MENU)return 7;if(value==APPS)return 3;if(value==SETTINGS)return 5;if(value==DISPSET)return 4;return 0;
}
static uint16_t iconMenuIntroDuration(){
  uint8_t count=iconMenuCount(iconMenuIntroScreen);
  return count?uint16_t((80+(count-1)*48)*1000UL/360UL):0;
}
static bool iconMenuIntroActive(){return iconMenuIntroStart&&iconMenuIntroScreen==screen&&millis()-iconMenuIntroStart<iconMenuIntroDuration();}
static int iconMenuIntroX(uint8_t index,uint8_t count){
  uint32_t age=millis()-iconMenuIntroStart;uint16_t duration=iconMenuIntroDuration();
  float t=duration?min(1.0f,float(age)/duration):1.0f;
  int rank=count-1-index,travel=80+(count-1)*48;
  return -32-rank*48+lroundf(t*travel);
}
static int curvedIconY(int iconX){
  float distance=min(48,abs((iconX+16)-64));
  const float radius=132.5f;
  int lift=lroundf(radius-sqrtf(radius*radius-distance*distance));
  return 16-lift;
}
static int carouselX=64;
static void drawAppsGridIcon(int x,int y){for(int row=0;row<2;row++)for(int col=0;col<2;col++){int xx=x+3+col*15,yy=y+3+row*15;rect(xx,yy,11,11);rect(xx+3,yy+3,5,5,true);}}
static void drawBluetoothMenuIcon(int x,int y){
  circle(x+16,y+16,14);line(x+15,y+4,x+15,y+28);line(x+15,y+4,x+23,y+11);line(x+23,y+11,x+15,y+17);line(x+15,y+17,x+23,y+23);line(x+23,y+23,x+15,y+28);line(x+7,y+9,x+21,y+23);line(x+7,y+23,x+21,y+9);
}
static void drawOriginalMain(){
  const uint8_t*icons[]={menu_settings,menu_stopwatch,menu_games,menu_alarm,nullptr,nullptr,menu_exit};
  originalCenter("MAIN MENU",0);
  originalBitmap(46,14,selectbar_top,36,8);
  originalBitmap(46,42,selectbar_bottom,36,8);
  int target=64-48*cursor;
  int distance=target-carouselX;
  if(distance){
    int step=((abs(distance)/6+1)*3+1)/2;
    if(step>15)step=15;
    carouselX+=distance>0?step:-step;
  }
  bool intro=iconMenuIntroActive();int x=carouselX-16;
  for(int i=0;i<7;i++,x+=48){
    if(intro)x=iconMenuIntroX(i,7);
    if(x>-32&&x<128){int y=curvedIconY(x);if(i==4)drawBluetoothMenuIcon(x,y);else if(i==5)drawAppsGridIcon(x,y);else originalBitmap(x,y,icons[i],32,32);}
  }
  originalCenter(mainItems[cursor],56);
}

static int appsCarouselX=64;
static void drawOriginalApps(){
  const uint8_t*icons[]={menu_stopwatch,menu_calc,menu_torch};
  originalCenter("APPS",0);
  originalBitmap(46,14,selectbar_top,36,8);
  originalBitmap(46,42,selectbar_bottom,36,8);
  int target=64-48*cursor;
  int distance=target-appsCarouselX;
  if(distance){
    int step=((abs(distance)/6+1)*3+1)/2;
    if(step>15)step=15;
    appsCarouselX+=distance>0?step:-step;
  }
  bool intro=iconMenuIntroActive();int x=appsCarouselX-16;
  for(int i=0;i<3;i++,x+=48){
    if(intro)x=iconMenuIntroX(i,3);
    if(x>-32&&x<128)originalBitmap(x,curvedIconY(x),icons[i],32,32);
  }
  originalCenter(appItems[cursor],56);
}
static void drawOriginalStringList(const char*name,const char**items,uint8_t count){
  originalCenter(name,0);
  uint8_t scroll=(cursor/6)*6;
  for(uint8_t row=0;row<6;row++){
    uint8_t idx=scroll+row;if(idx>=count)break;
    uint8_t y=8+row*8;
    if(idx==cursor)originalText(">",0,y);
    originalText(items[idx],7,y);
  }
}
static int settingsCarouselX=64;
static void drawOriginalSettings(){
  const uint8_t*icons[]={menu_timedate,menu_sleep,menu_diagnostic,menu_display,menu_exit};
  originalCenter("SETTINGS",0);
  originalBitmap(46,14,selectbar_top,36,8);
  originalBitmap(46,42,selectbar_bottom,36,8);
  int target=64-48*cursor;
  int distance=target-settingsCarouselX;
  if(distance){
    int step=((abs(distance)/6+1)*3+1)/2;if(step>15)step=15;
    settingsCarouselX+=distance>0?step:-step;
  }
  bool intro=iconMenuIntroActive();int x=settingsCarouselX-16;
  for(int i=0;i<5;i++,x+=48){if(intro)x=iconMenuIntroX(i,5);if(x>-32&&x<128)originalBitmap(x,curvedIconY(x),icons[i],32,32);}
  originalCenter(settingItems[cursor],56);
}
static int displayCarouselX=64;
static void drawVideoDisplay(){
  const uint8_t*icons[]={menu_brightness[brightness],menu_invert,menu_diagnostic,menu_exit};
  originalCenter("DISPLAY",0);originalBitmap(46,14,selectbar_top,36,8);originalBitmap(46,42,selectbar_bottom,36,8);
  int target=64-48*cursor,distance=target-displayCarouselX;if(distance){int step=((abs(distance)/6+1)*3+1)/2;if(step>15)step=15;displayCarouselX+=distance>0?step:-step;}
  bool intro=iconMenuIntroActive();int x=displayCarouselX-16;for(int i=0;i<4;i++,x+=48){if(intro)x=iconMenuIntroX(i,4);if(x>-32&&x<128)originalBitmap(x,curvedIconY(x),icons[i],32,32);}
  originalCenter(displayItems[cursor],56);
  if(cursor==2){char b[8];snprintf(b,sizeof(b),"%s",showFPS?"YES":"NO");originalText(b,55,33);}
}
static void beginIconMenuIntro(Screen value){
  iconMenuIntroScreen=value;iconMenuIntroStart=millis();
  if(value==MENU)carouselX=64-48*cursor;
  else if(value==APPS)appsCarouselX=64-48*cursor;
  else if(value==SETTINGS)settingsCarouselX=64-48*cursor;
  else if(value==DISPSET)displayCarouselX=64-48*cursor;
}
static void listKey(Key k,uint8_t count,Screen parent,const Screen*targets){if(k==LEFT)cursor=(cursor+count-1)%count;else if(k==RIGHT)cursor=(cursor+1)%count;else if(k==SELECT&&targets){screen=targets[cursor];cursor=0;}else if(k==LONG_SELECT){screen=parent;cursor=0;}}

static bool swRun=false;static uint32_t swStart=0,swAccum=0;
static void drawStopwatch(){
  uint32_t timer=swAccum+(swRun?millis()-swStart:0);
  if(timer>359999999UL)timer=359999999UL;
  uint8_t num1,num2,num3;uint32_t secs=timer/1000;
  if(timer<3600000UL){num3=(timer%1000)/10;num2=secs%60;num1=secs/60;}
  else{num3=secs%60;num2=(secs/60)%60;num1=secs/3600;}
  originalBitmap(1,20,(const uint8_t*)midFont+(num1/10)*57,19,24);
  originalBitmap(24,20,(const uint8_t*)midFont+(num1%10)*57,19,24);
  originalBitmap(49,20,colon,6,24);
  originalBitmap(60,20,(const uint8_t*)midFont+(num2/10)*57,19,24);
  originalBitmap(83,20,(const uint8_t*)midFont+(num2%10)*57,19,24);
  originalBitmap(104,28,(const uint8_t*)small2Font+(num3/10)*22,11,16);
  originalBitmap(116,28,(const uint8_t*)small2Font+(num3%10)*22,11,16);
  uint8_t h,m,s;getTime(h,m,s);char now[8];snprintf(now,sizeof(now),"%02u:%02u",h,m);originalCenter(now,0);
  originalBitmap(112,0,battIconHigh,16,8);
  originalText(swRun?"RUN":"STOP",0,56);
}
static int calcValue=0;static uint8_t calcStep=0;
static void drawCalc(){
  originalCenter("CALCULATOR",0);char b[18];snprintf(b,sizeof(b),"%d",calcValue);originalCenter(b,16);
  snprintf(b,sizeof(b),"STEP %u",1<<calcStep);originalCenter(b,28);
  rect(1,40,38,16);rect(45,40,38,16);rect(89,40,38,16);
  originalCenter("STEP",44);originalText("-",16,44);originalText("+",104,44);
  originalText("MINUS",0,56);originalText("PLUS",93,56);
}
static uint16_t torchStrobe=0;static uint32_t torchLast=0;static bool torchOn=true;
static void drawTorch(){
  if(!torchStrobe){clear(true);torchOn=true;return;}
  uint32_t now=millis();if(now-torchLast>=torchStrobe){torchLast=now;torchOn=!torchOn;}
  clear(torchOn);
}
static int ballX=64,ballY=36,ballDX=1,ballDY=-1,paddle=54,bricks=0xFF;
static void drawBreakout(){
  char b[8];snprintf(b,sizeof(b),"%u",8-__builtin_popcount((unsigned)bricks));originalText(b,0,0);
  static const uint8_t heart[]={0x06,0x0F,0x1F,0x3E,0x1F,0x0F,0x06};
  originalBitmap(104,0,heart,7,8);originalBitmap(112,0,heart,7,8);originalBitmap(120,0,heart,7,8);
  for(int i=0;i<8;i++)if(bricks&(1<<i))rect(2+i*16,10,14,6,true);
  rect(paddle,60,22,3,true);rect(ballX,ballY,2,2,true);
}
static void updateBreakout(){if(screen!=BREAKOUT)return;ballX+=ballDX;ballY+=ballDY;if(ballX<=0||ballX>=126)ballDX=-ballDX;if(ballY<=11)ballDY=1;if(ballY>=55&&ballX>=paddle-1&&ballX<=paddle+23)ballDY=-1;if(ballY>63){ballX=64;ballY=36;}if(ballY>=13&&ballY<21){int i=ballX/16;if(i<8&&(bricks&(1<<i))){bricks&=~(1<<i);ballDY=1;if(!bricks)bricks=0xFF;}}}
static int carX=60,enemyX=20,enemyY=12;static uint16_t score=0;
static void drawCar(){line(35,8,35,63);line(92,8,92,63);for(int y=12;y<64;y+=12)line(63,y,63,y+5);rect(carX,51,8,11,true);rect(enemyX,enemyY,8,10,true);char b[12];snprintf(b,sizeof(b),"%u",score);originalText(b,0,0);originalText("CAR",100,0);}
static void updateCar(){if(screen!=CARDODGE)return;enemyY+=2;if(enemyY>63){enemyY=12;enemyX=38+(esp_random()%47);score++;}if(enemyY>43&&enemyX+8>carX&&enemyX<carX+8){score=0;enemyY=12;}}
static void drawAlarm(){
  originalCenter("ALARMS",0);char b[24];snprintf(b,sizeof(b),"%c %02u:%02u MTWTFSS",alarmOn?'X':'.',alarmH,alarmM);originalText(b,7,16);
  originalText(alarmOn?"ALARM ENABLED":"ALARM DISABLED",7,32);
  originalText("HOUR",0,56);originalText("ON OFF",43,56);originalText("MIN",107,56);
}
static uint8_t timeField=0;
static void drawSetTime(){
  static const char*fields[]={"HOUR","MINUTE","DAY","MONTH","YEAR"};
  originalCenter("TIME & DATE",0);uint8_t h,m,s;getTime(h,m,s);char b[24];snprintf(b,sizeof(b),"%02u:%02u:%02u",h,m,s);originalCenter(b,16);snprintf(b,sizeof(b),"%02u/%02u/%04u",dateDay,dateMonth,dateYear);originalCenter(b,29);
  snprintf(b,sizeof(b),"> %s",fields[timeField]);originalCenter(b,43);originalText("DOWN",0,56);originalText("NEXT",50,56);originalText("UP",114,56);
}
static const char*sleepNames[]={"5S","10S","20S","5M","10M","ALWAYS ON"};
static uint32_t sleepSelectorStart=0,sleepTriangleSpinStart=0;
static int8_t sleepScrollDir=1;
static bool sleepValueAnimating=false;
static void drawSleepValue(){
  originalText("SLEEP",0,0);
  if(sleepSetting==5){text("ALWAYS",12,16,2);text("ON",24,39,3);}
  else{const char*value=sleepNames[sleepSetting];text(value,(73-strlen(value)*12)/2,23,3);}
  originalText("SAVE",20,56);
}
static void drawSleepSelector(){
  drawSleepValue();if(!sleepSelectorStart)return;
  const uint32_t slideMs=173,holdMs=600,endMs=900;uint32_t age=millis()-sleepSelectorStart;
  if(age>=endMs){sleepSelectorStart=0;sleepValueAnimating=false;return;}
  const int cx=135,cy=32,maxR=62,splitX=cx-maxR;
  float slideT=min(1.0f,float(age)/slideMs);slideT=slideT*slideT*(3.0f-2.0f*slideT);
  if(sleepValueAnimating){
    memcpy(newFb,fb,sizeof(fb));int shift=lroundf(slideT*64.0f);clear();
    for(int y=0;y<H;y++)for(int x=0;x<splitX;x++){
      bool v;if(shift>=64)v=bufferPx(newFb,x,y);
      else if(sleepScrollDir>0)v=bufferPx(oldFb,x,y+shift)||bufferPx(newFb,x,y-(64-shift));
      else v=bufferPx(oldFb,x,y-shift)||bufferPx(newFb,x,y+(64-shift));px(x,y,v);
    }
    if(age<holdMs)for(int y=0;y<H;y++)for(int x=splitX;x<W;x++)px(x,y,bufferPx(oldFb,x,y));
  }
  int r=maxR;if(age>holdMs)r=maxR-int(((age-holdMs)*maxR)/(endMs-holdMs));r=max(1,r);
  for(int y=max(0,cy-r+1);y<min(64,cy+r);y++)for(int x=max(splitX+1,cx-r+1);x<128;x++)if((x-cx)*(x-cx)+(y-cy)*(y-cy)<(r-1)*(r-1))px(x,y,false);
  circle(cx,cy,r);
  if(r>18){
    int scroll=sleepValueAnimating?lroundf(sleepScrollDir*13.0f*(1.0f-slideT)):0;
    for(int rel=-2;rel<=2;rel++){
      int y=30+rel*13+scroll,midY=y+2,dy=midY-cy,inside=(r-3)*(r-3)-dy*dy;if(inside<=0)continue;
      int x=cx-int(sqrtf(inside))+14;arcText(sleepNames[(sleepSetting+rel+6)%6],x,y,cx,cy,r-2);
    }
    uint32_t spinAge=millis()-sleepTriangleSpinStart;float angle=spinAge<127?spinAge*0.137f:0.0f;arcTriangle(82,32,angle,cx,cy,r-2);
  }
}
static void drawDiag(){
  originalCenter("DIAGNOSTICS",0);originalText("> MCU  ESP32-C3",0,12);originalText("  OLED SH1106",0,22);originalText(rtcPresent&&rtcTimeValid?"  RTC DS3231 OK":"  RTC 0X68 MISS",0,32);originalText(!bleEnabled?"  BLE DISABLED":!bleInitAttempted?"  BLE START DELAY":blePhoneReady()?"  BLE ANCS READY":blePhoneSecured()?"  BLE PAIRED":bleInitSucceeded?"  BLE ADVERTISING":"  BLE INIT FAILED",0,42);originalText("  FW  6.7 NATIVE",0,52);
}
static void drawAbout(){
  originalCenter("ABOUT",0);originalCenter("N|WATCH ESP32-C3",16);originalCenter("ZAK KEMBLE UI",28);originalCenter("ESP32 PORT",40);originalCenter("GPL V3",52);
}
static bool pairingResetConfirm=false,pairingResetYes=false;
static void drawConnection(){
  if(pairingResetConfirm){
    originalCenter("RESET PAIRING?",0);
    originalCenter("DELETE ALL BONDS",16);text("TIME + SETTINGS KEPT",24,29);
    originalText(pairingResetYes?"  NO      > YES":"> NO        YES",13,43);
    text("UP/DOWN CHOOSE",2,57);text("OK CONFIRM",85,57);return;
  }
  originalCenter("BLUETOOTH",0);drawBluetoothMenuIcon(2,16);
  if(!bleEnabled){text("RADIO DISABLED",40,17);text("SELECT TO ENABLE",40,29);}
  else if(!bleInitAttempted){text("STARTING RADIO",40,17);text("PLEASE WAIT...",40,29);}
  else if(!bleInitSucceeded){text("BLE INIT FAILED",40,17);text("WATCH STILL ACTIVE",40,29);}
  else if(blePhonePaused()){text("LINK PAUSED",40,17);text("TOGGLE OR RESET",40,29);}
  else if(blePhoneReady()){text("ANCS SUBSCRIBED",40,17);text("IPHONE CONNECTED",40,29);}
  else if(blePhoneSecured()){text("PAIRED",40,17);text("ANCS DISCOVERY",40,29);}
  else if(blePhoneConnected()){text("CONNECTED",40,17);text("WAITING FOR PAIR",40,29);}
  else{text("NWATCH C3",40,17);text("ADVERTISING...",40,29);}
  if(bleEnabled&&bleInitAttempted){char state[19];snprintf(state,sizeof(state),"%.18s",blePhoneStateText());originalText(state,1,39);}
  originalText(bleEnabled?"BLUETOOTH: ON":"BLUETOOTH: OFF",1,48);text("OK ON/OFF",1,58);text("DOWN RESET",87,58);
}
enum NoticeVisual:uint8_t{NOTICE_APP,NOTICE_MESSAGE,NOTICE_PHONE,NOTICE_MAIL,NOTICE_CALENDAR,NOTICE_VIDEO};
static NoticeVisual noticeVisual(const char*app,uint8_t category){
  if(category==1||category==2||strstr(app,"PHONE"))return NOTICE_PHONE;
  if(strstr(app,"MESSAGES")||strstr(app,"ZALO")||strstr(app,"MESSENGER")||strstr(app,"WHATSAPP")||category==4)return NOTICE_MESSAGE;
  if(strstr(app,"MAIL")||category==6)return NOTICE_MAIL;
  if(strstr(app,"CALENDAR")||category==5)return NOTICE_CALENDAR;
  if(strstr(app,"YOUTUBE")||strstr(app,"VIDEO"))return NOTICE_VIDEO;
  return NOTICE_APP;
}
static void roundedBox(int x,int y,int w,int h){line(x+2,y,x+w-3,y);line(x+2,y+h-1,x+w-3,y+h-1);line(x,y+2,x,y+h-3);line(x+w-1,y+2,x+w-1,y+h-3);px(x+1,y+1);px(x+w-2,y+1);px(x+1,y+h-2);px(x+w-2,y+h-2);}
static void drawNoticeIcon(NoticeVisual visual,int x,int y){
  roundedBox(x,y,18,18);
  if(visual==NOTICE_PHONE){
    rect(x+4,y+3,4,5,true);rect(x+10,y+10,4,5,true);line(x+6,y+7,x+11,y+12);px(x+7,y+8);px(x+10,y+11);
  }else if(visual==NOTICE_MESSAGE){
    roundedBox(x+3,y+4,12,9);line(x+5,y+13,x+4,y+16);line(x+4,y+16,x+9,y+13);px(x+6,y+8);px(x+9,y+8);px(x+12,y+8);
  }else if(visual==NOTICE_MAIL){
    rect(x+3,y+5,12,9);line(x+3,y+5,x+9,y+10);line(x+14,y+5,x+9,y+10);
  }else if(visual==NOTICE_CALENDAR){
    rect(x+3,y+4,12,11);line(x+3,y+7,x+14,y+7);line(x+6,y+2,x+6,y+6);line(x+11,y+2,x+11,y+6);px(x+6,y+10);px(x+9,y+10);px(x+12,y+10);
  }else if(visual==NOTICE_VIDEO){
    roundedBox(x+2,y+5,14,9);line(x+7,y+7,x+7,y+12);line(x+7,y+7,x+12,y+9);line(x+12,y+9,x+7,y+12);
  }else{
    for(int row=0;row<2;row++)for(int col=0;col<2;col++)rect(x+4+col*6,y+4+row*6,3,3,true);
  }
}
static void initialsFromName(const char*name,char out[3]){
  out[0]=name&&name[0]?name[0]:'?';out[1]=0;out[2]=0;
  if(!name)return;const char*last=nullptr;for(const char*p=name;*p;p++)if(*p==' '&&p[1])last=p+1;if(last){out[1]=*last;out[2]=0;}
}
static void drawContactAvatar(const char*name,int cx,int cy,int radius){
  circle(cx,cy,radius);char initials[3];initialsFromName(name,initials);int width=strlen(initials)*8;text(initials,cx-width/2,cy-5,2);
  circle(cx+radius-2,cy+radius-2,4);rect(cx+radius-4,cy+radius-2,5,3,true);
}
static void drawLargePhoneIcon(int cx,int cy){
  circle(cx,cy,17);rect(cx-9,cy-11,6,8,true);rect(cx+3,cy+4,6,8,true);line(cx-5,cy-4,cx+5,cy+7);line(cx-4,cy-3,cx+6,cy+8);rect(cx-10,cy-12,8,3,true);rect(cx+2,cy+10,8,3,true);
}
static void drawWrappedNotice(const char*value,int firstY,uint8_t rows){
  size_t length=strlen(value),at=0;for(uint8_t row=0;row<rows&&at<length;row++){
    while(at<length&&value[at]==' ')at++;size_t end=min(length,at+31),cut=end;
    if(end<length){while(cut>at&&value[cut]!=' ')cut--;if(cut==at)cut=end;}
    char lineText[32];size_t count=min(size_t(31),cut-at);memcpy(lineText,value+at,count);lineText[count]=0;text(lineText,2,firstY+row*8);at=cut;
  }
}
static void drawIncomingCall(const char*app,const char*caller,const char*detail){
  text(app,2,2);text("NOW",114,2);line(0,9,127,9);
  drawLargePhoneIcon(24,35);
  const char*state=currentNotification.category==2?"MISSED CALL":"INCOMING CALL";text(state,47,16);
  char shortName[11];snprintf(shortName,sizeof(shortName),"%.10s",caller[0]?caller:"UNKNOWN");text(shortName,47,27,2);
  char shortDetail[20];snprintf(shortDetail,sizeof(shortDetail),"%.19s",detail);text(shortDetail,47,42);
  uint8_t pulse=(millis()/160)%3;for(uint8_t i=0;i<pulse;i++){line(4-i*2,27-i*2,4-i*2,43+i*2);line(44+i*2,27-i*2,44+i*2,43+i*2);}
  line(53,57,121,57);text("IPHONE",72,52);
}
static void drawBluetoothNotification(){
  char app[21],heading[31],message[181];displayAscii(notificationAppName(currentNotification.app),app,sizeof(app));
  displayAscii(currentNotification.title,heading,sizeof(heading));displayAscii(currentNotification.message,message,sizeof(message));if(!app[0])snprintf(app,sizeof(app),"IPHONE");
  NoticeVisual visual=noticeVisual(app,currentNotification.category);
  if(visual==NOTICE_PHONE){drawIncomingCall(app,heading,message);return;}
  if(visual==NOTICE_MESSAGE)drawContactAvatar(heading,10,10,9);else drawNoticeIcon(visual,1,1);
  char appLine[21];snprintf(appLine,sizeof(appLine),"%.20s",app);text(appLine,23,2);text("NOW",114,2);
  char titleLine[15];snprintf(titleLine,sizeof(titleLine),"%.14s",heading[0]?heading:app);originalText(titleLine,23,10);line(0,21,127,21);
  drawWrappedNotice(message[0]?message:"NEW NOTIFICATION",25,4);line(53,61,75,61);
}

static uint32_t sleepTimeoutMs(){
  static const uint32_t values[]={5000UL,10000UL,20000UL,300000UL,600000UL,0};return values[sleepSetting];
}
static void enterDeviceSleep(){
  // Keep NimBLE alive even while temporarily disconnected so bonded iPhones can reconnect.
  if(bleEnabled&&bleInitSucceeded){
    if(torchBrightnessForced)restoreTorchBrightness();saveConfig();cmd(0xAE);displaySleeping=true;return;
  }
  // Never manually light-sleep while a native BLE host/controller is active.
  if(!blePhoneShutdown()){cmd(0xAE);displaySleeping=true;return;}
  if(torchBrightnessForced)restoreTorchBrightness();saveConfig();cmd(0xAE);delay(20);
  gpio_wakeup_enable((gpio_num_t)BTN_LEFT,GPIO_INTR_LOW_LEVEL);gpio_wakeup_enable((gpio_num_t)BTN_SELECT,GPIO_INTR_LOW_LEVEL);gpio_wakeup_enable((gpio_num_t)BTN_RIGHT,GPIO_INTR_LOW_LEVEL);esp_sleep_enable_gpio_wakeup();esp_light_sleep_start();
  while(!digitalRead(BTN_LEFT)||!digitalRead(BTN_SELECT)||!digitalRead(BTN_RIGHT))delay(5);
  uint32_t now=millis();for(uint8_t i=0;i<3;i++){buttons[i].last=true;buttons[i].down=0;buttons[i].debounce=now;buttons[i].longFired=false;}
  cmd(0xAF);sentFbValid=false;displaySleeping=false;screen=WATCH;cursor=0;transitionStart=0;faceSelectorStart=0;sleepSelectorStart=0;lastInteraction=now;rtcSyncFromHardware();
}
static void goBack(){
  if(screen==TORCH)restoreTorchBrightness();
  if(screen==MENU)screen=WATCH;
  else if(screen==APPS||screen==GAMES||screen==ALARMS||screen==SETTINGS||screen==CONNECTION)screen=MENU;
  else if(screen==DIAG||screen==SETTIME||screen==SLEEPSET||screen==DISPSET||screen==ABOUT)screen=SETTINGS;
  else if(screen==STOPWATCH||screen==TORCH)screen=MENU;
  else if(screen==CALC)screen=APPS;
  else if(screen==BREAKOUT||screen==CARDODGE)screen=GAMES;
  else screen=WATCH;
  cursor=0;
}
static void handle(Key k){
  if(k==NONE)return;
  if(k==LONG_SELECT){
    if(screen==CONNECTION&&pairingResetConfirm){pairingResetConfirm=false;pairingResetYes=false;return;}
    if(screen!=WATCH)goBack();return;
  }
  static const Screen mainTargets[]={SETTINGS,STOPWATCH,GAMES,ALARMS,CONNECTION,APPS,WATCH};
  static const Screen appTargets[]={STOPWATCH,CALC,TORCH};
  static const Screen gameTargets[]={BREAKOUT,CARDODGE};
  if(screen==WATCH){
    if(k==LEFT||k==RIGHT){
      uint32_t now=millis();if(!faceSelectorStart||now-faceSelectorStart>=900)triangleSpinStart=now;
      faceScrollDir=k==RIGHT?1:-1;watchface=(watchface+(k==LEFT?FACE_COUNT-1:1))%FACE_COUNT;faceSelectorStart=now;faceSavePending=true;faceSaveAt=now;
    }
    else{screen=MENU;cursor=0;if(faceSavePending){saveConfig();faceSavePending=false;}}
  }
  else if(screen==MENU)listKey(k,7,WATCH,mainTargets);
  else if(screen==APPS){if(k==SELECT&&cursor==2){torchStrobe=0;torchOn=true;}listKey(k,3,MENU,appTargets);if(screen==TORCH)forceTorchBrightness();}
  else if(screen==GAMES)listKey(k,2,MENU,gameTargets);
  else if(screen==SETTINGS){
    if(k==LEFT)cursor=(cursor+4)%5;else if(k==RIGHT)cursor=(cursor+1)%5;else if(k==SELECT){
      if(cursor==0){screen=SETTIME;timeField=0;cursor=0;}else if(cursor==1){screen=SLEEPSET;cursor=0;sleepSelectorStart=millis();sleepTriangleSpinStart=millis();sleepValueAnimating=false;}else if(cursor==2){screen=DIAG;cursor=0;}else if(cursor==3){screen=DISPSET;cursor=0;}else{screen=MENU;cursor=0;}
    }
  }
  else if(screen==SLEEPSET){
    if(k==LEFT||k==RIGHT){uint32_t now=millis();sleepScrollDir=k==RIGHT?1:-1;sleepSetting=(sleepSetting+(k==LEFT?5:1))%6;sleepSelectorStart=now;sleepTriangleSpinStart=now;sleepValueAnimating=true;saveConfig();}
    else if(k==SELECT){saveConfig();screen=SETTINGS;cursor=1;}
  }
  else if(screen==ALARMS){if(k==LEFT)alarmH=(alarmH+1)%24;else if(k==RIGHT)alarmM=(alarmM+1)%60;else alarmOn=!alarmOn;saveConfig();}
  else if(screen==DIAG){if(k==SELECT)screen=ABOUT;}
  else if(screen==STOPWATCH){if(k==LEFT){swRun=false;swAccum=0;}else if(k==SELECT){if(swRun){swAccum+=millis()-swStart;swRun=false;}else{swStart=millis();swRun=true;}}else goBack();}
  else if(screen==CALC){if(k==LEFT)calcValue-=1<<calcStep;else if(k==RIGHT)calcValue+=1<<calcStep;else calcStep=(calcStep+1)%7;}
  else if(screen==BREAKOUT){if(k==LEFT)paddle=max(0,paddle-8);else if(k==RIGHT)paddle=min(106,paddle+8);else goBack();}
  else if(screen==CARDODGE){if(k==LEFT)carX=max(38,carX-12);else if(k==RIGHT)carX=min(83,carX+12);else goBack();}
  else if(screen==TORCH){if(k==SELECT){if(torchStrobe<500)torchStrobe+=50*((torchStrobe/50)+1);else torchStrobe=0;torchLast=millis();}else goBack();}
  else if(screen==SETTIME){
    uint8_t h,m,s;getTime(h,m,s);int delta=k==RIGHT?1:k==LEFT?-1:0;
    if(delta){if(timeField==0){setH=(h+24+delta)%24;setM=m;clockBase=millis()-s*1000;}else if(timeField==1){setH=h;setM=(m+60+delta)%60;clockBase=millis()-s*1000;}else if(timeField==2)dateDay=1+(dateDay-1+31+delta)%31;else if(timeField==3)dateMonth=1+(dateMonth-1+12+delta)%12;else dateYear=constrain(int(dateYear)+delta,2024,2099);}
    else{timeField++;if(timeField>=5){timeField=0;rtcWriteCurrent();saveConfig();}}
  }
  else if(screen==DISPSET){
    if(k==LEFT)cursor=(cursor+3)%4;else if(k==RIGHT)cursor=(cursor+1)%4;else{
      if(cursor==0){brightness=(brightness+1)%4;applyConfiguredBrightness();}else if(cursor==1){invertDisplay=!invertDisplay;cmd(invertDisplay?0xA7:0xA6);}else if(cursor==2)showFPS=!showFPS;else{screen=SETTINGS;cursor=0;}saveConfig();
    }
  }
  else if(screen==CONNECTION){
    if(pairingResetConfirm){
      if(k==LEFT||k==RIGHT)pairingResetYes=!pairingResetYes;
      else if(k==SELECT){
        bool confirmed=pairingResetYes;pairingResetConfirm=false;pairingResetYes=false;
        if(confirmed){
          notificationVisible=false;
          bool ok=blePhoneResetPairing();
          bleEnabled=true;bleInitAttempted=true;bleInitSucceeded=ok;bleInitAt=0;
          // Only preserve Bluetooth ON; bond deletion never clears user config.
          saveConfig();lastInteraction=millis();sentFbValid=false;
        }
      }
      return;
    }
    if(k==RIGHT){pairingResetConfirm=true;pairingResetYes=false;notificationVisible=false;return;}
    if(k==SELECT){
      bleEnabled=!bleEnabled;saveConfig();notificationVisible=false;
      if(!bleEnabled){blePhoneShutdown();bleInitAttempted=false;bleInitSucceeded=false;bleInitAt=0;}
      else{bleInitAttempted=false;bleInitSucceeded=false;bleInitAt=millis()+500;}
      sentFbValid=false;lastInteraction=millis();
    }
  }
  else if(screen==ABOUT){goBack();}
}

static void render(){
  clear();
  if(notificationVisible)drawBluetoothNotification();
  else if(screen==WATCH)drawWatch();else if(screen==MENU)drawOriginalMain();else if(screen==APPS)drawOriginalApps();else if(screen==GAMES)drawOriginalStringList("GAMES",gameItems,2);else if(screen==SETTINGS)drawOriginalSettings();else if(screen==ALARMS)drawAlarm();else if(screen==DIAG)drawDiag();else if(screen==STOPWATCH)drawStopwatch();else if(screen==CALC)drawCalc();else if(screen==TORCH)drawTorch();else if(screen==BREAKOUT)drawBreakout();else if(screen==CARDODGE)drawCar();else if(screen==SETTIME)drawSetTime();else if(screen==SLEEPSET)drawSleepSelector();else if(screen==DISPSET)drawVideoDisplay();else if(screen==ABOUT)drawAbout();else if(screen==CONNECTION)drawConnection();
  composeTransition();flush();
}

static char bootTerminalLines[8][32];
static uint8_t bootTerminalCount=0;
static void drawBootTerminal(bool cursorOn=false){
  clear();for(uint8_t i=0;i<bootTerminalCount;i++)text(bootTerminalLines[i],0,i*8);
  if(cursorOn&&bootTerminalCount){int x=min(124,int(strlen(bootTerminalLines[bootTerminalCount-1])*4));rect(x,(bootTerminalCount-1)*8+6,3,1,true);}
  flush();
}
static void newBootTerminalLine(){
  if(bootTerminalCount<8){bootTerminalLines[bootTerminalCount][0]=0;bootTerminalCount++;}
  else{for(uint8_t i=0;i<7;i++)strcpy(bootTerminalLines[i],bootTerminalLines[i+1]);bootTerminalLines[7][0]=0;}
}
static void typeBootTerminalLine(const char*value,uint8_t characterMs,uint16_t waitMs){
  newBootTerminalLine();char*lineBuffer=bootTerminalLines[bootTerminalCount-1];uint8_t length=min(size_t(31),strlen(value));
  for(uint8_t i=0;i<length;i++){lineBuffer[i]=value[i];lineBuffer[i+1]=0;drawBootTerminal(true);delay(characterMs);}
  drawBootTerminal(false);delay(waitMs);
}
static void resetBootTerminal(){bootTerminalCount=0;memset(bootTerminalLines,0,sizeof(bootTerminalLines));}
static void wipeBootScreen(){
  memcpy(oldFb,fb,sizeof(fb));
  for(uint8_t step=0;step<=8;step++){
    clear();int inset=step*4;for(int y=inset;y<64-inset;y++)for(int x=0;x<128;x++)if(bufferPx(oldFb,x,y))px(x,y);flush();delay(18);
  }
}
static void showBootCredits(){
  for(uint8_t frame=0;frame<=20;frame++){
    float t=frame/20.0f;t=t*t*(3.0f-2.0f*t);clear();originalCenter("CREDITS",0);
    int rule=lroundf(t*54);line(64-rule,8,64+rule,8);
    originalCenter("ORIGINAL PROJECT",12);text("ZAKKEMBLE",-72+lroundf(t*100),21,2);
    originalCenter("ESP32-C3 REBUILD",38);text("HUY VECTOR",128-lroundf(t*104),47,2);
    flush();delay(20);
  }
  delay(720);
}
static void showBootLoading(){
  for(uint8_t frame=0;frame<=32;frame++){
    clear();text("N|WATCH",22,3,3);center("ESP32-C3 BLE RTC",22);circle(64,32,5);
    float angle=frame*0.52f;rect(63+lroundf(cosf(angle)*4),31+lroundf(sinf(angle)*4),3,3,true);
    rect(11,40,106,10);int fillWidth=(100*frame)/32;if(fillWidth)rect(14,43,fillWidth,4,true);
    originalCenter(frame<32?"LOADING SYSTEM":"SYSTEM READY",54);flush();delay(25);
  }
  delay(320);
}

void setup(){
  setCpuFrequencyMhz(80);
  uint32_t resetReason=uint32_t(esp_reset_reason());
  Serial.begin(115200);delay(40);Serial.printf("[BOOT] Stage 6.7.2 Native GATT + Back400 reason=%lu cpu=%uMHz heap=%u\n",resetReason,getCpuFrequencyMhz(),ESP.getFreeHeap());
  pinMode(BTN_LEFT,INPUT_PULLUP);pinMode(BTN_SELECT,INPUT_PULLUP);pinMode(BTN_RIGHT,INPUT_PULLUP);
  oledInit();loadConfig();rtcInitialize();clear();flush();delay(67);
  for(uint8_t y=0;y<64;y+=4){clear();line(0,y,127,y);line(0,max(0,int(y)-3),127,max(0,int(y)-3));flush();delay(8);}
  // Expanding wireframe keeps the original energy without lighting every OLED
  // pixel at once. The old solid 128x64 frame was a measurable current step.
  const uint8_t widths[]={2,8,24,64,124};for(uint8_t w:widths){clear();rect((128-w)/2,1,w,62);if(w>16)rect((128-w)/2+4,5,w-8,54);flush();delay(30);}
  clear();flush();delay(27);for(uint8_t x=0;x<128;x+=8){line(x,0,min(127,int(x)+31),63);}flush();delay(33);clear();flush();
  resetBootTerminal();typeBootTerminalLine("SYSTEM CHECK 05.26",8,80);
  typeBootTerminalLine("> CPU ESP32-C3...OK",8,60);
  typeBootTerminalLine("> RAM 320KB......OK",8,60);
  typeBootTerminalLine("> OLED 0X3C......OK",8,70);
  const char*rtcBootStatus=!rtcPresent?"> RTC 0X68....MISS":rtcInitializedFromFallback?"> RTC 0X68....INIT":"> RTC 0X68......OK";
  typeBootTerminalLine(rtcBootStatus,8,80);typeBootTerminalLine("> BUS RECOVERY...ON",8,80);
  typeBootTerminalLine(bleEnabled?"> BLE AFTER BOOT..ON":"> BLE AFTER BOOT.OFF",8,260);wipeBootScreen();
  showBootCredits();wipeBootScreen();showBootLoading();
  memcpy(oldFb,fb,sizeof(fb));transitionDuration=450;transitionStart=millis();lastInteraction=millis();bleInitAt=bleEnabled?millis()+3500:0;bootPowerReleaseAt=millis()+10000;
}
void loop(){
  if(bleEnabled&&!bleInitAttempted&&bleInitAt&&int32_t(millis()-bleInitAt)>=0){
    bleInitAttempted=true;Serial.printf("[BLE] deferred init start heap=%u\n",ESP.getFreeHeap());delay(50);bleInitSucceeded=blePhoneInit(bleResetBonds);if(bleInitSucceeded&&bleResetBonds){prefs.putUChar("bondver",64);bleResetBonds=false;}Serial.printf("[BLE] deferred init result=%s heap=%u\n",bleInitSucceeded?"OK":"FAIL",ESP.getFreeHeap());lastInteraction=millis();sentFbValid=false;
  }
  processBlePhone();
  updateBleQuietGuard();
  if(bootPowerGuard&&int32_t(millis()-bootPowerReleaseAt)>=0)releaseBootPowerGuard();
  if(displaySleeping){
    if(!digitalRead(BTN_LEFT)||!digitalRead(BTN_SELECT)||!digitalRead(BTN_RIGHT)){
      wakeDisplay();uint32_t now=millis();for(uint8_t i=0;i<3;i++){buttons[i].last=digitalRead(buttons[i].pin);buttons[i].down=0;buttons[i].debounce=now;buttons[i].longFired=true;}
    }
    if(displaySleeping){delay(3);return;}
  }
  Key key=readKey();
  if(notificationVisible&&key!=NONE){notificationVisible=false;lastInteraction=millis();key=NONE;sentFbValid=false;}
  if(key!=NONE)lastInteraction=millis();if(key!=NONE&&!iconMenuIntroActive()){
    Screen before=screen;memcpy(oldFb,fb,sizeof(fb));handle(key);
    if(screen!=before){transitionDuration=213;transitionStart=millis();if(isIconMenu(screen))beginIconMenuIntro(screen);}
  }
  static uint32_t physics,lastDraw,lastAlarmMinute=UINT32_MAX;
  if(!bleQuiet.active&&millis()-physics>=25){physics=millis();updateBreakout();updateCar();}
  serviceRtc();
  uint32_t minute=clockSeconds()/60;if(alarmOn&&minute!=lastAlarmMinute&&minute%1440==uint32_t(alarmH)*60+alarmM){lastAlarmMinute=minute;invertDisplay=!invertDisplay;cmd(invertDisplay?0xA7:0xA6);}
  if(faceSavePending&&millis()-faceSaveAt>1800){saveConfig();faceSavePending=false;}
  if(millis()-lastSave>3600000UL)saveConfig();
  // Battery build: 42 ms caps rendering at ~24 FPS. Animation durations,
  // button polling and BLE polling remain independent of the render cadence.
  if(!bleQuiet.active&&millis()-lastDraw>=42){lastDraw=millis();render();}
  uint32_t timeout=sleepTimeoutMs();if(!notificationVisible&&timeout&&millis()-lastInteraction>=timeout)enterDeviceSleep();
  delay(3);
}
