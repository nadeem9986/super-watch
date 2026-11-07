#include "../src/ble_quiet_policy.h"
#include <assert.h>
#include <stdio.h>
int main(){
  BleQuietPolicy p;
  p.update(true,false,false,false,false,0);assert(!p.active);
  p.update(true,true,false,false,false,1);assert(p.active);
  p.update(true,true,false,false,false,20000);assert(p.active);
  p.update(true,true,false,false,false,20001);assert(!p.active);
  p.update(true,true,false,false,false,30000);assert(!p.active); // no repeat per link
  p.update(true,false,false,false,false,30001);
  p.update(true,true,false,false,false,30002);assert(p.active);
  p.update(true,true,true,false,false,30003);assert(!p.active); // subscribed
  p.update(true,false,false,false,false,30004);
  p.update(true,true,false,false,false,30005);assert(p.active);
  p.update(true,true,false,false,true,30006);assert(!p.active); // button cancels
  p.update(true,false,false,false,false,30007);
  p.update(true,true,false,true,false,30008);assert(!p.active); // already sleeping
  p.update(false,false,false,false,false,30009);assert(!p.active && !p.used);
  p.update(true,true,false,false,false,UINT32_MAX-10);assert(p.active);
  p.update(true,true,false,false,false,20000);assert(!p.active); // timer wrap
  puts("Quiet policy: connect, ready, timeout, no rearm, buttons, sleep, OFF, wrap PASS");
}
