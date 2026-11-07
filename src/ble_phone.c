/* NWatch native NimBLE transport. All GAP/GATT/session state belongs to the
 * NimBLE host task. Arduino exchanges only commands and copied UI messages.
 * Reference architecture: Espressif bleprph/ble_ancs, RoadPing-C3.
 * No NimBLE-Arduino wrapper, virtual GATT OPEN or second physical connection. */
#include "ble_phone.h"
#include "ancs_parser.h"
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_nimble_hci.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* IDF 4.4 derives these GATT features from ROLE_CENTRAL, even though our
 * physical link remains peripheral. A symbol existing in ELF isn't enough:
 * its disabled implementation simply returns BLE_HS_ENOTSUP. */
#if !MYNEWT_VAL(BLE_GATT_DISC_ALL_SVCS) || !MYNEWT_VAL(BLE_GATT_DISC_ALL_CHRS) || !MYNEWT_VAL(BLE_GATT_DISC_ALL_DSCS) || !MYNEWT_VAL(BLE_GATT_WRITE) || !MYNEWT_VAL(BLE_GATT_READ)
#error "NWatch ANCS requires native GATT discovery/read/write compiled ON"
#endif

void ble_store_config_init(void);
static const ble_uuid128_t ANCS=BLE_UUID128_INIT(0xd0,0x00,0x2d,0x12,0x1e,0x4b,0x0f,0xa4,0x99,0x4e,0xce,0xb5,0x31,0xf4,0x05,0x79);
static const ble_uuid128_t NS=BLE_UUID128_INIT(0xbd,0x1d,0xa2,0x99,0xe6,0x25,0x58,0x8c,0xd9,0x42,0x01,0x63,0x0d,0x12,0xbf,0x9f);
static const ble_uuid128_t CP=BLE_UUID128_INIT(0xd9,0xd9,0xaa,0xfd,0xbd,0x9b,0x21,0x98,0xa8,0x49,0xe1,0x45,0xf3,0xd8,0xd1,0x69);
static const ble_uuid128_t DS=BLE_UUID128_INIT(0xfb,0x7b,0x7c,0xce,0x6a,0xb3,0x44,0xbe,0xb5,0x4b,0xd6,0x24,0xe9,0xc6,0xea,0x22);
static const ble_uuid16_t CTS=BLE_UUID16_INIT(0x1805), CT=BLE_UUID16_INIT(0x2a2b), CCCD=BLE_UUID16_INIT(0x2902);
static portMUX_TYPE view_mux=portMUX_INITIALIZER_UNLOCKED;
static struct { bool connected,secured,ready,paused; char text[32]; } view;
static QueueHandle_t commands, notices, times;
static SemaphoreHandle_t off_ack, host_done;
enum {CMD_ON=1,CMD_OFF,CMD_RESET};
static bool initialized, synced, enabled, clearing, busy, params_sent, slow_adv;
static uint8_t own_type, retries;
static uint16_t conn=BLE_HS_CONN_HANDLE_NONE;
static uint32_t epoch=1, due, deadline, secured_at, adv_at, security_at, time_at, alive_at;
static struct ble_npl_callout ticker;
typedef enum {IDLE,SERVICES,CHARS,DATA_DSC,DATA_SUB,SOURCE_DSC,SOURCE_SUB,TIME_CHARS,TIME_READ,RUNNING,ATTR_WRITE} Phase;
static Phase phase;
static uint16_t svc_start,svc_end,cts_start,cts_end,source,source_end,data_h,data_end,control,source_cccd,data_cccd,time_h;
typedef struct {uint32_t uid; uint8_t flags,category;} Notice;
static Notice requests[8];
static uint8_t request_count;
static AncsParser parser;
static uint32_t ms(void) {return (uint32_t)(esp_timer_get_time()/1000);}
static bool elapsed(uint32_t now,uint32_t at) {return (int32_t)(now-at)>=0;}
static void status(const char *s) {
    portENTER_CRITICAL(&view_mux);snprintf(view.text,sizeof(view.text),"%s",s);portEXIT_CRITICAL(&view_mux);
    printf("[NATIVE] %s\n",s);
}
static void flags(bool linked,bool secure,bool ready,bool paused) {
    portENTER_CRITICAL(&view_mux);view.connected=linked;view.secured=secure;view.ready=ready;view.paused=paused;portEXIT_CRITICAL(&view_mux);
}
static bool current(uint16_t h,void *token) {return enabled && conn==h && (uint32_t)(uintptr_t)token==epoch;}
static void *token(void) {return (void *)(uintptr_t)epoch;}
static void clear_client(void) {
    ++epoch;busy=false;phase=IDLE;request_count=0;memset(&parser,0,sizeof(parser));
    svc_start=svc_end=cts_start=cts_end=source=source_end=data_h=data_end=control=source_cccd=data_cccd=time_h=0;
}
static void retry(const char *op,int rc) {
    busy=false;parser.active=false;request_count=0;
    printf("[NATIVE] GATT %s rc=%d retry=%u\n",op,rc,retries);
    /* Timed-out callbacks carry an old epoch and cannot mutate a new attempt. */
    clear_client();
    if(++retries<=4) {phase=SERVICES;due=ms()+1500*retries;status("ANCS RETRY");}
    else {phase=IDLE;due=ms()+60000;status("ANCS WAIT IPHONE");}
    flags(conn!=BLE_HS_CONN_HANDLE_NONE,true,false,false);
}
static int gap(struct ble_gap_event *e,void *arg);
static void advertise(bool slow) {
    if(!synced || !enabled || conn!=BLE_HS_CONN_HANDLE_NONE) return;
    ble_gap_adv_stop();
    /* ANCS solicitation, not a hosted ANCS service. Name remains visible in
     * the primary advertisement so passive scanners can identify the watch. */
    uint8_t adv[]={2,0x01,0x06,17,0x15,
        0xd0,0x00,0x2d,0x12,0x1e,0x4b,0x0f,0xa4,0x99,0x4e,0xce,0xb5,0x31,0xf4,0x05,0x79};
#if CONFIG_IDF_TARGET_ESP32
    const uint8_t short_name[]={9,0x08,'N','W','a','t','c','h','3','2'};
    const uint8_t full_name[]={10,0x09,'N','W','a','t','c','h',' ','3','2'};
#else
    const uint8_t short_name[]={9,0x08,'N','W','a','t','c','h','C','3'};
    const uint8_t full_name[]={10,0x09,'N','W','a','t','c','h',' ','C','3'};
#endif
    uint8_t primary[31];memcpy(primary,adv,sizeof(adv));memcpy(primary+sizeof(adv),short_name,sizeof(short_name));
    int rc=ble_gap_adv_set_data(primary,sizeof(primary));
    if(!rc) rc=ble_gap_adv_rsp_set_data(full_name,sizeof(full_name));
    if(rc) {printf("[NATIVE] ADV payload rc=%d\n",rc);return;}
    struct ble_gap_adv_params p={0};p.conn_mode=BLE_GAP_CONN_MODE_UND;p.disc_mode=BLE_GAP_DISC_MODE_GEN;
    p.itvl_min=slow?800:160;p.itvl_max=slow?1200:240; /* 500–750 / 100–150ms */
    rc=ble_gap_adv_start(own_type,NULL,BLE_HS_FOREVER,&p,gap,NULL);
    slow_adv=slow;adv_at=ms();
    if(!rc) status(slow?"ADVERTISING ECO":"ADVERTISING");else printf("[NATIVE] ADV start rc=%d\n",rc);
}
static void secure_ready(void) {
    struct ble_gap_conn_desc d;
    if(!enabled || conn==BLE_HS_CONN_HANDLE_NONE || ble_gap_conn_find(conn,&d) || !d.sec_state.encrypted) return;
    if(secured_at) return;
    secured_at=ms();security_at=0;flags(true,true,false,false);
    printf("[NATIVE] SECURE h=%u bonded=%u idType=%u otaType=%u\n",conn,d.sec_state.bonded,d.peer_id_addr.type,d.peer_ota_addr.type);
    phase=SERVICES;due=ms()+1500;status("PAIRED - DISCOVER");
}
static void queue_notice(uint32_t uid,uint8_t flags_,uint8_t category) {
    for(unsigned i=0;i<request_count;i++) if(requests[i].uid==uid) {requests[i]=(Notice){uid,flags_,category};return;}
    if(request_count==8) {memmove(requests,requests+1,7*sizeof(*requests));request_count=7;}
    requests[request_count++]=(Notice){uid,flags_,category};
}
static void notify(uint16_t handle,struct os_mbuf *om) {
    uint16_t len=OS_MBUF_PKTLEN(om);
    if(handle==source && len>=8) {
        uint8_t b[8];if(os_mbuf_copydata(om,0,8,b))return;
        uint32_t uid=ancs_u32(b+4);
        printf("[NATIVE] ANCS event=%u cat=%u flags=%u uid=%lu\n",b[0],b[2],b[1],(unsigned long)uid);
        if(b[0]==2) {
            for(unsigned i=0;i<request_count;i++) if(requests[i].uid==uid) {memmove(requests+i,requests+i+1,(request_count-i-1)*sizeof(*requests));--request_count;break;}
            if(parser.active && parser.uid==uid) parser.active=false;
        } else if(b[0]<=1) queue_notice(uid,b[1],b[2]);
    } else if(handle==data_h && parser.active) {
        uint8_t b[64];
        for(uint16_t off=0;off<len;) {
            uint16_t n=len-off;if(n>sizeof(b))n=sizeof(b);
            if(os_mbuf_copydata(om,off,n,b))break;
            bool complete=ancs_parser_feed(&parser,b,n);off+=n;
            if(complete) {
                if(!parser.value.title[0])snprintf(parser.value.title,sizeof(parser.value.title),"%s",parser.value.app);
                if(xQueueSend(notices,&parser.value,0)!=pdTRUE) {BlePhoneNotification old;xQueueReceive(notices,&old,0);xQueueSend(notices,&parser.value,0);}
                parser.valid=false;printf("[NATIVE] NOTICE DELIVERED\n");break;
            }
        }
    }
}
static int gap(struct ble_gap_event *e,void *arg) {
    (void)arg;
    switch(e->type) {
    case BLE_GAP_EVENT_CONNECT:
        if(e->connect.status) {printf("[NATIVE] CONNECT fail=%d\n",e->connect.status);advertise(true);break;}
        clear_client();conn=e->connect.conn_handle;secured_at=0;params_sent=false;retries=0;
        flags(true,false,false,false);status("CONNECTED - PAIR");
        if(!enabled) {ble_gap_terminate(conn,BLE_ERR_REM_USER_CONN_TERM);break;}
        /* A restored bond may already be encrypted when CONNECT arrives. */
        secure_ready();if(!secured_at)security_at=ms()+400;
        break;
    case BLE_GAP_EVENT_ENC_CHANGE:
        if(e->enc_change.conn_handle!=conn)break;
        printf("[NATIVE] ENCRYPT status=%d h=%u\n",e->enc_change.status,conn);
        if(!e->enc_change.status) secure_ready();
        else if(e->enc_change.status==BLE_HS_ENOTCONN) {
            /* Security teardown reports ENOTCONN before GAP DISCONNECT.
             * Do not mislabel an already lost radio link as pairing refusal. */
            status("LINK LOST DURING PAIR");
        }
        else {status("PAIR FAILED - RESET?");ble_gap_terminate(conn,BLE_ERR_REM_USER_CONN_TERM);}
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        if(e->disconnect.conn.conn_handle!=conn)break;
        printf("[NATIVE] DISCONNECTED reason=%d hci=0x%02x h=%u\n",e->disconnect.reason,
            e->disconnect.reason>=BLE_HS_ERR_HCI_BASE && e->disconnect.reason<BLE_HS_ERR_HCI_BASE+256?e->disconnect.reason-BLE_HS_ERR_HCI_BASE:0,conn);
        conn=BLE_HS_CONN_HANDLE_NONE;secured_at=security_at=0;clear_client();
        xQueueReset(notices);xQueueReset(times);flags(false,false,false,false);
        if(clearing) {int rc=ble_store_clear();clearing=false;printf("[NATIVE] CLEAR BONDS rc=%d\n",rc);}
        if(enabled)advertise(false);else status("BLE OFF");
        break;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc d;
        if(ble_gap_conn_find(e->repeat_pairing.conn_handle,&d))return BLE_GAP_REPEAT_PAIRING_IGNORE;
        int rc=ble_store_util_delete_peer(&d.peer_id_addr);
        printf("[NATIVE] REPAIR stale peer rc=%d\n",rc);
        return rc?BLE_GAP_REPEAT_PAIRING_IGNORE:BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    case BLE_GAP_EVENT_NOTIFY_RX:
        if(enabled && e->notify_rx.conn_handle==conn)notify(e->notify_rx.attr_handle,e->notify_rx.om);
        break;
    case BLE_GAP_EVENT_CONN_UPDATE: {
        struct ble_gap_conn_desc d;
        if(!ble_gap_conn_find(conn,&d))printf("[NATIVE] PARAM rc=%d interval=%u latency=%u timeout=%u\n",e->conn_update.status,d.conn_itvl,d.conn_latency,d.supervision_timeout);
        break;
    }
    case BLE_GAP_EVENT_ADV_COMPLETE: if(enabled)advertise(true);break;
    default:break;
    }
    return 0;
}
static int service_cb(uint16_t h,const struct ble_gatt_error *err,const struct ble_gatt_svc *s,void *arg) {
    if(!current(h,arg))return 0;
    if(!err->status && s) {
        if(!ble_uuid_cmp(&s->uuid.u,&ANCS.u)) {svc_start=s->start_handle;svc_end=s->end_handle;}
        if(!ble_uuid_cmp(&s->uuid.u,&CTS.u)) {cts_start=s->start_handle;cts_end=s->end_handle;}
    } else if(err->status==BLE_HS_EDONE) {
        busy=false;printf("[NATIVE] SERVICES ANCS=%u-%u CTS=%u-%u\n",svc_start,svc_end,cts_start,cts_end);
        if(svc_start)phase=CHARS;else retry("ANCS absent",BLE_HS_ENOENT);
    } else retry("services",err->status);
    return 0;
}
static int chr_cb(uint16_t h,const struct ble_gatt_error *err,const struct ble_gatt_chr *c,void *arg) {
    if(!current(h,arg))return 0;
    if(!err->status && c) {
        if(phase==TIME_CHARS) {if(!ble_uuid_cmp(&c->uuid.u,&CT.u))time_h=c->val_handle;}
        else {
            if(source && c->def_handle>source && c->def_handle-1<source_end)source_end=c->def_handle-1;
            if(data_h && c->def_handle>data_h && c->def_handle-1<data_end)data_end=c->def_handle-1;
            if(!ble_uuid_cmp(&c->uuid.u,&NS.u)) {source=c->val_handle;source_end=svc_end;}
            if(!ble_uuid_cmp(&c->uuid.u,&DS.u)) {data_h=c->val_handle;data_end=svc_end;}
            if(!ble_uuid_cmp(&c->uuid.u,&CP.u))control=c->val_handle;
        }
    } else if(err->status==BLE_HS_EDONE) {
        busy=false;
        if(phase==TIME_CHARS)phase=time_h?TIME_READ:RUNNING;
        else if(source && data_h && control)phase=DATA_DSC;
        else retry("characteristics",BLE_HS_ENOENT);
    } else retry("characteristics",err->status);
    return 0;
}
static int dsc_cb(uint16_t h,const struct ble_gatt_error *err,uint16_t ch,const struct ble_gatt_dsc *d,void *arg) {
    (void)ch;if(!current(h,arg))return 0;
    if(!err->status && d) {
        if(!ble_uuid_cmp(&d->uuid.u,&CCCD.u)) {if(phase==DATA_DSC)data_cccd=d->handle;else source_cccd=d->handle;}
    } else if(err->status==BLE_HS_EDONE) {
        busy=false;if(phase==DATA_DSC && data_cccd)phase=DATA_SUB;
        else if(phase==SOURCE_DSC && source_cccd)phase=SOURCE_SUB;
        else retry("CCCD absent",BLE_HS_ENOENT);
    } else retry("descriptors",err->status);
    return 0;
}
static int write_cb(uint16_t h,const struct ble_gatt_error *err,struct ble_gatt_attr *a,void *arg) {
    (void)a;if(!current(h,arg))return 0;
    busy=false;
    if(err->status) {retry("write",err->status);return 0;}
    if(phase==DATA_SUB)phase=SOURCE_DSC;
    else if(phase==SOURCE_SUB) {
        flags(true,true,true,false);status("ANCS READY");phase=cts_start?TIME_CHARS:RUNNING;
    } else if(phase==ATTR_WRITE)phase=RUNNING;
    return 0;
}
static int read_cb(uint16_t h,const struct ble_gatt_error *err,struct ble_gatt_attr *a,void *arg) {
    if(!current(h,arg))return 0;
    busy=false;phase=RUNNING;time_at=ms();
    if(!err->status && a && OS_MBUF_PKTLEN(a->om)>=7) {
        uint8_t b[7];os_mbuf_copydata(a->om,0,7,b);
        BlePhoneTime t={b[0]|(uint16_t)b[1]<<8,b[2],b[3],b[4],b[5],b[6]};
        if(t.year>=2024 && t.year<=2099 && t.month>=1 && t.month<=12 && t.day>=1 && t.day<=31 && t.hour<24 && t.minute<60 && t.second<60) {
            xQueueOverwrite(times,&t);status("ANCS + CTS READY");
        }
    } else printf("[NATIVE] CTS read rc=%d\n",err->status);
    return 0;
}
static void request_attributes(void) {
    Notice n=requests[0];memmove(requests,requests+1,(--request_count)*sizeof(*requests));
    ancs_parser_begin(&parser,n.uid,n.flags,n.category);
    uint8_t b[]={0,(uint8_t)n.uid,(uint8_t)(n.uid>>8),(uint8_t)(n.uid>>16),(uint8_t)(n.uid>>24),0,1,72,0,3,180,0};
    phase=ATTR_WRITE;busy=true;deadline=ms()+8000;
    int rc=ble_gattc_write_flat(conn,control,b,sizeof(b),write_cb,token());
    if(rc)retry("attributes",rc);
}
static void work(void) {
    int rc=0;uint16_t on=1;
    printf("[NATIVE] GATT phase=%u h=%u epoch=%lu\n",phase,conn,(unsigned long)epoch);
    busy=true;deadline=ms()+10000;
    switch(phase) {
    case SERVICES:rc=ble_gattc_disc_all_svcs(conn,service_cb,token());break;
    case CHARS:rc=ble_gattc_disc_all_chrs(conn,svc_start,svc_end,chr_cb,token());break;
    case DATA_DSC:rc=ble_gattc_disc_all_dscs(conn,data_h,data_end,dsc_cb,token());break;
    case SOURCE_DSC:rc=ble_gattc_disc_all_dscs(conn,source,source_end,dsc_cb,token());break;
    case DATA_SUB:rc=ble_gattc_write_flat(conn,data_cccd,&on,2,write_cb,token());break;
    case SOURCE_SUB:rc=ble_gattc_write_flat(conn,source_cccd,&on,2,write_cb,token());break;
    case TIME_CHARS:rc=ble_gattc_disc_all_chrs(conn,cts_start,cts_end,chr_cb,token());break;
    case TIME_READ:rc=ble_gattc_read(conn,time_h,read_cb,token());break;
    default:busy=false;break;
    }
    if(rc)retry("start",rc);
}
static void tick(struct ble_npl_event *ev) {
    (void)ev;uint32_t now=ms();uint8_t command;
    while(xQueueReceive(commands,&command,0)==pdTRUE) {
        if(command==CMD_OFF) {
            enabled=false;ble_gap_adv_stop();clear_client();secured_at=security_at=0;
            if(conn!=BLE_HS_CONN_HANDLE_NONE)ble_gap_terminate(conn,BLE_ERR_REM_USER_CONN_TERM);
            flags(false,false,false,false);xQueueReset(notices);xQueueReset(times);
            /* Quiesce on the host task; the Arduino caller may now stop/deinit
             * the host without a periodic callout starting another procedure. */
            ble_npl_callout_stop(&ticker);xSemaphoreGive(off_ack);return;
        } else if(command==CMD_RESET) {
            enabled=(command==CMD_RESET);ble_gap_adv_stop();clear_client();
            secured_at=security_at=0;flags(conn!=BLE_HS_CONN_HANDLE_NONE,false,false,false);
            if(conn!=BLE_HS_CONN_HANDLE_NONE) {clearing=(command==CMD_RESET);ble_gap_terminate(conn,BLE_ERR_REM_USER_CONN_TERM);}
            else {if(command==CMD_RESET)printf("[NATIVE] CLEAR BONDS rc=%d\n",ble_store_clear());if(enabled)advertise(false);else status("BLE OFF");}
            xQueueReset(notices);xQueueReset(times);
        } else if(command==CMD_ON) {enabled=true;if(conn==BLE_HS_CONN_HANDLE_NONE)advertise(false);}
    }
    if(enabled && conn!=BLE_HS_CONN_HANDLE_NONE) {
        if(security_at && elapsed(now,security_at)) {
            security_at=0;secure_ready();
            if(!secured_at)printf("[NATIVE] SECURITY start rc=%d\n",ble_gap_security_initiate(conn));
        }
        if(secured_at) {
            if(!params_sent && now-secured_at>2500) {
                params_sent=true;
                struct ble_gap_upd_params p={.itvl_min=24,.itvl_max=36,.latency=2,.supervision_timeout=400};
                printf("[NATIVE] ECO PARAM request rc=%d\n",ble_gap_update_params(conn,&p));
            }
            if(busy && elapsed(now,deadline))retry("timeout",BLE_HS_ETIMEOUT);
            if(!busy && phase==IDLE && elapsed(now,due)) {retries=0;phase=SERVICES;}
            if(!busy && phase!=IDLE && phase!=RUNNING && elapsed(now,due))work();
            if(!busy && phase==RUNNING) {
                if(parser.active && elapsed(now,deadline)) {parser.active=false;due=now+1000;printf("[NATIVE] ATTRIBUTE timeout\n");}
                if(!parser.active && request_count && elapsed(now,due))request_attributes();
                else if(!parser.active && !request_count && time_h && now-time_at>3600000) {phase=TIME_READ;work();}
            }
        }
        if(now-alive_at>15000) {alive_at=now;printf("[NATIVE] ALIVE h=%u secure=%u phase=%u queued=%u\n",conn,secured_at!=0,phase,request_count);}
    } else if(enabled && !slow_adv && now-adv_at>20000)advertise(true);
    ble_npl_callout_reset(&ticker,ble_npl_time_ms_to_ticks32(enabled && conn!=BLE_HS_CONN_HANDLE_NONE?50:250));
}
static void sync_host(void) {
    int rc=ble_hs_util_ensure_addr(0);if(!rc)rc=ble_hs_id_infer_auto(0,&own_type);
    if(rc) {printf("[NATIVE] ADDRESS rc=%d\n",rc);return;}
    synced=true;ble_npl_callout_init(&ticker,nimble_port_get_dflt_eventq(),tick,NULL);
    ble_npl_callout_reset(&ticker,1);
}
static void reset_host(int reason) {printf("[NATIVE] HOST RESET %d\n",reason);synced=false;conn=BLE_HS_CONN_HANDLE_NONE;secured_at=0;clear_client();flags(false,false,false,false);}
static void host_task(void *p) {
    (void)p;nimble_port_run();ble_npl_callout_deinit(&ticker);
    xSemaphoreGive(host_done);nimble_port_freertos_deinit();
}
static bool send_command(uint8_t c) {return commands && xQueueSend(commands,&c,0)==pdTRUE;}
bool blePhoneInit(bool resetBonds) {
    if(!initialized) {
        if(!commands)commands=xQueueCreate(4,sizeof(uint8_t));
        if(!notices)notices=xQueueCreate(4,sizeof(BlePhoneNotification));
        if(!times)times=xQueueCreate(1,sizeof(BlePhoneTime));
        if(!off_ack)off_ack=xSemaphoreCreateBinary();
        if(!host_done)host_done=xSemaphoreCreateBinary();
        if(!commands || !notices || !times || !off_ack || !host_done) {status("BLE QUEUE FAILED");return false;}
        xQueueReset(commands);xQueueReset(notices);xQueueReset(times);
        xSemaphoreTake(off_ack,0);xSemaphoreTake(host_done,0);
        enabled=false;synced=false;clearing=false;conn=BLE_HS_CONN_HANDLE_NONE;secured_at=security_at=0;clear_client();
        esp_err_t err=esp_nimble_hci_and_controller_init();
        if(err!=ESP_OK) {printf("[NATIVE] HCI init=%d\n",err);return false;}
        nimble_port_init();
        ble_hs_cfg.reset_cb=reset_host;ble_hs_cfg.sync_cb=sync_host;ble_hs_cfg.store_status_cb=ble_store_util_status_rr;
        ble_hs_cfg.sm_io_cap=BLE_HS_IO_NO_INPUT_OUTPUT;ble_hs_cfg.sm_bonding=1;ble_hs_cfg.sm_mitm=0;ble_hs_cfg.sm_sc=1;
        ble_hs_cfg.sm_our_key_dist=BLE_SM_PAIR_KEY_DIST_ENC|BLE_SM_PAIR_KEY_DIST_ID;
        ble_hs_cfg.sm_their_key_dist=BLE_SM_PAIR_KEY_DIST_ENC|BLE_SM_PAIR_KEY_DIST_ID;
        ble_svc_gap_init();ble_svc_gatt_init();ble_svc_gap_device_name_set(
#if CONFIG_IDF_TARGET_ESP32
        "NWatch 32"
#else
        "NWatch C3"
#endif
    );ble_svc_gap_device_appearance_set(0x00c0);
        ble_store_config_init();
        esp_err_t p0=esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_N6);
        esp_err_t p1=esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_N6);
        /* This C3 controller only supports DEFAULT/ADV/SCAN power setters.
         * DEFAULT controls connected TX; per-CONN_HDL setters return 0x106. */
        printf("[NATIVE] TX -6dBm default=%d adv=%d modem-sleep enabled\n",p0,p1);
        printf("[NATIVE] GATT compiled svc=%d chr=%d dsc=%d read=%d write=%d\n",
            MYNEWT_VAL(BLE_GATT_DISC_ALL_SVCS),MYNEWT_VAL(BLE_GATT_DISC_ALL_CHRS),
            MYNEWT_VAL(BLE_GATT_DISC_ALL_DSCS),MYNEWT_VAL(BLE_GATT_READ),MYNEWT_VAL(BLE_GATT_WRITE));
        initialized=true;nimble_port_freertos_init(host_task);
    }
    return send_command(resetBonds?CMD_RESET:CMD_ON);
}
bool blePhoneShutdown(void) {
    if(!initialized)return true;
    if(!send_command(CMD_OFF) || xSemaphoreTake(off_ack,pdMS_TO_TICKS(3000))!=pdTRUE) {status("BLE OFF TIMEOUT");return false;}
    int rc=nimble_port_stop();
    if(rc) {printf("[NATIVE] STOP rc=%d\n",rc);return false;}
    if(xSemaphoreTake(host_done,pdMS_TO_TICKS(3000))!=pdTRUE) {status("HOST STOP TIMEOUT");return false;}
    nimble_port_deinit();
    esp_err_t err=esp_nimble_hci_and_controller_deinit();
    initialized=false;synced=false;conn=BLE_HS_CONN_HANDLE_NONE;
    flags(false,false,false,false);status("BLE OFF");
    printf("[NATIVE] CONTROLLER OFF rc=%d\n",err);
    return err==ESP_OK;
}
bool blePhoneResetPairing(void) {return initialized && send_command(CMD_RESET);}
bool blePhonePaused(void) {portENTER_CRITICAL(&view_mux);bool b=view.paused;portEXIT_CRITICAL(&view_mux);return b;}
void blePhonePoll(void) { /* Host owns processing; never block the OLED/input loop. */ }
bool blePhoneTakeTime(BlePhoneTime *v) {return times && xQueueReceive(times,v,0)==pdTRUE;}
bool blePhoneTakeNotification(BlePhoneNotification *v) {return notices && xQueueReceive(notices,v,0)==pdTRUE;}
bool blePhoneConnected(void) {portENTER_CRITICAL(&view_mux);bool b=view.connected;portEXIT_CRITICAL(&view_mux);return b;}
bool blePhoneSecured(void) {portENTER_CRITICAL(&view_mux);bool b=view.secured;portEXIT_CRITICAL(&view_mux);return b;}
bool blePhoneReady(void) {portENTER_CRITICAL(&view_mux);bool b=view.ready;portEXIT_CRITICAL(&view_mux);return b;}
const char *blePhoneStateText(void) {static char text[32];portENTER_CRITICAL(&view_mux);memcpy(text,view.text,sizeof(text));portEXIT_CRITICAL(&view_mux);return text;}
