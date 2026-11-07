#pragma once
#include "ble_phone.h"
#include <string.h>

/* Incremental ANCS command-0 parser. Transport may split at ANY byte. */
typedef struct {
    bool active, valid;
    uint32_t uid, received;
    uint8_t header[5], tuple[3], header_used, tuple_used, fields;
    uint16_t length, used;
    BlePhoneNotification value;
} AncsParser;
static inline uint32_t ancs_u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static inline void ancs_parser_begin(AncsParser *p, uint32_t uid, uint8_t flags, uint8_t category) {
    memset(p, 0, sizeof(*p)); p->active=true; p->uid=uid;
    p->value.flags=flags; p->value.category=category;
}
static inline bool ancs_parser_feed(AncsParser *p, const uint8_t *data, size_t size) {
    static const uint8_t expected[3]={0,1,3};
    for(size_t i=0; i<size && p->active; ++i) {
        uint8_t b=data[i];
        if(++p->received>4096) {p->active=false; break;}
        if(p->header_used<5) {
            p->header[p->header_used++]=b;
            if(p->header_used==5 && (p->header[0]!=0 || ancs_u32(p->header+1)!=p->uid)) p->active=false;
            continue;
        }
        if(p->tuple_used<3) {
            p->tuple[p->tuple_used++]=b;
            if(p->tuple_used<3) continue;
            if(p->fields>=3 || p->tuple[0]!=expected[p->fields]) {p->active=false; break;}
            p->length=p->tuple[1]|(uint16_t)p->tuple[2]<<8; p->used=0;
            if(p->length>2048) {p->active=false; break;}
        } else {
            char *out=p->fields==0?p->value.app:p->fields==1?p->value.title:p->value.message;
            size_t cap=p->fields==0?sizeof(p->value.app):p->fields==1?sizeof(p->value.title):sizeof(p->value.message);
            if(p->used<cap-1) out[p->used]=b?b:' ';
            ++p->used;
        }
        if(p->used==p->length) {
            p->tuple_used=0;
            if(++p->fields==3) {p->valid=true; p->active=false;}
        }
    }
    return p->valid;
}
