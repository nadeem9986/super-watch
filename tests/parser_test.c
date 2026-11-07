#include "../src/ancs_parser.h"
#include <assert.h>
#include <stdio.h>
static const uint8_t response[]={0,0x78,0x56,0x34,0x12,0,3,0,'a','p','p',1,4,0,'T','e','s','t',3,5,0,'H','e','l','l','o'};
int main(void) {
    AncsParser p;
    for(size_t split=0;split<=sizeof(response);++split) {
        ancs_parser_begin(&p,0x12345678,2,1);
        ancs_parser_feed(&p,response,split);ancs_parser_feed(&p,response+split,sizeof(response)-split);
        assert(p.valid && !p.active && !strcmp(p.value.app,"app") && !strcmp(p.value.title,"Test") && !strcmp(p.value.message,"Hello"));
        assert(p.value.category==1 && p.value.flags==2);
    }
    ancs_parser_begin(&p,0x12345678,0,0);
    for(size_t i=0;i<sizeof(response);++i)ancs_parser_feed(&p,response+i,1);
    assert(p.valid);
    ancs_parser_begin(&p,99,0,0);ancs_parser_feed(&p,response,sizeof(response));assert(!p.active && !p.valid);
    const uint8_t empty[]={0,1,0,0,0,0,0,0,1,0,0,3,0,0};
    ancs_parser_begin(&p,1,0,0);ancs_parser_feed(&p,empty,sizeof(empty));assert(p.valid);
    const uint8_t invalid[]={0,1,0,0,0,0,0xff,0xff};
    ancs_parser_begin(&p,1,0,0);ancs_parser_feed(&p,invalid,sizeof(invalid));assert(!p.valid && !p.active);
    uint8_t long_response[5+3+100+3+3];size_t n=0;
    memcpy(long_response,response,5);n=5;long_response[n++]=0;long_response[n++]=100;long_response[n++]=0;
    memset(long_response+n,'x',100);n+=100;
    long_response[n++]=1;long_response[n++]=0;long_response[n++]=0;long_response[n++]=3;long_response[n++]=0;long_response[n++]=0;
    ancs_parser_begin(&p,0x12345678,0,0);ancs_parser_feed(&p,long_response,n);
    assert(p.valid && strlen(p.value.app)==48 && p.value.app[48]==0);
    puts("ANCS parser: all split points, byte fragments, invalid UID/length, empty and truncated fields PASS");
}
