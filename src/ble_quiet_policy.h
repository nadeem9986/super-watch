#pragma once
#include <stdint.h>

// One bounded quiet window per connection; buttons always cancel it.
struct BleQuietPolicy {
    bool active=false, used=false;
    uint32_t started=0;
    void update(bool enabled,bool linked,bool ready,bool sleeping,bool button,uint32_t now) {
        if(!enabled || !linked) {active=false;used=false;return;}
        if(ready || sleeping || button) {active=false;used=true;return;}
        if(!used) {active=true;used=true;started=now;}
        if(active && uint32_t(now-started)>=20000)active=false;
    }
};
