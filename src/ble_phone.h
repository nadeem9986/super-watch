#pragma once

#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct BlePhoneTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} BlePhoneTime;

typedef struct BlePhoneNotification {
  uint8_t category;
  uint8_t flags;
  char app[49];
  char title[73];
  char message[181];
} BlePhoneNotification;

bool blePhoneInit(bool resetBonds);
bool blePhoneShutdown();
bool blePhoneResetPairing();
bool blePhonePaused();
void blePhonePoll();
bool blePhoneTakeTime(BlePhoneTime *value);
bool blePhoneTakeNotification(BlePhoneNotification *value);
bool blePhoneConnected();
bool blePhoneSecured();
bool blePhoneReady();
const char *blePhoneStateText();
#ifdef __cplusplus
}
#endif

