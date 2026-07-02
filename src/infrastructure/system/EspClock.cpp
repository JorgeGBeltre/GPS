#include "infrastructure/system/EspClock.h"

#include <time.h>

#include "config.h"

bool EspClock::configureNtp() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  time_t now = time(nullptr);
  unsigned long timeout = millis();
  while (now < 1000000000 && (millis() - timeout < 5000)) {
    delay(100);
    now = time(nullptr);
  }
  return now >= 1000000000;
}

String EspClock::getISOTimestamp() {
  time_t now = time(nullptr);
  if (now < 1000000000) {
    char fallback[32];
    snprintf(fallback, sizeof(fallback), "BOOT+%09lums", millis());
    return String(fallback);
  }
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}
