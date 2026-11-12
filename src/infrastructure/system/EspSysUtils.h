#ifndef INFRA_ESPSYSUTILS_H
#define INFRA_ESPSYSUTILS_H

#include <Arduino.h>
#include <cstdint>

// System helpers: Base64 utilities + RTC User Memory boot diagnostics
// (TRGR_212 PsmWakeStateService) and 2-stage loop-cadence watchdog
// (TRGR_212 MitWatchdogService) with zero heap usage.
namespace EspSysUtils {

bool isBase64Char(unsigned char c);
bool base64DecodeToBuffer(const String &encoded, uint8_t *out, size_t outMax,
                          size_t &outLen);

void initBootAndWatchdog();
void heartbeat();
void setWatchdogPaused(bool paused);
void markGracefulReboot();

uint32_t bootCount();
uint16_t unexpectedReboots();
uint16_t wdtOffences();
const char *resetReason();

} // namespace EspSysUtils

#endif // INFRA_ESPSYSUTILS_H
