#ifndef INFRA_ESPSYSUTILS_H
#define INFRA_ESPSYSUTILS_H

#include <Arduino.h>

// Misc system helpers that don't belong to a specific device. Formerly the
// Base64 portion of SysUtils. Kept as a free-function namespace since these are
// stateless utilities (used by the OTA path).
namespace EspSysUtils {

bool isBase64Char(unsigned char c);
bool base64DecodeToBuffer(const String &encoded, uint8_t *out, size_t outMax,
                          size_t &outLen);

} // namespace EspSysUtils

#endif // INFRA_ESPSYSUTILS_H
