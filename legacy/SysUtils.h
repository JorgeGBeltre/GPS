#ifndef SYSUTILS_H
#define SYSUTILS_H

#include <Arduino.h>
#include <time.h>

class SysUtils {
public:
  static bool configurarTiempoNTP();
  static String getISOTimestamp();

  static bool isBase64Char(unsigned char c);
  static bool base64DecodeToBuffer(const String &encoded, uint8_t *out,
                                   size_t outMax, size_t &outLen);
};

#endif // SYSUTILS_H
