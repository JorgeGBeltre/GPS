#include "SysUtils.h"
#include "config.h"

bool SysUtils::configurarTiempoNTP() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  time_t now = time(nullptr);
  unsigned long timeout = millis();
  while (now < 1000000000 && (millis() - timeout < 5000)) {
    delay(100);
    now = time(nullptr);
  }
  return now >= 1000000000;
}

String SysUtils::getISOTimestamp() {
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

bool SysUtils::isBase64Char(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

bool SysUtils::base64DecodeToBuffer(const String &encoded, uint8_t *out,
                                    size_t outMax, size_t &outLen) {
  outLen = 0;
  int inLen = encoded.length();
  int i = 0, j = 0, in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];

  while (inLen-- && (encoded[in_] != '=') &&
         SysUtils::isBase64Char(encoded[in_])) {
    char_array_4[i++] = encoded[in_];
    in_++;
    if (i == 4) {
      for (i = 0; i < 4; i++) {
        char_array_4[i] =
            (encoded[in_ - 4 + i] >= 'A' && encoded[in_ - 4 + i] <= 'Z')
                ? encoded[in_ - 4 + i] - 'A'
            : (encoded[in_ - 4 + i] >= 'a' && encoded[in_ - 4 + i] <= 'z')
                ? encoded[in_ - 4 + i] - 'a' + 26
            : (encoded[in_ - 4 + i] >= '0' && encoded[in_ - 4 + i] <= '9')
                ? encoded[in_ - 4 + i] - '0' + 52
            : (encoded[in_ - 4 + i] == '+') ? 62
                                            : 63;
      }
      char_array_3[0] =
          (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] =
          ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

      for (i = 0; (i < 3); i++) {
        if (outLen < outMax)
          out[outLen++] = char_array_3[i];
      }
      i = 0;
    }
  }
  return true;
}
