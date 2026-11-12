#include "infrastructure/system/EspSysUtils.h"
#include <Ticker.h>
#include <user_interface.h>

namespace EspSysUtils {

namespace {
// Exactly 16 bytes (4 x uint32_t) for ESP.rtcUserMemoryRead/Write at slot 64.
struct RtcBootState {
  uint32_t magic;
  uint32_t bootCount;
  uint16_t unexpectedReboots;
  uint16_t wdtOffences;
  uint8_t gracefulFlag;
  uint8_t wdtStage2Flag;
  uint16_t crc16;
};
static_assert(sizeof(RtcBootState) == 16, "RtcBootState must be 4-byte aligned");

constexpr uint32_t kRtcSlotOffset = 64;
constexpr uint32_t kRtcMagic = 0x54524752UL; // 'TRGR'

RtcBootState g_rtcState = {0, 0, 0, 0, 0, 0, 0};
const char *g_resetReasonStr = "UNKNOWN";
Ticker g_wdtTicker;
volatile unsigned long g_lastHeartbeatMs = 0;
volatile uint8_t g_wdtStage = 0;
volatile bool g_wdtPaused = false;
volatile bool g_stage1WarnPending = false;

uint16_t computeCrc16(const RtcBootState &st) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&st);
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < sizeof(RtcBootState) - sizeof(uint16_t); ++i) {
    crc ^= static_cast<uint16_t>(bytes[i]) << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void writeRtcState() {
  g_rtcState.crc16 = computeCrc16(g_rtcState);
  ESP.rtcUserMemoryWrite(kRtcSlotOffset,
                         reinterpret_cast<uint32_t *>(&g_rtcState),
                         sizeof(RtcBootState));
}

const char *mapEsp8266ResetReason(uint32 reason, bool graceful, bool wdtStage2) {
  if (wdtStage2)
    return "MIT_WDT_STAGE2";
  switch (reason) {
  case REASON_DEFAULT_RST:
    return "POWERON";
  case REASON_WDT_RST:
    return "HW_WDT";
  case REASON_EXCEPTION_RST:
    return "EXCEPTION";
  case REASON_SOFT_WDT_RST:
    return "SOFT_WDT";
  case REASON_SOFT_RESTART:
    return graceful ? "GRACEFUL_SW" : "UNEXPECTED_SW";
  case REASON_DEEP_SLEEP_AWAKE:
    return "DEEPSLEEP";
  case REASON_EXT_SYS_RST:
    return "EXT_PIN";
  default:
    return "UNKNOWN";
  }
}

void onWatchdogTick() {
  const unsigned long nowMs = millis();
  if (g_wdtPaused) {
    g_lastHeartbeatMs = nowMs;
    g_wdtStage = 0;
    return;
  }
  constexpr unsigned long kStallWindowMs = 20000UL;
  if (nowMs - g_lastHeartbeatMs < kStallWindowMs)
    return;

  if (g_wdtStage == 0) {
    g_wdtStage = 1;
    g_rtcState.wdtOffences++;
    g_lastHeartbeatMs = nowMs;
    g_stage1WarnPending = true;
    writeRtcState();
  } else {
    g_rtcState.gracefulFlag = 0;
    g_rtcState.wdtStage2Flag = 1;
    g_rtcState.wdtOffences++;
    writeRtcState();
    ESP.restart();
  }
}
} // namespace

void initBootAndWatchdog() {
  const rst_info *info = ESP.getResetInfoPtr();
  const uint32 reason = info ? info->reason : REASON_DEFAULT_RST;

  RtcBootState loaded{};
  bool validRtc = ESP.rtcUserMemoryRead(
      kRtcSlotOffset, reinterpret_cast<uint32_t *>(&loaded), sizeof(loaded));
  if (validRtc) {
    validRtc = (loaded.magic == kRtcMagic) &&
               (loaded.crc16 == computeCrc16(loaded));
  }

  if (!validRtc || reason == REASON_DEFAULT_RST) {
    g_rtcState.magic = kRtcMagic;
    g_rtcState.bootCount = 1;
    g_rtcState.unexpectedReboots = 0;
    g_rtcState.wdtOffences = 0;
    g_rtcState.gracefulFlag = 0;
    g_rtcState.wdtStage2Flag = 0;
    g_resetReasonStr = "POWERON";
  } else {
    g_rtcState = loaded;
    g_rtcState.bootCount++;
    const bool graceful = (g_rtcState.gracefulFlag != 0);
    const bool wdtStage2 = (g_rtcState.wdtStage2Flag != 0);
    g_resetReasonStr = mapEsp8266ResetReason(reason, graceful, wdtStage2);
    if (!graceful && reason != REASON_EXT_SYS_RST &&
        reason != REASON_DEEP_SLEEP_AWAKE) {
      g_rtcState.unexpectedReboots++;
    }
    g_rtcState.gracefulFlag = 0;
    g_rtcState.wdtStage2Flag = 0;
  }
  writeRtcState();

  g_lastHeartbeatMs = millis();
  g_wdtStage = 0;
  g_wdtPaused = false;
  g_wdtTicker.detach();
  g_wdtTicker.attach_ms(5000, onWatchdogTick);
}

void heartbeat() {
  g_lastHeartbeatMs = millis();
  g_wdtStage = 0;
  ESP.wdtFeed();
  if (g_stage1WarnPending) {
    g_stage1WarnPending = false;
    Serial.printf("[WDT] Etapa-1 recuperada (ofensas=%u)\n",
                  static_cast<unsigned>(g_rtcState.wdtOffences));
  }
}

void setWatchdogPaused(bool paused) {
  g_wdtPaused = paused;
  g_lastHeartbeatMs = millis();
  g_wdtStage = 0;
}

void markGracefulReboot() {
  g_rtcState.gracefulFlag = 1;
  g_rtcState.wdtStage2Flag = 0;
  writeRtcState();
}

uint32_t bootCount() { return g_rtcState.bootCount; }
uint16_t unexpectedReboots() { return g_rtcState.unexpectedReboots; }
uint16_t wdtOffences() { return g_rtcState.wdtOffences; }
const char *resetReason() { return g_resetReasonStr; }

bool isBase64Char(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

bool base64DecodeToBuffer(const String &encoded, uint8_t *out, size_t outMax,
                          size_t &outLen) {
  outLen = 0;
  int inLen = encoded.length();
  int i = 0, in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];

  while (inLen-- && (encoded[in_] != '=') && isBase64Char(encoded[in_])) {
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

} // namespace EspSysUtils
