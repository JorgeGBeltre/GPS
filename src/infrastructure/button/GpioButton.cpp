#include "infrastructure/button/GpioButton.h"

namespace gps::infra {

namespace {
volatile bool g_btnIsrTriggered = false;

void IRAM_ATTR onButtonFallingEdgeIsr() {
  g_btnIsrTriggered = true;
}
} // namespace

void GpioButton::init() {
  pinMode(pin_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_), onButtonFallingEdgeIsr, FALLING);
  pressTicks_ = 0;
  releaseTicks_ = 0;
  longPressFired_ = false;
  if (digitalRead(pin_) == LOW) {
    samplingActive_ = true;
    lastSampleMs_ = millis() - SAMPLE_TICK_MS;
  } else {
    samplingActive_ = false;
  }
}

bool GpioButton::isPhysicallyPressed() const {
  return digitalRead(pin_) == LOW;
}

ButtonEvent GpioButton::update() {
  const unsigned long now = millis();

  if ((g_btnIsrTriggered || digitalRead(pin_) == LOW) && !samplingActive_) {
    g_btnIsrTriggered = false;
    samplingActive_ = true;
    pressTicks_ = 0;
    releaseTicks_ = 0;
    longPressFired_ = false;
    lastSampleMs_ = now - SAMPLE_TICK_MS;
  }

  if (!samplingActive_) {
    return ButtonEvent::None;
  }

  if ((now - lastSampleMs_) < SAMPLE_TICK_MS) {
    return ButtonEvent::None;
  }
  lastSampleMs_ = now;

  const uint32_t debounceTicks =
      (debounceMs_ >= SAMPLE_TICK_MS) ? (debounceMs_ / SAMPLE_TICK_MS) : 2U;
  const uint32_t longPressTicks =
      (longPressMs_ >= SAMPLE_TICK_MS) ? (longPressMs_ / SAMPLE_TICK_MS) : 200U;

  const bool pinLow = (digitalRead(pin_) == LOW);
  if (pinLow) {
    releaseTicks_ = 0;
    ++pressTicks_;
    if (pressTicks_ >= longPressTicks && !longPressFired_) {
      longPressFired_ = true;
      return ButtonEvent::LongPress;
    }
    return ButtonEvent::None;
  }

  ++releaseTicks_;
  if (releaseTicks_ >= debounceTicks) {
    const bool validShortPress =
        (pressTicks_ >= debounceTicks && !longPressFired_);
    samplingActive_ = false;
    pressTicks_ = 0;
    releaseTicks_ = 0;
    longPressFired_ = false;
    g_btnIsrTriggered = false;
    if (validShortPress) {
      return ButtonEvent::ShortPress;
    }
  }

  return ButtonEvent::None;
}

} // namespace gps::infra

