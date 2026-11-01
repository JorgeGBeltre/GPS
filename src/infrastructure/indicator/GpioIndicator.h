#ifndef INFRA_GPIOINDICATOR_H
#define INFRA_GPIOINDICATOR_H

#include <Arduino.h>
#include <cstdint>

namespace gps::infra {

enum class IndicatorStatus : uint8_t {
  Disconnected = 0, // Sin Wi-Fi (1000 ms ON / 1000 ms OFF)
  Connecting = 1,   // Wi-Fi conectado, conectando a MQTT (200 ms ON / 1800 ms OFF)
  QuickPairing = 2, // Quick Pairing SmartConfig (100/100/100/700 ms: 2 destellos rápidos + pausa)
  CompatibleAp = 3, // SoftAP VESIS-<ChipID> (100 ms ON / 100 ms OFF continuo)
  Operational = 4,  // Conectado a Wi-Fi y MQTT (Encendido fijo si sledOnline=true)
  Error = 5         // 3 destellos rápidos + pausa
};

/**
 * @brief Generador de patrones no bloqueante para el LED de estado (GPIO 2 / D4 en NodeMCU, activo en LOW)
 * con base de tiempo de 100 ms.
 */
class GpioIndicator {
public:
  static constexpr unsigned long BASE_TICK_MS = 100;

  explicit GpioIndicator(uint8_t pin, bool activeLow = true)
      : pin_(pin), activeLow_(activeLow) {
    pinMode(pin_, OUTPUT);
    writePin(false);
  }

  void setSledOnline(bool enabled) {
    sledOnline_ = enabled;
    if (status_ == IndicatorStatus::Operational) {
      writePin(sledOnline_);
    }
  }

  bool isSledOnline() const { return sledOnline_; }

  void show(IndicatorStatus status) {
    if (status_ != status) {
      status_ = status;
      tickStep_ = 0;
      lastTickMs_ = 0;
    }
    if (status_ == IndicatorStatus::Operational) {
      writePin(sledOnline_);
    } else {
      update();
    }
  }

  void update() {
    if (status_ == IndicatorStatus::Operational) {
      writePin(sledOnline_);
      return;
    }

    const unsigned long now = millis();
    if (lastTickMs_ != 0 && (now - lastTickMs_) < BASE_TICK_MS) {
      return;
    }
    lastTickMs_ = now;

    bool turnOn = false;
    switch (status_) {
    case IndicatorStatus::Disconnected:
      turnOn = (tickStep_ % 20U) < 10U;
      break;
    case IndicatorStatus::Connecting:
      turnOn = (tickStep_ % 20U) < 2U;
      break;
    case IndicatorStatus::QuickPairing: {
      const uint32_t phase = tickStep_ % 10U;
      turnOn = (phase == 0U || phase == 2U);
      break;
    }
    case IndicatorStatus::CompatibleAp:
      turnOn = (tickStep_ % 2U) == 0U;
      break;
    case IndicatorStatus::Error: {
      const uint32_t phase = tickStep_ % 10U;
      turnOn = (phase == 0U || phase == 2U || phase == 4U);
      break;
    }
    case IndicatorStatus::Operational:
    default:
      turnOn = sledOnline_;
      break;
    }

    ++tickStep_;
    writePin(turnOn);
  }

private:
  void writePin(bool logicalOn) {
    if (activeLow_) {
      digitalWrite(pin_, logicalOn ? LOW : HIGH);
    } else {
      digitalWrite(pin_, logicalOn ? HIGH : LOW);
    }
  }

  uint8_t pin_;
  bool activeLow_;
  IndicatorStatus status_ = IndicatorStatus::Disconnected;
  unsigned long lastTickMs_ = 0;
  uint32_t tickStep_ = 0;
  bool sledOnline_ = true;
};

} // namespace gps::infra

#endif // INFRA_GPIOINDICATOR_H

