#ifndef INFRA_GPIOBUTTON_H
#define INFRA_GPIOBUTTON_H

#include <Arduino.h>
#include <cstdint>

namespace gps::infra {

enum class ButtonEvent : uint8_t {
  None = 0,
  ShortPress = 1,
  LongPress = 2
};

/**
 * @brief Máquina de estados del botón físico (GPIO 0 / D3 en ESP8266 NodeMCU):
 * - Muestreo periódico de 25 ms por tick
 * - Filtro antirrebote de 2 ticks (50 ms) tanto en presión como en liberación
 * - Disparo inmediato de LongPress al alcanzar 200 ticks (5,000 ms) sin esperar a soltar el botón
 */
class GpioButton {
public:
  static constexpr unsigned long SAMPLE_TICK_MS = 25;

  GpioButton(uint8_t pin, uint16_t debounceMs = 50,
             uint16_t longPressMs = 5000)
      : pin_(pin), debounceMs_(debounceMs), longPressMs_(longPressMs) {}

  void init();
  ButtonEvent update();
  uint8_t getPin() const { return pin_; }
  bool isPhysicallyPressed() const;

private:
  uint8_t pin_;
  uint16_t debounceMs_;
  uint16_t longPressMs_;

  unsigned long lastSampleMs_ = 0;
  uint32_t pressTicks_ = 0;
  uint32_t releaseTicks_ = 0;
  bool samplingActive_ = false;
  bool longPressFired_ = false;
};

} // namespace gps::infra

#endif // INFRA_GPIOBUTTON_H

