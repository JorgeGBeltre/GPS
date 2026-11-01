#ifndef INFRA_QUICKPAIRINGPORTAL_H
#define INFRA_QUICKPAIRINGPORTAL_H

#include <Arduino.h>
#include <cstdint>
#include "domain/interfaces/IConfigRepository.h"
#include "infrastructure/button/GpioButton.h"
#include "infrastructure/indicator/GpioIndicator.h"

namespace gps::infra {

enum class QuickPairingResult : uint8_t {
  Provisioned = 0,       // Credenciales recibidas por SmartConfig (ESP-Touch / AirKiss)
  SwitchToSoftAp = 1,    // 2ª pulsación larga de 5.0 s -> Conmutar a SoftAP VESIS-<ChipID>
  CancelledByButton = 2, // Pulsación corta -> Salir de modo emparejamiento
  Timeout = 3            // Tiempo agotado
};

/**
 * @brief Emparejamiento en 2 etapas para ESP8266 NodeMCU:
 * - Etapa 1 (`runQuickPairing`): SmartConfig nativo (`ESP-Touch / AirKiss`) con doble destello LED (`100/100/100/700 ms`),
 *   transición a Etapa 2 en una 2ª pulsación larga de 5.0 s y cancelación por pulsación corta.
 * - Etapa 2 (`runSoftApPortal`): Portal cautivo SoftAP `VESIS-<ChipID>` (clave `12345678`, IP `192.168.4.1`)
 *   con `ESP8266WebServer` + `DNSServer`, guardando Wi-Fi y `AccidentConfig` en LittleFS (`/config.json`).
 */
class QuickPairingPortal {
public:
  static QuickPairingResult runQuickPairing(IConfigRepository &configRepo,
                                            GpioIndicator &indicator,
                                            GpioButton &button,
                                            unsigned long timeoutMs = 180000UL);

  static bool runSoftApPortal(const String &chipId,
                              IConfigRepository &configRepo,
                              GpioIndicator &indicator, GpioButton &button,
                              unsigned long timeoutMs = 180000UL);

  static void runPairingWorkflow(const String &chipId,
                                 IConfigRepository &configRepo,
                                 GpioIndicator &indicator, GpioButton &button,
                                 bool isInitialUnprovisionedBoot = false);
};

} // namespace gps::infra

#endif // INFRA_QUICKPAIRINGPORTAL_H

