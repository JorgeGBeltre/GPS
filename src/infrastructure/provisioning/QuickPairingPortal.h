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

class QuickPairingPortal {
public:
  static void setCellularState(bool simPresent, bool apnAutoDetected,
                               const String &resolvedApn);

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
