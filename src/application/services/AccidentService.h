#ifndef APPLICATION_ACCIDENTSERVICE_H
#define APPLICATION_ACCIDENTSERVICE_H

#include "domain/entities/DeviceInfo.h"
#include "domain/interfaces/IAccidentDetector.h"
#include "domain/interfaces/IClock.h"
#include "domain/interfaces/IGpsProvider.h"
#include "domain/interfaces/IGsmAlertSender.h"
#include "domain/interfaces/IMqttTransport.h"

// Orchestrates accident handling: poll the detector, and on a fresh event fan
// out to the GSM alert and the MQTT telemetry topic. Knows nothing about
// concrete hardware — only the Domain interfaces, injected via the constructor.
class AccidentService {
public:
  AccidentService(IAccidentDetector &detector, IGpsProvider &gps,
                  IGsmAlertSender &gsm, IMqttTransport &mqtt, IClock &clock,
                  const DeviceInfo &device);

  // Call once per main-loop iteration.
  void process();

private:
  IAccidentDetector &m_detector;
  IGpsProvider &m_gps;
  IGsmAlertSender &m_gsm;
  IMqttTransport &m_mqtt;
  IClock &m_clock;
  const DeviceInfo &m_device;
};

#endif // APPLICATION_ACCIDENTSERVICE_H
