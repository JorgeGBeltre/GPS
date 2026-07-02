#ifndef DOMAIN_ICLOCK_H
#define DOMAIN_ICLOCK_H

#include <Arduino.h>

// Abstraction over wall-clock time / NTP. Concrete impl: EspClock.
class IClock {
public:
  virtual ~IClock() = default;

  virtual bool configureNtp() = 0;
  virtual String getISOTimestamp() = 0;
};

#endif // DOMAIN_ICLOCK_H
