#ifndef DOMAIN_ICONFIGREPOSITORY_H
#define DOMAIN_ICONFIGREPOSITORY_H

#include "config.h" // AccidentConfig

// Abstraction over persistence of device configuration. Concrete impl:
// LittleFsConfigRepository.
class IConfigRepository {
public:
  virtual ~IConfigRepository() = default;

  virtual void begin() = 0;
  virtual void load() = 0;
  virtual void save() = 0;
  virtual AccidentConfig &config() = 0;
};

#endif // DOMAIN_ICONFIGREPOSITORY_H
