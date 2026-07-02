#ifndef INFRA_LITTLEFSCONFIGREPOSITORY_H
#define INFRA_LITTLEFSCONFIGREPOSITORY_H

#include <Arduino.h>

#include "domain/interfaces/IConfigRepository.h"

// IConfigRepository backed by a JSON file on LittleFS. Formerly DeviceConfig.
class LittleFsConfigRepository : public IConfigRepository {
public:
  LittleFsConfigRepository();

  void begin() override;
  void load() override;
  void save() override;
  AccidentConfig &config() override { return currentConfig; }

private:
  AccidentConfig currentConfig;
  const char *configFilePath = "/config.json";
};

#endif // INFRA_LITTLEFSCONFIGREPOSITORY_H
