# Legacy sketch (pre-Clean-Architecture)

These are the **original** flat Arduino-IDE sources, kept for reference only.
They are **not compiled** — PlatformIO builds only `src/`, `lib/` and the
registered libraries, so anything in this folder is ignored by the build.

The code here was reorganized into layers under `src/`:

| Legacy file          | Now lives in                                             |
|----------------------|----------------------------------------------------------|
| `GPS.ino`            | `src/main.cpp` (composition root)                        |
| `Types.h`            | `src/domain/entities/*` + `SystemTypes.h`                |
| `config.h`           | `include/config.h`                                       |
| `AccidentMonitor.*`  | `src/infrastructure/sensors/Mpu6050Detector.*` + `src/application/AccidentAlgorithm.*` |
| `GpsModule.*`        | `src/infrastructure/gps/TinyGpsPlusProvider.*`           |
| `GsmModule.*`        | `src/infrastructure/gsm/Sim800lAlertSender.*`            |
| `MqttClientModule.*` | `src/infrastructure/mqtt/PubSubClientTransport.*`        |
| `DeviceConfig.*`     | `src/infrastructure/persistence/LittleFsConfigRepository.*` |
| `SysUtils.*`         | `src/infrastructure/system/EspClock.*` + `EspSysUtils.*` |

Once you've confirmed the PlatformIO build works on hardware, this folder can be
deleted.
