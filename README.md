# Pico 2 W Photoreactor Controller (PlatformIO)

This project starts from the supplied `main(4).cpp`, `temperature_ui(1).h`,
`oxygen_sensor(1).h`, and `sensor_debug.cpp`. Temperature control, default 0 C
setpoint, pin assignments, cooling polarity, encoder control, OLED layout and
PID constants are retained. Added: local Wi-Fi AP, dashboard, CSV logging, and
authenticated web OTA.

## Before first upload

1. Edit `include/device_config.h`: replace **both** password placeholders with
   different strong passwords (at least 12-16 random characters). Set the AP
   SSID and device ID for your instrument. Never deploy unchanged passwords.
2. Verify the installed board is **Pico 2 W**. PlatformIO board is `rpipico2w`.
3. Connect using USB and run `pio run -e pico -t upload` in the project root.
   Use PlatformIO's Upload task if `pio` is not on the system PATH.
4. Connect phone/PC to configured AP, then open `http://192.168.4.1`.
5. Use `http://192.168.4.1/update` to upload a compatible **firmware binary**
   after completing an experiment. Browser prompts for administrator credentials.

## Files

- `src/main.cpp`: original control loop with hooks for web + storage
- `include/temperature_ui.h`: original rotary decoder and UI setpoint logic
- `include/oxygen_sensor.h`: original oxygen sensor protocol
- `src/sensor_debug.cpp`: original, separate motor-off hardware debug program
- `include/device_config.h`: AP identity and credentials; firmware version
- `include/web_manager.h`: web dashboard, JSON API, CSV download and OTA
- `include/data_logger.h`: LittleFS log file with rollover

## API

- `/` status dashboard; live graph is browser-memory only (last 3600 readings)
- `/api/status` live JSON (read only)
- `/api/log/current` newest CSV
- `/api/log/previous` previous CSV (rotated out at approx. 150 kB)
- `/update` authenticated firmware upload (HTTP Basic auth over plain HTTP)

Logs have `uptime_ms` rather than calendar dates because no battery-backed RTC or
external NTP source is configured. The time counter restarts on each boot; logs
may include more than one boot. For production experiments needing auditable
wall-clock times, add RTC or time synchronization. CSV writes happen at most
once per 10 s and are NOT guaranteed to be deterministic in latency.

## Important deployment limitations

- This is a **source implementation**, not hardware-validated firmware.
- WebServer requests, filesystem writes, and OTA are blocking operations. These
  can delay the 50 ms PID schedule; OTA can stop ordinary control entirely.
  **Never perform OTA while a temperature-controlled experiment is running.**
- No pre-setpoint arming logic was introduced; startup still controls toward 0 C.
- AP WPA2 and Basic auth offer local access control, but Basic auth goes over
  **unencrypted HTTP**. Use a unique AP password and admin password; do not
  connect the device to the public Internet or forward port 80.
- For delivery-level OTA integrity, configure **signed binaries** according to
  Arduino-Pico's OTA signing documentation; this example does not implement
  signing and is not appropriate for untrusted networks.
- LittleFS deliberately does not auto-format on mount errors. First boot
  needs an initialized filesystem: if mount fails, create/upload a filesystem
  image using PlatformIO `Upload Filesystem Image` **before collecting data**.
  **Do not upload a filesystem image after field data exists** because it can
  overwrite logs. A blank `data/` folder is provided for first initialization.
- OTA uses LittleFS for a firmware staging image; the configured 1.5 MiB FS
  must fit the largest built firmware plus existing logs and overhead. Confirm
  actual built `.bin` size after compilation and check OTA with the biggest
  expected image before delivery. Rotation holds at most ~300 kB of CSV logs.
- USB/BOOTSEL recovery access is recommended in case Wi-Fi or OTA breaks.
- `sensor-debug` build does not drive motors; it retains the original sketch.

## Build / test checklist

- Build `pico` and `sensor-debug` PlatformIO environments
- Check both VNH5019 outputs and thermocouple error -> PWM=0
- Probe I2C OLED and O2 sensor simultaneously
- AP address and two different credentials; verify unauthorized OTA denied
- Verify live API, browser dashboard, 10-second CSV entries and log rotation
- Verify old temperature logs remain after successful OTA
- Test web refresh and CSV download during steady regulation for PID jitter
- Update firmware only during an idle/allowed maintenance window
- Test interrupted OTA and USB recovery on a spare board

Reference: https://arduino-pico.readthedocs.io/en/latest/ota.html
