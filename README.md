# Pico 2 W Photoreactor Controller

The Pico reads temperature and O? concentration, runs the temperature PID every
50 ms, and controls the Peltier PWM. Use the rotary encoder to set the target
temperature or toggle the experiment status. The OLED shows temperature, O₂,
experiment status, and Wi-Fi connectivity.

## Setup

1. In `src/main.cpp`, set `STATION_SSID` and `STATION_PASSWORD` to the WPA2
   network that can reach the broker. The local controller runs with these
   settings empty.
2. Check `MQTT_BROKER`, `MQTT_PORT`, and `MQTT_TOPIC_PREFIX` in `src/main.cpp`.
   The defaults are `192.168.10.2:1883` and `labauto/pp/photoreactor/`.
3. Connect the Pico 2 W over USB and run `pio run -e pico -t upload`, or use
   PlatformIO's Upload task. The same firmware can be uploaded to both boards.

## Reactor names

Open each board's serial monitor at 115200 baud and note its Chip ID. Edit
`data/reactors.ini` with one `CHIP_ID=reactor_name` entry per board. Names can
contain up to 32 letters, digits, underscores or hyphens.

Upload the same filesystem image to each board with `pio run -e pico -t uploadfs`
or PlatformIO's Upload Filesystem Image task. This replaces the LittleFS image
and can be done independently of the firmware upload. Reboot to apply the map.

At startup, the firmware looks up the board ID in `reactors.ini`. A registered
board uses its reactor name as the final topic segment, such as
`labauto/pp/photoreactor/reactor2`. If the file is unavailable or there is no
valid entry, the board uses its Chip ID as the final segment. The serial monitor
shows the Chip ID and selected topic.

## Control and MQTT

Core 1 owns the sensors, encoder, OLED, and PID. Core 0 owns Wi-Fi and MQTT.
The controller shares the latest telemetry snapshot with the communications
core. Wi-Fi reconnects and broker acknowledgements run on core 0.

Turn the encoder to select `TARGET C` or `EXPERIMENT`. Press on `TARGET C` to
start editing, turn to adjust the blinking setpoint, and press again to apply
it. Press on `EXPERIMENT` to toggle `ON EXP` or `OFF EXP`. This status is sent in
MQTT while the PID continues to regulate the configured temperature.
The firmware starts in `OFF EXP` with a target of 15 °C. The encoder adjusts
the target from 5 °C to 30 °C in steps of 0.5 °C. The startup defaults and
range are configured at the top of `src/main.cpp`.

The Pico publishes one JSON message every five seconds while connected, using
QoS 1 with retain disabled. A reconnect resumes with current readings.

| Field | Value |
| --- | --- |
| `is_running` | Experiment status selected on the OLED, initially `false` |
| `temp_pv` | Measured temperature in ?C, or `null` on sensor error |
| `temp_sv` | Confirmed setpoint in ?C |
| `o2_conc` | O? concentration in percent, or `null` on sensor error |
| `peltier_pwm` | Peltier PWM in percent |

## Sensor diagnostics

The separate `sensor-debug` environment probes the sensors and OLED with motor
output off. Use `pio run -e sensor-debug -t upload` to load it over USB.
