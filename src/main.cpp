#include <Arduino.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_SSD1306.h>
#include <MQTT.h>
#include <WiFi.h>
#include <Wire.h>
#include <pico/mutex.h>
#include "temperature_ui.h"
#include "oxygen_sensor.h"

bool core1_separate_stack = true;

// MAX31855 wiring: DO -> GP16, CS -> GP17, CLK -> GP18
constexpr uint8_t MAX31855_DO_PIN = 16;
constexpr uint8_t MAX31855_CS_PIN = 17;
constexpr uint8_t MAX31855_CLK_PIN = 18;
constexpr uint8_t MOTOR_INA_PIN = 13;
constexpr uint8_t MOTOR_INB_PIN = 14;
constexpr uint8_t MOTOR_PWM_PIN = 15;
constexpr unsigned long SAMPLE_INTERVAL_MS = 50;
// Default Wire on Pico 2: SDA GP4, SCL GP5.
constexpr uint8_t ENCODER_CLK_PIN = 6;
constexpr uint8_t ENCODER_DT_PIN = 7;
constexpr uint8_t ENCODER_SW_PIN = 8;
constexpr int ENCODER_DIRECTION = 1; // Set to -1 if clockwise decreases.
constexpr unsigned long DISPLAY_INTERVAL_MS = 100;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 30;

// User settings: temperature in degrees C; PID time units are seconds.
constexpr double PID_KP = 10.0; // Output percent per degree C.
constexpr double PID_KI = 0.3;  // Output percent per (degree C * second).
constexpr double PID_KD = 1;  // Output percent * second per degree C.
constexpr bool COOLING_MODE = true; // true: cooling, false: heating.
static_assert(PID_KP >= 0 && PID_KI >= 0 && PID_KD >= 0,
              "PID gains must be nonnegative");
static_assert(SAMPLE_INTERVAL_MS > 0, "Sample interval must be positive");

Adafruit_MAX31855 thermocouple(MAX31855_CLK_PIN, MAX31855_CS_PIN,
                               MAX31855_DO_PIN);

unsigned long lastSampleMs = 0;
double integralPercent = 0.0;
double previousTemperatureC = 0.0;
bool hasPreviousTemperature = false;
TemperatureUi ui;
RotaryDecoder rotary;
volatile int pendingEncoderSteps = 0;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool displayReady = false;
OxygenSensor oxygen;
double lastPowerPercent = 0.0;
double currentTemperatureC = NAN;
unsigned long lastDisplayMs = 0;
bool buttonRaw = HIGH;
bool buttonStable = HIGH;
unsigned long buttonChangedMs = 0;

namespace {
// Set these to the WPA2 Wi-Fi network that can reach the MQTT broker.
constexpr char STATION_SSID[] = "VORONOI_7F";
constexpr char STATION_PASSWORD[] = "qhfhshdl0!";
constexpr bool MQTT_ENABLED = STATION_SSID[0] && STATION_PASSWORD[0];
const IPAddress MQTT_BROKER(192, 168, 10, 2);
constexpr uint16_t MQTT_PORT = 1883;
constexpr char MQTT_TOPIC[] = "labauto/pp/photoreactor/t1";
constexpr unsigned long MQTT_PUBLISH_INTERVAL_MS = 5000;
constexpr unsigned long MQTT_RETRY_INTERVAL_MS = 10000;

struct Telemetry {
  double temperatureC;
  double targetC;
  double oxygenPercent;
  double pwmPercent;
};

// Core 1 updates this snapshot. Core 0 copies it before any network operation.
auto_init_mutex(telemetryMutex);
Telemetry latestTelemetry{};
bool hasTelemetry = false;

WiFiClient transport;
MQTTClient mqtt(256);
char clientId[24];
unsigned long lastPublishMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastBrokerAttemptMs = 0;
bool hadWifi = false;

void formatSensorValue(char (&value)[16], double reading) {
  if (isfinite(reading)) {
    const int length = snprintf(value, sizeof(value), "%.2f", reading);
    if (length > 0 && static_cast<size_t>(length) < sizeof(value)) return;
  }
  strcpy(value, "null");
}

void publishTelemetry(const Telemetry &sample) {
  char temperature[16];
  char oxygenValue[16];
  formatSensorValue(temperature, sample.temperatureC);
  formatSensorValue(oxygenValue, sample.oxygenPercent);

  char payload[160];
  const int length = snprintf(payload, sizeof(payload),
      "{\"is_running\":true,\"temp_pv\":%s,\"temp_sv\":%.2f,"
      "\"o2_conc\":%s,\"peltier_pwm\":%.2f}",
      temperature, sample.targetC, oxygenValue, sample.pwmPercent);
  if (length <= 0 || static_cast<size_t>(length) >= sizeof(payload)) {
    if (Serial) Serial.println("[MQTT] Publish failed: JSON formatting error");
    return;
  }

  const bool acknowledged = mqtt.publish(MQTT_TOPIC, payload, length, false, 1);
  if (!Serial) return;
  if (acknowledged) {
    Serial.printf("[MQTT] PUBACK %s %s\n", MQTT_TOPIC, payload);
  } else {
    Serial.printf("[MQTT] Publish failed on %s (error %d)\n", MQTT_TOPIC, mqtt.lastError());
  }
}

// Wi-Fi joins, TCP connections, and QoS 1 acknowledgements wait on core 0.
void pollMqtt() {
  const unsigned long now = millis();
  if (!WiFi.connected()) {
    if (hadWifi) {
      transport.stop();
      hadWifi = false;
      if (Serial) Serial.println("[MQTT] Wi-Fi disconnected");
    }
    if (now - lastWifiAttemptMs >= MQTT_RETRY_INTERVAL_MS) {
      Serial.println("[MQTT] Connecting to Wi-Fi");
      WiFi.begin(STATION_SSID, STATION_PASSWORD);
      lastWifiAttemptMs = millis();
    }
    return;
  }

  if (!hadWifi) {
    hadWifi = true;
    lastBrokerAttemptMs = now - MQTT_RETRY_INTERVAL_MS;
    Serial.println("[MQTT] Wi-Fi connected");
  }
  if (!mqtt.connected()) {
    if (now - lastBrokerAttemptMs < MQTT_RETRY_INTERVAL_MS) return;
    const bool connected = mqtt.connect(clientId);
    lastBrokerAttemptMs = millis();
    if (!connected) {
      Serial.printf("[MQTT] Broker connection failed (%d)\n", mqtt.lastError());
      return;
    }
    Serial.println("[MQTT] Broker connected");
  }
  if (!mqtt.loop()) {
    if (Serial) Serial.printf("[MQTT] Connection lost (error %d)\n", mqtt.lastError());
    return;
  }
  if (millis() - lastPublishMs < MQTT_PUBLISH_INTERVAL_MS) return;

  if (!mutex_try_enter(&telemetryMutex, nullptr)) return;
  const Telemetry sample = latestTelemetry;
  const bool ready = hasTelemetry;
  mutex_exit(&telemetryMutex);
  if (!ready) return;

  lastPublishMs = millis();
  publishTelemetry(sample);
}
}  // namespace

void encoderChanged() {
  const uint8_t state = (digitalRead(ENCODER_CLK_PIN) << 1) |
                        digitalRead(ENCODER_DT_PIN);
  const int step = rotary.update(state);
  // Bound the interrupt mailbox if the foreground is temporarily busy.
  if (step > 0 && pendingEncoderSteps < 1000) ++pendingEncoderSteps;
  if (step < 0 && pendingEncoderSteps > -1000) --pendingEncoderSteps;
}

void updateControls(unsigned long now) {
  noInterrupts();
  const int steps = pendingEncoderSteps;
  pendingEncoderSteps = 0;
  interrupts();
  ui.turn(steps * ENCODER_DIRECTION, now);

  const bool raw = digitalRead(ENCODER_SW_PIN);
  if (raw != buttonRaw) {
    buttonRaw = raw;
    buttonChangedMs = now;
  }
  if (raw != buttonStable && now - buttonChangedMs >= BUTTON_DEBOUNCE_MS) {
    buttonStable = raw;
    if (buttonStable == LOW && ui.confirm()) {
      // Discard integral effort accumulated for the old target.
      integralPercent = 0.0;
    }
  }
}

void updateDisplay(unsigned long now) {
  if (!displayReady || now - lastDisplayMs < DISPLAY_INTERVAL_MS) return;
  lastDisplayMs = now;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  // Two columns: temperature on the left, oxygen on the right.
  display.drawFastVLine(64, 0, 64, SSD1306_WHITE);
  const auto printTemperature = [](double temperatureC, int16_t y) {
    char value[16];
    if (isfinite(temperatureC)) {
      snprintf(value, sizeof(value), "%.1f", temperatureC);
    } else {
      strcpy(value, "ERROR");
    }
    // Keep unusually large or negative readings inside the left column.
    display.setTextSize(strlen(value) <= 5 ? 2 : 1);
    display.setCursor(0, y);
    display.print(value);
  };
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("CURRENT C");
  printTemperature(currentTemperatureC, 10);
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("TARGET C");
  if (ui.targetVisible(now)) {
    printTemperature(ui.editing ? ui.draftHalfC * 0.5 : ui.targetC(), 40);
  }
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(ui.editing ? "PRESS: SET" : "TURN: EDIT");

  display.setCursor(68, 0);
  display.print("O2 (%)");
  display.setTextSize(2);
  display.setCursor(68, 10);
  if (isfinite(oxygen.percent)) {
    display.print(oxygen.percent, 2);
  } else {
    display.print("ERROR");
  }
  display.display();
}

void setupDisplay() {
  Wire.begin();
  // SSD1306 begin() does not test for a connected panel. Probe first.
  for (const uint8_t address : {0x3C, 0x3D}) {
    Wire.beginTransmission(address);
    // Mbed Wire probes an empty write using a read, unsupported by some
    // OLEDs. Send a valid display-off command to check the write ACK instead.
    Wire.write(0x00);
    Wire.write(SSD1306_DISPLAYOFF);
    if (Wire.endTransmission() == 0) {
      displayReady = display.begin(SSD1306_SWITCHCAPVCC, address, false, false);
      break;
    }
  }
  if (displayReady) {
    display.setRotation(0); // Rotate 180 degrees for the mounted panel.
    display.setTextWrap(false);
  }
}

double clampPower(double percent) {
  return constrain(percent, 0.0, 100.0);
}

void setPower(double percent) {
  const int pwmValue = static_cast<int>(clampPower(percent) * 255.0 / 100.0 + 0.5);
  analogWrite(MOTOR_PWM_PIN, pwmValue);
  lastPowerPercent = clampPower(percent);
}

double calculatePid(double temperatureC, double dtSeconds) {
  const double direction = COOLING_MODE ? 1.0 : -1.0;
  const double error = direction * (temperatureC - ui.targetC());
  const double proportional = PID_KP * error;
  // Derivative on measurement avoids a kick when the target changes.
  const double derivative = hasPreviousTemperature
      ? direction * PID_KD * (temperatureC - previousTemperatureC) / dtSeconds
      : 0.0;
  const double integralChange = PID_KI * error * dtSeconds;
  const double candidateIntegral = integralPercent + integralChange;
  const double candidateOutput = proportional + candidateIntegral + derivative;
  // Stop integrating into saturation, but allow the integral to unwind.
  if ((candidateOutput >= 0.0 && candidateOutput <= 100.0) ||
      (candidateOutput > 100.0 && integralChange < 0.0) ||
      (candidateOutput < 0.0 && integralChange > 0.0)) {
    integralPercent = candidateIntegral;
  }
  previousTemperatureC = temperatureC;
  hasPreviousTemperature = true;
  return clampPower(proportional + integralPercent + derivative);
}

void setup() {
  Serial.begin(115200);
  if (!MQTT_ENABLED) return;

  WiFi.mode(WIFI_STA);
  WiFi.setTimeout(5000);
  mqtt.begin(MQTT_BROKER, MQTT_PORT, transport);
  mqtt.setTimeout(1000);
  snprintf(clientId, sizeof(clientId), "pp-%s", rp2040.getChipID());
  lastPublishMs = millis();
  lastWifiAttemptMs = lastPublishMs - MQTT_RETRY_INTERVAL_MS;
}

void loop() {
  if (MQTT_ENABLED) {
    pollMqtt();
  } else {
    static bool monitorWasOpen = false;
    const bool monitorOpen = static_cast<bool>(Serial);
    if (monitorOpen && !monitorWasOpen) {
      Serial.println("[MQTT] Configure STATION_SSID and STATION_PASSWORD to enable telemetry");
    }
    monitorWasOpen = monitorOpen;
  }
  delay(20);
}

void setup1() {
  thermocouple.begin();

  pinMode(MOTOR_INA_PIN, OUTPUT);
  pinMode(MOTOR_INB_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  digitalWrite(MOTOR_INA_PIN, HIGH);
  digitalWrite(MOTOR_INB_PIN, LOW);

  analogWriteResolution(8);
  setPower(0.0);
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  rotary.previous = (digitalRead(ENCODER_CLK_PIN) << 1) |
                    digitalRead(ENCODER_DT_PIN);
  buttonRaw = buttonStable = digitalRead(ENCODER_SW_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), encoderChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT_PIN), encoderChanged, CHANGE);
  setupDisplay();
  lastSampleMs = millis();
}

void loop1() {
  const unsigned long now = millis();
  updateControls(now);
  oxygen.update(now);
  const unsigned long elapsedMs = now - lastSampleMs;
  if (elapsedMs < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs = now;

  const double temperatureC = thermocouple.readCelsius();
  currentTemperatureC = temperatureC;
  if (!isfinite(temperatureC)) {
    setPower(0.0);
    integralPercent = 0.0;
    hasPreviousTemperature = false;
  } else {
    setPower(calculatePid(temperatureC, elapsedMs / 1000.0));
  }

  if (MQTT_ENABLED && mutex_try_enter(&telemetryMutex, nullptr)) {
    latestTelemetry = {currentTemperatureC, ui.targetC(), oxygen.percent, lastPowerPercent};
    hasTelemetry = true;
    mutex_exit(&telemetryMutex);
  }
  // I2C display update after control calculation.
  updateDisplay(millis());
}
