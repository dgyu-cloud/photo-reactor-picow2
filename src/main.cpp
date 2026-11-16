#include <Arduino.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "temperature_ui.h"
#include "oxygen_sensor.h"
#include "data_logger.h"
#include "web_manager.h"

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
DataLogger logger;
WebManager web(logger);
double lastPowerPercent = 0.0;
char pendingOxygenLine[65] = {};
double currentTemperatureC = NAN;
unsigned long lastDisplayMs = 0;
bool buttonRaw = HIGH;
bool buttonStable = HIGH;
unsigned long buttonChangedMs = 0;

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

// One USB packet at most. Drop telemetry when busy instead of delaying PID.
bool sendLog(const char* line) {
  if (!Serial) {
    return false;
  }
  const size_t length = strlen(line);
  if (length > 64 || Serial.availableForWrite() < static_cast<int>(length)) {
    return false;
  }
  return Serial.write(reinterpret_cast<const uint8_t*>(line), length) == length;
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
  if (!logger.begin()) Serial.println("[LOG] LittleFS mount failed; logs disabled. No auto-format.");
  web.begin();
  lastSampleMs = millis();
}

void loop() {
  const unsigned long now = millis();
  updateControls(now);
  web.poll();
  if (oxygen.update(now)) {
    if (isfinite(oxygen.percent)) {
      snprintf(pendingOxygenLine, sizeof(pendingOxygenLine),
               "O2: %.2f %%\n", oxygen.percent);
    } else {
      snprintf(pendingOxygenLine, sizeof(pendingOxygenLine),
               "O2 ERROR: %s\n", oxygen.status);
    }
  }
  // Keep the latest O2 reading until USB accepts it, including when the
  // monitor connects after startup. Never wait for USB in the PID loop.
  if (pendingOxygenLine[0] && sendLog(pendingOxygenLine)) {
    pendingOxygenLine[0] = '\0';
  }
  const unsigned long elapsedMs = now - lastSampleMs;
  if (elapsedMs < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs = now;

  const double temperatureC = thermocouple.readCelsius();
  currentTemperatureC = temperatureC;
  double powerPercent = 0.0;
  if (!isfinite(temperatureC)) {
    setPower(0.0);
    integralPercent = 0.0;
    hasPreviousTemperature = false;
    const uint8_t fault = thermocouple.readError();

    if (fault & MAX31855_FAULT_OPEN) {
      sendLog("ERROR: thermocouple open\n");
    } else if (fault & MAX31855_FAULT_SHORT_GND) {
      sendLog("ERROR: thermocouple short to GND\n");
    } else if (fault & MAX31855_FAULT_SHORT_VCC) {
      sendLog("ERROR: thermocouple short to VCC\n");
    } else {
      sendLog("ERROR: MAX31855 communication\n");
    }
  } else {
    powerPercent = calculatePid(temperatureC, elapsedMs / 1000.0);
    setPower(powerPercent);
  }

  char line[65];
  // Report the applied target even when the thermocouple has failed.
  const int length = isfinite(temperatureC)
      ? snprintf(line, sizeof(line),
          "Temperature: %.2f C, Target: %.2f C, PWM: %.1f %%\n",
          temperatureC, ui.targetC(), powerPercent)
      : snprintf(line, sizeof(line),
          "Temperature: ERROR, Target: %.2f C, PWM: %.1f %%\n",
          ui.targetC(), powerPercent);
  if (length > 0 && static_cast<size_t>(length) < sizeof(line)) {
    sendLog(line);
  }
  // Noncritical telemetry and storage after the control calculation.
  web.setTelemetry(currentTemperatureC, ui.targetC(), oxygen.percent,
                   lastPowerPercent, oxygen.status);
  logger.log(millis(), currentTemperatureC, ui.targetC(), oxygen.percent, lastPowerPercent);
  // I2C display update after control calculation.
  updateDisplay(millis());
}
