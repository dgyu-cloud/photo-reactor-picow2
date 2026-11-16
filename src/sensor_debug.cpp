// Standalone hardware diagnostic: pio run -e sensor-debug -t upload
// No PID or encoder interrupts. Motor PWM stays off throughout this sketch.
#include <Arduino.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "oxygen_sensor.h"

namespace {
Adafruit_MAX31855 thermocouple(18, 17, 16); // CLK, CS, DO
Adafruit_SSD1306 display(128, 64, &Wire, -1);
OxygenSensor oxygen;
bool found[128] = {};
bool displayReady = false;
uint32_t lastReport = 0;
bool ledOn = false;

void scanBus() {
  Serial.println("[DBG] I2C scan begin (SDA=GP4, SCL=GP5)");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    found[address] = Wire.endTransmission() == 0;
  }
  Serial.println("[DBG] I2C scan done");
}
} // namespace

void setup() {
  // Disable the connected motor before touching sensors.
  for (uint8_t pin : {13, 14, 15}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  // This diagnostic should also print when a terminal leaves DTR low.
  Serial.ignoreFlowControl(true);
  delay(3000); // Bounded startup window for opening a monitor.
  Serial.println("[DBG] sensor-debug boot; motor OFF");
  Serial.println("[DBG] MAX31855 begin");
  Serial.printf("[DBG] MAX31855 library init=%s (not a connection test)\n",
                thermocouple.begin() ? "ok" : "failed");
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(25);
  scanBus();
  for (uint8_t address : {0x3C, 0x3D}) {
    if (!found[address]) continue;
    Serial.printf("[DBG] OLED begin at 0x%02X\n", address);
    displayReady = display.begin(SSD1306_SWITCHCAPVCC, address, false, false);
    if (displayReady) {
      display.setRotation(2);
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
    }
    break;
  }
  Serial.println("[DBG] setup complete; send s to scan again");
}

void loop() {
  const uint32_t now = millis();
  oxygen.update(now);
  if (Serial.available() && Serial.read() == 's') scanBus();
  if (now - lastReport < 2000) return;
  lastReport = now;
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn);
  Serial.printf("\n[DBG] alive ms=%lu DTR=%d motor=OFF\n",
                static_cast<unsigned long>(now), Serial.dtr());
  Serial.print("[DBG] I2C ACK:");
  bool any = false;
  for (uint8_t address = 1; address < 127; ++address) {
    if (!found[address]) continue;
    Serial.printf(" 0x%02X", address);
    any = true;
  }
  if (!any) Serial.print(" none");
  Serial.println();
  Serial.println("[DBG] MAX31855 read begin (DO=16 CS=17 CLK=18)");
  const double temperature = thermocouple.readCelsius();
  Serial.printf("[DBG] MAX31855 C=%.2f internal=%.2f fault=0x%02X\n",
                temperature, thermocouple.readInternal(), thermocouple.readError());
  Serial.printf("[DBG] O2 addr=0x%02X status=%s percent=%.2f\n",
                oxygen.address, oxygen.status, oxygen.percent);
  Serial.printf("[DBG] OLED init=%s\n", displayReady ? "ok" : "failed/absent");
  if (displayReady) {
    Serial.println("[DBG] OLED draw begin");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SENSOR DEBUG / PWM OFF");
    display.print("Temp C: "); display.println(temperature, 2);
    display.print("O2 %: "); display.println(oxygen.percent, 2);
    display.println(oxygen.status);
    display.display();
    Serial.println("[DBG] OLED draw done");
  }
}
