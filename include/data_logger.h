#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <math.h>

class DataLogger {
public:
  bool ready = false;
  bool begin() {
    ready = LittleFS.begin();
    if (!ready) return false; // NEVER auto-format customer logs.
    return true;
  }
  void log(uint32_t ms, double measured, double target, double oxygen, double pwm) {
    if (!ready || ms - lastLogMs < 10000) return;
    lastLogMs = ms;
    char buffer[128];
    char t[20], o[20];
    if (isfinite(measured)) snprintf(t, sizeof(t), "%.2f", measured);
    else strcpy(t, "");
    if (isfinite(oxygen)) snprintf(o, sizeof(o), "%.2f", oxygen);
    else strcpy(o, "");
    // Uptime only; no RTC installed. Reboot restarts the millisecond counter.
    snprintf(buffer, sizeof(buffer), "%lu,%s,%.2f,%s,%.1f\n",
             (unsigned long)ms, t, target, o, pwm);
    File f = LittleFS.open("/temperature.csv", "a");
    if (!f) return;
    if (f.size() == 0) f.print("uptime_ms,temperature_c,target_c,oxygen_pct,pwm_pct\n");
    f.print(buffer);
    f.close();
    if (LittleFS.exists("/temperature.csv")) {
      File check = LittleFS.open("/temperature.csv", "r");
      size_t size = check ? check.size() : 0;
      if (check) check.close();
      if (size >= 150000) {
        LittleFS.remove("/temperature_previous.csv");
        LittleFS.rename("/temperature.csv", "/temperature_previous.csv");
      }
    }
  }
private:
  uint32_t lastLogMs = 0;
};
