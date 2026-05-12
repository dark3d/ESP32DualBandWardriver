#include "BatteryInterface.h"

BatteryInterface::BatteryInterface() {
  
}

void BatteryInterface::main(uint32_t currentTime) {
  if (currentTime != 0) {
    if (currentTime - initTime >= 3000) {
      this->initTime = millis();

      int8_t new_level = this->getBatteryLevel();
      if (this->battery_level != new_level) {
        Logger::log(STD_MSG, "Battery Level changed: " + (String)new_level);
        this->battery_level = new_level;
        Logger::log(STD_MSG, "Battery Level: " + (String)this->battery_level);
      }
    }
  }
}

// ============================================================
// Chunk 2: USB presence detection via GPIO25 ADC
//
// The XB8608 CHG signal is attenuated through the board's LED
// circuit and appears on GPIO25 at:
//   ~0V    (ADC ~0)    — battery only, no USB
//   ~0.85V (ADC ~1055) — USB power present
//
// A threshold of 500 counts (~0.4V) sits cleanly between both
// states with comfortable margin on either side.
//
// Averages CHG_ADC_SAMPLES reads with a 2ms gap between each
// to filter ADC noise.
// ============================================================

bool BatteryInterface::isCharging() {
  int total = 0;
  for (int i = 0; i < CHG_ADC_SAMPLES; i++) {
    total += analogRead(CHG_PIN);
    delay(2);
  }
  int avg = total / CHG_ADC_SAMPLES;

  Logger::log(STD_MSG, "[CHG] ADC avg: " + String(avg) +
              (avg > CHG_ADC_THRESHOLD ? " -> USB" : " -> BATTERY"));

  return (avg > CHG_ADC_THRESHOLD);
}

// ============================================================

void BatteryInterface::RunSetup() {
  byte error;

  #ifdef HAS_BATTERY

    Wire.begin(I2C_SDA, I2C_SCL);

    Logger::log(STD_MSG, "Checking for battery monitors...");

    Wire.beginTransmission(IP5306_ADDR);
    error = Wire.endTransmission();

    if (error == 0) {
      Logger::log(GUD_MSG, "Detected IP5306");
      this->has_ip5306 = true;
      this->i2c_supported = true;
    }

    Wire.beginTransmission(MAX17048_ADDR);
    error = Wire.endTransmission();

    if (error == 0) {
      if (maxlipo.begin()) {
        Logger::log(GUD_MSG, "Detected MAX17048");
        this->has_max17048 = true;
        this->i2c_supported = true;
      }
    }

    this->initTime = millis();

    // Log initial USB state at boot
    Logger::log(STD_MSG, "[CHG] Initial USB state: " +
                String(this->isCharging() ? "USB" : "BATTERY"));

  #endif
}

int8_t BatteryInterface::getBatteryLevel() {

  if (this->has_ip5306) {
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(0x78);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(IP5306_ADDR, 1)) {
      this->i2c_supported = true;
      switch (Wire.read() & 0xF0) {
        case 0xE0: return 25;
        case 0xC0: return 50;
        case 0x80: return 75;
        case 0x00: return 100;
        default: return 0;
      }
    }
    this->i2c_supported = false;
    return -1;
  }

  if (this->has_max17048) {
    float percent = this->maxlipo.cellPercent();

    if (percent >= 100)
      return 100;
    else if (percent <= 0)
      return 0;
    else
      return percent;
  }

  return 0;
}
