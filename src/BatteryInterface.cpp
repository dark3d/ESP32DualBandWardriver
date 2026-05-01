#include "BatteryInterface.h"
BatteryInterface::BatteryInterface() {
  
}

void BatteryInterface::main(uint32_t currentTime) {
  if (currentTime != 0) {
    if (currentTime - initTime >= 3000) {
      this->initTime = millis();

      // Battery level
      int8_t new_level = this->getBatteryLevel();
      if (this->battery_level != new_level) {
        Logger::log(STD_MSG, "Battery Level changed: " + (String)new_level);
        this->battery_level = new_level;
        Logger::log(STD_MSG, "Battery Level: " + (String)this->battery_level);
      }

      // Chunk 2: USB charging state debounce
      this->updateChargingState();
    }
  }
}

// ============================================================
// Chunk 2: USB / charging detection
// ============================================================

// Single raw read — no debounce, no caching.
// IP5306: register 0x70 bit 3 = USB present/charging.
// MAX17048: positive chargeRate = charging.
// Returns true if USB power is detected.
bool BatteryInterface::readRawCharging() {
  if (this->has_ip5306) {
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(IP5306_REG_STATUS);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(IP5306_ADDR, 1)) {
      uint8_t val = Wire.read();
      return (val & IP5306_CHARGE_BIT) != 0;
    }
    // I2C read failed — assume no change, return current state
    return this->charging_state;
  }

  if (this->has_max17048) {
    // chargeRate() > 0 means cell is gaining charge = USB present
    return this->maxlipo.chargeRate() > 0.0f;
  }

  return false; // no supported IC
}

// Debounce: require USB_DEBOUNCE_READS (3) consecutive identical reads
// before committing a state transition. This filters I2C glitches.
void BatteryInterface::updateChargingState() {
  bool raw = this->readRawCharging();

  if (raw == this->pending_charging_state) {
    this->usb_debounce_count++;
  } else {
    // Reading changed — reset counter and track new candidate
    this->pending_charging_state = raw;
    this->usb_debounce_count = 1;
  }

  if (this->usb_debounce_count >= USB_DEBOUNCE_READS) {
    if (raw != this->charging_state) {
      // State transition confirmed
      this->charging_state = raw;
      if (raw) {
        Logger::log(GUD_MSG, "[BAT] USB power connected — charging");
      } else {
        Logger::log(WARN_MSG, "[BAT] USB power removed — on battery");
      }
    }
    // Reset debounce so we keep checking cleanly
    this->usb_debounce_count = 0;
  }
}

// Public accessor — returns the debounced charging state.
bool BatteryInterface::isCharging() {
  return this->charging_state;
}

// ============================================================

void BatteryInterface::RunSetup() {
  byte error;
  byte addr;

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

    /*for(addr = 1; addr < 127; addr++ ) {
      Wire.beginTransmission(addr);
      error = Wire.endTransmission();

      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        
        if (addr<16)
          Serial.print("0");

        Serial.println(addr,HEX);
        
        if (addr == IP5306_ADDR) {
          this->has_ip5306 = true;
          this->i2c_supported = true;
        }

        if (addr == MAX17048_ADDR) {
          if (maxlipo.begin()) {
            Serial.println("Detected MAX17048");
            this->has_max17048 = true;
            this->i2c_supported = true;
          }
        }
      }
    }*/

    /*if (this->maxlipo.begin()) {
      Serial.println("Detected MAX17048");
      this->has_max17048 = true;
      this->i2c_supported = true;
    }*/
    
    this->initTime = millis();

    // Chunk 2: take an initial charging reading so state is valid before
    // the first main() cycle. No debounce on first read — just seed the state.
    bool initial = this->readRawCharging();
    this->charging_state         = initial;
    this->pending_charging_state = initial;
    this->usb_debounce_count     = 0;
    Logger::log(STD_MSG, "[BAT] Initial USB state: " + String(initial ? "CHARGING" : "BATTERY"));

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

    // Sometimes we dumb
    if (percent >= 100)
      return 100;
    else if (percent <= 0)
      return 0;
    else
      return percent;
  }

  return 0;
}
