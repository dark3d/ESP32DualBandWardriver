#pragma once

#ifndef BatteryInterface_h
#define BatteryInterface_h

#include <Arduino.h>

#include "configs.h"
#include "Adafruit_MAX1704X.h"
#include "utils.h"
#include "logger.h"

#include <Wire.h>

#define IP5306_ADDR    0x75
#define MAX17048_ADDR  0x36

// IP5306 register used for charging status detection
// Register 0x70, bit 3: 1 = USB power present / charging, 0 = on battery
#define IP5306_REG_STATUS 0x70
#define IP5306_CHARGE_BIT 0x08

class BatteryInterface {
  private:
    uint32_t initTime = 0;
    Adafruit_MAX17048 maxlipo;

    // --------------------------------------------------------
    // Chunk 2: USB/charging detection state
    // --------------------------------------------------------
    bool    charging_state         = false; // debounced current state
    bool    pending_charging_state = false; // candidate new state
    uint8_t usb_debounce_count     = 0;     // consecutive reads matching pending

    bool    readRawCharging();        // single raw read from IC
    void    updateChargingState();    // debounce logic, called from main()

  public:
    int8_t battery_level = 0;
    int8_t old_level     = 0;
    bool   i2c_supported = false;
    bool   has_max17048  = false;
    bool   has_ip5306    = false;

    BatteryInterface();

    void   RunSetup();
    void   main(uint32_t currentTime);
    int8_t getBatteryLevel();

    // Chunk 2: returns debounced USB charging state
    // true  = USB power present (charging or charge-complete)
    // false = running on battery only
    bool isCharging();
};

#endif
