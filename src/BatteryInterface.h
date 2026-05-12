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

class BatteryInterface {
  private:
    uint32_t initTime = 0;
    Adafruit_MAX17048 maxlipo;

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

    // Chunk 2: USB presence detection via GPIO25 ADC.
    // Samples the XB8608 CHG signal attenuated through the board's
    // LED circuit. Returns true when USB power is present.
    bool isCharging();
};

#endif
