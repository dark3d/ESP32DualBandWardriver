#include "configs.h"
#include "BatteryInterface.h"
#include "Buffer.h"
#include "settings.h"
#include "display.h"
#include "GpsInterface.h"
#include "SDInterface.h"
#include "Switches.h"
#include "WiFiOps.h"
#include "utils.h"
#include "ui.h"
#include "logger.h"

SET_LOOP_TASK_STACK_SIZE(12 * 1024);

Buffer buffer;
Settings settings;
GpsInterface gps;
BatteryInterface battery;
WiFiOps wifi_ops;
Utils utils;
UI ui_obj;
bool g_force_display_redraw = false;

SPIClass sharedSPI(SPI);
Display display = Display(&sharedSPI, TFT_CS, TFT_DC, TFT_RST);
SDInterface sd_obj = SDInterface(&sharedSPI, SD_CS);

Switches u_btn = Switches(U_BTN, 1000, U_PULL);
Switches d_btn = Switches(D_BTN, 1000, D_PULL);
Switches c_btn = Switches(C_BTN, 1000, C_PULL);

// Boot progress bar along the bottom of the splash.
static void bootBar(uint8_t pct) {
  if (pct > 100) pct = 100;
  int w = TFT_WIDTH - 8;
  display.tft->drawRect(4, 74, w, 5, ST77XX_WHITE);
  display.tft->fillRect(5, 75, (w - 2) * pct / 100, 3, ST77XX_CYAN);
}

void setup() {
  Serial.begin(115200);

  while (!Serial)
    delay(10);

  Logger::log(STD_MSG, "[HEAP] setup-top free=" + String(ESP.getFreeHeap()) +
              " maxAlloc=" + String(ESP.getMaxAllocHeap()));

  // Do SPI stuff first
  sharedSPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Give SPI some time I guess
  delay(100);

  // Init the display before SD
  display.begin();

  // Give SD some time
  delay(100);

  // Show us IDF information
  Logger::log(STD_MSG, "ESP-IDF version is: " + String(esp_get_idf_version()));

  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  // Load settings
  settings.begin();

  if (settings.getSettingType(SETTING_SANITY) == "") {
    Logger::log(WARN_MSG, "Current settings format not supported. Installing new default settings...");
    settings.createDefaultSettings(SPIFFS);
  }
  else {
    Logger::log(GUD_MSG, "Current settings format supported");
  }

  // Init our buffer for writing logs
  buffer = Buffer();

  // Init SD Card
  if(!sd_obj.initSD())
    Logger::log(WARN_MSG, "SD Card NOT Supported");

  // Check for firmware updates now
  Logger::log(STD_MSG, "Checking for firmware updates...");
  sd_obj.runUpdate();

  // Enable SD debug logging if setting is on
  Logger::enableSDLog(settings.loadSetting<bool>(DEBUG_LOG_NAME));
  bootBar(35);

  // Move any legacy root upload markers into /sc/ before anything checks them
  wifi_ops.migrateSidecars();

  // Capture boot-time button holds once (justPressed is edge-triggered).
  bool sel_held = c_btn.justPressed();
  int mode_override = 0;
  if (d_btn.justPressed()) mode_override = CORE_MODE;
  else if (u_btn.justPressed()) mode_override = SOLO_MODE;

  // Minimal-heap dock upload BEFORE the memory-heavy subsystems (GPS/UI/ADS-B/BLE)
  // init, so mbedTLS gets a clean contiguous heap. Reboots into normal mode when
  // done. Skipped when SELECT is held ("resume wardriving").
  if (!sel_held)
    wifi_ops.tryBootDockUpload();

  // Prune old wardrive logs before the UI opens each remaining log for its size
  wifi_ops.pruneOldLogs();

  // Init battery
  battery.RunSetup();
  battery.battery_level = battery.getBatteryLevel();
  bootBar(55);

  // Init GPS
  gps.begin();
  bootBar(75);

  // Init wifi and bluetooth
  wifi_ops.begin(sel_held || mode_override != 0, mode_override);

  // Init UI
  ui_obj.begin();

  settings.printJsonSettings(settings.getSettingsString());

  Logger::log(GUD_MSG, "Initialization complete!");
  Logger::log(STD_MSG, "[HEAP] boot free=" + String(ESP.getFreeHeap()) +
              " maxAlloc=" + String(ESP.getMaxAllocHeap()));
}

void loop() {
  // Diagnostic: send 'b' over serial to toggle a simulated GPS outage (buffering
  // test without unplugging the antenna). Sending 'g' forces it off.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b') wifi_ops.toggleSimNoFix();
  }

  // Take current time of this loop for functions
  uint32_t currentTime = millis();

  // Refresh all functions
  wifi_ops.main(currentTime, ui_obj.stat_display_mode == SD_FILES);
  settings.main(currentTime);
  battery.main(currentTime);
  gps.main();
  sd_obj.main();
  buffer.save();
  ui_obj.main(currentTime);

  // Solo or Core modes
  if (((gps.getFixStatus()) || wifi_ops.isGpsBufferingEnabled()) && (sd_obj.supported) && (ui_obj.stat_display_mode != SD_FILES))
    wifi_ops.setCurrentScanMode(WIFI_WARDRIVING);
  // Nodes
  else if ((wifi_ops.run_mode == NODE_MODE) && (wifi_ops.getNodeReady())) {
    wifi_ops.setCurrentScanMode(WIFI_WARDRIVING);
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    wifi_ops.setCurrentScanMode(WIFI_STANDBY);
    if (wifi_ops.run_mode == NODE_MODE)
      digitalWrite(LED_PIN, LOW);
  }
}
