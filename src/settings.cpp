#include "settings.h"

volatile bool Settings::write_in_flight = false;

String Settings::getSettingsString() {
  return this->json_settings_string;
}

bool Settings::isWriteInFlight() {
  return write_in_flight;
}

void Settings::safeRestart() {
  uint32_t start = millis();
  while (write_in_flight && (millis() - start) < 1000) {
    delay(5);
  }
  ESP.restart();
}

bool Settings::readSettingsFile(const char *path, String &out) {
  if (!SPIFFS.exists(path))
    return false;
  File f = SPIFFS.open(path, FILE_READ);
  if (!f)
    return false;
  out = f.readString();
  f.close();
  return true;
}

bool Settings::settingsBlobValid(const String &blob) {
  if (blob.length() < 20)
    return false;
  DynamicJsonDocument json(SETTINGS_JSON_SIZE);
  if (deserializeJson(json, blob))
    return false;
  JsonArrayConst arr = json["Settings"];
  if (arr.isNull() || arr.size() < 4)
    return false;
  return true;
}

bool Settings::writeSettingsBlob(const String &blob) {
  if (!settingsBlobValid(blob)) {
    Logger::log(WARN_MSG, "[SETTINGS] Refusing to persist invalid blob");
    return false;
  }

  write_in_flight = true;

  SPIFFS.remove(SETTINGS_TMP_FILE);
  File tmp = SPIFFS.open(SETTINGS_TMP_FILE, FILE_WRITE);
  if (!tmp) {
    write_in_flight = false;
    Logger::log(WARN_MSG, "[SETTINGS] Could not open temp file");
    return false;
  }
  size_t written = tmp.print(blob);
  tmp.flush();
  tmp.close();

  String check;
  if (written != blob.length() ||
      !readSettingsFile(SETTINGS_TMP_FILE, check) || check != blob) {
    SPIFFS.remove(SETTINGS_TMP_FILE);
    write_in_flight = false;
    Logger::log(WARN_MSG, "[SETTINGS] Temp file verify failed");
    return false;
  }

  String cur;
  if (readSettingsFile(WIFI_CONFIG, cur) && settingsBlobValid(cur)) {
    SPIFFS.remove(SETTINGS_BAK_FILE);
    File bak = SPIFFS.open(SETTINGS_BAK_FILE, FILE_WRITE);
    if (bak) {
      bak.print(cur);
      bak.flush();
      bak.close();
    }
  }

  SPIFFS.remove(WIFI_CONFIG);
  bool ok = SPIFFS.rename(SETTINGS_TMP_FILE, WIFI_CONFIG);
  if (!ok) {
    File p = SPIFFS.open(WIFI_CONFIG, FILE_WRITE);
    if (p) {
      size_t w = p.print(blob);
      p.flush();
      p.close();
      ok = (w == blob.length());
    }
    if (ok)
      SPIFFS.remove(SETTINGS_TMP_FILE);
  }

  if (ok)
    this->json_settings_string = blob;
  write_in_flight = false;
  return ok;
}

bool Settings::begin() {
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    Logger::log(WARN_MSG, "Settings SPIFFS Mount Failed");
    return false;
  }

  String blob;

  if (readSettingsFile(WIFI_CONFIG, blob) && settingsBlobValid(blob)) {
    this->json_settings_string = blob;

    String bak;
    if (!(readSettingsFile(SETTINGS_BAK_FILE, bak) && settingsBlobValid(bak))) {
      SPIFFS.remove(SETTINGS_BAK_FILE);
      File b = SPIFFS.open(SETTINGS_BAK_FILE, FILE_WRITE);
      if (b) {
        b.print(blob);
        b.flush();
        b.close();
      }
    }
    return true;
  }

  if ((readSettingsFile(SETTINGS_TMP_FILE, blob) && settingsBlobValid(blob)) ||
      (readSettingsFile(SETTINGS_BAK_FILE, blob) && settingsBlobValid(blob))) {
    Logger::log(WARN_MSG, "[SETTINGS] Primary invalid — recovering from backup");
    this->json_settings_string = blob;
    this->writeSettingsBlob(blob);
    return true;
  }

  Logger::log(WARN_MSG, "[SETTINGS] No valid settings found — creating defaults");
  if (this->createDefaultSettings(SPIFFS))
    return true;
  return false;
}

void Settings::wipeSPIFFS() {
  Logger::log(WARN_MSG, "[SPIFFS] Wiping filesystem...");

  File root = SPIFFS.open("/");
  if (!root || !root.isDirectory()) {
    Logger::log(WARN_MSG, "[SPIFFS] Failed to open root");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    String path = String(file.name());

    Logger::log(WARN_MSG, "[SPIFFS] Deleting: " + (String)path);

    file.close();  // MUST close before removing

    if (SPIFFS.remove("/" + path)) {
      Logger::log(WARN_MSG, "  -> Deleted");
    } else {
      Logger::log(WARN_MSG, "  -> Delete failed");
    }

    file = root.openNextFile();
  }

  Logger::log(WARN_MSG, "[SPIFFS] Wipe complete.");
}

template <typename T>
T Settings::loadSettingMin(String name) {}

template<>
int Settings::loadSettingMin<int>(String name) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json: ");
    Serial.println(error.f_str());
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == name)
      return json["Settings"][i]["range"]["min"];
  }

  return 0;
}

template <typename T>
T Settings::loadSettingMax(String name) {}

template<>
int Settings::loadSettingMax<int>(String name) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json: ");
    Serial.println(error.f_str());
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == name)
      return json["Settings"][i]["range"]["max"];
  }

  return 0;
}

template <typename T>
T Settings::loadSetting(String key) {}

// Get type int settings
template<>
int Settings::loadSetting<int>(String key) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json: ");
    Serial.println(error.f_str());
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key)
      return json["Settings"][i]["value"];
  }

  Logger::log(WARN_MSG, "Did not find setting named " + (String)key + ". Creating...");
  if (this->createDefaultSettings(SPIFFS, true, json["Settings"].size(), "Int", key))
    return 1;

  return 0;
}

// Get type string settings
template<>
String Settings::loadSetting<String>(String key) {
  //return this->json_settings_string;
  
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Serial.println("\nCould not parse json: ");
    Serial.println(error.f_str());
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key)
      return json["Settings"][i]["value"];
  }

  Logger::log(WARN_MSG, "Did not find setting named " + (String)key + ". Creating...");
  if (this->createDefaultSettings(SPIFFS, true, json["Settings"].size(), "String", key))
    return "";

  return "";
}

// Get type bool settings
template<>
bool Settings::loadSetting<bool>(String key) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  if (deserializeJson(json, this->json_settings_string)) {
    Logger::log(WARN_MSG, "Could not parse json to load");
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key)
      return json["Settings"][i]["value"];
  }

  Logger::log(WARN_MSG, "Did not find setting named " + (String)key + ". Creating...");
  if (this->createDefaultSettings(SPIFFS, true, json["Settings"].size(), "bool", key))
    return true;

  return false;
}

//Get type uint8_t settings
template<>
uint8_t Settings::loadSetting<uint8_t>(String key) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json: ");
    Serial.println(error.f_str());
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key)
      return json["Settings"][i]["value"];
  }

  return 0;
}

template <typename T>
T Settings::saveSetting(String key, bool value) {}

template<>
bool Settings::saveSetting<bool>(String key, bool value) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  if (deserializeJson(json, this->json_settings_string)) {
    Logger::log(WARN_MSG, "Could not parse json to save");
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key) {
      json["Settings"][i]["value"] = value;

      String settings_string;
      serializeJson(json, settings_string);
      return this->writeSettingsBlob(settings_string);
    }
  }

  Logger::log(WARN_MSG, "Did not find setting named " + (String)key + ". Creating...");
  if (this->createDefaultSettings(SPIFFS, true, json["Settings"].size(), "bool", key))
    return true;

  return false;
}

template <typename T>
T Settings::saveSetting(String key, int value, bool is_int) {}

template<>
bool Settings::saveSetting<bool>(String key, int value, bool is_int) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  if (deserializeJson(json, this->json_settings_string)) {
    Logger::log(WARN_MSG, "Could not parse json to save");
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key) {
      json["Settings"][i]["value"] = value;

      String settings_string;
      serializeJson(json, settings_string);
      return this->writeSettingsBlob(settings_string);
    }
  }

  Logger::log(WARN_MSG, "Did not find setting named " + (String)key + ". Creating...");
  if (this->createDefaultSettings(SPIFFS, true, json["Settings"].size(), "bool", key))
    return true;

  return false;
}

template <typename T>
T Settings::saveSetting(String key, String value) {}

template<>
bool Settings::saveSetting<bool>(String key, String value) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  if (deserializeJson(json, this->json_settings_string)) {
    Logger::log(WARN_MSG, "Could not parse json to save");
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key) {
      json["Settings"][i]["value"] = value;

      String settings_string;
      serializeJson(json, settings_string);
      return this->writeSettingsBlob(settings_string);
    }
  }

  Logger::log(WARN_MSG, "Did not find setting named " + (String)key + ". Creating...");
  if (this->createDefaultSettings(SPIFFS, true, json["Settings"].size(), "bool", key))
    return true;

  return false;
}

bool Settings::toggleSetting(String key) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json: ");
    Serial.println(error.f_str());
  }

  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key) {
      if (json["Settings"][i]["value"]) {
        saveSetting<bool>(key, false);
        Logger::log(STD_MSG, "Setting value to false");
        return false;
      }
      else {
        saveSetting<bool>(key, true);
        Logger::log(STD_MSG, "Setting value to true");
        return true;
      }

      return false;
    }
  }
}

String Settings::setting_index_to_name(int i) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json: ");
    Serial.println(error.f_str());
  }

  return json["Settings"][i]["name"];
}

int Settings::getNumberSettings() {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json: ");
    Serial.println(error.f_str());
  }

  return json["Settings"].size();
}

String Settings::getSettingType(String key) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, this->json_settings_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json: ");
    Serial.println(error.f_str());
  }
  
  for (int i = 0; i < json["Settings"].size(); i++) {
    if (json["Settings"][i]["name"].as<String>() == key)
      return json["Settings"][i]["type"];
  }

  return "";
}

void Settings::printJsonSettings(String json_string) {
  DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

  DeserializationError error = deserializeJson(json, json_string);

  if (error) {
    Logger::log(WARN_MSG, "Could not parse json to print: ");
    Serial.println(error.f_str());
  }

  Logger::log(WARN_MSG, "DynamicJsonDocument Capacity: " + (String)json.capacity() + " vs json_string Len: " + (String)json_string.length());
  
  Serial.println("Settings\n----------------------------------------------");
  for (int i = 0; i < json["Settings"].size(); i++) {
    Serial.println("Name: " + json["Settings"][i]["name"].as<String>());
    Serial.println("Type: " + json["Settings"][i]["type"].as<String>());
    Serial.println("Value: " + json["Settings"][i]["value"].as<String>());
    Serial.println("----------------------------------------------");
  }
}

bool Settings::createDefaultSettings(fs::FS &fs, bool spec, uint8_t index, String typeStr, String name) {
  (void)fs;
  Logger::log(STD_MSG, "Creating default settings file: settings.json");

  if (!spec)
    this->wipeSPIFFS();

  String settings_string;

  if (!spec) {

    DynamicJsonDocument jsonBuffer(SETTINGS_JSON_SIZE);

    jsonBuffer["Settings"][0]["name"] = "SavePCAP";
    jsonBuffer["Settings"][0]["type"] = "bool";
    jsonBuffer["Settings"][0]["value"] = true;
    jsonBuffer["Settings"][0]["range"]["min"] = false;
    jsonBuffer["Settings"][0]["range"]["max"] = true;

    jsonBuffer["Settings"][1]["name"] = "UpdateFile";
    jsonBuffer["Settings"][1]["type"] = "String";
    jsonBuffer["Settings"][1]["value"] = "";
    jsonBuffer["Settings"][1]["range"]["min"] = "";
    jsonBuffer["Settings"][1]["range"]["max"] = "";

    jsonBuffer["Settings"][2]["name"] = "e";
    jsonBuffer["Settings"][2]["type"] = "bool";
    jsonBuffer["Settings"][2]["value"] = false;
    jsonBuffer["Settings"][2]["range"]["min"] = false;
    jsonBuffer["Settings"][2]["range"]["max"] = true;

    jsonBuffer["Settings"][3]["name"] = "m";
    jsonBuffer["Settings"][3]["type"] = "Int";
    jsonBuffer["Settings"][3]["value"] = 1;
    jsonBuffer["Settings"][3]["range"]["min"] = 1;
    jsonBuffer["Settings"][3]["range"]["max"] = 10;

    // --------------------------------------------------------
    // Chunk 1: New settings entries (9-27)
    // --------------------------------------------------------

    // [9] WDG Wars API key
    jsonBuffer["Settings"][9]["name"] = WDG_KEY_NAME;
    jsonBuffer["Settings"][9]["type"] = "String";
    jsonBuffer["Settings"][9]["value"] = "";
    jsonBuffer["Settings"][9]["range"]["min"] = "";
    jsonBuffer["Settings"][9]["range"]["max"] = "";

    // [10] Dock trigger SSID
    jsonBuffer["Settings"][10]["name"] = TRIGGER_SSID_NAME;
    jsonBuffer["Settings"][10]["type"] = "String";
    jsonBuffer["Settings"][10]["value"] = "";
    jsonBuffer["Settings"][10]["range"]["min"] = "";
    jsonBuffer["Settings"][10]["range"]["max"] = "";

    // [11] Dock trigger SSID password
    jsonBuffer["Settings"][11]["name"] = TRIGGER_PASS_NAME;
    jsonBuffer["Settings"][11]["type"] = "String";
    jsonBuffer["Settings"][11]["value"] = "";
    jsonBuffer["Settings"][11]["range"]["min"] = "";
    jsonBuffer["Settings"][11]["range"]["max"] = "";

    // [12] Admin password for Basic Auth (empty = no auth)
    jsonBuffer["Settings"][12]["name"] = ADMIN_PASS_NAME;
    jsonBuffer["Settings"][12]["type"] = "String";
    jsonBuffer["Settings"][12]["value"] = "";
    jsonBuffer["Settings"][12]["range"]["min"] = "";
    jsonBuffer["Settings"][12]["range"]["max"] = "";

    // [13-22] SSID exclusion list (sx_0 through sx_9)
    for (int i = 0; i < MAX_SSID_EXCLUSIONS; i++) {
      String key = "sx_" + String(i);
      jsonBuffer["Settings"][13 + i]["name"] = key;
      jsonBuffer["Settings"][13 + i]["type"] = "String";
      jsonBuffer["Settings"][13 + i]["value"] = "";
      jsonBuffer["Settings"][13 + i]["range"]["min"] = "";
      jsonBuffer["Settings"][13 + i]["range"]["max"] = "";
    }

    // [23-27] Geofences (geo_0 through geo_4)
    // Value format: {"lat":0.000000,"lon":0.000000,"rad":0,"label":""}
    for (int i = 0; i < MAX_GEOFENCES; i++) {
      String key = "geo_" + String(i);
      jsonBuffer["Settings"][23 + i]["name"] = key;
      jsonBuffer["Settings"][23 + i]["type"] = "String";
      jsonBuffer["Settings"][23 + i]["value"] = "{\"lat\":0.000000,\"lon\":0.000000,\"rad\":0,\"label\":\"\"}";
      jsonBuffer["Settings"][23 + i]["range"]["min"] = "";
      jsonBuffer["Settings"][23 + i]["range"]["max"] = "";
    }

    // [28] SD debug log enabled (default: false)
    jsonBuffer["Settings"][28]["name"] = DEBUG_LOG_NAME;
    jsonBuffer["Settings"][28]["type"] = "bool";
    jsonBuffer["Settings"][28]["value"] = false;
    jsonBuffer["Settings"][28]["range"]["min"] = false;
    jsonBuffer["Settings"][28]["range"]["max"] = true;

    // [29] Keep newest N wardrive logs (prune older synced logs)
    jsonBuffer["Settings"][29]["name"] = LOG_KEEP_NAME;
    jsonBuffer["Settings"][29]["type"] = "Int";
    jsonBuffer["Settings"][29]["value"] = LOG_KEEP_DEFAULT;
    jsonBuffer["Settings"][29]["range"]["min"] = LOG_KEEP_MIN;
    jsonBuffer["Settings"][29]["range"]["max"] = LOG_KEEP_MAX;

    if (serializeJson(jsonBuffer, settings_string) == 0) {
      Logger::log(WARN_MSG, "Failed to write to string");
    }
  }

  else {
    DynamicJsonDocument json(SETTINGS_JSON_SIZE); // ArduinoJson v6

    if (deserializeJson(json, this->json_settings_string)) {
      Logger::log(WARN_MSG, "Could not parse json to create new setting");
      return false;
    }

    if (typeStr == "bool") {
      Logger::log(WARN_MSG, "Creating bool setting...");
      json["Settings"][index]["name"] = name;
      json["Settings"][index]["type"] = typeStr;
      json["Settings"][index]["value"] = false;
      json["Settings"][index]["range"]["min"] = false;
      json["Settings"][index]["range"]["max"] = true;

      if (serializeJson(json, settings_string) == 0) {
        Logger::log(WARN_MSG, "Failed to write to string");
      }
    }
    else if (typeStr == "String") {
      Logger::log(WARN_MSG, "Creating String setting...");
      json["Settings"][index]["name"] = name;
      json["Settings"][index]["type"] = typeStr;
      json["Settings"][index]["value"] = "";
      json["Settings"][index]["range"]["min"] = "";
      json["Settings"][index]["range"]["max"] = "";

      if (serializeJson(json, settings_string) == 0) {
        Logger::log(WARN_MSG, "Failed to write to string");
      }
    }
    else if (typeStr == "Int") {
      Logger::log(WARN_MSG, "Creating Int setting...");
      json["Settings"][index]["name"] = name;
      json["Settings"][index]["type"] = typeStr;
      json["Settings"][index]["value"] = 1;
      json["Settings"][index]["range"]["min"] = 1;
      json["Settings"][index]["range"]["max"] = 10;

      if (serializeJson(json, settings_string) == 0) {
        Logger::log(WARN_MSG, "Failed to write to string");
      }
    }
  }

  if (settings_string == "")
    return false;

  bool ok = this->writeSettingsBlob(settings_string);

  if (ok && !spec)
    this->printJsonSettings(settings_string);

  return ok;
}

void Settings::main(uint32_t currentTime) {
  
}
