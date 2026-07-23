#include "WiFiOps.h"
#include "BatteryInterface.h"
#include "Switches.h"
#include <algorithm>
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include <Update.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern WiFiOps wifi_ops;
extern BatteryInterface battery;
extern bool g_force_display_redraw;
extern Switches c_btn;

static String scPath(const String& filePath, const String& service);

// Survives ESP.restart() (reboot-to-upload dock path) but not a power cycle.
// Set once the arrival's dock upload has drained; blocks re-docking until the
// trigger SSID has been absent long enough to count as a departure.
RTC_NOINIT_ATTR bool rtc_dock_done;

// Set by a user trigger (dock menu / serial 'u'), consumed once on the next
// boot's clean-heap window. RTC_NOINIT_ATTR (not RTC_DATA_ATTR) so the flag
// survives ESP.restart — an initialized RTC_DATA var gets re-zeroed on the
// reboot on this C5. Cleared on a cold boot in otaCheckPending().
RTC_NOINIT_ATTR bool rtc_ota_check;

static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const char MAGIC[4] = {'E','N','O','W'};

static constexpr uint8_t ESPNOW_CHANNEL = 6;

// Retry behavior for nodes
static constexpr uint32_t REQ_INITIAL_MS = 300;   // first retry delay
static constexpr uint32_t REQ_MAX_MS     = 5000;  // cap retry interval

static uint8_t g_core_mac[6] = {0};
static bool g_have_core = false;
static bool g_secure_ready = false;

static uint32_t g_hb_counter = 0;
static unsigned long g_last_hb_ms = 0;

// Retry state
static unsigned long g_last_req_ms = 0;
static unsigned long g_last_debug_print = 0;
static uint32_t g_req_interval_ms = REQ_INITIAL_MS;

static inline uint16_t rd_le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd_be24(const uint8_t *p) {
  return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
}

static inline bool oui_type_match(const uint8_t *p, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t type) {
  return p[0] == b0 && p[1] == b1 && p[2] == b2 && p[3] == type;
}

static inline bool rsn_suite_is(const uint8_t *suite, uint8_t type) {
  return suite[0] == 0x00 && suite[1] == 0x0f && suite[2] == 0xac && suite[3] == type;
}

static inline bool ms_wpa_suite_is(const uint8_t *suite, uint8_t type) {
  return suite[0] == 0x00 && suite[1] == 0x50 && suite[2] == 0xf2 && suite[3] == type;
}

static const uint8_t scan_channels[] = {
  // 2.4 GHz
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,

  // 5 GHz
  36, 40, 44, 48,
  52, 56, 60, 64,
  100, 112, 116, 120, 124, 128, 132, 136, 140, 144,
  149, 153, 157, 161, 165, 169, 173, 177
};

#define NUM_SCAN_CHANNELS (sizeof(scan_channels) / sizeof(scan_channels[0]))

uint8_t assigned_start_idx = 0;
uint8_t assigned_end_idx = NUM_SCAN_CHANNELS - 1;
uint8_t assigned_node_index = 0;
uint8_t assigned_node_count = 1;
uint8_t assignment_version = 0;

uint8_t pmk[16];
uint8_t lmk[16];

NodeRecord node_table[MAX_NODES];

AircraftRecord* aircraft_table = nullptr;
portMUX_TYPE aircraft_mux = portMUX_INITIALIZER_UNLOCKED;
uint32_t aircraft_session_total = 0;

enum MsgType : uint8_t {
  MSG_CORE_REQUEST   = 1,
  MSG_CORE_REPLY     = 2,
  MSG_HEARTBEAT      = 3,
  MSG_TEXT           = 4,
  MSG_ADMIN          = 5,
  MSG_AIRCRAFT       = 6
};

WebServer server(80);

struct promisc_ap_t {
  uint8_t bssid[6];
  char    ssid[33];
  uint8_t channel;
  int8_t  rssi;
  uint8_t auth;
  uint8_t kind;
};

static QueueHandle_t g_ap_queue = nullptr;
static const uint16_t AP_QUEUE_LEN = 96;
static const uint32_t BLE_SCAN_INTERVAL_MS = 2000;

static const uint8_t FLOCK_OUIS[][3] = {
  {0x70,0xC9,0x4E},{0x3C,0x91,0x80},{0xD8,0xF3,0xBC},{0x80,0x30,0x49},{0xB8,0x35,0x32},
  {0x14,0x5A,0xFC},{0x74,0x4C,0xA1},{0x08,0x3A,0x88},{0x9C,0x2F,0x9D},{0xC0,0x35,0x32},
  {0x94,0x08,0x53},{0xE4,0xAA,0xEA},{0xF4,0x6A,0xDD},{0xF8,0xA2,0xD6},{0x24,0xB2,0xB9},
  {0x00,0xF4,0x8D},{0xD0,0x39,0x57},{0xE8,0xD0,0xFC},{0xE0,0x4F,0x43},{0xB8,0x1E,0xA4},
  {0x70,0x08,0x94},{0x58,0x8E,0x81},{0xEC,0x1B,0xBD},{0x3C,0x71,0xBF},{0x58,0x00,0xE3},
  {0x90,0x35,0xEA},{0x5C,0x93,0xA2},{0x64,0x6E,0x69},{0x48,0x27,0xEA},{0xA4,0xCF,0x12},
  {0x82,0x6B,0xF2},{0xB4,0x1E,0x52}
};
static const uint8_t FLOCK_OUI_COUNT = sizeof(FLOCK_OUIS) / 3;

static inline bool fuzz_is_flock_oui(const uint8_t* mac) {
  for (uint8_t i = 0; i < FLOCK_OUI_COUNT; i++)
    if (mac[0] == FLOCK_OUIS[i][0] && mac[1] == FLOCK_OUIS[i][1] && mac[2] == FLOCK_OUIS[i][2])
      return true;
  return false;
}

static void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT || g_ap_queue == nullptr) return;

  const wifi_promiscuous_pkt_t *ppkt = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *frame = ppkt->payload;
  const uint16_t len = ppkt->rx_ctrl.sig_len;

  if (frame == nullptr || len < 24) return;

  const uint8_t fc0 = frame[0];
  if (((fc0 >> 2) & 0x03) != 0) return;
  const uint8_t subtype = (fc0 >> 4) & 0x0F;

  if (subtype == 4) {
    const uint8_t *tx = frame + 10;
    if (!fuzz_is_flock_oui(tx)) return;
    if (len < 26 || frame[24] != 0 || frame[25] != 0) return;
    promisc_ap_t fr;
    memcpy(fr.bssid, tx, 6);
    fr.rssi    = ppkt->rx_ctrl.rssi;
    fr.channel = ppkt->rx_ctrl.channel;
    fr.auth    = 0;
    fr.ssid[0] = '\0';
    fr.kind    = 1;
    BaseType_t fhpw = pdFALSE;
    xQueueSendFromISR(g_ap_queue, &fr, &fhpw);
    return;
  }

  if (subtype != 8 && subtype != 5) return;
  if (len < 36) return;

  promisc_ap_t r;
  r.kind = 0;
  memcpy(r.bssid, frame + 16, 6);
  r.rssi    = ppkt->rx_ctrl.rssi;
  r.channel = ppkt->rx_ctrl.channel;
  r.auth    = (uint8_t)wifi_ops.getAuthType(ppkt);
  r.ssid[0] = '\0';

  const uint8_t *ies = frame + 36;
  size_t ies_len = (size_t)len - 36;
  while (ies_len >= 2) {
    const uint8_t id = ies[0];
    const uint8_t elen = ies[1];
    if (ies_len < (size_t)(2 + elen)) break;
    if (id == 0) {
      const uint8_t n = elen > 32 ? 32 : elen;
      memcpy(r.ssid, ies + 2, n);
      r.ssid[n] = '\0';
    } else if (id == 3 && elen >= 1) {
      r.channel = ies[2];
    }
    ies += (2 + elen);
    ies_len -= (2 + elen);
  }

  BaseType_t hpw = pdFALSE;
  xQueueSendFromISR(g_ap_queue, &r, &hpw);
}


class scanCallbacks : public NimBLEScanCallbacks {

  void onDiscovered(const NimBLEAdvertisedDevice* advertisedDevice) override {
    extern WiFiOps wifi_ops;

    uint8_t macBytes[6];

    if (wifi_ops.run_mode == SOLO_MODE) {
      // Only a fix WITH a timestamp can be stamped live; otherwise buffer (so a
      // fix-but-no-datetime hit gets a reconstructed FirstSeen on backfill,
      // never a blank one).
      bool can_stamp = wifi_ops.effectiveFix() && !gps.getDatetime().isEmpty();
      if ((gps.getGpsModuleStatus()) && (sd_obj.supported) &&
          (can_stamp || wifi_ops.isGpsBufferingEnabled())) {

        utils.stringToMac(advertisedDevice->getAddress().toString().c_str(), macBytes);

        if (wifi_ops.seen_mac(macBytes))
          return;

        wifi_ops.save_mac(macBytes);

        wifi_ops.setCurrentBLECount(wifi_ops.getCurrentBLECount() + 1);

        wifi_ops.setTotalBLECount(wifi_ops.getTotalBLECount() + 1);

        String ble_addr = (String)advertisedDevice->getAddress().toString().c_str();

        String ble_info = "";
        std::string ble_name = advertisedDevice->getName();
        if (!ble_name.empty()) {
          ble_info = (String)ble_name.c_str();
          ble_info.replace(",", "_");
          ble_info.replace(";", "_");
          ble_info.replace("\n", "_");
          ble_info.replace("\r", "_");
        }
        ble_info += ";";
        if (advertisedDevice->haveManufacturerData()) {
          uint8_t ble_md_count = advertisedDevice->getManufacturerDataCount();
          for (uint8_t ble_mi = 0; ble_mi < ble_md_count; ble_mi++) {
            std::string ble_md = advertisedDevice->getManufacturerData(ble_mi);
            for (size_t ble_bi = 0; ble_bi < ble_md.length(); ble_bi++) {
              char ble_hx[3];
              snprintf(ble_hx, sizeof(ble_hx), "%02X", (uint8_t)ble_md[ble_bi]);
              ble_info += ble_hx;
            }
          }
        }
        ble_info += ";";
        uint8_t ble_uuid_count = advertisedDevice->getServiceUUIDCount();
        for (uint8_t ble_ui = 0; ble_ui < ble_uuid_count; ble_ui++) {
          if (ble_ui) ble_info += "|";
          ble_info += (String)advertisedDevice->getServiceUUID(ble_ui).toString().c_str();
        }

        if (advertisedDevice->haveManufacturerData()) {
          std::string ble_md0 = advertisedDevice->getManufacturerData(0);
          if (ble_md0.length() >= 2 && (uint8_t)ble_md0[0] == FLOCK_BLE_CID_LO && (uint8_t)ble_md0[1] == FLOCK_BLE_CID_HI)
            wifi_ops.noteFuzzHit(FUZZ_CAM, "Flock", advertisedDevice->getRSSI(), nullptr);
        }
        if (macBytes[0] == AXON_OUI0 && macBytes[1] == AXON_OUI1 && macBytes[2] == AXON_OUI2)
          wifi_ops.noteFuzzHit(FUZZ_LEO, "Axon?", advertisedDevice->getRSSI(), nullptr);

        if (can_stamp) {
          gps_snapshot_t snap = gps.getSnapshot();
          String wardrive_line = ble_addr + "," + ble_info + ",[BLE]," + snap.datetime + ",0," + (String)advertisedDevice->getRSSI() + "," + snap.lat + "," + snap.lon + "," + snap.alt + "," + snap.accuracy + ",BLE";
          Logger::log(GUD_MSG, (String)wifi_ops.mac_history_cursor + " | " + wardrive_line);
          buffer.append(wardrive_line + "\n");
        }
        else {
          // No usable fix: queue the BLE hit to /pending.csv for backfill on reacquire.
          wifi_ops.bufferBleDetection(ble_addr, advertisedDevice->getRSSI());
        }
      }
    }
    else if (wifi_ops.run_mode == NODE_MODE) {
      utils.stringToMac(advertisedDevice->getAddress().toString().c_str(), macBytes);

      if (wifi_ops.seen_mac(macBytes))
        return;

      wifi_ops.save_mac(macBytes);

      wifi_ops.setCurrentBLECount(wifi_ops.getCurrentBLECount() + 1);

      wifi_ops.setTotalBLECount(wifi_ops.getTotalBLECount() + 1);

      String enow_line = (String)advertisedDevice->getAddress().toString().c_str() + ",,[BLE],0," + (String)(String)advertisedDevice->getRSSI() + ",B";
      Logger::log(GUD_MSG, (String)wifi_ops.mac_history_cursor + " | " + enow_line);
      if (wifi_ops.use_encryption)
        wifi_ops.sendEncryptedStringToCore(enow_line);
      else
        wifi_ops.sendBroadcastStringPlain(enow_line);
    }
  }
};

int WiFiOps::getAuthType(const wifi_promiscuous_pkt_t *ppkt) {
  if (!ppkt) {
    return WIFI_AUTH_OPEN;
  }

  const uint8_t *frame = ppkt->payload;
  const uint16_t len = ppkt->rx_ctrl.sig_len;

  if (!frame || len < 36) {
    return WIFI_AUTH_OPEN;
  }

  const uint8_t fc0 = frame[0];
  const uint8_t type = (fc0 >> 2) & 0x03;
  const uint8_t subtype = (fc0 >> 4) & 0x0F;

  if (type != 0) {
    return WIFI_AUTH_OPEN;
  }

  if (!(subtype == 8 || subtype == 5)) {
    return WIFI_AUTH_OPEN;
  }

  const size_t hdr_len = 24;
  const size_t fixed_len = 12;

  if (len < hdr_len + fixed_len) {
    return WIFI_AUTH_OPEN;
  }

  const uint8_t *fixed = frame + hdr_len;
  const uint16_t capability = rd_le16(fixed + 10);

  const bool privacy = (capability & 0x0010) != 0;

  const uint8_t *ies = frame + hdr_len + fixed_len;
  size_t ies_len = len - hdr_len - fixed_len;

  bool has_rsn = false;
  bool has_wpa = false;
  bool has_wapi = false;

  bool rsn_has_psk = false;
  bool rsn_has_8021x = false;
  bool rsn_has_sae = false;
  bool rsn_has_owe = false;

  bool wpa_has_psk = false;
  bool wpa_has_8021x = false;

  while (ies_len >= 2) {
    const uint8_t id = ies[0];
    const uint8_t elen = ies[1];

    if (ies_len < (size_t)(2 + elen)) {
      break;
    }

    const uint8_t *data = ies + 2;

    if (id == 48 && elen >= 8) {
      has_rsn = true;

      const uint8_t *p = data;
      size_t rem = elen;

      if (rem < 2) {
        goto next_ie;
      }
      p += 2;
      rem -= 2;

      if (rem < 4) {
        goto next_ie;
      }
      p += 4;
      rem -= 4;

      if (rem < 2) {
        goto next_ie;
      }
      uint16_t pairwise_count = rd_le16(p);
      p += 2;
      rem -= 2;

      if (rem < (size_t)pairwise_count * 4) {
        goto next_ie;
      }
      p += pairwise_count * 4;
      rem -= pairwise_count * 4;

      if (rem < 2) {
        goto next_ie;
      }
      uint16_t akm_count = rd_le16(p);
      p += 2;
      rem -= 2;

      if (rem < (size_t)akm_count * 4) {
        goto next_ie;
      }

      for (uint16_t i = 0; i < akm_count; ++i) {
        const uint8_t *akm = p + (i * 4);

        if (rsn_suite_is(akm, 1) || rsn_suite_is(akm, 5)) {
          rsn_has_8021x = true;
        } else if (rsn_suite_is(akm, 2) || rsn_suite_is(akm, 6)) {
          rsn_has_psk = true;
        } else if (rsn_suite_is(akm, 8) || rsn_suite_is(akm, 9)) {
          rsn_has_sae = true;
        } else if (rsn_suite_is(akm, 18)) {
          rsn_has_owe = true;
        }
      }

      goto next_ie;
    }

    if (id == 221 && elen >= 8) {
      if (oui_type_match(data, 0x00, 0x50, 0xf2, 0x01)) {
        has_wpa = true;

        const uint8_t *p = data + 4;
        size_t rem = elen - 4;

        if (rem < 2) {
          goto next_ie;
        }
        p += 2;
        rem -= 2;

        if (rem < 4) {
          goto next_ie;
        }
        p += 4;
        rem -= 4;

        if (rem < 2) {
          goto next_ie;
        }
        uint16_t ucount = rd_le16(p);
        p += 2;
        rem -= 2;

        if (rem < (size_t)ucount * 4) {
          goto next_ie;
        }
        p += ucount * 4;
        rem -= ucount * 4;

        if (rem < 2) {
          goto next_ie;
        }
        uint16_t akm_count = rd_le16(p);
        p += 2;
        rem -= 2;

        if (rem < (size_t)akm_count * 4) {
          goto next_ie;
        }

        for (uint16_t i = 0; i < akm_count; ++i) {
          const uint8_t *akm = p + (i * 4);

          if (ms_wpa_suite_is(akm, 1)) {
            wpa_has_8021x = true;
          } else if (ms_wpa_suite_is(akm, 2)) {
            wpa_has_psk = true;
          }
        }

        goto next_ie;
      }
    }

    if (id == 68) {
      has_wapi = true;
      goto next_ie;
    }

next_ie:
    ies += (2 + elen);
    ies_len -= (2 + elen);
  }

  // Classification

  #ifdef WIFI_AUTH_WAPI_PSK
  if (has_wapi) {
    return WIFI_AUTH_WAPI_PSK;
  }
  #endif

  #ifdef WIFI_AUTH_OWE
  if (has_rsn && rsn_has_owe) {
    return WIFI_AUTH_OWE;
  }
  #endif

  #ifdef WIFI_AUTH_WPA3_PSK
  if (has_rsn && rsn_has_sae && !rsn_has_psk) {
    return WIFI_AUTH_WPA3_PSK;
  }
  #endif

  #ifdef WIFI_AUTH_WPA2_WPA3_PSK
  if (has_rsn && rsn_has_sae && rsn_has_psk) {
    return WIFI_AUTH_WPA2_WPA3_PSK;
  }
  #endif

  if (has_rsn && rsn_has_8021x) {
    return WIFI_AUTH_WPA2_ENTERPRISE;
  }

  #ifdef WIFI_AUTH_ENTERPRISE
  if (has_wpa && wpa_has_8021x && !has_rsn) {
    return WIFI_AUTH_ENTERPRISE;
  }
  #else
  if (has_wpa && wpa_has_8021x && !has_rsn) {
    return WIFI_AUTH_WPA2_ENTERPRISE;
  }
  #endif

  if ((has_wpa && wpa_has_psk) && (has_rsn && rsn_has_psk)) {
    return WIFI_AUTH_WPA_WPA2_PSK;
  }

  if (has_rsn && rsn_has_psk) {
    return WIFI_AUTH_WPA2_PSK;
  }

  if (has_wpa && wpa_has_psk) {
    return WIFI_AUTH_WPA_PSK;
  }

  // WEP heuristic:
  // privacy bit set, but no WPA/RSN/WAPI IEs
  if (privacy && !has_rsn && !has_wpa && !has_wapi) {
    return WIFI_AUTH_WEP;
  }

  if (!privacy && !has_rsn && !has_wpa && !has_wapi) {
    return WIFI_AUTH_OPEN;
  }

  if (privacy) {
    return WIFI_AUTH_WEP;
  }

  return WIFI_AUTH_OPEN;
}

uint16_t WiFiOps::macToSuffix(const uint8_t* mac) {
  return ((uint16_t)mac[4] << 8) | mac[5];
}

void WiFiOps::macSuffixToStr(uint16_t suffix, char* out6) {
  sprintf(out6, "%02X:%02X", (suffix >> 8) & 0xFF, suffix & 0xFF);
}

int WiFiOps::findNodeByMacSuffix(uint16_t suffix) {
  for (int i = 0; i < MAX_NODES; i++) {
    if ((node_table[i].flags & NODE_FLAG_ACTIVE) &&
        node_table[i].mac_suffix == suffix) {
      return i;
    }
  }
  return -1;
}

int WiFiOps::findNodeByMac(const uint8_t* mac) {
  return this->findNodeByMacSuffix(this->macToSuffix(mac));
}

int WiFiOps::allocateNodeSlot(const uint8_t* mac) {
  uint16_t suffix = macToSuffix(mac);

  for (int i = 0; i < MAX_NODES; i++) {
    if (!(node_table[i].flags & NODE_FLAG_ACTIVE)) {
      node_table[i].mac_suffix = suffix;
      node_table[i].last_seen_ms = millis();
      node_table[i].assigned_index = 0;
      node_table[i].start_channel_idx = 0;
      node_table[i].end_channel_idx = NUM_SCAN_CHANNELS - 1;
      node_table[i].last_admin_version_sent = 0; // force update
      node_table[i].flags = NODE_FLAG_ACTIVE | NODE_FLAG_ADMIN_DIRTY;
      return i;
    }
  }

  return -1;
}

int WiFiOps::touchNode(const uint8_t* mac, bool& isNewNode) {
  isNewNode = false;
  uint16_t suffix = macToSuffix(mac);

  int slot = findNodeByMacSuffix(suffix);
  if (slot >= 0) {
    node_table[slot].last_seen_ms = millis();
    return slot;
  }

  slot = allocateNodeSlot(mac);
  if (slot >= 0) {
    node_table[slot].last_seen_ms = millis();
    isNewNode = true;
    char mac_str[] = "00:00";
    this->macSuffixToStr(suffix, mac_str);
    Serial.print("Node added: ");
    Serial.print(mac_str);
    Serial.println(" | Node count updated: " + (String)this->getActiveNodeCount());
  }
  return slot;
}

bool WiFiOps::removeStaleNodes() {
  bool changed = false;
  uint32_t now = millis();

  for (int i = 0; i < MAX_NODES; i++) {
    if (node_table[i].flags & NODE_FLAG_ACTIVE) {
      if ((uint32_t)(now - node_table[i].last_seen_ms) > NODE_TIMEOUT_MS) {
        memset(&node_table[i], 0, sizeof(NodeRecord));
        changed = true;
      }
    }
  }

  return changed;
}

void WiFiOps::recalculateChannelAssignments() {
  uint8_t active_slots[MAX_NODES];
  uint8_t active_count = 0;

  for (uint8_t i = 0; i < MAX_NODES; i++) {
    if (node_table[i].flags & NODE_FLAG_ACTIVE) {
      active_slots[active_count++] = i;
    }
  }

  if (active_count == 0) {
    return;
  }

  for (uint8_t node_num = 0; node_num < active_count; node_num++) {
    uint8_t slot = active_slots[node_num];

    uint8_t start_idx = (node_num * NUM_SCAN_CHANNELS) / active_count;
    uint8_t end_idx   = (((node_num + 1) * NUM_SCAN_CHANNELS) / active_count) - 1;

    node_table[slot].assigned_index = node_num;
    node_table[slot].start_channel_idx = start_idx;
    node_table[slot].end_channel_idx = end_idx;
    node_table[slot].flags |= NODE_FLAG_ADMIN_DIRTY;
  }
}

uint8_t WiFiOps::getNodeStartChannel(uint8_t slot) {
  return scan_channels[node_table[slot].start_channel_idx];
}

uint8_t WiFiOps::getNodeEndChannel(uint8_t slot) {
  return scan_channels[node_table[slot].end_channel_idx];
}

uint8_t WiFiOps::getActiveNodeCount() {
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_NODES; i++) {
    if (node_table[i].flags & NODE_FLAG_ACTIVE) {
      count++;
    }
  }

  return count;
}

void WiFiOps::touchAircraft(const enow_aircraft_msg_t* a, uint32_t now) {
  if (!aircraft_table) return;
  portENTER_CRITICAL(&aircraft_mux);
  int slot = -1, free_slot = -1;
  for (int i = 0; i < MAX_AIRCRAFT; i++) {
    if (aircraft_table[i].used && aircraft_table[i].icao == a->icao) { slot = i; break; }
    if (!aircraft_table[i].used && free_slot < 0) free_slot = i;
  }
  if (slot < 0) {
    if (free_slot < 0) { portEXIT_CRITICAL(&aircraft_mux); return; }
    slot = free_slot;
    aircraft_table[slot].used = true;
    aircraft_table[slot].icao = a->icao;
    aircraft_table[slot].last_written_ms = 0;
    aircraft_session_total++;
  }
  memcpy(aircraft_table[slot].flight, a->callsign, 8);
  aircraft_table[slot].flight[8] = '\0';
  aircraft_table[slot].lat = a->lat;
  aircraft_table[slot].lon = a->lon;
  aircraft_table[slot].alt_baro = a->altitude_ft;
  aircraft_table[slot].gs = a->ground_speed_kt;
  aircraft_table[slot].track = a->track_deg;
  aircraft_table[slot].last_seen_ms = now;
  portEXIT_CRITICAL(&aircraft_mux);
}

uint32_t WiFiOps::aircraftSessionTotal() {
  portENTER_CRITICAL(&aircraft_mux);
  uint32_t t = aircraft_session_total;
  portEXIT_CRITICAL(&aircraft_mux);
  return t;
}

void WiFiOps::flushAircraftBuffer(uint32_t now) {
  if (!aircraft_table) return;
  for (int i = 0; i < MAX_AIRCRAFT; i++) {
    bool do_write = false, do_evict = false;
    AircraftRecord rec;
    portENTER_CRITICAL(&aircraft_mux);
    if (aircraft_table[i].used) {
      if (aircraft_table[i].last_written_ms == 0 ||
          now - aircraft_table[i].last_written_ms >= AIRCRAFT_SD_WRITE_MS) {
        rec = aircraft_table[i];
        aircraft_table[i].last_written_ms = now;
        do_write = true;
      }
      if (aircraft_table[i].last_written_ms != 0 &&
          now - aircraft_table[i].last_seen_ms > AIRCRAFT_TIMEOUT_MS) {
        do_evict = true;
      }
    }
    portEXIT_CRITICAL(&aircraft_mux);

    if (do_write && sd_obj.supported) {
      File f = SD.open(AIRCRAFT_PENDING_FILE, FILE_APPEND);
      if (f) {
        f.printf("%06lx,%s,%.6f,%.6f,%ld,%u,%u\n",
                 (unsigned long)rec.icao, rec.flight, rec.lat, rec.lon,
                 (long)rec.alt_baro, rec.gs, rec.track);
        f.close();
      }
    }
    if (do_evict) {
      portENTER_CRITICAL(&aircraft_mux);
      aircraft_table[i].used = false;
      portEXIT_CRITICAL(&aircraft_mux);
    }
  }
}

int WiFiOps::aircraftCount() {
  if (!aircraft_table) return 0;
  uint32_t now = millis();
  int n = 0;
  portENTER_CRITICAL(&aircraft_mux);
  for (int i = 0; i < MAX_AIRCRAFT; i++) {
    if (!aircraft_table[i].used) continue;
    if (now - aircraft_table[i].last_seen_ms <= AIRCRAFT_TIMEOUT_MS) n++;
  }
  portEXIT_CRITICAL(&aircraft_mux);
  return n;
}

int WiFiOps::getAircraftList(AircraftRecord* out, int max_out) {
  if (!aircraft_table || !out || max_out <= 0) return 0;
  uint32_t now = millis();
  int n = 0;
  portENTER_CRITICAL(&aircraft_mux);
  for (int i = 0; i < MAX_AIRCRAFT && n < max_out; i++) {
    if (!aircraft_table[i].used) continue;
    if (now - aircraft_table[i].last_seen_ms > AIRCRAFT_TIMEOUT_MS) continue;
    out[n++] = aircraft_table[i];
  }
  portEXIT_CRITICAL(&aircraft_mux);
  return n;
}

void WiFiOps::markAllActiveNodesAdminDirty() {
  for (uint8_t i = 0; i < MAX_NODES; i++) {
    if (node_table[i].flags & NODE_FLAG_ACTIVE) {
      node_table[i].flags |= NODE_FLAG_ADMIN_DIRTY;
    }
  }
}

void WiFiOps::handleNodeTopologyChange() {
  this->current_assignment_version++;
  if (this->current_assignment_version == 0) 
    this->current_assignment_version = 1;

  this->recalculateChannelAssignments();
  this->markAllActiveNodesAdminDirty();
  //this->debugPrintNodeTable();
}

void WiFiOps::debugPrintNodeTable() {
  Serial.println("\n===== NODE TABLE =====");
  Serial.println("Current Assignment Version: " + (String)this->current_assignment_version);

  for (uint8_t i = 0; i < MAX_NODES; i++) {

    if (!(node_table[i].flags & NODE_FLAG_ACTIVE)) {
      continue;
    }

    uint8_t start_ch = scan_channels[node_table[i].start_channel_idx];
    uint8_t end_ch   = scan_channels[node_table[i].end_channel_idx];

    uint32_t age = millis() - node_table[i].last_seen_ms;

    Serial.printf(
      "Slot %u | MAC:%04X | idx:%u | ch:%u-%u | ver:%u | flags:0x%02X | age:%lu ms\n",
      i,
      node_table[i].mac_suffix,
      node_table[i].assigned_index,
      start_ch,
      end_ch,
      node_table[i].last_admin_version_sent,
      node_table[i].flags,
      age
    );
  }

  Serial.println("======================\n");
}

void WiFiOps::setFixedChannel(uint8_t ch) {
  // Disable power save (prevents weird timing/channel behavior)
  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_set_promiscuous(true);

  // Force primary channel
  esp_err_t e = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  if (e != ESP_OK) {
    Serial.printf("esp_wifi_set_channel failed: %d (0x%X)\n", (int)e, (unsigned)e);
    return;
  }

  // Verify
  uint8_t primary = 0;
  wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("Home channel is now: %u\n", primary);

  esp_wifi_set_promiscuous(false);
}

bool WiFiOps::addPeerWithMode(const uint8_t* mac, bool encrypt, const uint8_t lmk16[16]) {
  // If peer exists (possibly with wrong mode), delete it first
  if (esp_now_is_peer_exist(mac)) {
    esp_now_del_peer(mac);
    delay(10); // small settle
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;          // follow home channel (safer)
  peerInfo.encrypt = encrypt;

  if (encrypt) {
    memcpy(peerInfo.lmk, lmk16, 16);
  }

  return (esp_now_add_peer(&peerInfo) == ESP_OK);
}

bool WiFiOps::sendAdminToNodeSlot(uint8_t slot, const uint8_t* dest_mac) {
  extern WiFiOps wifi_ops;

  if (!(node_table[slot].flags & NODE_FLAG_ACTIVE)) return false;

  enow_admin_msg_t msg = {};
  memcpy(msg.magic, MAGIC, 4);
  msg.type = MSG_ADMIN;
  msg.assignment_version = wifi_ops.current_assignment_version;
  msg.node_index = node_table[slot].assigned_index;
  msg.node_count = wifi_ops.getActiveNodeCount();
  msg.start_channel_idx = node_table[slot].start_channel_idx;
  msg.end_channel_idx = node_table[slot].end_channel_idx;

  // Temporary plaintext peer
  if (!esp_now_is_peer_exist(dest_mac)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, dest_mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      return false;
    }
  }

  esp_err_t res = esp_now_send(dest_mac, (uint8_t*)&msg, sizeof(msg));
  if (res != ESP_OK) {
    char macStr[18];
    utils.macToStr(dest_mac, macStr);
    Serial.printf("ESPNOW send failed to %s err=%d\n", macStr, res);
    esp_now_del_peer(dest_mac);
    return false;
  }

  esp_now_del_peer(dest_mac);

  node_table[slot].last_admin_version_sent = wifi_ops.current_assignment_version;
  node_table[slot].flags &= ~NODE_FLAG_ADMIN_DIRTY;
  return true;
}

void WiFiOps::sendCoreRequest() {
  enow_text_msg_t msg = {};
  memcpy(msg.magic, MAGIC, 4);
  msg.type = MSG_CORE_REQUEST;
  msg.counter = 0;

  // Broadcast peer must be unencrypted
  addPeerWithMode(BROADCAST_MAC, false, nullptr);

  esp_err_t res = esp_now_send(BROADCAST_MAC, (uint8_t*)&msg, sizeof(msg));
  Serial.printf("NODE: Broadcast CORE_REQUEST -> %s (next in %lu ms)\n",
                (res == ESP_OK) ? "OK" : "FAIL",
                (unsigned long)g_req_interval_ms);
}

void WiFiOps::sendCoreReply(const uint8_t* destMac) {
  enow_text_msg_t msg = {};
  memcpy(msg.magic, MAGIC, 4);
  msg.type = MSG_CORE_REPLY;
  msg.counter = 0;

  if (!addPeerWithMode(destMac, false, nullptr)) {
    Logger::log(WARN_MSG, "CORE: Failed to add NODE peer (reply)");
    return;
  }

  esp_err_t res = esp_now_send(destMac, (uint8_t*)&msg, sizeof(msg));

  char macStr[18];
  utils.macToStr(destMac, macStr);

  Serial.printf("CORE: Sent CORE_REPLY to %s -> %s\n",
                macStr, (res == ESP_OK) ? "OK" : "FAIL");
}

void WiFiOps::runAdminWindowAfterScanCycle() {
  this->setFixedChannel(ESPNOW_CHANNEL);
  delay(5);
  this->sendHeartbeat();

  // Wait for ADMIN response
  unsigned long start = millis();
  while ((millis() - start) < ADMIN_WAIT_MS) {
    delay(5);
  }
}

void WiFiOps::startNextNodeAssignedScan() {
  if (assigned_start_idx >= NUM_SCAN_CHANNELS ||
      assigned_end_idx >= NUM_SCAN_CHANNELS ||
      assigned_start_idx > assigned_end_idx) {
    WiFi.scanNetworks(true, true, false, 80);
    return;
  }

  if (current_assigned_scan_idx < assigned_start_idx ||
      current_assigned_scan_idx > assigned_end_idx) {
    current_assigned_scan_idx = assigned_start_idx;
  }

  uint8_t channel = scan_channels[current_assigned_scan_idx];
  WiFi.scanNetworks(true, true, false, 80, channel);

  current_assigned_scan_idx++;
  if (current_assigned_scan_idx > assigned_end_idx) {
    current_assigned_scan_idx = assigned_start_idx;
  }
}

void WiFiOps::sendHeartbeat() {
  extern WiFiOps wifi_ops;

  if (wifi_ops.use_encryption) {
    if (!g_secure_ready) return;
  }

  enow_text_msg_t msg = {};
  memcpy(msg.magic, MAGIC, 4);
  msg.type = MSG_HEARTBEAT;
  msg.counter = g_hb_counter++;

  esp_err_t res;
  if (wifi_ops.use_encryption) {
    res = esp_now_send(g_core_mac, (uint8_t*)&msg, sizeof(msg));
  } else {
    res = esp_now_send(BROADCAST_MAC, (uint8_t*)&msg, sizeof(msg));
  }

  if (res != ESP_OK) {
    Logger::log(WARN_MSG, "NODE: Heartbeat send FAIL");
  } else {
    Logger::log(GUD_MSG, "NODE: Sent heartbeat");
  }
}

bool WiFiOps::sendEncryptedStringToCore(const String& s) {
  this->setFixedChannel(ESPNOW_CHANNEL);

  if (!g_secure_ready) {
    Logger::log(WARN_MSG, "NODE: Not secure-ready; cannot send encrypted text");
    return false;
  }

  enow_text_msg_t msg = {};
  memcpy(msg.magic, MAGIC, 4);
  msg.type = MSG_TEXT;
  msg.counter = 0;  // optional for text

  size_t n = s.length();
  if (n > ENOW_TEXT_MAX) n = ENOW_TEXT_MAX;

  memcpy(msg.text, s.c_str(), n);
  msg.text[n] = '\0';
  msg.len = (uint16_t)n;

  // SEND FULL STRUCT (like heartbeat)
  esp_err_t res = esp_now_send(g_core_mac, (uint8_t*)&msg, sizeof(msg));

  if (res != ESP_OK) {
    Serial.printf("NODE: Encrypted text send FAIL (err=%d)\n", (int)res);
    return false;
  }

  return true;
}

bool WiFiOps::sendBroadcastStringPlain(const String& s) {
  this->setFixedChannel(ESPNOW_CHANNEL);

  static const uint8_t bcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

  if (!esp_now_is_peer_exist(bcast_mac)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, bcast_mac, 6);
    peerInfo.channel = 0;       // follow current home channel
    peerInfo.encrypt = false;   // plaintext broadcast

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Logger::log(WARN_MSG, "NODE: Failed to add broadcast peer");
      return false;
    }
  }

  enow_text_msg_t msg = {};
  memcpy(msg.magic, MAGIC, 4);
  msg.type = MSG_TEXT;
  msg.counter = 0;

  size_t n = s.length();
  if (n > ENOW_TEXT_MAX) n = ENOW_TEXT_MAX;

  memcpy(msg.text, s.c_str(), n);
  msg.text[n] = '\0';
  msg.len = (uint16_t)n;

  esp_err_t res = esp_now_send(bcast_mac, (uint8_t*)&msg, sizeof(msg));

  if (res != ESP_OK) {
    Serial.printf("NODE: Broadcast plaintext send FAIL (err=%d)\n", (int)res);
    return false;
  }

  return true;
}

bool WiFiOps::parseWardriveLine(const enow_text_msg_t& msg, WardriveRecord& out) {
  const char* line = msg.text;   // <-- no copy
  int start = 0;
  int fieldIndex = 0;

  String fields[6];

  while (fieldIndex < 6) {
    const char* comma = strchr(line + start, ',');

    if (!comma) {
      fields[fieldIndex++] = String(line + start);
      break;
    }

    fields[fieldIndex++] = String(line + start).substring(0, comma - (line + start));
    start = (comma - line) + 1;
  }

  if (fieldIndex != 6) return false;

  out.bssid    = fields[0];
  out.essid    = fields[1];
  out.security = fields[2];
  out.channel  = fields[3].toInt();
  out.rssi     = fields[4].toInt();
  out.type     = fields[5];

  return true;
}

void WiFiOps::OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  extern WiFiOps wifi_ops;

  bool isNewNode = false;
  int slot = -1;

  // Need at least magic[4] + type[1]
  if (!info || !data || len < 5) return;

  // Validate protocol magic
  if (memcmp(data, MAGIC, 4) != 0) return;

  const uint8_t msgType = data[4];
  const int rssi = (info->rx_ctrl) ? info->rx_ctrl->rssi : 0;

  char srcMacStr[18];
  utils.macToStr(info->src_addr, srcMacStr);

  if (wifi_ops.run_mode == CORE_MODE) {

    if (msgType == MSG_CORE_REQUEST) {
      if (len < (int)sizeof(enow_text_msg_t)) return;

      const enow_text_msg_t* msg = (const enow_text_msg_t*)data;

      Serial.printf("CORE: RX CORE_REQUEST from %s | RSSI %d dBm\n", srcMacStr, rssi);

      // Track node regardless of encryption success
      slot = wifi_ops.touchNode(info->src_addr, isNewNode);

      // Plaintext reply so node can learn CORE MAC
      wifi_ops.sendCoreReply(info->src_addr);

      // Only set up encrypted peer if encryption is enabled
      if (wifi_ops.use_encryption) {
        if (!wifi_ops.addPeerWithMode(info->src_addr, true, lmk)) {
          Logger::log(WARN_MSG, "CORE: Failed to add ENCRYPTED peer for NODE");
        } else {
          Serial.printf("CORE: Encrypted peer ready for %s\n", srcMacStr);
          if (slot >= 0) {
            node_table[slot].flags |= NODE_FLAG_ENCRYPTED;
          }
        }
      }

      if (slot >= 0 && isNewNode) {
        wifi_ops.handleNodeTopologyChange();
      }

      // Initial pairing is a valid rendezvous point for admin
      if (slot >= 0 && (node_table[slot].flags & NODE_FLAG_ADMIN_DIRTY)) {
        if (wifi_ops.sendAdminToNodeSlot(slot, info->src_addr)) {
          Logger::log(GUD_MSG, "Successfully sent Admin message to Node");
          wifi_ops.debugPrintNodeTable();
        }
      }

      (void)msg;
      return;
    }

    if (msgType == MSG_HEARTBEAT) {
      if (len < (int)sizeof(enow_text_msg_t)) return;

      const enow_text_msg_t* msg = (const enow_text_msg_t*)data;

      Serial.printf("CORE: RX HEARTBEAT from %s | RSSI %d dBm | #%lu\n",
                    srcMacStr, rssi, (unsigned long)msg->counter);

      slot = wifi_ops.touchNode(info->src_addr, isNewNode);

      if (slot >= 0 && isNewNode) {
        wifi_ops.handleNodeTopologyChange();
      }

      // Heartbeat is the preferred rendezvous moment for admin
      if (slot >= 0 && (node_table[slot].flags & NODE_FLAG_ADMIN_DIRTY)) {
        if (wifi_ops.sendAdminToNodeSlot(slot, info->src_addr)) {
          Logger::log(GUD_MSG, "Successfully sent Admin message to Node");
          wifi_ops.debugPrintNodeTable();
        }
      }

      return;
    }

    if (msgType == MSG_TEXT) {
      if (len < (int)sizeof(enow_text_msg_t)) return;

      const enow_text_msg_t* t = (const enow_text_msg_t*)data;

      // Track sender as soon as a valid text packet arrives
      /*slot = wifi_ops.touchNode(info->src_addr, isNewNode);

      if (slot >= 0 && isNewNode) {
        wifi_ops.handleNodeTopologyChange();
      }

      if (slot >= 0 && (node_table[slot].flags & NODE_FLAG_ADMIN_DIRTY)) {
        wifi_ops.sendAdminToNodeSlot(slot, info->src_addr);
      }*/

      if ((t->len <= ENOW_TEXT_MAX) && (!wifi_ops.checkGeofences())) {
        enow_text_msg_t safe = *t;
        safe.text[safe.len] = '\0';
        Serial.printf("CORE: RX WARDRV TEXT from %s: %s\n", srcMacStr, safe.text);

        WardriveRecord rec;
        if (wifi_ops.parseWardriveLine(safe, rec)) {
          uint8_t bssid[6] = {0};
          utils.convertMacStringToUint8(rec.bssid, bssid);

          if (!wifi_ops.seen_mac(bssid)) {
            gps_snapshot_t snap = gps.getSnapshot();

            String type = "WIFI";
            if (rec.type == "B")
              type = "BLE";

            String wardrive_line =
              rec.bssid + "," +
              rec.essid + "," +
              rec.security + "," +
              snap.datetime + "," +
              (String)rec.channel + "," +
              (String)rec.rssi + "," +
              snap.lat + "," +
              snap.lon + "," +
              snap.alt + "," +
              snap.accuracy + "," +
              type;

            Logger::log(GUD_MSG, wardrive_line);

            if (snap.fix && snap.datetime[0] != '\0') {
              // Mark the MAC as logged only once we have a fix and
              // write the record.
              wifi_ops.save_mac(bssid);

              if (type == "WIFI") {
                wifi_ops.setCurrentNetCount(wifi_ops.getCurrentNetCount() + 1);
                wifi_ops.setTotalNetCount(wifi_ops.getTotalNetCount() + 1);

                if (rec.channel > 14) {
                  wifi_ops.setCurrent5gCount(wifi_ops.getCurrent5gCount() + 1);
                } else {
                  wifi_ops.setCurrent2g4Count(wifi_ops.getCurrent2g4Count() + 1);
                }
              } else if (type == "BLE") {
                wifi_ops.setCurrentBLECount(wifi_ops.getCurrentBLECount() + 1);
                wifi_ops.setTotalBLECount(wifi_ops.getTotalBLECount() + 1);
              }

              buffer.append(wardrive_line + "\n");

              // Roll log file if entry limit reached
              uint32_t total_entries = wifi_ops.getCurrent2g4Count() +
              wifi_ops.getCurrent5gCount() +
              wifi_ops.getCurrentBLECount();
              if (total_entries >= LOG_ROLL_ENTRIES) {
                Logger::log(STD_MSG, "[LOG] Rolling log — " +
                String(LOG_ROLL_ENTRIES) + " entries reached");
                wifi_ops.startLog(LOG_FILE_NAME);
                wifi_ops.setCurrent2g4Count(0);
                wifi_ops.setCurrent5gCount(0);
                wifi_ops.setCurrentBLECount(0);
                wifi_ops.setCurrentNetCount(0);
                wifi_ops.pruneOldLogs();
              }
            }
          }
        }
      } else {
        Logger::log(WARN_MSG, "CORE: RX WARDRV TEXT: text len: " + (String)t->len + " > ENOW_TEXT_MAX");
      }

      //if (slot >= 0 && isNewNode) {
      //  wifi_ops.handleNodeTopologyChange();
      //}

      // Do NOT send admin here; wait for heartbeat/check-in window
      return;
    }

    if (msgType == MSG_AIRCRAFT) {
      if (len < (int)sizeof(enow_aircraft_msg_t)) return;
      const enow_aircraft_msg_t* a = (const enow_aircraft_msg_t*)data;
      wifi_ops.touchAircraft(a, millis());
      return;
    }

    // Unknown / unhandled type on CORE
    Serial.printf("CORE: RX unknown type %u from %s | RSSI %d dBm\n",
                  msgType, srcMacStr, rssi);
    return;
  }

  if (wifi_ops.run_mode == NODE_MODE) {

    if (msgType == MSG_CORE_REPLY) {
      if (len < (int)sizeof(enow_text_msg_t)) return;

      memcpy(g_core_mac, info->src_addr, 6);
      g_have_core = true;

      char coreStr[18];
      utils.macToStr(g_core_mac, coreStr);
      Serial.printf("NODE: Learned CORE MAC (plaintext reply): %s | RSSI %d dBm\n", coreStr, rssi);

      if (wifi_ops.use_encryption) {
        if (!wifi_ops.addPeerWithMode(g_core_mac, true, lmk)) {
          Logger::log(WARN_MSG, "NODE: Failed to add ENCRYPTED peer for CORE");
          g_secure_ready = false;
        } else {
          g_secure_ready = true;
          Logger::log(GUD_MSG, "NODE: Encrypted peer ready; switching to encrypted heartbeats...");
        }
      } else {
        g_secure_ready = true;
      }

      return;
    }

    if (msgType == MSG_ADMIN) {
      if (len < (int)sizeof(enow_admin_msg_t)) return;

      const enow_admin_msg_t* admin = (const enow_admin_msg_t*)data;

      if (admin->assignment_version != assignment_version) {
        assignment_version = admin->assignment_version;
        assigned_node_index = admin->node_index;
        assigned_node_count = admin->node_count;
        assigned_start_idx = admin->start_channel_idx;
        assigned_end_idx = admin->end_channel_idx;

        Serial.printf("NODE: New admin assignment v%u | node %u/%u | idx %u-%u\n",
                      assignment_version,
                      assigned_node_index,
                      assigned_node_count,
                      assigned_start_idx,
                      assigned_end_idx);
      }

      return;
    }

    // Unknown / unhandled type on NODE
    Serial.printf("NODE: RX unknown type %u from %s | RSSI %d dBm\n",
                  msgType, srcMacStr, rssi);
    return;
  }
}

void WiFiOps::startESPNow() {
  this->setFixedChannel(ESPNOW_CHANNEL);
  this->computeKeysFromEnowKey();

  if (this->run_mode == CORE_MODE && !aircraft_table) {
    aircraft_table = (AircraftRecord*)calloc(MAX_AIRCRAFT, sizeof(AircraftRecord));
    if (!aircraft_table)
      Logger::log(WARN_MSG, "Failed to allocate aircraft table");
  }

  if (esp_now_init() != ESP_OK) {
    Logger::log(WARN_MSG, "ESP-NOW init failed");
    return;
  }

  if (esp_now_set_pmk(pmk) != ESP_OK) {
    Logger::log(WARN_MSG, "Warning: esp_now_set_pmk failed");
  }

  esp_now_register_recv_cb(OnDataRecv);

  if (this->run_mode == CORE_MODE) {
    Logger::log(STD_MSG, "Role: CORE");
    Logger::log(STD_MSG, "CORE: Fixed channel " + (String)ESPNOW_CHANNEL + ", Waiting for CORE_REQUEST...\n");
  }
  else if (this->run_mode == NODE_MODE) {
    Logger::log(STD_MSG, "Role: NODE");
    Logger::log(STD_MSG, "NODE: Fixed channel " + (String)ESPNOW_CHANNEL + ", probing for CORE...\n");

    g_last_req_ms = millis();

    if (this->use_encryption)
      this->sendCoreRequest();
  }
}

void WiFiOps::derive_key_16(const String& s, uint8_t out16[16]) {
  uint8_t hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char*)s.c_str(), s.length());
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);

  memcpy(out16, hash, 16); // first 16 bytes
}

void WiFiOps::computeKeysFromEnowKey() {
  extern WiFiOps wifi_ops;
  Logger::log(STD_MSG, "computeKeysFromEnowKey Compute key: " + wifi_ops.esp_now_key);
  derive_key_16(wifi_ops.esp_now_key + "_pmk", pmk);
  derive_key_16(wifi_ops.esp_now_key + "_lmk", lmk);
}

void WiFiOps::setCurrentScanMode(uint8_t scan_mode) {
  this->current_scan_mode = scan_mode;
}

bool WiFiOps::getHasCore() {
  return g_have_core;
}

bool WiFiOps::getSecureReady() {
  return g_secure_ready;
}

bool WiFiOps::getNodeReady() {
  if (!this->use_encryption)
    return true;
  else if ((this->getHasCore()) && (this->getSecureReady()))
    return true;

  return false;
}

uint8_t WiFiOps::getCurrentScanMode() {
  return this->current_scan_mode;
}

void WiFiOps::setTotalNetCount(uint32_t count) {
  this->total_net_count = count;
}

void WiFiOps::setTotalBLECount(uint32_t count) {
  this->total_ble_count = count;
}

void WiFiOps::setCurrentNetCount(uint32_t count) {
  this->current_net_count = count;
}

void WiFiOps::setCurrent2g4Count(uint32_t count) {
  this->current_2g4_count = count;
}

void WiFiOps::setCurrent5gCount(uint32_t count) {
  this->current_5g_count = count;
}

void WiFiOps::setCurrentBLECount(uint32_t count) {
  this->current_ble_count = count;
}

uint32_t WiFiOps::getTotalNetCount() {
  return this->total_net_count;
}

uint32_t WiFiOps::getTotalBLECount() {
  return this->total_ble_count;
}

uint32_t WiFiOps::getCurrentNetCount() {
  return this->current_net_count;
}

uint32_t WiFiOps::getCurrent2g4Count() {
  return this->current_2g4_count;
}

uint32_t WiFiOps::getCurrent5gCount() {
  return this->current_5g_count;
}

uint32_t WiFiOps::getCurrentBLECount() {
  return this->current_ble_count;
}

void WiFiOps::scanBLE() {
  pBLEScan->clearResults();
  pBLEScan->start(BLE_SCAN_DURATION, false, false);
}

int WiFiOps::runWardrive(uint32_t currentTime) {

  int scan_status = -1;

  // ---- Chunk 5: geofence check ----
  // Only check when we have a GPS fix — no position, no geofence.
  // Dock mode (Chunk 6) handles K1T upload trigger while paused.
  if (gps.getGpsModuleStatus() && gps.getFixStatus()) {
    if (this->checkGeofences()) {
      // Chunk 6: while geofence-paused, periodically scan for trigger SSID
      // so K1T can still trigger a dock+upload even when wardriving is paused.
      if (millis() - this->geo_passive_scan_time >= DOCK_SCAN_INTERVAL) {
        this->geo_passive_scan_time = millis();
        //Logger::log(STD_MSG, "Calling scanForTriggerSSID from runWardrive");
        bool seen = this->scanForTriggerSSID();
        if (!rtc_dock_done && this->armDock(seen, this->trigger_last_rssi)) {
          Logger::log(STD_MSG, "[DOCK] Trigger SSID confirmed during geofence pause");
          this->dock_state            = DOCK_STATE_CONNECTING;
          this->dock_connect_attempts = 0;
        }
      }
      return -1; // inside a geofence — pause wardriving
    }
  }
  // ---- end geofence check ----

  if (this->run_mode == NODE_MODE) {

    scan_status = WiFi.scanComplete();

    if (scan_status == WIFI_SCAN_RUNNING)
      delay(1);
    else if (scan_status == WIFI_SCAN_FAILED) {
      this->startNextNodeAssignedScan();
      delay(100);
      if (WiFi.scanComplete() == WIFI_SCAN_FAILED)
        Logger::log(WARN_MSG, "WiFi scan failed to start!");
    }
    else {
      this->current_net_count = 0;
      this->current_ble_count = 0;
      this->current_2g4_count = 0;
      this->current_5g_count = 0;

      this->processWardrive(scan_status);

      WiFi.scanDelete();

      if (current_assigned_scan_idx == assigned_start_idx)
        this->scanBLE();

      while(pBLEScan->isScanning())
        delay(1);

      if (current_assigned_scan_idx == assigned_start_idx)
        this->runAdminWindowAfterScanCycle();

      this->startNextNodeAssignedScan();
    }
  }
  else if (this->run_mode == SOLO_MODE) {
    if (gps.getGpsModuleStatus() && sd_obj.supported &&
        (gps.getFixStatus() || this->gps_buffering_enabled)) {
      this->runPromiscuousSolo(currentTime);
    }
  }

  return scan_status;
}

// ============================================================
// Chunk 5: Geofence — Haversine distance, cache loader,
//           and zone check with enter/exit logging
// ============================================================

// Haversine formula — returns great-circle distance in metres.
// Accurate to within ~0.5% which is well within any practical
// geofence radius.
float WiFiOps::haversineDistance(float lat1, float lon1,
                                  float lat2, float lon2) {
  const float R = 6371000.0f; // Earth mean radius, metres
  float phi1  = lat1 * DEG_TO_RAD;
  float phi2  = lat2 * DEG_TO_RAD;
  float dphi  = (lat2 - lat1) * DEG_TO_RAD;
  float dlam  = (lon2 - lon1) * DEG_TO_RAD;

  float a = sinf(dphi / 2.0f) * sinf(dphi / 2.0f) +
            cosf(phi1) * cosf(phi2) *
            sinf(dlam / 2.0f) * sinf(dlam / 2.0f);
  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return R * c;
}

// Parse all geo_0..geo_4 settings into geo_cache[].
// Called once from begin() and on-demand via reloadGeofenceCache().
// An entry is marked valid only when radius > 0 and lat/lon are set.
void WiFiOps::loadGeofenceCache() {
  int valid_count = 0;

  for (int i = 0; i < MAX_GEOFENCES; i++) {
    geo_cache[i].valid = false;

    String geoStr = settings.loadSetting<String>("geo_" + String(i));
    if (geoStr.isEmpty()) continue;

    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, geoStr) != DeserializationError::Ok) {
      Logger::log(WARN_MSG, "[GEO] Bad JSON for geo_" + String(i));
      continue;
    }

    float  lat   = doc["lat"]   | 0.0f;
    float  lon   = doc["lon"]   | 0.0f;
    int    rad   = doc["rad"]   | 0;
    String label = doc["label"] | "";

    if (rad <= 0) continue;           // unconfigured slot
    if (lat == 0.0f && lon == 0.0f) continue; // default placeholder

    geo_cache[i].lat   = lat;
    geo_cache[i].lon   = lon;
    geo_cache[i].rad   = rad;
    geo_cache[i].label = label.isEmpty() ? ("Zone " + String(i + 1)) : label;
    geo_cache[i].valid = true;
    valid_count++;

    Logger::log(GUD_MSG, "[GEO] Loaded zone " + String(i + 1) +
                ": \"" + geo_cache[i].label +
                "\" lat=" + String(lat, 6) +
                " lon=" + String(lon, 6) +
                " rad=" + String((int)(rad * 3.28084)) + "ft");
  }

  geo_cache_loaded = true;
  Logger::log(STD_MSG, "[GEO] Cache loaded: " + String(valid_count) +
              " active zone(s)");
}

// Public: invalidate cache so it reloads on next checkGeofences() call.
// Call this after saving new geofence settings.
void WiFiOps::reloadGeofenceCache() {
  geo_cache_loaded = false;
  Logger::log(STD_MSG, "[GEO] Cache invalidated — will reload on next check");
}

// Check whether the current GPS position falls inside any configured zone.
// Handles enter/exit state transitions with logging and TFT feedback.
// Returns true if inside any zone (wardriving should pause).
bool WiFiOps::checkGeofences(char* dist_str, size_t dist_str_len) {
  // Lazy-load cache if not yet populated
  if (!geo_cache_loaded)
    this->loadGeofenceCache();

  // Quick exit if no zones are configured
  bool any_valid = false;
  for (int i = 0; i < MAX_GEOFENCES; i++) {
    if (geo_cache[i].valid) {
      any_valid = true;
      break;
    }
  }
  if (!any_valid)
    return false;

  if (!gps.getFixStatus())
    return false;

  float cur_lat = gps.getLat().toFloat();
  float cur_lon = gps.getLon().toFloat();

  for (int i = 0; i < MAX_GEOFENCES; i++) {
    if (!geo_cache[i].valid)
      continue;

    float dist = this->haversineDistance(
      cur_lat,
      cur_lon,
      geo_cache[i].lat,
      geo_cache[i].lon);

    if (dist <= (float)geo_cache[i].rad) {
      // Populate caller's distance string if supplied
      if (dist_str && dist_str_len > 0) {
        int dist_ft = (int)(dist * 3.28084f);

        if (dist_ft >= 5280) {
          snprintf(
            dist_str,
            dist_str_len,
            "%.2fmi",
            dist_ft / 5280.0f);
        } else {
          snprintf(
            dist_str,
            dist_str_len,
            "%dft",
            dist_ft);
        }
      }

      // Inside this zone
      bool was_in = this->in_geofence;
      bool label_changed = (this->current_geo_label != geo_cache[i].label);

      if (!was_in || label_changed) {
        this->in_geofence = true;
        this->current_geo_label = geo_cache[i].label;

        Logger::log(
          STD_MSG,
          "[GEO] Entered zone: \"" +
          this->current_geo_label +
          "\"  dist=" +
          String((int)(dist * 3.28084)) +
          "ft  rad=" +
          String((int)(geo_cache[i].rad * 3.28084)) +
          "ft");

        display.clearScreen();
        display.tft->setCursor(0, 0);
        display.tft->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        display.tft->println("GEOFENCE PAUSED");
        display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);

        int dist_ft = (int)(dist * 3.28084f);

        String display_dist =
          (dist_ft >= 5280)
            ? String(dist_ft / 5280.0f, 2) + "mi"
            : String(dist_ft) + "ft";

        display.tft->println(
          this->current_geo_label + " " + display_dist);

        this->geo_display_shown = true;
      }

      return true;
    }
  }

  // Not inside any zone — handle exit transition
  if (this->in_geofence) {
    Logger::log(
      STD_MSG,
      "[GEO] Exited zone: \"" +
      this->current_geo_label +
      "\" — resuming wardrive");

    this->in_geofence = false;
    this->current_geo_label = "";
    this->geo_display_shown = false;

    display.clearScreen();
  }

  return false;
}

// ============================================================
// Chunk 4: SSID exclusion helper
// Checks a sanitized SSID against the caller's pre-loaded
// exclusion list. Empty exclusion slots are skipped.
// Comparison is case-sensitive (SSIDs are case-sensitive).
// ============================================================
bool WiFiOps::isSSIDExcluded(const String& ssid,
                              const String* list, int count) {
  if (ssid.isEmpty()) return false;
  for (int i = 0; i < count; i++) {
    if (!list[i].isEmpty() && list[i] == ssid)
      return true;
  }
  return false;
}

static long long gbufDaysFromCivil(int y, int m, int d) {
  y -= m <= 2;
  long long era = (y >= 0 ? y : y - 399) / 400;
  int yoe = (int)(y - era * 400);
  int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

static long long gbufToEpoch(int y, int mo, int d, int h, int mi, int s) {
  return gbufDaysFromCivil(y, mo, d) * 86400LL + h * 3600 + mi * 60 + s;
}

static String gbufEpochToStr(long long t) {
  long long days = t / 86400;
  long long rem = t % 86400;
  if (rem < 0) { rem += 86400; days -= 1; }
  int h = (int)(rem / 3600), mi = (int)((rem % 3600) / 60), s = (int)(rem % 60);
  long long z = days + 719468;
  long long era = (z >= 0 ? z : z - 146096) / 146097;
  int doe = (int)(z - era * 146097);
  int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int y = yoe + (int)(era * 400);
  int doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  int mp = (5 * doy + 2) / 153;
  int d = doy - (153 * mp + 2) / 5 + 1;
  int mo = mp + (mp < 10 ? 3 : -9);
  y += (mo <= 2);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
  return String(buf);
}

int WiFiOps::gpsBufferWindowMin() {
  int w = settings.loadSetting<int>(GPS_BUFFER_WINDOW_NAME);
  if (w < GPS_BUFFER_WINDOW_MIN_MIN || w > GPS_BUFFER_WINDOW_MIN_MAX)
    return GPS_BUFFER_WINDOW_DEFAULT_MIN;
  return w;
}

int WiFiOps::logKeepCount() {
  int n = settings.loadSetting<int>(LOG_KEEP_NAME);
  if (n < LOG_KEEP_MIN || n > LOG_KEEP_MAX)
    return LOG_KEEP_DEFAULT;
  return n;
}

bool WiFiOps::logFullySynced(String logPath) {
  bool wigle_cfg = !settings.loadSetting<String>("wu").isEmpty() &&
                   !settings.loadSetting<String>("wt").isEmpty();
  bool wdg_cfg   = !settings.loadSetting<String>(WDG_KEY_NAME).isEmpty();

  if (!wigle_cfg && !wdg_cfg) return false;
  if (wigle_cfg && !this->sidecarExists(logPath, "wigle")) return false;
  if (wdg_cfg   && !this->sidecarExists(logPath, "wdg"))   return false;
  return true;
}

void WiFiOps::pruneOldLogs() {
  if (!sd_obj.supported) return;

  int keep = this->logKeepCount();

  std::vector<String> logs;
  File root = SD.open("/");
  if (!root || !root.isDirectory()) return;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (f.isDirectory()) continue;
    String n = f.name();
    if (n.endsWith(".log") && n != "debug.log")
      logs.push_back(n);
  }
  root.close();

  if ((int)logs.size() <= keep) return;

  std::sort(logs.begin(), logs.end());   // ascending = oldest first

  String active = buffer.getFileName();  // e.g. "/wigle-..._0.log"
  int candidates = (int)logs.size() - keep;
  int pruned = 0, kept_unsynced = 0;

  for (int i = 0; i < candidates; i++) {
    String path = "/" + logs[i];
    if (active.length() && path == active) continue;    // never touch the active log
    if (!this->logFullySynced(path)) { kept_unsynced++; continue; }

    SD.remove(path);
    SD.remove(scPath(path, "wigle"));
    SD.remove(scPath(path, "wdg"));
    pruned++;
  }

  Logger::log(GUD_MSG, "[RETAIN] keep=" + String(keep) + " logs=" + String((int)logs.size()) +
              " pruned=" + String(pruned) + " keptUnsynced=" + String(kept_unsynced));
}

void WiFiOps::backfillPending() {
  if (!SD.exists("/pending.csv")) return;

  int ry, rmo, rd, rh, rmi, rs;
  if (sscanf(gps.getDatetime().c_str(), "%d-%d-%d %d:%d:%d",
             &ry, &rmo, &rd, &rh, &rmi, &rs) != 6) return;
  long long reacq_epoch  = gbufToEpoch(ry, rmo, rd, rh, rmi, rs);
  uint32_t  reacq_millis = millis();
  double    lat1 = gps.getLat().toDouble();
  double    lon1 = gps.getLon().toDouble();
  String    alt1 = String(gps.getAlt(), 2);
  uint32_t  window_ms = (uint32_t)this->gpsBufferWindowMin() * 60u * 1000u;
  bool bracketed = this->have_last_fix && (reacq_millis - this->last_fix_millis) <= window_ms;

  File in = SD.open("/pending.csv", FILE_READ);
  if (!in) return;

  int written = 0, dropped = 0;
  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);
    int c5 = line.indexOf(',', c4 + 1);
    int c6 = line.indexOf(',', c5 + 1);
    if (c6 < 0) continue;

    uint32_t rowMillis = (uint32_t)strtoul(line.substring(0, c1).c_str(), NULL, 10);
    String bssid = line.substring(c1 + 1, c2);
    String ssid  = line.substring(c2 + 1, c3);
    String auth  = line.substring(c3 + 1, c4);
    String ch    = line.substring(c4 + 1, c5);
    String rssi  = line.substring(c5 + 1, c6);
    String type  = line.substring(c6 + 1);

    uint32_t age_ms = reacq_millis - rowMillis;
    if (age_ms > window_ms) { dropped++; continue; }
    uint32_t age_s = age_ms / 1000;

    String firstSeen = gbufEpochToStr(reacq_epoch - (long long)age_s);

    double lat, lon;
    uint32_t edge_s;
    if (bracketed) {
      double span = (double)(reacq_millis - this->last_fix_millis);
      double frac = span > 0 ? (double)(rowMillis - this->last_fix_millis) / span : 1.0;
      if (frac < 0) frac = 0;
      if (frac > 1) frac = 1;
      lat = this->last_fix_lat + frac * (lat1 - this->last_fix_lat);
      lon = this->last_fix_lon + frac * (lon1 - this->last_fix_lon);
      uint32_t toLast = (rowMillis - this->last_fix_millis) / 1000;
      edge_s = age_s < toLast ? age_s : toLast;
    } else {
      lat = lat1;
      lon = lon1;
      edge_s = age_s;
    }
    float acc = 5.0 + (float)edge_s * GPS_BACKFILL_ACC_PER_S;
    if (acc > GPS_ACCURACY_UNKNOWN) acc = GPS_ACCURACY_UNKNOWN;

    String out = bssid + "," + ssid + "," + auth + "," + firstSeen + "," + ch + "," +
                 rssi + "," + String(lat, 7) + "," + String(lon, 7) + "," + alt1 + "," +
                 String(acc, 2) + "," + type;
    buffer.append(out + "\n");
    written++;
    if ((written % 20) == 0) delay(1);
  }
  in.close();
  // Preserve the buffered set for diagnosis instead of destroying it — pending.csv
  // is otherwise gone after backfill and can't be inspected.
  SD.remove("/pending-last.csv");
  if (!SD.rename("/pending.csv", "/pending-last.csv"))
    SD.remove("/pending.csv");
  this->pending_count = 0;
  Logger::log(GUD_MSG, "[GBUF] Backfilled " + String(written) + ", dropped " + String(dropped));
}

void WiFiOps::bufferPendingDetection(const String& line) {
  File f = SD.open("/pending.csv", FILE_APPEND);
  if (!f) {
    Logger::log(WARN_MSG, "[GBUF] Could not open /pending.csv");
    return;
  }
  f.println(line);
  f.close();
}

// Queue a BLE hit seen without a GPS fix. Same pending format as WiFi
// (millis,BSSID,SSID,auth,ch,RSSI,TYPE) so backfillPending() stamps it on
// reacquire: BSSID=address, SSID empty, auth=[BLE], ch=0, TYPE=BLE.
bool WiFiOps::effectiveFix() {
  return gps.getFixStatus() && !this->sim_no_fix;
}

void WiFiOps::toggleSimNoFix() {
  this->sim_no_fix = !this->sim_no_fix;
  Logger::log(GUD_MSG, String("[GBUF] SIM GPS loss ") +
              (this->sim_no_fix ? "ON — buffering (fix hidden)" : "OFF — fix restored"));
}

void WiFiOps::bufferBleDetection(const String& address, int rssi) {
  String pend = String(millis()) + "," + address + ",,[BLE],0," + String(rssi) + ",BLE";
  this->bufferPendingDetection(pend);
  this->pending_count++;
  Logger::log(GUD_MSG, "[GBUF] BLE buffered " + address + " (pending " +
              String(this->pending_count) + ")");
}

void WiFiOps::loadExclusionCache() {
  this->excl_cache_count = 0;
  for (int e = 0; e < MAX_SSID_EXCLUSIONS; e++) {
    this->excl_cache[e] = settings.loadSetting<String>("sx_" + String(e));
    if (!this->excl_cache[e].isEmpty()) this->excl_cache_count++;
  }
}

void WiFiOps::logWardriveAP(uint8_t* bssid_raw, const String& ssid_in, int channel, int rssi, int authtype) {
  if (this->seen_mac(bssid_raw))
    return;

  char this_bssid[18] = {0};
  sprintf(this_bssid, "%02X:%02X:%02X:%02X:%02X:%02X", bssid_raw[0], bssid_raw[1], bssid_raw[2], bssid_raw[3], bssid_raw[4], bssid_raw[5]);
  String bssid_str = this_bssid;

  if (this->run_mode == SOLO_MODE) {
    digitalWrite(LED_PIN, HIGH);

    String ssid = ssid_in;
    ssid.replace(",","_");

    if (this->excl_cache_count > 0 &&
        this->isSSIDExcluded(ssid, this->excl_cache, MAX_SSID_EXCLUSIONS)) {
      Logger::log(STD_MSG, "[EXCL] Skipping excluded SSID: \"" + ssid + "\"");
      digitalWrite(LED_PIN, LOW);
      return;
    }

    String display_string = "";
    if (ssid != "") {
      display_string.concat(ssid);
    }
    else {
      display_string.concat(bssid_str);
    }

    bool do_save = false;
    bool did_record = false;
    if (this->effectiveFix()) {
      do_save = !gps.getDatetime().isEmpty();
      display_string.concat(" | Lt: " + gps.getLat());
      display_string.concat(" | Ln: " + gps.getLon());
    }
    else {
      if (this->gps_buffering_enabled) {
        String pend = String(millis()) + "," + bssid_str + "," + ssid + "," +
                      this->security_int_to_string(authtype) + "," +
                      (String)channel + "," + (String)rssi + ",WIFI";
        this->bufferPendingDetection(pend);
        this->save_mac(bssid_raw);
        this->pending_count++;
        did_record = true;
        display_string.concat(" | GPS: No Fix [BUF:" + String(this->pending_count) + "]");
      }
      else {
        display_string.concat(" | GPS: No Fix");
      }
    }

    int temp_len = display_string.length();

    #ifdef HAS_SCREEN
      for (int k = 0; k < 40 - temp_len; k++)
      {
        display_string.concat(" ");
      }
    #endif

    String wardrive_line = bssid_str + "," + ssid + "," + this->security_int_to_string(authtype) + "," + gps.getDatetime() + "," + (String)channel + "," + (String)rssi + "," + gps.getLat() + "," + gps.getLon() + "," + gps.getAlt() + "," + gps.getAccuracy() + ",WIFI";
    Logger::log(GUD_MSG, (String)this->mac_history_cursor + " | " + wardrive_line);

    digitalWrite(LED_PIN, LOW);

    if (do_save) {
      this->save_mac(bssid_raw);
      buffer.append(wardrive_line + "\n");
      did_record = true;
    }

    if (did_record) {
      this->setCurrentNetCount(this->getCurrentNetCount() + 1);
      this->setTotalNetCount(this->getTotalNetCount() + 1);
      if (channel > 14)
        this->setCurrent5gCount(this->getCurrent5gCount() + 1);
      else
        this->setCurrent2g4Count(this->getCurrent2g4Count() + 1);

      if (bssid_raw[0] == CRADLEPOINT_OUI0 && bssid_raw[1] == CRADLEPOINT_OUI1 && bssid_raw[2] == CRADLEPOINT_OUI2)
        this->noteFuzzHit(FUZZ_LEO, ssid.length() ? ssid : String("Cradlepoint"), rssi, bssid_raw);
      else if (bssid_raw[0] == AXON_OUI0 && bssid_raw[1] == AXON_OUI1 && bssid_raw[2] == AXON_OUI2)
        this->noteFuzzHit(FUZZ_LEO, ssid.length() ? ssid : String("Axon?"), rssi, bssid_raw);
    }
  }
  else if (this->run_mode == NODE_MODE) {
    this->save_mac(bssid_raw);
    String ssid = ssid_in;
    ssid.replace(",","_");

    if (this->excl_cache_count > 0 &&
        this->isSSIDExcluded(ssid, this->excl_cache, MAX_SSID_EXCLUSIONS)) {
      Logger::log(STD_MSG, "[EXCL] Node skipping excluded SSID: \"" + ssid + "\"");
      return;
    }

    String enow_line = bssid_str + "," + ssid + "," + this->security_int_to_string(authtype) + "," + (String)channel + "," + (String)rssi + ",W";
    Logger::log(GUD_MSG, (String)this->mac_history_cursor + " | " + enow_line);
    if (this->use_encryption)
      this->sendEncryptedStringToCore(enow_line);
    else
      this->sendBroadcastStringPlain(enow_line);
  }
}

bool WiFiOps::fuzzLeoSibling(const uint8_t* mac) {
  for (uint8_t i = 0; i < this->fuzz_leo_mem_count; i++) {
    uint8_t* s = this->fuzz_leo_macs[i];
    if (s[0] == mac[0] && s[1] == mac[1] && s[2] == mac[2] && s[3] == mac[3] && s[4] == mac[4]) {
      int d = (int)s[5] - (int)mac[5];
      if (d < 0) d = -d;
      if (d != 0 && d <= FUZZ_SIBLING_DELTA) return true;
    }
  }
  return false;
}

void WiFiOps::fuzzRememberLeo(const uint8_t* mac) {
  for (uint8_t i = 0; i < this->fuzz_leo_mem_count; i++)
    if (memcmp(this->fuzz_leo_macs[i], mac, 6) == 0) return;
  memcpy(this->fuzz_leo_macs[this->fuzz_leo_mem_cursor % FUZZ_LEO_MEM], mac, 6);
  this->fuzz_leo_mem_cursor++;
  if (this->fuzz_leo_mem_count < FUZZ_LEO_MEM) this->fuzz_leo_mem_count++;
}

void WiFiOps::noteFuzzHit(uint8_t cat, const String& label, int rssi, const uint8_t* mac) {
  if (cat == FUZZ_LEO && mac != nullptr) {
    bool sibling = this->fuzzLeoSibling(mac);
    this->fuzzRememberLeo(mac);
    if (sibling) return;
  }
  if (cat == FUZZ_CAM) this->fuzz_cam_count++;
  else if (cat == FUZZ_LEO) this->fuzz_leo_count++;
  else return;
  this->fuzz_last_cat  = cat;
  this->fuzz_last_rssi = rssi;
  this->fuzz_last_ms   = millis();
  String l = label;
  l.replace("\n", " ");
  l.replace("\r", " ");
  strncpy(this->fuzz_last_label, l.c_str(), sizeof(this->fuzz_last_label) - 1);
  this->fuzz_last_label[sizeof(this->fuzz_last_label) - 1] = '\0';
  Logger::log(GUD_MSG, "[FUZZ] " + String(cat == FUZZ_CAM ? "CAM " : "FLEET ") + label + " " + String(rssi));
}

void WiFiOps::processWardrive(uint16_t networks) {
  if (this->effectiveFix() && !gps.getDatetime().isEmpty()) {
    if (this->gps_buffering_enabled)
      this->backfillPending();
    this->last_fix_lat    = gps.getLat().toDouble();
    this->last_fix_lon    = gps.getLon().toDouble();
    this->last_fix_millis = millis();
    this->have_last_fix   = true;
  }

  this->loadExclusionCache();

  // Process results if networks found
  if (networks > 0) {
    for (int i = 0; i < networks; i++) {
      this->logWardriveAP(WiFi.BSSID(i), WiFi.SSID(i), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i));
    }
  }

  if (this->run_mode == SOLO_MODE)
    digitalWrite(LED_PIN, LOW);
}

uint16_t WiFiOps::dwellForChannel(uint8_t ch) {
  if (ch == 1 || ch == 6 || ch == 11) return 200;
  if (ch <= 14) return 40;
  if ((ch >= 36 && ch <= 48) || (ch >= 149 && ch <= 165)) return 100;
  return 40;
}

void WiFiOps::startPromiscuousCapture() {
  if (this->promisc_started) return;

  if (g_ap_queue == nullptr)
    g_ap_queue = xQueueCreate(AP_QUEUE_LEN, sizeof(promisc_ap_t));

  esp_wifi_set_ps(WIFI_PS_NONE);

  wifi_promiscuous_filter_t filt = {};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  esp_wifi_set_promiscuous(true);

  this->promisc_started = true;
}

void WiFiOps::stopPromiscuousCapture() {
  if (!this->promisc_started) return;
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_promiscuous_rx_cb(nullptr);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  this->promisc_started = false;
}

void WiFiOps::runPromiscuousSolo(uint32_t currentTime) {
  if (millis() - this->last_ble_ms >= BLE_SCAN_INTERVAL_MS) {
    this->last_ble_ms = millis();
    this->stopPromiscuousCapture();
    this->scanBLE();
    while (pBLEScan->isScanning())
      delay(1);
  }

  this->startPromiscuousCapture();

  if (this->dwell_idx == 0) {
    this->current_net_count = 0;
    this->current_ble_count = 0;
    this->current_2g4_count = 0;
    this->current_5g_count = 0;
    this->trig_found_sweep = false;

    if (this->effectiveFix() && !gps.getDatetime().isEmpty()) {
      if (this->gps_buffering_enabled)
        this->backfillPending();
      this->last_fix_lat    = gps.getLat().toDouble();
      this->last_fix_lon    = gps.getLon().toDouble();
      this->last_fix_millis = millis();
      this->have_last_fix   = true;
    }
  }

  uint8_t ch = scan_channels[this->dwell_idx];
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);

  delay(WiFiOps::dwellForChannel(ch));

  this->loadExclusionCache();

  promisc_ap_t r;
  while (xQueueReceive(g_ap_queue, &r, 0) == pdTRUE) {
    if (r.kind == 1) {
      if (!this->seen_mac(r.bssid)) {
        this->save_mac(r.bssid);
        this->noteFuzzHit(FUZZ_CAM, "Flock?", (int)r.rssi, nullptr);
        char fm[18];
        sprintf(fm, "%02X:%02X:%02X:%02X:%02X:%02X", r.bssid[0], r.bssid[1], r.bssid[2], r.bssid[3], r.bssid[4], r.bssid[5]);
        Logger::log(GUD_MSG, "[FLOCK] probe " + String(fm) + " ch" + String((int)r.channel) + " " + (String)(int)r.rssi + "dBm " + gps.getLat() + "," + gps.getLon());
      }
      continue;
    }
    String ssid = String(r.ssid);
    int di = this->matchDockSSID(ssid);
    if (di >= 0) {
      this->trig_found_sweep = true;
      if ((int)r.rssi > this->trig_best_rssi_sweep) {
        this->trig_best_rssi_sweep = (int)r.rssi;
        this->dock_matched_idx = di;
      }
    }
    this->logWardriveAP(r.bssid, ssid, (int)r.channel, (int)r.rssi, (int)r.auth);
  }

  this->dwell_idx++;
  if (this->dwell_idx < NUM_SCAN_CHANNELS)
    return;

  this->dwell_idx = 0;

  if (this->anyDockConfigured()) {
    if (!rtc_dock_done &&
        this->armDock(this->trig_found_sweep, this->trig_best_rssi_sweep)) {
      Logger::log(STD_MSG, "[DOCK] Docking network confirmed (armed) — docking: " + this->dockSSID(this->dock_matched_idx));
      this->dock_state            = DOCK_STATE_CONNECTING;
      this->dock_connect_attempts = 0;
      this->trig_found_sweep      = false;
      this->trig_best_rssi_sweep  = -127;
      return;
    }

    if (rtc_dock_done) {
      if (this->trig_found_sweep) {
        this->dock_rearm_absent = 0;
      } else if (currentTime - this->dock_rearm_last_check >= DOCK_SCAN_INTERVAL) {
        this->dock_rearm_last_check = currentTime;
        this->dock_rearm_absent++;
        Logger::log(STD_MSG, "[DOCK] Trigger absent while docked (" +
                    String(this->dock_rearm_absent) + "/" +
                    String(DOCK_DEPART_SCANS) + ")");
        if (this->dock_rearm_absent >= DOCK_DEPART_SCANS) {
          rtc_dock_done = false;
          this->dock_rearm_absent = 0;
          Logger::log(STD_MSG, "[DOCK] Departed — dock re-armed");
        }
      }
    }
  }
  this->trig_found_sweep = false;
  this->trig_best_rssi_sweep = -127;
}

static inline uint32_t bloom_hash_a(const unsigned char* mac) {
  uint32_t h = 2166136261u;
  for (int i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
  return h;
}

static inline uint32_t bloom_hash_b(const unsigned char* mac) {
  uint32_t h = 0;
  for (int i = 0; i < 6; i++) { h += mac[i]; h += h << 10; h ^= h >> 6; }
  h += h << 3; h ^= h >> 11; h += h << 15;
  return h | 1u;
}

bool WiFiOps::seen_mac(unsigned char* mac) {
  uint32_t a = bloom_hash_a(mac);
  uint32_t b = bloom_hash_b(mac);
  for (int i = 0; i < BLOOM_HASHES; i++) {
    uint32_t bit = (a + (uint32_t)i * b) % BLOOM_BITS;
    if (!(this->bloom_bits[bit >> 3] & (1u << (bit & 7))))
      return false;
  }
  return true;
}

void WiFiOps::save_mac(unsigned char* mac) {
  uint32_t a = bloom_hash_a(mac);
  uint32_t b = bloom_hash_b(mac);
  for (int i = 0; i < BLOOM_HASHES; i++) {
    uint32_t bit = (a + (uint32_t)i * b) % BLOOM_BITS;
    this->bloom_bits[bit >> 3] |= (1u << (bit & 7));
  }
  this->mac_history_cursor++;
}

String WiFiOps::security_int_to_string(int security_type) {
  //Provide a security type int from WiFi.encryptionType(i) to convert it to a String which Wigle CSV expects.
  String authtype = "";

  switch (security_type) {
    case WIFI_AUTH_OPEN:
      authtype = "[OPEN]";
      break;
  
    case WIFI_AUTH_WEP:
      authtype = "[WEP]";
      break;
  
    case WIFI_AUTH_WPA_PSK:
      authtype = "[WPA_PSK]";
      break;
  
    case WIFI_AUTH_WPA2_PSK:
      authtype = "[WPA2_PSK]";
      break;
  
    case WIFI_AUTH_WPA_WPA2_PSK:
      authtype = "[WPA_WPA2_PSK]";
      break;
  
    case WIFI_AUTH_WPA2_ENTERPRISE:
      authtype = "[WPA2]";
      break;

    //Requires at least v2.0.0 of https://github.com/espressif/arduino-esp32/
    case WIFI_AUTH_WPA3_PSK:
      authtype = "[WPA3_PSK]";
      break;

    case WIFI_AUTH_WPA2_WPA3_PSK:
      authtype = "[WPA2_WPA3_PSK]";
      break;

    case WIFI_AUTH_WAPI_PSK:
      authtype = "[WAPI_PSK]";
      break;
        
    default:
      authtype = "[UNDEFINED]";
  }

  return authtype;
}

void WiFiOps::clearMacHistory() {
    memset(this->bloom_bits, 0, sizeof(this->bloom_bits));
    this->mac_history_cursor = 0;
}

void WiFiOps::startLog(String file_name) {
  // Build WiGLE-convention filename: wigle-YYYY-MM-DDTHHMMSS+0000.log
  // Falls back to base file_name if GPS has no fix yet
  String timestamped_name = file_name;
  if (gps.getFixStatus()) {
    // Pull raw components from GPS (MicroNMEA provides these individually)
    // We build the filename ourselves to get proper zero-padding
    char fname[48];
    // gps.getDatetime() returns "YYYY-M-D H:MM:SS" — not suitable directly,
    // so we reconstruct from the wardrive line fields via the buffer datetime.
    // For now use the formatted datetime string and reformat it.
    // Format stored is "YYYY-M-D H:MM:SS" — parse and reformat.
    String dt = gps.getDatetime(); // e.g. "2026-5-1 13:34:37"
    if (dt.length() >= 10) {
      // Extract fields robustly
      int y = dt.substring(0, 4).toInt();
      int mo = dt.substring(5, dt.indexOf('-', 5)).toInt();
      int rest = dt.indexOf('-', 5);
      int d  = dt.substring(rest + 1, dt.indexOf(' ')).toInt();
      int sp = dt.indexOf(' ');
      int h  = dt.substring(sp + 1, dt.indexOf(':', sp)).toInt();
      int c1 = dt.indexOf(':', sp);
      int mi = dt.substring(c1 + 1, dt.indexOf(':', c1 + 1)).toInt();
      int c2 = dt.indexOf(':', c1 + 1);
      int s  = dt.substring(c2 + 1).toInt();
      snprintf(fname, sizeof(fname), "wigle-%04d-%02d-%02dT%02d%02d%02d+0000",
               y, mo, d, h, mi, s);
      timestamped_name = String(fname);
    }
  }

  buffer.logOpen(
    timestamped_name,
    #if defined(HAS_SD)
      sd_obj.supported ? &SD :
    #endif
    NULL,
    false
  );

  String header_line = "WigleWifi-1.4,appRelease=" + (String)FIRMWARE_VERSION +
                       ",model=" + (String)DEVICE_NAME +
                       ",release=" + (String)FIRMWARE_VERSION +
                       ",device=" + (String)DEVICE_NAME +
                       ",display=SPI TFT,board=ESP32-C5-DevKit,brand=Dark3D\n"
                       "MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,"
                       "CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type";
  buffer.append(header_line + "\n");
  Logger::log(GUD_MSG, "Log started: " + timestamped_name + ".log");
}

void WiFiOps::initWiFi(bool set_country) {
  if (set_country) {
    Logger::log(STD_MSG, "Setting country code...");
    esp_wifi_init(&cfg);
    esp_wifi_set_country(&country);
  }

  WiFi.STA.begin();
  WiFi.setBandMode(WIFI_BAND_MODE_AUTO);
  delay(100);
}

void WiFiOps::deinitWiFi() {
  this->stopPromiscuousCapture();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);

}

void WiFiOps::deinitBLE() {
  // Dock-mode crash fix (ESP32-C5 + NimBLE 2.3.0).
  // This is called only from the dock path, when a dock is triggered mid-wardrive.
  // Full BLE stack teardown is unnecessary, and was causing C5 to crash.
  // We only STOP the scan to free the radio for the STA connection/upload.
  // Stack stays alive and wardriving scanBLE() restarts scanning after the dock undocks.
  if (pBLEScan != nullptr && pBLEScan->isScanning()) {
    Logger::log(STD_MSG, "Stopping BLE scan for dock...");
    pBLEScan->stop();
    unsigned long start = millis();
    while (pBLEScan->isScanning() && millis() - start < 3000) {
      delay(10);
      esp_task_wdt_reset(); // feed watchdog — BLE stop can take a bit of time
    }
    pBLEScan->clearResults();
  }
  Logger::log(STD_MSG, "BLE scan stopped (stack kept initialized)");
}

void WiFiOps::initBLE() {
  // The dock path no longer fully tears down BLE (see deinitBLE), so the
  // stack can still be up when departDock() calls this to resume wardriving.
  // no-op if BLE is already initialized.
  if (ble_initialized) return;

  NimBLEDevice::init("");
  //delete pBLEScan;
  pBLEScan = NimBLEDevice::getScan();

  pBLEScan->setScanCallbacks(new scanCallbacks(), false);
  pBLEScan->setActiveScan(true);
  pBLEScan->setDuplicateFilter(false);       // Disables internal filtering based on MAC
  pBLEScan->setMaxResults(0);                // Prevent storing results in NimBLEScanResults
  ble_initialized = true;
}

bool WiFiOps::tryConnectToWiFi(unsigned long timeoutMs) {

  display.clearScreen();
  display.tft->setCursor(0, 0);
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Check if file exists
  if (!SPIFFS.exists(WIFI_CONFIG)) {
    Logger::log(WARN_MSG, "No saved WiFi config found.");
    return false;
  }

  display.tft->print("Joining WiFi: ");

  this->user_ap_ssid = settings.loadSetting<String>("s");
  this->user_ap_password = settings.loadSetting<String>("p");
  this->wigle_user = settings.loadSetting<String>("wu");
  this->wigle_token = settings.loadSetting<String>("wt");

  Logger::log(STD_MSG, "Attempting to connect with: ");
  Logger::log(STD_MSG, this->user_ap_ssid);
  display.tft->print(this->user_ap_ssid);
  display.tft->println("...");

  // Connect to WiFi with AP credentials
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setBandMode(WIFI_BAND_MODE_AUTO);
  WiFi.begin(this->user_ap_ssid.c_str(), this->user_ap_password.c_str());

  // Wait while we connect
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(500);
    Serial.print(".");
  }

  // Output status of connection attempt
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    Logger::log(STD_MSG, "[WIFI] associated rssi=" + String(WiFi.RSSI()) +
                " ch=" + String(WiFi.channel()) + " (ch>14 = 5GHz)");
    display.clearScreen();
    display.tft->setCursor(0, 0);
    display.tft->print("Connected: ");
    display.tft->println(this->user_ap_ssid);
    display.tft->print("IP: ");
    display.tft->println(WiFi.localIP());
    Logger::log(GUD_MSG, "WiFi connected!");
    Logger::log(GUD_MSG, "IP address: " + WiFi.localIP().toString());
    return true;
  } else {
    Logger::log(WARN_MSG, "Failed to connect to WiFi.");
    display.tft->println("Failed to connect");
    WiFi.disconnect(true);
    return false;
  }
}

void WiFiOps::startAccessPoint() {
  display.clearScreen();
  display.tft->setCursor(0, 0);
  display.tft->print("Starting AP: ");
  display.tft->println(this->apSSID);
  WiFi.softAP(this->apSSID, this->apPassword);
  Logger::log(GUD_MSG, "Access Point started");
  Logger::log(GUD_MSG, "IP: ");
  Logger::log(GUD_MSG, WiFi.softAPIP().toString());
  display.tft->print("IP: ");
  display.tft->println(WiFi.softAPIP().toString());
}

bool WiFiOps::wigleUpload(String filePath) {
  //server.begin();

  delay(100);

  if (!SD.exists(filePath)) {
    Logger::log(WARN_MSG, "File does not exist: " + filePath);
    return false;
  }

  File fileToUpload = SD.open(filePath);
  if (!fileToUpload) {
    Logger::log(WARN_MSG, "Could not open file: " + filePath);
    return false;
  }

  // Load credentials
  String username = settings.loadSetting<String>("wu");
  String token = settings.loadSetting<String>("wt");
  if (username.isEmpty() || token.isEmpty()) {
    fileToUpload.close();
    Logger::log(WARN_MSG, "Missing wigle credentials");
    return false;
  }

  Logger::log(STD_MSG, "Username: " + username);
  Logger::log(STD_MSG, "Token: " + token);

  String boundary = "----ESP32BOUNDARY";
  String contentType = "multipart/form-data; boundary=" + boundary;

  // Build parts
  String part1 = "--" + boundary + "\r\n";
  part1 += "Content-Disposition: form-data; name=\"file\"; filename=\"" + filePath + "\"\r\n";
  part1 += "Content-Type: application/octet-stream\r\n\r\n";

  String part2 = "\r\n--" + boundary + "\r\n";
  part2 += "Content-Disposition: form-data; name=\"donate\"\r\n\r\non\r\n";

  String part3 = "--" + boundary + "--\r\n";

  int totalLength = part1.length() + fileToUpload.size() + part2.length() + part3.length();

  Logger::log(STD_MSG, "part1.length(): " + String(part1.length()));
  Logger::log(STD_MSG, "fileToUpload.size(): " + String(fileToUpload.size()));
  Logger::log(STD_MSG, "part2.length(): " + String(part2.length()));
  Logger::log(STD_MSG, "part3.length(): " + String(part3.length()));
  Logger::log(STD_MSG, "Total Content-Length: " + String(totalLength));

  Serial.print("File size: ");
  Serial.println(fileToUpload.size());

  // Connect manually via WiFiClientSecure
  //WiFiClientSecure *client = new WiFiClientSecure();
  //client.stop();
  client->setInsecure();
  client->setTimeout(5000);

  this->freeTlsGuard();

  Logger::log(STD_MSG, "[HEAP] wigle pre-connect free=" + String(ESP.getFreeHeap()) +
              " maxAlloc=" + String(ESP.getMaxAllocHeap()));

  if (!client->connect("api.wigle.net", 443, 8000)) {
    fileToUpload.close();
    //delete client;
    client->stop();
    this->reserveTlsGuard();
    Logger::log(WARN_MSG, "Failed to connected to api.wigle.net");
    return false;
  }

  Serial.println("Connected");
  client->setNoDelay(true);
  Logger::log(STD_MSG, "[WIGLE] TLS connected=" + String(client->connected()) +
              " wifi=" + String(WiFi.status()) + " rssi=" + String(WiFi.RSSI()) +
              " nodelay=" + String(client->getNoDelay()));

  // Compose headers
  String auth = utils.base64Encode(username + ":" + token);

  Serial.println("Finished encoding");

  client->println("POST /api/v2/file/upload HTTP/1.1");
  client->println("Host: api.wigle.net");
  client->println("User-Agent: ESP32Uploader/1.0");
  client->println("Accept: application/json");
  client->println("Authorization: Basic " + auth);
  client->println("Content-Type: " + contentType);
  client->print("Content-Length: ");
  client->println(totalLength);
  client->println();
  delay(100);

  Serial.println("Finished sending header");

  // Send body
  client->print(part1);
  const size_t BUFFER_SIZE = 4096;
  uint8_t buffer[BUFFER_SIZE];

  Serial.println("Finished sending part1");
  {
    char errbuf[100] = {0};
    int le = client->lastError(errbuf, sizeof(errbuf));
    Logger::log(STD_MSG, "[WIGLE] post-headers connected=" + String(client->connected()) +
                " avail=" + String(client->available()) + " wifi=" + String(WiFi.status()) +
                " writeErr=" + String(client->getWriteError()) +
                " tlsErr=" + String(le) + " (" + String(errbuf) + ")");
  }

  uint8_t percent_sent = 0;
  int wpb_last = -1;

  String display_percent = "";

  display.tft->setTextSize(1);
  display.tft->fillRect(0, 10, TFT_WIDTH, 28, ST77XX_BLACK);
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->setCursor(0, 12);
  display.tft->print("wigle uploading");
  display.tft->setCursor(0, 24);
  {
    String fn = filePath;
    if (fn.startsWith("/")) fn = fn.substring(1);
    if (fn.length() > 26) fn = fn.substring(0, 26);
    display.tft->print(fn);
  }

  size_t fileSize = fileToUpload.size();
  Logger::log(STD_MSG, "[WIGLE][REQ] POST api.wigle.net/api/v2/file/upload CL=" +
              String(totalLength) + " fileSize=" + String((unsigned)fileSize) +
              " authLen=" + String(auth.length()) + " part1=" + String(part1.length()) +
              " heapFree=" + String(ESP.getFreeHeap()) +
              " maxAlloc=" + String(ESP.getMaxAllocHeap()));

  size_t totalBytesSent = 0;
  size_t shortWrites = 0;
  bool sendConnLost = false;
  int earlyAvail = 0;
  int lastDecile = -1;
  uint32_t sendStart = millis();
  while (fileToUpload.available()) {
    size_t bytesRead = fileToUpload.read(buffer, BUFFER_SIZE);
    size_t w = client->write(buffer, bytesRead);
    totalBytesSent += bytesRead;
    if (w != bytesRead) {
      shortWrites++;
      Logger::log(WARN_MSG, "[WIGLE][TX] SHORT WRITE " + String((unsigned)w) + "/" +
                  String((unsigned)bytesRead) + " @off " + String((unsigned)totalBytesSent) +
                  " connected=" + String(client->connected()));
    }
    if (!client->connected() && !sendConnLost) {
      sendConnLost = true;
      Logger::log(WARN_MSG, "[WIGLE][TX] SOCKET DROPPED mid-send @off " +
                  String((unsigned)totalBytesSent));
    }
    if (!client->connected()) {
      char eb[100] = {0};
      int le = client->lastError(eb, sizeof(eb));
      Logger::log(WARN_MSG, "[WIGLE][TX] ABORT — socket dead @off " +
                  String((unsigned)totalBytesSent) + "/" + String((unsigned)fileSize) +
                  " tlsErr=" + String(le) + " (" + String(eb) + ") writeErr=" +
                  String(client->getWriteError()) + " wifi=" + String(WiFi.status()));
      break;
    }
    int a = client->available();
    if (a > 0 && earlyAvail == 0) {
      earlyAvail = a;
      Logger::log(WARN_MSG, "[WIGLE][TX] peer sent " + String(a) +
                  "B mid-send @off " + String((unsigned)totalBytesSent) + " (early response?)");
    }
    percent_sent = fileSize ? (totalBytesSent * 100) / fileSize : 100;
    if ((int)percent_sent / 10 != lastDecile) {
      lastDecile = (int)percent_sent / 10;
      Logger::log(STD_MSG, "[WIGLE][TX] " + String(percent_sent) + "% " +
                  String((unsigned)(totalBytesSent/1024)) + "KB conn=" +
                  String(client->connected()) + " heap=" + String(ESP.getFreeHeap()));
    }
    if ((int)percent_sent != wpb_last) { wpb_last = percent_sent;
      int wpb=percent_sent, wpbw=TFT_WIDTH-4, wpbf=((wpbw-2)*wpb)/100;
      display.tft->drawRect(2,54,wpbw,10,ST77XX_WHITE);
      display.tft->fillRect(3,55,wpbf,8,ST77XX_GREEN);
      display.tft->fillRect(3+wpbf,55,(wpbw-2)-wpbf,8,ST77XX_BLACK);
      display.tft->fillRect(0, 32, TFT_WIDTH, 8, ST77XX_BLACK);
      display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      display.tft->setCursor(0, 32);
      display.tft->print(String(totalBytesSent / 1024) + "KB/" +
                         String(fileSize / 1024) + "KB");
    }
  }

  size_t w2 = client->print(part2);
  size_t w3 = client->print(part3);
  client->flush();
  uint32_t sendMs = millis() - sendStart;
  Logger::log(STD_MSG, "[WIGLE][TX] body complete: " + String((unsigned)totalBytesSent) + "/" +
              String((unsigned)fileSize) + "B in " + String(sendMs) + "ms shortWrites=" +
              String((unsigned)shortWrites) + " tail=" + String((unsigned)(w2 + w3)) +
              " connected=" + String(client->connected()) + " avail=" + String(client->available()));

  fileToUpload.close();


  // Read response (instrumented)
  String response;
  response.reserve(1200);
  const uint32_t R_TIMEOUT = 8000;    // hard cap; we normally break early once idle
  uint32_t rStart = millis();
  uint32_t firstByteMs = 0;
  uint32_t lastData = 0;
  bool peerClosed = false;
  while (millis() - rStart < R_TIMEOUT) {
    bool got = false;
    while (client->available()) {
      if (firstByteMs == 0) firstByteMs = millis() - rStart;
      char c = client->read();
      if (response.length() < 1100) response += c;
      got = true;
    }
    if (got) lastData = millis();
    if (!client->connected() && !client->available()) { peerClosed = true; break; }
    // Full response headers received and the stream has gone idle — done.
    if (lastData && response.indexOf("\r\n\r\n") >= 0 && (millis() - lastData) > 400) break;
    delay(5);
  }
  uint32_t rMs = millis() - rStart;
  bool timeoutHit = (millis() - rStart >= R_TIMEOUT);

  Logger::log(STD_MSG, "[WIGLE][RX] respBytes=" + String(response.length()) +
              " firstByteMs=" + String(firstByteMs) + " elapsedMs=" + String(rMs) +
              " peerClosed=" + String(peerClosed) + " timeoutHit=" + String(timeoutHit) +
              " connected=" + String(client->connected()) + " heap=" + String(ESP.getFreeHeap()));
  Logger::log(STD_MSG, "[WIGLE][RX] raw>>>" + response + "<<<");

  client->stop();
  this->reserveTlsGuard();

  bool ok = response.indexOf("200 OK") >= 0 ||
            response.indexOf("\"success\":true") >= 0 ||
            response.indexOf("409") >= 0;
  Logger::log(STD_MSG, "[WIGLE] classified ok=" + String(ok));

  return ok;
}

// ============================================================
// Chunk 3: WDG Wars upload + sidecar tracking system
// ============================================================

// Upload-state markers live in /sc/ so the SD root holds only .log files —
// keeps the file-menu directory walk fast. Marker for "/NAME.log" is
// "/sc/NAME.log.<service>".
static String scPath(const String& filePath, const String& service) {
  String base = filePath;
  if (base.startsWith("/")) base = base.substring(1);
  return "/sc/" + base + "." + service;
}

// Check whether a service sidecar exists for a given log file.
// service is "wigle" or "wdg".
// filePath should include leading slash, e.g. "/wardrive.log"
bool WiFiOps::sidecarExists(String filePath, String service) {
  return SD.exists(scPath(filePath, service));
}

// Write a sidecar file recording the upload timestamp.
void WiFiOps::writeSidecar(String filePath, String service) {
  if (!SD.exists("/sc")) SD.mkdir("/sc");
  String sidecarPath = scPath(filePath, service);
  File f = SD.open(sidecarPath, FILE_WRITE);
  if (f) {
    f.println("uploaded=" + gps.getDatetime());
    f.close();
    Logger::log(GUD_MSG, "[UPLOAD] Sidecar written: " + sidecarPath);
  } else {
    Logger::log(WARN_MSG, "[UPLOAD] Could not write sidecar: " + sidecarPath);
  }
}

// One-time migration: move any legacy root sidecars into /sc/. Gated by a
// marker so it only walks the root once.
void WiFiOps::migrateSidecars() {
  if (!sd_obj.supported) return;
  if (SD.exists("/sc/.migrated")) return;

  if (!SD.exists("/sc")) SD.mkdir("/sc");

  // Collect first — renaming during the directory walk can corrupt the iterator.
  std::vector<String> markers;
  File root = SD.open("/");
  if (root && root.isDirectory()) {
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      if (f.isDirectory()) continue;
      String n = f.name();
      if (n.endsWith(".wigle") || n.endsWith(".wdg")) markers.push_back(n);
    }
    root.close();
  }

  int moved = 0;
  for (size_t i = 0; i < markers.size(); i++) {
    if (SD.rename("/" + markers[i], "/sc/" + markers[i])) moved++;
  }

  File m = SD.open("/sc/.migrated", FILE_WRITE);
  if (m) { m.println("1"); m.close(); }
  Logger::log(GUD_MSG, "[SC] Sidecar migration done — moved " + String(moved) + " marker(s) to /sc/");
}

// Upload one log file to WDG Wars.
// Mirrors backendUpload() but uses X-API-Key auth and wdgwars.pl endpoint.
bool WiFiOps::wdgwarsUpload(String filePath) {
  delay(100);

  if (!SD.exists(filePath)) {
    Logger::log(WARN_MSG, "[WDG] File not found: " + filePath);
    return false;
  }

  String apiKey = settings.loadSetting<String>(WDG_KEY_NAME);
  if (apiKey.isEmpty()) {
    Logger::log(WARN_MSG, "[WDG] No WDG Wars API key configured");
    return false;
  }

  File fileToUpload = SD.open(filePath);
  if (!fileToUpload) {
    Logger::log(WARN_MSG, "[WDG] Could not open: " + filePath);
    return false;
  }

  // Build multipart body
  String boundary   = "----ESP32BOUNDARY";
  String part1      = "--" + boundary + "\r\n";
  part1 += "Content-Disposition: form-data; name=\"file\"; filename=\"" +
           filePath + "\"\r\n";
  part1 += "Content-Type: application/octet-stream\r\n\r\n";
  String part2      = "\r\n--" + boundary + "--\r\n";
  int totalLength   = part1.length() + fileToUpload.size() + part2.length();

  Logger::log(STD_MSG, "[WDG] File size: " + String(fileToUpload.size()));
  Logger::log(STD_MSG, "[WDG] Total length: " + String(totalLength));

  client->setInsecure();
  client->setTimeout(5000);

  this->freeTlsGuard();

  Logger::log(STD_MSG, "[HEAP] wdg pre-connect free=" + String(ESP.getFreeHeap()) +
              " maxAlloc=" + String(ESP.getMaxAllocHeap()));

  if (!client->connect("wdgwars.pl", 443, 8000)) {
    fileToUpload.close();
    client->stop();
    this->reserveTlsGuard();
    Logger::log(WARN_MSG, "[WDG] Failed to connect to wdgwars.pl");
    return false;
  }
  client->setNoDelay(true);

  // HTTP request
  client->println("POST /api/v2/upload-csv HTTP/1.1");
  client->println("Host: wdgwars.pl");
  client->println("User-Agent: ESP32Uploader/1.0");
  client->println("Accept: application/json");
  client->println("X-API-Key: " + apiKey);
  client->println("Content-Type: multipart/form-data; boundary=" + boundary);
  client->print("Content-Length: ");
  client->println(totalLength);
  client->println();

  // Send body
  client->print(part1);

  const size_t CHUNK = 4096;
  uint8_t buf[CHUNK];
  size_t totalSent = 0;
  uint8_t pct = 0;
  int wpb_last = -1;
  String pctStr;

  display.tft->setTextSize(1);
  display.tft->fillRect(0, 10, TFT_WIDTH, 28, ST77XX_BLACK);
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->setCursor(0, 12);
  display.tft->print("wdg uploading");
  display.tft->setCursor(0, 24);
  {
    String fn = filePath;
    if (fn.startsWith("/")) fn = fn.substring(1);
    if (fn.length() > 26) fn = fn.substring(0, 26);
    display.tft->print(fn);
  }

  size_t wdgFileSize = fileToUpload.size();
  Logger::log(STD_MSG, "[WDG][REQ] POST wdgwars.pl/api/v2/upload-csv CL=" +
              String(totalLength) + " fileSize=" + String((unsigned)wdgFileSize) +
              " keyLen=" + String(apiKey.length()) + " heapFree=" + String(ESP.getFreeHeap()) +
              " maxAlloc=" + String(ESP.getMaxAllocHeap()));

  size_t shortWrites = 0;
  bool sendConnLost = false;
  int lastDecile = -1;
  uint32_t sendStart = millis();
  while (fileToUpload.available()) {
    size_t n = fileToUpload.read(buf, CHUNK);
    size_t w = client->write(buf, n);
    totalSent += n;
    if (w != n) {
      shortWrites++;
      Logger::log(WARN_MSG, "[WDG][TX] SHORT WRITE " + String((unsigned)w) + "/" +
                  String((unsigned)n) + " @off " + String((unsigned)totalSent) +
                  " connected=" + String(client->connected()));
    }
    if (!client->connected() && !sendConnLost) {
      sendConnLost = true;
      Logger::log(WARN_MSG, "[WDG][TX] SOCKET DROPPED mid-send @off " + String((unsigned)totalSent));
    }
    if (!client->connected()) {
      Logger::log(WARN_MSG, "[WDG][TX] ABORT — socket dead, stopping send @off " +
                  String((unsigned)totalSent) + "/" + String((unsigned)wdgFileSize));
      break;
    }
    pct = wdgFileSize ? (totalSent * 100) / wdgFileSize : 100;
    if ((int)pct / 10 != lastDecile) {
      lastDecile = (int)pct / 10;
      Logger::log(STD_MSG, "[WDG][TX] " + String(pct) + "% " +
                  String((unsigned)(totalSent/1024)) + "KB conn=" +
                  String(client->connected()) + " heap=" + String(ESP.getFreeHeap()));
    }
    if ((int)pct != wpb_last) { wpb_last = pct;
      int wpb=pct, wpbw=TFT_WIDTH-4, wpbf=((wpbw-2)*wpb)/100;
      display.tft->drawRect(2,54,wpbw,10,ST77XX_WHITE);
      display.tft->fillRect(3,55,wpbf,8,ST77XX_GREEN);
      display.tft->fillRect(3+wpbf,55,(wpbw-2)-wpbf,8,ST77XX_BLACK);
      display.tft->fillRect(0, 32, TFT_WIDTH, 8, ST77XX_BLACK);
      display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      display.tft->setCursor(0, 32);
      display.tft->print(String(totalSent / 1024) + "KB/" +
                         String(wdgFileSize / 1024) + "KB");
    }
  }

  size_t w2 = client->print(part2);
  client->flush();
  fileToUpload.close();
  uint32_t sendMs = millis() - sendStart;
  Logger::log(STD_MSG, "[WDG][TX] body complete: " + String((unsigned)totalSent) + "/" +
              String((unsigned)wdgFileSize) + "B in " + String(sendMs) + "ms shortWrites=" +
              String((unsigned)shortWrites) + " tail=" + String((unsigned)w2) +
              " connected=" + String(client->connected()) + " avail=" + String(client->available()));

  // Read response (instrumented)
  String response;
  response.reserve(1200);
  const uint32_t R_TIMEOUT = 8000;    // hard cap; we normally break early once idle
  uint32_t rStart = millis();
  uint32_t firstByteMs = 0;
  uint32_t lastData = 0;
  bool peerClosed = false;
  while (millis() - rStart < R_TIMEOUT) {
    bool got = false;
    while (client->available()) {
      if (firstByteMs == 0) firstByteMs = millis() - rStart;
      char c = client->read();
      if (response.length() < 1100) response += c;
      got = true;
    }
    if (got) lastData = millis();
    if (!client->connected() && !client->available()) { peerClosed = true; break; }
    // Full response headers received and the stream has gone idle — done.
    if (lastData && response.indexOf("\r\n\r\n") >= 0 && (millis() - lastData) > 400) break;
    delay(5);
  }
  uint32_t rMs = millis() - rStart;
  bool timeoutHit = (millis() - rStart >= R_TIMEOUT);
  Logger::log(STD_MSG, "[WDG][RX] respBytes=" + String(response.length()) +
              " firstByteMs=" + String(firstByteMs) + " elapsedMs=" + String(rMs) +
              " peerClosed=" + String(peerClosed) + " timeoutHit=" + String(timeoutHit) +
              " connected=" + String(client->connected()) + " heap=" + String(ESP.getFreeHeap()));
  Logger::log(STD_MSG, "[WDG][RX] raw>>>" + response + "<<<");

  client->stop();
  this->reserveTlsGuard();

  bool ok = response.indexOf("202 Accepted") >= 0 ||
            response.indexOf("200 OK") >= 0 ||
            response.indexOf("\"ok\":true") >= 0 ||
            response.indexOf("409") >= 0;
  Logger::log(STD_MSG, "[WDG] classified ok=" + String(ok));

  return ok;
}

// Upload a single file to both WiGLE and WDG Wars.
// Skips any service that already has a sidecar, unless retry=true.
// Writes sidecar for each service that succeeds.
// Returns true only if both uploads succeed (or were already done).
bool WiFiOps::uploadFile(String filePath, bool retry, uint8_t upload_type) {
  Logger::log(STD_MSG, "[UPLOAD] uploadFile: " + filePath +
              (retry ? " (retry)" : ""));

  bool wigle_already = !retry && this->sidecarExists(filePath, "wigle");
  bool wdg_already   = !retry && this->sidecarExists(filePath, "wdg");

  bool wigle_ok = wigle_already;
  bool wdg_ok   = wdg_already;

  if ((upload_type == WIGLE_UPLOAD) || (upload_type == BOTH_UPLOAD)) {
    if (!wigle_already) {
      Logger::log(STD_MSG, "[UPLOAD] Uploading to WiGLE: " + filePath);
      wigle_ok = this->wigleUpload(filePath);
      if (wigle_ok) {
        this->writeSidecar(filePath, "wigle");
        Logger::log(GUD_MSG, "[UPLOAD] WiGLE upload succeeded");
      } else {
        Logger::log(WARN_MSG, "[UPLOAD] WiGLE upload failed");
      }
    } else {
      Logger::log(STD_MSG, "[UPLOAD] WiGLE already uploaded, skipping");
    }
  }

  if ((upload_type == WDG_UPLOAD) || (upload_type == BOTH_UPLOAD)) {
    if (!wdg_already) {
      Logger::log(STD_MSG, "[UPLOAD] Uploading to WDG Wars: " + filePath);
      wdg_ok = this->wdgwarsUpload(filePath);
      if (wdg_ok) {
        this->writeSidecar(filePath, "wdg");
        Logger::log(GUD_MSG, "[UPLOAD] WDG Wars upload succeeded");
      } else {
        Logger::log(WARN_MSG, "[UPLOAD] WDG Wars upload failed");
      }
    } else {
      Logger::log(STD_MSG, "[UPLOAD] WDG Wars already uploaded, skipping");
    }
  }

  if (upload_type == WIGLE_UPLOAD)
    return wigle_ok;
  else if (upload_type == WDG_UPLOAD)
    return wdg_ok;
  else if (upload_type == BOTH_UPLOAD)
    return wigle_ok && wdg_ok;
  
  return false;
}

// Scan the SD card root for all .log files that are missing at least
// one service sidecar, and upload them. Used by dock mode (Chunk 6).
void WiFiOps::reserveTlsGuard() {
  if (this->tls_heap_guard) return;
  for (size_t sz = 42 * 1024; sz >= 36 * 1024; sz -= 2 * 1024) {
    void *p = malloc(sz);
    if (p) {
      this->tls_heap_guard = p;
      this->tls_heap_guard_sz = sz;
      Logger::log(STD_MSG, "[HEAP] TLS guard reserved " + String((unsigned)(sz / 1024)) +
                  "KB, maxAlloc now " + String(ESP.getMaxAllocHeap()));
      return;
    }
  }
  Logger::log(WARN_MSG, "[HEAP] TLS guard reserve FAILED (maxAlloc " +
              String(ESP.getMaxAllocHeap()) + ")");
}

void WiFiOps::freeTlsGuard() {
  if (!this->tls_heap_guard) return;
  free(this->tls_heap_guard);
  this->tls_heap_guard = nullptr;
}

void WiFiOps::drawUploadCounts(int ok, int failed, int pending) {
  display.tft->setCursor(0, 40);
  display.tft->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  display.tft->print("OK " + String(ok) + " ");
  display.tft->setTextColor(failed > 0 ? ST77XX_RED : ST77XX_GREEN, ST77XX_BLACK);
  display.tft->print("Fail " + String(failed) + " ");
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->print("Left " + String(pending));
}

// ============================================================
// Online OTA — pull the latest dark3d release from GitHub
// ============================================================
void WiFiOps::requestOnlineUpdate() { rtc_ota_check = true; }
bool WiFiOps::otaCheckPending() {
  esp_reset_reason_t rr = esp_reset_reason();
  if (rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT || rr == ESP_RST_UNKNOWN) {
    rtc_ota_check = false;                     // cold boot: clear NOINIT garbage/stale flag
    return false;
  }
  return rtc_ota_check;                         // SW reset (the trigger's reboot): honor it
}

// Boot-time entry: runs in the early clean-heap window (mbedTLS wants a large
// contiguous heap, same reason the dock upload runs here). Connects WiFi, runs
// the check, tears WiFi back down if it didn't flash+reboot.
void WiFiOps::runOnlineUpdateCheck() {
  rtc_ota_check = false;                       // consume the request (no loop)
  Logger::log(STD_MSG, "[OTA] online update check requested");

  this->initWiFi();                            // bring the radio up first (as tryBootDockUpload does)
  bool connected = this->connectForUpload();   // present dock net, else Network
  if (!connected) {
    Logger::log(WARN_MSG, "[OTA] no WiFi for update check");
    display.clearScreen();
    display.drawCenteredText("No WiFi for update", true);
    delay(1500);
    return;
  }

  this->checkForOnlineUpdate();                 // reboots on a successful flash
  this->deinitWiFi();
}

// Fetch releases/latest, compare tag to FIRMWARE_VERSION, and if different
// stream the .bin asset straight into the OTA partition. Returns true only when
// it flashed (in which case it has already rebooted).
bool WiFiOps::checkForOnlineUpdate() {
  display.clearScreen();
  display.drawCenteredText("Checking for updates", true);

  WiFiClientSecure secure;
  secure.setInsecure();
  secure.setTimeout(10000);

  HTTPClient https;
  https.setReuse(false);
  https.setTimeout(10000);
  if (!https.begin(secure, OTA_RELEASES_API)) {
    Logger::log(WARN_MSG, "[OTA] api begin failed");
    return false;
  }
  https.addHeader("User-Agent", "dark3d-wardriver");
  https.addHeader("Accept", "application/vnd.github+json");
  https.addHeader("Accept-Encoding", "identity");   // no gzip -> parseable body

  int code = https.GET();
  if (code != HTTP_CODE_OK) {
    Logger::log(WARN_MSG, "[OTA] api GET " + String(code));
    https.end();
    display.drawCenteredText("Update check failed", true);
    delay(1500);
    return false;
  }

  // getString() dechunks the (chunked) GitHub response; passing the raw
  // getStream() to the parser fails on the chunk-size framing (IncompleteInput).
  String payload = https.getString();
  https.end();

  // Keep only tag_name + each asset's name / download URL out of the big payload
  DynamicJsonDocument filter(256);
  filter["tag_name"] = true;
  filter["assets"][0]["name"] = true;
  filter["assets"][0]["browser_download_url"] = true;

  DynamicJsonDocument doc(3072);
  DeserializationError err =
    deserializeJson(doc, payload, DeserializationOption::Filter(filter));
  if (err) {
    Logger::log(WARN_MSG, "[OTA] json parse: " + String(err.c_str()));
    return false;
  }

  String tag = doc["tag_name"] | "";
  Logger::log(STD_MSG, "[OTA] latest " + tag + " / current " + String(FIRMWARE_VERSION));
  if (tag.length() == 0) return false;
  if (tag == FIRMWARE_VERSION) {
    display.clearScreen();
    display.tft->setTextSize(2);                       // bigger, readable
    display.tft->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    display.tft->setCursor((TFT_WIDTH - 10 * 12) / 2, 18);
    display.tft->print("Up to date");
    display.tft->setTextSize(1);
    display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display.tft->setCursor((TFT_WIDTH - (int)tag.length() * 6) / 2, 46);
    display.tft->print(tag);                           // version on its own row
    delay(2200);
    return false;
  }

  String binUrl;
  for (JsonObject a : doc["assets"].as<JsonArray>()) {
    String n = a["name"] | "";
    if (n.endsWith(".bin")) { binUrl = a["browser_download_url"] | ""; break; }
  }
  if (binUrl.length() == 0) {
    Logger::log(WARN_MSG, "[OTA] no .bin asset in latest release");
    return false;
  }

  display.clearScreen();
  display.tft->setCursor(0, 8);
  display.drawCenteredText("Updating to:", false);
  display.tft->setCursor(0, 20);
  display.drawCenteredText(tag, false);
  Logger::log(STD_MSG, "[OTA] downloading " + binUrl);

  WiFiClientSecure dlsec;
  dlsec.setInsecure();
  dlsec.setTimeout(15000);

  HTTPClient dl;
  dl.setReuse(false);
  dl.setTimeout(15000);
  dl.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);   // github -> CDN 302
  if (!dl.begin(dlsec, binUrl)) return false;
  dl.addHeader("User-Agent", "dark3d-wardriver");

  int dcode = dl.GET();
  if (dcode != HTTP_CODE_OK) {
    Logger::log(WARN_MSG, "[OTA] download GET " + String(dcode));
    dl.end();
    return false;
  }
  int len = dl.getSize();
  if (len <= 0) {
    Logger::log(WARN_MSG, "[OTA] bad content-length " + String(len));
    dl.end();
    return false;
  }
  Logger::log(STD_MSG, "[OTA] size " + String(len) + "B heap " + String(ESP.getFreeHeap()));

  // Live progress bar so the flash never looks stuck (writeStream is ~30-60s)
  Update.onProgress([](size_t done, size_t total) {
    static int last = -1;
    int pct = total ? (int)(done * 100 / total) : 0;
    if (pct == last) return;
    last = pct;
    int bx = 12, by = 52, bw = TFT_WIDTH - 24, bh = 14;
    display.tft->drawRect(bx, by, bw, bh, ST77XX_WHITE);
    display.tft->fillRect(bx + 2, by + 2, (bw - 4) * pct / 100, bh - 4, ST77XX_CYAN);
    char b[6]; snprintf(b, sizeof(b), "%d%%", pct);
    display.tft->setTextSize(1);
    display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display.tft->setCursor(TFT_WIDTH / 2 - (int)strlen(b) * 3, 38);
    display.tft->print(b);
  });

  WiFiClient* stream = dl.getStreamPtr();
  bool ok = sd_obj.performUpdate(*stream, (size_t)len);
  dl.end();

  if (ok) {
    const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
    if (next) esp_ota_set_boot_partition(next);
    display.clearScreen();
    display.drawCenteredText("Updated. Rebooting", true);
    delay(1000);
    Settings::safeRestart();
    return true;
  }

  display.drawCenteredText("Update failed", true);
  delay(1500);
  return false;
}

bool WiFiOps::tryBootDockUpload() {
  esp_reset_reason_t rr = esp_reset_reason();
  Logger::log(STD_MSG, "[DOCK] boot rr=" + String((int)rr) + " latch=" + String(rtc_dock_done ? 1 : 0));
  if (rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT || rr == ESP_RST_UNKNOWN)
    rtc_dock_done = false;

  if (rtc_dock_done) return false;                       // already synced this dock session

  // Upload if any docking network or the boot Network is configured
  if (!this->anyDockConfigured() &&
      settings.loadSetting<String>("s").isEmpty()) return false;

  Logger::log(STD_MSG, "[DOCK] Minimal-boot dock upload — clean heap before feature init");
  Logger::log(STD_MSG, "[HEAP] pre-upload free=" + String(ESP.getFreeHeap()) +
              " maxAlloc=" + String(ESP.getMaxAllocHeap()));

  this->initWiFi();
  bool connected = this->connectForUpload();
  this->connected_as_client = connected;

  if (connected) {
    display.clearScreen();
    display.tft->setCursor(0, 0);
    display.tft->setTextSize(2);
    display.tft->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
    display.tft->println("UPLOADING");
    display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display.tft->setTextSize(1);
    display.tft->println("Syncing pending logs...");
    this->uploadAllPendingV2();
  } else {
    Logger::log(WARN_MSG, "[DOCK] Minimal-boot: WiFi connect failed — skipping upload");
  }

  rtc_dock_done = true;
  Logger::log(GUD_MSG, "[DOCK] Minimal-boot upload done — rebooting to normal mode");
  delay(200);
  ESP.restart();
  return true;                                           // unreachable (reboots)
}

void WiFiOps::uploadAllPendingV2() {
  if (!sd_obj.supported) { Logger::log(WARN_MSG, "[UPLOAD] SD not available"); return; }
  Logger::log(STD_MSG, "[HEAP] dock-upload start free=" + String(ESP.getFreeHeap()) +
              " maxAlloc=" + String(ESP.getMaxAllocHeap()));

  size_t ma = ESP.getMaxAllocHeap();
  if (ma < 60 * 1024) {
    Logger::log(WARN_MSG, "[UPLOAD] Skipping — maxAlloc " + String((unsigned)ma) +
                " < 60KB; not enough contiguous heap for TLS (needs minimal-boot path)");
    return;
  }

  this->reserveTlsGuard();

  UploadManager mgr;
  mgr.addService(UploadService{
    "wigle", "wigle", 0, 300000,
    [this](const String& path){
      return this->wigleUpload(path) ? UploadResult::Success : UploadResult::Throttled;
    }});
  mgr.addService(UploadService{
    "wdg", "wdg", 0, 300000,
    [this](const String& path){
      return this->wdgwarsUpload(path) ? UploadResult::Success : UploadResult::Throttled;
    }});

  mgr.setScan(
    [](std::vector<String>& out){
      File root = SD.open("/");
      if (!root || !root.isDirectory()) return;
      File f = root.openNextFile();
      while (f) {
        if (!f.isDirectory()) {
          String n = f.name();
          if (n.endsWith(".log") && n != "debug.log") out.push_back("/" + n);
        }
        f = root.openNextFile();
      }
      root.close();
    },
    [this](const String& path, const char* tag){ return this->sidecarExists(path, tag); },
    [this](const String& path, const char* tag){ this->writeSidecar(path, tag); });

  mgr.begin();
  display.clearScreen();
  display.tft->setTextSize(1);
  display.tft->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  display.tft->setCursor(0, 0);
  display.tft->print("== DOCK UPLOAD ==");
  {
    UploadManager::Stats ss0 = mgr.stats();
    this->drawUploadCounts(ss0.ok, ss0.failed, ss0.pending);
  }
  while (mgr.runNext(millis())) {
    UploadManager::Stats ss = mgr.stats();
    display.tft->fillRect(0, 10, TFT_WIDTH, 40, ST77XX_BLACK);
    String stage = mgr.lastStage();
    bool badStage = (stage == "backoff" || stage == "error");
    display.tft->setTextColor(badStage ? ST77XX_YELLOW : ST77XX_WHITE, ST77XX_BLACK);
    display.tft->setCursor(0, 12);
    display.tft->print(String(mgr.lastService()) + " " + stage);
    display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display.tft->setCursor(0, 24);
    String fn = String(mgr.lastFile()); if (fn.startsWith("/")) fn = fn.substring(1);
    if (fn.length() > 26) fn = fn.substring(0, 26);
    display.tft->print(fn);
    this->drawUploadCounts(ss.ok, ss.failed, ss.pending);
    if (!this->tls_heap_guard && mgr.stats().ok >= 3) {
      Logger::log(GUD_MSG, "[UPLOAD] Heap low after " + String(mgr.stats().ok) + " uploads, rebooting to continue");
      delay(300);
      Settings::safeRestart();
    }
  }

  UploadManager::Stats st = mgr.stats();
  Logger::log(GUD_MSG, "[UPLOAD] V2 done. OK:" + String(st.ok) + " Failed:" +
              String(st.failed) + " Skipped:" + String(st.skipped) +
              " Pending(backoff):" + String(st.pending));
  this->freeTlsGuard();
}

void WiFiOps::uploadAllPending() {
  Logger::log(STD_MSG, "[UPLOAD] Scanning SD for pending uploads...");
  this->reserveTlsGuard();

  if (!sd_obj.supported) {
    Logger::log(WARN_MSG, "[UPLOAD] SD not available");
    return;
  }

  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    Logger::log(WARN_MSG, "[UPLOAD] Could not open SD root");
    return;
  }

  int uploaded = 0;
  int skipped  = 0;
  int failed   = 0;
  bool made_progress = false;

  LinkedList<String> logs;
  File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String name = f.name();
      if (name.endsWith(".log") && name != "debug.log")
        logs.add("/" + name);
    }
    f = root.openNextFile();
  }
  root.close();

  for (int idx = 0; idx < logs.size(); idx++) {
    String path = logs.get(idx);
    bool wigle_done = this->sidecarExists(path, "wigle");
    bool wdg_done   = this->sidecarExists(path, "wdg");

    if (wigle_done && wdg_done) {
      skipped++;
    } else {
      Logger::log(STD_MSG, "[UPLOAD] Pending: " + path);
      bool wigle_needed = !wigle_done;
      bool wdg_needed   = !wdg_done;
      bool ok = this->uploadFile(path, false, BOTH_UPLOAD);
      // Count as uploaded if all needed services succeeded
      bool wigle_now = this->sidecarExists(path, "wigle");
      bool wdg_now   = this->sidecarExists(path, "wdg");
      bool fully_done = (!wigle_needed || wigle_now) && (!wdg_needed || wdg_now);
      if (fully_done) uploaded++;
      else            failed++;

      if ((wigle_needed && wigle_now) || (wdg_needed && wdg_now))
        made_progress = true;

      if (!this->tls_heap_guard && made_progress) {
        Logger::log(GUD_MSG, "[UPLOAD] Heap low, rebooting to continue on clean heap");
        delay(300);
        Settings::safeRestart();
      }
    }
  }

  Logger::log(GUD_MSG, "[UPLOAD] Done. Uploaded:" + String(uploaded) +
              " Skipped:" + String(skipped) + " Failed:" + String(failed));
  this->freeTlsGuard();
}

// ============================================================
// Returns true if auth passes (or no password is set).
// Sends 401 and returns false if auth fails.
// ============================================================
bool WiFiOps::checkAuth() {
  String adminPass = settings.loadSetting<String>(ADMIN_PASS_NAME);
  if (adminPass.isEmpty()) return true; // no password set — open access

  // Expect "Authorization: Basic <base64("admin:<pass>")>"
  String expected = "Basic " + utils.base64Encode("admin:" + adminPass);
  if (server.hasHeader("Authorization") &&
      server.header("Authorization") == expected) {
    return true;
  }

  server.sendHeader("WWW-Authenticate", "Basic realm=\"C5 Wardriver\"");
  server.send(401, "text/plain", "Authentication required");
  return false;
}

void WiFiOps::serveConfigPage() {

  // ---- GET / : main config page ----
  bool cur_dbg_en = settings.loadSetting<bool>(DEBUG_LOG_NAME);
  server.on("/", HTTP_GET, [this, cur_dbg_en]() {
    if (!this->checkAuth()) return;
    this->last_web_client_activity = millis();

    // Load current values to pre-populate the form
    String cur_ssid        = settings.loadSetting<String>("s");
    String cur_wigle_user  = settings.loadSetting<String>("wu");
    String cur_wdg_key     = settings.loadSetting<String>(WDG_KEY_NAME);
    bool   cur_dbg_en      = settings.loadSetting<bool>(DEBUG_LOG_NAME);
    bool   cur_gps_buf     = settings.loadSetting<bool>(GPS_BUFFER_NAME);
    int    cur_gps_buf_win = this->gpsBufferWindowMin();
    int    cur_log_keep    = this->logKeepCount();
    int    cur_mode        = settings.loadSetting<int>("m");
    bool   cur_enc         = settings.loadSetting<bool>("e");

    String modeStr = (cur_mode == CORE_MODE) ? "Core" : (cur_mode == NODE_MODE ? "Node" : "Solo");
    bool set_pass  = !settings.loadSetting<String>("p").isEmpty();
    bool set_wt    = !settings.loadSetting<String>("wt").isEmpty();
    bool set_wdg   = !cur_wdg_key.isEmpty();
    bool set_admin = !settings.loadSetting<String>(ADMIN_PASS_NAME).isEmpty();
    const char* DOTS = "&bull;&bull;&bull;&bull;&bull;&bull;&bull;&bull;";

    String html;
    html.reserve(10000);
    html += "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">";
    html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<title>Wardriver Configuration</title><style>";
    html += ":root{--bg:#e9e1d3;--card:#faf6ee;--bd:#d3c6b0;--bds:#b9a888;--tx:#2e2820;--mut:#7c7161;--acc:#6f4e37;--acd:#573c2a;--ok:#4f7a3a;--okb:#e7efdd}";
    html += "*{box-sizing:border-box}html,body{margin:0;padding:0}";
    html += "body{background:var(--bg);color:var(--tx);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Helvetica,Arial,sans-serif;font-size:15px;line-height:1.5;padding:0 16px 96px}";
    html += ".wrap{max-width:620px;margin:0 auto}";
    html += "header{padding:22px 2px 6px}header h1{margin:0;font-size:22px;font-weight:600}header .meta{color:var(--mut);font-size:13px;margin-top:2px}";
    html += "section{background:var(--card);border:1px solid var(--bd);border-radius:8px;padding:16px 16px 4px;margin:16px 0}";
    html += "section>h2{margin:0 0 3px;font-size:16px;font-weight:600;color:var(--acc);padding-bottom:8px;border-bottom:1px solid var(--bd)}";
    html += ".hint{color:var(--mut);font-size:13px;margin:8px 0 14px}";
    html += ".field{margin:0 0 14px}.field>label{display:block;font-size:13px;font-weight:600;margin:0 0 5px}";
    html += "input[type=text],input[type=password],input[type=number]{width:100%;background:#fff;border:1px solid var(--bds);color:var(--tx);border-radius:5px;padding:9px 11px;font-size:15px;font-family:inherit;outline:none}";
    html += "input:focus{border-color:var(--acc)}.num{max-width:150px}";
    html += ".row{display:flex;gap:14px;flex-wrap:wrap}.row>.field{flex:1;min-width:120px}";
    html += ".saved{display:inline-block;font-size:11px;font-weight:700;color:var(--ok);background:var(--okb);border:1px solid #c4d6b3;border-radius:4px;padding:1px 6px;margin-left:6px}";
    html += ".check{display:flex;align-items:flex-start;gap:9px;margin:0 0 14px}.check input{margin:2px 0 0;width:16px;height:16px;flex:0 0 auto}";
    html += ".check .lab{font-size:14px}.check .lab small{display:block;color:var(--mut);font-size:12px;margin-top:1px}";
    html += ".radios{display:flex;gap:20px;flex-wrap:wrap;padding:2px 0 12px}.radios label{display:flex;align-items:center;gap:7px;font-size:15px}.radios input{width:16px;height:16px}";
    html += ".dock-row{display:flex;gap:8px;align-items:center;padding:4px 0}";
    html += ".dock-row .n{color:var(--mut);font-size:13px;font-weight:600;width:14px;flex:0 0 auto}";
    html += ".dock-row .ssid{flex:2;min-width:0}.dock-row .pass{flex:1;min-width:0}";
    html += ".exgrid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:10px}.exgrid input{min-width:0}";
    html += ".zone{border-top:1px solid var(--bd);padding:14px 0 2px}.zone:first-of-type{border-top:0}.zone .zh{font-size:13px;font-weight:700;margin:0 0 8px}";
    html += "table.files{width:100%;border-collapse:collapse;font-size:14px}table.files td{padding:9px 6px;border-top:1px solid var(--bd);vertical-align:middle}table.files tr:first-child td{border-top:0}";
    html += ".files .fn{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12px;word-break:break-all}";
    html += ".files .sz{color:var(--mut);white-space:nowrap;text-align:right}.files .st{white-space:nowrap;text-align:right}";
    html += ".files a{color:var(--acc);text-decoration:none}.done{color:var(--ok);font-weight:600}";
    html += ".actions{position:sticky;bottom:0;margin:20px -16px 0;padding:14px 16px;background:var(--bg);border-top:1px solid var(--bds)}.actions .inner{max-width:620px;margin:0 auto}";
    html += "button.save{width:100%;border:1px solid var(--acd);border-radius:6px;padding:12px;font-size:16px;font-weight:600;color:#fff;background:var(--acc);cursor:pointer}";
    html += ".foot{text-align:center;color:var(--mut);font-size:14px;padding:16px 0}.foot a{color:var(--acc)}";
    html += "</style></head><body><div class=\"wrap\">";

    html += "<header><h1>Wardriver Configuration</h1><div class=\"meta\">";
    html += String(DEVICE_NAME) + " &middot; " + FIRMWARE_VERSION + " &middot; " + WiFi.localIP().toString() + " &middot; " + modeStr;
    html += "</div></header><form action=\"/save\" method=\"POST\">";

    // ---- Device Mode ----
    html += "<section><h2>Device Mode</h2><div class=\"radios\">";
    html += "<label><input type=\"radio\" name=\"device_mode\" value=\"solo\"" + String(cur_mode == SOLO_MODE ? " checked" : "") + "> Solo</label>";
    html += "<label><input type=\"radio\" name=\"device_mode\" value=\"core\"" + String(cur_mode == CORE_MODE ? " checked" : "") + "> Core</label>";
    html += "<label><input type=\"radio\" name=\"device_mode\" value=\"node\"" + String(cur_mode == NODE_MODE ? " checked" : "") + "> Node</label>";
    html += "</div></section>";

    // ---- Admin ----
    html += "<section><h2>Admin</h2>";
    html += "<p class=\"hint\">Setting a password requires a login to open this page. Recommended on a shared network.</p>";
    html += "<div class=\"field\"><label>Admin password";
    if (set_admin) html += " <span class=\"saved\">Saved</span>";
    html += "</label><input type=\"password\" name=\"admin_pass\" placeholder=\"" + String(set_admin ? DOTS : "") + "\"></div>";
    html += "<label class=\"check\"><input type=\"checkbox\" name=\"dbg_en\" value=\"true\"";
    if (cur_dbg_en) html += " checked";
    html += "><span class=\"lab\">Write debug log to SD<small>Saves every log entry to " + String(DEBUG_LOG_FILE) + "</small></span></label>";
    html += "<label class=\"check\"><input type=\"checkbox\" name=\"gps_buf\" value=\"true\"";
    if (cur_gps_buf) html += " checked";
    html += "><span class=\"lab\">Buffer scans without a GPS fix<small>Log networks seen with no lock, then backfill positions once GPS returns</small></span></label>";
    html += "<div class=\"field num\"><label>Buffer window (min, " + String(GPS_BUFFER_WINDOW_MIN_MIN) + "-" + String(GPS_BUFFER_WINDOW_MIN_MAX) + ")</label><input type=\"number\" class=\"num\" name=\"gps_buf_win\" value=\"" + String(cur_gps_buf_win) + "\" min=\"" + String(GPS_BUFFER_WINDOW_MIN_MIN) + "\" max=\"" + String(GPS_BUFFER_WINDOW_MIN_MAX) + "\" step=\"5\"></div></section>";

    // ---- Storage ----
    html += "<section><h2>Storage</h2>";
    html += "<p class=\"hint\">Keep only the newest logs on the SD card. Older logs are removed once uploaded to every service you use; anything not yet uploaded is kept.</p>";
    html += "<div class=\"field num\"><label>Logs to keep (" + String(LOG_KEEP_MIN) + "-" + String(LOG_KEEP_MAX) + ")</label><input type=\"number\" class=\"num\" name=\"log_keep\" value=\"" + String(cur_log_keep) + "\" min=\"" + String(LOG_KEEP_MIN) + "\" max=\"" + String(LOG_KEEP_MAX) + "\"></div></section>";

    // ---- WiGLE ----
    html += "<section><h2>WiGLE</h2>";
    html += "<div class=\"field\"><label>API name</label><input type=\"text\" name=\"wigle_user\" value=\"" + cur_wigle_user + "\"></div>";
    html += "<div class=\"field\"><label>API token";
    if (set_wt) html += " <span class=\"saved\">Saved</span>";
    html += "</label><input type=\"password\" name=\"wigle_token\" placeholder=\"" + String(set_wt ? DOTS : "") + "\"></div></section>";

    // ---- WDG Wars ----
    html += "<section><h2>WDG Wars</h2>";
    html += "<div class=\"field\"><label>API key";
    if (set_wdg) html += " <span class=\"saved\">Saved</span>";
    html += "</label><input type=\"password\" name=\"wdg_key\" placeholder=\"" + String(set_wdg ? DOTS : "") + "\"></div></section>";

    // ---- Network ----
    html += "<section><h2>Network</h2>";
    html += "<p class=\"hint\">WiFi to join at boot for web access. Also used to upload if none of your docking networks are in range.</p>";
    html += "<div class=\"field\"><label>WiFi name (SSID)</label><input type=\"text\" name=\"ssid\" value=\"" + cur_ssid + "\"></div>";
    html += "<div class=\"field\"><label>WiFi password";
    if (set_pass) html += " <span class=\"saved\">Saved</span>";
    html += "</label><input type=\"password\" name=\"password\" placeholder=\"" + String(set_pass ? DOTS : "") + "\"></div></section>";

    // ---- Docking Networks ----
    html += "<section><h2>Docking Networks</h2>";
    html += "<p class=\"hint\">Docks and uploads whenever any of these is in range (home, work, a hotspot), using that network's password. Leave a row blank to disable it.</p>";
    for (int i = 0; i < MAX_DOCK_SSIDS; i++) {
      bool dpset = !this->dock_pass_cache[i].isEmpty();
      html += "<div class=\"dock-row\"><span class=\"n\">" + String(i + 1) + "</span>";
      html += "<input type=\"text\" class=\"ssid\" name=\"ds_" + String(i) + "\" value=\"" + this->dock_ssid_cache[i] + "\" placeholder=\"Network name\">";
      html += "<input type=\"password\" class=\"pass\" name=\"dp_" + String(i) + "\" placeholder=\"" + String(dpset ? DOTS : "Password") + "\"></div>";
    }
    html += "</section>";

    // ---- Excluded Networks ----
    html += "<section><h2>Excluded Networks</h2>";
    html += "<p class=\"hint\">Networks with these names are never logged. Up to " + String(MAX_SSID_EXCLUSIONS) + ".</p>";
    html += "<div class=\"exgrid\">";
    for (int i = 0; i < MAX_SSID_EXCLUSIONS; i++) {
      String val = settings.loadSetting<String>("sx_" + String(i));
      html += "<input type=\"text\" name=\"sx_" + String(i) + "\" value=\"" + val + "\" placeholder=\"" + String(i + 1) + "\">";
    }
    html += "</div></section>";

    // ---- Geofences ----
    html += "<section><h2>Geofences</h2>";
    html += "<p class=\"hint\">Wardriving pauses inside a zone. If a docking network is in range there, it uploads. Up to " + String(MAX_GEOFENCES) + ".</p>";
    for (int i = 0; i < MAX_GEOFENCES; i++) {
      String geoStr = settings.loadSetting<String>("geo_" + String(i));
      float  gLat = 0.0, gLon = 0.0;
      int    gRad = 0;
      String gLabel = "";

      // Parse stored JSON geo string
      DynamicJsonDocument geoDoc(256);
      if (!geoStr.isEmpty() && deserializeJson(geoDoc, geoStr) == DeserializationError::Ok) {
        gLat   = geoDoc["lat"]   | 0.0f;
        gLon   = geoDoc["lon"]   | 0.0f;
        gRad   = geoDoc["rad"]   | 0;
        gLabel = geoDoc["label"] | "";
      }
      float gRadMiles = gRad > 0 ? gRad / 1609.34 : 0.0;
      char gRadMilesStr[10];
      dtostrf(gRadMiles, 4, 2, gRadMilesStr);

      html += "<div class=\"zone\"><div class=\"zh\">Zone " + String(i + 1) + "</div>";
      html += "<div class=\"field\"><label>Label</label><input type=\"text\" name=\"geo_" + String(i) + "_label\" value=\"" + gLabel + "\"></div>";
      html += "<div class=\"row\">";
      html += "<div class=\"field\"><label>Latitude</label><input type=\"text\" name=\"geo_" + String(i) + "_lat\" value=\"" + String(gLat, 6) + "\"></div>";
      html += "<div class=\"field\"><label>Longitude</label><input type=\"text\" name=\"geo_" + String(i) + "_lon\" value=\"" + String(gLon, 6) + "\"></div>";
      html += "<div class=\"field num\"><label>Radius (mi, 0 = off)</label><input type=\"number\" class=\"num\" name=\"geo_" + String(i) + "_rad\" value=\"" + String(gRadMilesStr) + "\" min=\"0\" max=\"1\" step=\"0.05\"></div>";
      html += "</div></div>";
    }
    html += "</section>";

    // ---- Encryption ----
    html += "<section><h2>Encryption (ESP-NOW)</h2>";
    html += "<div class=\"field\"><label>Key</label><input type=\"text\" name=\"enow_key\" placeholder=\"\"></div>";
    html += "<label class=\"check\"><input type=\"checkbox\" name=\"use_encryption\" value=\"true\"";
    if (cur_enc) html += " checked";
    html += "><span class=\"lab\">Use encryption</span></label></section>";

    // ---- Files on SD Card ----
    html += "<section><h2>Files on SD Card</h2><table class=\"files\">";
    File root = SD.open("/");
    if (root && root.isDirectory()) {
      File file = root.openNextFile();
      while (file) {
        if (!file.isDirectory()) {
          String filename = file.name();
          if (filename.endsWith(".log") && filename != "debug.log") {
            bool wigleDone = SD.exists("/sc/" + filename + ".wigle");
            bool wdgDone   = SD.exists("/sc/" + filename + ".wdg");

            html += "<tr><td class=\"fn\">" + filename + "</td><td class=\"sz\">" + String((file.size() + 1023) / 1024) + " KB</td><td class=\"st\">";
            if (this->connected_as_client) {
              if (wigleDone) html += "<span class=\"done\">WiGLE&check;</span> ";
              else           html += "<a href=\"/upload?file=" + filename + "&svc=wigle\">Upload WiGLE</a> ";
              if (wdgDone)   html += "<span class=\"done\">WDG&check;</span> ";
              else           html += "<a href=\"/upload?file=" + filename + "&svc=wdg\">Upload WDG</a> ";
            }
            html += "&nbsp;<a href=\"/download?file=" + filename + "\">download</a></td></tr>";
          } else if (!filename.endsWith(".wigle") && !filename.endsWith(".wdg")) {
            html += "<tr><td class=\"fn\">" + filename + "</td><td class=\"sz\">" + String((file.size() + 1023) / 1024) + " KB</td><td class=\"st\"><a href=\"/download?file=" + filename + "\">download</a></td></tr>";
          }
        }
        file = root.openNextFile();
      }
    } else {
      html += "<tr><td>Unable to access SD card.</td></tr>";
    }
    html += "</table></section>";

    html += "<div class=\"foot\">&#128196; <a href=\"/log\">View live log</a></div>";
    html += "<div class=\"actions\"><div class=\"inner\"><button class=\"save\" type=\"submit\">Save Settings</button></div></div>";
    html += "</form></div></body></html>";
    server.send(200, "text/html", html);
  });

  // ---- POST /save : save all settings ----
  server.on("/save", HTTP_POST, [this]() {
    if (!this->checkAuth()) return;
    this->last_web_client_activity = millis();

    bool anyChange = false;

    // Network
    if (server.hasArg("ssid") && server.arg("ssid") != "") {
      this->user_ap_ssid = server.arg("ssid");
      settings.saveSetting<bool>("s", this->user_ap_ssid);
      anyChange = true;
    }
    if (server.hasArg("password") && server.arg("password") != "") {
      this->user_ap_password = server.arg("password");
      settings.saveSetting<bool>("p", this->user_ap_password);
      anyChange = true;
    }

    // WiGLE
    if (server.hasArg("wigle_user") && server.arg("wigle_user") != "") {
      this->wigle_user = server.arg("wigle_user");
      settings.saveSetting<bool>("wu", this->wigle_user);
      anyChange = true;
    }
    if (server.hasArg("wigle_token") && server.arg("wigle_token") != "") {
      this->wigle_token = server.arg("wigle_token");
      settings.saveSetting<bool>("wt", this->wigle_token);
      anyChange = true;
    }

    // WDG Wars
    if (server.hasArg("wdg_key") && server.arg("wdg_key") != "") {
      settings.saveSetting<bool>(WDG_KEY_NAME, server.arg("wdg_key"));
      anyChange = true;
    }

    // Docking networks (SSID always saved so a row can be cleared; password
    // only overwritten when a new one is typed, so "Saved" fields are kept).
    bool dockChanged = false;
    for (int i = 0; i < MAX_DOCK_SSIDS; i++) {
      String sk = String(DOCK_SSID_PREFIX) + i;
      String pk = String(DOCK_PASS_PREFIX) + i;
      if (server.hasArg(sk)) {
        settings.saveSetting<bool>(sk, server.arg(sk));
        anyChange = true; dockChanged = true;
      }
      if (server.hasArg(pk) && server.arg(pk) != "") {
        settings.saveSetting<bool>(pk, server.arg(pk));
        anyChange = true; dockChanged = true;
      }
    }
    if (dockChanged) this->loadDockCache();

    // Admin password
    if (server.hasArg("admin_pass") && server.arg("admin_pass") != "") {
      settings.saveSetting<bool>(ADMIN_PASS_NAME, server.arg("admin_pass"));
      anyChange = true;
    }
    bool dbgEn = server.hasArg("dbg_en") && server.arg("dbg_en") == "true";
    settings.saveSetting<bool>(DEBUG_LOG_NAME, dbgEn);
    Logger::enableSDLog(dbgEn);
    anyChange = true;

    bool gpsBuf = server.hasArg("gps_buf") && server.arg("gps_buf") == "true";
    settings.saveSetting<bool>(GPS_BUFFER_NAME, gpsBuf);
    this->gps_buffering_enabled = gpsBuf;
    anyChange = true;

    if (server.hasArg("gps_buf_win")) {
      int w = server.arg("gps_buf_win").toInt();
      if (w < GPS_BUFFER_WINDOW_MIN_MIN) w = GPS_BUFFER_WINDOW_MIN_MIN;
      if (w > GPS_BUFFER_WINDOW_MIN_MAX) w = GPS_BUFFER_WINDOW_MIN_MAX;
      settings.saveSetting<bool>(GPS_BUFFER_WINDOW_NAME, w, true);
      anyChange = true;
    }

    if (server.hasArg("log_keep")) {
      int n = server.arg("log_keep").toInt();
      if (n < LOG_KEEP_MIN) n = LOG_KEEP_MIN;
      if (n > LOG_KEEP_MAX) n = LOG_KEEP_MAX;
      settings.saveSetting<bool>(LOG_KEEP_NAME, n, true);
      anyChange = true;
    }

    // SSID exclusions
    for (int i = 0; i < MAX_SSID_EXCLUSIONS; i++) {
      String key = "sx_" + String(i);
      if (server.hasArg(key)) {
        settings.saveSetting<bool>(key, server.arg(key));
        anyChange = true;
      }
    }

    // Geofences — reconstruct JSON string from individual form fields
    for (int i = 0; i < MAX_GEOFENCES; i++) {
      String latKey   = "geo_" + String(i) + "_lat";
      String lonKey   = "geo_" + String(i) + "_lon";
      String radKey   = "geo_" + String(i) + "_rad";
      String labelKey = "geo_" + String(i) + "_label";

      if (server.hasArg(latKey)) {
        float  lat   = server.arg(latKey).toFloat();
        float  lon   = server.arg(lonKey).toFloat();
        float radMiles = server.arg(radKey).toFloat();
        if (radMiles <= 0.0) radMiles = 0.0;                        // zone disabled
        else if (radMiles < 0.10) radMiles = 0.10;                  // floor at 0.1 mi
        if (radMiles > 1.00) radMiles = 1.00;                       // cap at 1.0 mi
        int rad = (int)(radMiles * 1609.34);     // convert to meters for storage
        String label = server.arg(labelKey);

        DynamicJsonDocument geoDoc(256);
        geoDoc["lat"]   = lat;
        geoDoc["lon"]   = lon;
        geoDoc["rad"]   = rad;
        geoDoc["label"] = label;
        String geoStr;
        serializeJson(geoDoc, geoStr);

        settings.saveSetting<bool>("geo_" + String(i), geoStr);
        anyChange = true;
      }
    }

    // Device mode
    if (server.hasArg("device_mode") && server.arg("device_mode") != "") {
      int mode_arg = SOLO_MODE;
      if (server.arg("device_mode") == "core") mode_arg = CORE_MODE;
      else if (server.arg("device_mode") == "node") mode_arg = NODE_MODE;
      this->run_mode = mode_arg;
      settings.saveSetting<bool>("m", this->run_mode, true);
      anyChange = true;
    }

    // Encryption
    if (server.hasArg("enow_key") && server.arg("enow_key") != "") {
      this->esp_now_key = server.arg("enow_key");
      settings.saveSetting<bool>("ek", this->esp_now_key);
      anyChange = true;
    }
    if (server.hasArg("use_encryption")) {
      this->use_encryption = (server.arg("use_encryption") == "true");
      settings.saveSetting<bool>("e", this->use_encryption);
      anyChange = true;
    }

    if (anyChange)
      Logger::log(GUD_MSG, "Settings saved successfully");
    else
      Logger::log(WARN_MSG, "Save called but no changes detected");

    // Chunk 5: invalidate geofence cache so changes take effect immediately
    this->reloadGeofenceCache();

    server.send(200, "text/html",
      "<html><body><h3>Settings saved.</h3>"
      "<a href=\"/\">Back</a></body></html>");

    this->last_web_client_activity = 0;
    this->shutdownAccessPoint();
  });

  server.on("/download", HTTP_GET, [this]() {
    if (!this->checkAuth()) return;
    this->last_web_client_activity = millis();
    if (!server.hasArg("file")) {
      server.send(400, "text/plain", "Missing file parameter.");
      return;
    }

    String path = server.arg("file");
    Logger::log(GUD_MSG, "User downloading: " + path);
    if (!SD.exists("/" + path)) {
      server.send(404, "text/plain", "File not found.");
      return;
    }

    File downloadFile = SD.open("/" + path, FILE_READ);
    if (downloadFile) {
      server.sendHeader("Content-Disposition", "attachment; filename=\"" + path + "\"");
      server.streamFile(downloadFile, "application/octet-stream");
    }
    else
      Logger::log(GUD_MSG, "Failed to open file: " + path);
    downloadFile.close();
  });

  // ---- GET /upload : per-service upload with sidecar tracking ----
  server.on("/upload", HTTP_GET, [this]() {
    if (!this->checkAuth()) return;
    this->last_web_client_activity = millis();

    if (!server.hasArg("file")) {
      server.send(400, "text/plain", "Missing file parameter.");
      return;
    }

    String filePath = "/" + server.arg("file");
    if (!SD.exists(filePath)) {
      server.send(404, "text/plain", "File not found: " + filePath);
      return;
    }

    // ?svc=wigle|wdg|both  ?retry=1
    String svc   = server.hasArg("svc")   ? server.arg("svc")   : "both";
    bool   retry = server.hasArg("retry") && server.arg("retry") == "1";

    bool wigle_ok = true;
    bool wdg_ok   = true;

    if (svc == "wigle") {
      bool already = !retry && this->sidecarExists(filePath, "wigle");
      if (!already) {
        wigle_ok = this->wigleUpload(filePath);
        if (wigle_ok) this->writeSidecar(filePath, "wigle");
      }
    } else if (svc == "wdg") {
      bool already = !retry && this->sidecarExists(filePath, "wdg");
      if (!already) {
        wdg_ok = this->wdgwarsUpload(filePath);
        if (wdg_ok) this->writeSidecar(filePath, "wdg");
      }
    } else {
      // both
      bool ok = this->uploadFile(filePath, retry, BOTH_UPLOAD);
      wigle_ok = ok;
      wdg_ok   = ok;
    }

    String result = "<html><body><h3>Upload result for " + filePath + "</h3>";
    result += "WiGLE: " + String(wigle_ok ? "&#10003; OK" : "&#10007; Failed") + "<br>";
    result += "WDG Wars: " + String(wdg_ok  ? "&#10003; OK" : "&#10007; Failed") + "<br><br>";
    result += "<a href=\"/\">Back</a></body></html>";

    server.send(200, "text/html", result);
    this->last_web_client_activity = millis();
  });

  // ---- GET /log : live log viewer ----
  server.on("/log", HTTP_GET, [this]() {
    this->last_web_client_activity = millis();

    // Build log output oldest-first from the ring buffer
    String logHtml = "<!DOCTYPE html><html><head>";
    logHtml += "<meta charset='utf-8'>";
    logHtml += "<meta http-equiv='refresh' content='5'>";
    logHtml += "<title>C5 Wardriver Log</title>";
    logHtml += "<style>";
    logHtml += "body{background:#000;color:#0f0;font-family:monospace;font-size:12px;padding:8px;}";
    logHtml += ".warn{color:#ff0;} .good{color:#0f0;} .std{color:#aaa;}";
    logHtml += "a{color:#08f;}";
    logHtml += "</style></head><body>";
    logHtml += "<b>C5 Wardriver — Live Log</b> ";
    logHtml += "<a href='/'>&#8592; Back</a><br>";
    logHtml += "<small>Auto-refreshes every 5s. Showing last " +
    String(Logger::ring_count) + "/" +
    String(LOG_RING_SIZE) + " lines.</small><hr>";

    // Walk ring buffer oldest-first
    int start = (Logger::ring_count < LOG_RING_SIZE)
    ? 0
    : Logger::ring_head;

    for (int i = 0; i < Logger::ring_count; i++) {
      int idx = (start + i) % LOG_RING_SIZE;
      String line = Logger::ring[idx];

      String css = "std";
      if (line.startsWith("[!]")) css = "warn";
      else if (line.startsWith("[+]")) css = "good";

      logHtml += "<span class='" + css + "'>" + line + "</span><br>";
    }

    logHtml += "</body></html>";
    server.send(200, "text/html", logHtml);
  });
  server.begin();
}

bool WiFiOps::monitorAP(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    this->showCountdown();
    delay(100);
    if (WiFi.softAPgetStationNum() > 0) {
      Logger::log(GUD_MSG, "Client connected.");
      return true;
    }
  }

  Logger::log(STD_MSG, "Timeout reached with no client.");
  shutdownAccessPoint();
  return false;
}

void WiFiOps::shutdownAccessPoint(bool ap_active) {
  if ((ap_active) && (WiFi.status() != WL_CONNECTED))
    Logger::log(STD_MSG, "Shutting down Access Point and Web Server...");
  else
    Logger::log(STD_MSG, "Shutting down Web Server...");

  server.stop();
  if (ap_active) {
    if (WiFi.status() != WL_CONNECTED)
      WiFi.softAPdisconnect(true);  // true = wipe SSID/password
  }
  delay(100);  // small delay for stability

  this->serving = false;
}

void WiFiOps::showCountdown() {
  if (millis() - this->last_timer > TIMER_UPDATE) {
    this->last_timer = millis();
    display.tft->fillRect(0, SMALL_CHAR_HEIGHT * 2, TFT_WIDTH, TFT_HEIGHT - (SMALL_CHAR_HEIGHT * 2), ST77XX_BLACK);
    display.tft->setCursor(0, SMALL_CHAR_HEIGHT * 4);
    display.tft->println("Wardring starts...\n");
    display.tft->setTextSize(2);
    display.tft->println(60 - ((millis() - this->last_web_client_activity) / 1000));
    display.tft->setTextSize(1);
  }
}

bool WiFiOps::begin(bool skip_admin, int mode_override) {
  this->current_scan_mode = WIFI_STANDBY;
  bool boot_dock_handoff = false;

  esp_reset_reason_t reset_reason = esp_reset_reason();
  if (reset_reason == ESP_RST_POWERON ||
      reset_reason == ESP_RST_BROWNOUT ||
      reset_reason == ESP_RST_UNKNOWN) {
    rtc_dock_done = false;
  }

  // Holding SELECT at boot means "resume wardriving" — latch the dock so the
  // first scan doesn't immediately re-dock on the home/trigger SSID.
  if (skip_admin) {
    rtc_dock_done = true;
  }

  this->run_mode = settings.loadSetting<int>("m");
  if (mode_override != 0) this->run_mode = mode_override;
  this->gps_buffering_enabled = settings.loadSetting<bool>(GPS_BUFFER_NAME);

  if (!skip_admin) {
    // Init WiFi
    this->initWiFi();

    // Run Admin stuff and wait for clients first
    bool connected = this->tryConnectToWiFi();

    this->connected_as_client = connected;

    if (!connected) {
      delay(1000);
      this->startAccessPoint();
    }

    // Boot-time dock upload now runs earlier via tryBootDockUpload() (minimal-heap
    // path in setup(), before the memory-heavy subsystems init) and reboots here,
    // so by the time begin() runs the pending logs are already synced.

    this->serveConfigPage();

    this->serving = true;

    this->last_web_client_activity = millis();

    // Run AP loop
    if (!connected) {
      if (this->monitorAP()) {
        while (true) {
          server.handleClient();

          static bool wasConnected = WiFi.softAPgetStationNum() > 0;
          bool nowConnected = WiFi.softAPgetStationNum() > 0;

          if (wasConnected && !nowConnected) {
            Logger::log(STD_MSG, "Client disconnected.");
            this->shutdownAccessPoint();
            break;
          }

          wasConnected = nowConnected;
        }
      }
    } else { // Or run web server loop
      this->last_timer = millis();

      while (this->serving) {
        server.handleClient();
        battery.main(millis());
        gps.main();

        // SEL hands the boot-time dock off to the runtime dock monitor so the
        // main loop runs and the on-device Dock Menu becomes usable (SOLO only).
        if (this->run_mode == SOLO_MODE && c_btn.justPressed()) {
          Logger::log(STD_MSG, "[DOCK] SEL at boot dock — handing off to runtime monitor");
          this->dock_ip            = WiFi.localIP().toString();
          this->dock_state         = DOCK_STATE_MONITORING;
          this->dock_last_scan_time = millis();
          this->dock_depart_count  = 0;
          this->dock_autoopen_menu = true;
          boot_dock_handoff        = true;
          break;
        }

        if (millis() - this->dock_last_ui_time >= DOCK_UI_REFRESH_MS) {
          this->dock_last_ui_time = millis();
          this->drawServingScreen();
        }

        // Stay connected until K1T actually disappears — no inactivity
        // timeout when connected as a client to a known WLAN.
        if (WiFi.status() != WL_CONNECTED) {
          Logger::log(WARN_MSG, "WiFi connection lost — resuming wardriving...");
          this->shutdownAccessPoint(false);
          break;
        }
      }
    }

    if (!boot_dock_handoff) {
      this->connected_as_client = false;
      this->deinitWiFi();
    }
  }

  // Sanity check for modes and keys
  if (this->esp_now_key != "") {
    if (!settings.saveSetting<bool>("ek", this->esp_now_key))
      Logger::log(WARN_MSG, "Failed to save setting");
  }
  else
    this->esp_now_key = settings.loadSetting<String>("ek");

  //this->run_mode = settings.loadSetting<int>("m");
  this->use_encryption = settings.loadSetting<bool>("e");

  Logger::log(STD_MSG, "ENOW Key: " + this->esp_now_key);

  if (this->run_mode == SOLO_MODE)
    Logger::log(STD_MSG, "Mode: SOLO");
  if (this->run_mode == NODE_MODE)
    Logger::log(STD_MSG, "Mode: NODE");
  if (this->run_mode == CORE_MODE)
    Logger::log(STD_MSG, "Mode: CORE");

  if (this->use_encryption)
    Logger::log(STD_MSG, "Encryption: Enabled");
  else
    Logger::log(STD_MSG, "Encryption: Disabled");

  // Free the boot-upload TLS arena regardless of path
  this->freeTlsGuard();

  // Skip the wardrive transition when handing the boot dock off to the runtime
  // monitor — keep the STA connection and web server up for the Dock Menu.
  if (!boot_dock_handoff) {
    this->initWiFi();

    // Init NimBLE
    this->initBLE(); // NimBLE needs to not be init in order to upload to wigle

    if (this->run_mode != SOLO_MODE)
      this->startESPNow();

    if ((this->run_mode == SOLO_MODE) || (this->run_mode == CORE_MODE))
      startLog(LOG_FILE_NAME);

    // Random delay for nodes to stagger channels
    if (this->run_mode == NODE_MODE) {
      delay(1000);
      this->runAdminWindowAfterScanCycle();
      delay(random(100, 5000));
    }
  }

  this->init_time = millis();

  // Chunk 5: load geofence cache now that settings are fully initialised
  this->loadGeofenceCache();

  // Force Screen 1 redraw to clear any AP/admin phase display remnants
  g_force_display_redraw = true;

  return true;
}

// ============================================================
// Chunk 6: Dock mode state machine
// ============================================================

// Synchronous passive scan for the configured trigger SSID.
// Safe to call while in promiscuous mode (cancels ongoing async scan first).
// Returns true if the trigger SSID is visible.
// ---- Docking-network helpers (multi-dock) -------------------------------
String WiFiOps::dockSSID(int i) {
  if (i < 0 || i >= MAX_DOCK_SSIDS) return "";
  return this->dock_ssid_cache[i];
}
String WiFiOps::dockPass(int i) {
  if (i < 0 || i >= MAX_DOCK_SSIDS) return "";
  return this->dock_pass_cache[i];
}
int WiFiOps::matchDockSSID(const String& ssid) {
  if (ssid.isEmpty()) return -1;
  for (int i = 0; i < MAX_DOCK_SSIDS; i++)
    if (this->dock_ssid_cache[i].length() && this->dock_ssid_cache[i] == ssid) return i;
  return -1;
}
bool WiFiOps::anyDockConfigured() {
  for (int i = 0; i < MAX_DOCK_SSIDS; i++)
    if (this->dock_ssid_cache[i].length()) return true;
  return false;
}
void WiFiOps::loadDockCache() {
  for (int i = 0; i < MAX_DOCK_SSIDS; i++) {
    this->dock_ssid_cache[i] = settings.loadSetting<String>(String(DOCK_SSID_PREFIX) + i);
    this->dock_pass_cache[i] = settings.loadSetting<String>(String(DOCK_PASS_PREFIX) + i);
  }
}
void WiFiOps::migrateDockSSIDs() {
  String legacy = settings.loadSetting<String>(TRIGGER_SSID_NAME);
  if (legacy.isEmpty()) return;                              // nothing to migrate
  if (!this->dock_ssid_cache[0].isEmpty()) return;           // slot 0 already set — don't clobber
  String legacyPass = settings.loadSetting<String>(TRIGGER_PASS_NAME);
  settings.saveSetting<bool>(String(DOCK_SSID_PREFIX) + "0", legacy);
  settings.saveSetting<bool>(String(DOCK_PASS_PREFIX) + "0", legacyPass);
  Logger::log(GUD_MSG, "[DOCK] Migrated legacy trigger '" + legacy + "' into docking slot 1");
}
void WiFiOps::initDockConfig() {
  this->loadDockCache();      // ensures ds_/dp_ keys exist (as String) and caches them
  this->migrateDockSSIDs();   // fold legacy single trigger into slot 0 if empty
  this->loadDockCache();      // refresh cache with any migrated value
}

// Scan for any configured docking network. Records the strongest match's RSSI
// (trigger_last_rssi) and index (dock_matched_idx). Returns true if any present.
bool WiFiOps::scanForTriggerSSID() {
  if (!this->anyDockConfigured()) return false;

  this->stopPromiscuousCapture();

  // Don't interrupt a running async scan — skip this cycle
  if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) return false;
  WiFi.scanDelete();

  int n = WiFi.scanNetworks(false, true, false, 100); // synchronous, include hidden

  bool found = false;
  this->trigger_last_rssi = -127;
  this->dock_matched_idx  = -1;
  for (int i = 0; i < n; i++) {
    int di = this->matchDockSSID(WiFi.SSID(i));
    if (di >= 0) {
      found = true;
      int rs = WiFi.RSSI(i);
      if (rs > this->trigger_last_rssi) { this->trigger_last_rssi = rs; this->dock_matched_idx = di; }
    }
  }
  WiFi.scanDelete();
  return found;
}

// Scan and connect to a present docking network (strongest wins), using its own
// password. Falls back to the boot Network (s/p) if no docking net is in range.
bool WiFiOps::connectForUpload() {
  this->stopPromiscuousCapture();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setBandMode(WIFI_BAND_MODE_AUTO);
  if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) delay(200);
  WiFi.scanDelete();

  int n = WiFi.scanNetworks(false, true, false, 100);
  int bestDock = -1, bestRssi = -999;
  String userSSID = settings.loadSetting<String>("s");
  bool userSeen = false;
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    int rs = WiFi.RSSI(i);
    int di = this->matchDockSSID(s);
    if (di >= 0 && rs > bestRssi) { bestRssi = rs; bestDock = di; }
    if (!userSSID.isEmpty() && s == userSSID) userSeen = true;
  }
  WiFi.scanDelete();

  String ssid, pass;
  if (bestDock >= 0) {
    ssid = this->dockSSID(bestDock); pass = this->dockPass(bestDock);
    // If this docking net has no password of its own but matches the Network,
    // reuse the Network password (covers the legacy single-trigger migration).
    if (pass.isEmpty() && ssid == userSSID) pass = settings.loadSetting<String>("p");
    Logger::log(STD_MSG, "[DOCK] Upload via docking net: " + ssid + " (" + String(bestRssi) + ")");
  } else if (userSeen) {
    ssid = userSSID; pass = settings.loadSetting<String>("p");
    Logger::log(STD_MSG, "[DOCK] No docking net present — fallback to Network: " + ssid);
  } else {
    Logger::log(WARN_MSG, "[DOCK] No docking net or Network in range — skipping upload");
    return false;
  }

  this->user_ap_ssid = ssid;
  this->wigle_user   = settings.loadSetting<String>("wu");
  this->wigle_token  = settings.loadSetting<String>("wt");

  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < STATION_CONNECT_TIMEOUT) {
    delay(500); Serial.print(".");
  }
  return WiFi.status() == WL_CONNECTED;
}

// Debounce dock entry: require DOCK_ARM_SIGHTINGS consecutive sightings of the
// trigger SSID at >= DOCK_RSSI_MIN before committing (a single/weak beacon must
// not flip the device into dock mode, which costs a reboot).
bool WiFiOps::armDock(bool seen, int rssi) {
  if (seen && rssi >= DOCK_RSSI_MIN) {
    this->dock_arm_count++;
    if (this->dock_arm_count >= DOCK_ARM_SIGHTINGS) {
      this->dock_arm_count = 0;
      return true;
    }
    Logger::log(STD_MSG, "[DOCK] Trigger seen rssi=" + String(rssi) +
                " arming " + String(this->dock_arm_count) + "/" + String(DOCK_ARM_SIGHTINGS));
    return false;
  }
  if (this->dock_arm_count > 0)
    Logger::log(STD_MSG, "[DOCK] Trigger weak/absent (rssi=" + String(rssi) + ") — arm reset");
  this->dock_arm_count = 0;
  return false;
}

// Dispatcher — called from main() when dock_state != DOCK_STATE_NONE.
void WiFiOps::runDockMode(uint32_t currentTime) {
  switch (this->dock_state) {
    case DOCK_STATE_CONNECTING:
      Logger::log(STD_MSG, "[DOCK] Trigger detected, rebooting into upload path");
      delay(50);
      Settings::safeRestart();
      break;
    case DOCK_STATE_UPLOADING:
      this->handleDockUploading();
      break;
    case DOCK_STATE_MONITORING:
      this->handleDockMonitoring(currentTime);
      break;
    case DOCK_STATE_FAILED:
      // Wait for failure display timeout then resume wardriving
      if (currentTime - this->dock_fail_time >= DOCK_FAIL_DISPLAY_MS) {
        Logger::log(STD_MSG, "[DOCK] Failure display expired — resuming wardrive");
        this->dock_state            = DOCK_STATE_NONE;
        this->dock_connect_attempts = 0;
        this->initWiFi();
        this->initBLE();
        display.clearScreen();
      }
      break;
    default:
      this->dock_state = DOCK_STATE_NONE;
      break;
  }
}

// One blocking connection attempt to the trigger SSID.
// Retries up to DOCK_CONNECT_ATTEMPTS times across successive main() cycles.
void WiFiOps::handleDockConnecting() {
  String trigSSID = settings.loadSetting<String>(TRIGGER_SSID_NAME);
  String trigPass = settings.loadSetting<String>(TRIGGER_PASS_NAME);

  Logger::log(STD_MSG, "[DOCK] Connect attempt " +
              String(this->dock_connect_attempts + 1) + "/" +
              String(DOCK_CONNECT_ATTEMPTS) + " to: " + trigSSID);

  display.clearScreen();
  display.tft->setCursor(0, 0);
  display.tft->setTextSize(2);
  display.tft->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  display.tft->println("DOCK MODE");
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->println("Connecting");
  display.tft->println(trigSSID);
  display.tft->setTextSize(1);
  display.tft->println("Attempt " +
  String(this->dock_connect_attempts + 1) + "/3");

  // Switch WiFi from scan/promiscuous to STA client
  this->deinitBLE();
  this->deinitWiFi();
  this->initWiFi();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setBandMode(WIFI_BAND_MODE_AUTO);
  WiFi.begin(trigSSID.c_str(), trigPass.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - start < DOCK_CONNECT_TIMEOUT) {
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    this->dock_ip = WiFi.localIP().toString();
    Logger::log(GUD_MSG, "[DOCK] Connected! IP: " + this->dock_ip);

    // Show dock banner on TFT
    display.clearScreen();
    display.tft->setCursor(0, 0);
    display.tft->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    display.tft->println("DOCKED");
    display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display.tft->println(trigSSID);
    display.tft->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    display.tft->println(this->dock_ip);
    display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display.tft->println("browse to config");
    display.tft->println("GPS: " + gps.getFixStatusAsString() + " (" + gps.getNumSatsString() + " sat)");
    if (battery.i2c_supported)
      display.tft->println("Batt: " + String(battery.getBatteryLevel()) + "%");

    // Start web server so you can access config/files while docked
    this->serveConfigPage();
    this->serving = true;
    this->last_web_client_activity = millis();

    if (this->dock_webui_only) {
      // Tier 1: no GPS fix — serve web UI only, skip upload
      Logger::log(STD_MSG, "[DOCK] Tier 1 — web UI only (no GPS fix)");
      display.tft->setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      display.tft->println("No GPS - web UI only");
      this->dock_state = DOCK_STATE_MONITORING;
    } else {
      // Tier 2: GPS fix — full dock mode
      this->dock_state = DOCK_STATE_UPLOADING;
    }

  } else {
    WiFi.disconnect(true);
    this->dock_connect_attempts++;
    Logger::log(WARN_MSG, "[DOCK] Attempt " +
                String(this->dock_connect_attempts) + " failed");

    if (this->dock_connect_attempts >= DOCK_CONNECT_ATTEMPTS) {
      Logger::log(WARN_MSG, "[DOCK] All attempts exhausted — resuming wardrive in " +
                  String(DOCK_FAIL_DISPLAY_MS / 1000) + "s");

      display.clearScreen();
      display.tft->setCursor(0, 0);
      display.tft->setTextColor(ST77XX_RED, ST77XX_BLACK);
      display.tft->println("DOCK FAILED");
      display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      display.tft->println(trigSSID);
      display.tft->println("Resuming in");
      display.tft->println(String(DOCK_FAIL_DISPLAY_MS / 1000) + "s");

      this->dock_state    = DOCK_STATE_FAILED;
      this->dock_fail_time = millis();
    }
    // else: stay in DOCK_STATE_CONNECTING — next main() cycle retries
  }
}

// Run once after successful connection: upload all pending files
// then transition to monitoring.
void WiFiOps::handleDockUploading() {
  Logger::log(STD_MSG, "[DOCK] Starting bulk upload of pending files");

  display.clearScreen();
  display.tft->setCursor(0, 0);
  display.tft->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  display.tft->println("UPLOADING");
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->println(this->dock_ip);

  this->uploadAllPending(); // Chunk 3

  Logger::log(GUD_MSG, "[DOCK] Upload complete — monitoring for departure");

  String trigSSID = settings.loadSetting<String>(TRIGGER_SSID_NAME);

  display.clearScreen();
  display.tft->setCursor(0, 0);
  display.tft->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  display.tft->println("SYNCED");
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->println(this->dock_ip);
  display.tft->println("Watching for");
  display.tft->println("departure...");

  this->dock_last_scan_time = millis();
  this->dock_depart_count   = 0;
  this->dock_state          = DOCK_STATE_MONITORING;
}

// Called every main() cycle while docked.
// Services web server and runs passive scan every DOCK_SCAN_INTERVAL.
// Departs when trigger SSID is absent for DOCK_DEPART_SCANS consecutive scans.
void WiFiOps::drawServingScreen() {
  display.clearScreen();
  display.tft->setCursor(0, 0);
  display.tft->setTextSize(2);
  display.tft->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  display.tft->println("CONFIG");
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->println(WiFi.localIP().toString());
  display.tft->println("GPS:" + String(gps.getFixStatus() ? "Y" : "N") + " Sat:" + gps.getNumSatsString());
  if (battery.i2c_supported)
    display.tft->println("Bat: " + String(battery.getBatteryLevel()) + "%");
  display.tft->setTextSize(1);
  display.tft->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  display.tft->setCursor(0, TFT_HEIGHT - 8);
  display.tft->print("SEL=Menu");
}

void WiFiOps::drawDockMonitorScreen() {
  display.clearScreen();
  display.tft->setCursor(0, 0);
  display.tft->setTextSize(2);
  display.tft->setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  display.tft->println("DOCKED");
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->println(this->dock_ip);
  display.tft->println("GPS:" + String(gps.getFixStatus() ? "Y" : "N") + " Sat:" + gps.getNumSatsString());
  if (battery.i2c_supported)
    display.tft->println("Bat: " + String(battery.getBatteryLevel()) + "%");
  display.tft->setTextSize(1);
  display.tft->setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  display.tft->setCursor(0, TFT_HEIGHT - 8);
  display.tft->print("SEL=Menu");
}

void WiFiOps::handleDockMonitoring(uint32_t currentTime) {
  // Service any web browser clients
  server.handleClient();
  // Keep battery main running so chargeRate sampling fires
  battery.main(currentTime);

  // While the on-device Dock Menu is up, the UI owns the screen and the user is
  // interacting — don't redraw the dock screen or scan/depart underneath them.
  if (this->dock_menu_open)
    return;

  if (currentTime - this->dock_last_ui_time >= DOCK_UI_REFRESH_MS) {
    this->dock_last_ui_time = currentTime;
    this->drawDockMonitorScreen();
  }

  // Tier 1 → Tier 2 upgrade: GPS fix acquired while in web-UI-only mode
  if (this->dock_webui_only && gps.getFixStatus() && sd_obj.supported) {
    Logger::log(GUD_MSG, "[DOCK] GPS fix acquired — upgrading Tier 1 -> Tier 2");
    this->dock_webui_only = false;
    this->dock_depart_time = millis();
    this->handleDockUploading();
    return;
  }

  if (currentTime - this->dock_last_scan_time < DOCK_SCAN_INTERVAL)
    return; // not time yet

  this->dock_last_scan_time = currentTime;
  Logger::log(STD_MSG, "[DOCK] Passive scan — checking for trigger SSID");

  bool found = this->scanForTriggerSSID();

  if (found) {
    Logger::log(STD_MSG, "[DOCK] Trigger SSID still visible — staying docked");
    this->dock_depart_count = 0;

  } else {
    this->dock_depart_count++;
    Logger::log(STD_MSG, "[DOCK] Trigger SSID absent (" +
                String(this->dock_depart_count) + "/" +
                String(DOCK_DEPART_SCANS) + ")");

    if (this->dock_depart_count >= DOCK_DEPART_SCANS)
      this->departDock();
  }
}

// Clean teardown: stop web server, disconnect WiFi,
// reinitialise for wardriving, start a fresh log file.
void WiFiOps::departDock() {
  Logger::log(STD_MSG, "[DOCK] Departing — tearing down dock mode");

  // Stop web server (keep STA mode, AP was never started)
  this->shutdownAccessPoint(false);

  // Disconnect from trigger SSID
  WiFi.disconnect(true);
  delay(100);

  // Reinitialise WiFi and BLE for wardriving
  this->deinitWiFi();
  this->initWiFi();
  this->initBLE();

  // Fresh log file so post-dock drive gets its own file
  this->startLog(LOG_FILE_NAME);

  // Reset all dock state
  this->dock_state            = DOCK_STATE_NONE;
  this->dock_connect_attempts = 0;
  this->dock_depart_count     = 0;
  this->dock_ip               = "";
  this->dock_webui_only       = false;
  g_force_display_redraw = true;

  Logger::log(GUD_MSG, "[DOCK] Departed — wardriving resumed");

  display.clearScreen();
  display.tft->setCursor(0, 0);
  display.tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.tft->println("WARDRIVE");
  display.tft->println("RESUMED");
}

void WiFiOps::startWardrivingFromDock() {
  // User chose to wardrive from the Dock Menu — latch the dock so the standby
  // scan doesn't immediately re-dock on the still-present trigger SSID.
  rtc_dock_done = true;
  this->dock_menu_open = false;
  this->departDock();
}

// ============================================================

void WiFiOps::main(uint32_t currentTime, bool in_sd_files) {
  // Chunk 6: dock mode takes priority over normal wardrive cycle
  if (this->dock_state != DOCK_STATE_NONE) {
    this->runDockMode(currentTime);
    return;
  }

  if (this->current_scan_mode == WIFI_WARDRIVING)
    this->runWardrive(currentTime);

  if ((this->run_mode == NODE_MODE) && (!g_have_core) && (this->use_encryption)) {
    if (currentTime - g_last_req_ms >= g_req_interval_ms) {
      g_last_req_ms = currentTime;
      this->sendCoreRequest();

      // simple backoff: double up to max
      uint32_t nextInterval = g_req_interval_ms * 2;
      g_req_interval_ms = (nextInterval > REQ_MAX_MS) ? REQ_MAX_MS : nextInterval;
    }
    return;
  }
  // Chunk 6: Standby K1T detection — runs even without GPS fix.
  // Tier 1 (no GPS): connect + web UI only.
  // Tier 2 (GPS fix): full dock mode with upload.
  if (this->current_scan_mode == WIFI_STANDBY &&
    this->dock_state == DOCK_STATE_NONE &&
    this->run_mode == SOLO_MODE &&
    !in_sd_files) {
    if (this->anyDockConfigured() && !rtc_dock_done &&
      currentTime - this->standby_scan_time >= STANDBY_SCAN_INTERVAL &&
      currentTime - this->dock_depart_time >= 60000) { // 60s cooldown after departing
      this->standby_scan_time = currentTime;
      //Logger::log(STD_MSG, "Calling scanForTriggerSSID from main");
      bool seen = this->scanForTriggerSSID();
      if (this->armDock(seen, this->trigger_last_rssi)) {
        Logger::log(STD_MSG, "[DOCK] Docking network confirmed in standby: " + this->dockSSID(this->dock_matched_idx));
        this->dock_webui_only       = !gps.getFixStatus();
        this->dock_state            = DOCK_STATE_CONNECTING;
        this->dock_connect_attempts = 0;
      }
    }
  }

  if (this->run_mode == CORE_MODE) {
    if (currentTime - g_last_debug_print >= DEBUG_OUTPUT_DELAY) {
      g_last_debug_print = currentTime;
      Logger::log(STD_MSG, "Timed Node Table Output: ");
      this->debugPrintNodeTable();
    }
    if (this->removeStaleNodes()) {
      this->handleNodeTopologyChange();
    }
    this->flushAircraftBuffer(currentTime);
  }
}
