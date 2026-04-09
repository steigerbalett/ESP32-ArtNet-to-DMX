/*

  ESP32-ArtNet-to-DMX (+ RDM Extension)

  Receives 2 ArtNet universes via WiFi and outputs them as
  2 physical DMX universes.

  Based on the DMX_write sketch by Mitch Weisbord (https://github.com/someweisguy/esp_dmx)
  and on ArtNetWifiNeoPixel sketch by rstephan (https://github.com/rstephan/ArtnetWifi)

  Original: 2023 - Emanuele Signoretta (https://github.com/signorettae/ESP32-ArtNet-to-DMX)
  Optimized & RDM Extension: 2026

  Improvements over original:
  ─────────────────────────────────────────────────────────────────
  STABILITY:
  - Removed blocking calls from callback → loop() handles DMX sending
  - WiFi reconnect with automatic ESP.restart() on total failure
  - mDNS restart after each reconnect
  - OTA error callback

  PERFORMANCE:
  - memcpy instead of for-loop for data copy
  - Pins as const, data buffers as volatile
  - Buffer overflow protection

  BUGFIXES:
  - receivePinB was identical to receivePinA (GPIO 16) → changed to GPIO 22
    !! Please adapt to your own hardware !!
  - Dead code (universesReceived array) removed

  RDM (E1.20):
  - Periodic discovery on BOTH ports (every 15 seconds)
  - Device data structure with status tracking (active/inactive)
  - PIDs: DeviceInfo, DeviceLabel, SoftwareVersionLabel,
           DmxStartAddress (read+set), IdentifyDevice (read+set)
  - ACK/NACK evaluation with plain-text output
  - Detection of newly added and disappeared devices
  ─────────────────────────────────────────────────────────────────

  HARDWARE REQUIREMENT FOR RDM:
  Bidirectional RS-485 transceiver (e.g. MAX485 or SN75176)
  with a separate DE/RE control pin. receivePin must be correctly wired.

  ─────────────────────────────────────────────────────────────────
  WIFI MODE SELECTION:
  Set WIFI_MODE to one of the three options:

    WIFI_MODE_STA     – Connect to an existing WiFi network
                        (hardcoded credentials below)

    WIFI_MODE_CAPTIVE – Captive portal on first boot (no hardcoded credentials).
                        Opens an AP named ESP-ArtNet-XXYYZZ; connecting device
                        is redirected to a config page. Credentials are stored
                        in flash and reused on subsequent boots.
                        Requires library: https://github.com/tzapu/WiFiManager

    WIFI_MODE_AP      – ESP32 acts as its own Access Point.
                        No external router needed; ArtNet controller connects
                        directly. Fixed IP: 192.168.4.1
                        OTA updates are disabled in this mode.
  ─────────────────────────────────────────────────────────────────
*/

#include <Arduino.h>
#include <esp_dmx.h>
#include <rdm/controller.h>
#include "ArtnetWifi.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ═══════════════════════════════════════════════════════════════
//  WIFI MODE – select one
// ═══════════════════════════════════════════════════════════════
#define WIFI_MODE_STA     0
#define WIFI_MODE_CAPTIVE 1
#define WIFI_MODE_AP      2

#define WIFI_MODE WIFI_MODE_STA   // <── change here to switch mode

#if WIFI_MODE == WIFI_MODE_CAPTIVE
  #include <WiFiManager.h>        // https://github.com/tzapu/WiFiManager
#endif

// ═══════════════════════════════════════════════════════════════
//  CONFIGURATION – adapt here
// ═══════════════════════════════════════════════════════════════

// Credentials – used only in WIFI_MODE_STA
const char* ssid     = "MyArtNetNetwork";
const char* password = "MyArtNetNetwork";

// AP password – used only in WIFI_MODE_AP
const char* apPassword = "artnet123";

char dynamicName[32] = "ESP-ArtNet-XXXXXX";

// DMX pins Port A
const int transmitPinA = 17;
const int receivePinA  = 16;
const int enablePinA   = 4;

// DMX pins Port B
const int transmitPinB = 21;
const int receivePinB  = 22;
const int enablePinB   = 19;

// ArtNet universe offset (0 = Universe 0 → Port A, 1 → Port B)
const int startUniverse = 0;
const int maxUniverses  = 2;

// RDM discovery interval in milliseconds
const unsigned long RDM_DISCOVERY_INTERVAL = 15000;

// Maximum number of RDM devices per port
const int RDM_MAX_DEVICES = 32;

// ═══════════════════════════════════════════════════════════════
//  DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════

// Device record – updated on every discovery run
struct RdmDevice {
  rdm_uid_t uid;
  uint16_t  dmxAddress;
  uint8_t   footprint;       // Number of DMX channels required
  uint16_t  subDeviceCount;
  char      label[33];       // Device label (max 32 chars + \0)
  char      swVersion[33];   // Software version string
  bool      active;          // false = not seen since last scan
};

RdmDevice rdmDevicesA[RDM_MAX_DEVICES];
RdmDevice rdmDevicesB[RDM_MAX_DEVICES];
int rdmCountA = 0;
int rdmCountB = 0;

// ═══════════════════════════════════════════════════════════════
//  GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════

dmx_port_t dmxPortA = 1;
dmx_port_t dmxPortB = 2;

// DMX buffers (volatile: accessed from both callback and loop())
volatile byte dataA[DMX_PACKET_SIZE];
volatile byte dataB[DMX_PACKET_SIZE];

// Flags: set by callback, consumed by loop() to trigger DMX send
volatile bool dmxUpdateA = false;
volatile bool dmxUpdateB = false;

ArtnetWifi artnet;

unsigned long lastRdmDiscovery = 0;
bool rdmInitDone = false;

// ═══════════════════════════════════════════════════════════════
//  HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════

// Return NACK reason as human-readable string
const char* nackReasonStr(uint16_t reason) {
  switch (reason) {
    case 0x0000: return "UNKNOWN_PID";
    case 0x0001: return "FORMAT_ERROR";
    case 0x0002: return "HARDWARE_FAULT";
    case 0x0003: return "PROXY_REJECT";
    case 0x0004: return "WRITE_PROTECT";
    case 0x0005: return "UNSUPPORTED_COMMAND_CLASS";
    case 0x0006: return "DATA_OUT_OF_RANGE";
    case 0x0007: return "BUFFER_FULL";
    case 0x0008: return "PACKET_SIZE_UNSUPPORTED";
    case 0x0009: return "SUB_DEVICE_OUT_OF_RANGE";
    case 0x000A: return "PROXY_BUFFER_FULL";
    default:     return "UNKNOWN";
  }
}

// Print ACK/NACK status for a given PID name
void printAckStatus(const rdm_ack_t& ack, const char* pidName) {
  if (ack.type == RDM_RESPONSE_TYPE_NACK_REASON) {
    Serial.printf("    [NACK] %s → Reason: %s (0x%04X)\n",
                  pidName, nackReasonStr(ack.nack_reason), ack.nack_reason);
  } else if (ack.type == RDM_RESPONSE_TYPE_ACK_TIMER) {
    Serial.printf("    [ACK_TIMER] %s → Wait: %d ms\n",
                  pidName, ack.timer);
  }
  // RDM_RESPONSE_TYPE_ACK = success, no output needed
}

// Connect to WiFi with timeout – used in WIFI_MODE_STA only
bool connectWiFi(unsigned long timeoutMs = 15000) {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeoutMs) {
      Serial.println("\nWiFi timeout!");
      return false;
    }
    Serial.print(".");
    delay(200);
  }
  Serial.println("\nConnected: " + WiFi.localIP().toString());
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  RDM DISCOVERY & DEVICE DATA
// ═══════════════════════════════════════════════════════════════

/*
  Reads all available standard PIDs for a given device.
  Provides detailed diagnostic output including NACK reasons.
*/
void rdmQueryDevice(dmx_port_t port, rdm_uid_t* uid, RdmDevice& dev) {
  rdm_sub_device_t sub = RDM_SUB_DEVICE_ROOT;
  rdm_ack_t ack;

  // ── DeviceInfo (footprint, sub-device count) ────────────────
  rdm_device_info_t info;
  memset(&info, 0, sizeof(info));
  if (rdm_send_get_device_info(port, uid, sub, &info, &ack)) {
    dev.footprint      = info.footprint;
    dev.subDeviceCount = info.sub_device_count;
    Serial.printf("    Footprint: %d channels | Sub-devices: %d\n",
                  dev.footprint, dev.subDeviceCount);
  } else {
    printAckStatus(ack, "GET_DEVICE_INFO");
  }

  // ── Read DMX start address ──────────────────────────────────
  if (rdm_send_get_dmx_start_address(port, uid, sub, &dev.dmxAddress, &ack)) {
    Serial.printf("    DMX start address: %d\n", dev.dmxAddress);
  } else {
    printAckStatus(ack, "GET_DMX_START_ADDRESS");
    dev.dmxAddress = 0;
  }

  // ── Read device label ───────────────────────────────────────
  memset(dev.label, 0, sizeof(dev.label));
  if (rdm_send_get_device_label(port, uid, sub,
                                 dev.label, sizeof(dev.label) - 1, &ack)) {
    Serial.printf("    Label: \"%s\"\n", dev.label);
  } else {
    printAckStatus(ack, "GET_DEVICE_LABEL");
    strncpy(dev.label, "(unknown)", sizeof(dev.label) - 1);
  }

  // ── Read software version label ─────────────────────────────
  memset(dev.swVersion, 0, sizeof(dev.swVersion));
  if (rdm_send_get_software_version_label(port, uid, sub,
                                           dev.swVersion,
                                           sizeof(dev.swVersion) - 1, &ack)) {
    Serial.printf("    SW version: \"%s\"\n", dev.swVersion);
  } else {
    printAckStatus(ack, "GET_SOFTWARE_VERSION_LABEL");
    strncpy(dev.swVersion, "(unknown)", sizeof(dev.swVersion) - 1);
  }

  // ── Query identify status ───────────────────────────────────
  bool identifying = false;
  if (rdm_send_get_identify_device(port, uid, sub, &identifying, &ack)) {
    Serial.printf("    Identify active: %s\n", identifying ? "YES" : "No");
  } else {
    printAckStatus(ack, "GET_IDENTIFY_DEVICE");
  }
}

/*
  Sets the DMX start address of a single device via RDM.
  Returns true on success.
*/
bool rdmSetDmxAddress(dmx_port_t port, rdm_uid_t* uid, uint16_t address) {
  rdm_ack_t ack;
  rdm_sub_device_t sub = RDM_SUB_DEVICE_ROOT;
  bool ok = rdm_send_set_dmx_start_address(port, uid, sub, address, &ack);
  if (!ok) printAckStatus(ack, "SET_DMX_START_ADDRESS");
  return ok;
}

/*
  Enables or disables identify mode on a device.
*/
bool rdmSetIdentify(dmx_port_t port, rdm_uid_t* uid, bool enable) {
  rdm_ack_t ack;
  rdm_sub_device_t sub = RDM_SUB_DEVICE_ROOT;
  bool ok = rdm_send_set_identify_device(port, uid, sub, enable, &ack);
  if (!ok) printAckStatus(ack, "SET_IDENTIFY_DEVICE");
  return ok;
}

/*
  Discovery for one port:
  - Finds all devices via rdm_discover_devices_simple()
  - Marks newly appeared and disappeared devices
  - Fully queries PIDs for any new device
*/
void rdmDiscoverPort(dmx_port_t port, RdmDevice* devices,
                     int& count, const char* portName) {
  Serial.printf("\n[RDM] Discovery on %s...\n", portName);

  rdm_uid_t foundUids[RDM_MAX_DEVICES];
  int found = rdm_discover_devices_simple(port, foundUids, RDM_MAX_DEVICES);
  Serial.printf("[RDM] %s: %d device(s) found\n", portName, found);

  // Mark all previously known devices as inactive
  for (int i = 0; i < count; i++) devices[i].active = false;

  // Process each discovered device
  for (int i = 0; i < found; i++) {
    bool isNew = true;

    // Check if the device was already known (compare UIDs)
    for (int j = 0; j < count; j++) {
      if (memcmp(&devices[j].uid, &foundUids[i], sizeof(rdm_uid_t)) == 0) {
        devices[j].active = true;
        isNew = false;

        // Re-read DMX address – may have changed since last scan
        rdm_ack_t ack;
        uint16_t newAddr = 0;
        if (rdm_send_get_dmx_start_address(port, &foundUids[i],
                                            RDM_SUB_DEVICE_ROOT, &newAddr, &ack)) {
          if (newAddr != devices[j].dmxAddress) {
            Serial.printf("[RDM] %s Device %d: address changed %d → %d\n",
                          portName, j, devices[j].dmxAddress, newAddr);
            devices[j].dmxAddress = newAddr;
          }
        }
        break;
      }
    }

    // New device: query all PIDs and append to array
    if (isNew && count < RDM_MAX_DEVICES) {
      Serial.printf("[RDM] %s New device found: UID " UIDSTR "\n",
                    portName, UID2STR(foundUids[i]));
      RdmDevice& dev = devices[count];
      memset(&dev, 0, sizeof(RdmDevice));
      dev.uid    = foundUids[i];
      dev.active = true;
      rdmQueryDevice(port, &foundUids[i], dev);
      count++;
    }
  }

  // Report devices that have disappeared since last scan
  for (int i = 0; i < count; i++) {
    if (!devices[i].active) {
      Serial.printf("[RDM] %s Device UID " UIDSTR " no longer reachable!\n",
                    portName, UID2STR(devices[i].uid));
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  ARTNET CALLBACK
//  ONLY copy data and set flag – NO dmx_send() here!
//  Blocking calls inside a callback cause packet loss / instability.
// ═══════════════════════════════════════════════════════════════

void onArtNetFrame(uint16_t universe, uint16_t numberOfChannels,
                   uint8_t sequence, uint8_t* dmxData) {

  uint16_t idx = universe - (uint16_t)startUniverse;
  if (idx >= (uint16_t)maxUniverses) return;

  // Clamp to buffer size to prevent overflow
  uint16_t channels = min((uint16_t)(DMX_PACKET_SIZE - 1), numberOfChannels);

  if (universe == (uint16_t)startUniverse) {
    memcpy((void*)&dataA[1], dmxData, channels);
    dmxUpdateA = true;
  } else if (universe == (uint16_t)(startUniverse + 1)) {
    memcpy((void*)&dataB[1], dmxData, channels);
    dmxUpdateB = true;
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 ArtNet→DMX (Optimized + RDM) ===");

  // Initialize DMX buffers to 0 (DMX start byte must be 0)
  memset((void*)dataA, 0, DMX_PACKET_SIZE);
  memset((void*)dataB, 0, DMX_PACKET_SIZE);

  // ── Build unique device name from MAC address ────────────────
  // Uses the last 3 bytes of the STA MAC address.
  // Both the WiFi SSID (AP/captive) and the mDNS/OTA hostname
  // are set to this name, making each device uniquely identifiable.
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(dynamicName, sizeof(dynamicName),
           "ESP-ArtNet-%02X%02X%02X", mac[3], mac[4], mac[5]);
  Serial.println("Device name: " + String(dynamicName));

  // ── WiFi setup (mode-dependent) ─────────────────────────────
#if WIFI_MODE == WIFI_MODE_STA
  // Station mode: connect to an existing network using fixed credentials
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(dynamicName);
  if (!connectWiFi()) {
    Serial.println("WiFi connection failed – restarting in 5 s...");
    delay(5000);
    ESP.restart();
  }

#elif WIFI_MODE == WIFI_MODE_CAPTIVE
  // Captive portal mode: no hardcoded credentials required.
  // On first boot (or when saved credentials are missing),
  // WiFiManager opens an AP named dynamicName and redirects any
  // connecting client to a configuration web page.
  // After the user enters SSID + password, they are saved to flash
  // and the ESP32 reconnects automatically on all subsequent boots.
  WiFiManager wifiManager;
  wifiManager.setHostname(dynamicName);
  if (!wifiManager.autoConnect(dynamicName)) {
    Serial.println("Captive portal: connection failed – restarting in 5 s...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Connected (via captive portal): " + WiFi.localIP().toString());

#elif WIFI_MODE == WIFI_MODE_AP
  // Access Point mode: the ESP32 creates its own WiFi network.
  // No external router is needed; ArtNet controllers connect directly.
  // Fixed gateway IP: 192.168.4.1  – ArtNet target: 192.168.4.255 (broadcast)
  // Note: OTA updates are not available in AP mode.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(dynamicName, apPassword);
  Serial.println("AP started: " + String(dynamicName));
  Serial.println("AP IP:      " + WiFi.softAPIP().toString());
#endif

  // ── mDNS (STA and captive portal only) ──────────────────────
#if WIFI_MODE != WIFI_MODE_AP
  if (MDNS.begin(dynamicName)) {
    Serial.println("mDNS: " + String(dynamicName) + ".local");
  }
#endif

  // ── OTA (STA and captive portal only) ───────────────────────
#if WIFI_MODE != WIFI_MODE_AP
  ArduinoOTA.setHostname(dynamicName);
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "Sketch" : "Filesystem";
    Serial.println("OTA update starting: " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA complete – restarting...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error [%u]: ", error);
    if      (error == OTA_AUTH_ERROR)    Serial.println("Auth error");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin error");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect error");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive error");
    else if (error == OTA_END_ERROR)     Serial.println("End error");
  });
  ArduinoOTA.begin();
#endif

  // ── ArtNet ──────────────────────────────────────────────────
  artnet.setArtDmxCallback(onArtNetFrame);
  artnet.begin(dynamicName);

  // ── DMX driver (esp_dmx v4 API) ─────────────────────────────
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = { {512, "Full Universe"} };
  const int personality_count = 1;

  dmx_driver_install(dmxPortA, &config, personalities, personality_count);
  dmx_driver_install(dmxPortB, &config, personalities, personality_count);

  dmx_set_pin(dmxPortA, transmitPinA, receivePinA, enablePinA);
  dmx_set_pin(dmxPortB, transmitPinB, receivePinB, enablePinB);

  Serial.println("DMX driver installed.");

  // ── Initial RDM discovery ────────────────────────────────────
  rdmDiscoverPort(dmxPortA, rdmDevicesA, rdmCountA, "Port A");
  rdmDiscoverPort(dmxPortB, rdmDevicesB, rdmCountB, "Port B");
  lastRdmDiscovery = millis();
  rdmInitDone = true;

  Serial.println("\n=== System ready ===\n");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {

  // ── WiFi reconnect (STA and captive portal only) ─────────────
#if WIFI_MODE == WIFI_MODE_STA || WIFI_MODE == WIFI_MODE_CAPTIVE
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Connection lost – reconnecting...");
    WiFi.disconnect();
    delay(500);
    if (connectWiFi(10000)) {
      // Restart mDNS after reconnect
      MDNS.end();
      if (MDNS.begin(dynamicName)) {
        Serial.println("[mDNS] Restarted after reconnect.");
      }
    } else {
      Serial.println("[WiFi] Reconnect failed – restarting in 5 s");
      delay(5000);
      ESP.restart();
    }
    return; // restart loop() immediately
  }

  // ── OTA handler (STA and captive portal only) ────────────────
  ArduinoOTA.handle();
#endif

  // ── ArtNet packet handler ────────────────────────────────────
  artnet.read();

  // ── Send DMX when new data is available ─────────────────────
  // Both ports are handled independently without blocking each other.
  // Port B does not wait for dmx_wait_sent on Port A.
  if (dmxUpdateA) {
    dmxUpdateA = false;
    dmx_write(dmxPortA, (const void*)dataA, DMX_PACKET_SIZE);
    dmx_send(dmxPortA);
  }

  if (dmxUpdateB) {
    dmxUpdateB = false;
    dmx_write(dmxPortB, (const void*)dataB, DMX_PACKET_SIZE);
    dmx_send(dmxPortB);
  }

  // ── Periodic RDM discovery ───────────────────────────────────
  if (rdmInitDone &&
      millis() - lastRdmDiscovery > RDM_DISCOVERY_INTERVAL) {
    lastRdmDiscovery = millis();
    rdmDiscoverPort(dmxPortA, rdmDevicesA, rdmCountA, "Port A");
    rdmDiscoverPort(dmxPortB, rdmDevicesB, rdmCountB, "Port B");
  }
}

// ═══════════════════════════════════════════════════════════════
//  PUBLIC HELPER FUNCTIONS
//  Can be called externally (e.g. via Serial commands or MQTT).
// ═══════════════════════════════════════════════════════════════

/*
  Sets the DMX start address of a device by its index in the Port A array.
  Returns true on success.

  Example:
    setDeviceAddressA(0, 100);  // Set first device on Port A to address 100
*/
bool setDeviceAddressA(int deviceIndex, uint16_t address) {
  if (deviceIndex < 0 || deviceIndex >= rdmCountA) return false;
  if (!rdmDevicesA[deviceIndex].active) return false;
  bool ok = rdmSetDmxAddress(dmxPortA, &rdmDevicesA[deviceIndex].uid, address);
  if (ok) rdmDevicesA[deviceIndex].dmxAddress = address;
  return ok;
}

bool setDeviceAddressB(int deviceIndex, uint16_t address) {
  if (deviceIndex < 0 || deviceIndex >= rdmCountB) return false;
  if (!rdmDevicesB[deviceIndex].active) return false;
  bool ok = rdmSetDmxAddress(dmxPortB, &rdmDevicesB[deviceIndex].uid, address);
  if (ok) rdmDevicesB[deviceIndex].dmxAddress = address;
  return ok;
}

/*
  Enables or disables identify mode on a device by its index.

  Example:
    identifyDeviceA(0, true);   // Device 0 on Port A starts blinking
    identifyDeviceA(0, false);  // Identify off
*/
bool identifyDeviceA(int deviceIndex, bool enable) {
  if (deviceIndex < 0 || deviceIndex >= rdmCountA) return false;
  if (!rdmDevicesA[deviceIndex].active) return false;
  return rdmSetIdentify(dmxPortA, &rdmDevicesA[deviceIndex].uid, enable);
}

bool identifyDeviceB(int deviceIndex, bool enable) {
  if (deviceIndex < 0 || deviceIndex >= rdmCountB) return false;
  if (!rdmDevicesB[deviceIndex].active) return false;
  return rdmSetIdentify(dmxPortB, &rdmDevicesB[deviceIndex].uid, enable);
}

/*
  Prints all known RDM devices on both ports to the Serial monitor.
*/
void printRdmDeviceList() {
  Serial.println("\n── RDM Device List ──────────────────────────");
  Serial.printf("Port A: %d device(s)\n", rdmCountA);
  for (int i = 0; i < rdmCountA; i++) {
    Serial.printf("  [%d] UID:" UIDSTR " | Addr:%d | %s | SW:%s | %s\n",
                  i, UID2STR(rdmDevicesA[i].uid),
                  rdmDevicesA[i].dmxAddress, rdmDevicesA[i].label,
                  rdmDevicesA[i].swVersion,
                  rdmDevicesA[i].active ? "ACTIVE" : "OFFLINE");
  }
  Serial.printf("Port B: %d device(s)\n", rdmCountB);
  for (int i = 0; i < rdmCountB; i++) {
    Serial.printf("  [%d] UID:" UIDSTR " | Addr:%d | %s | SW:%s | %s\n",
                  i, UID2STR(rdmDevicesB[i].uid),
                  rdmDevicesB[i].dmxAddress, rdmDevicesB[i].label,
                  rdmDevicesB[i].swVersion,
                  rdmDevicesB[i].active ? "ACTIVE" : "OFFLINE");
  }
  Serial.println("─────────────────────────────────────────────");
}