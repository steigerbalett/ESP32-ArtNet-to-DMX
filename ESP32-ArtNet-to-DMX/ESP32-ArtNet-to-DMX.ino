/*

  ESP32-ArtNet-to-DMX (+ RDM Extension)

  Receives 2 ArtNet universes via WiFi and outputs them as
  2 physical DMX universes.

  Based on the DMX_write sketch by Mitch Weisbord (https://github.com/someweisguy/esp_dmx)
  and on ArtNetWifiNeoPixel sketch by rstephan (https://github.com/rstephan/ArtnetWifi)

  Original: 2023 - Emanuele Signoretta (https://github.com/signorettae/ESP32-ArtNet-to-DMX)
  Optimized & RDM Extension: 2026

  ─────────────────────────────────────────────────────────────────
  WIFI MODE SELECTION:

  On EVERY first boot (or after reset), the device starts a
  password-free Captive Portal AP named "ESP-ArtNet-XXYYZZ".
  Connect to it and you will be redirected (or navigate to
  http://192.168.4.1) to choose:

    • WIFI_MODE_STA  – Connect to an existing WiFi network.
                       Enter SSID + password in the portal.

    • WIFI_MODE_AP   – ESP32 acts as its own Access Point.
                       No external router needed.
                       ArtNet controllers connect directly.
                       Fixed IP: 192.168.4.1

  The choice is stored in NVS (Preferences). To change it later,
  press the BOOT button for > 3 s while the device is running,
  or send "reset-wifi\n" via Serial. The device will reboot into
  the Captive Portal again.

  Compatible with:
    - esp_dmx  4.1.0
    - ESP32 Arduino Core 3.x (IDF 5.x)
  ─────────────────────────────────────────────────────────────────
*/

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_dmx.h>
#include <rdm/controller.h>
#include "ArtnetWifi.h"

// ═══════════════════════════════════════════════════════════════
//  RUNTIME WIFI MODE  (stored in NVS, not a compile-time #define)
// ═══════════════════════════════════════════════════════════════
#define MODE_CAPTIVE 0
#define MODE_STA     1
#define MODE_AP      2

// ═══════════════════════════════════════════════════════════════
//  HARDWARE CONFIGURATION – adapt to your wiring
// ═══════════════════════════════════════════════════════════════

// GPIO used to force Captive Portal on long-press (active LOW, internal pull-up)
// Set to -1 to disable the button feature
#define RESET_WIFI_PIN 0   // BOOT button on most DevKit boards

// DMX pins Port A
const int transmitPinA = 17;
const int receivePinA  = 16;
const int enablePinA   = 4;

// DMX pins Port B
const int transmitPinB = 21;
const int receivePinB  = 22;
const int enablePinB   = 19;

// ArtNet universe offset
const int startUniverse = 0;
const int maxUniverses  = 2;

// RDM discovery interval (ms)
const unsigned long RDM_DISCOVERY_INTERVAL = 15000;

// Maximum RDM devices per port
const int RDM_MAX_DEVICES = 32;

// AP fixed IP (used in MODE_AP and Captive Portal)
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// ═══════════════════════════════════════════════════════════════
//  DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════

struct RdmDevice {
  rdm_uid_t uid;
  uint16_t  dmxAddress;
  uint8_t   footprint;
  uint16_t  subDeviceCount;
  char      label[33];
  char      swVersion[33];
  bool      active;
};

RdmDevice rdmDevicesA[RDM_MAX_DEVICES];
RdmDevice rdmDevicesB[RDM_MAX_DEVICES];
int rdmCountA = 0;
int rdmCountB = 0;

// ═══════════════════════════════════════════════════════════════
//  GLOBAL STATE
// ═══════════════════════════════════════════════════════════════

Preferences prefs;

dmx_port_t dmxPortA = 1;
dmx_port_t dmxPortB = 2;

volatile byte dataA[DMX_PACKET_SIZE];
volatile byte dataB[DMX_PACKET_SIZE];
volatile bool dmxUpdateA = false;
volatile bool dmxUpdateB = false;

ArtnetWifi artnet;

unsigned long lastRdmDiscovery = 0;
bool rdmInitDone = false;

char dynamicName[32] = "ESP-ArtNet-XXXXXX";

// Button debounce
unsigned long btnPressStart = 0;
bool btnWasPressed = false;

// ═══════════════════════════════════════════════════════════════
//  NVS HELPERS
// ═══════════════════════════════════════════════════════════════

// Returns the stored WiFi mode; MODE_CAPTIVE means "not configured yet"
uint8_t loadWifiMode() {
  prefs.begin("wificonf", true);
  uint8_t m = prefs.getUChar("mode", 255);  // 255 = factory default
  prefs.end();
  return m;
}

void saveWifiMode(uint8_t mode) {
  prefs.begin("wificonf", false);
  prefs.putUChar("mode", mode);
  prefs.end();
}

void saveStaCredentials(const String& ssid, const String& pw) {
  prefs.begin("wificonf", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pw", pw);
  prefs.end();
}

String loadStaSsid() {
  prefs.begin("wificonf", true);
  String s = prefs.getString("ssid", "");
  prefs.end();
  return s;
}

String loadStaPassword() {
  prefs.begin("wificonf", true);
  String s = prefs.getString("pw", "");
  prefs.end();
  return s;
}

void clearWifiConfig() {
  prefs.begin("wificonf", false);
  prefs.clear();
  prefs.end();
}

// ═══════════════════════════════════════════════════════════════
//  CAPTIVE PORTAL
//  Opens an open (no-password) AP and serves a config page.
//  The user selects STA or AP mode and optionally enters WiFi
//  credentials. On submit the choice is saved and the device reboots.
// ═══════════════════════════════════════════════════════════════

WebServer portalServer(80);
DNSServer dnsServer;

// Minimal inline HTML for the config page
static const char PORTAL_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 ArtNet – WiFi Setup</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:system-ui,sans-serif;background:#111;color:#eee;
       display:flex;justify-content:center;align-items:center;min-height:100vh;padding:16px}
  .card{background:#1e1e1e;border:1px solid #333;border-radius:12px;
        padding:32px;width:100%;max-width:420px}
  h1{font-size:1.25rem;margin-bottom:4px;color:#fff}
  .sub{color:#888;font-size:.85rem;margin-bottom:24px}
  label{display:block;font-size:.85rem;color:#aaa;margin-bottom:4px;margin-top:14px}
  input,select{width:100%;padding:10px 12px;background:#2a2a2a;border:1px solid #444;
               border-radius:8px;color:#eee;font-size:1rem}
  input:focus,select:focus{outline:none;border-color:#4f98a3}
  .sta-fields{transition:opacity .2s}
  button{margin-top:24px;width:100%;padding:12px;background:#01696f;color:#fff;
         border:none;border-radius:8px;font-size:1rem;cursor:pointer}
  button:hover{background:#0c4e54}
  .badge{display:inline-block;padding:2px 8px;border-radius:99px;font-size:.75rem;
         background:#2a2a2a;border:1px solid #444;color:#888;margin-left:8px}
</style>
</head>
<body>
<div class="card">
  <h1>ESP32 ArtNet &#8594; DMX</h1>
  <p class="sub">WiFi Configuration &mdash; <span id="dn"></span></p>
  <form method="POST" action="/save">
    <label>WiFi Mode</label>
    <select name="mode" id="modeSelect" onchange="toggleSta()">
      <option value="sta">Station (STA) &ndash; connect to existing network</option>
      <option value="ap">Access Point (AP) &ndash; standalone, no router needed</option>
    </select>

    <div class="sta-fields" id="staFields">
      <label>Network SSID</label>
      <input type="text" name="ssid" id="ssid" autocomplete="off" placeholder="My WiFi Network">
      <label>Password</label>
      <input type="password" name="pw" autocomplete="off" placeholder="(leave blank if open)">
    </div>

    <button type="submit">Save &amp; Restart</button>
  </form>
</div>
<script>
  document.getElementById('dn').textContent = '%DEVNAME%';
  function toggleSta(){
    var s=document.getElementById('modeSelect').value==='sta';
    document.getElementById('staFields').style.opacity=s?'1':'.35';
    document.getElementById('ssid').required=s;
  }
  toggleSta();
</script>
</body>
</html>
)rawhtml";

static const char SAVED_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Saved</title>
<style>body{font-family:system-ui,sans-serif;background:#111;color:#eee;
display:flex;justify-content:center;align-items:center;min-height:100vh;text-align:center}
h1{font-size:1.4rem;margin-bottom:8px}p{color:#888}</style></head>
<body><div><h1>&#10003; Settings saved</h1><p>The device is restarting…</p></div></body>
</html>
)rawhtml";

void handlePortalRoot() {
  String page = FPSTR(PORTAL_HTML);
  page.replace("%DEVNAME%", String(dynamicName));
  portalServer.send(200, "text/html", page);
}

void handlePortalSave() {
  String mode = portalServer.arg("mode");
  String ssid = portalServer.arg("ssid");
  String pw   = portalServer.arg("pw");

  if (mode == "ap") {
    saveWifiMode(MODE_AP);
  } else {
    // Default to STA
    saveStaCredentials(ssid, pw);
    saveWifiMode(MODE_STA);
  }

  portalServer.send(200, "text/html", FPSTR(SAVED_HTML));
  delay(1500);
  ESP.restart();
}

// Redirect all unknown requests to the portal root (captive portal trick)
void handlePortalNotFound() {
  portalServer.sendHeader("Location", "http://192.168.4.1/", true);
  portalServer.send(302, "text/plain", "");
}

void runCaptivePortal() {
  Serial.println("[Captive] Starting open AP: " + String(dynamicName));

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, AP_SUBNET);
  // No password → open network
  WiFi.softAP(dynamicName);

  Serial.println("[Captive] AP IP: " + WiFi.softAPIP().toString());

  // DNS: redirect every hostname to our IP
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", AP_IP);

  portalServer.on("/",       HTTP_GET,  handlePortalRoot);
  portalServer.on("/save",   HTTP_POST, handlePortalSave);
  portalServer.onNotFound(handlePortalNotFound);
  portalServer.begin();

  Serial.println("[Captive] Portal ready – connect to AP and open http://192.168.4.1");

  // Block here until the user submits the form (handlePortalSave → ESP.restart())
  while (true) {
    dnsServer.processNextRequest();
    portalServer.handleClient();
  }
}

// ═══════════════════════════════════════════════════════════════
//  WIFI HELPERS
// ═══════════════════════════════════════════════════════════════

bool connectWiFi(const String& ssid, const String& pw, unsigned long timeoutMs = 15000) {
  Serial.print("[WiFi] Connecting to "" + ssid + """);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(dynamicName);
  if (pw.length() > 0) {
    WiFi.begin(ssid.c_str(), pw.c_str());
  } else {
    WiFi.begin(ssid.c_str());
  }
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeoutMs) {
      Serial.println("\n[WiFi] Timeout!");
      return false;
    }
    Serial.print(".");
    delay(200);
  }
  Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  OTA SETUP
// ═══════════════════════════════════════════════════════════════

void setupOTA() {
  ArduinoOTA.setHostname(dynamicName);
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "Sketch" : "Filesystem";
    Serial.println("OTA start: " + type);
  });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA complete."); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%\r", p / (t / 100));
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("OTA error [%u]\n", e);
  });
  ArduinoOTA.begin();
}

// ═══════════════════════════════════════════════════════════════
//  RDM HELPERS
// ═══════════════════════════════════════════════════════════════

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

void printAckStatus(const rdm_ack_t& ack, const char* pidName) {
  if (ack.type == RDM_RESPONSE_TYPE_NACK_REASON) {
    Serial.printf("    [NACK] %s → Reason: %s (0x%04X)\n",
                  pidName, nackReasonStr(ack.nack_reason), ack.nack_reason);
  } else if (ack.type == RDM_RESPONSE_TYPE_ACK_TIMER) {
    Serial.printf("    [ACK_TIMER] %s → Wait: %d ms\n", pidName, ack.timer);
  }
}

void rdmQueryDevice(dmx_port_t port, rdm_uid_t* uid, RdmDevice& dev) {
  rdm_sub_device_t sub = RDM_SUB_DEVICE_ROOT;
  rdm_ack_t ack;

  rdm_device_info_t info;
  memset(&info, 0, sizeof(info));
  if (rdm_send_get_device_info(port, uid, sub, &info, &ack)) {
    dev.footprint      = info.footprint;
    dev.subDeviceCount = info.sub_device_count;
    Serial.printf("    Footprint: %d ch | Sub-devices: %d\n",
                  dev.footprint, dev.subDeviceCount);
  } else { printAckStatus(ack, "GET_DEVICE_INFO"); }

  if (rdm_send_get_dmx_start_address(port, uid, sub, &dev.dmxAddress, &ack)) {
    Serial.printf("    DMX start: %d\n", dev.dmxAddress);
  } else { printAckStatus(ack, "GET_DMX_START_ADDRESS"); dev.dmxAddress = 0; }

  memset(dev.label, 0, sizeof(dev.label));
  if (rdm_send_get_device_label(port, uid, sub, dev.label, sizeof(dev.label) - 1, &ack)) {
    Serial.printf("    Label: \"%s\"\n", dev.label);
  } else { printAckStatus(ack, "GET_DEVICE_LABEL"); strncpy(dev.label, "(unknown)", sizeof(dev.label) - 1); }

  memset(dev.swVersion, 0, sizeof(dev.swVersion));
  if (rdm_send_get_software_version_label(port, uid, sub, dev.swVersion, sizeof(dev.swVersion) - 1, &ack)) {
    Serial.printf("    SW: \"%s\"\n", dev.swVersion);
  } else { printAckStatus(ack, "GET_SW_VERSION"); strncpy(dev.swVersion, "(unknown)", sizeof(dev.swVersion) - 1); }

  bool identifying = false;
  if (rdm_send_get_identify_device(port, uid, sub, &identifying, &ack)) {
    Serial.printf("    Identify: %s\n", identifying ? "YES" : "No");
  } else { printAckStatus(ack, "GET_IDENTIFY_DEVICE"); }
}

bool rdmSetDmxAddress(dmx_port_t port, rdm_uid_t* uid, uint16_t address) {
  rdm_ack_t ack;
  bool ok = rdm_send_set_dmx_start_address(port, uid, RDM_SUB_DEVICE_ROOT, address, &ack);
  if (!ok) printAckStatus(ack, "SET_DMX_START_ADDRESS");
  return ok;
}

bool rdmSetIdentify(dmx_port_t port, rdm_uid_t* uid, bool enable) {
  rdm_ack_t ack;
  bool ok = rdm_send_set_identify_device(port, uid, RDM_SUB_DEVICE_ROOT, enable, &ack);
  if (!ok) printAckStatus(ack, "SET_IDENTIFY_DEVICE");
  return ok;
}

void rdmDiscoverPort(dmx_port_t port, RdmDevice* devices,
                     int& count, const char* portName) {
  Serial.printf("\n[RDM] Discovery on %s...\n", portName);
  rdm_uid_t foundUids[RDM_MAX_DEVICES];
  int found = rdm_discover_devices_simple(port, foundUids, RDM_MAX_DEVICES);
  Serial.printf("[RDM] %s: %d device(s)\n", portName, found);

  for (int i = 0; i < count; i++) devices[i].active = false;

  for (int i = 0; i < found; i++) {
    bool isNew = true;
    for (int j = 0; j < count; j++) {
      if (memcmp(&devices[j].uid, &foundUids[i], sizeof(rdm_uid_t)) == 0) {
        devices[j].active = true;
        isNew = false;
        rdm_ack_t ack;
        uint16_t newAddr = 0;
        if (rdm_send_get_dmx_start_address(port, &foundUids[i], RDM_SUB_DEVICE_ROOT, &newAddr, &ack)) {
          if (newAddr != devices[j].dmxAddress) {
            Serial.printf("[RDM] %s Dev%d: addr %d→%d\n", portName, j, devices[j].dmxAddress, newAddr);
            devices[j].dmxAddress = newAddr;
          }
        }
        break;
      }
    }
    if (isNew && count < RDM_MAX_DEVICES) {
      Serial.printf("[RDM] %s New: UID " UIDSTR "\n", portName, UID2STR(foundUids[i]));
      RdmDevice& dev = devices[count];
      memset(&dev, 0, sizeof(RdmDevice));
      dev.uid    = foundUids[i];
      dev.active = true;
      rdmQueryDevice(port, &foundUids[i], dev);
      count++;
    }
  }
  for (int i = 0; i < count; i++) {
    if (!devices[i].active) {
      Serial.printf("[RDM] %s UID " UIDSTR " lost!\n", portName, UID2STR(devices[i].uid));
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  ARTNET CALLBACK
// ═══════════════════════════════════════════════════════════════

void onArtNetFrame(uint16_t universe, uint16_t numberOfChannels,
                   uint8_t sequence, uint8_t* dmxData) {
  uint16_t idx = universe - (uint16_t)startUniverse;
  if (idx >= (uint16_t)maxUniverses) return;
  uint16_t channels = min((uint16_t)(DMX_PACKET_SIZE - 1), numberOfChannels);
  if (universe == (uint16_t)startUniverse) {
    memcpy((void*)&dataA[1], dmxData, channels);
    dmxUpdateA = true;
  } else {
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
  Serial.println("\n=== ESP32 ArtNet→DMX (Runtime WiFi Config) ===");

  // BOOT / reset button
#if RESET_WIFI_PIN >= 0
  pinMode(RESET_WIFI_PIN, INPUT_PULLUP);
#endif

  // Zero DMX buffers
  memset((void*)dataA, 0, DMX_PACKET_SIZE);
  memset((void*)dataB, 0, DMX_PACKET_SIZE);

  // Build unique device name from MAC
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(dynamicName, sizeof(dynamicName),
           "ESP-ArtNet-%02X%02X%02X", mac[3], mac[4], mac[5]);
  Serial.println("Device name: " + String(dynamicName));

  // ── Determine WiFi mode ──────────────────────────────────────
  uint8_t storedMode = loadWifiMode();
  bool forcePortal   = (storedMode == 255);  // 255 = never configured

  // Check BOOT button: if held for > 3 s, force Captive Portal
#if RESET_WIFI_PIN >= 0
  if (!forcePortal && digitalRead(RESET_WIFI_PIN) == LOW) {
    Serial.println("[Button] BOOT held – checking for long-press...");
    unsigned long held = millis();
    while (digitalRead(RESET_WIFI_PIN) == LOW) {
      if (millis() - held > 3000) {
        Serial.println("[Button] Long-press detected – resetting WiFi config.");
        clearWifiConfig();
        forcePortal = true;
        break;
      }
      delay(50);
    }
  }
#endif

  if (forcePortal || storedMode == MODE_CAPTIVE) {
    // Opens blocking portal; exits only via ESP.restart() after save
    runCaptivePortal();
    return; // never reached
  }

  // ── MODE_STA ─────────────────────────────────────────────────
  if (storedMode == MODE_STA) {
    String ssid = loadStaSsid();
    String pw   = loadStaPassword();
    WiFi.setHostname(dynamicName);
    if (ssid.length() == 0 || !connectWiFi(ssid, pw)) {
      Serial.println("[WiFi] Stored credentials failed – launching portal.");
      clearWifiConfig();
      runCaptivePortal();
      return;
    }
  }

  // ── MODE_AP ──────────────────────────────────────────────────
  if (storedMode == MODE_AP) {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_IP, AP_SUBNET);
    WiFi.softAP(dynamicName);  // open AP, no password
    Serial.println("[AP] Started: " + String(dynamicName));
    Serial.println("[AP] IP: " + WiFi.softAPIP().toString());
  }

  // ── mDNS (STA only) ──────────────────────────────────────────
  if (storedMode == MODE_STA) {
    if (MDNS.begin(dynamicName)) {
      Serial.println("[mDNS] " + String(dynamicName) + ".local");
    }
  }

  // ── OTA (STA only) ───────────────────────────────────────────
  if (storedMode == MODE_STA) {
    setupOTA();
  }

  // ── ArtNet ───────────────────────────────────────────────────
  artnet.setArtDmxCallback(onArtNetFrame);
  artnet.begin(dynamicName);

  // ── DMX driver (esp_dmx 4.1.0 API) ──────────────────────────
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = { {512, "Full Universe"} };
  const int personality_count = 1;

  dmx_driver_install(dmxPortA, &config, personalities, personality_count);
  dmx_driver_install(dmxPortB, &config, personalities, personality_count);

  dmx_set_pin(dmxPortA, transmitPinA, receivePinA, enablePinA);
  dmx_set_pin(dmxPortB, transmitPinB, receivePinB, enablePinB);

  Serial.println("[DMX] Driver installed.");

  // ── Initial RDM discovery ─────────────────────────────────────
  rdmDiscoverPort(dmxPortA, rdmDevicesA, rdmCountA, "Port A");
  rdmDiscoverPort(dmxPortB, rdmDevicesB, rdmCountB, "Port B");
  lastRdmDiscovery = millis();
  rdmInitDone = true;

  Serial.println("\n=== System ready ===\n");
  Serial.println("Tip: Send \"reset-wifi\" via Serial to re-open the config portal.");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {

  // ── Serial command: "reset-wifi" ─────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "reset-wifi") {
      Serial.println("[Serial] Resetting WiFi config – rebooting into portal...");
      clearWifiConfig();
      delay(500);
      ESP.restart();
    } else if (cmd == "rdm-list") {
      printRdmDeviceList();
    }
  }

  uint8_t storedMode = loadWifiMode();

  // ── WiFi reconnect (STA only) ────────────────────────────────
  if (storedMode == MODE_STA && WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost – reconnecting...");
    WiFi.disconnect();
    delay(500);
    String ssid = loadStaSsid();
    String pw   = loadStaPassword();
    if (connectWiFi(ssid, pw, 10000)) {
      MDNS.end();
      if (MDNS.begin(dynamicName)) {
        Serial.println("[mDNS] Restarted.");
      }
    } else {
      Serial.println("[WiFi] Reconnect failed – restarting in 5 s");
      delay(5000);
      ESP.restart();
    }
    return;
  }

  // ── OTA handler (STA only) ───────────────────────────────────
  if (storedMode == MODE_STA) {
    ArduinoOTA.handle();
  }

  // ── ArtNet ───────────────────────────────────────────────────
  artnet.read();

  // ── DMX send ─────────────────────────────────────────────────
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
// ═══════════════════════════════════════════════════════════════

bool setDeviceAddressA(int idx, uint16_t addr) {
  if (idx < 0 || idx >= rdmCountA || !rdmDevicesA[idx].active) return false;
  bool ok = rdmSetDmxAddress(dmxPortA, &rdmDevicesA[idx].uid, addr);
  if (ok) rdmDevicesA[idx].dmxAddress = addr;
  return ok;
}

bool setDeviceAddressB(int idx, uint16_t addr) {
  if (idx < 0 || idx >= rdmCountB || !rdmDevicesB[idx].active) return false;
  bool ok = rdmSetDmxAddress(dmxPortB, &rdmDevicesB[idx].uid, addr);
  if (ok) rdmDevicesB[idx].dmxAddress = addr;
  return ok;
}

bool identifyDeviceA(int idx, bool enable) {
  if (idx < 0 || idx >= rdmCountA || !rdmDevicesA[idx].active) return false;
  return rdmSetIdentify(dmxPortA, &rdmDevicesA[idx].uid, enable);
}

bool identifyDeviceB(int idx, bool enable) {
  if (idx < 0 || idx >= rdmCountB || !rdmDevicesB[idx].active) return false;
  return rdmSetIdentify(dmxPortB, &rdmDevicesB[idx].uid, enable);
}

void printRdmDeviceList() {
  Serial.println("\n── RDM Device List ──────────────────────────");
  Serial.printf("Port A: %d\n", rdmCountA);
  for (int i = 0; i < rdmCountA; i++) {
    Serial.printf("  [%d] UID:" UIDSTR " | Addr:%d | %s | SW:%s | %s\n",
                  i, UID2STR(rdmDevicesA[i].uid),
                  rdmDevicesA[i].dmxAddress, rdmDevicesA[i].label,
                  rdmDevicesA[i].swVersion,
                  rdmDevicesA[i].active ? "ACTIVE" : "OFFLINE");
  }
  Serial.printf("Port B: %d\n", rdmCountB);
  for (int i = 0; i < rdmCountB; i++) {
    Serial.printf("  [%d] UID:" UIDSTR " | Addr:%d | %s | SW:%s | %s\n",
                  i, UID2STR(rdmDevicesB[i].uid),
                  rdmDevicesB[i].dmxAddress, rdmDevicesB[i].label,
                  rdmDevicesB[i].swVersion,
                  rdmDevicesB[i].active ? "ACTIVE" : "OFFLINE");
  }
  Serial.println("─────────────────────────────────────────────");
}
