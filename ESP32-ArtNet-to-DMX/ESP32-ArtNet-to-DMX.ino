/*

  ESP32-ArtNet-to-DMX

  This sketch allows you to receive 2 ArtNet universes from a WiFi network
  and to send the received data in 2 physical DMX universes.

  Based on the DMX_write sketch by Mitch Weisbord (https://github.com/someweisguy/esp_dmx)
  and on ArtNetWifiNeoPixel sketch by rstephan (https://github.com/rstephan/ArtnetWifi)

  2023 - Emanuele Signoretta

  Migration to v4 API + RDM

*/

#include <Arduino.h>
#include <esp_dmx.h>
#include <rdm/controller.h>
#include "ArtnetWifi.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid     = "MyArtNetNetwork";
const char* password = "MyArtNetNetwork";
const char* hostname = "ESP32-ArtNet-to-DMX";

const int transmitPinA = 17;
const int receivePinA  = 16;
const int enablePinA   = 4;

const int transmitPinB = 21;
const int receivePinB  = 22;   // CHANGED: was 16 (same as Port A)
const int enablePinB   = 19;

dmx_port_t dmxPortA = 1;
dmx_port_t dmxPortB = 2;

volatile byte dataA[DMX_PACKET_SIZE];
volatile byte dataB[DMX_PACKET_SIZE];

volatile bool dmxUpdateA = false;
volatile bool dmxUpdateB = false;

ArtnetWifi artnet;

const int startUniverse = 0;
const int maxUniverses  = 2;

rdm_uid_t uids[32];

bool connectWiFi(unsigned long timeoutMs = 15000) {
  Serial.print("Connect to WiFi");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeoutMs) {
      Serial.println("\nWiFi-Timeout!");
      return false;
    }
    Serial.print(".");
    delay(200);
  }
  Serial.println("\nConnected: " + WiFi.localIP().toString());
  return true;
}

void onArtNetFrame(uint16_t universe, uint16_t numberOfChannels,
                   uint8_t sequence, uint8_t* dmxData) {

  uint16_t idx = universe - startUniverse;
  if (idx >= maxUniverses) return;

  uint16_t channels = min((uint16_t)(DMX_PACKET_SIZE - 1), numberOfChannels);

  if (universe == (uint16_t)startUniverse) {
    memcpy((void*)&dataA[1], dmxData, channels);
    dmxUpdateA = true;
  } else if (universe == (uint16_t)(startUniverse + 1)) {
    memcpy((void*)&dataB[1], dmxData, channels);
    dmxUpdateB = true;
  }
}

void setup() {
  Serial.begin(115200);

  memset((void*)dataA, 0, DMX_PACKET_SIZE);
  memset((void*)dataB, 0, DMX_PACKET_SIZE);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  if (!connectWiFi()) {
    Serial.println("Restart WiFi-Error...");
    delay(3000);
    ESP.restart();
  }

  if (MDNS.begin(hostname)) {
    Serial.println("mDNS started: " + String(hostname) + ".local");
  }

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA-Update started...");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA-Error [%u]\n", error);
  });
  ArduinoOTA.begin();

  artnet.setArtDmxCallback(onArtNetFrame);
  artnet.begin(hostname);

  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {512, "Full Universe"}
  };
  const int personality_count = 1;

  dmx_driver_install(dmxPortA, &config, personalities, personality_count);
  dmx_driver_install(dmxPortB, &config, personalities, personality_count);

  dmx_set_pin(dmxPortA, transmitPinA, receivePinA, enablePinA);
  dmx_set_pin(dmxPortB, transmitPinB, receivePinB, enablePinB);


  Serial.println("Start RDM Discovery Port A...");
  int devicesFound = rdm_discover_devices_simple(dmxPortA, uids, 32);
  Serial.printf("RDM-GDevice found: %d\n", devicesFound);

  for (int i = 0; i < devicesFound; i++) {
    Serial.printf("Device %d: UID " UIDSTR "\n", i, UID2STR(uids[i]));

    rdm_uid_t dest = uids[i];
    rdm_sub_device_t sub = RDM_SUB_DEVICE_ROOT;
    rdm_ack_t ack;
    uint16_t dmxAddr;

    if (rdm_send_get_dmx_start_address(dmxPortA, &dest, sub, &dmxAddr, &ack)) {
      Serial.printf("  DMX-Startadress: %d\n", dmxAddr);
    }
  }

  Serial.println("System bereit.");
}

void loop() {

  // WiFi-Reconnect
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost – reconnecting...");
    WiFi.disconnect();
    delay(500);
    if (connectWiFi(10000)) {
      MDNS.end();
      MDNS.begin(hostname);
    } else {
      Serial.println("Reconnect failed – restart in 5s");
      delay(5000);
      ESP.restart();
    }
    return;
  }

  ArduinoOTA.handle();
  artnet.read();

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
}
