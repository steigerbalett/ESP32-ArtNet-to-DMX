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
#include <rdm/controller.h>   // RDM-Support
#include "ArtnetWifi.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiUDP UdpSend;
ArtnetWifi artnet;

const char* ssid     = "MyArtNetNetwork";
const char* password = "MyArtNetNetwork";

int transmitPinA = 17;
int receivePinA  = 16;
int enablePinA   = 4;

int transmitPinB = 21;
int receivePinB  = 16;
int enablePinB   = 19;

dmx_port_t dmxPortA = 1;
dmx_port_t dmxPortB = 2;

byte dataA[DMX_PACKET_SIZE];
byte dataB[DMX_PACKET_SIZE];

const int startUniverse    = 0;
const int maxUniverses     = 2;
bool universesReceived[maxUniverses];

// RDM: UID-Array
rdm_uid_t uids[32];

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected: " + WiFi.localIP().toString());

  ArduinoOTA.setHostname("ESP32-ArtNet-to-DMX-Converter");
  ArduinoOTA.begin();
  artnet.setArtDmxCallback(onArtNetFrame);
  artnet.begin("ESP32-ArtNet-to-DMX-Converter");

  // --- esp_dmx v4 API: dmx_driver_install with personality ---
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {512, "Full Universe"}
  };
  int personality_count = 1;

  dmx_driver_install(dmxPortA, &config, personalities, personality_count);
  dmx_driver_install(dmxPortB, &config, personalities, personality_count);

  dmx_set_pin(dmxPortA, transmitPinA, receivePinA, enablePinA);
  dmx_set_pin(dmxPortB, transmitPinB, receivePinB, enablePinB);

  // --- RDM Discovery on Port A ---
  Serial.println("Start RDM Discovery on Port A...");
  int devicesFound = rdm_discover_devices_simple(dmxPortA, uids, 32);
  Serial.printf("RDM device found: %d\n", devicesFound);

  for (int i = 0; i < devicesFound; i++) {
    Serial.printf("Gerät %d: UID " UIDSTR "\n", i, UID2STR(uids[i]));

    rdm_uid_t dest = uids[i];
    rdm_sub_device_t sub = RDM_SUB_DEVICE_ROOT;
    rdm_ack_t ack;

    uint16_t dmxAddr;
    if (rdm_send_get_dmx_start_address(dmxPortA, &dest, sub, &dmxAddr, &ack)) {
      Serial.printf("  DMX-Startadresse: %d\n", dmxAddr);
    }
  }
}

void onArtNetFrame(uint16_t universe, uint16_t numberOfChannels,
                   uint8_t sequence, uint8_t* dmxData) {
  if ((universe - startUniverse) < maxUniverses)
    universesReceived[universe - startUniverse] = 1;

  for (int i = 0; i < numberOfChannels; i++) {
    if (universe == startUniverse)
      dataA[i + 1] = dmxData[i];
    else if (universe == startUniverse + 1)
      dataB[i + 1] = dmxData[i];
  }

  // --- v4 API: dmx_write, dmx_send ---
  dmx_write(dmxPortA, dataA, DMX_PACKET_SIZE);
  dmx_write(dmxPortB, dataB, DMX_PACKET_SIZE);
  dmx_send(dmxPortA);
  dmx_send(dmxPortB);
  dmx_wait_sent(dmxPortA, DMX_TIMEOUT_TICK);
  dmx_wait_sent(dmxPortB, DMX_TIMEOUT_TICK);

  memset(universesReceived, 0, maxUniverses);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    artnet.read();
  }
}