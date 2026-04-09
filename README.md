# ESP32-ArtNet-to-DMX + RDM

This is a simple ArtNet to DMX converter based on an ESP32 and two MAX485 modules.

Based on the [DMX_write](https://github.com/someweisguy/esp_dmx) sketch by Mitch Weisbord (https://github.com/someweisguy/esp_dmx)

and on [ArtNetWifiNeoPixel](https://github.com/rstephan/ArtnetWifi) sketch by rstephan (https://github.com/rstephan/ArtnetWifi)

Original: 2023 - Emanuele Signoretta (https://github.com/signorettae/ESP32-ArtNet-to-DMX)

This is a brief introduction to the project published on [ElettronicaIN #276](https://futuranet.it/prodotto/n-276-dicembre-2023-gennaio-2024/).

and on [elektormagazine](https://www.elektormagazine.de/articles/esp32-basierter-artnet-zu-dmx-konverter-aktualisieren-sie-ihr-alteres-dmx-gerat)

Per la versione README in italiano clicca [qui](README-ITA.md).



# Hardware

- [ESP32 board](https://store.open-electronics.org/ESPWROOM32_ESP32_ESP-32S_DevelopmentBoard)

- 2 MAX 485 modules

- 2 DMX XLR connectors (female, panel mount)

- DC-DC voltage regulator (5V - 3A)

- USB cable

- DC plug, panel mount

- Platisc box for electronics

### Wiring table for MAX485 and XLR connector

| MAX 485 PIN | DMX Common Name | XLR connector pin |
|:-----------:|:---------------:|:-----------------:|
| GND         | GND/Shield      | 1                 |
| A           | Data +          | 3                 |
| B           | Data -          | 2                 |

### 

### Wiring tables for MAX485 and ESP32

###### Universe A

| ESP32 pin | MAX 485 pin |
|:---------:|:-----------:|
| 5V        | VCC         |
| GND       | GND         |
| 4         | DE          |
| 17        | DI          |
| 16        | RE          |

###### Universe B

| ESP32 pin | MAX 485 pin |
|:---------:|:-----------:|
| 5V        | VCC         |
| GND       | GND         |
| 21        | DE          |
| 19        | DI          |
| 22        | RE          |

## Wiring diagram

![Image deactivate](Media/wiring.png)

----

# Software

## Configuration
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


-------

Feel free to edit the sketch, to improve the functionalities and to send pull requests.
Readme template taken from [here](https://github.com/bremme/arduino-project/blob/master/README.md)
