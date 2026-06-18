# esp32.io-control

![Version](https://img.shields.io/badge/version-1.3.3-blue)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Donate](https://img.shields.io/badge/Donate-PayPal-00457C.svg?logo=paypal)](https://www.paypal.com/donate/?business=martin%40bchmnn.de&currency_code=EUR)

> **Universal IO-Controller für ESP32** — GPIOs per Browser steuern, I2C/SPI/PWM nutzen. Integriert in [iobroker.esp-hub](https://github.com/MPunktBPunkt/iobroker.esp-hub).

---

## Überblick

`esp32.io-control` macht aus einem ESP32 einen vollwertigen IO-Modul mit grafischem Board-Pinout, GPIO-Tabelle, I2C-Scanner, SPI-Transfer und PWM-Steuerung. Alle konfigurierten Pins werden im ESP-Hub-Heartbeat als IO-Werte gemeldet.

---

## Features

- **Interaktives Pinout** — Wemos D1 Mini ESP32 als SVG, farbig nach Modus
- **GPIO-Tabelle** — INPUT, OUTPUT, PWM, ADC, DAC, I2C, SPI
- **I2C-Tools** — Scan mit Geräteerkennung, Read/Write
- **SPI-Transfer** — VSPI konfigurierbar, Byte-Array TX/RX
- **PWM / LEDC** — Frequenz und Duty per Slider
- **ESP-Hub** — alle aktiven Pins als `ios`-Werte
- **Browser-OTA** — unter `/ota`

---

## Voraussetzungen

| Typ | Details |
|-----|---------|
| **Board** | Wemos D1 Mini ESP32 (Pinout optimiert) |
| **WiFiManager** | tablatronix / tzapu |
| **ArduinoJson** | bblanchon v6 oder v7 |
| **ioBroker** | [iobroker.esp-hub](https://github.com/MPunktBPunkt/iobroker.esp-hub) |

---

## Quickstart

1. Sketch öffnen, optional anpassen:

```cpp
#define DEVICE_NAME  "IO-Control"
#define HUB_HOST     "192.168.178.113"
#define HUB_PORT     8093
```

2. Flashen → Hotspot **`ESP-IO-Setup`** → WLAN konfigurieren
3. `http://<ESP-IP>/` öffnen

---

## Web-Oberfläche

| Tab | Inhalt |
|-----|--------|
| **Board** | SVG-Pinout, Klick springt zur GPIO-Tabelle |
| **GPIO** | Modus wählen, Werte lesen/schreiben |
| **Protokolle** | I2C Scan/Read/Write, SPI Transfer |
| **PWM** | Frequenz + Duty-Slider |
| **Status** | Chip, Heap, Uptime, OTA-Link |

---

## Pin-Modi

| Modus | Beschreibung |
|-------|--------------|
| INPUT / INPUT_PU | Digitaler Eingang |
| OUTPUT | Digitaler Ausgang |
| PWM | LEDC (1 Hz – 40 MHz) |
| ADC | 12-bit Analog (0–4095) |
| DAC | GPIO 25/26 (0–255) |
| I2C / SPI | Bus-Pins konfigurieren |

---

## ioBroker-Integration (ESP-Hub)

Aktive GPIOs erscheinen unter `esp-hub.0.devices.<MAC>.ios`.

Dashboard: `http://<ioBroker-IP>:8093`

---

## API (Auszug)

| Methode | Pfad | Beschreibung |
|---------|------|--------------|
| GET | `/api/pins` | Alle Pins mit Modus und Wert |
| POST | `/api/pin-set` | Modus setzen |
| POST | `/api/pin-write` | Ausgang schreiben |
| GET | `/api/i2c-scan` | I2C-Bus scannen |
| POST | `/api/spi-xfer` | SPI Transfer |

---

## Lizenz

GNU General Public License v3.0 © MPunktBPunkt — siehe [LICENSE](LICENSE)

[![Donate](https://img.shields.io/badge/Donate-PayPal-00457C.svg?logo=paypal)](https://www.paypal.com/donate/?business=martin%40bchmnn.de&currency_code=EUR)
