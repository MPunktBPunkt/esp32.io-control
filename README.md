# esp32.io-control

![Version](https://img.shields.io/badge/version-1.6.0-blue)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Donate](https://img.shields.io/badge/Donate-PayPal-00457C.svg?logo=paypal)](https://www.paypal.com/donate/?business=martin%40bchmnn.de&currency_code=EUR)

> **Universal IO-Controller für ESP32** — GPIOs per Browser steuern, I2C/SPI/PWM nutzen. Integriert in [iobroker.esp-hub](https://github.com/MPunktBPunkt/iobroker.esp-hub).

---

## Überblick

`esp32.io-control` macht aus einem ESP32 einen vollwertigen IO-Modul mit grafischem Board-Pinout, GPIO-Tabelle, I2C-Scanner, SPI-Transfer und PWM-Steuerung. Alle konfigurierten Sensor-/Output-Pins werden im ESP-Hub-Heartbeat als IO-Werte gemeldet.

---

## Features

- **Interaktives Pinout** — Wemos D1 Mini ESP32 / ESP32-S3 als SVG, farbig nach Modus
- **GPIO-Tabelle** — INPUT, OUTPUT, PWM, ADC, DAC, I2C, SPI, CLOCK, RGB
- **Raw / Volt** — ADC und DAC umschaltbar (Rohwert oder Volt)
- **Pin-Schutz** — BOOT / USB / UART nur nach Bestätigung schreibbar
- **I2C-Tools** — Scan mit Geräteerkennung, Read/Write (board-spezifische Defaults)
- **SPI-Transfer** — konfigurierbar, Byte-Array TX/RX
- **PWM / LEDC** — Frequenz und Duty per Slider
- **ESP-Hub** — aktive Pins als `ios` (`sensor` / `output`, ADC/DAC in Volt)
- **Browser-OTA** — unter `/ota`

---

## Voraussetzungen

| Typ | Details |
|-----|---------|
| **Board** | Wemos D1 Mini ESP32 oder ESP32-S3 |
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

## Vorkompilierte Firmware

Schema: `{name}.{version}.{family}.bin`

| Datei | Board |
|-------|-------|
| `io-control.1.6.0.esp32.bin` | ESP32 / D1 Mini |
| `io-control.1.6.0.esp32s3.bin` | ESP32-S3 |

> Hinweis: Vorkompilierte Bins entsprechen Sketch-Stand **1.6.0**.

---

## Web-Oberfläche

| Tab | Inhalt |
|-----|--------|
| **Board** | SVG-Pinout, Klick springt zur GPIO-Tabelle |
| **GPIO** | Modus wählen, Werte lesen/schreiben, Raw/Volt-Umschalter |
| **Protokolle** | I2C Scan/Read/Write, SPI Transfer |
| **PWM** | Frequenz + Duty-Slider |
| **Takt** | Rechteck-Taktgenerator (CLOCK, 50% Duty) |
| **DMM** | Spannung, Logik, Frequenz/Period/Duty, Counter |
| **Oszi** | ADC-Capture mit Canvas-Chart |
| **Status** | Chip, Heap, Uptime, OTA-Link |
| **RGB** | WS2812 (ESP32-S3, GPIO38) |

---

## Pin-Modi

| Modus | Beschreibung |
|-------|--------------|
| INPUT / INPUT_PU | Digitaler Eingang |
| OUTPUT | Digitaler Ausgang |
| PWM | LEDC (1 Hz – 40 MHz) |
| ADC | 12-bit Analog, Anzeige raw oder Volt (11 dB ≈ 0–3,3 V) |
| DAC | GPIO 25/26 nur klassisches ESP32 (0–255 / 0–3,3 V) |
| I2C / SPI | Bus-Pins setzen und Bus initialisieren |
| CLOCK | Rechteck-Takt |
| RGB | WS2812 (S3) |
| COUNT | Interrupt-Flankenzaehler (FALLING, Pull-up), Frequenz/s |

### Geschützte Pins

| Board | Pins |
|-------|------|
| ESP32 | 0 (BOOT), 1 (TX0), 3 (RX0) |
| ESP32-S3 | 0 (BOOT), 19/20 (USB), 43/44 (UART0) |

Schreiben (OUTPUT/PWM/DAC/…) nur nach UI-Bestätigung bzw. API `"force":true`.

### Bus-Defaults

| Board | I2C | SPI |
|-------|-----|-----|
| ESP32 | SDA 21 / SCL 22 | MOSI 23 / MISO 19 / SCK 18 / CS 5 |
| ESP32-S3 | SDA 8 / SCL 9 | MOSI 11 / MISO 13 / SCK 12 / CS 10 |

---

## ioBroker-Integration (ESP-Hub)

Aktive GPIOs erscheinen unter `esp-hub.0.devices.<MAC>.ios`.

- `INPUT` / `INPUT_PU` / `ADC` → `sensor`
- `OUTPUT` / `PWM` / `DAC` / `CLOCK` / `RGB` → `output`
- I2C_/SPI_-Modi werden nicht als Fake-IO gemeldet
- ADC/DAC-Werte: Volt (+ `raw`)

Dashboard: `http://<ioBroker-IP>:8093`

---

## API (Auszug)

| Methode | Pfad | Beschreibung |
|---------|------|--------------|
| GET | `/api/pins` | Alle Pins inkl. `mV`, `restricted`, `adc2` |
| POST | `/api/pin-set` | Modus setzen (`force` für geschützte Pins) |
| POST | `/api/pin-write` | Ausgang schreiben; DAC optional `"volts"` |
| GET | `/api/pin-read` | Pin lesen (`value` + `mV`) |
| GET | `/api/events` | SSE Live-Updates |
| POST | `/api/i2c-init` | I2C starten |
| GET | `/api/i2c-scan` | I2C-Bus scannen |
| POST | `/api/i2c-write` / `/api/i2c-read` | I2C Write/Read |
| POST | `/api/spi-init` / `/api/spi-xfer` | SPI Init/Transfer |
| POST | `/api/rgb` | WS2812 Farbe |
| GET | `/api/scope` | ADC-Capture `?gpio=&samples=&rate=` |
| GET | `/api/dmm` | Multimeter `?gpio=&method=volt\|logic\|freq\|period\|duty\|count` |
| POST | `/api/count-reset` | Counter zurücksetzen |
| POST | `/api/count-pulse` | Self-Test-Pulse auf COUNT-Pin |

---

## Lizenz

GNU General Public License v3.0 © MPunktBPunkt — siehe [LICENSE](LICENSE)

[![Donate](https://img.shields.io/badge/Donate-PayPal-00457C.svg?logo=paypal)](https://www.paypal.com/donate/?business=martin%40bchmnn.de&currency_code=EUR)
