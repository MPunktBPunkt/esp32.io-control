// ╔══════════════════════════════════════════════════════════════╗
// ║  esp32.io-control.ino — ESP32 Universal IO Controller       ║
// ║  Version: 1.2.0                                             ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  Bibliotheken (Arduino Library Manager):                    ║
// ║    - WiFiManager  von tablatronix / tzapu                   ║
// ║    - ArduinoJson  von bblanchon (v6 oder v7)                ║
// ║  Architektur:                                               ║
// ║    - Raw String Literals (R"html(...)html") fuer HTML/JS    ║
// ║    - SSE (Server-Sent Events) statt Polling                 ║
// ║    - driveOutputPins() gegen WiFi-Stack-Eingriffe           ║
// ║    - Software-Takt fuer CLOCK < 50 Hz                       ║
// ╚══════════════════════════════════════════════════════════════╝

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>

// dacWrite() existiert nur auf ESP32 (nicht S2, S3, C3 etc.)
// Fuer andere Chips: Stub damit der Sketch kompiliert
#if !defined(CONFIG_IDF_TARGET_ESP32)
  #define DAC_SUPPORTED 0
  inline void dacWrite(uint8_t, uint8_t) {}   // No-Op auf Chips ohne DAC
#else
  #define DAC_SUPPORTED 1
#endif

// ================================================================
//  KONFIGURATION
// ================================================================
#define DEVICE_NAME           "IO-Control"
#define FW_VERSION            "io-control v1.3.3"
#define HUB_HOST              "192.168.178.113"
#define HUB_PORT              8093
#define WIFI_AP_NAME          "ESP-IO-Setup"
#define WIFI_PORTAL_TIMEOUT_S 180
#define DEFAULT_INTERVAL_S    30
#define RESET_BUTTON_PIN      0
#define RESET_HOLD_SEC        3

// ================================================================
//  PIN-MODI
// ================================================================
#define PM_DISABLED  0
#define PM_INPUT     1
#define PM_INPUT_PU  2
#define PM_OUTPUT    3
#define PM_PWM       4
#define PM_ADC       5
#define PM_DAC       6
#define PM_I2C_SDA   7
#define PM_I2C_SCL   8
#define PM_SPI_MOSI  9
#define PM_SPI_MISO  10
#define PM_SPI_SCK   11
#define PM_SPI_CS    12
#define PM_CLOCK     13
#define PM_RGB       14   // WS2812 RGB-LED (neopixelWrite)
#define PM_MODE_MAX  15

const char* modeNames[PM_MODE_MAX] = {
    "DISABLED","INPUT","INPUT_PU","OUTPUT","PWM","ADC","DAC",
    "I2C_SDA","I2C_SCL","SPI_MOSI","SPI_MISO","SPI_SCK","SPI_CS","CLOCK","RGB"
};
const char* modeColors[PM_MODE_MAX] = {
    "#444","#2ecc71","#27ae60","#e74c3c","#e67e22","#9b59b6","#8e44ad",
    "#3498db","#2980b9","#1abc9c","#16a085","#17a589","#0e6655","#f39c12","#e91e8c"
};

struct PinInfo {
    uint8_t     gpio;
    const char* label;
    bool        inputOnly;
    bool        hasADC;
    bool        hasPWM;
    bool        hasDAC;
    uint8_t     mode;
    int         lastValue;
    uint32_t    pwmFreq;
    uint8_t     pwmDuty;
};

// ================================================================
//  PIN-TABELLEN (Board-spezifisch)
// ================================================================

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// ── ESP32-S3 Pins (keine Flash-Pins 26-32) ──
PinInfo pins[] = {
//  gpio  label           inOnly  adc    pwm    dac
    {  0, "IO0/BOOT",     false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    {  1, "IO1",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    {  2, "IO2",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    {  3, "IO3",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    {  4, "IO4",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    {  5, "IO5",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    {  6, "IO6",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    {  7, "IO7",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    {  8, "IO8",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    {  9, "IO9",          false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 10, "IO10",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 11, "IO11",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 12, "IO12",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 13, "IO13",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 14, "IO14",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 15, "IO15",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 16, "IO16",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 17, "IO17",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 18, "IO18",         false, true,   true,  false, PM_INPUT, 0, 1000, 128 },
    { 19, "IO19/USB-",    false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 20, "IO20/USB+",    false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 21, "IO21",         false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 35, "IO35",         false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 36, "IO36",         false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 37, "IO37",         false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 38, "IO38/RGB",     false, false,  true,  false, PM_RGB,  0, 1000, 128 },
    { 39, "IO39/MTCK",    false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 40, "IO40/MTDO",    false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 41, "IO41/MTDI",    false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 42, "IO42/MTMS",    false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 43, "IO43/TX0",     false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 44, "IO44/RX0",     false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 45, "IO45",         false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 46, "IO46",         false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 47, "IO47",         false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 48, "IO48",         false, false,  true,  false, PM_INPUT, 0, 1000, 128 },
};
#define BOARD_TYPE "esp32s3"
#define RGB_LED_GPIO 38

#else
// ── ESP32 (klassisch, Wemos D1 Mini) ──
PinInfo pins[] = {
    {  0, "IO0/BOOT",   false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    {  1, "IO1/TX0",    false, false, true,  false, PM_INPUT, 0, 1000, 128 },
    {  2, "IO2/LED",    false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    {  3, "IO3/RX0",    false, false, false, false, PM_INPUT, 0, 1000, 128 },
    {  4, "IO4",        false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    {  5, "IO5",        false, false, true,  false, PM_INPUT, 0, 1000, 128 },
    { 12, "IO12",       false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 13, "IO13",       false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 14, "IO14",       false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 15, "IO15",       false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 16, "IO16",       false, false, false, false, PM_INPUT, 0, 1000, 128 },
    { 17, "IO17",       false, false, false, false, PM_INPUT, 0, 1000, 128 },
    { 18, "IO18/SCK",   false, false, true,  false, PM_INPUT, 0, 1000, 128 },
    { 19, "IO19/MISO",  false, false, true,  false, PM_INPUT, 0, 1000, 128 },
    { 21, "IO21/SDA",   false, false, true,  false, PM_INPUT, 0, 1000, 128 },
    { 22, "IO22/SCL",   false, false, true,  false, PM_INPUT, 0, 1000, 128 },
    { 23, "IO23/MOSI",  false, false, true,  false, PM_INPUT, 0, 1000, 128 },
    { 25, "IO25/DAC1",  false, true,  true,  true,  PM_INPUT, 0, 1000, 128 },
    { 26, "IO26/DAC2",  false, true,  true,  true,  PM_INPUT, 0, 1000, 128 },
    { 27, "IO27",       false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 32, "IO32",       false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 33, "IO33",       false, true,  true,  false, PM_INPUT, 0, 1000, 128 },
    { 34, "IO34",       true,  true,  false, false, PM_ADC,   0, 0,    0   },
    { 35, "IO35",       true,  true,  false, false, PM_ADC,   0, 0,    0   },
    { 36, "SVP/IO36",   true,  true,  false, false, PM_ADC,   0, 0,    0   },
    { 39, "SVN/IO39",   true,  true,  false, false, PM_ADC,   0, 0,    0   },
};
#define BOARD_TYPE "esp32"
#define RGB_LED_GPIO -1
#endif

const int PIN_COUNT = sizeof(pins) / sizeof(pins[0]);

// ================================================================
//  GLOBALE STATE
// ================================================================
Preferences   prefs;
WiFiManager   wifiManager;
WebServer     webServer(80);

String        deviceName        = DEVICE_NAME;
String        hubHost           = HUB_HOST;
int           hubPort           = HUB_PORT;
unsigned long lastHeartbeat     = 0;
unsigned long heartbeatInterval = (unsigned long)DEFAULT_INTERVAL_S * 1000UL;
bool          otaPending        = false;
String        otaUrl            = "";
bool          i2cReady          = false;
int           i2cSda = 21,    i2cScl = 22;
bool          spiReady          = false;
int           spiMosi = 23, spiMiso = 19, spiSck = 18, spiCs = 5;
uint32_t      spiFreq           = 1000000;

// SSE: ein Client gleichzeitig (ESP32 Single-Thread)
static WiFiClient sseCli;
static bool       sseAlive      = false;
static unsigned long sseLastPush = 0;

// Software-Takt fuer CLOCK < 50 Hz
struct SoftClock {
    uint8_t       gpio       = 255;
    unsigned long halfUs     = 0;
    unsigned long nextToggle = 0;
    bool          state      = false;
};
static SoftClock    softClocks[8];
static const uint32_t SW_CLK_THRESHOLD = 50;

// ================================================================
//  FORWARD DECLARATIONS
// ================================================================
PinInfo*    findPin(int gpio);
void        applyPinMode(PinInfo* p);
void        readAllPins();
void        driveOutputPins();
void        tickSoftClocks();
void        startSoftClock(uint8_t gpio, uint32_t freqHz);
void        stopSoftClock(uint8_t gpio);
void        saveModesToPrefs();
void        loadModesFromPrefs();
String      buildPinsJson();
void        pushSSE(const char* event, const String& data);
void        pushPinsSSE();
void        sendHeartbeat();
void        performOta(const String& url);
String      getMac();
String      getLocalIp();
String      toMdnsName(const String& name);
int         parseHex(const String& s, uint8_t* buf, int max);
void        initI2C(int sda, int scl);
String      i2cWriteBytes(uint8_t addr, const uint8_t* data, int len);
String      i2cReadBytes(uint8_t addr, int reg, int len);
void        initSPIBus(int mosi, int miso, int sck, int cs, uint32_t freq);
String      spiXferBytes(int cs, const uint8_t* tx, uint8_t* rx, int len);
// API handlers
void        handleApiPins();
void        handleApiPinSet();
void        handleApiPinWrite();
void        handleApiPinRead();
void        handleApiI2cInit();
void        handleApiI2cScan();
void        handleApiI2cWrite();
void        handleApiI2cRead();
void        handleApiSpiInit();
void        handleApiSpiXfer();
void        handleApiRgb();
void        handleApiStatus();
void        handleSSE();
void        handleRoot();
void        handleOtaPage();
void        handleOtaUpload();
void        handleOtaUploadFinish();
void        handleNotFound();
void        setupWebServer();
void        checkResetButton();
void        setupWifi();
String      buildHeartbeat();

// ================================================================
//  HILFSFUNKTIONEN
// ================================================================
String getMac() {
    String m = WiFi.macAddress(); m.replace(":",""); m.toUpperCase(); return m;
}
String getLocalIp() { return WiFi.localIP().toString(); }

String toMdnsName(const String& name) {
    String out = "";
    for (int i = 0; i < (int)name.length() && (int)out.length() < 63; i++) {
        char c = name[i];
        if (c >= 'A' && c <= 'Z') c += 32;
        if (c == ' ' || c == '_') c = '-';
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-') out += c;
    }
    while (out.length() > 0 && out[0] == '-') out = out.substring(1);
    while (out.length() > 0 && out[out.length()-1] == '-') out = out.substring(0, out.length()-1);
    if (out.length() == 0) out = "esp32";
    return out;
}

PinInfo* findPin(int gpio) {
    for (int i = 0; i < PIN_COUNT; i++)
        if (pins[i].gpio == (uint8_t)gpio) return &pins[i];
    return nullptr;
}

int parseHex(const String& s, uint8_t* buf, int maxLen) {
    int len = 0, i = 0;
    while (i < (int)s.length() && len < maxLen) {
        while (i < (int)s.length() && s[i] == ' ') i++;
        if (i + 2 <= (int)s.length()) {
            char h[3] = { s[i], s[i+1], 0 };
            buf[len++] = (uint8_t)strtol(h, nullptr, 16);
            i += 2;
        } else break;
    }
    return len;
}

// ================================================================
//  PERSISTENZ (flat format: "gpio,mode,freq,duty,lastval|...")
// ================================================================
void saveModesToPrefs() {
    String s = "";
    bool first = true;
    for (int i = 0; i < PIN_COUNT; i++) {
        PinInfo& p = pins[i];
        bool isDefault = (p.inputOnly && p.mode == PM_ADC) ||
                         (!p.inputOnly && p.mode == PM_INPUT);
        if (!isDefault) {
            if (!first) s += "|";
            int lv = (p.mode == PM_OUTPUT) ? p.lastValue : 0;
            s += String(p.gpio) + "," + String(p.mode) + ","
               + String(p.pwmFreq) + "," + String(p.pwmDuty) + "," + String(lv);
            first = false;
        }
    }
    prefs.begin("ioctrl", false);
    prefs.putString("modes", s);
    prefs.end();
    Serial.println("[PREFS] " + s);
}

void loadModesFromPrefs() {
    prefs.begin("ioctrl", true);
    String s = prefs.getString("modes", "");
    prefs.end();
    if (s.length() < 3) return;
    // Altes [[...]]-Format loeschen
    if (s.startsWith("[")) {
        prefs.begin("ioctrl", false); prefs.remove("modes"); prefs.end();
        return;
    }
    int pos = 0;
    while (pos <= (int)s.length()) {
        int sep = s.indexOf('|', pos);
        if (sep < 0) sep = s.length();
        String e = s.substring(pos, sep);
        e.trim();
        if (e.length() > 0) {
            int c1 = e.indexOf(',');
            int c2 = e.indexOf(',', c1+1);
            int c3 = e.indexOf(',', c2+1);
            int c4 = e.indexOf(',', c3+1);
            if (c1>0 && c2>c1 && c3>c2) {
                int      gpio = e.substring(0,c1).toInt();
                int      mode = e.substring(c1+1,c2).toInt();
                uint32_t freq = (uint32_t)e.substring(c2+1,c3).toInt();
                uint8_t  duty;
                int      lv   = 0;
                if (c4 > c3) { duty=(uint8_t)e.substring(c3+1,c4).toInt(); lv=e.substring(c4+1).toInt(); }
                else          { duty=(uint8_t)e.substring(c3+1).toInt(); }
                PinInfo* p = findPin(gpio);
                if (p && mode >= 0 && mode < PM_MODE_MAX) {
                    p->mode=mode; p->pwmFreq=freq?freq:1000;
                    p->pwmDuty=duty; p->lastValue=lv;
                }
            }
        }
        if (sep >= (int)s.length()) break;
        pos = sep + 1;
    }
}

// ================================================================
//  SOFTWARE CLOCK
// ================================================================
void stopSoftClock(uint8_t gpio) {
    for (int i = 0; i < 8; i++)
        if (softClocks[i].gpio == gpio) { softClocks[i].gpio = 255; return; }
}

void startSoftClock(uint8_t gpio, uint32_t freqHz) {
    stopSoftClock(gpio);
    for (int i = 0; i < 8; i++) {
        if (softClocks[i].gpio == 255) {
            softClocks[i] = { gpio, 500000UL / (unsigned long)freqHz,
                              micros(), false };
            digitalWrite(gpio, LOW);
            return;
        }
    }
}

void tickSoftClocks() {
    unsigned long now = micros();
    for (int i = 0; i < 8; i++) {
        if (softClocks[i].gpio == 255) continue;
        if ((long)(now - softClocks[i].nextToggle) >= 0) {
            softClocks[i].state = !softClocks[i].state;
            digitalWrite(softClocks[i].gpio, softClocks[i].state ? HIGH : LOW);
            softClocks[i].nextToggle += softClocks[i].halfUs;
        }
    }
}

// ================================================================
//  PIN MANAGEMENT
// ================================================================
void applyPinMode(PinInfo* p) {
    if (!p) return;
    if (p->mode != PM_CLOCK) stopSoftClock(p->gpio);
    switch (p->mode) {
        case PM_INPUT:    pinMode(p->gpio, INPUT);         break;
        case PM_INPUT_PU: pinMode(p->gpio, INPUT_PULLUP);  break;
        case PM_OUTPUT:
            pinMode(p->gpio, OUTPUT);
            digitalWrite(p->gpio, p->lastValue ? HIGH : LOW);
            break;
        case PM_PWM:
            if (!p->inputOnly && p->hasPWM) {
                ledcAttach(p->gpio, p->pwmFreq, 8);
                ledcWrite(p->gpio, p->pwmDuty);
            }
            break;
        case PM_CLOCK:
            if (!p->inputOnly && p->hasPWM) {
                stopSoftClock(p->gpio);
                ledcDetach(p->gpio);
                if (p->pwmFreq >= SW_CLK_THRESHOLD) {
                    ledcAttach(p->gpio, p->pwmFreq, 8);
                    ledcWrite(p->gpio, 127);
                } else {
                    pinMode(p->gpio, OUTPUT);
                    startSoftClock(p->gpio, p->pwmFreq);
                }
            }
            break;
        case PM_ADC:   pinMode(p->gpio, INPUT);  break;
        case PM_DAC:   if (p->hasDAC) dacWrite(p->gpio, (uint8_t)p->lastValue); break;
        case PM_RGB:
            // WS2812 RGB-LED: Pin als OUTPUT, LED aus
            pinMode(p->gpio, OUTPUT);
            neopixelWrite(p->gpio, 0, 0, 0);
            break;
        case PM_DISABLED:
        default:       if (!p->inputOnly) pinMode(p->gpio, INPUT); break;
    }
}

void readAllPins() {
    for (int i = 0; i < PIN_COUNT; i++) {
        PinInfo& p = pins[i];
        if (p.mode == PM_INPUT || p.mode == PM_INPUT_PU)
            p.lastValue = digitalRead(p.gpio);
        else if (p.mode == PM_ADC)
            p.lastValue = analogRead(p.gpio);
        // OUTPUT/PWM/DAC/CLOCK/RGB: lastValue bleibt (kein digitalRead)
    }
}

void driveOutputPins() {
    static unsigned long lastDrive = 0;
    unsigned long now = millis();
    if (now - lastDrive < 100) return;
    lastDrive = now;
    for (int i = 0; i < PIN_COUNT; i++)
        if (pins[i].mode == PM_OUTPUT)
            digitalWrite(pins[i].gpio, pins[i].lastValue ? HIGH : LOW);
}

// ================================================================
//  I2C
// ================================================================
void initI2C(int sda, int scl) {
    Wire.end(); Wire.begin(sda, scl);
    i2cSda = sda; i2cScl = scl; i2cReady = true;
}

String i2cWriteBytes(uint8_t addr, const uint8_t* data, int len) {
    if (!i2cReady) return "ERR:not_init";
    Wire.beginTransmission(addr); Wire.write(data, len);
    uint8_t e = Wire.endTransmission();
    return e == 0 ? "OK" : "ERR:" + String(e);
}

String i2cReadBytes(uint8_t addr, int reg, int len) {
    if (!i2cReady) return "ERR:not_init";
    if (reg >= 0) {
        Wire.beginTransmission(addr); Wire.write((uint8_t)reg);
        if (Wire.endTransmission(false) != 0) return "ERR:nak";
    }
    Wire.requestFrom(addr, (uint8_t)len);
    String out = "["; bool f = true;
    while (Wire.available()) { if (!f) out += ","; out += String(Wire.read()); f = false; }
    return out + "]";
}

// ================================================================
//  SPI
// ================================================================
void initSPIBus(int mosi, int miso, int sck, int cs, uint32_t freq) {
    SPI.end(); SPI.begin(sck, miso, mosi, cs);
    spiMosi=mosi; spiMiso=miso; spiSck=sck; spiCs=cs; spiFreq=freq;
    if (cs >= 0) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
    spiReady = true;
}

String spiXferBytes(int cs, const uint8_t* tx, uint8_t* rx, int len) {
    if (!spiReady) return "ERR:not_init";
    int cp = (cs >= 0) ? cs : spiCs;
    SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE0));
    if (cp >= 0) digitalWrite(cp, LOW);
    for (int i = 0; i < len; i++) rx[i] = SPI.transfer(tx[i]);
    if (cp >= 0) digitalWrite(cp, HIGH);
    SPI.endTransaction();
    String o = "[";
    for (int i = 0; i < len; i++) { if (i) o += ","; o += String(rx[i]); }
    return o + "]";
}

// ================================================================
//  SSE (Server-Sent Events)
// ================================================================
String buildPinsJson() {
    readAllPins();
    String j = "[";
    for (int i = 0; i < PIN_COUNT; i++) {
        PinInfo& p = pins[i];
        if (i) j += ",";
        j += "{\"gpio\":"      + String(p.gpio);
        j += ",\"label\":\""   + String(p.label) + "\"";
        j += ",\"inputOnly\":" + String(p.inputOnly ? "true":"false");
        j += ",\"hasADC\":"    + String(p.hasADC   ? "true":"false");
        j += ",\"hasPWM\":"    + String(p.hasPWM   ? "true":"false");
        j += ",\"hasDAC\":"    + String(p.hasDAC   ? "true":"false");
        j += ",\"mode\":"      + String(p.mode);
        j += ",\"modeStr\":\"" + String(modeNames[p.mode]) + "\"";
        j += ",\"value\":"     + String(p.lastValue);
        j += ",\"pwmFreq\":"   + String(p.pwmFreq);
        j += ",\"pwmDuty\":"   + String(p.pwmDuty);
        j += "}";
    }
    return j + "]";
}

void pushSSE(const char* event, const String& data) {
    if (!sseAlive) return;
    if (!sseCli.connected()) { sseAlive = false; return; }
    sseCli.print("event: "); sseCli.println(event);
    sseCli.print("data: ");  sseCli.println(data);
    sseCli.println();
}

void pushPinsSSE() {
    if (!sseAlive) return;
    pushSSE("pins", buildPinsJson());
    sseLastPush = millis();
}

void handleSSE() {
    sseCli   = webServer.client();
    sseAlive = true;
    // HTTP response direkt schreiben (WebServer wuerde die Verbindung schliessen)
    sseCli.println("HTTP/1.1 200 OK");
    sseCli.println("Content-Type: text/event-stream");
    sseCli.println("Cache-Control: no-cache");
    sseCli.println("Connection: keep-alive");
    sseCli.println("Access-Control-Allow-Origin: *");
    sseCli.println();
    sseCli.print("retry: 3000\n\n");
    sseCli.flush();
    // Sofort initialen State senden
    pushPinsSSE();
    Serial.println("[SSE] Client verbunden: " + sseCli.remoteIP().toString());
}

// ================================================================
//  API HANDLER
// ================================================================
void handleApiPins() {
    webServer.send(200, "application/json", buildPinsJson());
}

void handleApiPinSet() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(256);
    #endif
    if (deserializeJson(doc, webServer.arg("plain"))) { webServer.send(400); return; }
    int gpio = doc["gpio"] | -1;
    int mode = doc["mode"] | -1;
    if (gpio < 0 || mode < 0 || mode >= PM_MODE_MAX) {
        webServer.send(400, "application/json", "{\"err\":\"param\"}"); return;
    }
    PinInfo* p = findPin(gpio);
    if (!p) { webServer.send(404); return; }
    if (p->inputOnly && (mode==PM_OUTPUT||mode==PM_PWM||mode==PM_DAC||mode==PM_CLOCK)) {
        webServer.send(400, "application/json", "{\"err\":\"input_only\"}"); return;
    }
    if (p->mode == PM_PWM && mode != PM_PWM) ledcDetach(p->gpio);
    p->mode = mode;
    if (doc.containsKey("freq")) p->pwmFreq = (uint32_t)(long)doc["freq"];
    if (doc.containsKey("duty")) p->pwmDuty = (uint8_t)(int)doc["duty"];
    applyPinMode(p);
    saveModesToPrefs();
    webServer.send(200, "application/json",
        "{\"ok\":true,\"gpio\":" + String(gpio) + ",\"mode\":" + String(mode) + "}");
    pushPinsSSE();   // sofort an SSE-Client pushen
}

void handleApiPinWrite() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(128);
    #endif
    if (deserializeJson(doc, webServer.arg("plain"))) { webServer.send(400); return; }
    int gpio = doc["gpio"] | -1, value = doc["value"] | 0;
    PinInfo* p = findPin(gpio); if (!p) { webServer.send(404); return; }
    p->lastValue = value;
    switch (p->mode) {
        case PM_OUTPUT: digitalWrite(p->gpio, value?HIGH:LOW); break;
        case PM_PWM:    p->pwmDuty=(uint8_t)constrain(value,0,255); ledcWrite(p->gpio,p->pwmDuty); break;
        case PM_DAC:    if (p->hasDAC) dacWrite(p->gpio,(uint8_t)constrain(value,0,255)); break;
        default: break;
    }
    if (p->mode == PM_OUTPUT) saveModesToPrefs();
    webServer.send(200, "application/json", "{\"ok\":true}");
    pushPinsSSE();
}

void handleApiPinRead() {
    int gpio = webServer.arg("gpio").toInt();
    PinInfo* p = findPin(gpio); if (!p) { webServer.send(404); return; }
    int val = (p->mode==PM_INPUT||p->mode==PM_INPUT_PU) ? digitalRead(p->gpio)
            : (p->mode==PM_ADC) ? analogRead(p->gpio) : p->lastValue;
    p->lastValue = val;
    webServer.send(200, "application/json",
        "{\"gpio\":" + String(gpio) + ",\"value\":" + String(val) + "}");
}

void handleApiI2cInit() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(128);
    #endif
    deserializeJson(doc, webServer.arg("plain"));
    int sda = doc["sda"]|21, scl = doc["scl"]|22;
    initI2C(sda, scl);
    webServer.send(200, "application/json",
        "{\"ok\":true,\"sda\":" + String(sda) + ",\"scl\":" + String(scl) + "}");
}

void handleApiI2cScan() {
    if (!i2cReady) initI2C(21, 22);
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(1024);
    #endif
    JsonArray arr = doc.to<JsonArray>();
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            #if ARDUINOJSON_VERSION_MAJOR >= 7
              JsonObject obj = arr.add<JsonObject>();
            #else
              JsonObject obj = arr.createNestedObject();
            #endif
            obj["addr"] = addr;
            char hb[7]; snprintf(hb, sizeof(hb), "0x%02X", addr);
            obj["hex"] = hb;
            const char* n = "";
            switch (addr) {
                case 0x27: n="LCD PCF8574";     break;
                case 0x3C: n="SSD1306 OLED";    break;
                case 0x3D: n="SSD1306 OLED";    break;
                case 0x48: n="ADS1115/PCF8591"; break;
                case 0x57: n="EEPROM AT24";     break;
                case 0x68: n="MPU6050/DS3231";  break;
                case 0x76: n="BME280/BMP280";   break;
                case 0x77: n="BME280/BMP280";   break;
            }
            obj["name"] = n;
        }
        delay(1);
    }
    String out; serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

void handleApiI2cWrite() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(512);
    #endif
    deserializeJson(doc, webServer.arg("plain"));
    uint8_t addr = (uint8_t)(int)doc["addr"];
    String  hs   = doc["hex"] | "";
    uint8_t buf[64]; int len = parseHex(hs, buf, 64);
    String r = i2cWriteBytes(addr, buf, len);
    webServer.send(200, "application/json",
        "{\"ok\":" + String(r=="OK"?"true":"false") + ",\"result\":\"" + r + "\"}");
}

void handleApiI2cRead() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(128);
    #endif
    deserializeJson(doc, webServer.arg("plain"));
    uint8_t addr = (uint8_t)(int)doc["addr"];
    int reg = doc["reg"]|-1, len = doc["len"]|1;
    webServer.send(200, "application/json",
        "{\"ok\":true,\"data\":" + i2cReadBytes(addr, reg, len) + "}");
}

void handleApiSpiInit() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(128);
    #endif
    deserializeJson(doc, webServer.arg("plain"));
    initSPIBus(doc["mosi"]|23, doc["miso"]|19, doc["sck"]|18,
               doc["cs"]|5,   doc["freq"]|1000000);
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiSpiXfer() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(512);
    #endif
    deserializeJson(doc, webServer.arg("plain"));
    int csPin = doc["cs"]|-1;
    String hs = doc["hex"]|"";
    uint8_t tx[64], rx[64]; int len = parseHex(hs, tx, 64);
    if (!spiReady) initSPIBus(23, 19, 18, 5, 1000000);
    webServer.send(200, "application/json",
        "{\"ok\":true,\"rx\":" + spiXferBytes(csPin, tx, rx, len) + "}");
}

// POST /api/rgb  {r, g, b}  — steuert WS2812-RGB-LED via neopixelWrite()
// Kein eigener Pin-Modus noetig: neopixelWrite() setzt den Pin intern
// Der Pin wird beim ersten Aufruf automatisch als OUTPUT konfiguriert
static bool rgbInitDone = false;

void handleApiRgb() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(128);
    #endif
    if (deserializeJson(doc, webServer.arg("plain"))) { webServer.send(400); return; }
    int r = constrain((int)(doc["r"]|0), 0, 255);
    int g = constrain((int)(doc["g"]|0), 0, 255);
    int b = constrain((int)(doc["b"]|0), 0, 255);
    int gpio = doc["gpio"] | RGB_LED_GPIO;
    if (gpio < 0) { webServer.send(400, "application/json", "{\"err\":\"no_rgb\"}"); return; }
    // Pin als OUTPUT vorbereiten (neopixelWrite braucht das)
    if (!rgbInitDone) {
        pinMode(gpio, OUTPUT);
        rgbInitDone = true;
    }
    // neopixelWrite(gpio, r, g, b) — Core 3.x, WS2812 GRB wird intern korrekt gesetzt
    neopixelWrite(gpio, r, g, b);
    Serial.printf("[RGB] GPIO%d R=%d G=%d B=%d\n", gpio, r, g, b);
    webServer.send(200, "application/json",
        "{\"ok\":true,\"r\":" + String(r) + ",\"g\":" + String(g) + ",\"b\":" + String(b) + "}");
}

void handleApiStatus() {
    String j = "{\"chip\":\"" + String(ESP.getChipModel()) + "\"";
    j += ",\"mac\":\""    + getMac() + "\"";
    j += ",\"ip\":\""     + getLocalIp() + "\"";
    j += ",\"mdns\":\""   + toMdnsName(deviceName) + ".local\"";
    j += ",\"rssi\":"     + String(WiFi.RSSI());
    j += ",\"uptime\":"   + String(millis()/1000UL);
    j += ",\"freeHeap\":" + String(ESP.getFreeHeap());
    j += ",\"freeSketch\":"+ String(ESP.getFreeSketchSpace());
    j += ",\"version\":\"" + String(FW_VERSION) + "\"";
    j += ",\"boardType\":\"" + String(BOARD_TYPE) + "\"";
    j += ",\"rgbLed\":"      + String(RGB_LED_GPIO);
    j += ",\"i2cReady\":"    + String(i2cReady?"true":"false");
    j += ",\"spiReady\":"    + String(spiReady?"true":"false") + "}";
    webServer.send(200, "application/json", j);
}

// ================================================================
//  WEB UI  — Raw String Literals
// ================================================================
void handleRoot() {
    // ── CSS ──────────────────────────────────────────────────────
    static const char CSS[] PROGMEM = R"css(
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#e6edf3;font-family:sans-serif;font-size:13px}
header{background:#161b22;border-bottom:1px solid #30363d;padding:12px 20px;display:flex;align-items:center;gap:10px}
h1{font-size:17px;color:#58a6ff}
.badge{background:rgba(88,166,255,.15);color:#58a6ff;border:1px solid rgba(88,166,255,.3);padding:2px 8px;border-radius:10px;font-size:11px}
.tabs{display:flex;background:#161b22;border-bottom:1px solid #30363d;padding:0 16px;overflow-x:auto}
.tab{padding:9px 18px;cursor:pointer;border-bottom:2px solid transparent;color:#8b949e;white-space:nowrap;font-weight:500;font-size:13px}
.tab.active,.tab:hover{color:#58a6ff;border-bottom-color:#58a6ff}
.pane{display:none;padding:18px;max-width:1000px}
.pane.active{display:block}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin-bottom:12px}
.card h3{font-size:11px;color:#8b949e;text-transform:uppercase;letter-spacing:.5px;margin-bottom:10px}
table{width:100%;border-collapse:collapse}
td,th{padding:6px 8px;border-bottom:1px solid #21262d;text-align:left}
th{color:#8b949e;font-weight:500;font-size:11px}
select,input[type=text],input[type=number]{background:#0d1117;color:#e6edf3;border:1px solid #30363d;border-radius:4px;padding:4px 6px;font-size:12px}
.btn{background:#21262d;color:#e6edf3;border:1px solid #30363d;padding:4px 10px;border-radius:4px;cursor:pointer;font-size:12px}
.btn:hover{background:#30363d}
.btn-g{background:#238636;border-color:#2ea043;color:#fff}.btn-g:hover{background:#2ea043}
.btn-b{background:#1f6feb;border-color:#388bfd;color:#fff}.btn-b:hover{background:#388bfd}
.btn-r{background:#b91c1c;border-color:#ef4444;color:#fff}
.btn-sm{padding:2px 7px;font-size:11px}
.mono{font-family:monospace;color:#58a6ff}
.vhi{color:#3fb950;font-weight:bold}.vlo{color:#8b949e}.vadc{color:#a371f7}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:600px){.grid2{grid-template-columns:1fr}}
textarea{background:#0d1117;color:#e6edf3;border:1px solid #30363d;border-radius:4px;padding:6px;font-family:monospace;font-size:12px;width:100%;resize:vertical}
.rbox{background:#0d1117;border:1px solid #30363d;border-radius:4px;padding:8px;font-family:monospace;font-size:12px;min-height:40px;color:#3fb950;margin-top:8px;white-space:pre-wrap}
.row{display:flex;gap:8px;align-items:center;margin-bottom:8px;flex-wrap:wrap}
label{font-size:12px;color:#8b949e;white-space:nowrap}
#bsvg{width:100%;max-width:1000px;display:block;margin:0 auto}
.pc{cursor:pointer;transition:filter .15s}.pc:hover{filter:brightness(1.5)}
)css";

    // ── JAVASCRIPT ───────────────────────────────────────────────
    static const char JS[] PROGMEM = R"js(
var MC=['#444','#2ecc71','#27ae60','#e74c3c','#e67e22','#9b59b6','#8e44ad',
        '#3498db','#2980b9','#1abc9c','#16a085','#17a589','#0e6655','#f39c12','#e91e8c'];
var MN=['DISABLED','INPUT','INPUT_PU','OUTPUT','PWM','ADC','DAC',
        'I2C_SDA','I2C_SCL','SPI_MOSI','SPI_MISO','SPI_SCK','SPI_CS','CLOCK','RGB'];

// Board layout: ViewBox 1000x300
// 4 Spalten × 10 Reihen nach echtem Wemos D1 Mini ESP32 Pinout
// OL=outer-left x=55, IL=inner-left x=130
// ── ESP32 (Wemos D1 Mini): 4 Spalten x 10 Reihen ─────────────
var LY_ESP32=[
  // IL inner-left
  {g:-1, lb:'RST',  s:'IL',row:0, sp:'#7f8c8d'},
  {g:36, lb:'SVP',  s:'IL',row:1},
  {g:26, lb:'IO26', s:'IL',row:2},
  {g:18, lb:'IO18', s:'IL',row:3},
  {g:19, lb:'IO19', s:'IL',row:4},
  {g:23, lb:'IO23', s:'IL',row:5},
  {g:5,  lb:'IO5',  s:'IL',row:6},
  {g:-2, lb:'3V3',  s:'IL',row:7, sp:'#c0392b'},
  {g:13, lb:'IO13', s:'IL',row:8},
  {g:-3, lb:'SD3',  s:'IL',row:9, sp:'#5d2e2e'},
  // OL outer-left
  {g:-4, lb:'GND',  s:'OL',row:0, sp:'#2c3e50'},
  {g:-5, lb:'NC',   s:'OL',row:1, sp:'#444'},
  {g:39, lb:'SVN',  s:'OL',row:2},
  {g:35, lb:'IO35', s:'OL',row:3},
  {g:33, lb:'IO33', s:'OL',row:4},
  {g:34, lb:'IO34', s:'OL',row:5},
  {g:14, lb:'IO14', s:'OL',row:6},
  {g:-6, lb:'NC',   s:'OL',row:7, sp:'#444'},
  {g:-7, lb:'SD2',  s:'OL',row:8, sp:'#5d2e2e'},
  {g:-8, lb:'CMD',  s:'OL',row:9, sp:'#5d2e2e'},
  // IR inner-right
  {g:1,  lb:'TX0',  s:'IR',row:0},
  {g:3,  lb:'RX0',  s:'IR',row:1},
  {g:22, lb:'IO22', s:'IR',row:2},
  {g:21, lb:'IO21', s:'IR',row:3},
  {g:17, lb:'IO17', s:'IR',row:4},
  {g:16, lb:'IO16', s:'IR',row:5},
  {g:-9, lb:'GND',  s:'IR',row:6, sp:'#2c3e50'},
  {g:-10,lb:'+5V',  s:'IR',row:7, sp:'#c0392b'},
  {g:15, lb:'IO15', s:'IR',row:8},
  {g:-11,lb:'SD0',  s:'IR',row:9, sp:'#5d2e2e'},
  // OR outer-right
  {g:-12,lb:'GND',  s:'OR',row:0, sp:'#2c3e50'},
  {g:27, lb:'IO27', s:'OR',row:1},
  {g:25, lb:'IO25', s:'OR',row:2},
  {g:32, lb:'IO32', s:'OR',row:3},
  {g:12, lb:'IO12', s:'OR',row:4},
  {g:4,  lb:'IO4',  s:'OR',row:5},
  {g:0,  lb:'IO0',  s:'OR',row:6},
  {g:2,  lb:'IO2',  s:'OR',row:7},
  {g:-13,lb:'SD1',  s:'OR',row:8, sp:'#5d2e2e'},
  {g:-14,lb:'CLK',  s:'OR',row:9, sp:'#5d2e2e'}
];

// ── ESP32-S3 Dev Module: 2 Spalten x 22 Reihen ────────────────
// Linke Seite (s:'L'), rechte Seite (s:'R')
var LY_S3=[
  // Linke Seite (von oben)
  {g:-1, lb:'3V3',  s:'L',row:0,  sp:'#c0392b'},
  {g:-2, lb:'3V3',  s:'L',row:1,  sp:'#c0392b'},
  {g:-3, lb:'RST',  s:'L',row:2,  sp:'#7f8c8d'},
  {g:4,  lb:'IO4',  s:'L',row:3},
  {g:5,  lb:'IO5',  s:'L',row:4},
  {g:6,  lb:'IO6',  s:'L',row:5},
  {g:7,  lb:'IO7',  s:'L',row:6},
  {g:15, lb:'IO15', s:'L',row:7},
  {g:16, lb:'IO16', s:'L',row:8},
  {g:17, lb:'IO17', s:'L',row:9},
  {g:18, lb:'IO18', s:'L',row:10},
  {g:8,  lb:'IO8',  s:'L',row:11},
  {g:3,  lb:'IO3',  s:'L',row:12},
  {g:46, lb:'IO46', s:'L',row:13},
  {g:9,  lb:'IO9',  s:'L',row:14},
  {g:10, lb:'IO10', s:'L',row:15},
  {g:11, lb:'IO11', s:'L',row:16},
  {g:12, lb:'IO12', s:'L',row:17},
  {g:13, lb:'IO13', s:'L',row:18},
  {g:14, lb:'IO14', s:'L',row:19},
  {g:-4, lb:'5V',   s:'L',row:20, sp:'#c0392b'},
  {g:-5, lb:'GND',  s:'L',row:21, sp:'#2c3e50'},
  // Rechte Seite (von oben)
  {g:-6, lb:'GND',  s:'R',row:0,  sp:'#2c3e50'},
  {g:43, lb:'TX0',  s:'R',row:1},
  {g:44, lb:'RX0',  s:'R',row:2},
  {g:1,  lb:'IO1',  s:'R',row:3},
  {g:2,  lb:'IO2',  s:'R',row:4},
  {g:42, lb:'IO42', s:'R',row:5},
  {g:41, lb:'IO41', s:'R',row:6},
  {g:40, lb:'IO40', s:'R',row:7},
  {g:39, lb:'IO39', s:'R',row:8},
  {g:38, lb:'IO38\u25cf',s:'R',row:9, rgb:true},  // RGB-LED
  {g:37, lb:'IO37', s:'R',row:10},
  {g:36, lb:'IO36', s:'R',row:11},
  {g:35, lb:'IO35', s:'R',row:12},
  {g:0,  lb:'IO0',  s:'R',row:13},
  {g:45, lb:'IO45', s:'R',row:14},
  {g:48, lb:'IO48', s:'R',row:15},
  {g:47, lb:'IO47', s:'R',row:16},
  {g:21, lb:'IO21', s:'R',row:17},
  {g:20, lb:'IO20', s:'R',row:18},
  {g:19, lb:'IO19', s:'R',row:19},
  {g:-7, lb:'GND',  s:'R',row:20, sp:'#2c3e50'},
  {g:-8, lb:'GND',  s:'R',row:21, sp:'#2c3e50'}
];

// SVG PCB-Inhalte fuer Board-Wechsel
var SVG_32_INNER = '<rect x="148" y="5" width="704" height="285" rx="6" fill="#1a4d2e" stroke="#0d3b22" stroke-width="2"/><rect x="165" y="55" width="230" height="170" rx="4" fill="#111" stroke="#444" stroke-width="1.5"/><text x="280" y="132" text-anchor="middle" fill="#555" font-family="monospace" font-size="10">ESPRESSIF</text><text x="280" y="145" text-anchor="middle" fill="#444" font-family="monospace" font-size="8">ESP32-WROOM-32</text><text x="280" y="157" text-anchor="middle" fill="#2a6" font-family="monospace" font-size="7">WiFi+BT</text><rect x="340" y="57" width="52" height="24" rx="2" fill="none" stroke="#2a5" stroke-width="1" stroke-dasharray="3,2"/><text x="366" y="73" text-anchor="middle" fill="#2a5" font-family="sans-serif" font-size="6">ANT</text><rect x="580" y="190" width="60" height="45" rx="3" fill="#162" stroke="#0a3" stroke-width="0.8" opacity="0.7"/><text x="610" y="217" text-anchor="middle" fill="#0a3" font-family="monospace" font-size="6">CP2102</text><rect x="770" y="240" width="70" height="32" rx="3" fill="#555" stroke="#888"/><rect x="775" y="244" width="60" height="24" rx="2" fill="#333"/><text x="805" y="260" text-anchor="middle" fill="#bbb" font-family="sans-serif" font-size="7">USB</text><rect x="760" y="14" width="24" height="14" rx="3" fill="#333" stroke="#666"/><text x="772" y="24" text-anchor="middle" fill="#888" font-family="sans-serif" font-size="6">RST</text><circle cx="720" cy="21" r="5" fill="#1a4" stroke="#0d3" stroke-width="1"/><text x="720" y="34" text-anchor="middle" fill="#2a6" font-family="sans-serif" font-size="6">LED</text><line x1="148" y1="18" x2="148" y2="270" stroke="#0d3b22" stroke-width="14" stroke-dasharray="5,17"/><line x1="165" y1="18" x2="165" y2="270" stroke="#0d3b22" stroke-width="14" stroke-dasharray="5,17"/><line x1="835" y1="18" x2="835" y2="270" stroke="#0d3b22" stroke-width="14" stroke-dasharray="5,17"/><line x1="852" y1="18" x2="852" y2="270" stroke="#0d3b22" stroke-width="14" stroke-dasharray="5,17"/>';

var SVG_S3_INNER = '<rect x="290" y="5" width="420" height="505" rx="6" fill="#1a1a3e" stroke="#111133" stroke-width="2"/><rect x="310" y="25" width="200" height="200" rx="4" fill="#111" stroke="#444" stroke-width="1.5"/><text x="410" y="115" text-anchor="middle" fill="#555" font-family="monospace" font-size="10">ESPRESSIF</text><text x="410" y="128" text-anchor="middle" fill="#444" font-family="monospace" font-size="8">ESP32-S3-WROOM-1</text><text x="410" y="140" text-anchor="middle" fill="#48f" font-family="monospace" font-size="7">WiFi+BT+USB</text><rect x="430" y="27" width="78" height="28" rx="2" fill="none" stroke="#2a5" stroke-width="1" stroke-dasharray="3,2"/><text x="469" y="45" text-anchor="middle" fill="#2a5" font-family="sans-serif" font-size="6">ANTENNA</text><circle cx="475" cy="310" r="8" fill="#200" stroke="#f00" stroke-width="1.5"/><circle cx="475" cy="310" r="4" fill="#300" stroke="#0f0" stroke-width="1"/><circle cx="475" cy="310" r="1.5" fill="#030"/><text x="475" y="326" text-anchor="middle" fill="#f55" font-family="sans-serif" font-size="6">RGB@38</text><rect x="380" y="450" width="55" height="25" rx="3" fill="#555" stroke="#888"/><text x="407" y="466" text-anchor="middle" fill="#bbb" font-family="sans-serif" font-size="6">UART</text><rect x="460" y="450" width="55" height="25" rx="3" fill="#555" stroke="#888"/><text x="487" y="466" text-anchor="middle" fill="#bbb" font-family="sans-serif" font-size="6">USB</text><rect x="415" y="360" width="36" height="22" rx="2" fill="#333" stroke="#666"/><text x="433" y="375" text-anchor="middle" fill="#888" font-family="sans-serif" font-size="5">BOOT</text><rect x="465" y="360" width="36" height="22" rx="2" fill="#333" stroke="#666"/><text x="483" y="375" text-anchor="middle" fill="#888" font-family="sans-serif" font-size="5">RST</text><line x1="290" y1="12" x2="290" y2="493" stroke="#111133" stroke-width="12" stroke-dasharray="5,16"/><line x1="710" y1="12" x2="710" y2="493" stroke="#111133" stroke-width="12" stroke-dasharray="5,16"/>';
var BOARD = 'esp32';
var RGB_GPIO = -1;

// Layout-Koordinaten ESP32 (4-spaltig, schmales PCB 200..600, viewBox 800x300)
var CX_32  = {OL:30,  IL:90,  IR:710, OR:770};
var LX_32  = {OL:{x:20,a:'end'}, IL:{x:80,a:'end'}, IR:{x:720,a:'start'}, OR:{x:780,a:'start'}};
var PE_32  = {OL:200, IL:200, IR:600, OR:600};
// Layout-Koordinaten S3 (2-spaltig, 22 Zeilen)
var CX_S3L = 280, CX_S3R = 720;
var VB_S3  = '0 0 1000 520';
var VB_32  = '0 0 800 300';

function py32(r){ return 18 + r*28; }
function pyS3(r){ return 12 + r*23; }

var PD = [];

// Tab wechseln
function showTab(id, el) {
  document.querySelectorAll('.pane').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.getElementById('pane-' + id).classList.add('active');
  el.classList.add('active');
  if (id === 'board')  renderBoard();
  if (id === 'gpio')   { fetch('/api/pins').then(r=>r.json()).then(d=>{PD=d;renderGpio();}); }
  if (id === 'pwm')    renderPwm();
  if (id === 'rgb')    rgbPreview();
  if (id === 'status') fetchStatus();
}

// ── Board SVG ─────────────────────────────────────────────────
function renderBoard() {
  var svg = document.getElementById('bsvg');
  svg.querySelectorAll('.dyn').forEach(e => e.remove());
  var pm = {}; PD.forEach(p => pm[p.gpio] = p);
  var ns = 'http://www.w3.org/2000/svg';

  if (BOARD === 'esp32s3') {
    renderBoardS3(svg, pm, ns);
  } else {
    renderBoard32(svg, pm, ns);
  }

  // Legende
  var leg = document.getElementById('bleg');
  leg.innerHTML = '';
  MN.forEach(function(n, i) {
    if (i === 0 || PD.some(p => p.mode === i)) {
      var sp = document.createElement('span');
      sp.style.cssText = 'font-size:11px;color:#8b949e;display:flex;align-items:center;gap:4px';
      sp.innerHTML = '<span style="display:inline-block;width:10px;height:10px;border-radius:2px;background:' + MC[i] + ';"></span>' + n;
      leg.appendChild(sp);
    }
  });
  if (BOARD === 'esp32') {
    var fw = document.createElement('span');
    fw.style.cssText = 'font-size:11px;color:#a55;display:flex;align-items:center;gap:4px';
    fw.innerHTML = '<span style="display:inline-block;width:10px;height:10px;border-radius:2px;background:#5d2e2e;"></span>FLASH';
    leg.appendChild(fw);
  }
  if (RGB_GPIO >= 0) {
    var rg = document.createElement('span');
    rg.style.cssText = 'font-size:11px;color:#e74c3c;display:flex;align-items:center;gap:4px';
    rg.innerHTML = '<span style="display:inline-block;width:10px;height:10px;border-radius:50%;background:linear-gradient(135deg,#f00,#0f0,#00f);"></span>RGB-LED (IO38)';
    leg.appendChild(rg);
  }
}

// ── ESP32 Board rendern (4 Spalten) ──────────────────────────
function renderBoard32(svg, pm, ns) {
  LY_ESP32.forEach(function(lp) {
    var pd  = pm[lp.g];
    var col = lp.sp || (pd ? MC[pd.mode] : '#444');
    var x   = CX_32[lp.s];
    var y   = py32(lp.row);
    var grp = makePad(ns, lp, pd, col, x, y, CX_32, PE_32, LX_32, py32);
    svg.appendChild(grp);
  });
}

// ── ESP32-S3 Board rendern (2 Spalten) ───────────────────────
function renderBoardS3(svg, pm, ns) {
  LY_S3.forEach(function(lp) {
    var pd  = pm[lp.g];
    // RGB-LED bekommt Regenbogenfarbe
    var col = lp.rgb ? '#e74c3c' : (lp.sp || (pd ? MC[pd.mode] : '#444'));
    var x   = (lp.s === 'L') ? CX_S3L : CX_S3R;
    var y   = pyS3(lp.row);
    var isL = (lp.s === 'L');
    var lxObj = {x: isL ? x-14 : x+14, a: isL ? 'end' : 'start'};
    var cxMap = {L:CX_S3L, R:CX_S3R};
    var peMap = {L:290, R:710};
    var lxMap = {L:{x:CX_S3L-14,a:'end'}, R:{x:CX_S3R+14,a:'start'}};
    var grp = makePad(ns, lp, pd, col, x, y, cxMap, peMap, lxMap, pyS3);
    svg.appendChild(grp);
  });
}

// ── Gemeinsame Pad-Erstellung ─────────────────────────────────
function makePad(ns, lp, pd, col, x, y, cxMap, peMap, lxMap, pyFn) {
    var grp = document.createElementNS(ns, 'g');
    grp.classList.add('dyn');
    if (lp.g > 0 && !lp.sp) {
      grp.style.cursor = 'pointer';
      grp.onclick = (function(gp){ return function() {
        showTab('gpio', document.querySelectorAll('.tab')[1]);
        setTimeout(function(){ var r=document.getElementById('r'+gp); if(r) r.scrollIntoView({behavior:'smooth',block:'center'}); }, 200);
      };})(lp.g);
    }
    // Verbindungslinie
    var ln = document.createElementNS(ns, 'line');
    ln.setAttribute('x1', x); ln.setAttribute('y1', y);
    ln.setAttribute('x2', peMap[lp.s]); ln.setAttribute('y2', y);
    ln.setAttribute('stroke', col); ln.setAttribute('stroke-width', '1.2'); ln.setAttribute('opacity', '0.35');
    grp.appendChild(ln);
    // Pad
    var pad = document.createElementNS(ns, 'rect');
    pad.setAttribute('x', x-5); pad.setAttribute('y', y-5);
    pad.setAttribute('width', '10'); pad.setAttribute('height', '10');
    pad.setAttribute('rx', '2'); pad.setAttribute('fill', col);
    pad.setAttribute('stroke', '#000'); pad.setAttribute('stroke-width', '0.8');
    if (!lp.sp) pad.classList.add('pc');
    grp.appendChild(pad);
    // Bohrung
    var hole = document.createElementNS(ns, 'circle');
    hole.setAttribute('cx', x); hole.setAttribute('cy', y);
    hole.setAttribute('r', '2'); hole.setAttribute('fill', '#0d1117');
    grp.appendChild(hole);
    // Zustandsring
    if (pd && (pd.mode===1||pd.mode===2||pd.mode===3)) {
      var ring = document.createElementNS(ns, 'circle');
      ring.setAttribute('cx', x); ring.setAttribute('cy', y);
      ring.setAttribute('r', '4.5'); ring.setAttribute('fill', 'none');
      ring.setAttribute('stroke', pd.value ? '#3fb950' : '#555');
      ring.setAttribute('stroke-width', '1.2');
      grp.appendChild(ring);
    }
    // Label
    var tx = document.createElementNS(ns, 'text');
    tx.setAttribute('x', lxMap[lp.s].x); tx.setAttribute('y', y+3);
    tx.setAttribute('text-anchor', lxMap[lp.s].a);
    var lc = lp.rgb ? '#f55' : (lp.sp ? (lp.sp==='#5d2e2e'?'#a55':'#888') : (pd && pd.mode > 0 ? MC[pd.mode] : '#888'));
    tx.setAttribute('fill', lc); tx.setAttribute('font-size', '8.5'); tx.setAttribute('font-family', 'monospace');
    var vs = '';
    if (pd && (pd.mode===1||pd.mode===2||pd.mode===3)) vs = pd.value ? '=H' : '=L';
    else if (pd && pd.mode===5) vs = '=' + pd.value;
    else if (pd && (pd.mode===4||pd.mode===13)) vs = '~';
    tx.textContent = lp.lb + vs;
    grp.appendChild(tx);
    return grp;
}

// ── Helper: Wert + Aktion fuer eine Zeile ─────────────────────
function rowVA(p) {
  var vs = '—', vc = '', act = '';
  if (p.mode===1||p.mode===2||p.mode===3) { vs=p.value?'HIGH':'LOW'; vc=p.value?'vhi':'vlo'; }
  else if (p.mode===4)  { vs=p.pwmDuty+'/255 @ '+p.pwmFreq+'Hz'; vc='vadc'; }
  else if (p.mode===5)  { vs=p.value+' (ADC)'; vc='vadc'; }
  else if (p.mode===6)  { vs=p.value+'/255 DAC'; vc='vadc'; }
  else if (p.mode===13) { vs='CLK '+p.pwmFreq+'Hz'; vc='vadc'; }
  else if (p.mode===14) { vs='&#127752; RGB-LED'; vc='vadc'; }
  if (p.mode===3)  act='<button class="btn btn-sm btn-g" onclick="pinToggle('+p.gpio+')">'+(p.value?'&#9679; HI':'&#9675; LO')+'</button>';
  else if (p.mode===4)  act='<input type=range min=0 max=255 value='+p.pwmDuty+' style="width:80px;accent-color:#e67e22" oninput="pinPwmDuty('+p.gpio+',this.value)">';
  else if (p.mode===6)  act='<input type=range min=0 max=255 value='+p.value+' style="width:80px;accent-color:#8e44ad" oninput="pinDac('+p.gpio+',this.value)">';
  else if (p.mode===5)  act='<button class="btn btn-sm" onclick="pinRead('+p.gpio+')">Lesen</button>';
  else if (p.mode===13) act='<input type=number value='+p.pwmFreq+' min=1 max=40000000 style="width:90px;font-size:11px" onchange="pinClockFreq('+p.gpio+',this.value)"> Hz';
  else if (p.mode===14) act='<button class="btn btn-sm" style="background:#e91e8c;color:#fff" onclick="showTab(\'rgb\',document.getElementById(\'rgb-tab\'))">&#127752; RGB-Tab</button>';
  return {vs:vs, vc:vc, act:act};
}

// ── GPIO Tabelle: Rebuild (nur bei Tab-Wechsel) ────────────────
function renderGpio() {
  var tb = document.getElementById('gtb');
  if (!PD.length) { tb.innerHTML='<tr><td colspan=5 style=color:#555;text-align:center>Lade...</td></tr>'; return; }
  var h = '';
  PD.forEach(function(p) {
    var va = rowVA(p);
    var opts = '';
    for (var m=0; m<MN.length; m++) {
      if (p.inputOnly && (m===3||m===4||m===6||m===13)) continue;
      if (!p.hasADC && m===5) continue;
      if (!p.hasDAC && m===6) continue;
      if (!p.hasPWM && (m===4||m===13)) continue;
      opts += '<option value='+m+(p.mode===m?' selected':'')+'>'+MN[m]+'</option>';
    }
    h += '<tr id="r'+p.gpio+'">';
    h += '<td class="mono">'+p.gpio+'</td>';
    h += '<td style="font-size:11px;color:#8b949e">'+p.label+'</td>';
    h += '<td><select data-gpio='+p.gpio+' style="font-size:11px" onchange="pinModeChange(this)">'+opts+'</select></td>';
    h += '<td class="'+va.vc+'" id="val-'+p.gpio+'">'+va.vs+'</td>';
    h += '<td id="act-'+p.gpio+'">'+va.act+'</td></tr>';
  });
  tb.innerHTML = h;
}

// ── Nur Wert+Aktion einer Zeile aktualisieren ─────────────────
function updateOneRow(gpio) {
  var p = PD.find(x => x.gpio === gpio);
  if (!p) return;
  var va = rowVA(p);
  var vc = document.getElementById('val-'+gpio);
  var ac = document.getElementById('act-'+gpio);
  if (vc) { vc.className=va.vc; vc.innerHTML=va.vs; }
  if (ac) { ac.innerHTML=va.act; }
}

// ── Alle Werte aktualisieren (vom SSE-Event) ──────────────────
function updateGpioValues() {
  PD.forEach(function(p) {
    var sel = document.querySelector('select[data-gpio="'+p.gpio+'"]');
    if (sel && document.activeElement===sel) return;
    var vc = document.getElementById('val-'+p.gpio);
    if (!vc) return;
    var va = rowVA(p);
    vc.className = va.vc; vc.innerHTML = va.vs;
    if (p.mode===4) { var sl=document.querySelector('#act-'+p.gpio+' input[type=range]'); if(sl) sl.value=p.pwmDuty; }
    if (p.mode===3) { var btn=document.querySelector('#act-'+p.gpio+' button'); if(btn) btn.innerHTML=p.value?'&#9679; HI':'&#9675; LO'; }
  });
}

// ── SSE Verbindung ────────────────────────────────────────────
var es;
function connectSSE() {
  es = new EventSource('/api/events');
  es.addEventListener('pins', function(e) {
    PD = JSON.parse(e.data);
    document.getElementById('hbb').textContent = '✅';
    var pn = document.querySelector('.pane.active');
    if (pn && pn.id === 'pane-board') renderBoard();
    if (pn && pn.id === 'pane-gpio')  updateGpioValues();
  });
  es.onerror = function() {
    document.getElementById('hbb').textContent = '❌';
    es.close();
    setTimeout(connectSSE, 3000);
  };
}
connectSSE();
fetchStatus();  // Board-Typ sofort erkennen

// ── Modus-Aenderung ───────────────────────────────────────────
function pinModeChange(sel) {
  var gpio = parseInt(sel.getAttribute('data-gpio'));
  var mode = parseInt(sel.value);
  var p = PD.find(x => x.gpio === gpio);
  if (p) p.mode = mode;
  updateOneRow(gpio);
  fetch('/api/pin-set', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({gpio:gpio, mode:mode})})
  .then(r => r.json())
  .then(d => { if (!d.ok) fetch('/api/pins').then(r=>r.json()).then(d2=>{PD=d2;renderGpio();}); });
}

function pinToggle(gpio) {
  var p = PD.find(x => x.gpio === gpio);
  if (!p) return;
  p.value = p.value ? 0 : 1;
  updateOneRow(gpio);
  fetch('/api/pin-write', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({gpio:gpio, value:p.value})});
}
function pinPwmDuty(gpio, duty) {
  var p = PD.find(x => x.gpio===gpio); if(p) p.pwmDuty=parseInt(duty);
  fetch('/api/pin-write', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({gpio:gpio, value:parseInt(duty)})});
}
function pinClockFreq(gpio, freq) {
  var p = PD.find(x => x.gpio===gpio); if(p) p.pwmFreq=parseInt(freq)||1000;
  updateOneRow(gpio);
  fetch('/api/pin-set', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({gpio:gpio, mode:13, freq:parseInt(freq)||1000})});
}
function pinDac(gpio, val) {
  var p = PD.find(x => x.gpio===gpio); if(p) p.lastValue=parseInt(val);
  fetch('/api/pin-write', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({gpio:gpio, value:parseInt(val)})});
}
function pinRead(gpio) {
  fetch('/api/pin-read?gpio='+gpio).then(r=>r.json()).then(d => {
    var p = PD.find(x => x.gpio===gpio); if(p) p.lastValue=d.value;
    var vc = document.getElementById('val-'+gpio);
    if (vc) { vc.className='vadc'; vc.textContent=d.value+' (ADC)'; }
  });
}

// ── PWM Tab ───────────────────────────────────────────────────
function renderPwm() {
  var sel = document.getElementById('pwm-gpio');
  sel.innerHTML = '';
  PD.forEach(function(p) {
    if (p.hasPWM && !p.inputOnly) {
      var o = document.createElement('option');
      o.value = p.gpio;
      o.text = 'GPIO '+p.gpio+' ('+p.label+')'+(p.mode===4?' PWM':p.mode===13?' CLK':'');
      sel.appendChild(o);
    }
  });
  var lst = document.getElementById('pwm-list');
  var act = PD.filter(p => p.mode===4);
  if (!act.length) { lst.innerHTML='<span style=color:#555>Keine aktiven PWM-Pins</span>'; return; }
  lst.innerHTML = act.map(function(p) {
    return '<div style="margin-bottom:6px;display:flex;gap:8px;align-items:center">'
      +'<span class=mono>GPIO '+p.gpio+'</span>'
      +'<span style=color:#8b949e>'+p.pwmFreq+'Hz</span>'
      +'<input type=range min=0 max=255 value='+p.pwmDuty+' style="width:100px;accent-color:#e67e22" oninput="pinPwmDuty('+p.gpio+',this.value)"> '+p.pwmDuty+'/255'
      +'</div>';
  }).join('');
}
function setPwm() {
  var gpio=parseInt(document.getElementById('pwm-gpio').value);
  var freq=parseInt(document.getElementById('pwm-fr').value)||1000;
  var duty=parseInt(document.getElementById('pwm-dn').value);
  var p=PD.find(x=>x.gpio===gpio); if(p){p.mode=4;p.pwmFreq=freq;p.pwmDuty=duty;}
  fetch('/api/pin-set',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({gpio:gpio,mode:4,freq:freq,duty:duty})})
  .then(()=>document.getElementById('pwm-res').textContent='GPIO '+gpio+' @ '+freq+'Hz Duty='+duty+'/255');
}
function stopPwm() {
  var gpio=parseInt(document.getElementById('pwm-gpio').value);
  var p=PD.find(x=>x.gpio===gpio); if(p) p.mode=0;
  fetch('/api/pin-set',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({gpio:gpio,mode:0})})
  .then(()=>document.getElementById('pwm-res').textContent='GPIO '+gpio+' gestoppt');
}

// ── I2C ───────────────────────────────────────────────────────
function i2cInit() {
  var sda=parseInt(document.getElementById('i2c-sda').value);
  var scl=parseInt(document.getElementById('i2c-scl').value);
  fetch('/api/i2c-init',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({sda:sda,scl:scl})})
  .then(r=>r.json()).then(d=>document.getElementById('i2c-sr').textContent=d.ok?'OK: SDA=GPIO'+sda+', SCL=GPIO'+scl:'Fehler');
}
function i2cScan() {
  document.getElementById('i2c-sr').textContent='Scanne...';
  fetch('/api/i2c-scan').then(r=>r.json()).then(function(d) {
    if (!d.length) { document.getElementById('i2c-sr').textContent='Keine Geraete'; return; }
    document.getElementById('i2c-sr').innerHTML=d.map(x=>x.hex+(x.name?' — <b style=color:#58a6ff>'+x.name+'</b>':'')).join('<br>');
  });
}
function i2cWrite() {
  var addr=parseInt(document.getElementById('i2c-wa').value);
  var data=document.getElementById('i2c-wd').value.trim();
  fetch('/api/i2c-write',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({addr:addr,hex:data})})
  .then(r=>r.json()).then(d=>document.getElementById('i2c-wres').textContent=d.result);
}
function i2cRead() {
  var addr=parseInt(document.getElementById('i2c-ra').value);
  var reg=parseInt(document.getElementById('i2c-rr').value);
  var len=parseInt(document.getElementById('i2c-rl').value);
  fetch('/api/i2c-read',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({addr:addr,reg:reg,len:len})})
  .then(r=>r.json()).then(d=>{
    var hx=d.data.map(b=>b.toString(16).padStart(2,'0').toUpperCase()).join(' ');
    document.getElementById('i2c-rres').textContent='HEX: '+hx+'\nDEC: '+d.data.join(', ');
  });
}

// ── SPI ───────────────────────────────────────────────────────
function spiInit() {
  var b={mosi:parseInt(document.getElementById('spi-mo').value),
         miso:parseInt(document.getElementById('spi-mi').value),
         sck:parseInt(document.getElementById('spi-sc').value),
         cs:parseInt(document.getElementById('spi-cs').value),
         freq:parseInt(document.getElementById('spi-fr').value)};
  fetch('/api/spi-init',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})
  .then(r=>r.json()).then(d=>document.getElementById('spi-st').textContent=d.ok?'SPI bereit':'Fehler');
}
function spiXfer() {
  var hex=document.getElementById('spi-tx').value.trim();
  var cs=parseInt(document.getElementById('spi-co').value);
  fetch('/api/spi-xfer',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cs:cs,hex:hex})})
  .then(r=>r.json()).then(d=>{
    var hx=d.rx.map(b=>b.toString(16).padStart(2,'0').toUpperCase()).join(' ');
    document.getElementById('spi-res').textContent='RX HEX: '+hx+'\nRX DEC: '+d.rx.join(', ');
  });
}

// ── Status ────────────────────────────────────────────────────
function fetchStatus() {
  fetch('/api/status').then(r=>r.json()).then(d=>{
    // Board-Typ setzen und SVG anpassen
    if (d.boardType && d.boardType !== BOARD) {
      BOARD = d.boardType;
      RGB_GPIO = d.rgbLed || -1;
      // RGB-Tab anzeigen wenn Board eine RGB-LED hat
      var rgbTab = document.getElementById('rgb-tab');
      if (rgbTab) rgbTab.style.display = (RGB_GPIO >= 0) ? '' : 'none';
      // Board-Titel aktualisieren
      var bt = document.getElementById('board-title');
      if (bt) bt.innerHTML = BOARD === 'esp32s3'
        ? '&#128204; ESP32-S3 Dev Module &mdash; Pinout'
        : '&#128204; Wemos D1 Mini ESP32 &mdash; Pinout';
      var svg = document.getElementById('bsvg');
      svg.setAttribute('viewBox', BOARD === 'esp32s3' ? VB_S3 : VB_32);
      // S3 PCB-Inhalt tauschen
      var pcb = document.getElementById('svgPCB');
      if (pcb) pcb.innerHTML = BOARD === 'esp32s3' ? SVG_S3_INNER : SVG_32_INNER;
      renderBoard();
    }
    document.getElementById('st-ip').textContent=d.ip;
    document.getElementById('st-mdns').innerHTML='<a href="http://'+d.mdns+'/" style=color:#58a6ff>'+d.mdns+'</a>';
    document.getElementById('st-rs').textContent=d.rssi+' dBm';
    document.getElementById('st-up').textContent=fmtU(d.uptime);
    document.getElementById('st-hp').textContent=Math.round(d.freeHeap/1024)+' KB';
    document.getElementById('st-fl').textContent=Math.round(d.freeSketch/1024)+' KB';
    document.getElementById('st-i2').textContent=d.i2cReady?'Initialisiert':'—';
    document.getElementById('st-sp').textContent=d.spiReady?'Initialisiert':'—';
  });
}
// ── RGB-LED ───────────────────────────────────────────────────
function rgbPreview() {
  var r=parseInt(document.getElementById('rgb-r').value);
  var g=parseInt(document.getElementById('rgb-g').value);
  var b=parseInt(document.getElementById('rgb-b').value);
  document.getElementById('rgb-prev').style.background='rgb('+r+','+g+','+b+')';
}
function rgbSet(r,g,b) {
  document.getElementById('rgb-r').value=r; document.getElementById('rv').textContent=r;
  document.getElementById('rgb-g').value=g; document.getElementById('gv').textContent=g;
  document.getElementById('rgb-b').value=b; document.getElementById('bv').textContent=b;
  rgbPreview(); rgbSend();
}
function rgbOff() { rgbSet(0,0,0); }
function rgbSend() {
  var r=parseInt(document.getElementById('rgb-r').value);
  var g=parseInt(document.getElementById('rgb-g').value);
  var b=parseInt(document.getElementById('rgb-b').value);
  fetch('/api/rgb',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({r:r,g:g,b:b})})
  .then(res=>res.json())
  .then(d=>{document.getElementById('rgb-res').textContent=d.ok?'OK: R='+r+' G='+g+' B='+b:'Fehler';})
  .catch(()=>document.getElementById('rgb-res').textContent='Fehler');
}

function fmtU(s){if(s<60)return s+'s';if(s<3600)return Math.floor(s/60)+'min '+s%60+'s';return Math.floor(s/3600)+'h '+Math.floor((s%3600)/60)+'min';}
)js";

    // ── HTML zusammenbauen ────────────────────────────────────────
    String html;
    html.reserve(6000);    // Nur Gerüst im RAM — CSS+JS kommen aus PROGMEM
    html += F("<!DOCTYPE html><html lang='de'><head>"
        "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>IO-Control</title><style>");
    html += FPSTR(CSS);
    html += F("</style></head><body>");

    // Header
    html += F("<header><h1>&#9889; ");
    html += deviceName;
    html += F("</h1><span class='badge'>&#127757; ");
    html += FW_VERSION;
    html += F("</span><span class='badge' id='hbb'>&#128154;</span></header>");

    // Tabs
    html += F("<div class='tabs'>"
        "<div class='tab active' onclick='showTab(\"board\",this)'>&#128204; Board</div>"
        "<div class='tab' onclick='showTab(\"gpio\",this)'>&#9889; GPIO</div>"
        "<div class='tab' onclick='showTab(\"proto\",this)'>&#128260; Protokolle</div>"
        "<div class='tab' onclick='showTab(\"pwm\",this)'>&#126; PWM</div>"
        "<div class='tab' onclick='showTab(\"status\",this)'>&#128202; Status</div>"
        "<div class='tab' id='rgb-tab' style='display:none' onclick='showTab(\"rgb\",this)'>&#127752; RGB</div>"
        "</div>");

    // ── Board Pane ────────────────────────────────────────────────
    html += F(R"html(
<div class='pane active' id='pane-board'><div class='card'>
<h3 id='board-title'>&#128204; Wemos D1 Mini ESP32 &mdash; Pinout</h3>
<svg id='bsvg' viewBox='0 0 800 300' xmlns='http://www.w3.org/2000/svg'>
  <g id='svgPCB'>
    <!-- ESP32 PCB: schmales Board, 4 Spalten eng -->
    <rect x='200' y='5' width='400' height='285' rx='6' fill='#1a4d2e' stroke='#0d3b22' stroke-width='2'/>
    <rect x='212' y='50' width='175' height='160' rx='4' fill='#111' stroke='#444' stroke-width='1.5'/>
    <text x='299' y='122' text-anchor='middle' fill='#555' font-family='monospace' font-size='9'>ESPRESSIF</text>
    <text x='299' y='133' text-anchor='middle' fill='#444' font-family='monospace' font-size='7'>ESP32-WROOM-32</text>
    <text x='299' y='143' text-anchor='middle' fill='#2a6' font-family='monospace' font-size='6'>WiFi+BT</text>
    <rect x='330' y='52' width='42' height='20' rx='2' fill='none' stroke='#2a5' stroke-width='1' stroke-dasharray='3,2'/>
    <text x='351' y='66' text-anchor='middle' fill='#2a5' font-family='sans-serif' font-size='5'>ANT</text>
    <rect x='430' y='185' width='45' height='35' rx='3' fill='#162' stroke='#0a3' stroke-width='0.8' opacity='0.7'/>
    <text x='452' y='206' text-anchor='middle' fill='#0a3' font-family='monospace' font-size='5'>CP2102</text>
    <rect x='530' y='230' width='52' height='24' rx='3' fill='#555' stroke='#888'/>
    <text x='556' y='246' text-anchor='middle' fill='#bbb' font-family='sans-serif' font-size='6'>USB</text>
    <rect x='530' y='14' width='20' height='12' rx='2' fill='#333' stroke='#666'/>
    <text x='540' y='23' text-anchor='middle' fill='#888' font-family='sans-serif' font-size='5'>RST</text>
    <circle cx='510' cy='20' r='4' fill='#1a4' stroke='#0d3' stroke-width='1'/>
    <text x='510' y='31' text-anchor='middle' fill='#2a6' font-family='sans-serif' font-size='5'>LED</text>
    <!-- Pin-Streifen: 2 Reihen links, 2 Reihen rechts -->
    <line x1='200' y1='14' x2='200' y2='270' stroke='#0d3b22' stroke-width='10' stroke-dasharray='4,18'/>
    <line x1='213' y1='14' x2='213' y2='270' stroke='#0d3b22' stroke-width='10' stroke-dasharray='4,18'/>
    <line x1='587' y1='14' x2='587' y2='270' stroke='#0d3b22' stroke-width='10' stroke-dasharray='4,18'/>
    <line x1='600' y1='14' x2='600' y2='270' stroke='#0d3b22' stroke-width='10' stroke-dasharray='4,18'/>
  </g>
</svg>
<div id='bleg' style='display:flex;flex-wrap:wrap;gap:8px;margin-top:10px'></div>
</div></div>
)html");

    // ── GPIO Pane ─────────────────────────────────────────────────
    html += F(R"html(
<div class='pane' id='pane-gpio'><div class='card'>
<h3>&#9889; GPIO Konfiguration</h3>
<p style='font-size:11px;color:#8b949e;margin-bottom:10px'>
&#128204; GPIO2=Onboard-LED (LOW=an). GPIO0=BOOT, GPIO1=TX, GPIO3=RX vorsichtig verwenden.</p>
<div style='overflow-x:auto'>
<table><thead><tr>
<th>GPIO</th><th>Label</th><th>Modus</th><th>Wert</th><th>Aktion</th>
</tr></thead>
<tbody id='gtb'><tr><td colspan='5' style='color:#555;text-align:center'>Verbinde SSE...</td></tr></tbody>
</table></div></div></div>
)html");

    // ── Protokolle Pane ───────────────────────────────────────────
    html += F(R"html(
<div class='pane' id='pane-proto'><div class='grid2'>
<div class='card'><h3>&#128260; I2C</h3>
<div class='row'>
  <label>SDA</label><input type='number' id='i2c-sda' value='21' style='width:55px'>
  <label>SCL</label><input type='number' id='i2c-scl' value='22' style='width:55px'>
  <button class='btn btn-b' onclick='i2cInit()'>Init</button>
</div>
<button class='btn btn-g' style='width:100%;margin-bottom:8px' onclick='i2cScan()'>&#128269; Scanner</button>
<div class='rbox' id='i2c-sr'>&#8212;</div>
<hr style='border-color:#21262d;margin:12px 0'>
<div class='row'>
  <label>Addr</label><input type='text' id='i2c-wa' value='0x3C' style='width:65px'>
  <label>Reg</label><input type='text' id='i2c-wr' value='-1' style='width:45px'>
</div>
<label>Write Bytes (hex: 00 FF A0)</label>
<textarea id='i2c-wd' rows='2' placeholder='00 FF...'></textarea>
<button class='btn btn-g' style='margin-top:6px' onclick='i2cWrite()'>&#128228; Schreiben</button>
<div class='rbox' id='i2c-wres'>&#8212;</div>
<hr style='border-color:#21262d;margin:12px 0'>
<div class='row'>
  <label>Addr</label><input type='text' id='i2c-ra' value='0x3C' style='width:65px'>
  <label>Reg</label><input type='text' id='i2c-rr' value='0x00' style='width:45px'>
  <label>Len</label><input type='number' id='i2c-rl' value='4' style='width:50px'>
</div>
<button class='btn btn-b' onclick='i2cRead()'>&#128229; Lesen</button>
<div class='rbox' id='i2c-rres'>&#8212;</div>
</div>
<div class='card'><h3>&#128299; SPI (VSPI)</h3>
<div class='row'>
  <label>MOSI</label><input type='number' id='spi-mo' value='23' style='width:50px'>
  <label>MISO</label><input type='number' id='spi-mi' value='19' style='width:50px'>
</div>
<div class='row'>
  <label>SCK</label><input type='number' id='spi-sc' value='18' style='width:50px'>
  <label>CS</label><input type='number' id='spi-cs' value='5' style='width:50px'>
</div>
<div class='row'>
  <label>Freq Hz</label><input type='number' id='spi-fr' value='1000000' style='width:110px'>
</div>
<button class='btn btn-b' style='margin-bottom:8px' onclick='spiInit()'>Init SPI</button>
<span id='spi-st' style='margin-left:8px;font-size:11px;color:#8b949e'></span>
<hr style='border-color:#21262d;margin:12px 0'>
<label>TX Bytes (hex: 9F 00 00)</label>
<textarea id='spi-tx' rows='2' placeholder='9F 00 00'></textarea>
<div class='row' style='margin-top:6px'>
  <label>CS Override (-1=default)</label>
  <input type='number' id='spi-co' value='-1' style='width:55px'>
</div>
<button class='btn btn-g' onclick='spiXfer()'>&#9889; Transfer</button>
<div class='rbox' id='spi-res'>&#8212;</div>
</div></div></div>
)html");

    // ── PWM Pane ──────────────────────────────────────────────────
    html += F(R"html(
<div class='pane' id='pane-pwm'>
<div class='card'><h3>&#126; PWM / LEDC</h3>
<div class='row'>
  <label>GPIO</label>
  <select id='pwm-gpio' style='width:165px'></select>
  <label>Freq Hz</label>
  <input type='number' id='pwm-fr' value='1000' style='width:95px'>
</div>
<div class='row' style='align-items:center'>
  <label>Duty 0-255</label>
  <input type='range' id='pwm-dr' min='0' max='255' value='128' style='flex:1;accent-color:#e67e22'
    oninput='document.getElementById("pwm-dn").value=this.value'>
  <input type='number' id='pwm-dn' value='128' min='0' max='255' style='width:55px'
    oninput='document.getElementById("pwm-dr").value=this.value'>
</div>
<div class='row' style='margin-top:4px'>
  <button class='btn btn-g' onclick='setPwm()'>&#9889; Starten</button>
  <button class='btn btn-r' onclick='stopPwm()' style='margin-left:6px'>&#9632; Stoppen</button>
</div>
<div class='rbox' id='pwm-res'>&#8212;</div>
<hr style='border-color:#21262d;margin:14px 0'>
<h3 style='margin-bottom:8px'>Aktive PWM-Pins</h3>
<div id='pwm-list'>&#8212;</div>
</div>
<div class='card'><h3>&#9432; Info</h3>
<table>
<tr><td style='color:#8b949e'>Max. Freq</td><td>~40 MHz (8-bit LEDC)</td></tr>
<tr><td style='color:#8b949e'>CLOCK &lt; 50Hz</td><td>Software-Toggle (micros)</td></tr>
<tr><td style='color:#8b949e'>CLOCK &ge; 50Hz</td><td>LEDC Hardware-PWM 50% Duty</td></tr>
</table></div></div>
)html");

    // ── Status Pane ───────────────────────────────────────────────
    html += F("<div class='pane' id='pane-status'><div class='card'><h3>&#128202; Systeminfo</h3><table>"
        "<tr><td style='color:#8b949e'>Name</td><td>");
    html += deviceName;
    html += F("</td></tr><tr><td style='color:#8b949e'>Chip</td><td>");
    html += String(ESP.getChipModel());
    html += F("</td></tr><tr><td style='color:#8b949e'>MAC</td><td class='mono'>");
    html += getMac();
    html += F(R"html(</td></tr>
<tr><td style='color:#8b949e'>IP</td><td class='mono' id='st-ip'>-</td></tr>
<tr><td style='color:#8b949e'>mDNS</td><td class='mono' id='st-mdns'>-</td></tr>
<tr><td style='color:#8b949e'>RSSI</td><td id='st-rs'>-</td></tr>
<tr><td style='color:#8b949e'>Uptime</td><td id='st-up'>-</td></tr>
<tr><td style='color:#8b949e'>Heap</td><td id='st-hp'>-</td></tr>
<tr><td style='color:#8b949e'>Flash</td><td id='st-fl'>-</td></tr>
<tr><td style='color:#8b949e'>I2C</td><td id='st-i2'>-</td></tr>
<tr><td style='color:#8b949e'>SPI</td><td id='st-sp'>-</td></tr>
</table></div>
<div class='card'><h3>&#128640; OTA Update</h3>
<a href='/ota' class='btn btn-g' style='text-decoration:none;display:inline-block'>&#128640; OTA Update</a>
</div></div>
)html");

    // ── RGB Pane (nur sichtbar wenn RGB-LED vorhanden) ────────────
    html += F(R"html(
<div class='pane' id='pane-rgb'>
<div class='card'><h3>&#127752; RGB-LED Steuerung</h3>
<p style='font-size:11px;color:#8b949e;margin-bottom:12px'>
WS2812 an IO38 &mdash; neopixelWrite() &mdash; kein separates Library noetig (Core 3.x)</p>
<div class='row'>
  <label>Rot</label>
  <input type='range' id='rgb-r' min='0' max='255' value='0' style='flex:1;accent-color:#e74c3c'
    oninput='document.getElementById("rv").textContent=this.value;rgbPreview()'>
  <span id='rv' style='width:28px;text-align:right;font-family:monospace'>0</span>
</div>
<div class='row'>
  <label>Gruen</label>
  <input type='range' id='rgb-g' min='0' max='255' value='0' style='flex:1;accent-color:#2ecc71'
    oninput='document.getElementById("gv").textContent=this.value;rgbPreview()'>
  <span id='gv' style='width:28px;text-align:right;font-family:monospace'>0</span>
</div>
<div class='row'>
  <label>Blau</label>
  <input type='range' id='rgb-b' min='0' max='255' value='0' style='flex:1;accent-color:#3498db'
    oninput='document.getElementById("bv").textContent=this.value;rgbPreview()'>
  <span id='bv' style='width:28px;text-align:right;font-family:monospace'>0</span>
</div>
<div class='row' style='margin-top:8px;align-items:center'>
  <div id='rgb-prev' style='width:48px;height:48px;border-radius:50%;border:2px solid #30363d;background:#000;margin-right:8px'></div>
  <div style='display:flex;flex-direction:column;gap:6px;flex:1'>
    <button class='btn btn-g' onclick='rgbSend()'>&#9889; Senden</button>
    <button class='btn' onclick='rgbOff()'>&#9632; Aus (0,0,0)</button>
  </div>
</div>
<hr style='border-color:#21262d;margin:14px 0'>
<h3 style='margin-bottom:8px'>Schnellfarben</h3>
<div style='display:flex;flex-wrap:wrap;gap:8px'>
  <button class='btn' style='background:#e00' onclick='rgbSet(255,0,0)'>Rot</button>
  <button class='btn' style='background:#0a0' onclick='rgbSet(0,255,0)'>Gruen</button>
  <button class='btn' style='background:#00c' onclick='rgbSet(0,0,255)'>Blau</button>
  <button class='btn' style='background:#880' onclick='rgbSet(255,255,0)'>Gelb</button>
  <button class='btn' style='background:#088' onclick='rgbSet(0,255,255)'>Cyan</button>
  <button class='btn' style='background:#808' onclick='rgbSet(255,0,255)'>Magenta</button>
  <button class='btn' style='background:#fff;color:#000' onclick='rgbSet(255,255,255)'>Weiss</button>
  <button class='btn' style='background:#f80' onclick='rgbSet(255,128,0)'>Orange</button>
  <button class='btn' style='background:#048' onclick='rgbSet(0,64,128)'>Dunkelblau</button>
</div>
<div class='rbox' id='rgb-res' style='margin-top:10px'>&#8212;</div>
</div></div>
)html");

    // Script
    html += F("<script>");
    html += FPSTR(JS);
    html += F("</script></body></html>");

    webServer.send(200, "text/html", html);
}

// ================================================================
//  OTA
// ================================================================
void handleOtaPage() {
    static const char OTA_HTML[] PROGMEM = R"html(
<!DOCTYPE html><html><head><meta charset='UTF-8'><title>OTA</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#e6edf3;font-family:sans-serif;font-size:14px}
header{background:#161b22;border-bottom:1px solid #30363d;padding:14px 20px}
h1{font-size:18px;color:#58a6ff}
.content{padding:20px;max-width:600px}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:20px}
.drop{border:2px dashed #30363d;border-radius:8px;padding:32px;text-align:center;cursor:pointer;color:#8b949e}
.drop:hover{border-color:#58a6ff}
.btn{background:#238636;color:#fff;border:none;padding:10px 24px;border-radius:6px;cursor:pointer;width:100%;font-size:14px;margin-top:14px}
.bar{height:8px;background:#21262d;border-radius:4px;overflow:hidden;margin-top:14px}
.bf{height:100%;background:#58a6ff;border-radius:4px;width:0%;transition:width .3s}
.msg{margin-top:8px;font-size:13px;color:#8b949e;text-align:center}
input[type=file]{display:none}
</style></head><body>
)html";

    String html = FPSTR(OTA_HTML);
    html += F("<header><h1>&#128640; OTA &mdash; ");
    html += deviceName;
    html += F(" <small style='font-size:13px;color:#8b949e'>");
    html += FW_VERSION;
    html += F(R"html(</small></h1></header>
<div class='content'><div class='card'>
<div class='drop' id='dp' onclick='document.getElementById("fw").click()'>
  &#128190; <b>.bin</b> hier ablegen oder klicken
</div>
<input type='file' id='fw' accept='.bin'>
<div id='fn' style='margin-top:8px;font-size:12px;color:#8b949e;text-align:center'></div>
<button class='btn' onclick='go()'>&#9889; Flashen</button>
<div class='bar'><div class='bf' id='bar'></div></div>
<div class='msg' id='msg'></div>
</div></div>
<script>
var inp=document.getElementById('fw');
inp.onchange=function(){if(inp.files[0])document.getElementById('fn').textContent=inp.files[0].name+' ('+Math.round(inp.files[0].size/1024)+' KB)';};
var dp=document.getElementById('dp');
dp.ondragover=function(e){e.preventDefault();dp.style.borderColor='#58a6ff';};
dp.ondragleave=function(){dp.style.borderColor='#30363d';};
dp.ondrop=function(e){
  e.preventDefault();dp.style.borderColor='#30363d';
  var f=e.dataTransfer.files[0];
  if(f&&f.name.endsWith('.bin')){var dt=new DataTransfer();dt.items.add(f);inp.files=dt.files;
    document.getElementById('fn').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';}
};
function go(){
  if(!inp.files[0]){alert('Bitte .bin auswaehlen');return;}
  var fd=new FormData();fd.append('firmware',inp.files[0]);
  var x=new XMLHttpRequest();x.open('POST','/ota-upload');
  x.upload.onprogress=function(e){if(e.lengthComputable){
    var p=Math.round(e.loaded/e.total*100);
    document.getElementById('bar').style.width=p+'%';
    document.getElementById('msg').textContent='Hochladen: '+p+'%';}};
  x.onload=function(){
    document.getElementById('msg').textContent=x.status===200?'Erfolgreich! Neustart...':'Fehler: '+x.responseText;
    document.getElementById('bar').style.background=x.status===200?'#3fb950':'#f85149';};
  x.send(fd);
}
</script></body></html>
)html");

    webServer.send(200, "text/html", html);
}

void handleOtaUpload() {
    HTTPUpload& u = webServer.upload();
    if      (u.status == UPLOAD_FILE_START)
        { if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial); }
    else if (u.status == UPLOAD_FILE_WRITE)
        { if (Update.write(u.buf, u.currentSize) != u.currentSize) Update.printError(Serial); }
    else if (u.status == UPLOAD_FILE_END)
        { if (!Update.end(true)) Update.printError(Serial); }
}

void handleOtaUploadFinish() {
    if (Update.hasError()) webServer.send(500, "text/plain", "OTA failed");
    else                   webServer.send(200, "text/plain", "OK");
    delay(500); ESP.restart();
}

void handleNotFound() {
    webServer.sendHeader("Location", "/", true); webServer.send(302);
}

// ================================================================
//  WEB SERVER SETUP
// ================================================================
void setupWebServer() {
    webServer.on("/",              HTTP_GET,  handleRoot);
    webServer.on("/ota",           HTTP_GET,  handleOtaPage);
    webServer.on("/ota-upload",    HTTP_POST, handleOtaUploadFinish, handleOtaUpload);
    webServer.on("/api/events",    HTTP_GET,  handleSSE);
    webServer.on("/api/pins",      HTTP_GET,  handleApiPins);
    webServer.on("/api/pin-set",   HTTP_POST, handleApiPinSet);
    webServer.on("/api/pin-write", HTTP_POST, handleApiPinWrite);
    webServer.on("/api/pin-read",  HTTP_GET,  handleApiPinRead);
    webServer.on("/api/i2c-init",  HTTP_POST, handleApiI2cInit);
    webServer.on("/api/i2c-scan",  HTTP_GET,  handleApiI2cScan);
    webServer.on("/api/i2c-write", HTTP_POST, handleApiI2cWrite);
    webServer.on("/api/i2c-read",  HTTP_POST, handleApiI2cRead);
    webServer.on("/api/spi-init",  HTTP_POST, handleApiSpiInit);
    webServer.on("/api/spi-xfer",  HTTP_POST, handleApiSpiXfer);
    webServer.on("/api/rgb",       HTTP_POST, handleApiRgb);
    webServer.on("/api/status",    HTTP_GET,  handleApiStatus);
    webServer.onNotFound(handleNotFound);
    webServer.begin();
    Serial.println("[WEB] http://" + getLocalIp() + "/");
}

// ================================================================
//  RESET + WIFI
// ================================================================
void checkResetButton() {
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(RESET_BUTTON_PIN) == HIGH) return;
    unsigned long t = millis();
    while (digitalRead(RESET_BUTTON_PIN) == LOW) {
        if (millis()-t > (unsigned long)RESET_HOLD_SEC*1000UL) {
            wifiManager.resetSettings();
            prefs.begin("esphub",false); prefs.clear(); prefs.end();
            prefs.begin("ioctrl",false); prefs.clear(); prefs.end();
            delay(500); ESP.restart();
        }
        delay(100);
    }
}

void setupWifi() {
    prefs.begin("esphub", false);
    String sn = prefs.getString("name",     deviceName);
    String sh = prefs.getString("hub_host", hubHost);
    int    sp = prefs.getInt   ("hub_port", hubPort);
    prefs.end();
    deviceName = sn; hubHost = sh; hubPort = sp;

    WiFiManagerParameter pN("name",     "Geraetename", deviceName.c_str(), 32);
    WiFiManagerParameter pH("hub_host", "ESP-Hub IP",  hubHost.c_str(),    40);
    WiFiManagerParameter pP("hub_port", "ESP-Hub Port",String(hubPort).c_str(), 6);
    wifiManager.addParameter(&pN);
    wifiManager.addParameter(&pH);
    wifiManager.addParameter(&pP);
    wifiManager.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT_S);
    wifiManager.setAPCallback([](WiFiManager*){
        Serial.println("[WiFi] Portal: " WIFI_AP_NAME);
    });
    if (!wifiManager.autoConnect(WIFI_AP_NAME)) {
        delay(1000); ESP.restart();
    }
    prefs.begin("esphub", false);
    prefs.putString("name",     String(pN.getValue()));
    prefs.putString("hub_host", String(pH.getValue()));
    prefs.putInt   ("hub_port", String(pP.getValue()).toInt());
    prefs.end();
    deviceName = String(pN.getValue());
    hubHost    = String(pH.getValue());
    hubPort    = String(pP.getValue()).toInt();
    Serial.println("[WiFi] IP:" + getLocalIp() + " Name:" + deviceName);
}

// ================================================================
//  HEARTBEAT (ESP-Hub)
// ================================================================
String buildHeartbeat() {
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(1024);
    #endif
    doc["mac"]        = getMac();
    doc["name"]       = deviceName;
    doc["hwType"]     = "esp32";
    doc["chipModel"]  = ESP.getChipModel();
    doc["version"]    = FW_VERSION;
    doc["ip"]         = getLocalIp();
    doc["rssi"]       = WiFi.RSSI();
    doc["uptime"]     = millis() / 1000UL;
    doc["freeHeap"]   = ESP.getFreeHeap();
    doc["freeSketch"] = ESP.getFreeSketchSpace();
    JsonObject ios = doc["ios"].to<JsonObject>();
    for (int i = 0; i < PIN_COUNT; i++) {
        if (pins[i].mode == PM_DISABLED) continue;
        String k = "gpio" + String(pins[i].gpio);
        JsonObject io = ios[k].to<JsonObject>();
        io["type"]  = (pins[i].mode >= PM_OUTPUT) ? "output" : "sensor";
        io["value"] = (float)pins[i].lastValue;
    }
    String out; serializeJson(doc, out); return out;
}

void sendHeartbeat() {
    if (WiFi.status() != WL_CONNECTED) return;
    readAllPins();
    HTTPClient http;
    http.begin("http://" + hubHost + ":" + String(hubPort) + "/api/register");
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(8000);
    int code = http.POST(buildHeartbeat());
    if (code == 200) {
        #if ARDUINOJSON_VERSION_MAJOR >= 7
          JsonDocument resp;
        #else
          DynamicJsonDocument resp(512);
        #endif
        if (deserializeJson(resp, http.getString()) == DeserializationError::Ok) {
            if (resp.containsKey("interval")) {
                unsigned long ni = (unsigned long)(int)resp["interval"] * 1000UL;
                if (ni >= 5000UL) heartbeatInterval = ni;
            }
            if (resp.containsKey("otaUrl") && !resp["otaUrl"].isNull()) {
                String u = resp["otaUrl"].as<String>();
                if (u.length() > 0) { otaPending = true; otaUrl = u; }
            }
            // Name-Sync: Hub kann einen gespeicherten Namen zuruecksenden
            if (resp.containsKey("name") && !resp["name"].isNull()) {
                String newName = resp["name"].as<String>();
                if (newName.length() > 0 && newName != deviceName) {
                    deviceName = newName;
                    prefs.begin("esphub", false);
                    prefs.putString("name", deviceName);
                    prefs.end();
                    Serial.println("[HB] Name vom Hub: " + deviceName);
                }
            }
        }
    }
    http.end();
}

void performOta(const String& url) {
    HTTPClient http; http.begin(url); http.setTimeout(30000);
    if (http.GET() != 200) { http.end(); return; }
    int tl = http.getSize();
    if (!Update.begin(tl > 0 ? tl : UPDATE_SIZE_UNKNOWN)) { http.end(); return; }
    WiFiClient* s = http.getStreamPtr();
    uint8_t buf[512]; size_t w = 0;
    while (http.connected() && (tl <= 0 || w < (size_t)tl)) {
        size_t av = s->available(); if (!av) { delay(1); continue; }
        size_t r = s->readBytes(buf, min(av, sizeof(buf))); if (!r) break;
        Update.write(buf, r); w += r;
    }
    if (Update.end(true)) { http.end(); delay(500); ESP.restart(); }
    http.end();
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
    Serial.begin(115200); delay(500);
    Serial.println("\n=== " FW_VERSION " ===");
    checkResetButton();
    setupWifi();
    String mdnsName = toMdnsName(deviceName);
    if (MDNS.begin(mdnsName.c_str())) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("[mDNS] http://" + mdnsName + ".local/");
    }
    loadModesFromPrefs();
    // Auf Chips ohne DAC (S3, C3, S2): hasDAC deaktivieren
    #if !DAC_SUPPORTED
    for (int i = 0; i < PIN_COUNT; i++) {
        pins[i].hasDAC = false;
        if (pins[i].mode == PM_DAC) pins[i].mode = PM_INPUT;
    }
    #endif
    for (int i = 0; i < PIN_COUNT; i++) applyPinMode(&pins[i]);
    // RGB-LED: applyPinMode(PM_RGB) initialisiert IO38 bereits als OUTPUT + aus
    setupWebServer();
    lastHeartbeat = millis();
    sendHeartbeat();
    Serial.println("[READY] http://" + getLocalIp() + "/");
}

void loop() {
    tickSoftClocks();    // Software-Takt (< 50 Hz)
    driveOutputPins();   // OUTPUT-Pins aktiv halten

    // SSE: periodisch Pins pushen (auch ohne Aenderung alle 5s als Keepalive)
    if (sseAlive && millis() - sseLastPush > 5000) {
        pushPinsSSE();
    }

    webServer.handleClient();

    unsigned long now = millis();
    if (now - lastHeartbeat >= heartbeatInterval) {
        lastHeartbeat = now;
        sendHeartbeat();
    }
    if (otaPending) { otaPending = false; performOta(otaUrl); otaUrl = ""; }

    if (WiFi.status() != WL_CONNECTED) {
        delay(5000);
        if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
    }
}
