// ╔══════════════════════════════════════════════════════════════╗
// ║  esp32.io-control.ino — ESP32 Universal IO Controller       ║
// ║  Version: 1.6.1                                             ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  Bibliotheken (Arduino Library Manager):                    ║
// ║    - WiFiManager  von tablatronix / tzapu                   ║
// ║    - ArduinoJson  von bblanchon (v6 oder v7)                ║
// ║  Architektur:                                               ║
// ║    - Raw String Literals (R"html(...)html") fuer HTML/JS    ║
// ║    - SSE (Server-Sent Events) statt Polling                 ║
// ║    - driveOutputPins() gegen WiFi-Stack-Eingriffe           ║
// ║    - Software-Takt fuer CLOCK < 50 Hz                       ║
// ║    - Pin-Schutz, ADC/DAC Volt, board-spezifische Bus-Pins   ║
// ║    - Interrupt-Counter (COUNT) + ADC Mini-Oszi              ║
// ║    - Taktgenerator-Tab + Digital-Multimeter-Tab            ║
// ╚══════════════════════════════════════════════════════════════╝

#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "esphub/HardwareInfo.h"
#include "esphub/CpuLoad.h"
#include "web/UiPages.h"
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

// v1.7.0: esp-hub-base Shared-Core A/B (HardwareInfo/CpuLoad,
//         web/*.html → gzip-PROGMEM wie Base 1.10.5)
// ================================================================
//  KONFIGURATION
// ================================================================
#define DEVICE_NAME           "IO-Control"
#define FW_VERSION            "1.7.0"
#define HUB_HOST              "192.168.178.113"
#define HUB_PORT              8093
#define WIFI_AP_NAME          "ESP-IO-Setup"
#define WIFI_PORTAL_TIMEOUT_S 180
#define DEFAULT_INTERVAL_S    30
#define RESET_BUTTON_PIN      0
#define RESET_HOLD_SEC        3
#define DAC_VREF_MV           3300
#define COUNT_MAX             8
#define SCOPE_MAX_SAMPLES     400

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
#define PM_COUNT     15   // Interrupt-Flankenzaehler (FALLING)
#define PM_MODE_MAX  16

const char* modeNames[PM_MODE_MAX] = {
    "DISABLED","INPUT","INPUT_PU","OUTPUT","PWM","ADC","DAC",
    "I2C_SDA","I2C_SCL","SPI_MOSI","SPI_MISO","SPI_SCK","SPI_CS","CLOCK","RGB","COUNT"
};
const char* modeColors[PM_MODE_MAX] = {
    "#444","#2ecc71","#27ae60","#e74c3c","#e67e22","#9b59b6","#8e44ad",
    "#3498db","#2980b9","#1abc9c","#16a085","#17a589","#0e6655","#f39c12","#e91e8c","#f1c40f"
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
    int         lastMv;    // runtime: ADC mV oder DAC-Soll in mV
    uint32_t    countHz;   // runtime: COUNT Frequenz (Hz)
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
HardwareInfo hwInfo;
String        hubHost           = HUB_HOST;
int           hubPort           = HUB_PORT;
unsigned long lastHeartbeat     = 0;
unsigned long heartbeatInterval = (unsigned long)DEFAULT_INTERVAL_S * 1000UL;
bool          otaPending        = false;
String        otaUrl            = "";
bool          i2cReady          = false;
bool          spiReady          = false;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
int           i2cSda = 8,  i2cScl = 9;
int           spiMosi = 11, spiMiso = 13, spiSck = 12, spiCs = 10;
#else
int           i2cSda = 21, i2cScl = 22;
int           spiMosi = 23, spiMiso = 19, spiSck = 18, spiCs = 5;
#endif
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

// Interrupt-Flankenzaehler (COUNT)
struct EdgeCounter {
    uint8_t           gpio        = 255;
    volatile uint32_t count       = 0;
    uint32_t          lastCount   = 0;
    uint32_t          freqHz      = 0;
    unsigned long     windowStart = 0;
};
static EdgeCounter edgeCounters[COUNT_MAX];
static uint16_t    scopeBuf[SCOPE_MAX_SAMPLES];

// ================================================================
//  FORWARD DECLARATIONS
// ================================================================
PinInfo*    findPin(int gpio);
bool        isRestrictedGpio(int gpio);
bool        isAdc2Gpio(int gpio);
bool        isDriveMode(int mode);
const char* hubIoType(uint8_t mode);
int         dacRawFromVolts(float volts);
float       dacVoltsFromRaw(int raw);
void        releasePinHardware(PinInfo* p);
void        syncI2cFromPinModes();
void        syncSpiFromPinModes();
void        applyPinMode(PinInfo* p);
void        readAllPins();
void        driveOutputPins();
void        tickSoftClocks();
void        startSoftClock(uint8_t gpio, uint32_t freqHz);
void        stopSoftClock(uint8_t gpio);
void        startEdgeCounter(uint8_t gpio);
void        stopEdgeCounter(uint8_t gpio);
void        tickEdgeCounters();
void        resetEdgeCounter(uint8_t gpio);
uint32_t    getEdgeCount(uint8_t gpio);
uint32_t    getEdgeFreq(uint8_t gpio);
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
void        handleApiScope();
void        handleApiCountPulse();
void        handleApiCountReset();
void        handleApiDmm();
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

bool isRestrictedGpio(int gpio) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    return gpio == 0 || gpio == 19 || gpio == 20 || gpio == 43 || gpio == 44;
#else
    return gpio == 0 || gpio == 1 || gpio == 3;
#endif
}

bool isAdc2Gpio(int gpio) {
#if defined(CONFIG_IDF_TARGET_ESP32)
    return gpio == 0 || gpio == 2 || gpio == 4
        || (gpio >= 12 && gpio <= 15)
        || (gpio >= 25 && gpio <= 27);
#else
    (void)gpio;
    return false;
#endif
}

bool isDriveMode(int mode) {
    return mode == PM_OUTPUT || mode == PM_PWM || mode == PM_DAC
        || mode == PM_CLOCK || mode == PM_RGB
        || (mode >= PM_I2C_SDA && mode <= PM_SPI_CS);
}

const char* hubIoType(uint8_t mode) {
    switch (mode) {
        case PM_INPUT:
        case PM_INPUT_PU:
        case PM_ADC:
        case PM_COUNT:
            return "sensor";
        case PM_OUTPUT:
        case PM_PWM:
        case PM_DAC:
        case PM_CLOCK:
        case PM_RGB:
            return "output";
        default:
            return nullptr; // DISABLED + Bus-Modi nicht melden
    }
}

int dacRawFromVolts(float volts) {
    if (volts < 0) volts = 0;
    if (volts > 3.3f) volts = 3.3f;
    return (int)constrain((int)lroundf(volts / 3.3f * 255.0f), 0, 255);
}

float dacVoltsFromRaw(int raw) {
    raw = constrain(raw, 0, 255);
    return (raw / 255.0f) * 3.3f;
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
    if (freqHz == 0) freqHz = 1;
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
//  EDGE COUNTER (COUNT)
// ================================================================
void IRAM_ATTR edgeIsr(void* arg) {
    EdgeCounter* c = (EdgeCounter*)arg;
    if (c) c->count++;
}

void stopEdgeCounter(uint8_t gpio) {
    for (int i = 0; i < COUNT_MAX; i++) {
        if (edgeCounters[i].gpio == gpio) {
            detachInterrupt(digitalPinToInterrupt(gpio));
            edgeCounters[i].gpio = 255;
            edgeCounters[i].count = 0;
            edgeCounters[i].lastCount = 0;
            edgeCounters[i].freqHz = 0;
            return;
        }
    }
}

void startEdgeCounter(uint8_t gpio) {
    stopEdgeCounter(gpio);
    for (int i = 0; i < COUNT_MAX; i++) {
        if (edgeCounters[i].gpio == 255) {
            edgeCounters[i].gpio = gpio;
            edgeCounters[i].count = 0;
            edgeCounters[i].lastCount = 0;
            edgeCounters[i].freqHz = 0;
            edgeCounters[i].windowStart = millis();
            pinMode(gpio, INPUT_PULLUP);
            attachInterruptArg(digitalPinToInterrupt(gpio), edgeIsr, &edgeCounters[i], FALLING);
            return;
        }
    }
}

void resetEdgeCounter(uint8_t gpio) {
    for (int i = 0; i < COUNT_MAX; i++) {
        if (edgeCounters[i].gpio == gpio) {
            noInterrupts();
            edgeCounters[i].count = 0;
            edgeCounters[i].lastCount = 0;
            edgeCounters[i].freqHz = 0;
            edgeCounters[i].windowStart = millis();
            interrupts();
            return;
        }
    }
}

uint32_t getEdgeCount(uint8_t gpio) {
    for (int i = 0; i < COUNT_MAX; i++)
        if (edgeCounters[i].gpio == gpio) return edgeCounters[i].count;
    return 0;
}

uint32_t getEdgeFreq(uint8_t gpio) {
    for (int i = 0; i < COUNT_MAX; i++)
        if (edgeCounters[i].gpio == gpio) return edgeCounters[i].freqHz;
    return 0;
}

void tickEdgeCounters() {
    unsigned long now = millis();
    for (int i = 0; i < COUNT_MAX; i++) {
        if (edgeCounters[i].gpio == 255) continue;
        if (now - edgeCounters[i].windowStart >= 1000UL) {
            uint32_t c = edgeCounters[i].count;
            edgeCounters[i].freqHz = c - edgeCounters[i].lastCount;
            edgeCounters[i].lastCount = c;
            edgeCounters[i].windowStart = now;
        }
    }
}

// ================================================================
//  PIN MANAGEMENT
// ================================================================
void releasePinHardware(PinInfo* p) {
    if (!p) return;
    stopSoftClock(p->gpio);
    stopEdgeCounter(p->gpio);
    if (p->hasPWM) {
        ledcDetach(p->gpio);
    }
}

void syncI2cFromPinModes() {
    int sda = -1, scl = -1;
    for (int i = 0; i < PIN_COUNT; i++) {
        if (pins[i].mode == PM_I2C_SDA) sda = pins[i].gpio;
        if (pins[i].mode == PM_I2C_SCL) scl = pins[i].gpio;
    }
    if (sda >= 0 && scl >= 0) initI2C(sda, scl);
}

void syncSpiFromPinModes() {
    int mosi = -1, miso = -1, sck = -1, cs = -1;
    for (int i = 0; i < PIN_COUNT; i++) {
        if (pins[i].mode == PM_SPI_MOSI) mosi = pins[i].gpio;
        if (pins[i].mode == PM_SPI_MISO) miso = pins[i].gpio;
        if (pins[i].mode == PM_SPI_SCK)  sck  = pins[i].gpio;
        if (pins[i].mode == PM_SPI_CS)   cs   = pins[i].gpio;
    }
    if (mosi >= 0 && miso >= 0 && sck >= 0) {
        initSPIBus(mosi, miso, sck, cs >= 0 ? cs : -1, spiFreq);
    }
}

void applyPinMode(PinInfo* p) {
    if (!p) return;
    releasePinHardware(p);
    switch (p->mode) {
        case PM_INPUT:    pinMode(p->gpio, INPUT);         break;
        case PM_INPUT_PU: pinMode(p->gpio, INPUT_PULLUP);  break;
        case PM_OUTPUT:
            pinMode(p->gpio, OUTPUT);
            digitalWrite(p->gpio, p->lastValue ? HIGH : LOW);
            break;
        case PM_PWM:
            if (!p->inputOnly && p->hasPWM) {
                ledcAttach(p->gpio, p->pwmFreq ? p->pwmFreq : 1000, 8);
                ledcWrite(p->gpio, p->pwmDuty);
            }
            break;
        case PM_CLOCK:
            if (!p->inputOnly && p->hasPWM) {
                if (p->pwmFreq >= SW_CLK_THRESHOLD) {
                    ledcAttach(p->gpio, p->pwmFreq, 8);
                    ledcWrite(p->gpio, 127);
                } else {
                    pinMode(p->gpio, OUTPUT);
                    startSoftClock(p->gpio, p->pwmFreq ? p->pwmFreq : 1);
                }
            }
            break;
        case PM_ADC:
            pinMode(p->gpio, INPUT);
            analogSetPinAttenuation(p->gpio, ADC_11db);
            break;
        case PM_DAC:
            if (p->hasDAC) {
                p->lastValue = constrain(p->lastValue, 0, 255);
                p->lastMv = (int)lroundf(dacVoltsFromRaw(p->lastValue) * 1000.0f);
                dacWrite(p->gpio, (uint8_t)p->lastValue);
            }
            break;
        case PM_RGB:
            pinMode(p->gpio, OUTPUT);
            neopixelWrite(p->gpio, 0, 0, 0);
            break;
        case PM_COUNT:
            startEdgeCounter(p->gpio);
            p->lastValue = 0;
            p->countHz = 0;
            break;
        case PM_I2C_SDA:
        case PM_I2C_SCL:
            pinMode(p->gpio, INPUT_PULLUP);
            syncI2cFromPinModes();
            break;
        case PM_SPI_MOSI:
        case PM_SPI_MISO:
        case PM_SPI_SCK:
            pinMode(p->gpio, INPUT);
            syncSpiFromPinModes();
            break;
        case PM_SPI_CS:
            pinMode(p->gpio, OUTPUT);
            digitalWrite(p->gpio, HIGH);
            syncSpiFromPinModes();
            break;
        case PM_DISABLED:
        default:
            if (!p->inputOnly) pinMode(p->gpio, INPUT);
            break;
    }
}

void readAllPins() {
    tickEdgeCounters();
    for (int i = 0; i < PIN_COUNT; i++) {
        PinInfo& p = pins[i];
        if (p.mode == PM_INPUT || p.mode == PM_INPUT_PU) {
            p.lastValue = digitalRead(p.gpio);
            p.lastMv = 0;
        } else if (p.mode == PM_ADC) {
            p.lastValue = analogRead(p.gpio);
            p.lastMv = analogReadMilliVolts(p.gpio);
        } else if (p.mode == PM_DAC) {
            p.lastMv = (int)lroundf(dacVoltsFromRaw(p.lastValue) * 1000.0f);
        } else if (p.mode == PM_COUNT) {
            p.lastValue = (int)getEdgeCount(p.gpio);
            p.countHz = getEdgeFreq(p.gpio);
        }
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
        j += ",\"restricted\":"+ String(isRestrictedGpio(p.gpio) ? "true":"false");
        j += ",\"adc2\":"      + String(isAdc2Gpio(p.gpio) ? "true":"false");
        j += ",\"mode\":"      + String(p.mode);
        j += ",\"modeStr\":\"" + String(modeNames[p.mode]) + "\"";
        j += ",\"value\":"     + String(p.lastValue);
        j += ",\"mV\":"        + String(p.lastMv);
        j += ",\"countHz\":"   + String(p.countHz);
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
    bool force = doc["force"] | false;
    if (gpio < 0 || mode < 0 || mode >= PM_MODE_MAX) {
        webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"param\"}"); return;
    }
    PinInfo* p = findPin(gpio);
    if (!p) { webServer.send(404, "application/json", "{\"ok\":false,\"err\":\"not_found\"}"); return; }
    if (p->inputOnly && (mode==PM_OUTPUT||mode==PM_PWM||mode==PM_DAC||mode==PM_CLOCK||mode==PM_RGB
            || (mode >= PM_I2C_SDA && mode <= PM_SPI_CS))) {
        webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"input_only\"}"); return;
    }
    if (!p->hasADC && mode == PM_ADC) {
        webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"no_adc\"}"); return;
    }
    if (!p->hasDAC && mode == PM_DAC) {
        webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"no_dac\"}"); return;
    }
    if (!p->hasPWM && (mode == PM_PWM || mode == PM_CLOCK)) {
        webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"no_pwm\"}"); return;
    }
    if (isRestrictedGpio(gpio) && isDriveMode(mode) && !force) {
        webServer.send(403, "application/json",
            "{\"ok\":false,\"err\":\"restricted\",\"gpio\":" + String(gpio) + "}");
        return;
    }
    p->mode = mode;
    if (doc.containsKey("freq")) p->pwmFreq = (uint32_t)(long)doc["freq"];
    if (doc.containsKey("duty")) p->pwmDuty = (uint8_t)(int)doc["duty"];
    applyPinMode(p);
    saveModesToPrefs();
    webServer.send(200, "application/json",
        "{\"ok\":true,\"gpio\":" + String(gpio) + ",\"mode\":" + String(mode) + "}");
    pushPinsSSE();
}

void handleApiPinWrite() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(192);
    #endif
    if (deserializeJson(doc, webServer.arg("plain"))) { webServer.send(400); return; }
    int gpio = doc["gpio"] | -1;
    bool force = doc["force"] | false;
    PinInfo* p = findPin(gpio); if (!p) { webServer.send(404); return; }
    if (isRestrictedGpio(gpio) && isDriveMode(p->mode) && !force) {
        webServer.send(403, "application/json",
            "{\"ok\":false,\"err\":\"restricted\",\"gpio\":" + String(gpio) + "}");
        return;
    }
    int value = doc["value"] | 0;
    if (p->mode == PM_DAC && doc.containsKey("volts")) {
        value = dacRawFromVolts((float)doc["volts"]);
    }
    p->lastValue = value;
    switch (p->mode) {
        case PM_OUTPUT:
            digitalWrite(p->gpio, value ? HIGH : LOW);
            break;
        case PM_PWM:
            p->pwmDuty = (uint8_t)constrain(value, 0, 255);
            ledcWrite(p->gpio, p->pwmDuty);
            break;
        case PM_DAC:
            if (p->hasDAC) {
                p->lastValue = constrain(value, 0, 255);
                p->lastMv = (int)lroundf(dacVoltsFromRaw(p->lastValue) * 1000.0f);
                dacWrite(p->gpio, (uint8_t)p->lastValue);
            }
            break;
        default: break;
    }
    if (p->mode == PM_OUTPUT || p->mode == PM_DAC) saveModesToPrefs();
    webServer.send(200, "application/json",
        "{\"ok\":true,\"value\":" + String(p->lastValue) + ",\"mV\":" + String(p->lastMv) + "}");
    pushPinsSSE();
}

void handleApiPinRead() {
    int gpio = webServer.arg("gpio").toInt();
    PinInfo* p = findPin(gpio); if (!p) { webServer.send(404); return; }
    int val = p->lastValue;
    int mV = p->lastMv;
    if (p->mode == PM_INPUT || p->mode == PM_INPUT_PU) {
        val = digitalRead(p->gpio);
        mV = 0;
    } else if (p->mode == PM_ADC) {
        val = analogRead(p->gpio);
        mV = analogReadMilliVolts(p->gpio);
    } else if (p->mode == PM_DAC) {
        mV = (int)lroundf(dacVoltsFromRaw(p->lastValue) * 1000.0f);
    }
    p->lastValue = val;
    p->lastMv = mV;
    webServer.send(200, "application/json",
        "{\"gpio\":" + String(gpio) + ",\"value\":" + String(val)
        + ",\"mV\":" + String(mV) + "}");
}

void handleApiI2cInit() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(128);
    #endif
    deserializeJson(doc, webServer.arg("plain"));
    int sda = doc["sda"] | i2cSda, scl = doc["scl"] | i2cScl;
    initI2C(sda, scl);
    webServer.send(200, "application/json",
        "{\"ok\":true,\"sda\":" + String(sda) + ",\"scl\":" + String(scl) + "}");
}

void handleApiI2cScan() {
    if (!i2cReady) initI2C(i2cSda, i2cScl);
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
    initSPIBus(doc["mosi"]|spiMosi, doc["miso"]|spiMiso, doc["sck"]|spiSck,
               doc["cs"]|spiCs,   doc["freq"]|1000000);
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
    if (!spiReady) initSPIBus(spiMosi, spiMiso, spiSck, spiCs, spiFreq);
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
    String j = "{";
    j += "\"name\":\"" + deviceName + "\"";
    j += ",\"version\":\"" FW_VERSION "\"";
    j += ",\"chip\":\"" + hwInfo.chipModel() + "\"";
    j += ",\"mac\":\"" + getMac() + "\"";
    j += ",\"ip\":\"" + getLocalIp() + "\"";
    j += ",\"mdns\":\"" + toMdnsName(deviceName) + ".local\"";
    j += ",\"rssi\":" + String(WiFi.RSSI());
    j += ",\"uptime\":" + String(millis()/1000UL);
    j += ",\"heap\":" + String(hwInfo.freeHeap());
    j += ",\"freeHeap\":" + String(hwInfo.freeHeap());
    j += ",\"psram\":" + String(hwInfo.freePsram());
    j += ",\"psramSize\":" + String(hwInfo.psramSize());
    j += ",\"sketch\":" + String(hwInfo.freeSketchSpace());
    j += ",\"freeSketch\":" + String(hwInfo.freeSketchSpace());
    j += ",\"boardType\":\"" + String(BOARD_TYPE) + "\"";
    j += ",\"rgbLed\":"      + String(RGB_LED_GPIO);
    j += ",\"i2cSda\":"      + String(i2cSda);
    j += ",\"i2cScl\":"      + String(i2cScl);
    j += ",\"spiMosi\":"     + String(spiMosi);
    j += ",\"spiMiso\":"     + String(spiMiso);
    j += ",\"spiSck\":"      + String(spiSck);
    j += ",\"spiCs\":"       + String(spiCs);
    j += ",\"dacSupported\":"+ String(DAC_SUPPORTED ? "true" : "false");
    j += ",\"scopeMax\":"    + String(SCOPE_MAX_SAMPLES);
    j += ",\"i2cReady\":"    + String(i2cReady?"true":"false");
    j += ",\"spiReady\":"    + String(spiReady?"true":"false");
    hwInfo.appendStatusFields(j);
    CpuLoad::appendStatusFields(j);
    j += "}";
    webServer.send(200, "application/json", j);
}

void handleApiScope() {
    int gpio = webServer.hasArg("gpio") ? webServer.arg("gpio").toInt() : -1;
    int samples = webServer.hasArg("samples") ? webServer.arg("samples").toInt() : 200;
    int rate = webServer.hasArg("rate") ? webServer.arg("rate").toInt() : 5000;
    samples = constrain(samples, 10, SCOPE_MAX_SAMPLES);
    rate = constrain(rate, 100, 20000);
    PinInfo* p = findPin(gpio);
    if (!p || !p->hasADC) {
        webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"no_adc\"}");
        return;
    }
    analogSetPinAttenuation(gpio, ADC_11db);
    unsigned long periodUs = 1000000UL / (unsigned long)rate;
    unsigned long t0 = micros();
    int vmin = 4095, vmax = 0;
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        int v = analogRead(gpio);
        scopeBuf[i] = (uint16_t)v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
        sum += v;
        unsigned long next = t0 + (unsigned long)(i + 1) * periodUs;
        while ((long)(micros() - next) < 0) { /* spin wait */ }
    }
    String j = "{\"ok\":true,\"gpio\":" + String(gpio);
    j += ",\"samples\":" + String(samples);
    j += ",\"rate\":" + String(rate);
    j += ",\"min\":" + String(vmin);
    j += ",\"max\":" + String(vmax);
    j += ",\"avg\":" + String((int)(sum / samples));
    j += ",\"data\":[";
    for (int i = 0; i < samples; i++) {
        if (i) j += ",";
        j += String(scopeBuf[i]);
    }
    j += "]}";
    webServer.send(200, "application/json", j);
}

void handleApiCountReset() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(128);
    #endif
    if (deserializeJson(doc, webServer.arg("plain"))) { webServer.send(400); return; }
    int gpio = doc["gpio"] | -1;
    PinInfo* p = findPin(gpio);
    if (!p || p->mode != PM_COUNT) {
        webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"not_count\"}");
        return;
    }
    resetEdgeCounter(gpio);
    p->lastValue = 0;
    p->countHz = 0;
    webServer.send(200, "application/json", "{\"ok\":true,\"gpio\":" + String(gpio) + ",\"count\":0}");
    pushPinsSSE();
}

// Self-test: erzeugt N FALLING-Flanken am COUNT-Pin (ohne externe Verdrahtung)
void handleApiCountPulse() {
    if (!webServer.hasArg("plain")) { webServer.send(400); return; }
    #if ARDUINOJSON_VERSION_MAJOR >= 7
      JsonDocument doc;
    #else
      DynamicJsonDocument doc(128);
    #endif
    if (deserializeJson(doc, webServer.arg("plain"))) { webServer.send(400); return; }
    int gpio = doc["gpio"] | -1;
    int n = doc["n"] | 10;
    n = constrain(n, 1, 1000);
    PinInfo* p = findPin(gpio);
    if (!p || p->mode != PM_COUNT) {
        webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"not_count\"}");
        return;
    }
    if (isRestrictedGpio(gpio) && !(doc["force"] | false)) {
        webServer.send(403, "application/json", "{\"ok\":false,\"err\":\"restricted\"}");
        return;
    }
    uint32_t before = getEdgeCount(gpio);
    for (int i = 0; i < n; i++) {
        pinMode(gpio, OUTPUT);
        digitalWrite(gpio, LOW);
        delayMicroseconds(80);
        pinMode(gpio, INPUT_PULLUP);
        delayMicroseconds(80);
    }
    // Interrupt wieder sicher stellen
    detachInterrupt(digitalPinToInterrupt(gpio));
    for (int i = 0; i < COUNT_MAX; i++) {
        if (edgeCounters[i].gpio == gpio) {
            attachInterruptArg(digitalPinToInterrupt(gpio), edgeIsr, &edgeCounters[i], FALLING);
            break;
        }
    }
    delay(20);
    uint32_t after = getEdgeCount(gpio);
    p->lastValue = (int)after;
    webServer.send(200, "application/json",
        "{\"ok\":true,\"gpio\":" + String(gpio)
        + ",\"pulses\":" + String(n)
        + ",\"before\":" + String(before)
        + ",\"after\":" + String(after)
        + ",\"delta\":" + String(after - before) + "}");
    pushPinsSSE();
}

void handleApiDmm() {
    int gpio = webServer.hasArg("gpio") ? webServer.arg("gpio").toInt() : -1;
    String method = webServer.hasArg("method") ? webServer.arg("method") : "volt";
    method.toLowerCase();
    PinInfo* p = findPin(gpio);
    if (!p) {
        webServer.send(404, "application/json", "{\"ok\":false,\"err\":\"not_found\"}");
        return;
    }

    if (method == "volt" || method == "voltage") {
        if (!p->hasADC) {
            webServer.send(400, "application/json", "{\"ok\":false,\"err\":\"no_adc\"}");
            return;
        }
        analogSetPinAttenuation(gpio, ADC_11db);
        const int N = 24;
        int vmin = 4095, vmax = 0;
        long sumRaw = 0, sumMv = 0;
        int lastRaw = 0, lastMv = 0;
        for (int i = 0; i < N; i++) {
            lastRaw = analogRead(gpio);
            lastMv = analogReadMilliVolts(gpio);
            if (lastRaw < vmin) vmin = lastRaw;
            if (lastRaw > vmax) vmax = lastRaw;
            sumRaw += lastRaw;
            sumMv += lastMv;
            delayMicroseconds(200);
        }
        int avgRaw = (int)(sumRaw / N);
        int avgMv = (int)(sumMv / N);
        String j = "{\"ok\":true,\"method\":\"volt\",\"gpio\":" + String(gpio);
        j += ",\"raw\":" + String(lastRaw);
        j += ",\"mV\":" + String(lastMv);
        j += ",\"volts\":" + String(lastMv / 1000.0f, 4);
        j += ",\"avgRaw\":" + String(avgRaw);
        j += ",\"avgMv\":" + String(avgMv);
        j += ",\"avgVolts\":" + String(avgMv / 1000.0f, 4);
        j += ",\"minRaw\":" + String(vmin);
        j += ",\"maxRaw\":" + String(vmax);
        j += ",\"adc2\":" + String(isAdc2Gpio(gpio) ? "true" : "false");
        j += "}";
        webServer.send(200, "application/json", j);
        return;
    }

    if (method == "logic") {
        uint8_t modeSave = p->mode;
        pinMode(gpio, INPUT_PULLUP);
        delayMicroseconds(50);
        int level = digitalRead(gpio);
        p->mode = modeSave;
        applyPinMode(p);
        String j = "{\"ok\":true,\"method\":\"logic\",\"gpio\":" + String(gpio);
        j += ",\"level\":" + String(level);
        j += ",\"high\":" + String(level ? "true" : "false");
        j += ",\"label\":\"" + String(level ? "HIGH" : "LOW") + "\"}";
        webServer.send(200, "application/json", j);
        return;
    }

    if (method == "freq" || method == "period" || method == "duty") {
        uint8_t modeSave = p->mode;
        pinMode(gpio, INPUT);
        unsigned long timeoutU = 500000UL;
        unsigned long tHigh = pulseIn(gpio, HIGH, timeoutU);
        unsigned long tLow  = pulseIn(gpio, LOW,  timeoutU);
        p->mode = modeSave;
        applyPinMode(p);

        if (tHigh == 0 || tLow == 0) {
            webServer.send(200, "application/json",
                "{\"ok\":false,\"err\":\"no_signal\",\"method\":\"" + method +
                "\",\"gpio\":" + String(gpio) + "}");
            return;
        }
        unsigned long periodUs = tHigh + tLow;
        float freq = 1000000.0f / (float)periodUs;
        float duty = 100.0f * (float)tHigh / (float)periodUs;
        String j = "{\"ok\":true,\"method\":\"" + method + "\",\"gpio\":" + String(gpio);
        j += ",\"periodUs\":" + String((unsigned long)periodUs);
        j += ",\"highUs\":" + String((unsigned long)tHigh);
        j += ",\"lowUs\":" + String((unsigned long)tLow);
        j += ",\"freqHz\":" + String(freq, 3);
        j += ",\"dutyPct\":" + String(duty, 2);
        j += "}";
        webServer.send(200, "application/json", j);
        return;
    }

    if (method == "count") {
        if (p->mode == PM_COUNT) {
            readAllPins();
            String j = "{\"ok\":true,\"method\":\"count\",\"gpio\":" + String(gpio);
            j += ",\"count\":" + String(p->lastValue);
            j += ",\"freqHz\":" + String(p->countHz) + "}";
            webServer.send(200, "application/json", j);
            return;
        }
        uint8_t prev = p->mode;
        p->mode = PM_COUNT;
        applyPinMode(p);
        delay(1100);
        readAllPins();
        uint32_t cnt = getEdgeCount(gpio);
        uint32_t hz = getEdgeFreq(gpio);
        p->mode = prev;
        applyPinMode(p);
        String j = "{\"ok\":true,\"method\":\"count\",\"gpio\":" + String(gpio);
        j += ",\"count\":" + String(cnt);
        j += ",\"freqHz\":" + String(hz);
        j += ",\"windowMs\":1000}";
        webServer.send(200, "application/json", j);
        return;
    }

    webServer.send(400, "application/json",
        "{\"ok\":false,\"err\":\"method\",\"hint\":\"volt|logic|freq|period|duty|count\"}");
}

// ================================================================
//  WEB UI  — Raw String Literals
// ================================================================
void handleRoot() {
    webServer.sendHeader(F("Content-Encoding"), F("gzip"));
    webServer.sendHeader(F("Cache-Control"), F("no-store"));
    webServer.send_P(200, "text/html", (PGM_P)PAGE_MAIN_GZ, PAGE_MAIN_GZ_LEN);
}

void handleOtaPage() {
    webServer.sendHeader(F("Content-Encoding"), F("gzip"));
    webServer.sendHeader(F("Cache-Control"), F("no-store"));
    webServer.send_P(200, "text/html", (PGM_P)PAGE_OTA_GZ, PAGE_OTA_GZ_LEN);
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
    webServer.on("/api/scope",     HTTP_GET,  handleApiScope);
    webServer.on("/api/dmm",       HTTP_GET,  handleApiDmm);
    webServer.on("/api/count-reset", HTTP_POST, handleApiCountReset);
    webServer.on("/api/count-pulse", HTTP_POST, handleApiCountPulse);
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
      DynamicJsonDocument doc(4096);
    #endif
    doc["mac"]        = getMac();
    doc["name"]       = deviceName;
    doc["hwType"]     = BOARD_TYPE;
    doc["chipModel"]  = hwInfo.chipModel();
    doc["version"]    = FW_VERSION;
    doc["ip"]         = getLocalIp();
    doc["rssi"]       = WiFi.RSSI();
    doc["uptime"]     = millis() / 1000UL;
    doc["freeHeap"]   = hwInfo.freeHeap();
    doc["freeSketch"] = hwInfo.freeSketchSpace();
    doc["hwSummary"]  = hwInfo.summaryLine();
    doc["module"]     = hwInfo.moduleGuess();
    {
        const int c0 = CpuLoad::core0Percent();
        const int c1 = CpuLoad::core1Percent();
        doc["cpu"] = (c1 >= 0) ? ((c0 + c1) / 2) : c0;
    }
    JsonObject ios = doc["ios"].to<JsonObject>();
    for (int i = 0; i < PIN_COUNT; i++) {
        const char* t = hubIoType(pins[i].mode);
        if (!t) continue;
        String k = "gpio" + String(pins[i].gpio);
        JsonObject io = ios[k].to<JsonObject>();
        io["type"]  = t;
        if (pins[i].mode == PM_ADC) {
            io["value"] = (float)pins[i].lastMv / 1000.0f;
            io["unit"]  = "V";
            io["raw"]   = pins[i].lastValue;
        } else if (pins[i].mode == PM_DAC) {
            io["value"] = dacVoltsFromRaw(pins[i].lastValue);
            io["unit"]  = "V";
            io["raw"]   = pins[i].lastValue;
        } else if (pins[i].mode == PM_COUNT) {
            io["value"] = (float)pins[i].lastValue;
            io["freq"]  = pins[i].countHz;
            io["unit"]  = "count";
        } else {
            io["value"] = (float)pins[i].lastValue;
        }
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
                newName.trim();
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
    hwInfo.probe();
    hwInfo.logToSerial();
    CpuLoad::begin();
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
    CpuLoad::poll();
    tickSoftClocks();    // Software-Takt (< 50 Hz)
    tickEdgeCounters();  // COUNT Frequenz-Fenster
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
