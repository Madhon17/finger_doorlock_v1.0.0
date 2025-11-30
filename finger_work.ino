/* FULL SKETCH FINAL - Auto-find free ID (C)
 * AS608 (Adafruit_Fingerprint) + ESP32 + RainMaker integration
 * ENROLL: 2x scans (image2Tz(1), image2Tz(2), createModel, storeModel)
 * REMOVE: scan -> deleteModel(id) -> clear EEPROM
 *
 * Important:
 *  - Wiring: AS608 TX -> ESP RX (AS608_TX -> AS608_RX pin)
 *             AS608 RX -> ESP TX (AS608_RX -> AS608_TX pin)
 *  - VCC: 3.3V or 5V depending on your module (5V often recommended if board has regulator)
 *  - Serial monitor: 115200
 */

#include <RMaker.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiProv.h>
#include <EEPROM.h>
#include "Adafruit_Fingerprint.h"

// ---------- CONFIG PINS & DEFAULTS ----------
#define DEFAULT_RELAY_MODE true          // true => relay default OFF (RELAY_OFF)
#define RELAY_ON  LOW
#define RELAY_OFF HIGH

const char *service_name = "PROV_RFID_LOCK";
const char *pop = "1234567";

static const uint8_t PIN_RESET_BTN = 0;
static const uint8_t PIN_RELAY    = 33;
static const uint8_t PIN_SWITCH   = 32;
static const uint8_t PIN_BUZZER   = 15;
static const uint8_t PIN_ONBOARD_LED = 2;

// AS608 UART2 pins
static const int AS608_RX = 16;   // ESP32 RX <- AS608 TX
static const int AS608_TX = 17;   // ESP32 TX -> AS608 RX
static const int AS608_BAUD = 57600;

// EEPROM
#define EEPROM_SIZE 512
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MAGIC_VAL 0xA6
#define ID_MAX 127

// Globals
bool relay_state   = false;
bool buzzer_state  = false;
bool add_button    = false;
bool remove_button = false;
int  SWITCH_STATE  = HIGH;
bool buzz          = true; // buzzer on by default

// RainMaker device
static Device my_lock("RFID LOCK", "custom.device.device");

// Fingerprint
HardwareSerial FingerSerial(2);
Adafruit_Fingerprint finger(&FingerSerial);

// Forward
void sysProvEvent(arduino_event_t *sys_event);
void write_callback(Device *device, Param *param,
                    const param_val_t val, void *priv_data,
                    write_ctx_t *ctx);
void beep();
void success_buzzer();
void Failure_buzzer();
void add_switch_off();
void remove_switch_off();
void initEEPROM();
bool eepromIsInitialized();
int  findNextFreeID();
void markIDUsed(int id);
void markIDFree(int id);
String getValueFromFinger();
int  scanFingerID();
uint8_t enrollFingerAt(int id);
bool removeFingerByID(int id);
bool removeFingerByScan();
String authorized_access();
void authorized_access_offline();

// Implementation
void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {
        case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n",
                          service_name, pop);
            printQR(service_name, pop, "ble");
#else
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n",
                          service_name, pop);
            printQR(service_name, pop, "softap");
#endif
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            digitalWrite(PIN_ONBOARD_LED, HIGH);
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            digitalWrite(PIN_ONBOARD_LED, LOW);
            break;
        case ARDUINO_EVENT_PROV_CRED_RECV:
            Serial.println("Received WIFI credentials");
            break;
        case ARDUINO_EVENT_PROV_INIT:
            wifi_prov_mgr_disable_auto_stop(10000);
            break;
        case ARDUINO_EVENT_PROV_CRED_SUCCESS:
            wifi_prov_mgr_stop_provisioning();
            break;
    }
}

void write_callback(Device *device, Param *param,
                    const param_val_t val, void *priv_data,
                    write_ctx_t *ctx)
{
    const char *device_name = device->getDeviceName();
    const char *param_name  = param->getParamName();
    if (strcmp(device_name, "RFID LOCK") != 0) return;

    if (strcmp(param_name, "display") == 0) {
        Serial.printf("Display set: %s\n", val.val.s);
        param->updateAndReport(val);
    }

    if (strcmp(param_name, "BUZZER") == 0) {
        buzzer_state = val.val.b;
        buzz = buzzer_state;
        Serial.printf("Buzzer set to %s\n", buzz ? "ON" : "OFF");
        if (buzz) beep();
        param->updateAndReport(val);
    }

    if (strcmp(param_name, "DOOR OPEN") == 0) {
        relay_state = val.val.b;
        Serial.printf("DOOR OPEN set to %s\n", relay_state ? "OPEN" : "CLOSED");
        digitalWrite(PIN_RELAY, relay_state ? RELAY_ON : RELAY_OFF);
        param->updateAndReport(val);
    }

    if (strcmp(param_name, "ADD FINGER") == 0) {
        add_button = val.val.b;
        Serial.println("ADD FINGER triggered");
        if (add_button) {
            int id = findNextFreeID();
            if (id < 0) {
                Serial.println("No free ID available (EEPROM full)");
                Failure_buzzer();
            } else {
                uint8_t res = enrollFingerAt(id);
                if (res == FINGERPRINT_OK) {
                    markIDUsed(id);
                    Serial.printf("Enroll success ID=%d\n", id);
                    success_buzzer();
                } else {
                    Serial.printf("Enroll failed code=%d\n", res);
                    Failure_buzzer();
                }
            }
        } else {
            Failure_buzzer();
        }
        add_switch_off();
        delay(300);
    }

    if (strcmp(param_name, "REMOVE FINGER") == 0) {
        remove_button = val.val.b;
        Serial.println("REMOVE FINGER triggered");
        if (remove_button) {
            bool ok = removeFingerByScan();
            if (ok) {
                Serial.println("Remove operation succeeded");
                success_buzzer();
            } else {
                Serial.println("Remove operation failed or no matching finger");
                Failure_buzzer();
            }
        } else {
            Failure_buzzer();
        }
        remove_switch_off();
        delay(300);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    Serial.println("\n--- AS608 Fingerprint Lock (FINAL, auto-ID) ---");

    pinMode(PIN_ONBOARD_LED, OUTPUT);
    pinMode(PIN_RESET_BTN, INPUT);
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_SWITCH, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);

    digitalWrite(PIN_RELAY, DEFAULT_RELAY_MODE ? RELAY_OFF : RELAY_ON);

    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("EEPROM init failed!");
        delay(2000);
    }
    initEEPROM();

    // Init fingerprint serial (only this; do NOT call finger.begin() twice)
    FingerSerial.begin(AS608_BAUD, SERIAL_8N1, AS608_RX, AS608_TX);
    delay(200);

    Serial.println("Checking fingerprint sensor...");
    finger.begin(AS608_BAUD); // allowed once to setup library internals
    if (finger.verifyPassword()) {
        Serial.println("Fingerprint sensor detected!");
    } else {
        Serial.println("Fingerprint sensor NOT detected - check wiring & power!");
    }

    // RMaker
    Node my_node;
    my_node = RMaker.initNode("RFIDLOCK");
    my_lock.addNameParam();

    Param disp("display", "custom.param.display", value("Ready"), PROP_FLAG_READ);
    disp.addUIType(ESP_RMAKER_UI_TEXT);
    my_lock.addParam(disp);

    Param open_switch("DOOR OPEN", "custom.param.power", value(relay_state),
                      PROP_FLAG_READ | PROP_FLAG_WRITE);
    open_switch.addUIType(ESP_RMAKER_UI_TOGGLE);
    my_lock.addParam(open_switch);

    Param add_switch("ADD FINGER", "custom.param.power", value(add_button),
                     PROP_FLAG_READ | PROP_FLAG_WRITE);
    add_switch.addUIType(ESP_RMAKER_UI_TOGGLE);
    my_lock.addParam(add_switch);

    Param remove_switch("REMOVE FINGER", "custom.param.power", value(remove_button),
                        PROP_FLAG_READ | PROP_FLAG_WRITE);
    remove_switch.addUIType(ESP_RMAKER_UI_TOGGLE);
    my_lock.addParam(remove_switch);

    Param buzz_switch("BUZZER", "custom.param.power", value(buzzer_state),
                      PROP_FLAG_READ | PROP_FLAG_WRITE);
    buzz_switch.addUIType(ESP_RMAKER_UI_TOGGLE);
    my_lock.addParam(buzz_switch);

    my_lock.addCb(write_callback);
    my_node.addDevice(my_lock);

    my_lock.updateAndReportParam("DOOR OPEN", relay_state);
    my_lock.updateAndReportParam("ADD FINGER", add_button);
    my_lock.updateAndReportParam("REMOVE FINGER", remove_button);
    my_lock.updateAndReportParam("BUZZER", buzzer_state);

    RMaker.enableOTA(OTA_USING_PARAMS);
    RMaker.enableTZService();
    RMaker.enableSchedule();
    RMaker.start();

    WiFi.onEvent(sysProvEvent);
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE,
                            WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                            WIFI_PROV_SECURITY_1,
                            pop, service_name);

    // Sync EEPROM with sensor templates
    Serial.println("Syncing EEPROM with AS608 templates (this may take a while)...");
    for (int i = 1; i <= ID_MAX; i++) {
        uint8_t r = finger.loadModel(i);
        EEPROM.write(i, (r == FINGERPRINT_OK) ? 1 : 0);
    }
    EEPROM.commit();
    Serial.println("Sync completed.");

    Serial.println("Setup complete.");
}

void loop()
{
    // Manual switch open
    SWITCH_STATE = digitalRead(PIN_SWITCH);
    if (SWITCH_STATE == LOW) {
        Serial.println("Manual switch pressed -> offline authorized open");
        authorized_access_offline();
        delay(500);
    }

    // Poll for quick verification
    int id = scanFingerID();
    if (id > 0) {
        Serial.printf("Finger matched ID %d --> grant access\n", id);
        my_lock.updateAndReportParam("display", "Access Authorized");
        beep();
        digitalWrite(PIN_RELAY, RELAY_ON);
        delay(5000);
        digitalWrite(PIN_RELAY, RELAY_OFF);
        delay(500); // debounce between accesses
    }

    // Reset long press
    if (digitalRead(PIN_RESET_BTN) == LOW) {
        Serial.println("Reset button pressed; measuring duration...");
        delay(100);
        unsigned long start = millis();
        while (digitalRead(PIN_RESET_BTN) == LOW) delay(50);
        unsigned long held = millis() - start;
        if (held > 5000) {
            Serial.println("Long press detected: RMakerFactoryReset");
            RMakerFactoryReset(2);
        }
    }

    delay(120);
}

// EEPROM helpers
void initEEPROM()
{
    if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) {
        Serial.println("EEPROM magic not present; initializing ID map...");
        EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
        for (int i = 1; i <= ID_MAX; i++) EEPROM.write(i, 0);
        EEPROM.commit();
    } else {
        Serial.println("EEPROM magic OK");
    }
}

bool eepromIsInitialized()
{
    return (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VAL);
}

int findNextFreeID()
{
    if (!eepromIsInitialized()) initEEPROM();
    for (int i = 1; i <= ID_MAX; ++i) {
        if (EEPROM.read(i) == 0) return i;
    }
    return -1;
}

void markIDUsed(int id)
{
    if (id < 1 || id > ID_MAX) return;
    EEPROM.write(id, 1);
    EEPROM.commit();
    Serial.printf("Marked ID %d used in EEPROM\n", id);
}

void markIDFree(int id)
{
    if (id < 1 || id > ID_MAX) return;
    EEPROM.write(id, 0);
    EEPROM.commit();
    Serial.printf("Marked ID %d free in EEPROM\n", id);
}

// Finger functions
int scanFingerID()
{
    uint8_t p = finger.getImage();
    if (p == FINGERPRINT_NOFINGER) return -1;
    if (p != FINGERPRINT_OK) {
        Serial.printf("getImage error: %d\n", p);
        return -1;
    }
    p = finger.image2Tz();
    if (p != FINGERPRINT_OK) {
        Serial.printf("image2Tz error: %d\n", p);
        return -1;
    }
    p = finger.fingerFastSearch();
    if (p != FINGERPRINT_OK) {
        Serial.printf("fingerFastSearch error: %d\n", p);
        return -1;
    }
    Serial.printf("Found ID %d (confidence %d)\n", finger.fingerID, finger.confidence);
    return finger.fingerID;
}

uint8_t enrollFingerAt(int id)
{
    if (id < 1 || id > ID_MAX) return 0xEE;
    Serial.printf("Enroll: target ID=%d\n", id);
    uint8_t p;
    unsigned long start;

    // FIRST SCAN
    Serial.println("Place finger for 1st scan...");
    start = millis();
    while (true) {
        p = finger.getImage();
        if (p == FINGERPRINT_OK) break;
        if (p == FINGERPRINT_NOFINGER) {
            if (millis() - start > 20000) { // 20s
                Serial.println("Timeout 1st (no image captured)");
                return 0xE1;
            }
            delay(200);
            continue;
        } else {
            Serial.printf("getImage (1st) error: %d\n", p);
            return p;
        }
    }
    p = finger.image2Tz(1);
    if (p != FINGERPRINT_OK) {
        Serial.printf("image2Tz(1) failed: %d\n", p);
        return p;
    }

    Serial.println("Remove finger...");
    start = millis();
    while (millis() - start < 8000) {
        uint8_t n = finger.getImage();
        if (n == FINGERPRINT_NOFINGER) break;
        delay(150);
    }
    delay(300);

    // SECOND SCAN
    Serial.println("Place same finger for 2nd scan...");
    start = millis();
    while (true) {
        p = finger.getImage();
        if (p == FINGERPRINT_OK) break;
        if (p == FINGERPRINT_NOFINGER) {
            if (millis() - start > 20000) {
                Serial.println("Timeout 2nd (no image captured)");
                return 0xE2;
            }
            delay(200);
            continue;
        } else {
            Serial.printf("getImage (2nd) error: %d\n", p);
            return p;
        }
    }
    p = finger.image2Tz(2);
    if (p != FINGERPRINT_OK) {
        Serial.printf("image2Tz(2) failed: %d\n", p);
        return p;
    }

    p = finger.createModel();
    if (p != FINGERPRINT_OK) {
        Serial.printf("createModel failed: %d\n", p);
        return p;
    }

    p = finger.storeModel(id);
    if (p != FINGERPRINT_OK) {
        Serial.printf("storeModel failed: %d\n", p);
        return p;
    }

    Serial.printf("Stored fingerprint at ID %d\n", id);
    return FINGERPRINT_OK;
}

bool removeFingerByID(int id)
{
    if (id < 1 || id > ID_MAX) return false;
    uint8_t p = finger.deleteModel(id);
    if (p == FINGERPRINT_OK) {
        markIDFree(id);
        Serial.printf("Deleted template ID %d from sensor and EEPROM\n", id);
        return true;
    } else {
        Serial.printf("deleteModel failed code %d\n", p);
        return false;
    }
}

bool removeFingerByScan()
{
    Serial.println("Scan finger to remove (waiting up to 10s)...");
    int found = -1;
    unsigned long start = millis();
    while (millis() - start < 10000) {
        int id = scanFingerID();
        if (id > 0) {
            found = id;
            break;
        }
        delay(200);
    }
    if (found <= 0) {
        Serial.println("No matching finger detected for removal");
        return false;
    }
    Serial.printf("Finger matched ID %d -> deleting...\n", found);
    return removeFingerByID(found);
}

String authorized_access()
{
    Serial.println("Authorized access - opening relay");
    my_lock.updateAndReportParam("display", "Access Authorized");
    beep();
    digitalWrite(PIN_RELAY, RELAY_ON);
    delay(5000);
    digitalWrite(PIN_RELAY, RELAY_OFF);
    return "Access Authorized";
}

void authorized_access_offline()
{
    Serial.println("Offline authorized access triggered");
    digitalWrite(PIN_RELAY, RELAY_ON);
    delay(5000);
    digitalWrite(PIN_RELAY, RELAY_OFF);
}

void success_buzzer()
{
    if (buzz) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(600);
        digitalWrite(PIN_BUZZER, LOW);
    }
}

void Failure_buzzer()
{
    if (buzz) {
        for (int i = 0; i < 3; ++i) {
            digitalWrite(PIN_BUZZER, HIGH);
            delay(120);
            digitalWrite(PIN_BUZZER, LOW);
            delay(80);
        }
    }
}

void beep()
{
    if (buzz) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(80);
        digitalWrite(PIN_BUZZER, LOW);
    }
}

void add_switch_off()
{
    add_button = false;
    my_lock.updateAndReportParam("ADD FINGER", add_button);
    Serial.println("ADD FINGER flag cleared");
}

void remove_switch_off()
{
    remove_button = false;
    my_lock.updateAndReportParam("REMOVE FINGER", remove_button);
    Serial.println("REMOVE FINGER flag cleared");
}
