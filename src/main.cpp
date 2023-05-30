/**
 * @file       main.cpp
 * @author     Fa-b
 * @license    This project is released under the MIT License (MIT)
 * @date       Sep 2021
 * @brief      Entry, not much done here yet
 */

#include <Arduino.h>
#include <BlynkSimpleEsp8266.h>
#include <SparkFunBME280.h>
#include <Ticker.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusMaster.h>
#include <iomanip>
#include <sstream>

#include "privates.h"
#include "typedefs.h"
#include "OTA_updater.h"
#include "command_parser.h"

#define SAMPLE_INTERVAL 1000  // 1000 milliseconds
#define ADC_ERROR_OFFSET 18   // Somehow there is a massive offset error, this has to be manually determined once per ESP8266
#define ACS712_VOLTS_PER_AMPERE ((double) 0.066) // 66mV for -30A, 100mV for -20A, 185mV for -5A

#define SDA_PIN                     0
#define SCL_PIN                     2
#define DALLAS_WIRE_PIN             14
#define TRACER_WREN_PIN             4
#define ADS1000_ADDR                0x48 // for type BD0, BD1 addr is 0x49

#define vPIN_PV_POWER                   V11
#define vPIN_PV_CURRENT                 V12
#define vPIN_PV_VOLTAGE                 V13
#define vPIN_LOAD_CURRENT               V14
#define vPIN_LOAD_POWER                 V15
#define vPIN_BATT_TEMP                  V16
#define vPIN_BATT_VOLTAGE               V17
#define vPIN_BATT_REMAIN                V18
#define vPIN_CONTROLLER_TEMP            V19
#define vPIN_BATTERY_CHARGE_CURRENT     V20
#define vPIN_BATTERY_CHARGE_POWER       V21
#define vPIN_BATTERY_OVERALL_CURRENT    V22
#define vPIN_LOAD_ENABLED               V24

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

// extern in privates.h
DEVICE_INFO_T device_info;
const char name[] = DEVICE_NAME;
const char auth[] = AUTH_TOKEN;
const char version[] = VERSION;
char hostname[] = HOSTNAME;
char ssid[] = SSID;
char pass[] = WIFI_PASS;
char remoteAuth[] = REMOTE_AUTH;

// extern in typedefs.h
#ifdef DEBUG_TERMINAL
WidgetTerminal terminal(DEBUG_TERMINAL);
#elif DEBUG_BRIDGE
BridgeTerminal bridge(DEBUG_BRIDGE);
#endif

// Modifying lib to avoid linker errors (multiple file projects)
static WiFiClient _blynkWifiClient;
static BlynkArduinoClient _blynkTransport(_blynkWifiClient);
BlynkWifi Blynk(_blynkTransport);

// lib instancces
OneWire oneWire(DALLAS_WIRE_PIN);
Ticker updater;
BME280 bm280;
DallasTemperature ds18b20(&oneWire);
ModbusMaster node;

// Global vars
float last_humidity = 0;
float last_pressure = 0;
float last_altitude = 0;
float last_temperature = 0;
float last_ambient = 0;
float last_voltage = 0;
float last_current = 0;

unsigned int regNum = 0;
float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
unsigned int result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;

static void writeHumidity(float);
static void writePressure(float);
static void writeAltitude(float);
static void writeTemperature(float);
static void writeAmbient(float);
static void writeVoltage(float);
static void writeCurrent(float);


static void resetADS1000(uint8_t addr);
static int16_t readADS1000(uint8_t addr);

static void periodicRead(void);

static void AddressRegistry_3100(void);
static void AddressRegistry_3106(void);
static void AddressRegistry_310D(void);
static void AddressRegistry_311A(void);
static void AddressRegistry_331B(void);

// A list of the regisities to query in order
typedef void (*RegistryList[])();

RegistryList Registries = {
  AddressRegistry_3100,
  AddressRegistry_3106,
  AddressRegistry_310D,
  AddressRegistry_311A,
  AddressRegistry_331B,
};

void setup() {

    // Problematic since there may be multiple devices with the same name
    // Add serial number to hostname
    WiFi.hostname(String(name) + String("_") + String(ESP.getChipId()));

    Blynk.begin(auth, ssid, pass, hostname, 8080);
    Wire.begin(SDA_PIN, SCL_PIN);
    bm280.beginI2C(Wire);
    ds18b20.begin();
    ds18b20.setResolution(11);
    // Don't wait for conversion results (watchdog)
    ds18b20.setWaitForConversion(false);

    resetADS1000(ADS1000_ADDR);
    
    pinMode(TRACER_WREN_PIN, OUTPUT);
    digitalWrite(TRACER_WREN_PIN, 0);
    Serial.begin(115200);
    // Modbus slave ID 1
    node.begin(1, Serial);

    node.preTransmission([]() {
        digitalWrite(TRACER_WREN_PIN, 1);
    });

    node.postTransmission([]() {
        digitalWrite(TRACER_WREN_PIN, 0);
    });

    node.idle([]() {
        ESP.wdtFeed();
    });
}

BLYNK_CONNECTED() {
#ifdef DEBUG_BRIDGE
    bridge.setAuthToken(remoteAuth);
#endif
    INFO_PRINT("Just connected.\n");
    DEBUG_PRINT("Debug mode is on which is why I will spam here :-)\n\n");

	// OTA Server update controller
    checkForUpdates();

    // Don't do this when updates are found
    updater.once_ms(SAMPLE_INTERVAL, periodicRead);

}

void loop() {
    Blynk.run();
}

static void writePressure(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V0, value);
    last_pressure = value;
}

static void writeHumidity(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V1, value);
    last_humidity = value;
}

static void writeTemperature(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V2, value);
    last_temperature = value;
}

static void writeAmbient(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V3, value);
    last_ambient = value;
}

static void writeCurrent(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V4, value);
    last_current = value;
}

static void resetADS1000(uint8_t addr) {
    Wire.beginTransmission(addr);
    // Default setting
    // ST/BSY 0 0 SC 0 0 PGA1 PGA0
    Wire.write(0x80);
    Wire.endTransmission(addr);
}

static int16_t readADS1000(uint8_t addr) {
    union adc_data {
        uint8_t arr[2];
        int16_t val;
    } data;

    uint8_t len = Wire.requestFrom(addr, 2);
    if (len >= 2) {
        data.arr[1] = Wire.read();
        data.arr[0] = Wire.read();
    }

    return data.val;
}

static void periodicRead() {
    float pressure = bm280.readFloatPressure() / 100.0F;
    float humidity = bm280.readFloatHumidity();
    float temperature = bm280.readTempC();
    float altitude = bm280.readFloatAltitudeMeters();
    ds18b20.requestTemperaturesByIndex(0);
    float ambient = ds18b20.getTempCByIndex(0);
    float volt_val = analogRead(A0) - ADC_ERROR_OFFSET;
    volt_val /= 100;
    float cur_val = readADS1000(ADS1000_ADDR);
    float current = cur_val * (volt_val / (2048 * ACS712_VOLTS_PER_AMPERE));

    DEBUG_PRINTF("Voltage: %f\nCurrent: %f\n", volt_val, cur_val);

    if ((int)(last_pressure * 100) != (int)(pressure * 100)) {
        writePressure(pressure);
    }

    if ((int)last_humidity != (int)humidity) {
        writeHumidity(humidity);
    }

    if ((int)(last_temperature * 10) != (int)(temperature * 10)) {
        writeTemperature(temperature);
    }

    if ((int)(last_ambient * 10) != (int)(ambient * 10)) {
        writeAmbient(ambient);
    }

    if ((int)(last_current * 100) != (int)(current * 100)) {
        writeCurrent(current);
    }

    //ESP.wdtDisable();
    Registries[regNum]();
    //ESP.wdtEnable(1000);

    if (++regNum >= ARRAY_SIZE(Registries)) {
        regNum = 0;
    }
    
    updater.once_ms(SAMPLE_INTERVAL, periodicRead);
}

static void AddressRegistry_3100() {
    result = node.readInputRegisters(0x3100, 6);

    if (result == node.ku8MBSuccess) {        
        pvvoltage = node.getResponseBuffer(0x00) / 100.0f;
        pvcurrent = node.getResponseBuffer(0x01) / 100.0f;
        pvpower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;        
        bvoltage = node.getResponseBuffer(0x04) / 100.0f;
        battChargeCurrent = node.getResponseBuffer(0x05) / 100.0f;

        Blynk.virtualWrite(vPIN_PV_VOLTAGE, pvvoltage);
        Blynk.virtualWrite(vPIN_PV_CURRENT, pvcurrent);
        Blynk.virtualWrite(vPIN_PV_POWER, pvpower);
        Blynk.virtualWrite(vPIN_BATT_VOLTAGE, bvoltage);
        Blynk.virtualWrite(vPIN_BATTERY_CHARGE_CURRENT, battChargeCurrent);
    } else {
        rs485DataReceived = false;
    }
}

void AddressRegistry_3106(){
    result = node.readInputRegisters(0x3106, 2);

    if (result == node.ku8MBSuccess) {
        battChargePower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16)  / 100.0f;

        Blynk.virtualWrite(vPIN_BATTERY_CHARGE_POWER, battChargePower);
    } else {
        rs485DataReceived = false;
    }    
}

void AddressRegistry_310D() {
    result = node.readInputRegisters(0x310D, 3);

    if (result == node.ku8MBSuccess) {
        lcurrent = node.getResponseBuffer(0x00) / 100.0f;
        lpower = (node.getResponseBuffer(0x01) | node.getResponseBuffer(0x02) << 16) / 100.0f;
        
        Blynk.virtualWrite(vPIN_LOAD_CURRENT, lcurrent);
        Blynk.virtualWrite(vPIN_LOAD_POWER, lpower);
    } else {
        rs485DataReceived = false;
    }    
} 

void AddressRegistry_311A() {
    result = node.readInputRegisters(0x311A, 2);

    if (result == node.ku8MBSuccess) {    
        bremaining = node.getResponseBuffer(0x00) / 1.0f;        
        btemp = node.getResponseBuffer(0x01) / 100.0f;

        Blynk.virtualWrite(vPIN_BATT_TEMP, btemp);
        Blynk.virtualWrite(vPIN_BATT_REMAIN, bremaining);
    } else {
        rs485DataReceived = false;
    }
}

void AddressRegistry_331B() {
    result = node.readInputRegisters(0x331B, 2);

    if (result == node.ku8MBSuccess) {
        battOverallCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
        
        Blynk.virtualWrite(vPIN_BATTERY_OVERALL_CURRENT, battOverallCurrent);
    } else {
        rs485DataReceived = false;
    }
}

