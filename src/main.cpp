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

#define BME_SDA_PIN                     0
#define BME_SCL_PIN                     2
#define DALLAS_WIRE_PIN                 14
#define TRACER_WREN_PIN                 4

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
#ifdef ERROR_TERMINAL
WidgetTerminal terminal(DEBUG_TERMINAL);
#elif ERROR_BRIDGE
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
    Blynk.begin(auth, ssid, pass, hostname, 8080);

    Wire.begin(0, 2);  //SDA, SCL
    bm280.beginI2C(Wire);
    ds18b20.begin();
    // Don't wait for conversion results (watchdog)
    ds18b20.setWaitForConversion(false);
    
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

    Blynk.virtualWrite(V4, "clr");
    Blynk.virtualWrite(V4, "add", 0, "Pressure", " hPa");
    writePressure(bm280.readFloatPressure() / 100.0F);
    Blynk.virtualWrite(V4, "add", 1, "Altitude", " m");
    writeAltitude(bm280.readFloatAltitudeMeters());
    Blynk.virtualWrite(V4, "add", 2, "Humidity", " % RH");
    writeHumidity(bm280.readFloatHumidity());
    Blynk.virtualWrite(V4, "add", 3, "Temperature", " °C");
    writeTemperature(bm280.readTempC());
    //ds18b20.requestTemperatures();
    Blynk.virtualWrite(V4, "add", 4, "Ambient", " °C");
    writeAmbient(ds18b20.getTempCByIndex(0));
    Blynk.virtualWrite(V4, "add", 5, "Voltage", " V");
    writeVoltage(0.00);
}

BLYNK_CONNECTED() {
    bridge.setAuthToken(remoteAuth);
    INFO_PRINT("Just connected.\n");
    DEBUG_PRINT("Debug mode is on which is why I will spam here :-)\n\n");

	// OTA Server update controller
    checkForUpdates();
    // Don't do this when updates are found
    updater.attach_ms(SAMPLE_INTERVAL, periodicRead);
}

void loop() {
    Blynk.run();
}

static void writePressure(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V1, value);
    stream << std::fixed << std::setprecision(2) << value;
    Blynk.virtualWrite(V4, "update", 0, "Pressure", (stream.str() + " hPa").c_str());
    last_pressure = value;
}

static void writeAltitude(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V2, value);
    stream << std::fixed << (int)value;
    Blynk.virtualWrite(V4, "update", 1, "Altitude", (stream.str() + " m").c_str());
    last_altitude = value;
}

static void writeHumidity(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V0, value);
    stream << std::fixed << (int)value;
    Blynk.virtualWrite(V4, "update", 2, "Humidity", (stream.str() + " % RH").c_str());
    last_humidity = value;
}

static void writeTemperature(float value) {
    std::stringstream stream;
    Blynk.virtualWrite(V3, value);
    stream << std::fixed << std::setprecision(1) << value;
    Blynk.virtualWrite(V4, "update", 3, "Temperature", (stream.str() + " °C").c_str());
    last_temperature = value;
}

static void writeAmbient(float value) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << value;
    Blynk.virtualWrite(V4, "update", 4, "Ambient", (stream.str() + " °C").c_str());
    last_ambient = value;
}

static void writeVoltage(float value) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    Blynk.virtualWrite(V4, "update", 5, "Voltage", (stream.str() + " V").c_str());
    last_voltage = value;
}

static void periodicRead() {
    float humidity = bm280.readFloatHumidity();
    float pressure = bm280.readFloatPressure() / 100.0F;
    float altitude = bm280.readFloatAltitudeMeters();
    float temperature = bm280.readTempC();
    ds18b20.requestTemperaturesByIndex(0);
    float ambient = ds18b20.getTempCByIndex(0);
    float voltage = analogRead(A0);
    voltage /= 100;

    if ((int)(last_pressure * 100) != (int)(pressure * 100)) {
        writePressure(pressure);
        Blynk.virtualWrite(V4, "pick", 0);
    }

    if ((int)last_altitude != (int)altitude) {
        writeAltitude(altitude);
        Blynk.virtualWrite(V4, "pick", 1);
    }

    if ((int)last_humidity != (int)humidity) {
        writeHumidity(humidity);
        Blynk.virtualWrite(V4, "pick", 2);
    }

    if ((int)(last_temperature * 10) != (int)(temperature * 10)) {
        writeTemperature(temperature);
        Blynk.virtualWrite(V4, "pick", 3);
    }

    if ((int)(last_ambient * 10) != (int)(ambient * 10)) {
        writeAmbient(ambient);
        Blynk.virtualWrite(V4, "pick", 4);
    }

    if ((int)(last_voltage * 100) != (int)(voltage * 100)) {
        writeVoltage(voltage);
        Blynk.virtualWrite(V4, "pick", 5);
    }

    //ESP.wdtDisable();
    Registries[regNum]();
    //ESP.wdtEnable(1000);

    if (++regNum >= ARRAY_SIZE(Registries)) {
        regNum = 0;
    }
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

