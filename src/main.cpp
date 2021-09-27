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
#include <iomanip>
#include <sstream>

#include "privates.h"
#include "typedefs.h"
#include "OTA_updater.h"

#define SAMPLE_INTERVAL 1000  // 1000 milliseconds
#define ADC_ERROR_OFFSET 18   // Somehow there is a massive error

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
Ticker updater;
BME280 bm280;

// Global vars
float last_humidity = 0;
float last_pressure = 0;
float last_altitude = 0;
float last_temperature = 0;
float last_current = 0;

static void readBME(void);
static void writeHumidity(float);
static void writePressure(float);
static void writeAltitude(float);
static void writeTemperature(float);
static void writeCurrent(float);

void setup() {
    // Debug console
    //Serial.begin(115200);
    Blynk.begin(auth, ssid, pass, hostname, 8080);

    Wire.begin(0, 2);  //SDA, SCL
    bm280.beginI2C(Wire);

    Blynk.virtualWrite(V4, "clr");
    Blynk.virtualWrite(V4, "add", 0, "Pressure", " hPa");
    writePressure(bm280.readFloatPressure() / 100.0F);
    Blynk.virtualWrite(V4, "add", 1, "Altitude", " m");
    writeAltitude(bm280.readFloatAltitudeMeters());
    Blynk.virtualWrite(V4, "add", 2, "Humidity", " % RH");
    writeHumidity(bm280.readFloatHumidity());
    Blynk.virtualWrite(V4, "add", 3, "Temperature", " °C");
    writeTemperature(bm280.readTempC());
    Blynk.virtualWrite(V4, "add", 4, "Current", " A");
    writeCurrent(0.0);
}

BLYNK_CONNECTED() {
    bridge.setAuthToken(remoteAuth);
    INFO_PRINT("Just connected.\n");
    DEBUG_PRINT("Debug mode is on which is why I will spam here :-)\n\n");

    updater.attach_ms(SAMPLE_INTERVAL, readBME);
	// OTA Server update controller
    checkForUpdates();
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

static void writeCurrent(float value) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << value;
    Blynk.virtualWrite(V4, "update", 4, "Current", (stream.str() + " A").c_str());
    last_current = value;
}

static void readBME() {
    float humidity = bm280.readFloatHumidity();
    float pressure = bm280.readFloatPressure() / 100.0F;
    float altitude = bm280.readFloatAltitudeMeters();
    float temperature = bm280.readTempC();
    float current = analogRead(A0) - 512;
    current -= ADC_ERROR_OFFSET;
    current /= 13.5036;

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

    if ((int)(last_current * 10) != (int)(current * 10)) {
        writeCurrent(current);
        Blynk.virtualWrite(V4, "pick", 4);
    }
}
