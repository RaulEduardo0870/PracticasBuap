//#include <DHT.h>
#include <DHT_U.h>

#include <DHTesp.h>

#include <Wire.h>
#include <Adafruit_BMP085_U.h>
#include "DHT.h"
#include <MPU6050.h>

// Define pins
#define DHT_PIN 15
#define SOLENOID_PIN 2

// Define DHT sensor type
#define DHT_TYPE DHT11

// Create sensor objects
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified();
DHT dht(DHT_PIN, DHT_TYPE);
MPU6050 mpu;

// Variables for altitude calculation
float altitudInicial;
float altitud_previa = 0.0;
unsigned long tiempo_previo = 0;
int activar = 0;

void setup() {
    Serial.begin(115200);
    
    // Initialize DHT sensor
    dht.begin();
    
    // Initialize BMP180 sensor
    if (!bmp.begin()) {
        Serial.println("Couldn't find the BMP180 sensor");
        while (1);
    }
    
    // Initialize MPU6050 sensor
    mpu.initialize();
    
    // Set up GPIO
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW);
    
    // Get initial altitude
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
        altitudInicial = bmp.pressureToAltitude(1013.25, event.pressure) + 120;
    }
    
    tiempo_previo = millis();
}

void loop() {
    // Read BMP180 sensor
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
        float altitud_actual = bmp.pressureToAltitude(1013.25, event.pressure) + 120;
        float delta_altitud = altitud_actual - altitud_previa;
        unsigned long tiempo_actual = millis();
        float delta_tiempo = (tiempo_actual - tiempo_previo) / 1000.0;
        float velocidad_vertical = delta_altitud / delta_tiempo;

        altitud_previa = altitud_actual;
        tiempo_previo = tiempo_actual;

        // Create message to send
        char message[256];
        snprintf(message, sizeof(message), "|%.2f:%.2f:%.2f|", event.pressure, altitud_actual, velocidad_vertical);

        // Activate solenoid if conditions are met
        if (altitud_actual == 300) {
            activar = 1;
        }

        if (activar == 1 && altitud_actual == 200) {
            for (int i = 0; i <= 10; i++) {
                digitalWrite(SOLENOID_PIN, HIGH);
                delay(300);
                digitalWrite(SOLENOID_PIN, LOW);
                delay(300);
            }
        }

        Serial.println("Sending message: " + String(message));
        // Implement LoRa send function here

        delay(1000); // Wait a second before the next loop
    }
}
