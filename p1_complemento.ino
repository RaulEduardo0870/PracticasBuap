#include <DHT_U.h>

#include <DHTesp.h>
#include <Wire.h>
#include <Adafruit_BMP085_U.h>
#include "DHT.h"
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// Configuración de LoRa
const String lora_band = "915000000";      // Banda de frecuencia
const String lora_networkid = "18";        // Identificación de la red LoRa
const String lora_address = "1";           // Dirección del módulo LoRa
const String lora_RX_address = "2";        // Dirección del módulo receptor LoRa
const String lora_Parameters = "9,7,1,12"; // Parámetros de configuración de LoRa

const int DHT_PIN = 15;
const int SOLENOID_PIN = 2;
const int LoRa_DELAY = 500;
const int GPS_READ_INTERVAL = 800;
const float PRESION_NIVEL_MAR = 1013.25;  // Presión al nivel del mar en mbar

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified();
DHT dht(DHT_PIN, DHT11);
MPU6050 mpu;
SoftwareSerial ss(34, 35); // RX, TX para el GPS
TinyGPS gps;

float altitudInicial;
float altitud_previa = 0.0;
unsigned long tiempo_previo = 0;
int activar = 0;

void LoRaConfig() {
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // Configura los pines RX y TX para LoRa
  delay(1500);
  Serial2.println("AT+RESET");
  delay(LoRa_DELAY);
  Serial2.println("AT+BAND=" + lora_band);
  delay(LoRa_DELAY);
  Serial2.println("AT+ADDRESS=" + lora_address);
  delay(LoRa_DELAY);
  Serial2.println("AT+NETWORKID=" + lora_networkid);
  delay(LoRa_DELAY);
  Serial2.println("AT+PARAMETER=" + lora_Parameters);
  delay(1500);
}

void LoRaSend(String message) {
  Serial2.println("AT+SEND=" + lora_RX_address + "," + String(message.length()) + "," + message);
  delay(100);
}

void setup() {
    Serial.begin(115200);
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW);

    // Inicializar sensores
    dht.begin();
    
    if (!bmp.begin()) {
        Serial.println("Couldn't find the BMP180 sensor");
        while (1);
    }
    
    mpu.initialize();
    
    // Configurar GPS
    ss.begin(9600);
    
    // Inicializar LoRa
    LoRaConfig();

    // Obtener altitud inicial
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
        altitudInicial = bmp.pressureToAltitude(1013.25, event.pressure) + 120;
    }
    
    tiempo_previo = millis();
}

void loop() {
    // Leer GPS
    bool newData = false;
    for (unsigned long start = millis(); millis() - start < GPS_READ_INTERVAL;) {
        while (ss.available()) {
            char c = ss.read();
            if (gps.encode(c))
                newData = true;
        }
    }

    float latitud = 0.0, longitud = 0.0, altitud = 0.0;
    if (newData) {
        gps.f_get_position(&latitud, &longitud);
        altitud = gps.altitude();
    }

    // Leer sensores
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

        // Leer DHT
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();

        // Leer MPU6050
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getAcceleration(&ax, &ay, &az);
        mpu.getRotation(&gx, &gy, &gz);
        float accelerationX = (float)ax / 16384.0 * 9.81;
        float accelerationY = (float)ay / 16384.0 * 9.81;
        float accelerationZ = (float)az / 16384.0 * 9.81;
        float gyroX = (float)gx / 131.0;
        float gyroY = (float)gy / 131.0;
        float gyroZ = (float)gz / 131.0;

        // Construir mensaje
        String message = "|";
        message += String(event.pressure) + ":"; // Presión
        message += String(altitud_actual) + ":"; // Altitud
        message += String(velocidad_vertical) + ":"; // Velocidad vertical
        message += String(temperature) + ":"; // Temperatura DHT
        message += String(humidity) + ":"; // Humedad DHT
        message += String(latitud, 6) + ":"; // Latitud GPS
        message += String(longitud, 6) + ":"; // Longitud GPS
        message += String(gyroX) + ":" + String(gyroY) + ":" + String(gyroZ) + ":"; // Datos del giroscopio
        message += String(accelerationX) + ":" + String(accelerationY) + ":" + String(accelerationZ); // Datos del acelerómetro
        message += ":" + String(velocidad_vertical) + "|"; // Velocidad vertical

        // Control del solenoide
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

        // Enviar mensaje por LoRa
        Serial.println(message);
        LoRaSend(message);
    }

    delay(1000); // Esperar un segundo antes del siguiente ciclo
}
