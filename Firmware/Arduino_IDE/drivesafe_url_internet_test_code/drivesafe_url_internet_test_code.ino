// ======================= LIBRARIES =======================
#include <Arduino.h>
#include "driver/twai.h"
#include <TinyGPS++.h>

// ======================= PIN CONFIG ======================
#define CAN_TX 5
#define CAN_RX 4

#define GSM_RX 26
#define GSM_TX 27

#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// ======================= OBJECTS =========================
HardwareSerial modem(1);      // GSM
HardwareSerial gpsSerial(2);  // GPS
TinyGPSPlus gps;

// ======================= NETWORK =========================
// Airtel Normal SIM : airtelgprs.com
// Airtel IoT SIM    : IOT.COM
#define APN "IOT.COM"

// ======================= SERVER ==========================
#define SERVER_URL "------paste your receiver url here -----"

// ======================= VEHICLE DATA ====================
int rpm = 0;
int speedKmh = 0;
int coolant = 0;
int fuel = 0;

double lat = 0.0;
double lon = 0.0;
bool gpsValid = false;

// ======================= UTILITIES =======================
void readModem() {
  while (modem.available()) {
    Serial.write(modem.read());
  }
}

void sendAT(const char *cmd, uint32_t waitMs = 2000) {
  Serial.print(">> ");
  Serial.println(cmd);
  modem.println(cmd);
  delay(waitMs);
  readModem();
  Serial.println();
}

// ======================= CAN FUNCTIONS ===================
void requestPID(uint8_t pid) {
  twai_message_t tx = {};
  tx.identifier = 0x7DF;
  tx.data_length_code = 8;
  tx.data[0] = 0x02;
  tx.data[1] = 0x01;
  tx.data[2] = pid;
  twai_transmit(&tx, pdMS_TO_TICKS(20));
}

void waitForPID(uint8_t pid) {
  twai_message_t rx;
  if (twai_receive(&rx, pdMS_TO_TICKS(300)) == ESP_OK) {
    if (rx.data[1] == 0x41 && rx.data[2] == pid) {
      if (pid == 0x0C) rpm = ((rx.data[3] << 8) | rx.data[4]) / 4;
      if (pid == 0x0D) speedKmh = rx.data[3];
      if (pid == 0x05) coolant = rx.data[3] - 40;
      if (pid == 0x2F) fuel = (rx.data[3] * 100) / 255;
    }
  }
}

// ======================= HTTP SEND =======================
void sendTelemetry() {

  String json = "{";
  json += "\"device_id\":\"Javid's Toy\",";
  json += "\"rpm\":" + String(rpm) + ",";
  json += "\"speed\":" + String(speedKmh) + ",";
  json += "\"coolant\":" + String(coolant) + ",";
  json += "\"fuel\":" + String(fuel);

  if (gpsValid) {
    json += ",\"latitude\":" + String(lat, 6);
    json += ",\"longitude\":" + String(lon, 6);
  }
  json += "}";

  Serial.println("📤 Sending JSON:");
  Serial.println(json);

  // --- HTTP sequence (A7670C safe) ---
  sendAT("AT+HTTPTERM", 1000);
  sendAT("AT+HTTPINIT", 2000);
  sendAT("AT+HTTPPARA=\"CID\",1", 1000);
  sendAT("AT+HTTPPARA=\"URL\",\"" SERVER_URL "\"", 2000);
  sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000);

  modem.print("AT+HTTPDATA=");
  modem.print(json.length());
  modem.println(",5000");
  delay(2000);          // wait for DOWNLOAD
  modem.print(json);
  delay(6000);

  modem.println("AT+HTTPACTION=1"); // POST

  unsigned long t = millis();
  while (millis() - t < 10000) {
    if (modem.available()) {
      String r = modem.readString();
      Serial.println(r);
      if (r.indexOf("+HTTPACTION:") >= 0) break;
    }
  }

  sendAT("AT+HTTPREAD", 3000);
  Serial.println("✅ HTTP POST DONE\n");
}

// ======================= SETUP ===========================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("🚗 ESP32 CAN + GPS + GSM (A7670C)");

  // ---------- CAN INIT ----------
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX,
      (gpio_num_t)CAN_RX,
      TWAI_MODE_NORMAL
    );

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
  Serial.println("✅ CAN started");

  // ---------- GPS INIT ----------
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("✅ GPS started");

  // ---------- GSM INIT ----------
  modem.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(5000);

  sendAT("AT");
  sendAT("ATE0");
  sendAT("AT+CFUN=1");
  sendAT("AT+CPIN?");
  sendAT("AT+CSQ");
  sendAT("AT+CREG?");

  // ---------- INTERNET INIT ----------
  sendAT("AT+CGATT=1");
  sendAT("AT+CGDCONT=1,\"IP\",\"" APN "\"");
  sendAT("AT+CGACT=1,1");

  // close if already opened
  sendAT("AT+NETCLOSE", 2000);
  sendAT("AT+NETOPEN", 3000);

  Serial.println("✅ GSM Internet Ready");
}

// ======================= LOOP ============================
void loop() {

  // ---------- GPS ----------
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    gpsValid = true;
  }

  // ---------- CAN ----------
  requestPID(0x0C); waitForPID(0x0C);   // RPM
  requestPID(0x0D); waitForPID(0x0D);   // Speed
  requestPID(0x05); waitForPID(0x05);   // Coolant
  requestPID(0x2F); waitForPID(0x2F);   // Fuel

  // ---------- SEND ----------
  sendTelemetry();

  delay(15000);
}
