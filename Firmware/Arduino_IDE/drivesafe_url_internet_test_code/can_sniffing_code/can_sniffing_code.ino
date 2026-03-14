#include <Arduino.h>
#include "driver/twai.h"
#include <TinyGPS++.h>

/* ================= PIN DEFINITIONS ================= */

#define CAN_TX 5
#define CAN_RX 4

#define GSM_RX 27
#define GSM_TX 26

#define GPS_RX 16
#define GPS_TX 17

/* ================= SERIAL OBJECTS ================= */

HardwareSerial gsmSerial(1);
HardwareSerial gpsSerial(2);

TinyGPSPlus gps;

/* ================= GLOBAL VARIABLES ================= */

unsigned long lastPIDrequest = 0;
unsigned long lastPrint = 0;

bool canStarted = false;

/* ================= CAN MESSAGE STRUCT ================= */

twai_message_t rxMsg;

/* ================= SLCAN OUTPUT FUNCTION ================= */

void sendFrameToSavvyCAN(twai_message_t &msg)
{
  char frame[40];

  if (msg.extd)
  {
    sprintf(frame, "T%08X%d", msg.identifier, msg.data_length_code);
  }
  else
  {
    sprintf(frame, "t%03X%d", msg.identifier, msg.data_length_code);
  }

  Serial.print(frame);

  for (int i = 0; i < msg.data_length_code; i++)
  {
    if (msg.data[i] < 16)
      Serial.print("0");

    Serial.print(msg.data[i], HEX);
  }

  Serial.println();
}

/* ================= CAN INITIALIZATION ================= */

void initCAN()
{
  Serial.println("Initializing CAN...");

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX,
                                  (gpio_num_t)CAN_RX,
                                  TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    if (twai_start() == ESP_OK)
    {
      Serial.println("CAN Started");
      canStarted = true;
    }
    else
    {
      Serial.println("CAN Start Failed");
    }
  }
  else
  {
    Serial.println("CAN Driver Install Failed");
  }
}

/* ================= OBD PID REQUEST ================= */

void requestPID(uint8_t pid)
{
  twai_message_t txMsg = {};

  txMsg.identifier = 0x7DF;
  txMsg.data_length_code = 8;

  txMsg.data[0] = 0x02;
  txMsg.data[1] = 0x01;
  txMsg.data[2] = pid;

  for (int i = 3; i < 8; i++)
    txMsg.data[i] = 0;

  if (twai_transmit(&txMsg, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    Serial.print("PID Request Sent: ");
    Serial.println(pid, HEX);
  }
  else
  {
    Serial.println("PID TX Failed");
  }
}

/* ================= READ CAN RESPONSE ================= */

void readCAN()
{
  while (twai_receive(&rxMsg, 0) == ESP_OK)
  {
    sendFrameToSavvyCAN(rxMsg);

    if (rxMsg.identifier >= 0x7E8 && rxMsg.identifier <= 0x7EF)
    {
      uint8_t pid = rxMsg.data[2];

      if (pid == 0x0C)
      {
        int rpm = ((rxMsg.data[3] << 8) | rxMsg.data[4]) / 4;

        Serial.print("Engine RPM = ");
        Serial.println(rpm);
      }

      if (pid == 0x0D)
      {
        int speed = rxMsg.data[3];

        Serial.print("Vehicle Speed = ");
        Serial.println(speed);
      }
    }
  }
}

/* ================= GPS TASK ================= */

void processGPS()
{
  while (gpsSerial.available())
  {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated())
  {
    Serial.print("LAT:");
    Serial.print(gps.location.lat(), 6);

    Serial.print(" LON:");
    Serial.println(gps.location.lng(), 6);
  }
}

/* ================= GSM TEST ================= */

void gsmTest()
{
  if (gsmSerial.available())
  {
    Serial.write(gsmSerial.read());
  }
}

/* ================= SETUP ================= */

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("ESP32 CAN + GPS + GSM Debug System");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("GPS Initialized");

  gsmSerial.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);

  Serial.println("GSM UART Ready");

  initCAN();

  Serial.println("System Ready");
}

/* ================= MAIN LOOP ================= */

void loop()
{
  processGPS();

  gsmTest();

  if (canStarted)
  {
    readCAN();
  }

  if (millis() - lastPIDrequest > 2000)
  {
    lastPIDrequest = millis();

    requestPID(0x0C); // RPM
    requestPID(0x0D); // Speed
  }
}