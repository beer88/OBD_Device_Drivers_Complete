#include <Arduino.h>
#include "driver/twai.h"

/* ================= CAN PINS ================= */
#define CAN_TX 5
#define CAN_RX 4

/* ================= OBD CONSTANTS ================= */
#define OBD_REQ_ID     0x7DF

/* ================= SUPPORTED PIDs ================= */
uint8_t supportedPIDs[] = {
  0x01,0x04,0x05,0x0B,0x0C,0x0D,0x0F,0x10,
  0x11,0x1C,0x1F,0x20,
  0x21,0x23,0x2C,0x30,0x31,0x33,0x34,0x35,
  0x3E,0x40,
  0x41,0x42,0x45,0x46,0x49,0x4A,0x4C,0x4D,
  0x4E,0x51,0x59,0x5A,0x5D,0x5E,0x60,
  0x61,0x62,0x65,0x66,0x67,0x68,0x69,0x6A,
  0x6D,0x70,0x75,0x77,0x78,0x7A,0x7C,0x7F,
  0x80,
  0x86,0x87,0x88,0x8C,0x8F,0x92
};

const int PID_COUNT = sizeof(supportedPIDs) / sizeof(supportedPIDs[0]);

unsigned long lastPID = 0;
int pidIndex = 0;

/* ================= SEND OBD REQUEST ================= */

void sendOBD(uint8_t pid)
{
  twai_message_t tx = {};

  tx.identifier = OBD_REQ_ID;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 8;

  tx.data[0] = 0x02;
  tx.data[1] = 0x01;
  tx.data[2] = pid;

  for(int i=3;i<8;i++) tx.data[i] = 0;

  twai_transmit(&tx, pdMS_TO_TICKS(10));
}

/* ================= SETUP ================= */

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==========================================");
  Serial.println("KIA SELTOS – FULL CAN RAW LOGGER");
  Serial.println("Capturing ALL CAN traffic");
  Serial.println("==========================================");

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(
          (gpio_num_t)CAN_TX,
          (gpio_num_t)CAN_RX,
          TWAI_MODE_NORMAL
      );

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config,&t_config,&f_config) != ESP_OK ||
      twai_start() != ESP_OK)
  {
      Serial.println("CAN INIT FAILED");
      while(1);
  }

  Serial.println("CAN STARTED @ 500kbps\n");
}

/* ================= LOOP ================= */

void loop()
{
  /* ===== CONTINUOUS CAN SNIFFER ===== */

  twai_message_t rx;

  if(twai_receive(&rx,0) == ESP_OK)
  {
      Serial.printf("ID:0x%03X DLC:%d DATA:",
                    rx.identifier,
                    rx.data_length_code);

      for(int i=0;i<rx.data_length_code;i++)
          Serial.printf(" %02X",rx.data[i]);

      Serial.println();
  }

  /* ===== SEND OBD REQUEST OCCASIONALLY ===== */

  if(millis() - lastPID > 100)
  {
      sendOBD(supportedPIDs[pidIndex]);

      pidIndex++;

      if(pidIndex >= PID_COUNT)
          pidIndex = 0;

      lastPID = millis();
  }
}