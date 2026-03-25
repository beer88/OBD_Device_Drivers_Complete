#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 5
#define CAN_INT 4

MCP_CAN CAN(CAN_CS);

// LIN UART
HardwareSerial LIN(2);

void setup() {
  Serial.begin(115200);

  // ================= CAN INIT =================
  while (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init fail, retrying...");
    delay(1000);
  }
  Serial.println("CAN init success!");
  CAN.setMode(MCP_NORMAL);

  pinMode(CAN_INT, INPUT);

  // ================= LIN INIT =================
  LIN.begin(19200, SERIAL_8N1, 16, 17);
  Serial.println("LIN init success!");

  Serial.println("System Ready...");
}

void loop() {

  // ================= CAN READ =================
  if (!digitalRead(CAN_INT)) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    CAN.readMsgBuf(&rxId, &len, rxBuf);

    Serial.print("CAN ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" Data: ");

    for (int i = 0; i < len; i++) {
      if (rxBuf[i] < 16) Serial.print("0");
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // ================= LIN READ =================
  while (LIN.available()) {
    byte b = LIN.read();

    if (b == 0x55) {
      Serial.print("\nLIN: ");
    }

    if (b < 16) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");
  }
}