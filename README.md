# OBD_Device_Drivers_Complete
ESP32-based OBD vehicle monitoring device that reads CAN bus data via SN65HVD230, collects GPS location using NEO-6M, and transmits telemetry through the A7670C GSM module. Designed for automotive diagnostics, fleet tracking, and telematics applications.

# ESP32 OBD Vehicle Monitoring Device

## Overview

This repository contains the complete hardware and firmware design for an **ESP32-based OBD (On-Board Diagnostics) device**.
The device connects to a vehicle’s **CAN bus through the OBD-II port**, reads diagnostic data, collects GPS location, and transmits information using a GSM network.

The system is designed for **vehicle diagnostics, fleet monitoring, telematics, and embedded automotive projects**.

---

## Key Features

* Vehicle data acquisition through **CAN Bus (OBD-II interface)**
* **ESP32 microcontroller** for processing and communication
* **SN65HVD230 CAN transceiver** for CAN-bus interface
* **NEO-6M GPS module** for real-time location tracking
* **A7670C GSM module** for cellular communication
* **Status LEDs** for system monitoring
* **12V vehicle power input with DC-DC conversion**
* Hardware protection including **TVS diodes and filtering**

---

## System Architecture

Vehicle OBD-II Port
↓
CANH / CANL
↓
SN65HVD230 CAN Transceiver
↓
ESP32 Microcontroller

ESP32 communicates with:

* GPS module (UART)
* GSM module (UART)
* CAN bus (TWAI interface)

Collected data can be transmitted over GSM to a remote server.

---

## Hardware Components

| Component      | Description                           |
| -------------- | ------------------------------------- |
| ESP32-WROOM-32 | Main microcontroller                  |
| SN65HVD230     | CAN bus transceiver                   |
| A7670C         | GSM communication module              |
| NEO-6M         | GPS receiver module                   |
| LM2596         | 12V to 5V buck converter              |
| AMS1117-3.3    | 5V to 3.3V regulator                  |
| PESD1CAN       | CAN bus TVS protection                |
| LEDs           | Power, GPS, and GSM status indicators |

---

## Power Architecture

Vehicle Battery (12V)
↓
LM2596 Buck Converter
↓
5V Rail

From 5V:

* AMS1117 → 3.3V (ESP32, CAN, GPS)
* Dedicated 4V rail → GSM module

Protection and filtering components are included to ensure stable operation in automotive environments.

---

## CAN Bus Interface

The device connects to the vehicle CAN bus via the OBD connector.

Typical OBD-II CAN pins:

* Pin 6 → CAN High (CANH)
* Pin 14 → CAN Low (CANL)
* Pin 4 / 5 → Ground

Internal signal path:

```id="1xm1px"
Vehicle CANH/CANL
        ↓
TVS diode (PESD1CAN)  ← protection only
        ↓
CANH / CANL pins of SN65HVD230
        ↓
ESP32 (CAN_RX / CAN_TX)
```

External connection:

```id="wr24th"
[ Connector ]
   |
   |── CANH ──→ TVS diode ──→ SN65HVD230 CANH
   |
   |── CANL ──→ TVS diode ──→ SN65HVD230 CANL
   |
   |── GND  ───────────────→ Board GND
```

---

## Firmware Responsibilities

The ESP32 firmware performs the following tasks:

1. Initialize CAN interface
2. Listen to vehicle CAN frames
3. Parse OBD-II PIDs
4. Read GPS location
5. Communicate through GSM network
6. Send telemetry data to remote server
7. Control status LEDs

---

## Repository Structure

```
OBD_Device/
│
├── docs/           → Documentation and architecture
├── hardware/       → Schematics, PCB, and BOM
├── firmware/       → ESP32 firmware source code
├── testing/        → Hardware validation programs
└── tools/          → Debugging and utility scripts
```

---

## Status LEDs

| LED       | Purpose                           |
| --------- | --------------------------------- |
| Power LED | Indicates board power             |
| GPS LED   | Indicates GPS fix status          |
| GSM LED   | Shows network registration status |

---

## Development Environment

Recommended tools:

* **PlatformIO** or **Arduino IDE** for ESP32 firmware
* **EasyEDA** for hardware design
* **GitHub** for repository hosting

---

## Safety Notes

* Do not add extra CAN termination when connecting to a vehicle.
* Ensure a common ground between the device and vehicle.
* Automotive power can be noisy; proper filtering is required.

---

## Future Improvements

* CAN message filtering
* OBD-II diagnostic command support
* Cloud telemetry integration
* Firmware OTA updates
* Vehicle data logging

---

## License

This project is released under the **MIT License**.

---

## Author

Beer Bhadra Jain - Hardware Engineer

Embedded Systems Development Project
ESP32 Automotive OBD Interface
