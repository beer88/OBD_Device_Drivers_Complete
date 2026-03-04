# Hardware Design

This folder contains all hardware-related design files for the OBD device.

## Contents

- Schematic files
- PCB layout
- Bill of Materials (BOM)
- Pick and Place data
- Manufacturing files

## Main Hardware Blocks

- ESP32 microcontroller
- CAN bus interface using SN65HVD230
- GSM communication module (A7670C)
- GPS module (NEO-6M)
- Automotive power supply regulation

## CAN Bus Connection

CANH and CANL from the vehicle OBD connector are connected to the SN65HVD230 CAN transceiver with TVS protection for automotive surge protection.