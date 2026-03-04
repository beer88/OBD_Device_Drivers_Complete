# Project Overview

This project implements an ESP32-based OBD vehicle monitoring system capable of reading vehicle CAN bus data, acquiring GPS location, and transmitting telemetry via GSM.

## System Components

- ESP32-WROOM-32 Microcontroller
- SN65HVD230 CAN Transceiver
- A7670C GSM Module
- NEO-6M GPS Module
- Automotive power supply (12V input)

## Data Flow

Vehicle CAN Bus → CAN Transceiver → ESP32  
ESP32 → GSM Module → Cloud Server  
ESP32 ← GPS Module (Location)

## Applications

- Fleet tracking
- Vehicle diagnostics
- Automotive telemetry
- IoT vehicle monitoring