# OBD2 Engine Simulator

## Project Overview

Arduino UNO-based OBD-II ECU simulator using an MCP2515 CAN bus shield. Simulates basic OBD-II CAN communication and a selection of standard diagnostic PIDs. The modular firmware can be extended with additional PID responses to suit individual testing requirements.

Includes a C# Windows Forms application communicating with the Arduino over USB serial. Supports testing with ELM327 and GODIAG 101 diagnostic interfaces via CAN-H and CAN-L.

J2534 Pass-Thru integration is a potential future development.
