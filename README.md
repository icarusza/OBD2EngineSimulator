# OBD2 Engine Simulator

## Project Overview

Arduino UNO-based OBD-II ECU simulator using an MCP2515 CAN bus shield. Simulates basic OBD-II CAN communication and a selection of standard diagnostic PIDs. The modular firmware can be extended with additional PID responses to suit individual testing requirements.

Includes a C# Windows Forms application communicating with the Arduino over USB serial. Designed for testing with ELM327 and GODIAG 101 diagnostic interfaces connected to the CAN bus.

J2534 Pass-Thru integration is a potential future development.

## Supported OBD-II PIDs

The simulator currently implements 14 standard OBD-II Mode 01 data PIDs.

| PID | Parameter | Description |
|---|---|---|
| 04 | Engine Load | Calculated engine load |
| 05 | Coolant Temperature | Engine coolant temperature |
| 06 | Short Term Fuel Trim | STFT |
| 07 | Long Term Fuel Trim | LTFT |
| 0B | Manifold Pressure | MAP |
| 0C | Engine RPM | Engine speed |
| 0D | Vehicle Speed | Vehicle speed |
| 0E | Ignition Timing | Ignition advance |
| 0F | Intake Temperature | Intake air temperature |
| 10 | Mass Air Flow | MAF |
| 11 | Throttle Position | Throttle position |
| 13 | Oxygen Sensor Locations | Sensor availability |
| 24 | Wide-range Oxygen Sensor | Lambda calculated from measured AFR; sensor voltage currently fixed at 0.5 V |
| 44 | Commanded Equivalence Ratio | Commanded equivalence ratio |

### PID Support Information

The simulator also responds to the following PID availability requests:

- PID 00: Supported PIDs 01–20
- PID 20: Supported PIDs 21–40
- PID 40: Supported PIDs 41–60

These responses allow diagnostic software to identify which PIDs the simulator advertises.

## Serial Engine Parameter Control

The C# Windows Forms application communicates with the Arduino UNO over USB serial.

The firmware uses a fixed serial baud rate of 115200 baud. The application includes a baud-rate selection dropdown, but 115200 must be selected to communicate correctly with the current firmware.

The firmware accepts parameter commands in the following format:

`PARAMETER:VALUE`

Available parameters include RPM, MAP, STFT, LTFT, AFR, MEASURED_AFR, ECT, IAT, TPS, SPEED, LOAD, MAF and TIMING.

This allows the PC application to change simulated engine operating conditions while diagnostic equipment independently requests OBD-II data over CAN.

## Windows Forms Application

The project includes a C# Windows Forms application developed in Visual Studio.

The application communicates with the Arduino UNO over USB serial and provides a graphical interface for controlling the simulated ECU parameters.

### Features

- Automatic COM port detection and refresh.
- USB serial connection and disconnection.
- Configurable serial baud-rate selection (115200 required by the current firmware).
- CAN protocol selection and display. The current Arduino firmware uses a fixed 29-bit, 500 kbit/s CAN configuration.
- Individual ECU parameter controls using sliders and numeric inputs.
- Live parameter updates sent to the Arduino.
- Driving simulation with different tuning modes.
- Simulated engine RPM, MAP, fuel trims, AFR, throttle position, vehicle speed, engine load, MAF and ignition timing.
- Live serial communication log.
- Lambda display for commanded and measured AFR.

### Project Structure

- `OBD2EngineSimulator.slnx` – Visual Studio solution.
- `OBD2EngineSimulator/OBD2EngineSimulator.csproj` – C# project file.
- `OBD2EngineSimulator/Form1.cs` – Main application code and interface logic.
- `OBD2EngineSimulator/Form1.Designer.cs` – Windows Forms designer file.
- `OBD2EngineSimulator/Form1.resx` – Form resources.
- `OBD2EngineSimulator/Program.cs` – Application entry point.

### Serial Communication Example

The application sends engine parameter updates to the Arduino using simple text commands over USB serial.

```text
RPM:850
MAP:85
STFT:3.1
LTFT:-2.3
AFR:14.7
MEASURED_AFR:14.7
ECT:90
IAT:25
TPS:10
SPEED:60
LOAD:35
MAF:12.5
TIMING:18
```

The Arduino firmware receives these commands and updates the simulated ECU values.

## Extending the Simulator

The Arduino firmware is designed to allow additional PID responses to be implemented.

PID processing is handled inside the `ProcessOBDRequest()` function using a C++ switch statement.

Developers can extend the simulator by:

1. Adding a new PID case to `ProcessOBDRequest()`.
2. Reading or calculating the required engine parameter.
3. Converting the value into the appropriate OBD-II response format.
4. Setting the correct response data length.
5. Updating the supported PID bitmap so diagnostic software can discover the new PID.

The existing `SendOBDResponse()` function handles CAN response transmission, meaning additional PIDs can reuse the existing communication infrastructure.

Developers should consult the relevant OBD-II PID definitions for the correct response formulas, scaling and data formats.

## Future Development

- Additional standard OBD-II PID support.
- Expanded engine simulation parameters.
- Additional diagnostic functionality.
- J2534 Pass-Thru integration and compatibility testing.
<img width="1195" height="896" alt="UNO2" src="https://github.com/user-attachments/assets/b6897a9f-c7d8-421a-967e-31e9032de84b" />
<img width="896" height="1195" alt="UNO" src="https://github.com/user-attachments/assets/b329b943-22cb-433b-b981-d717059fea61" />
<img width="1072" height="833" alt="image" src="https://github.com/user-attachments/assets/4804e5e3-0da0-4f91-bc44-ab0bb5e22622" />

