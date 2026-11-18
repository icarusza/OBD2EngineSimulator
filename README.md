# OBD2 Engine Simulator

## Project Overview

Arduino UNO-based OBD-II ECU simulator using an MCP2515 CAN bus shield. Simulates OBD-II CAN communication (ISO 15765-4: 11-bit or 29-bit identifiers at 500 or 250 kbit/s, selectable at runtime) and a selection of standard diagnostic PIDs and services. The modular firmware can be extended with additional PID responses to suit individual testing requirements.

The firmware requires the `mcp_can` library by coryjfowler (MCP_CAN_lib, available in the Arduino Library Manager). Open the sketch folder `Arduino-Firmware/OBD2EngineSimulator/`. For MCP2515 boards with an 8 MHz crystal, change `MCP_CLOCK` in `Config.h`.

### Firmware Configuration (`Config.h`)

| Setting | Default | Description |
|---|---|---|
| `SPI_CS_PIN` | 10 | MCP2515 chip-select pin |
| `MCP_CLOCK` | `MCP_16MHZ` | MCP2515 crystal; use `MCP_8MHZ` for 8 MHz modules |
| `DEFAULT_CAN_MODE` | 6 | CAN mode used when the EEPROM holds no saved mode |
| `DEFAULT_VIN` | `1M8GDM9AXKP042788` | VIN at boot (17 characters) |
| `DEFAULT_CALID` | `OBDSIM-CAL-0001` | Calibration ID at boot (up to 16 characters) |
| `DEFAULT_ECU_NAME` | `ECM-EngineControl` | ECU name at boot (up to 20 characters) |
| `OBD_STANDARD` | `0x06` (EOBD) | Value reported in PID 1C |
| `MAX_DTCS_PER_LIST` | 8 | Size of each DTC list |

VIN, calibration ID and ECU name changed over serial are kept in RAM only and return to these defaults after a reset.

Includes a C# Windows Forms application communicating with the Arduino over USB serial. Designed for testing with ELM327 and GODIAG 101 diagnostic interfaces connected to the CAN bus.

J2534 Pass-Thru integration is a potential future development.

## Supported OBD-II PIDs

The simulator currently implements 28 standard OBD-II Mode 01 data PIDs. Up to six PIDs can be requested in one message.

| PID | Parameter | Description |
|---|---|---|
| 01 | Monitor Status | MIL state and number of stored DTCs; spark ignition. Catalyst, EVAP, O2 sensor and O2 heater monitors report "not complete" for 5 minutes of engine running after DTCs are cleared |
| 03 | Fuel System Status | Set with `FUELSYS`: 1 open loop (cold), 2 closed loop, 4 open loop (driving conditions), 8 open loop (fault), 16 closed loop (fault) |
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
| 1C | OBD Standard | Set by `OBD_STANDARD` in `Config.h` (default EOBD) |
| 1F | Run Time Since Engine Start | Seconds since RPM rose above 300 |
| 21 | Distance With MIL On | Accumulated from vehicle speed |
| 24 | Wide-range Oxygen Sensor | Lambda calculated from measured AFR; sensor voltage set with `O2V` |
| 2F | Fuel Tank Level | Fuel level |
| 30 | Warm-ups Since Codes Cleared | Counted when ECT goes from below 50 °C to 70 °C |
| 31 | Distance Since Codes Cleared | Accumulated from vehicle speed |
| 33 | Barometric Pressure | Barometric pressure |
| 42 | Control Module Voltage | Battery voltage |
| 44 | Commanded Equivalence Ratio | Commanded equivalence ratio |
| 46 | Ambient Air Temperature | Ambient temperature |
| 4D | Time Run With MIL On | Minutes |
| 4E | Time Since Codes Cleared | Minutes |
| 5C | Engine Oil Temperature | Oil temperature |

### PID Support Information

The simulator also responds to the following PID availability requests:

- PID 00: Supported PIDs 01–20
- PID 20: Supported PIDs 21–40
- PID 40: Supported PIDs 41–60

These responses allow diagnostic software to identify which PIDs the simulator advertises. They are generated from the implemented PIDs, so they stay correct when PIDs are added.

### Supported Services

| Service | Description |
|---|---|
| 01 | Current data (PIDs above) |
| 02 | Freeze frame (frame 0), captured when the first stored DTC is set. PID 02 returns the DTC that caused it (0000 if none). Snapshot PIDs (03–07, 0B–11, 24, 2F, 33, 42, 44, 46, 5C) are answered only while a freeze frame exists |
| 03 / 07 / 0A | Stored, pending and permanent DTCs (up to 8 each). A stored DTC turns the MIL on |
| 04 | Clear stored and pending DTCs, freeze frame and "since cleared" counters (PIDs 21, 30, 31, 4D, 4E). Permanent DTCs stay |
| 06 | MID 00 (supported MIDs) and MID 01 (O2 sensor bank 1 sensor 1): test IDs 01 and 02 with UASID 0A (volts), value from `O2V`, limits 0.3–0.6 V |
| 09 | InfoType 00 (supported), 02 VIN, 04 calibration ID, 0A ECU name (multi-frame), and message counts 01/03/09 for older tools. An ECU name such as `ECM-EngineControl` is sent in J1979 layout: acronym padded to 4 bytes, `-`, then the name |

Responses longer than one CAN frame use ISO-TP (First Frame, Flow Control, Consecutive Frames). The firmware honours the tester's block size, separation time (STmin) and WAIT frames, and aborts if no Flow Control arrives within 1 second. Serial commands keep being processed while it waits. Unsupported requests sent to the physical address get a negative response (`7F <service> 11` for an unknown service, `7F <service> 12` for an unsupported PID or InfoType). Unsupported functional (broadcast) requests are ignored.

The MCP2515 hardware filters accept only the request IDs of the active mode. If the MCP2515 fails to start, the firmware reports an error and retries every 2 seconds while serial commands keep working. With the ignition off (`IGN:0`) the simulator does not answer any CAN request and the run-time and distance counters stop.

| CAN mode | Identifiers | Bitrate | Request IDs | Response ID |
|---|---|---|---|---|
| 6 (default) | 11-bit | 500 kbit/s | 7DF / 7E0 | 7E8 |
| 7 | 29-bit | 500 kbit/s | 18DB33F1 / 18DA10F1 | 18DAF110 |
| 8 | 11-bit | 250 kbit/s | 7DF / 7E0 | 7E8 |
| 9 | 29-bit | 250 kbit/s | 18DB33F1 / 18DA10F1 | 18DAF110 |

The mode numbers match ELM327 protocols (`ATSP6` to `ATSP9`). The selected mode is saved in EEPROM and used when the Arduino runs without the application. When the application connects, it sets the mode selected in its CAN protocol dropdown.

## Serial Engine Parameter Control

The C# Windows Forms application communicates with the Arduino UNO over USB serial.

The firmware uses a fixed serial baud rate of 115200 baud. The application includes a baud-rate selection dropdown, but 115200 must be selected to communicate correctly with the current firmware.

The firmware accepts parameter commands in the following format:

`PARAMETER:VALUE`

Available parameters:

| Parameter | PID | Unit | Range | Decimals | Default |
|---|---|---|---|---|---|
| RPM | 0C | rpm | 0 – 12000 | 0 | 850 |
| MAP | 0B | kPa | 0 – 170 | 0 | 85 |
| STFT | 06 | % | -100 – 99.2 | 1 | 3.1 |
| LTFT | 07 | % | -100 – 99.2 | 1 | -2.3 |
| AFR | 44 | AFR | 8 – 20 | 1 | 14.7 |
| MEASURED_AFR | 24 | AFR | 8 – 20 | 1 | 14.7 |
| ECT | 05 | °C | -40 – 215 | 0 | 90 |
| IAT | 0F | °C | -40 – 215 | 0 | 25 |
| TPS | 11 | % | 0 – 100 | 1 | 0 |
| SPEED | 0D | km/h | 0 – 255 | 0 | 0 |
| LOAD | 04 | % | 0 – 100 | 1 | 20 |
| MAF | 10 | g/s | 0 – 254 | 1 | 3.5 |
| TIMING | 0E | ° | -64 – 63.5 | 1 | 10 |
| FUEL | 2F | % | 0 – 100 | 1 | 75 |
| BARO | 33 | kPa | 0 – 255 | 0 | 101 |
| VBATT | 42 | V | 0 – 65.5 | 2 | 14.2 |
| AMBIENT | 46 | °C | -40 – 215 | 0 | 20 |
| OIL | 5C | °C | -40 – 215 | 0 | 90 |
| O2V | 24, Mode 06 | V | 0 – 5 | 3 | 0.45 |
| FUELSYS | 03 | code | 0, 1, 2, 4, 8, 16 | 0 | 2 |

Values outside the range are limited to it, and values are rounded to the listed decimals. The echo in the reply shows the result.

Other commands:

| Command | Description |
|---|---|
| `CAN:6` … `CAN:9` | Select the CAN mode (see table above) |
| `IGN:0` / `IGN:1` | Ignition off (no CAN replies) / on |
| `LOG:0` / `LOG:1` / `LOG:2` | Serial log: errors only / OBD requests and replies / every CAN frame |
| `DTC:ADD:S:P0171`, `DTC:DEL:S:P0171` | Add or remove a DTC. List type `S` stored, `P` pending, `X` permanent |
| `DTC:SET:S:P0171,P0300` | Replace a DTC list (empty list clears it) |
| `DTC:CLEAR`, `DTC:CLEAR:ALL` | Clear like Mode 04 / also clear permanent DTCs |
| `VIN:<17 chars>`, `CALID:<text>`, `ECUNAME:<text>` | Mode 09 vehicle information |
| `INFO`, `GET`, `DTC?` | Read back firmware status, all parameter values, DTC lists |

Several commands can be sent on one line (up to 200 characters), separated by `;`. A line can start with a sequence number (`#12 RPM:850;MAP:35`). The Arduino answers every line with `OK #12 RPM=850;MAP=35`, echoing the values it actually applied (after range limits and rounding), preceded by `ERR #12 <reason> <name>` for each rejected command. A line that cannot be used at all gets a single `ERR #12 <reason>` and no `OK`. The application uses these replies to confirm every value.

| Error reason | Meaning |
|---|---|
| `BAD_FORMAT` | Command without `:` |
| `BAD_VALUE` | Value not a number, out of the allowed set, or invalid text/VIN/DTC |
| `UNKNOWN` | Unknown command name |
| `LIST_FULL` | DTC list already holds 8 codes |
| `CAN_INIT_FAILED` | MCP2515 did not start in the requested CAN mode (retried every 2 s) |
| `BAD_SEQUENCE`, `EMPTY`, `LINE_TOO_LONG` | Whole line rejected |

Query replies:

- `INFO` → `OK FW=2.0.0;CAN=6;CANOK=1;IGN=1;LOG=1;UP=<ms since boot>;SEQ=<last sequence number>;MIL=0;LOGDROP=<skipped log lines>;VIN=…;CALID=…;ECUNAME=…`
- `GET` → `OK RPM=850;MAP=85;…` (all parameters)
- `DTC?` → `OK DTC=S:P0171|P:P0171|X:;FF=P0171` (`FF` is the DTC that captured the freeze frame, `-` if none)

The Arduino also prints lines on its own:
- `READY` after booting.
- `CAN READY | MODE …` when CAN starts.
- `OBD REQ …` / `OBD RSP …` at log level 1 or higher.
- `CAN RX …` / `CAN TX …` at log level 2.
- `EVENT DTC CLEARED BY TESTER` after a scan tool sends Mode 04.
- `ERROR: …` messages.

Log lines that would make the Arduino wait for the serial port are skipped, so busy buses show gaps in the log. They are counted in `LOGDROP`.

This allows the PC application to change simulated engine operating conditions while diagnostic equipment independently requests OBD-II data over CAN.

## Windows Forms Application

The project includes a C# Windows Forms application developed in Visual Studio.

The application communicates with the Arduino UNO over USB serial and provides a graphical interface for controlling the simulated ECU parameters.

### Features

- Automatic COM port detection and refresh.
- USB serial connection and disconnection.
- Configurable serial baud-rate selection (115200 required by the current firmware).
- CAN protocol selection (11-bit or 29-bit, 500 or 250 kbit/s). The choice is sent to the Arduino, and the active mode is shown once the Arduino confirms it, including whether the MCP2515 started.
- Ignition on/off and Arduino log-level selection.
- "Log serial traffic" option to show every command sent and every `OK` reply in the log.
- Individual ECU parameter controls using sliders and numeric inputs.
- Live parameter updates sent to the Arduino, batched about 10 times per second.
- Connection handshake: after opening the port, the application waits up to 4 seconds for the Arduino's `READY` line or an `INFO` reply, then sends all settings, values, vehicle information and DTC lists.
- Confirmation of every value:
  - Each parameter tab shows ✓ confirmed, … waiting, ⚠ limited by the firmware range, or ✗ not confirmed, with the last confirmed value.
  - A value without a reply within 300 ms is resent, up to 3 times.
  - A read-back every 2 seconds (`INFO`, `GET`, `DTC?`) resends any value the Arduino does not hold.
  - An Arduino restart (`READY`, or uptime going back) is detected, and all values are restored.
  - The link status shows reply age, round-trip time, retries and failures.
- Diagnostics tab:
  - Stored, pending and permanent DTC lists: add, remove, "Clear stored + pending" (same as a scan tool's Mode 04) and "Clear all".
  - The MIL state and the freeze frame DTC reported by the Arduino.
  - VIN (validated: 17 characters, no I, O or Q), calibration ID and ECU name.
  - DTCs cleared by a scan tool are reflected in the lists.
- Driving simulation with different tuning modes:
  - It continues from the current temperatures and fuel level.
  - MAF and load are calculated from RPM, MAP and intake temperature (2.0 L speed-density model).
  - Coolant warms up towards 90 °C, oil follows coolant, and intake temperature falls with speed.
  - Fuel level drains with fuel used, battery voltage is 14.2 V while running, and O2 voltage follows the measured AFR.
  - Fuel system status changes: open loop when cold (below 40 °C), at wide-open throttle and on overrun fuel cut; closed loop otherwise.
- Optional auto DTCs during simulation:
  - Total fuel trim above +20 % (lean) or below -20 % (rich) in closed loop sets P0171 or P0172 as pending after 10 seconds and as stored (MIL on) after 30 seconds.
  - Bad tune sets P0300 the same way.
  - Clearing DTCs restarts the detection.
- Optional linking of MAF and load to RPM, MAP and IAT in manual mode (on by default).
- Simulated engine RPM, MAP, fuel trims, AFR, throttle position, vehicle speed, engine load, MAF, ignition timing, fuel level, barometric pressure, battery voltage, ambient and oil temperature, O2 sensor voltage and fuel system status.
- Live serial communication log.
- Lambda display for commanded and measured AFR.

### Project Structure

- `OBD2EngineSimulator.slnx` – Visual Studio solution.
- `OBD2EngineSimulator/OBD2EngineSimulator.csproj` – C# project file.
- `OBD2EngineSimulator/Form1.cs` – Main interface logic.
- `OBD2EngineSimulator/ArduinoLink.cs` – Serial link: batching, sequence numbers, confirmation, retries, read-back.
- `OBD2EngineSimulator/EngineModel.cs` – Driving simulation.
- `OBD2EngineSimulator/ParameterDefinitions.cs` – Parameter names, ranges and precision.
- `OBD2EngineSimulator/Form1.Designer.cs` – Windows Forms designer file.
- `OBD2EngineSimulator/Form1.resx` – Form resources.
- `OBD2EngineSimulator/Program.cs` – Application entry point.
- `Arduino-Firmware/OBD2EngineSimulator/` – Arduino sketch: `OBD2EngineSimulator.ino`, `Config.h`, `EngineState`, `CanLink`, `ObdService`, `SerialCommands`.

### Serial Communication Example

The application sends engine parameter updates to the Arduino using simple text commands over USB serial. The commands below are also accepted one per line, for example from a serial terminal (the application itself batches them with sequence numbers as described above).

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

The Arduino firmware receives these commands and updates the simulated ECU values, answering each line with `OK` and the applied value (for example `OK TPS=10.0`).

The application sends batched lines such as:

```text
#41 RPM:3120;MAP:57;STFT:1.8;LTFT:-1.4;AFR:14.7;MEASURED_AFR:14.7
OK #41 RPM=3120;MAP=57;STFT=1.8;LTFT=-1.4;AFR=14.7;MEASURED_AFR=14.7
```

## Extending the Simulator

The Arduino firmware is designed to allow additional PID responses to be implemented.

PID processing is handled inside the `EncodePid()` function in `ObdService.cpp` using a C++ switch statement.

Developers can extend the simulator by:

1. Adding a new PID case to `EncodePid()`.
2. Reading or calculating the required engine parameter (new parameters go in `PARAM_DEFS` in `EngineState.cpp` and `ParameterDefinitions.cs`, with the same decimals).
3. Converting the value into the appropriate OBD-II response format.
4. Setting the correct response data length.

The supported PID bitmaps are generated from `EncodePid()`, so diagnostic software discovers the new PID automatically. Mode 02 (freeze frame) uses the same encoders; PIDs that are not snapshot values return early with `LIVE_ONLY()`.

The existing `IsoTpSend()` function handles CAN response transmission, including multi-frame responses, meaning additional PIDs can reuse the existing communication infrastructure.

Developers should consult the relevant OBD-II PID definitions for the correct response formulas, scaling and data formats.

## Future Development

- Additional standard OBD-II PID support.
- J2534 Pass-Thru integration and compatibility testing.
<img width="1195" height="896" alt="UNO2" src="https://github.com/user-attachments/assets/b6897a9f-c7d8-421a-967e-31e9032de84b" />
<img width="896" height="1195" alt="UNO" src="https://github.com/user-attachments/assets/b329b943-22cb-433b-b981-d717059fea61" />
<img width="1072" height="833" alt="image" src="https://github.com/user-attachments/assets/4804e5e3-0da0-4f91-bc44-ab0bb5e22622" />

