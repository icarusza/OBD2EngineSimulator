# OBD2 Engine Simulator

## Project Overview

Arduino UNO-based OBD-II ECU simulator using an MCP2515 CAN bus shield. Simulates OBD-II CAN communication (ISO 15765-4: 11-bit or 29-bit identifiers at 500 or 250 kbit/s, selectable at runtime) with an engine ECU and an optional transmission ECU, the standard OBD services 01–0A, and minimal OBD on UDS (SAE J1979-2). DTCs follow a realistic lifecycle over drive cycles and, like the odometer and vehicle information, are kept in EEPROM across power cycles. The modular firmware can be extended with additional PID responses to suit individual testing requirements.

The firmware requires the `mcp_can` library by coryjfowler (MCP_CAN_lib, available in the Arduino Library Manager). Open the sketch folder `Arduino-Firmware/OBD2EngineSimulator/`. For MCP2515 boards with an 8 MHz crystal, change `MCP_CLOCK` in `Config.h`.

### Firmware Configuration (`Config.h`)

| Setting | Default | Description |
|---|---|---|
| `SPI_CS_PIN` | 10 | MCP2515 chip-select pin |
| `MCP_CLOCK` | `MCP_16MHZ` | MCP2515 crystal; use `MCP_8MHZ` for 8 MHz modules |
| `DEFAULT_CAN_MODE` | 6 | CAN mode used when the EEPROM holds no saved mode |
| `DEFAULT_VIN` | `1M8GDM9AXKP042788` | VIN until changed (17 characters) |
| `DEFAULT_CALID` | `OBDSIM-CAL-0001` | Calibration ID until changed (up to 16 characters) |
| `DEFAULT_ECU_NAME` | `ECM-EngineControl` | Engine ECU name until changed (up to 20 characters) |
| `DEFAULT_ESN` | `OBDSIM-ENG-00001` | Engine serial number until changed (up to 16 characters) |
| `TCM_ECU_NAME` | `TCM-TransmissionCtl` | Transmission ECU name |
| `OBD_STANDARD` | `0x06` (EOBD) | Value reported in PID 1C |
| `DUAL_BANK` | 1 | V engine: bank 2 fuel trims and O2 sensors |
| `FUEL_TYPE` | `0x01` (gasoline) | Value reported in PID 51 |
| `ETHANOL_PERCENT` | 10 | Value reported in PID 52 |
| `REFERENCE_TORQUE_NM` | 250 | Value reported in PID 63 |
| `ECM_ADDRESS_29`, `TCM_ADDRESS_29` | `0x10`, `0x18` | 29-bit ECU addresses |
| `CYCLE_MIN_RUN_S` | 60 | Engine run time for a drive cycle to count |
| `CYCLES_TO_CONFIRM` | 2 | Consecutive failing drive cycles before a DTC is confirmed |
| `CYCLES_TO_MIL_OFF` | 3 | Consecutive passing drive cycles before the MIL goes off |
| `WARMUPS_TO_ERASE` | 40 | Warm-ups after MIL off before a DTC is erased |
| `MAX_DTCS` | 10 | DTC records (engine and transmission together) |
| `MAX_SNAPSHOTS` | 3 | Freeze frames |

VIN, calibration ID, ECU name, engine serial number, DTCs, freeze frames, counters, the odometer and the TCM setting are stored in EEPROM. Changes are written in the background, one byte at a time, so the simulator never stalls. There are two copies; each save goes to the older one, so a reset or power loss during a save keeps the previous state. DTC changes are saved at once, counters and the odometer at most once a minute and when the ignition goes off. `FACTORY:1` (or "Factory reset ECU memory" in the application) restores the defaults.

### Feature Switches and Memory Use

The Arduino UNO has 32 256 bytes of program memory and 2 048 bytes of RAM. All features fit with every switch on (the default). Values are handled as fixed-point integers, so the floating-point library is not needed. Measured with PlatformIO (avr-gcc 7.3, coryjfowler `mcp_can`):

| Build | Flash | RAM (static) |
|---|---|---|
| All features (default) | 28 810 bytes | 1 182 bytes |
| All switches off | 24 526 bytes | 1 124 bytes |

| Switch | Feature | Flash saved when off |
|---|---|---|
| `FEATURE_UDS` | Minimal OBD on UDS and multi-frame requests | 2 336 bytes (and 58 bytes RAM) |
| `FEATURE_TCM` | Transmission ECU | 824 bytes |
| `FEATURE_MODE08` | Mode 08 EVAP leak test | 528 bytes |
| `FEATURE_EXTRA_PIDS` | EGR/EVAP, catalyst temperature, torque and sensor PIDs | 476 bytes |
| `FEATURE_MODE06_EXT` | Mode 06 catalyst, EGR, EVAP and misfire MIDs | 324 bytes |
| `FEATURE_IPT` | Mode 09 in-use performance tracking | 198 bytes |

If additions make a build too large, switch features off in this order: `FEATURE_MODE08`, `FEATURE_MODE06_EXT`, `FEATURE_IPT`, `FEATURE_EXTRA_PIDS`, `FEATURE_UDS`, `FEATURE_TCM`.

Includes a C# Windows Forms application communicating with the Arduino over USB serial. Designed for testing with ELM327 and GODIAG 101 diagnostic interfaces connected to the CAN bus.

J2534 Pass-Thru integration is a potential future development.

## Supported OBD-II PIDs

The engine ECU implements the following standard OBD-II Mode 01 data PIDs. Up to six PIDs can be requested in one message.

| PID | Parameter | Description |
|---|---|---|
| 01 | Monitor Status | MIL state (also set while it flashes) and number of confirmed DTCs; spark ignition. Catalyst, EVAP, O2 sensor, O2 heater and EGR monitors report "not complete" after DTCs are cleared, until a drive cycle completes |
| 03 | Fuel System Status | Set with `FUELSYS`: 1 open loop (cold), 2 closed loop, 4 open loop (driving conditions), 8 open loop (fault), 16 closed loop (fault) |
| 04 | Engine Load | Calculated engine load |
| 05 | Coolant Temperature | Engine coolant temperature |
| 06 / 07 | Short / Long Term Fuel Trim Bank 1 | `STFT`, `LTFT` |
| 08 / 09 | Short / Long Term Fuel Trim Bank 2 | `STFT2`, `LTFT2` (with `DUAL_BANK`) |
| 0B | Manifold Pressure | MAP |
| 0C | Engine RPM | Engine speed |
| 0D | Vehicle Speed | Vehicle speed |
| 0E | Ignition Timing | Ignition advance |
| 0F | Intake Temperature | Intake air temperature |
| 10 | Mass Air Flow | MAF |
| 11 | Throttle Position | Throttle position |
| 13 | Oxygen Sensor Locations | B1S1, B1S2 (and B2S1, B2S2 with `DUAL_BANK`) |
| 15 / 19 | O2 Sensor B1S2 / B2S2 (narrowband, downstream) | About 0.65 V; follows the upstream sensor while P0420 (catalyst) is active |
| 1C | OBD Standard | Set by `OBD_STANDARD` in `Config.h` (default EOBD) |
| 1F | Run Time Since Engine Start | Seconds since RPM rose above 300 |
| 21 | Distance With MIL On | Accumulated from vehicle speed |
| 23 | Fuel Rail Gauge Pressure | `RAILP` (direct injection) |
| 24 / 28 | Wide-range O2 Sensor B1S1 / B2S1 (upstream) | Lambda calculated from measured AFR; sensor voltage set with `O2V` |
| 2F | Fuel Tank Level | Fuel level |
| 30 | Warm-ups Since Codes Cleared | Counted when ECT goes from below 50 °C to 70 °C |
| 31 | Distance Since Codes Cleared | Accumulated from vehicle speed |
| 33 | Barometric Pressure | Barometric pressure |
| 41 | Monitor Status This Drive Cycle | Monitors complete once the engine ran `CYCLE_MIN_RUN_S` in this cycle |
| 42 | Control Module Voltage | Battery voltage |
| 43 | Absolute Load | Equal to the calculated load |
| 44 | Commanded Equivalence Ratio | Commanded equivalence ratio |
| 45 | Relative Throttle Position | Throttle position minus 2 % |
| 46 | Ambient Air Temperature | Ambient temperature |
| 47 | Absolute Throttle Position B | Throttle position plus 4 % |
| 49 / 4A | Accelerator Pedal Position D / E | Derived from throttle position (E is half of D) |
| 4C | Commanded Throttle Actuator | Equal to throttle position |
| 4D | Time Run With MIL On | Minutes |
| 4E | Time Since Codes Cleared | Minutes |
| 51 | Fuel Type | `FUEL_TYPE` |
| 52 | Ethanol Fuel | `ETHANOL_PERCENT` |
| 59 | Fuel Rail Absolute Pressure | `RAILP` plus barometric pressure |
| 5C | Engine Oil Temperature | Oil temperature |
| 5E | Engine Fuel Rate | MAF / measured AFR, in L/h |
| A6 | Odometer | Accumulated from vehicle speed, stored in EEPROM, set with `ODO` |

With `FEATURE_EXTRA_PIDS` (default on):

| PID | Parameter | Description |
|---|---|---|
| 2C / 2D | Commanded EGR / EGR Error | 10 % at part load (20–60 % load, 1200–3500 rpm); error -25 % while P0401 is active |
| 2E | Commanded Evaporative Purge | 15 % in closed loop with ECT ≥ 70 °C |
| 32 | Evap System Vapour Pressure | 0, or the Mode 08 leak test pressure curve |
| 3C–3F | Catalyst Temperature B1S1, B2S1, B1S2, B2S2 | Follows 350 °C + 4.5 °C per % load (20 s lag) |
| 61 / 62 | Driver's Demand / Actual Engine Torque | Derived from load |
| 63 | Engine Reference Torque | `REFERENCE_TORQUE_NM` |
| 67 | Engine Coolant Temperature (sensor 1) | Engine coolant temperature |
| 68 | Intake Air Temperature (sensor 1) | Intake air temperature |

PIDs 14/18 (narrowband upstream) and 22 (port-injection rail pressure) are not offered, because this engine is modelled with wideband upstream sensors and direct injection; a real car reports one or the other. PID 5F (emission requirements) is not offered: it is a bit-encoded value whose code list is only published in the SAE J1979 digital annex.

The transmission ECU (`FEATURE_TCM`) answers PIDs 01 (its own MIL and DTC count), 0D (vehicle speed), 1C and A4 (byte A bit 1 = supported, bytes C–D = actual gear ratio × 1000, from `GEAR`; the gear number is also put in the upper half of byte B, a position public PID tables do not document).

### PID Support Information

The simulator responds to the PID availability requests 00, 20, 40, 60, 80 and A0 (supported PIDs 01–C0) for each ECU. These responses allow diagnostic software to identify which PIDs each ECU advertises. They are generated from the implemented PIDs, so they stay correct when PIDs are added.

### Supported Services

| Service | Description |
|---|---|
| 01 | Current data (PIDs above) |
| 02 | Freeze frame 0: the highest-priority stored snapshot (misfire and fuel system DTCs first, otherwise the oldest). PID 02 returns its DTC (0000 if none). Snapshot PIDs: 03–09, 0B–11, 24, 2F, 33, 42, 44, 46, 5C |
| 03 / 07 / 0A | Confirmed, pending and permanent DTCs of the ECU addressed |
| 04 | Clear DTCs, freeze frames and "since cleared" counters (PIDs 21, 30, 31, 4D, 4E). Refused with `7F 04 22` while the engine runs (RPM ≥ 300). Permanent DTCs stay |
| 06 | MID 01 (O2 sensor B1S1, test IDs 01/02, UASID 0A). With `FEATURE_MODE06_EXT`: MID 21 catalyst (fails while P0420 is active), 31 EGR (P0401), 3C EVAP 0.020" leak (result of the Mode 08 test; all zero until it has run), A1 misfire all cylinders and A2–A7 cylinders 1–6 (test IDs 0B/0C, UASID 24 counts; P0300 adds counts on all cylinders, P030n on cylinder n). Catalyst, EGR and EVAP use manufacturer test ID 80 |
| 08 | `FEATURE_MODE08`: TID 01 EVAP leak test. Refused with `7F 08 22` while the vehicle moves. Pulls vacuum for 3 s and holds it until 10 s; P0456 or P0442 as an active fault makes the pressure rise and the test fail |
| 09 | InfoTypes 00 (supported), 02 VIN, 04 calibration ID, 06 calibration verification number (CRC32 of the calibration ID), 08 in-use performance tracking (`FEATURE_IPT`, 16 counters), 0A ECU name, 0D engine serial number, and message counts 01/03/09 for older tools. An ECU name such as `ECM-EngineControl` is sent in J1979 layout: acronym padded to 4 bytes, `-`, then the name. The transmission ECU answers 00, 01, 02, 09 and 0A |

With `FEATURE_UDS` (default on), the ECUs also answer minimal OBD on UDS (SAE J1979-2 / ISO 14229):

| Service | Description |
|---|---|
| 10 | Diagnostic session control: 01 default, 03 extended (back to default after 5 s without requests). Reply carries P2 = 50 ms and P2* = 5000 ms |
| 3E | Tester present (subfunction 00; 80 suppresses the reply) |
| 22 | Read data by identifier, several DIDs per request: F4xx = Mode 01 PID xx, F6xx = Mode 06 MID xx, F8xx = Mode 09 InfoType xx, F190 = VIN |
| 19 | Read DTC information: 01 number of DTCs by status mask, 02 DTCs by status mask (3-byte DTCs + ISO 14229 status byte, format identifier 04), 04 freeze frame by DTC (as F4xx DIDs), 55 permanent DTCs |
| 14 | Clear diagnostic information for group FFFFFF or FFFF33. Refused with NRC 22 while the engine runs |

Requests longer than one frame (for example a 22 request with several DIDs) are received with ISO-TP multi-frame reception (physical requests only). Negative responses: 11 unknown service, 12 unsupported subfunction/PID/InfoType, 13 wrong length, 22 conditions not correct, 31 out of range, 78 response pending. For functional (broadcast) requests, 11, 12 and 31 are not sent.

Responses longer than one CAN frame use ISO-TP (First Frame, Flow Control, Consecutive Frames). The firmware honours the tester's block size, separation time (STmin) and WAIT frames, and aborts if no Flow Control arrives within 1 second. Requests that arrive while an ECU is busy (waiting for Flow Control, simulating a response delay) are queued (4 frames) and answered afterwards. Serial commands keep being processed meanwhile.

The `RESPDELAY` setting simulates a slow ECU: above 40 ms the ECU first answers "response pending" (`7F <service> 78`), repeated every 4 s, then the real response. On a functional request every answering ECU sends its "response pending" first and they share the wait. Clear requests (04, UDS 14) always get one "response pending" first, as many real ECUs do.

The MCP2515 hardware filters accept only the request IDs of the active mode. If the MCP2515 fails to start, the firmware reports an error and retries every 2 seconds while serial commands keep working. With the ignition off (`IGN:0`) the simulator does not answer any CAN request and the run-time and distance counters stop.

| CAN mode | Identifiers | Bitrate | Functional request | Engine ECU request / response | Transmission ECU request / response |
|---|---|---|---|---|---|
| 6 (default) | 11-bit | 500 kbit/s | 7DF | 7E0 / 7E8 | 7E1 / 7E9 |
| 7 | 29-bit | 500 kbit/s | 18DB33F1 | 18DA10F1 / 18DAF110 | 18DA18F1 / 18DAF118 |
| 8 | 11-bit | 250 kbit/s | 7DF | 7E0 / 7E8 | 7E1 / 7E9 |
| 9 | 29-bit | 250 kbit/s | 18DB33F1 | 18DA10F1 / 18DAF110 | 18DA18F1 / 18DAF118 |

A functional request is answered by every ECU, the engine ECU first. The mode numbers match ELM327 protocols (`ATSP6` to `ATSP9`). The selected mode is saved in EEPROM and used when the Arduino runs without the application. When the application connects, it sets the mode selected in its CAN protocol dropdown.

### DTC Lifecycle

DTCs behave like on a real OBD-II ECU:

- A fault is reported with `FAULT:<code>:1` (or `2` for a severe misfire, which makes the MIL flash). The DTC becomes **pending** at once, and a freeze frame is captured.
- A **drive cycle** ends when the ignition goes off after the engine ran at least `CYCLE_MIN_RUN_S`, or with `CYCLE:1` (only while the ignition is on; each cycle counts once). A fault still present when a new cycle starts fails again in that cycle. A cycle without a failure report counts as passing.
- Live fault states are not kept across power-up: after a restart, faults count as gone until they are reported again.
- A DTC that failed in two consecutive drive cycles becomes **confirmed**: the MIL turns on and a **permanent** DTC is set.
- A drive cycle without the fault clears pending. After three passing cycles the MIL goes off and the permanent DTC is erased.
- After the MIL is off, the confirmed DTC is erased after 40 warm-ups without the fault.
- Mode 04 / UDS 14 clear confirmed and pending DTCs and freeze frames. Permanent DTCs stay until a passing drive cycle.
- Codes P0700–P0999 belong to the transmission ECU.

`DTC:ADD/DEL/SET` set the lists directly, for testing without waiting for drive cycles.

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
| MEASURED_AFR | 24, 28 | AFR | 8 – 20 | 1 | 14.7 |
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
| O2V | 24, 28, Mode 06 | V | 0 – 5 | 3 | 0.45 |
| FUELSYS | 03 | code | 0, 1, 2, 4, 8, 16 | 0 | 2 |
| STFT2 | 08 | % | -100 – 99.2 | 1 | 2.5 |
| LTFT2 | 09 | % | -100 – 99.2 | 1 | -1.8 |
| RAILP | 23, 59 | kPa | 0 – 60000 | 0 | 5000 |
| GEAR | A4 (transmission ECU) | gear | 0 – 8 | 0 | 0 |

Values outside the range are limited to it, and values are rounded to the listed decimals. The echo in the reply shows the result.

Other commands:

| Command | Description |
|---|---|
| `CAN:6` … `CAN:9` | Select the CAN mode (see table above) |
| `IGN:0` / `IGN:1` | Ignition off (no CAN replies; may complete a drive cycle) / on |
| `LOG:0` / `LOG:1` / `LOG:2` | Serial log: errors only / OBD requests and replies / every CAN frame |
| `TCM:0` / `TCM:1` | Transmission ECU absent / present (stored in EEPROM) |
| `RESPDELAY:<0–5000>` | Simulated response time in ms |
| `FAULT:P0171:1` | Live test result: `0` passed, `1` failed, `2` severe (flashing MIL) |
| `CYCLE:1` | End the current drive cycle now |
| `DTC:ADD:S:P0171`, `DTC:DEL:S:P0171` | Add or remove a DTC directly. List type `S` confirmed (stored), `P` pending, `X` permanent |
| `DTC:SET:S:P0171,P0300` | Replace a DTC list (empty list clears it) |
| `DTC:CLEAR`, `DTC:CLEAR:ALL` | Clear like Mode 04 / also clear permanent DTCs |
| `VIN:<17 chars>`, `CALID:<text>`, `ECUNAME:<text>`, `ESN:<text>` | Mode 09 vehicle information (stored in EEPROM) |
| `ODO:<km>` | Set the odometer (one decimal) |
| `FACTORY:1` | Reset everything stored in EEPROM to the defaults |
| `INFO`, `GET`, `DTC?` | Read back firmware status, all parameter values, DTCs |

Several commands can be sent on one line (up to 200 characters), separated by `;`. A line can start with a sequence number (`#12 RPM:850;MAP:35`). The Arduino answers every line with `OK #12 RPM=850;MAP=35`, echoing the values it actually applied (after range limits and rounding), preceded by `ERR #12 <reason> <name>` for each rejected command. A line that cannot be used at all gets a single `ERR #12 <reason>` and no `OK`. The application uses these replies to confirm every value.

| Error reason | Meaning |
|---|---|
| `BAD_FORMAT` | Command without `:` |
| `BAD_VALUE` | Value not a number, out of the allowed set, or invalid text/VIN/DTC |
| `UNKNOWN` | Unknown command name |
| `LIST_FULL` | All `MAX_DTCS` DTC records are in use |
| `CAN_INIT_FAILED` | MCP2515 did not start in the requested CAN mode (retried every 2 s) |
| `IGN_OFF` | `CYCLE:1` without an open drive cycle (ignition off) |
| `BAD_SEQUENCE`, `EMPTY`, `LINE_TOO_LONG` | Whole line rejected |

Query replies:

- `INFO` → `OK FW=3.0.0;CAN=6;CANOK=1;IGN=1;LOG=1;UP=<ms since boot>;SEQ=<last sequence number>;MIL=<0 off, 1 on, 2 flashing>;LOGDROP=<skipped log lines>;TCM=1;RESPDELAY=0;ODO=<km>;CYCLE=<drive cycles since boot>;VIN=…;CALID=…;ECUNAME=…;ESN=…`
- `GET` → `OK RPM=850;MAP=85;…` (all parameters)
- `DTC?` → `OK DTC=S:P0171|P:P0171|X:P0171;DTCX=P0171:AF:2:0:0:1:E;FF=P0171`
  - `DTC=` lists confirmed (`S`), pending (`P`) and permanent (`X`) DTCs of both ECUs.
  - `DTCX=` gives each DTC record as `<code>:<ISO 14229 status byte, hex>:<failed cycles>:<passed cycles>:<warm-ups since MIL off>:<permanent 0/1>:<E engine | T transmission>`.
  - `FF` is the DTC of the Mode 02 freeze frame (`-` if none).

The Arduino also prints lines on its own:
- `READY` after booting.
- `CAN READY | MODE …` when CAN starts.
- `OBD REQ PHYS|FUNC ECM|TCM …` / `OBD RSP ECM|TCM …` at log level 1 or higher.
- `CAN RX …` / `CAN TX …` at log level 2.
- `EVENT DTC CLEARED BY TESTER` after a scan tool sends Mode 04 or UDS 14.
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
- Connection handshake: after opening the port, the application waits up to 4 seconds for the Arduino's `READY` line or an `INFO` reply. It then reads the DTCs, vehicle information and TCM setting stored in the Arduino's EEPROM, and sends its settings and values.
- Confirmation of every value:
  - Each parameter tab shows ✓ confirmed, … waiting, ⚠ limited by the firmware range, or ✗ not confirmed, with the last confirmed value.
  - A value without a reply within 300 ms is resent, up to 3 times.
  - A read-back every 2 seconds (`INFO`, `GET`, `DTC?`) resends any value the Arduino does not hold. Values stored on the Arduino (DTCs, vehicle information, TCM) are taken from the Arduino instead, so connecting never overwrites them and a scan tool's clear is shown.
  - An Arduino restart (`READY`, or uptime going back) is detected, and all values are restored.
  - The link status shows reply age, round-trip time, retries and failures.
- Diagnostics tab:
  - Confirmed, pending and permanent DTC lists: add, remove, "Clear stored + pending" (same as a scan tool's Mode 04) and "Clear all".
  - The MIL state (off, on, flashing) and the freeze frame DTC reported by the Arduino.
  - DTC status table: for each DTC its ECU, ISO 14229 status, live fault, pending, confirmed, MIL, permanent, failed and passed drive cycles and warm-ups.
  - Fault injection: report a fault as present (failing or severe) or gone, and "End drive cycle", to walk a DTC through its lifecycle.
- Vehicle & ECU tab:
  - VIN (validated: 17 characters, no I, O or Q), calibration ID, ECU name and engine serial number.
  - Odometer display and "Set odometer".
  - Transmission ECU on/off and response delay.
  - "Factory reset ECU memory" (with confirmation).
- Driving simulation with different tuning modes (Normal, Lean, Rich, Bad, Severe Misfire):
  - It continues from the current temperatures and fuel level.
  - MAF and load are calculated from RPM, MAP and intake temperature (2.0 L speed-density model).
  - Coolant warms up towards 90 °C, oil follows coolant, and intake temperature falls with speed.
  - Fuel level drains with fuel used, battery voltage is 14.2 V while running, and O2 voltage follows the measured AFR.
  - Fuel system status changes: open loop when cold (below 40 °C), at wide-open throttle and on overrun fuel cut; closed loop otherwise.
  - Bank 2 trims follow bank 1, fuel rail pressure rises with load (4–15 MPa) and the gear follows vehicle speed.
- Optional auto faults during simulation: the simulation reports faults and the Arduino runs the DTC lifecycle.
  - Total fuel trim above +20 % (lean) or below -20 % (rich) for 5 seconds in closed loop reports P0171 or P0172 (gone again after 5 seconds within limits).
  - Bad Tune reports P0300; Severe Misfire reports P0302 with a flashing MIL.
  - Faults the simulation reported end when it stops. Clearing DTCs restarts the detection.
  - Every known fault state (present or gone) is re-sent every 2 seconds, so a report lost to a disconnect or an Arduino restart is corrected. The Arduino forgets live fault states at power-up and only saves real changes to EEPROM.
- Optional linking of MAF and load to RPM, MAP and IAT in manual mode (on by default).
- Simulated engine RPM, MAP, fuel trims (both banks), AFR, throttle position, vehicle speed, engine load, MAF, ignition timing, fuel level, barometric pressure, battery voltage, ambient and oil temperature, O2 sensor voltage, fuel system status, fuel rail pressure and gear.
- Live serial communication log.
- Lambda display for commanded and measured AFR.

### Project Structure

- `OBD2EngineSimulator.slnx` – Visual Studio solution.
- `OBD2EngineSimulator/OBD2EngineSimulator.csproj` – C# project file.
- `OBD2EngineSimulator/Form1.cs` – Main interface logic.
- `OBD2EngineSimulator/ArduinoLink.cs` – Serial link: batching, sequence numbers, confirmation, retries, read-back.
- `OBD2EngineSimulator/EngineModel.cs` – Driving simulation and automatic faults.
- `OBD2EngineSimulator/ParameterDefinitions.cs` – Parameter names, ranges and precision.
- `OBD2EngineSimulator/Form1.Designer.cs` – Windows Forms designer file.
- `OBD2EngineSimulator/Form1.resx` – Form resources.
- `OBD2EngineSimulator/Program.cs` – Application entry point.
- `Arduino-Firmware/OBD2EngineSimulator/` – Arduino sketch:
  - `OBD2EngineSimulator.ino` – setup and main loop.
  - `Config.h` – hardware, CAN, vehicle, drive-cycle settings and feature switches.
  - `EngineState` – engine values, DTC lifecycle, freeze frames, counters.
  - `Persist` – EEPROM storage.
  - `CanLink` – CAN modes, filters, ISO-TP, request queue.
  - `ObdService` – OBD services 01–0A and request dispatch.
  - `UdsService` – minimal OBD on UDS.
  - `SerialCommands` – USB serial protocol.

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

Walking a DTC through its lifecycle from a serial terminal:

```text
FAULT:P0420:1        P0420 pending, freeze frame captured
CYCLE:1              first failing drive cycle
CYCLE:1              second failing cycle: confirmed, MIL on, permanent
FAULT:P0420:0        fault gone
CYCLE:1              three passing cycles turn the MIL off
CYCLE:1
CYCLE:1
DTC?
```

## Extending the Simulator

The Arduino firmware is designed to allow additional PID responses to be implemented.

PID processing is handled inside the `EncodeEcmPid()` function (and `EncodeTcmPid()` for the transmission ECU) in `ObdService.cpp` using a C++ switch statement. Values are fixed-point integers scaled by 10^decimals (`V(P_RPM)` is the RPM, `V(P_STFT)` the trim in tenths of a percent).

Developers can extend the simulator by:

1. Adding a new PID case to `EncodeEcmPid()`.
2. Reading or calculating the required engine parameter (new parameters go in `PARAM_DEFS` in `EngineState.cpp` and `ParameterDefinitions.cs`, with the same decimals).
3. Converting the value into the appropriate OBD-II response format.
4. Setting the correct response data length.

The supported PID bitmaps are generated from the encoders, so diagnostic software discovers the new PID automatically, also through UDS (DID F4xx). To include a PID in freeze frames, add it to `FREEZE_FRAME_PIDS` (snapshots hold up to 32 bytes). Mode 06 MIDs and Mode 09 InfoTypes work the same way in `EncodeMid()` and `EncodeInfoType()`.

The existing `IsoTpSend()` function handles CAN response transmission, including multi-frame responses, meaning additional PIDs can reuse the existing communication infrastructure.

Developers should consult the relevant OBD-II PID definitions for the correct response formulas, scaling and data formats.

## Future Development

- Additional standard OBD-II PIDs and Mode 06 monitors.
- Broader UDS support (security access, routine control).
- Other protocols (K-line, J1850, CAN FD), which need different hardware.
- J2534 Pass-Thru integration and compatibility testing.
<img width="1195" height="896" alt="UNO2" src="https://github.com/user-attachments/assets/b6897a9f-c7d8-421a-967e-31e9032de84b" />
<img width="896" height="1195" alt="UNO" src="https://github.com/user-attachments/assets/b329b943-22cb-433b-b981-d717059fea61" />
<img width="1072" height="833" alt="image" src="https://github.com/user-attachments/assets/4804e5e3-0da0-4f91-bc44-ab0bb5e22622" />
