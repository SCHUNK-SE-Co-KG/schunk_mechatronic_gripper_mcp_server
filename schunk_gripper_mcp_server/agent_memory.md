<!--
  AGENTS.md — Guidance file for AI coding agents (Copilot, etc.)
  Product family: SCHUNK EGU / EGK / EZU mechatronic grippers (SFP platform)

  PURPOSE
  Teach an AI agent how to COMMUNICATE WITH and CONTROL the gripper and what
  FEATURES exist, so it can write application/integration code (Python, C++,
  NVIDIA Omniverse, µ0, ML pipelines) WITHOUT a human first reading the manual.

  SOURCE OF TRUTH (files in ./EGU_EGK_EZU/)
  - Cyclic process data, control/status word, command set, boot sequence:
    "EGU V53X Inbetriebnahmeanleitung für PROFINET.pdf" (firmware 5.3.1).
  - Acyclic parameters (limits, defaults, units): parameters_customer_5.3.X.xlsx.
  - Device descriptions for engineering tools:
    SCHUNK_SFP-5.3.X_*_IODD / _GSDML / _ESI / _EDS.zip.
  - Modbus RTU register map, addressing, endianness, gripping modes (§3a, §2a):
    EGK-Modbus.txt / EGU-Modbus.txt commissioning manuals (firmware 5.3.1),
    both located in the repository root next to this file.
  - Firmware version this file documents: 5.3.1.117131 (fieldbus doc);
    Modbus RTU manuals documented: firmware 5.3.1.
  Values below were extracted from those files. If anything conflicts, the files win.

  MAINTENANCE
  - Remaining `TODO:` items: µ0/SDK integration details.
  - When firmware changes, re-export the files and update version + values here.
-->

# AGENTS.md — SCHUNK EGU / EGK / EZU mechatronic grippers

## 1. What these grippers are

The SCHUNK **EGU**, **EGK** and **EZU** are electrically actuated **mechatronic
gripping systems** with an integrated controller, built on the shared **SFP
platform** (firmware 5.3.1). They are commanded over an industrial fieldbus.
**All variants and sizes share the same cyclic process-data frame, the same
control/status word, the same command set and the same parameter model** — only
the mechanical envelope (stroke, force, finger count, grip behavior) differs.

- **EGU** — 2-finger parallel gripper, strong-grip.
- **EGK** — 2-finger parallel gripper, soft-grip (sensitive). 
- **EZU** — 3-finger centric gripper, strong-grip.

Module option letters seen in the device descriptions: `M` = with **GPE** (grip
force & position maintenance, the gripper is equipped with a brake which is engaged to maintain the grip force and position), `N` = without GPE; `B` = base, `SD` = reduced
stroke / different drive variant; `IL` = IO-Link, `PN` = PROFINET, `EC` = EtherCAT.

<!--
  2. STROKE PER SIZE
  Pulled from parameters_customer_5.3.X.xlsx (param 0x0608 max_pos, in mm).
  These are the total jaw-to-jaw stroke maxima used to validate target positions.
  Absolute force limits in Newton per size: see 0x0658/0x0660/0x06A8 in §8
  (values are read from the connected gripper at runtime, not hardcoded here).
-->
## 2. Sizes & stroke (max position, mm)

| Variant | Sizes (stroke in mm) |
|---------|----------------------|
| **EGU** | 50 → 102, 60 → 120, 70 → 140, 80 → 160 |
| **EGU (SD)** | 50 → 82, 60 → 100, 70 → 120, 80 → 140 |
| **EZU** | 30 → 60, 35 → 70, 40 → 80 |
| **EZU (SD)** | 30 → 40, 35 → 50, 40 → 60 |
| **EGK** | 25 → 53, 40 → 83, 50 → 103 |

> Position is given as the **total jaw-to-jaw opening**, value `0` = zero point (closed reference).
> Always validate target position against the size's max stroke before sending.

<!--
  2a. GRIPPING MODES — from EGK/EGU Modbus commissioning manuals §1.2.3.
  Mode is not a separate command bit; it is selected implicitly by the
  gripping-velocity and gripping-force values sent with `grip workpiece` /
  `grip workpiece at expected position`.
-->
## 2a. Gripping modes

| Mode | Variants | Force range | Velocity field | GPE |
|------|----------|-------------|-----------------|-----|
| **BasicGrip** | all | 50–100 % of `max_grp_force` | EGK: must be `0` µm/s (module derives velocity from force). EGU/EZU: no velocity field used. | optional (M modules) |
| **SoftGrip** | EGK only | 50–100 % | `min_vel` ≤ v ≤ (force% × `max_grp_vel`) — gentler than BasicGrip at the same force | optional (M modules) |
| **StrongGrip** | EGU/EZU **M (GPE) modules only** | 101–200 % (EGU 50/60/80/EZU) or 101–150 % (EGU 70) | always uses `max_grp_vel` internally, no field | **mandatory** — GPE auto-activates; re-grip capped at 2 s before GPE takes over |

> EGK never supports StrongGrip (max force is always 100 %). EGU/EZU never
> support SoftGrip. Requesting StrongGrip force (>100 %) on an `N` (no-GPE)
> module, or on EGK, is rejected as `WRN_NOT_FEASIBLE` / additional code
> `0x29` (force out of range).

## 3. Communication interfaces

All transports expose the **same logical data model**; only addressing differs.
Device-description files (in `./EGU_EGK_EZU/`) are imported into the PLC/master
engineering tool, not parsed at runtime:

- **IO-Link** — IODD: `SCHUNK-EGU_EGK_EZU-…-IODD1.1.xml` (in `*_IO-Link_IODD.zip`).
- **PROFINET** — GSDML: `GSDML-V2.45-SCHUNK-EGU-EGK-EZU-….xml` (in `*_Profinet_GSDML.zip`).
- **EtherCAT** — ESI: one XML per size (in `*_EtherCAT_ESI.zip`).
- **EtherNet/IP** — EDS (in `*_EtherNetIP_EDS.zip`).
- **Modbus RTU** — no device-description file; addressed directly by register
  number (see §3a). Covered by the EGK/EGU Modbus RTU commissioning manuals
  (firmware 5.3.1). EZU is not separately documented but shares the SFP
  platform and behaves like EGU (StrongGrip, no SoftGrip).

PROFINET supports RT and IRT (min cycle 0.25 ms). **If fieldbus communication is
lost, the module performs a fast stop and reports `ERR_COMM_LOST`.**

<!--
  3a. MODBUS RTU — from the EGK/EGU "Commissioning Manual, Firmware 5.3.1"
  (Modbus RTU interface). This is the transport the MCP server's `serial_port`
  / `device_id` tools (scan_grippers, add_gripper, locate_gripper) target.
-->
## 3a. Modbus RTU specifics

- **Physical layer:** RS-485 half-duplex, 2-wire. Frame: 8 data bits, even
  parity, 1 stop bit (8E1); no flow control.
- **Baud rate:** default **115200** bit/s; other allowed values are 19200,
  230400, 460800, 921600. Set via parameter `baudrate` (0x11A0); a change only
  takes effect after a restart (softreset).
- **Slave ID:** default **12** (0x0C decimal); valid range 1–247. Set via
  parameter `slave_id` (0x11A8). Must be unique per RS-485 bus.
- **Function codes:** `0x03` Read Holding Registers (read parameters),
  `0x10` Write Multiple Registers (write parameters). No other function codes
  are supported.
- **Register addressing:** register addresses **equal the hex parameter code**
  from §8 (e.g. `plc_sync_input` = 0x0040 = register 64). Modbus RTU has no
  cyclic/acyclic distinction — the cyclic frame is just the parameters
  `plc_sync_output` (0x0048, read+write, control word/position/velocity/force)
  and `plc_sync_input` (0x0040, read-only, status word/position/diag), polled
  repeatedly. All other §8 parameters are read/written the same way.
- **Register size:** every register is 16 bits. A 4-byte `float`/`int32`
  parameter = 2 registers; `plc_sync_output`/`plc_sync_input` (16 bytes) = 8
  registers. Reads/writes must cover a parameter's registers completely —
  partial/array-element access is not possible.
- **Endianness — the #1 gotcha:** Modbus *protocol* fields (register address,
  quantity, byte count) are transmitted **big-endian**. Parameter *payload*
  values (control/status word, position, velocity, force, floats, etc.) are
  transmitted **little-endian**. Don't mix these up when encoding/decoding raw
  register payloads.
- **On-the-wire register address is off by one:** the address byte pair sent
  in the request equals `code_in_registers - 1` (Modbus convention); many
  master stacks add this offset automatically, some don't — verify against the
  worked examples below before assuming.
- **Data integrity:** every frame ends with a 16-bit CRC (Modbus standard
  Appendix B algorithm).
- **Worked example — reading `plc_sync_input` (slave 12):**
  Request `0C 03 00 3F 00 08 75 1D` (func 03, start reg 0x003F i.e. 0x0040-1,
  quantity 8, CRC). Response `0C 03 10 <16 bytes little-endian payload> <CRC>`
  where the payload is status word (4B) + actual position µm (4B) +
  reserved (4B) + diagnosis word (4B), byte 13=warning/error code, byte
  12=additional_code, both little-endian within the diagnosis double word.
- **Worked example — writing `plc_sync_output` (slave 12):**
  Request `0C 10 00 47 00 08 10 <16 bytes little-endian payload> <CRC>` (func
  16, start reg 0x0047 i.e. 0x0048-1, quantity 8, byte count 0x10) where the
  payload is control word (4B) + target position µm (4B) + target velocity
  µm/s (4B) + gripping force % (4B), all little-endian. Response echoes
  function code, register address and quantity (no payload).
- **Factory reset via Modbus:** write `UINT16` value `0x8C` (140) to register
  `0x0100` (256) while in error state; wait for `ready for shutdown` (status
  bit 2) before power-cycling or restarting.

<!--
  4. CYCLIC PROCESS DATA — the heart of runtime control.
  Fixed 16-byte (4 double-word) frame in BOTH directions. Source: manual §2.1.1.
  Multi-byte numeric fields are signed 32-bit. Confirm byte/word order for your
  transport in §11 before coding (PROFINET = big-endian).
-->
## 4. Cyclic process data (fixed 16-byte frame each direction)

**Outputs — PLC → gripper (commands):**

| Bytes | Field | Type | Unit / scaling |
|-------|-------|------|----------------|
| 0–3   | **Control double word** | bitfield (uint32) | see §5 |
| 4–7   | Target position | int32 | µm (1000 µm = 1 mm) |
| 8–11  | Target velocity | int32 | µm/s (1000 µm/s = 1 mm/s) |
| 12–15 | Gripping force | int32 | % of `max_grp_force` |

**Inputs — gripper → PLC (feedback):**

| Bytes | Field | Type | Unit / scaling |
|-------|-------|------|----------------|
| 0–3   | **Status double word** | bitfield (uint32) | see §6 |
| 4–7   | Actual position | int32 | µm |
| 8–11  | Reserved | — | — |
| 12–15 | **Diagnosis double word** | uint32 | byte 13 = warning/error code, byte 12 = additional_code |

<!--
  5. CONTROL DOUBLE WORD — exact bit positions from manual §2.1.1.1 / §7.2.
  Bit 0 = LSB. Reserved bits must be 0. Motion is triggered together with the
  toggle handshake in §7.
-->
## 5. Control double word (bit → meaning)

| Bit | Name (long) | Short | Purpose |
|-----|-------------|-------|---------|
| 0  | fast stop | fast stop | Immediate stop; held at boot |
| 1  | stop | stop | Controlled stop |
| 2  | acknowledge | ack | Acknowledge / reset errors |
| 3  | prepare for shutdown | prep shutdown | Persist data, prepare power-off |
| 4  | softreset | softreset | Software restart |
| 5  | release for manual movement | release manual movement | Allow manual jog of fingers |
| 6  | repeat command toggle | rpt cmd tgl | Re-issue the same command (toggle) |
| 7  | grip direction | grip dir | Grip inward vs. outward |
| 8  | jog mode negative | jog − | Jog toward 0 |
| 9  | jog mode positive | jog + | Jog away from 0 |
| 10 | reserved | — | must be 0 |
| 11 | release workpiece | release wp | Release a gripped workpiece |
| 12 | grip workpiece | grp wp | Grip with set force/direction |
| 13 | move to absolute position | pos absolute | Position to target (abs) |
| 14 | move to relative position | pos relative | Position by delta (rel) |
| 15 | reserved | — | must be 0 |
| 16 | grip workpiece at expected position | grp wp at pos | Grip only at expected position |
| 17–29 | reserved | — | must be 0 |
| 30 | brake test | brake test | Trigger brake test |
| 31 | activate grip force & position maintenance | activate GPE | `M` modules only; must be 0 on `N` |

> On `N` (no-GPE) modules, **bit 31 must always be 0**; setting it returns
> `not feasible` + `WRN_NOT_FEASIBLE`.

<!--
  6. STATUS DOUBLE WORD — exact bit positions from manual §2.1.1.2 / §7.3.
-->
## 6. Status double word (bit → meaning)

| Bit | Name (long) | Short | Meaning |
|-----|-------------|-------|---------|
| 0  | ready for operation | ready for op | Module operational |
| 1  | control authority fieldbus | ctrl authority fb | Fieldbus has control |
| 2  | ready for shutdown | ready for sd | Safe to power off |
| 3  | not feasible | not feasible | Last command rejected |
| 4  | command successfully processed | cmd success | Command completed OK |
| 5  | command received toggle | cmd rcvd tgl | Toggles when a command is received |
| 6  | warning | warning | Warning present (see diag word) |
| 7  | error | error | Error present (see diag word) |
| 8  | released for manual movement | manual movement released | Manual jog active |
| 9  | software limit reached | softlimit reached | Hit a configured soft limit |
| 10 | reserved | — | — |
| 11 | no workpiece detected | no wp detected | Grip found no workpiece |
| 12 | workpiece gripped | wp gripped | Workpiece held |
| 13 | position reached | pos reached | Target position reached |
| 14 | workpiece pre-grip started | wp pre-grip started | Re-grip / pre-grip running |
| 15 | reserved | — | — |
| 16 | workpiece lost | wp lost | Workpiece lost during hold |
| 17 | wrong workpiece gripped | wrong wp gripped | Size outside expected window |
| 18–30 | reserved | — | — |
| 31 | grip force & position maintenance activated | GPE activated | GPE currently active |

<!--
  7. COMMAND HANDSHAKE & BOOT SEQUENCE — manual §3.1.1 / §2.1.1.
  This is the #1 thing an agent must get right. Commands use a toggle handshake.
-->
## 7. Command handshake & boot sequence

**Handshake (every command):**
1. Set the desired motion bit(s) + payload (position/velocity/force) in the
   cyclic outputs and keep transmitting cyclically.
2. The module flips **`command received toggle` (status bit 5)** as soon as it
   accepts the request — this only confirms *receipt*, not success.
3. Wait for **`command successfully processed` (bit 4)** for completion, or
   **`not feasible` (bit 3)** for rejection (read the diagnosis word for the code).
4. To repeat an identical command, flip **`repeat command toggle` (control bit 6)**.

**Boot → ready (required before any motion):**
```text
At boot the module is in the ERROR state (error=1, error code 0xD9).
PLC sends cyclically:  fast stop (bit0)=1  AND  acknowledge (bit2)=1
Module responds:       command received toggle flips, then ready for operation (status bit0)=1
=> Module is now operational.
```
> During boot, transmit all control bits = 0 first to avoid unintended commands;
> the module internally forces control bits to 1 at boot for safety.

<!--
  8. CUSTOMER PARAMETERS (acyclic) — from parameters_customer_5.3.X.xlsx.
  These are configuration parameters (not the cyclic frame). Same codes for all
  sizes; ranges/defaults that vary by size are noted. Access acyclically
  (IO-Link ISDU / PROFINET record / EtherCAT SDO) by the hex CODE.
-->
## 8. Customer parameters (acyclic, by hex code)

| Code | Name | Type | Unit | Range | Default | Notes |
|------|------|------|------|-------|---------|-------|
| 0x0380 | grp_prehold_time | uint16 | ms | 0 … 60000 | 0 | |
| 0x0528 | wp_lost_dst | float | mm | 0.1 … 50 | 2 | Workpiece-lost distance |
| 0x0540 | wp_release_delta | float | mm | 1 … 50 | 5 | Release travel |
| 0x0580 | grp_pos_margin | float | mm | 1 … 10 | 2 | Grip position tolerance |
| 0x05A8 | grp_prepos_delta | float | mm | 1 … 50 | 5 | Pre-position delta |
| 0x0600 | min_pos | float | mm | 0 … max stroke | 0 | Lower soft limit (size-dependent max) |
| 0x0608 | max_pos | float | mm | 0 … max stroke | = stroke | Upper soft limit (see §2) |
| 0x0610 | zero_pos_ofs | float | mm | −10000 … 10000 | 0 | Zero-point offset |
| 0x0880 | min_wrn_mot_volt | float | V | 19 … 36 | 21.6 | Motor undervoltage warn |
| 0x0888 | max_wrn_mot_volt | float | V | 19 … 36 | 26.4 | Motor overvoltage warn |
| 0x0890 | min_wrn_lgc_volt | float | V | 11 … 48 | 21.6 | Logic undervoltage warn |
| 0x0898 | max_wrn_lgc_volt | float | V | 11 … 48 | 26.4 | Logic overvoltage warn |
| 0x0628 | min_vel | float | mm/s | read-only, size-dependent | — | e.g. EGU/EZU 10; EGK 5–6.25 |
| 0x0630 | max_vel | float | mm/s | read-only, size-dependent | — | Max positioning speed |
| 0x0650 | max_grp_vel | float | mm/s | read-only, size-dependent | — | e.g. EGU/EZU 25; EGK 20–25 |
| 0x0658 | min_grp_force | float | N | read-only, size-dependent | — | |
| 0x0660 | max_grp_force | float | N | read-only, size-dependent | — | 100 % reference for force % |
| 0x06A8 | max_allow_force | float | N | read-only | — | **EGU/EZU only** — StrongGrip force ceiling |
| 0x11A0 | baudrate | uint32 | bit/s | 19200/115200/230400/460800/921600 | 115200 | Modbus RTU only; needs restart to apply |
| 0x11A8 | slave_id | uint8 | — | 1 … 247 | 12 | Modbus RTU only |

- **Gripping force** is commanded as a **percentage of `max_grp_force`** (read
  from the connected gripper via §8, not hardcoded) — always call
  `get_gripper_specification` / read `max_grp_force` before validating force %.
- **Velocity limits** (`min_vel`/`max_vel`/`max_grp_vel`) are size-dependent and
  read-only — read them per gripper rather than assuming a fixed number.

## 9. Command catalog (features)

| Feature | Control bit(s) | Inputs | Completion / feedback |
|---------|----------------|--------|-----------------------|
| Acknowledge / reset error | bit 2 | — | error (bit7)=0, ready for operation (bit0)=1 |
| Move to absolute position | bit 13 | target position, velocity | position reached (bit13) |
| Move to relative position | bit 14 | delta position, velocity | position reached (bit13) |
| Grip workpiece | bit 12 (+ bit 7 direction) | force % (+ velocity for EGK SoftGrip, see §2a) | workpiece gripped (bit12) / no workpiece (bit11) |
| Grip at expected position | bit 16 | force %, expected pos (+ velocity for EGK SoftGrip) | workpiece gripped / wrong workpiece (bit17) |
| Release workpiece | bit 11 | — | (released; check no wp detected) |
| Jog +/− | bit 9 / bit 8 | velocity | position feedback |
| Stop / fast stop | bit 1 / bit 0 | — | motion halted |
| Brake test | bit 30 | — | command successfully processed |
| Activate GPE (M modules) | bit 31 | — | GPE activated (bit31) |

## 10. Typical workflow (pick)

```text
1. Boot:   set fast stop + acknowledge -> wait ready for operation (bit0)=1
2. (Re)acknowledge any error -> wait error (bit7)=0
3. Move to pre-grip pos: set target position + velocity, set bit13
   -> wait command received toggle flips -> wait position reached (bit13)
4. Grip: set force %, set grip direction (bit7), set grip workpiece (bit12)
   -> wait command received toggle -> wait workpiece gripped (bit12)
   -> verify no workpiece (bit11)=0 and wrong workpiece (bit17)=0
5. Move to place position (bit13) -> wait position reached
6. Release: set release workpiece (bit11) -> confirm
```

<!--
  11 collects the silent-bug facts. PROFINET process data is big-endian;
  Modbus RTU parameter payloads are little-endian (§3a) — don't conflate the two.
-->
## 11. Units, conventions & gotchas

- **Positions / velocities:** signed **int32 in µm and µm/s** (1000 = 1 mm or mm/s).
- **Force:** signed int32 as **% of `max_grp_force`**.
- **Endianness:** PROFINET cyclic data is **big-endian**. Confirm word order for
  any other transport before encoding multi-byte values.
- **Diagnosis word:** byte 13 = warning/error code, byte 12 = additional_code.
  Codes are unique across warnings and errors (full list in §12). `0xD9` =
  post-boot error state (cleared by acknowledge). `WRN_NOT_FEASIBLE` (0x94) =
  command rejected; read additional_code for the reason.
- **Keep transmitting** the cyclic frame; loss of comms triggers a fast stop
  (`ERR_COMM_LOST`).
- Always check `command received toggle` (receipt) AND `command successfully
  processed` / `not feasible` (outcome) — never assume completion.
- **Grip force ranges (mode-dependent, see §2a):** BasicGrip/SoftGrip
  `50–100 %` (all variants); StrongGrip `101–200 %` (EGU 50/60/80/EZU) or
  `101–150 %` (EGU 70) — **EGU/EZU M modules only**, never EGK, never `N`
  modules. Out-of-range or wrong-mode force → `WRN_NOT_FEASIBLE` + additional
  code `0x29`.

<!--
  12. DIAGNOSIS CODES — from manual §6 (warnings/errors) and §7.4 (additional
  codes for WRN_NOT_FEASIBLE). Code travels in diagnosis-word byte 13; the
  additional_code (byte 12) only applies to WRN_NOT_FEASIBLE (0x94).
  "Acknowledgeable" = cleared via control bit 2 once the cause is gone;
  "self-clearing" = clears itself when the cause disappears;
  "mandatory ack" = must be acknowledged; some severe errors are non-recoverable.
-->
## 12. Diagnosis codes (diagnosis double word)

**Warnings** (status bit 6 `warning`; all acknowledgeable / self-clearing):

| Hex | Dec | Code | Meaning |
|-----|-----|------|---------|
| 0x90 | 144 | WRN_LGC_TEMP_LO | Logic temperature too low |
| 0x92 | 146 | WRN_MOT_TEMP_LO | Motor temperature too low |
| 0x93 | 147 | WRN_MOT_TEMP_HI | Motor temperature too high |
| 0x94 | 148 | WRN_NOT_FEASIBLE | Command not feasible (see additional_code below) |
| 0x95 | 149 | WRN_POS_LIMIT | Jog ended at min/max position |
| 0x96 | 150 | WRN_LGC_VOLT_LO | Logic supply voltage too low |
| 0x97 | 151 | WRN_LGC_VOLT_HI | Logic supply voltage too high |
| 0x98 | 152 | WRN_MOT_VOLT_LO | Motor supply voltage too low |
| 0x99 | 153 | WRN_MOT_VOLT_HI | Motor supply voltage too high |

**Errors** (status bit 7 `error`; module leaves operation, forced to standstill):

| Hex | Dec | Code | Meaning | Ack |
|-----|-----|------|---------|-----|
| 0x28 | 40  | ERR_BT_FAILED | Brake test failed | mandatory |
| 0x6C | 108 | ERR_MOT_TEMP_LO | Motor temperature too low | mandatory |
| 0x6D | 109 | ERR_MOT_TEMP_HI | Motor temperature too high | mandatory |
| 0x70 | 112 | ERR_LGC_TEMP_LO | Logic temperature too low | mandatory |
| 0x71 | 113 | ERR_LGC_TEMP_HI | Logic temperature too high | mandatory |
| 0x72 | 114 | ERR_LGC_VOLT_LO | Logic supply voltage too low | mandatory |
| 0x73 | 115 | ERR_LGC_VOLT_HI | Logic supply voltage too high | mandatory |
| 0x74 | 116 | ERR_MOT_VOLT_LO | Motor supply voltage too low (not monitored while GPE active) | mandatory |
| 0x75 | 117 | ERR_MOT_VOLT_HI | Motor supply voltage too high (not monitored while GPE active) | mandatory |
| 0xD5 | 213 | ERR_SOFT_LOW | Lower software limit reached/exceeded | mandatory |
| 0xD6 | 214 | ERR_SOFT_HIGH | Upper software limit reached/exceeded | mandatory |
| 0xD9 | 217 | ERR_FAST_STOP | Fast stop triggered (also the post-boot state) | mandatory |
| 0xE4 | 228 | ERR_TOO_FAST | Max velocity exceeded by factor 1.2 | mandatory |
| 0xEF | 239 | ERR_COMM_LOST | Communication with controller/app lost | mandatory |
| 0xF1 | 241 | ERR_MOV_ABORT_TO | Positioning timed out | mandatory |
| 0xF4 | 244 | ERR_MOVE_BLOCKED | Drive blocked | mandatory |

> If an error not in this list occurs, contact SCHUNK service. Some severe errors
> are non-recoverable (module cannot leave the error state).

**Additional codes for `WRN_NOT_FEASIBLE` (0x94)** — diagnosis-word byte 12:

| Hex | Code | Reason |
|-----|------|--------|
| 0x00 | NF_NO_REASON | No warning present |
| 0x01 | NF_IOLINK_FUNCTION_NOT_SUPPORTED | Unsupported acyclic IO-Link function requested |
| 0x02 | NF_IOLINK_RESET_CONDITION_ONLY_ALLOWED_IN_ERROR_STATE | App/Factory/Back-To-Box reset only allowed in error state |
| 0x03 | NF_SHUTDOWN_NOT_FEASIBLE_IN_CURRENT_STATE | Shutdown not allowed from current state |
| 0x04 | NF_RESET_NOT_FEASIBLE_IN_CURRENT_STATE | Restart not allowed from current state |
| 0x05 | NF_COMMAND_NOT_ALLOWED_IN_CURRENT_STATE | Factory reset not allowed from current state |
| 0x06 | NF_COMMAND_NOT_ALLOWED_IN_ERROR_STATE | Function triggered while in error state |
| 0x08 | NF_SOFT_RESET_DISABLED_BY_PARAMETER | Restart disabled via `enable_softreset` |
| 0x09 | NF_CANNOT_TRIGGER_COMMAND_WHILE_FAST_STOP_IS_ACTIVE | `fast stop` (bit 0) was reset when command issued |
| 0x0A | NF_MULTIPLE_COMMANDS_TRIGGERED_SIMULTANEOUSLY | Several commands requested at once |
| 0x0C | NF_COMMAND_NOT_ALLOWED_DURING_BRAKE_TEST | Illegal function during brake test |
| 0x0D | NF_RELEASE_BRAKE_ONLY_ALLOWED_IN_ERROR_STATE | Manual removal only allowed in error state |
| 0x0E | NF_RELEASE_WORKPIECE_COMMAND_ONLY_ALLOWED_WHILE_GRIPPING | Release issued while no workpiece held |
| 0x0F | NF_COMMAND_NOT_ALLOWED_WHILE_HOLDING_A_WORKPIECE | Illegal function while holding a workpiece |
| 0x10 | NF_DESIRED_POSITION_OUT_OF_RANGE | Target position outside limits |
| 0x11 | NF_CURRENT_POSITION_ALREADY_INSIDE_WORKPIECE | Invalid workpiece-position/grip-direction combo |
| 0x12 | NF_COMMAND_NOT_ALLOWED_DURING_GRIPPING_COMMAND | Illegal function during gripping |
| 0x13 | NF_DESIRED_VELOCITY_OUT_OF_RANGE | Velocity outside limits |
| 0x15 | NF_COMMAND_NOT_ALLOWED_DURING_MOVE_TO_POSITION | Illegal function during positioning |
| 0x1D | NF_COMMAND_NOT_ALLOWED_DURING_RELEASE_BRAKE_COMMAND | Illegal function during manual-removal state |
| 0x1E | NF_COMMAND_NOT_ALLOWED_DURING_RELEASE_WORKPIECE_COMMAND | Illegal function while releasing workpiece |
| 0x22 | NF_MINIMUM_POSITION_OUT_OF_RANGE | `min_pos` write out of range |
| 0x23 | NF_MINIMUM_POSITION_ABOVE_MAXIMUM_POSITION | `min_pos` > `max_pos` |
| 0x24 | NF_MAXIMUM_POSITION_OUT_OF_RANGE | `max_pos` write out of range |
| 0x25 | NF_MAXIMUM_POSITION_BELOW_MINIMUM_POSITION | `max_pos` < `min_pos` |
| 0x26 | NF_RELEASE_WORKPIECE_WOULD_VIOLATE_SOFTWARE_LIMIT | Release target outside limits |
| 0x27 | NF_MOVEMENT_INTO_WORKPIECE_NOT_ALLOWED | Release target lies inside the workpiece |
| 0x28 | NF_GPE_FEATURE_NOT_AVAILABLE_ON_GRIPPER_WITHOUT_BRAKE | GPE requested on a module without GPE |
| 0x29 | NF_DESIRED_FORCE_OUT_OF_RANGE | Grip force outside allowed range |

## 13. Safety & critical constraints

- Never command position outside `[min_pos, max_pos]` for the size (§2).
- Never set GPE (control bit 31) on `N` modules.
- Treat `error` (bit7), `workpiece lost` (bit16), `wrong workpiece` (bit17) and
  `no workpiece detected` (bit11) as conditions the agent must surface, not ignore.
- Respect the boot handshake before issuing motion (§7).
- TODO: add applicable safety standards / certification notes.

<!--
  14. INTEGRATION — list the ecosystems and give a minimal correct skeleton.
  Confirm transport + SDK before generating production code.
-->
## 14. Integration frameworks & example

- **Languages:** Python and C++ for application/control code.
- **Target ecosystems:** TODO (NVIDIA Omniverse, µ0, ROS, custom ML pipelines).
- **µ0:** TODO — define this framework / link its SDK.
- **Recommended clients:** PLC/master engineering tool for fieldbus setup using
  the device-description files in §3; for direct access use the vendor SDK or a
  fieldbus stack matching the transport (e.g. an IO-Link master API, a PROFINET
  controller). TODO: confirm the SDK your team standardizes on.

```python
# Illustrative control-frame encoding (PROFINET, big-endian).
# Replace transport read/write with your fieldbus/master API.
import struct

# Control double word bit positions (see §5)
ACK, GRIP_WP, MOVE_ABS, FAST_STOP, GRIP_DIR = 2, 12, 13, 0, 7

def build_outputs(ctrl_bits: int, pos_um: int, vel_um_s: int, force_pct: int) -> bytes:
    return struct.pack(">Iiii", ctrl_bits, pos_um, vel_um_s, force_pct)

def parse_inputs(frame: bytes):
    status, actual_pos_um, _reserved, diag = struct.unpack(">Iiii", frame)
    return {
        "ready":           bool(status & (1 << 0)),
        "not_feasible":    bool(status & (1 << 3)),
        "cmd_done":        bool(status & (1 << 4)),
        "cmd_rcvd_toggle": bool(status & (1 << 5)),
        "error":           bool(status & (1 << 7)),
        "no_wp":           bool(status & (1 << 11)),
        "wp_gripped":      bool(status & (1 << 12)),
        "pos_reached":     bool(status & (1 << 13)),
        "actual_pos_um":   actual_pos_um,
        "warn_err_code":   (diag >> 8) & 0xFF,   # byte 13
        "additional_code": diag & 0xFF,          # byte 12
    }

# Boot: fast stop + acknowledge until ready
boot = build_outputs((1 << FAST_STOP) | (1 << ACK), 0, 0, 0)

# Move to absolute 20 mm at 50 mm/s:
move = build_outputs((1 << MOVE_ABS), 20_000, 50_000, 0)

# Grip inward at 60% force:
grip = build_outputs((1 << GRIP_WP) | (1 << GRIP_DIR), 0, 0, 60)
```

## 15. Glossary

| Term | Meaning |
|------|---------|
| EGU / EGK / EZU | 2-finger strong / 2-finger soft / 3-finger centric gripper |
| SFP | SCHUNK gripper firmware platform (this family, FW 5.3.x) |
| GPE | Grip force & position maintenance (`M` modules) |
| M / N | Module with / without GPE |
| B / SD | Base / reduced-stroke (SD) variant |
| Steuerdoppelwort / Statusdoppelwort | Control / status double word |
| Diagnosedoppelwort | Diagnosis double word (warning/error + additional code) |
| ISDU | IO-Link Service Data Unit (acyclic parameter access) |
| µ0 | TODO: define |

## 16. References (in ./EGU_EGK_EZU/)

- `EGU V53X Inbetriebnahmeanleitung für PROFINET.pdf` — doc 1514033-EGU-PN-FW5.3.1
  (cyclic data, control/status word, commands, boot sequence).
- `parameters_customer_5.3.X.xlsx` — acyclic parameter list (FW 5.3.1.117131).
- `SCHUNK_SFP-5.3.X_IO-Link_IODD.zip` — IODD (+ MD PDF inside).
- `SCHUNK_SFP-5.3.X_Profinet_GSDML.zip` — GSDML V2.45.
- `SCHUNK_SFP-5.3.X_EtherCAT_ESI(.bits).zip` — ESI per size.
- `SCHUNK_SFP-5.3.X_EtherNetIP_EDS.zip` — EDS.
- TODO: Modbus commissioning manual (if Modbus is a target); per-size datasheets
  (force in N, max velocity); µ0 / vendor SDK docs.
