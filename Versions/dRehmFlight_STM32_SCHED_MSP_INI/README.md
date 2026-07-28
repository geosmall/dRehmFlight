# dRehmFlight STM32 SCHED_MSP_INI

Extends [dRehmFlight STM32 SCHED](../dRehmFlight_STM32_SCHED/) (cooperative scheduler + SerialRx
4-layer failsafe) with three additions: a minimal **MSP V1** telemetry interface, a **modal
CLI** for on-board parameter tuning, and **parameter persistence to internal MCU flash**.
The flight control law is unchanged.

## Upstream References

- **Original dRehmFlight**: https://github.com/nickrehm/dRehmFlight
- **SCHED baseline**: `../dRehmFlight_STM32_SCHED/` (scheduler + SerialRx failsafe)
- **STM32 BETA 1.3** (port baseline): `../dRehmFlight_STM32_BETA_1.3/`
- **Teensy BETA 1.3** (original): `../dRehmFlight_Teensy_BETA_1.3/`

For the full lineage Teensy → STM32 → SCHED → SCHED_MSP_INI, see
`../../doc/DF_EVOLUTION.md`.

## Supported Boards

The board is auto-detected from the Arduino board selection. "Persistence" = the target
reserves an internal-flash config region (`BOARD_FLASH_CONFIG_START`); boards without it run
tuning in RAM only (changes are lost on reboot).

| Board | MCU | IMU | Persistence |
|-------|-----|-----|-------------|
| OPEN_REVO | STM32F405 | MPU-6000 | Internal flash |
| BKMN_NERO | STM32F722 | ICM-20602 | Internal flash |
| JHEF_JHEF411 (Noxe F411) | STM32F411 | ICM-42688-P | Internal flash |
| MATEK_H743VI | STM32H743 | ICM-42688-P | Internal flash |
| DEVEBOX_H743 | STM32H743 | ICM-42688-P | Internal flash |
| BEFH_BETAFPVG473 | STM32G473 | ICM-42688-P | Internal flash |
| WEACT_G474CE | STM32G474 | ICM-42688-P | Internal flash |
| BLACKPILL_F411CE | STM32F411 | MPU-9250 | RAM-only |
| NUCLEO_F411RE | STM32F411 | ICM-42688-P | RAM-only |
| NUCLEO_G474RE | STM32G474 | ICM-42688-P | RAM-only |

## What Changed from SCHED

### 1. MSP V1 — read-only telemetry

A compact **MSP V1** implementation (`$M` framing, XOR checksum) provides a binary telemetry
link over USB Serial for a ground-station / configurator. It is **read-only**: there are no
MSP write or set commands — all configuration is done through the CLI. The handler set is
intentionally small (`mspComm.ino`, `msp.h`):

| Command | Code | Returns |
|---------|------|---------|
| `MSP_API_VERSION` | 1 | Protocol/API version (1.0) |
| `MSP_FC_VARIANT` | 2 | `"DRHM"` (this firmware's own identifier) |
| `MSP_FC_VERSION` | 3 | FC version 1.4.0 |
| `MSP_BOARD_INFO` | 4 | Board ID + target name |
| `MSP_STATUS` | 101 | Cycle time, CPU load %, sensor mask, armed bit |
| `MSP_RAW_IMU` | 102 | Accel / gyro / mag |
| `MSP_MOTOR` | 104 | 8 motor outputs (1000–2000 µs) |
| `MSP_RC` | 105 | 6 RC channels |
| `MSP_ATTITUDE` | 108 | Roll / pitch (decidegrees), yaw (degrees) |
| `MSP_ANALOG` | 110 | Battery/RSSI stub (zeros — no ADC) |
| `MSP_REBOOT` | 68 | Reboot to firmware / ROM DFU / UF2 bootloader |

The command numbering follows the Betaflight/INav MSP convention so standard tooling can do
the identity handshake, but the firmware identifies as its own variant (`DRHM`), not as
Betaflight or INav. `MSP_REBOOT` honors the Betaflight reboot-mode byte: it refuses while
armed, ACKs, then resets — into normal firmware, the ST ROM DFU bootloader, or (where the UF2
bootloader is present) the UF2 bootloader.

### 2. Modal serial: MSP by default, CLI on `#`

`SCHED`'s `TASK_TELEMETRY` (100 Hz debug print) is replaced by `TASK_SERIAL` (100 Hz),
a modal processor (`cliComm.ino` → `processSerial()`):

```
processSerial() [100 Hz, TASK_SERIAL]
├── MSP mode (default) → mspProcessByte()        (binary MSP frames)
│                         └── '#' when disarmed → CLI mode (after 100 ms quiet-period guard)
└── CLI mode          → cli.process()            (text commands)
                          └── leading '$' → auto-return to MSP mode
```

CLI entry is blocked while armed. The 100 ms guard prevents a stray `0x23` (`#`) byte in a
corrupt MSP stream from dropping into the CLI mid-flight — the same pattern Betaflight/INav use.

| Task | Rate | Priority | Purpose |
|------|------|----------|---------|
| `TASK_FLIGHT` | 2000 Hz | REALTIME | IMU, Madgwick, PID, mixer, motors |
| `TASK_RC` | 500 Hz | HIGH | SerialRx input + 4-layer failsafe |
| `TASK_SERIAL` | 100 Hz | MEDIUM | MSP / CLI processor |
| `TASK_BLINK` | 2 Hz | LOW | LED heartbeat + arm-ready chime |

### 3. CLI commands (EmbeddedCLI)

Entered with `#` when disarmed. The command table (`cliComm.ino`):

| Command | Description |
|---------|-------------|
| `help` | List commands |
| `status` | Armed state, loop Hz, CPU load % |
| `version` | Firmware build provenance (git SHA + dirty, branch, UTC, board) |
| `set [name [value]]` | List all params, or read/write one (range-clamped) |
| `diff [all]` | `diff`: params changed from defaults; `diff all`: every param |
| `save` | Save all RAM parameters to config flash |
| `cal` | Calibrate accel (level + still) and auto-save (Betaflight-style) |
| `defaults` | Reset RAM parameters to compile-time defaults |
| `dump` | Print the stored config-flash INI text (debug; reads flash, not RAM) |
| `exit` | Return to MSP mode (does **not** reboot) |
| `reboot` | `NVIC_SystemReset()` |
| `bl` | Enter the UF2 bootloader (only on boards built with it) |

`set` with no argument prints the whole parameter table with each value, its `[min:max]`
range, and group. `set <name> <value>` strictly parses the number (rejecting garbage rather
than silently writing 0) and clamps to range. `set <name>` reads one parameter.

`diff`/`diff all` and `version` emit a build-stamped provenance header
(`# ... fw_git: <sha> ... board: ...`) — the firmware self-identifies its source commit. The
stamp (`build_id.h`) is generated automatically by the core prebuild hook on every compile (no
flag needed): a clean tree gives a bare SHA, a dirty tree `<sha>-dirty`, an out-of-tree build
`nogit`. The boot banner carries the same stamp. `diff`/`diff all` output `set name = value`
lines under that header, so a
captured tune pastes straight back into the CLI (the `#` header lines are treated as comments).
This is what `../../tools/save_tune.sh` captures into `../../tunes/` — see `../../tunes/README.md`.
Unlike `diff` (live RAM), `dump` reads what's actually persisted in config flash, so it's the
way to confirm a `save` landed.

### 4. Parameter persistence — internal-flash INI

Parameters are serialized as INI text (`[pid]` + `name=value` lines) and stored in a dedicated
**internal MCU flash region** using the same append-log format as this project's UF2
bootloader (`cliComm.ino`, via `<ini_flash_config.h>`). Per-family flash primitives cover
F4/F7 (word program, sector erase), G4 (doubleword, page erase) and H7 (32-byte flash word,
dual-bank, D-cache sync). This is **not** a filesystem — there is no LittleFS, SD card, or
`pid.ini` file.

- **Boot**: `loadConfig()` snapshots the compile-time defaults, then overlays any stored values.
- **Save**: CLI `save` (and `cal`, which auto-saves) writes the current RAM values to flash.
- **No-config-flash boards** (Nucleos, NUCLEO_G474RE): `save`/`dump` print
  `No config flash on this board`; tuning is RAM-only and lost on reboot.

### 5. Parameter table

`set`, `save`, `defaults`, and persistence all operate on one table (`cliComm.ino`):

| Group | Parameters |
|-------|-----------|
| PID Angle | `Kp/Ki/Kd_roll_angle`, `Kp/Ki/Kd_pitch_angle` |
| PID Yaw | `Kp/Ki/Kd_yaw` |
| PID Rate | `Kp/Ki/Kd_roll_rate`, `Kp/Ki/Kd_pitch_rate` |
| Loop Damping | `B_loop_roll`, `B_loop_pitch` |
| Limits | `i_limit`, `maxRoll`, `maxPitch`, `maxYaw` |
| Throttle | `thr_mid`, `thr_expo`, `throttle_limit` |
| Mixer | `yaw_motors_reversed`, `motor_idle` |
| Board | `align_board_roll`, `align_board_pitch`, `align_board_yaw` |
| Filters | `B_madgwick`, `B_accel`, `B_gyro`, `B_mag` |
| IMU Cal | `AccErrorX`, `AccErrorY`, `AccErrorZ` |
| Controller | `controller` |

(`motor_output_reordering`, a CSV permutation, is handled alongside the table as a special
case in `set`/`diff`/INI.)

Gyro bias is **not** in this table — it is measured at boot (and at first arm) by a
non-blocking wait-for-still calibration and is not persisted.

`controller` selects the PID structure: `0` = `controlANGLE()` (default, stock dRehmFlight
behavior), `1` = `controlANGLE2()` (cascade). It is latched once at boot — `set`, `save`,
reboot to change — so the structure can never switch mid-flight; `status` shows the active
structure. Before flying the cascade, read the Controller Selection caution in
`QUAD_TUNING.md`: stock `*_rate` gains limit-cycle small craft.

### 6. Betaflight-derived flight behaviors (new since SCHED)

These touch the command/arming/calibration path, not the PID math:

- **Throttle curve** — `getDesState()` runs throttle through `throttleCurve()`, a port of
  Betaflight 4.5's two-segment Bézier (`thr_mid`/`thr_expo`) plus a throttle-limit scale.
  Defaults: `thr_mid=0.40`, `thr_expo=0.55`, `throttle_limit=0.75`.
- **Mixer** — Betaflight QuadX motor numbering, a runtime `yaw_motors_reversed` sign, and a
  `motor_idle` armed idle floor (default `0.06` ≈ BF `dshot_idle_value=600`).
- **Universal arm convention** — on every RX protocol, **CH5 HIGH (>1500) = arm**, LOW = off
  (Betaflight convention). `channel_5_fs` is 1000 (LOW), so TX loss always disarms.
- **Arm-ready latch** — arming is refused until the arm switch has been seen OFF on a valid
  link (Betaflight `ARMING_DISABLED_ARM_SWITCH`); a board powered up with the switch ON will
  not auto-arm.
- **Non-blocking gyro calibration** — replaces the blocking 12000-sample
  `calculate_IMU_error()`. Accel level calibration (`cal`) is persisted; gyro bias is measured
  by a wait-for-still routine at boot and re-measured at first arm. Arming is gated on gyro
  calibration completing. The blink task shows solid-ON during calibration and plays a rising
  Bluejay ESC-beacon chime once armable (DShot targets only).
- **Default FSR** changed to **1000 DPS / 4G** (was 250 DPS / 2G), targeting larger builds
  that clip at the original defaults.

## What Did NOT Change

The flight control law is identical to SCHED / BETA 1.3:

- PID controllers (`controlANGLE()`, `controlANGLE2()`, `controlRATE()`)
- Control mixer math, Madgwick filter body, command scaling
- IMU read + filtering, motor/servo command path
- INav cooperative scheduler structure
- SerialRx 4-layer failsafe
- All user-specified `#define`s (RC protocol, IMU, gyro/accel FSR)

`radioComm.ino` is byte-identical to SCHED.

## Radio Channel Mapping

| Channel | Function | Notes |
|:-------:|----------|-------|
| 1 | Throttle | 1000 min, 2000 max |
| 2 | Roll | 1000 left, 2000 right, 1500 center |
| 3 | Pitch | 1000 up, 2000 down, 1500 center |
| 4 | Yaw | 2000 left, 1000 right, 1500 center |
| 5 | Arm | **HIGH (>1500) = armed**, LOW = disarmed |
| 6 | Aux1 | Free auxiliary channel |

**Failsafe defaults** (on signal loss): Throttle 1000, Roll/Pitch/Yaw 1500, CH5 1000 (disarm),
Aux1 2000.

**Arming**: CH5 HIGH (>1500) **and** throttle below 1050 **and** the arm-ready latch is set
(arm switch was seen OFF on a valid link) **and** gyro calibration has completed.

## File Structure

```
dRehmFlight_STM32_SCHED_MSP_INI/
├── dRehmFlight_STM32_SCHED_MSP_INI.ino   # Main sketch (setup, flight logic, calibration, helpers)
├── msp.h                           # MSP V1 frame constants, command codes, parser states
├── mspComm.ino                     # MSP V1 parser + 10 read-only handlers + reboot
├── cliComm.ino                     # EmbeddedCLI, modal serial processor, param table, flash persistence
├── radioComm.ino                   # SerialRx adapter (channel mapping, 4-layer failsafe)
├── task_list.h                     # Task enum (FLIGHT, RC, SERIAL, BLINK)
├── tasks.ino                       # Task config table and implementations
├── COPYING.txt                     # MIT License
└── README.md                       # This file
```

## Configuration Options

```cpp
// RC Receiver — leave all three commented to auto-select (CRSF on G4 ELRS boards, else SBUS)
// #define USE_IBUS_RX
// #define USE_SBUS_RX
// #define USE_CRSF_RX

// IMU (required — choose one)
#define USE_ICM42688P     // ICM-42688-P, ICM-20602, MPU-6000 (auto-detect)
// #define USE_MPU9250_SPI // MPU-9250/9255 (enables magnetometer)

// Gyro Full Scale Range (default 1000 DPS)
// #define GYRO_250DPS
// #define GYRO_500DPS
#define GYRO_1000DPS
// #define GYRO_2000DPS

// Accelerometer Full Scale Range (default 4G)
// #define ACCEL_2G
#define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G
```

## STM32 Libraries Used

- **IMU** — IMU wrapper with auto-detection (ICM-42688-P, MPU-6000, ICM-20602, MPU-9250)
- **SerialRx** — IBus/SBUS/CRSF parser with 4-layer failsafe
- **PWMOutputBank** — MotorManager (OneShot125 / DShot) and ServoManager
- **BoardConfig** — multi-board pin abstraction (`targets/*.h`)
- **BoardAlignment** — chip + board rotation matrix (FLU vehicle frame)
- **scheduler** — INav cooperative task scheduler
- **EmbeddedCLI** — lightweight CLI with command registration
- **ini_flash_config** — internal-flash INI append-log (shared with the UF2 bootloader)
- **printf** — float-capable `sprintf_` (newlib-nano lacks `%f`)

All libraries available in [Arduino_Core_STM32](https://github.com/geosmall/Arduino_Core_STM32).

## Build

**Requires the `STM32_Robotics` core, version `robo-2.1.0` or later** (earlier releases
lack `BoardAlignment` and several targets — the sketch will not compile). Install via
Arduino Board Manager / arduino-cli:

```bash
INDEX=https://github.com/geosmall/BoardManagerFiles/raw/main/package_stm32_robotics_index.json
arduino-cli core update-index --additional-urls "$INDEX"
arduino-cli core install STM32_Robotics:stm32 --additional-urls "$INDEX"
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=BEFH_BETAFPVF405 <path-to-this-sketch>
```

(Arduino IDE: add the index URL under Preferences → Additional Board Manager URLs, install
"STM32 Robotics Core", pick the board under Tools.)

```bash
# From the repository root
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=OPEN_REVO Versions/dRehmFlight_STM32_SCHED_MSP_INI

# Other targets
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=BKMN_NERO Versions/dRehmFlight_STM32_SCHED_MSP_INI
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=MATEK_H743VI Versions/dRehmFlight_STM32_SCHED_MSP_INI
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=DEVEBOX_H743 Versions/dRehmFlight_STM32_SCHED_MSP_INI
arduino-cli compile --fqbn STM32_Robotics:stm32:Nucleo_64:pnum=NUCLEO_F411RE Versions/dRehmFlight_STM32_SCHED_MSP_INI
```

## Known Limitations

1. **MSP is read-only** — there is no MSP-driven configuration; all tuning is via the CLI.
2. **No-config-flash boards** (BLACKPILL_F411CE, NUCLEO_F411RE, NUCLEO_G474RE) are RAM-only —
   PID/parameter changes are lost on reboot.
3. **No battery/RSSI telemetry** — `MSP_ANALOG` returns zeros (no ADC wired).

## License

GPL v3 — same as the original dRehmFlight (see `COPYING.txt`). The bundled INav-derived
`Scheduler` library is likewise GPL.

## Attribution

Original work by Nicholas Rehm: https://github.com/nickrehm/dRehmFlight

STM32 port maintains the educational focus and clean code style of the original while adding a
minimal MSP telemetry link, an on-board CLI, and internal-flash parameter persistence.

---

## Appendix A: PID Persistence Round-Trip Test

Manual test using a serial terminal. Connect to the board's USB CDC port (e.g.
`/dev/ttyACM0`) at 115200 baud:

```bash
picocom -b 115200 /dev/ttyACM0
# or: minicom -D /dev/ttyACM0 -b 115200
# or: Arduino Serial Monitor (115200 baud)
```

**Step 1 — Enter CLI and read a default value:**
```
#                              ← type '#' to enter CLI (disarmed only)

dRehmFlight CLI
Type 'help' for commands, 'exit' to return, 'reboot' to restart

# set Kp_roll_angle
Kp_roll_angle = 0.200000 [0.000000:10.000000] {PID Angle}
```

**Step 2 — Change and save:**
```
# set Kp_roll_angle 0.25
Kp_roll_angle = 0.250000 [0.000000:10.000000] {PID Angle}
# save
Saved to config flash
```

**Step 3 — Reboot and verify:**
```
# reboot
Rebooting...
```
After reconnecting:
```
#
# set Kp_roll_angle
Kp_roll_angle = 0.250000 ...   ← value survived reboot ✓
```

**Step 4 — Restore:**
```
# set Kp_roll_angle 0.2
# save
# reboot
```

### Pass Criteria

- Step 2: `save` prints `Saved to config flash`
- Step 3: after reboot, `Kp_roll_angle` reads `0.250000` (not the default `0.200000`)
- Step 4: after restore + reboot, it reads `0.200000`

On a RAM-only board, `save` prints `No config flash on this board` and the value does not
survive reboot — this is expected.

## Appendix B: Inspecting Stored Config

`dump` prints the raw INI text currently stored in config flash:

```
# dump
[pid]
Kp_roll_angle=0.250000
Ki_roll_angle=0.300000
...
```

On a board with no stored config yet, `dump` prints `(no config stored)`. On a board without
a config-flash region, it prints `No config flash on this board`.
