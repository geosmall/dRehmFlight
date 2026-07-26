# dRehmFlight STM32 SCHED

Extends the [dRehmFlight STM32 BETA 1.3](../dRehmFlight_STM32_BETA_1.3/)
port with an INav-style cooperative scheduler and the SerialRx 4-layer failsafe, plus a
runtime board-alignment matrix and CRSF/ELRS support. The flight control law is unchanged.

## Upstream References

- **Original dRehmFlight**: https://github.com/nickrehm/dRehmFlight
- **Teensy BETA 1.3**: `../dRehmFlight_Teensy_BETA_1.3/`
- **STM32 BETA 1.3** (comparison baseline): `../dRehmFlight_STM32_BETA_1.3/`
- **Diff reports** (Teensy vs STM32 BETA 1.3): `../dRehmFlight_STM32_BETA_1.3/diff_reports/`

For the full lineage Teensy → STM32 → SCHED → SCHED_MSP_INI, see
`../../doc/DF_EVOLUTION.md`.

## Supported Boards

Auto-detected from the Arduino board selection.

| Board | MCU | IMU | Notes |
|-------|-----|-----|-------|
| OPEN_REVO | STM32F405 | MPU-6000 | OpenPilot Revolution FC |
| BKMN_NERO | STM32F722 | ICM-20602 | NERO F7 flight controller |
| MATEK_H743VI | STM32H743 | ICM-42688-P | MATEK H743-WLITE FC |
| BLACKPILL_F411CE | STM32F411 | MPU-9250 | Compact dev board (9-DOF) |
| NUCLEO_F411RE | STM32F411 | ICM-42688-P | Development board |
| BEFH_BETAFPVG473 | STM32G473 | ICM-42688-P | BetaFPV G473 FC (DShot600, CRSF/ELRS) |
| WEACT_G474CE | STM32G474 | ICM-42688-P | G473 surrogate (DShot600, CRSF) |
| NUCLEO_G474RE | STM32G474 | ICM-42688-P | G4 dev board (DShot600, CRSF) |

## What Changed from BETA 1.3

### 1. INav cooperative scheduler

BETA 1.3 runs everything in a single monolithic `loop()` at 2 kHz with `loopRate(2000)`.
SCHED replaces this with an INav-style cooperative scheduler that separates concerns into
priority-based tasks:

| Task | Rate | Priority | Replaces |
|------|------|----------|----------|
| `TASK_FLIGHT` | 2000 Hz | REALTIME | Main loop body (IMU, PID, motors) |
| `TASK_RC` | 500 Hz | HIGH | `getCommands()` + failsafe in main loop |
| `TASK_TELEMETRY` | 100 Hz | MEDIUM | `print_counter` self-limiting in main loop |
| `TASK_BLINK` | 2 Hz | LOW | `loopBlink()` function |

```cpp
// BETA 1.3 - monolithic loop
void loop() {
  dt = (micros() - prev_time)/1000000.0;
  loopBlink();
  getCommands(); failSafe(); getIMUdata(); Madgwick(...);
  controlANGLE(); controlMixer(); scaleCommands();
  throttleCut(); commandMotors(); commandServos();
  loopRate(2000);
}

// SCHED - scheduler dispatches tasks
void loop() {
  scheduler();  // INav cooperative scheduler
}
```

Task definitions are in `task_list.h` (enum) and `tasks.ino` (config table + implementations).
The scheduler library is a genuine INav fork (it forked from Cleanflight/Betaflight); the
canonical idioms are all in use — `cfTask_t`, `TASK_PERIOD_HZ()`, `TASK_PRIORITY_*`,
`getTaskDeltaTime()`, `averageSystemLoadPercent`.

**dt calculation**: Now uses `getTaskDeltaTime(TASK_SELF)` with a safety clamp (0 < dt ≤ 10 ms,
falling back to 500 µs), instead of raw `micros()` arithmetic.

### 2. SerialRx 4-layer failsafe

BETA 1.3 uses a `failSafe()` function that checks PWM range bounds (800–2200) on every channel.
SCHED replaces this with the SerialRx library's 4-layer failsafe detection, gated in `TASK_RC`:

| Layer | Method | Description |
|-------|--------|-------------|
| 1 | Protocol flag | SBUS/CRSF failsafe / frame-lost bits |
| 2 | Timeout | No valid frames within timeout period |
| 3 | Range check | Channel values outside valid protocol range |
| 4 | Expiry | Stale data beyond idle threshold |

```cpp
// BETA 1.3 - range-check failsafe in main loop
failSafe();  // checks 800 < channel_x_pwm < 2200

// SCHED - SerialRx failsafe in TASK_RC
if (radioSignalLost()) {  // rx.isSignalLost() - 4-layer check
    channel_1_pwm = channel_1_fs;
    // ...
}
```

New adapter functions in `radioComm.ino`:
- `radioSignalLost()` → `rx.isSignalLost()`
- `radioSignalStatus()` → human-readable status string

### 3. Board-alignment matrix

BETA 1.3 baked the sensor-axis sign flips into the `Madgwick()` call arguments. SCHED moves
this into a runtime rotation matrix: `getIMUdata()` multiplies gyro/accel/mag by
`boardAlignMatrix` (chip-to-board × board-to-vehicle, Betaflight-style FLU vehicle frame),
built in `IMUinit()` from the target's alignment plus the `board_align_{yaw,pitch,roll}_degrees`
globals. The `Madgwick()` call now passes clean axes. For the default orientation the result is
equivalent, but this is a real change to how sensor axes reach the estimator — review for any
board whose alignment is not identity.

### 4. CRSF / ELRS and DShot (G4 boards)

- New `USE_CRSF_RX` path (420000 baud) with Betaflight AETR→TAER channel remap in `getCommands()`.
- CH5 polarity branch: CRSF builds use the Betaflight convention (CH5 **HIGH = armed**),
  IBus/SBUS keep the original dRehmFlight convention (CH5 **> 1500 = throttle cut**).
  `channel_5_fs` flips accordingly (CRSF 1000, others 2000) so TX loss always disarms.
- Motor commands are carried as normalized 0–1 values (`m*_command_scaled`) through
  `MotorManager`, supporting OneShot125 and DShot300/600. Three STM32G4 ELRS boards added.

### 5. Telemetry

- `printRadioData()` appends the link-status string: `CH1:1000 ... CH6:2000 [OK]`.
- New `printSchedulerStats()` (selectable in `TASK_TELEMETRY`) shows CPU load and task rates:
  `CPU:12% | FLIGHT:2000Hz | RC:500Hz | dt:500us`.

## What Did NOT Change

The flight control law is identical to BETA 1.3:

- PID controllers (`controlANGLE()`, `controlANGLE2()`, `controlRATE()`)
- Control mixer math (`controlMixer()`, original dRehmFlight motor numbering)
- Madgwick filter body (`Madgwick()`, `Madgwick6DOF()`), command scaling (`scaleCommands()`)
- Desired-state computation (`getDesState()`), arming logic core, throttle-cut safety
- IMU read + filtering core (only the alignment-matrix multiply at the end of `getIMUdata()`
  is new — see §3), motor/servo command path, ESC calibration
- All PID tuning parameters and filter coefficients
- All user-specified defines (RC protocol, IMU, gyro/accel FSR)

## Radio Channel Mapping

PWM microseconds (1000–2000). Re-assign in `radioComm.ino` (`updateRadioChannels()`) if your
radio uses a different channel order.

| Channel | Function | Notes |
|:-------:|----------|-------|
| 1 | Throttle | 1000 min, 2000 max |
| 2 | Roll | 1000 left, 2000 right, 1500 center |
| 3 | Pitch | 1000 up, 2000 down, 1500 center |
| 4 | Yaw | 2000 left, 1000 right, 1500 center |
| 5 | Gear / Arm | IBus/SBUS: > 1500 = throttle cut. CRSF: HIGH = armed |
| 6 | Aux1 | Free auxiliary channel |

**Failsafe defaults** (on signal loss): Throttle 1000, Roll/Pitch/Yaw 1500, Aux1 2000. CH5
defaults to the safe (disarm / throttle-cut) value for the active protocol (CRSF 1000, else 2000).

**Arming** (IBus/SBUS): CH5 below 1500 **and** throttle below 1050. (CRSF inverts the CH5 sense.)

## File Structure

```
dRehmFlight_STM32_SCHED/
├── dRehmFlight_STM32_SCHED.ino   # Main sketch (setup, flight logic, helpers)
├── radioComm.ino           # SerialRx adapter (radio setup, channel mapping, failsafe)
├── task_list.h             # Task enum (FLIGHT, RC, TELEMETRY, BLINK)
├── tasks.ino               # Task config table and implementations
├── COPYING.txt             # MIT License
└── README.md               # This file
```

## Configuration Options

```cpp
// RC Receiver — leave all three commented to auto-select (CRSF on G4 ELRS boards, else SBUS)
// #define USE_IBUS_RX
// #define USE_SBUS_RX
// #define USE_CRSF_RX

// IMU (required - choose one)
#define USE_ICM42688P     // ICM-42688-P, ICM-20602, MPU-6000 (auto-detect)
// #define USE_MPU9250_SPI // MPU-9250/9255 (enables magnetometer)

// Gyro Full Scale Range
#define GYRO_250DPS       // Default - highest resolution
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Accelerometer Full Scale Range
#define ACCEL_2G          // Default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G
```

## STM32 Libraries Used

- **IMU** — IMU wrapper with auto-detection (ICM-42688-P, MPU-6000, ICM-20602, MPU-9250)
- **SerialRx** — IBus/SBUS/CRSF parser with 4-layer failsafe and optional DMA
- **PWMOutputBank** — MotorManager (OneShot125 / DShot) and ServoManager (50 Hz PWM)
- **BoardConfig** — multi-board pin abstraction (`targets/*.h`)
- **BoardAlignment** — chip + board rotation matrix (FLU vehicle frame)
- **scheduler** — INav cooperative task scheduler

All libraries available in [Arduino_Core_STM32](https://github.com/geosmall/Arduino_Core_STM32).

## Build

```bash
# From the repository root
arduino-cli compile --fqbn STM32_Robotics:stm32:Nucleo_64:pnum=NUCLEO_F411RE Versions/dRehmFlight_STM32_SCHED

# Flight controllers
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=OPEN_REVO Versions/dRehmFlight_STM32_SCHED
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=BKMN_NERO Versions/dRehmFlight_STM32_SCHED
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=MATEK_H743VI Versions/dRehmFlight_STM32_SCHED
```

**Requirements**: arduino-cli 1.0.0+, `STM32_Robotics` core **`robo-2.1.0` or later**
(Board Manager URL:
`https://github.com/geosmall/BoardManagerFiles/raw/main/package_stm32_robotics_index.json`).

## License

GPL v3 — same as the original dRehmFlight (see `COPYING.txt`). The bundled INav-derived
`Scheduler` library is likewise GPL.

## Attribution

Original work by Nicholas Rehm: https://github.com/nickrehm/dRehmFlight

STM32 port maintains the educational focus and clean code style of the original while
demonstrating minimal-change hardware abstraction patterns.
