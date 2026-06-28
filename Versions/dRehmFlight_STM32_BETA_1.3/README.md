# dRehmFlight STM32 Port - BETA 1.3

Minimal-change port of [dRehmFlight](https://github.com/nickrehm/dRehmFlight) BETA 1.3 from Teensy 4.0 to STM32 for 4-motor conventional quadcopter.

## Overview

This port preserves 100% of Nicholas Rehm's flight control logic while adapting only the hardware interface layer for STM32F4/F7/H7 microcontrollers.

**Supported Boards** (5 boards):

| Board | MCU | IMU | Notes |
|-------|-----|-----|-------|
| NUCLEO_F411RE | STM32F411RE | ICM-42688-P | Development board (breadboard IMU) |
| BLACKPILL_F411CE | STM32F411CE | MPU-9250 | Compact dev board (9-DOF) |
| OPEN_REVO | STM32F405RGT6 | MPU-6000 | OpenPilot Revolution FC |
| BKMN_NERO | STM32F722RET6 | ICM-20602 | NERO F7 flight controller |
| MATEK_H743VI | STM32H743VIT6 | ICM-42688-P | MATEK H743-WLITE FC |

## Upstream Links

- **Original dRehmFlight**: https://github.com/nickrehm/dRehmFlight
- **Teensy reference**: `Versions/dRehmFlight_Teensy_BETA_1.3/`

**Diff Reports** (`diff_reports/`, side-by-side Teensy → STM32):
- `diff_main.html` - Main sketch comparison
- `diff_radioComm.html` - Radio communication comparison

**Updating Diff Reports**:
- **Automatic**: A git pre-commit hook regenerates diffs when `.ino` files are committed
- **Manual** (from `dRehmFlight/`):
  ```bash
  ../ci/tools/winmerge_diff.py \
    Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino \
    Versions/dRehmFlight_STM32_BETA_1.3/dRehmFlight_STM32_BETA_1.3.ino \
    Versions/dRehmFlight_STM32_BETA_1.3/diff_reports/diff_main.html

  ../ci/tools/winmerge_diff.py \
    Versions/dRehmFlight_Teensy_BETA_1.3/radioComm.ino \
    Versions/dRehmFlight_STM32_BETA_1.3/radioComm.ino \
    Versions/dRehmFlight_STM32_BETA_1.3/diff_reports/diff_radioComm.html
  ```

## What Changed

**Hardware Interface Layer Only** (flight control logic untouched):

### 1. IMU Integration - IMU Library
- `IMUinit()`: Uses IMU library with auto-detection (ICM-42688-P, MPU-6000, MPU-9250)
- `getIMUdata()`: Library-based reads with configurable FSR
- **Preserved**: Error correction, low-pass filtering, all math

### 2. Radio RX - SerialRx Library
- `radioSetup()`: SerialRx initialization (SBUS default, IBus optional)
- `updateRadioChannels()`: Adapter pattern (SerialRx → channel_X_raw). SBUS→µs conversion uses
  the iNav formula via `channelToPWM()` (vs the Teensy's hand-tuned `*0.615 + 895` mapping)
- **Optional**: DMA mode for reduced interrupt overhead

### 3. Motor Control - MotorManager (TimerPWM)
- `commandMotors()`: Hardware timers via BoardConfig abstraction
- `armMotors()`: ESC arming sequence
- **Protocol**: OneShot125 (125-250µs pulses)

### 4. Servo Control - ServoManager (TimerPWM)
- `commandServos()`: Standard PWM via BoardConfig abstraction
- **Protocol**: 50 Hz PWM (1000-2000µs)

### 5. Pin Configuration - BoardConfig System
- **Auto-detection**: Board type from Arduino IDE selection
- **Abstraction**: Motors, servos, IMU, RC all via BoardConfig
- **Multi-board**: Single codebase supports all 5 boards

## What Did NOT Change

**Flight Control Law Preserved** (numerically identical for the default 6-DOF config):
- ✅ PID Controllers (`controlANGLE()`, `controlANGLE2()`, `controlRATE()`)
- ✅ Control Mixer (`controlMixer()`)
- ✅ Madgwick Filter (`Madgwick6DOF()`)
- ✅ Command Scaling — motor path (`scaleCommands()`, `*125 + 125`)
- ✅ Failsafe Logic (`failSafe()`)
- ✅ Arming Logic (`armedStatus()`)
- ✅ Loop Timing (2kHz)
- ✅ All PID Tuning Parameters and filter coefficients

**Two minor edits inside flight-logic files** (neither changes the control law):
- `Madgwick()` — the compile-time `#if defined USE_MPU6050_I2C → Madgwick6DOF()` short-circuit
  was removed. On the default 6-DOF IMUs the existing runtime `mx==my==mz==0` guard routes to
  `Madgwick6DOF()` anyway, so the result is identical — just no longer byte-for-byte.
- `scaleCommands()` servo path — servo output changes from a 0–180° angle (Teensy `PWMServo`)
  to a 1000–2000 µs pulse (`ServoManager`). Motor scaling is unchanged.

**Sensor-config note:** the IMU library's BALANCED preset enables on-chip filtering ahead of
the same software `B_gyro`/`B_accel` filters, whereas the Teensy ran the sensor DLPF off. The
raw data feeding the (identical) software filters is therefore not configured identically.

## Metrics (NUCLEO_F411RE)

| Metric | Value |
|--------|-------|
| Binary Size | 49KB (9% of 512KB flash) |
| RAM Usage | 3.7KB (2% of 128KB RAM) |
| Line Count | 1933 → 1734 (-10%) |
| Flight Logic Modified | 0 functions |

## Configuration Options

Enable features in the main `.ino` file:

```cpp
// RC Receiver (required - choose one protocol)
#define USE_SERIAL_RX
// #define USE_IBUS_RX // IBus protocol (FlySky)
#define USE_SBUS_RX    // SBUS protocol (FrSky, etc.) - default
// #define USE_RC_DMA  // Optional: UART DMA for reduced IRQ overhead

// IMU (required - choose one)
#define USE_ICM42688P   // ICM-42688-P, ICM-20602, MPU-6000
// #define USE_MPU9250_SPI // MPU-9250/9255 (enables magnetometer)

// Gyro Full Scale Range
#define GYRO_250DPS     // Default - highest resolution
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS  // Widest range for aerobatics

// Accelerometer Full Scale Range
#define ACCEL_2G        // Default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G
```

## STM32 Libraries Used

- **IMU** - High-level IMU wrapper with auto-detection (ICM-42688-P, MPU-6000, MPU-9250)
- **SerialRx** - IBus/SBUS protocol parser with optional DMA
- **PWMOutputBank** - MotorManager (OneShot125) and ServoManager (50Hz PWM)
- **BoardConfig** - Multi-board pin abstraction (targets/*.h)

All libraries available in [Arduino_Core_STM32](https://github.com/geosmall/Arduino_Core_STM32).

## Build

```bash
# From workspace root (Arduino/)
./ci/build.sh dRehmFlight/Versions/dRehmFlight_STM32_BETA_1.3

# Or with arduino-cli directly
arduino-cli compile --fqbn STM32_Robotics:stm32:Nucleo_64:pnum=NUCLEO_F411RE \
  dRehmFlight/Versions/dRehmFlight_STM32_BETA_1.3

# Build for other boards
arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=OPEN_REVO \
  dRehmFlight/Versions/dRehmFlight_STM32_BETA_1.3

arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=BKMN_NERO \
  dRehmFlight/Versions/dRehmFlight_STM32_BETA_1.3

arduino-cli compile --fqbn STM32_Robotics:stm32:FlightCtr:pnum=MATEK_H743VI \
  dRehmFlight/Versions/dRehmFlight_STM32_BETA_1.3
```

**Requirements**:
- arduino-cli 1.0.0+
- STM32_Robotics core installed

## Debugging

Uses standard `Serial.print()` like the original Teensy version:

```cpp
Serial.begin(115200);  // Initialized in setup()
Serial.println("Status message");
```

**Example Output**:
```
dRehmFlight STM32 BETA 1.3
Radio RX initialized
IMU initialized successfully
Gyro X:0.35 Y:-0.81 Z:0.30
```

## Status

This is the **port baseline**: it establishes the STM32 hardware-abstraction layer (IMU,
SerialRx, TimerPWM, BoardConfig) with the flight control law unchanged from the Teensy. Flight
validation and tuning happened on the downstream sketches — `sketches/dRehmFlight_SCHED`
(scheduler + 4-layer failsafe) and `sketches/dRehmFlight_SCHED_MSP_INI` (MSP/CLI + persistence).
Use those for hardware deployment; this version is kept as the clean minimal-change reference
for the Teensy → STM32 diff.

## License

MIT License - Same as original dRehmFlight

## Attribution

Original work by Nicholas Rehm: https://github.com/nickrehm/dRehmFlight

STM32 port maintains the educational focus and clean code style of the original while demonstrating minimal-change hardware abstraction patterns.
