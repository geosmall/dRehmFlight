# dRehmFlight STM32

[Nicholas Rehm's dRehmFlight](https://github.com/nickrehm/dRehmFlight) — the
teaching-oriented VTOL flight controller — ported to STM32 and evolved into a
multi-board flight stack. **The control law is untouched**: every change is in
the I/O boundary, loop architecture, sensor-axis handling, or configuration UX,
never the PID math. The repo carries the full evolution as diffable in-repo
stages:

| Stage | Directory | What it adds |
|-------|-----------|--------------|
| 0 | `Versions/dRehmFlight_Teensy_BETA_1.3/` | Upstream Teensy original |
| 1 | `Versions/dRehmFlight_STM32_BETA_1.3/` | STM32 hardware-abstraction port (F4/F7/H7) |
| 2 | `Versions/dRehmFlight_STM32_SCHED/` | INav cooperative scheduler, 4-layer RX failsafe, board-alignment matrix, CRSF/ELRS, DShot |
| 3 | `Versions/dRehmFlight_STM32_SCHED_MSP_INI/` | MSP telemetry, modal CLI, blackbox, internal-flash parameter persistence |

**Build and fly `dRehmFlight_STM32_SCHED_MSP_INI`.** It is the maintained,
flight-validated configuration; the earlier stages are kept as lineage
artifacts for study and diffing. Full stage-by-stage change record:
`doc/DF_EVOLUTION.md`.

**Companion: [dRehm Configurator](https://geosmall.github.io/dRehm-configurator/)**
— a browser-based configurator PWA (Chrome/Edge, Web Serial; source:
[geosmall/dRehm-configurator](https://github.com/geosmall/dRehm-configurator))
that speaks this firmware's MSP telemetry and CLI. Connect over USB to watch
live status/attitude, RC channels, and sensor graphs; set parameters in the
grouped PID/filter editor and save them to the board; or use its CLI terminal
to enter the bootloader (`bl`) and reflash from the Arduino IDE (once the UF2
bootloader is present on the board).

## Supported boards

Board is selected via the Arduino board menu (`pnum`). "Persistence" = tuning
survives reboot (internal-flash config region); RAM-only boards lose `set`
changes at power-off.

| Board | MCU | IMU | Persistence |
|-------|-----|-----|-------------|
| BEFH_BETAFPVF405 (Pavo Pico II AIO) | STM32F405 | ICM-42688-P | Internal flash |
| BEFH_BETAFPVG473 (Air75 AIO) | STM32G473 | ICM-42688-P | Internal flash |
| OPEN_REVO | STM32F405 | MPU-6000 | Internal flash |
| BKMN_NERO | STM32F722 | ICM-20602 | Internal flash |
| JHEF_JHEF411 (Noxe F411) | STM32F411 | ICM-42688-P | Internal flash |
| MATEK_H743VI | STM32H743 | ICM-42688-P | Internal flash |
| DEVEBOX_H743 | STM32H743 | ICM-42688-P | Internal flash |
| WEACT_G474CE | STM32G474 | ICM-42688-P | Internal flash |
| BLACKPILL_F411CE | STM32F411 | MPU-9250 | RAM-only |
| NUCLEO_F411RE / NUCLEO_G474RE | F411 / G474 | ICM-42688-P | RAM-only |

Flight-validated: BEFH_BETAFPVG473 (Air75 whoop) and BEFH_BETAFPVF405
(Pavo Pico II). Other targets are build-verified and bench-tested to varying
degrees — see `doc/BOARD_ALIGNMENT_REQUIREMENTS.md` for the honest per-target
validation ledger before trusting one in the air.

## Install and build

Requires the **`STM32_Robotics` Arduino core, `robo-2.1.0` or later**
([geosmall/Arduino_Core_STM32](https://github.com/geosmall/Arduino_Core_STM32) —
a fork of the STM32 Arduino core with type-safe pins, peripheral-aware bus
constructors, DShot, and UF2 bootloader support).

**Arduino IDE:** Preferences → Additional Board Manager URLs →
`https://github.com/geosmall/BoardManagerFiles/raw/main/package_stm32_robotics_index.json`,
then Boards Manager → install "STM32 Robotics Core", pick your board under
Tools, open `Versions/dRehmFlight_STM32_SCHED_MSP_INI/dRehmFlight_STM32_SCHED_MSP_INI.ino`.
(Works on Windows, macOS, and Linux. arduino-cli users: the same index URL and
core work from the command line — full steps in the sketch README.)

**Flashing** flight controllers with the UF2 bootloader installed: enter the
bootloader (CLI `bl` command, or MSP reboot-to-bootloader), a UF2 drive
appears, then upload with `upload_method=bootuf2Method` or drag the built
`.uf2`. The UF2 bootloader binaries and first-time installation / recovery
documentation live in the [core repository](https://github.com/geosmall/Arduino_Core_STM32)
(`bootloaders/`, `doc/`).

## Before you fly — safety gates

This is firmware for spinning blades. Work through these in order; skipping
them is how flyaways happen:

1. **IMU alignment bench check** (`doc/IMU_ALIGNMENT_BENCH_PROCEDURE.md`,
   `tools/imu_align_check.py`) — props off, read-only. Run before the first
   flight of ANY new or changed board alignment. A wrong axis sign flips a
   correction into an amplification the moment the craft leaves the ground.
2. **Motor order / direction check** (`tools/motor_order_check.py`, or the
   CLI `motor <1-4> <pct>` single-motor bench test) — props off, verify
   pad→position mapping and spin direction against your frame.
3. **Props-off arm test** — arm, verify throttle response and throttle-cut
   on all four motors, verify disarm.
4. **First hover** — props on only after 1–3 pass, in a safe area, low, brief.

The `align_board_*` and `motor_output_reordering` parameters are runtime
(`set`/`save`) — most boards can be adapted without recompiling, but every
alignment change re-triggers gate 1.

Converting a stock Betaflight craft? A worked end-to-end walkthrough —
bootloader install, radio setup, calibration — for the BETAFPV Air75 is in
`Versions/dRehmFlight_STM32_SCHED_MSP_INI/README.md` → "First Flight".

## Tuning and tools

- All gains and filters are runtime CLI parameters: `#` enters the CLI,
  `set`, `save`, `diff all` (Betaflight-style paste-back). See
  `Versions/dRehmFlight_STM32_SCHED_MSP_INI/README.md` and `Versions/dRehmFlight_STM32_SCHED_MSP_INI/QUAD_TUNING.md`.
- The [dRehm Configurator](https://geosmall.github.io/dRehm-configurator/)
  (see top) offers the same parameter editing and CLI from the browser.
- `tools/` — host-side helpers that talk to the firmware over USB serial:
  blackbox fetch (`bb_fetch.py`), MSP queries, restrained-rig test,
  tune capture with build provenance (`save_tune.sh`).
- `tunes/` — reference tunes for the flight-validated craft, self-stamped
  with the exact firmware commit they flew on.

## License

GPL v3, same as upstream dRehmFlight (`LICENSE`). The bundled INav-derived
scheduler is likewise GPL. Copyright original work Nicholas Rehm; STM32
port and flight-stack evolution by the maintainers of this fork.
