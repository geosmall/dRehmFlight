# Quad PID Tuning Notes

Pre-flight tuning analysis for two vehicle classes. Except where marked flight-validated, values are starting points based on physics reasoning, not empirical data. Real tuning comes from flying.

## Controller Selection (flight-validated)

The persistent `controller` param selects the control structure at boot:

- `controller = 0` (default): `controlANGLE()` — flattened angle PID, stock dRehmFlight behavior.
- `controller = 1`: `controlANGLE2()` — cascaded angle→rate PID with a full inner rate loop.

The value is latched once at boot: `set controller = 1`, `save`, then reboot. `status` reports the active structure; the structure never changes mid-flight.

**Caution — stock rate gains limit-cycle small craft on the cascade.** The swap to `controlANGLE2()` at shipped `*_rate` gains produced an immediate fixed-frequency oscillation on both craft tested (Air75: 10 Hz; Pavo Pico II: 8–9 Hz). Before the first cascade flight, cut `Kp_*_rate` to half stock or below, then tune inner-first: halve `Kp_*_rate` per oscillating axis until the cycle breaks, set `Kd_*_rate ≈ Kp_rate/(2π·f_cycle)` (gate on motor temperature by touch), then walk `Kp_*_rate` back up.

Flight-validated cascade operating points (outer angle gains per each craft's saved tune):

| Craft | Kp_rate | Ki_rate | Kd_rate | B_loop | Status |
|-------|---------|---------|---------|--------|--------|
| Air75 (75mm 1S) | 0.10 | 0.2 | 0.001 | 0.9 | Validated; beats `controlANGLE()` in calm and active flight |
| Pavo Pico II | 0.075 | 0.2 | 0.001 | 0.9 | Provisional (calm-air verification pending) |

**Pavo Pico II strongly wants the cascade.** On `controlANGLE()` it is flyable only at reduced gains (roll Kp 0.15 / Kd 0.035) with a visible residual ripple and soft attitude hold — the flattened controller's rate damping hits a phase-lag gain ceiling at 6–8 Hz on this airframe. On `controlANGLE2()` with inner D it holds 2× that rate feedback cleanly and set the craft's best recorded hover. The Air75 flies well either way; the cascade is a documented option on both.

## Signal Path

`controlANGLE()` computes:
```
roll_PID = 0.01 * (Kp * error_deg + Ki * integral_deg_s - Kd * gyro_dps)
```

Output is roughly -1 to 1, added directly to throttle (0-1) in the mixer. So `roll_PID = 0.1` means 10% of motor range applied differentially. Yaw is rate-controlled: `error_yaw = yaw_des - GyroZ` (deg/s error, not angle error).

All gains are adjustable at runtime via CLI `set` command and persist across reboots via `save`/`load`. Gyro/accel FSR are compile-time only (`#define` in sketch header).

## Loop Rate

2 KHz is sufficient for both vehicles in angle mode.

- Filter coefficients (`B_gyro = 0.1`, `B_accel = 0.14`) are tuned for 2 KHz — do not change loop rate without retuning filters
- DShot300 frame time (~53 us) fits easily within 500 us loop period
- Betaflight uses 8 KHz on 5" builds, but that's driven by acro/rate mode with aggressive D-terms
- If moving to `controlRATE()` or `controlANGLE2()` for aggressive flying on the 250mm, consider whether 2 KHz D-term update rate is sufficient

## Gyro/Accel FSR Selection

| Vehicle | Gyro FSR | Accel FSR | Notes |
|---------|----------|-----------|-------|
| Air75 (75mm 1S) | `GYRO_250DPS` | `ACCEL_2G` | Defaults are fine — low authority limits rotation rates |
| 250mm 4S | `GYRO_1000DPS` | `ACCEL_4G` | **Defaults will clip — must change** |

The 250mm build can easily exceed 250 deg/s even in angle-mode corrections, and any banked turn exceeds 2G. Clipped gyro data causes the Madgwick filter to compute wrong attitude and the D-term to see artificial zero-derivative during saturation — both dangerous.

Required changes for 250mm in sketch header:
```cpp
//#define GYRO_250DPS
#define GYRO_1000DPS    // safe for angle mode, use GYRO_2000DPS for acro

//#define ACCEL_2G
#define ACCEL_4G        // minimum for banked flight, use ACCEL_8G for aggressive maneuvers
```

---

## Air75 II (75mm Ducted Whoop)

**Vehicle:** BetaFPV Air75 II, 0802 19500KV motors, 1S, ~25g AUW

### Vehicle Characteristics

- **Low inertia**: tiny arms, tiny motors — responds quickly to small torques
- **Low motor torque**: 0802 on 1S, limited thrust margin (~2:1 T/W)
- **Aerodynamic damping**: ducted props provide significant free damping (acts like a passive D-term)
- **Poor yaw authority**: small props close together, limited differential torque
- **Thermal sensitivity**: 0802 motors overheat quickly from D-term noise

Low inertia and low motor torque partially cancel: the vehicle responds quickly but doesn't have much authority to give.

### Starting Gains

| Parameter | Default | Air75 | Rationale |
|-----------|---------|-------|-----------|
| `Kp_roll_angle` | 0.2 | **0.2** | Keep — low inertia compensates for low motor authority |
| `Ki_roll_angle` | 0.3 | **0.15** | Halve — low authority makes integrator wind-up slow to unwind |
| `Kd_roll_angle` | 0.05 | **0.03** | Reduce — ducts provide aerodynamic damping, tiny motors overheat from D noise |
| `Kp_pitch_angle` | 0.2 | **0.2** | Same as roll (symmetric quad) |
| `Ki_pitch_angle` | 0.3 | **0.15** | Same as roll |
| `Kd_pitch_angle` | 0.05 | **0.03** | Same as roll |
| `Kp_yaw` | 0.3 | **0.3** | Keep — whoops need all the yaw P they can get |
| `Ki_yaw` | 0.05 | **0.05** | Keep — helps overcome yaw steady-state error |
| `Kd_yaw` | 0.00015 | **0.0001** | Slight reduction — uses error derivative (noisy) |
| `i_limit` | 25.0 | **15.0** | Lower — 25 deg-s of wind-up is too much authority for 1S whoop |
| `maxRoll` | 30.0 | **30.0** | Keep for first flights, increase to 40-45 once stable |
| `maxPitch` | 30.0 | **30.0** | Same |

At full stick (30 deg error): P contributes 6% motor range, D contributes ~10% at 200 deg/s gyro.

### Tuning Notes

- If motors get hot, reduce `Kd` first — D-term noise is the primary cause of motor heating
- Ducted whoops tolerate less D than open-prop quads due to the aerodynamic damping already present

---

## 250mm Quad (5" Freestyle)

**Vehicle:** 250mm class, 2207 2450KV motors, 4S LiPo, 5030x3 triblade props

### Vehicle Characteristics

- **High inertia**: 250mm arms with heavy 2207 stator motors at tips — resists rotation, needs more torque to change attitude
- **High motor authority**: 4S with 5030 triblades, easily 8-12:1 thrust-to-weight
- **No aerodynamic damping from ducts**: open props, needs active D-term damping
- **Good yaw authority**: large props with wide spacing give strong differential torque
- **Prop wash turbulence**: triblades produce significant disturbed air in descents — tests PID robustness
- **Thermal headroom**: 2207 motors tolerate D-term noise much better than 0802

High inertia + high authority = the controller has both the need and the means to apply large corrections. Gains should be higher than the whoop.

### Starting Gains

| Parameter | Default | 250mm | Rationale |
|-----------|---------|-------|-----------|
| `Kp_roll_angle` | 0.2 | **0.3** | Higher inertia needs more P for the same angular response |
| `Ki_roll_angle` | 0.3 | **0.3** | Keep — plenty of motor authority to unwind integrator |
| `Kd_roll_angle` | 0.05 | **0.07** | Increase — no duct damping, 2207 motors tolerate D noise well |
| `Kp_pitch_angle` | 0.2 | **0.3** | Same as roll (symmetric quad) |
| `Ki_pitch_angle` | 0.3 | **0.3** | Same as roll |
| `Kd_pitch_angle` | 0.05 | **0.07** | Same as roll |
| `Kp_yaw` | 0.3 | **0.25** | Can reduce slightly — good yaw authority from large props at wide spacing |
| `Ki_yaw` | 0.05 | **0.05** | Keep |
| `Kd_yaw` | 0.00015 | **0.00015** | Keep — error derivative, don't over-amplify noise |
| `i_limit` | 25.0 | **25.0** | Keep — this build has the authority to use it |
| `maxRoll` | 30.0 | **35.0** | Slightly higher — 30 deg feels sluggish on a 5" quad |
| `maxPitch` | 30.0 | **35.0** | Same |
| `maxYaw` | 160.0 | **200.0** | 5" quads can comfortably yaw faster |

At full stick (30 deg error): P contributes 9% motor range, D contributes 21% at 300 deg/s gyro.

### Tuning Notes

- Without duct damping, D is essential — increase until oscillation damps crisply
- Oscillation will be lower frequency than the whoop (more inertia) and more visible
- Watch for yaw oscillation at high throttle (prop torque effect)
- Watch for integrator-driven bobble in hover (slow rhythmic tilting = too much I)

### Prop Wash Handling

5030 triblades produce heavy prop wash in descents and quick direction changes. If the quad oscillates violently when descending through its own prop wash:
- This is normal and difficult to fully eliminate with a single-loop angle PID
- Reducing I helps (less wind-up during the disturbance)
- `controlANGLE2()` (cascaded angle/rate) handles prop wash better but requires more tuning effort
- The real solution for aggressive flight is `controlRATE()` with a well-tuned D-term

---

## Tuning Procedure (Both Vehicles)

1. **P only first** — set `Ki` and `Kd` to 0 for roll/pitch. Increase `Kp` until mild oscillation, back off 20-30%
2. **Add D** — increase `Kd` slowly until oscillation damps quickly. Monitor motor temperature
3. **Add I last** — bring `Ki` up until hover holds level without drift. Keep `i_limit` appropriate for the build
4. **Yaw separately** — yaw is rate-controlled, tune independently from roll/pitch

Use CLI to adjust live without reflashing: `set Kp_roll_angle 0.25`

## Vehicle Comparison

| Aspect | Air75 (75mm 1S) | 250mm 4S |
|--------|-----------------|----------|
| Inertia | Very low | High |
| Motor authority | Low | Very high |
| Free damping | Ducts (significant) | None |
| D-term tolerance | Low (motor heating) | High |
| Yaw authority | Poor | Good |
| Gyro FSR needed | 250 DPS | 1000+ DPS |
| Accel FSR needed | 2G | 4G minimum |
| Overall gain level | Lower | Higher |
