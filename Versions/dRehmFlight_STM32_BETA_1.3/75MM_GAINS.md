# 75mm Whoop First-Flight Gain Tuning

## Hardware

- **Frame**: 75mm class whoop
- **Board**: BEFH-BETAFPVG473 (BetaFPV STM32G473)
- **Motors**: 0802 brushless / 1S LiPo
- **Radio**: SBUS (ELRS)
- **AUW**: ~25-30g with battery

## Why Reduce Gains

The defaults target a general-purpose quadcopter. A 0802/1S 75mm whoop has **much lower moment of inertia** and **limited thrust authority with 1S voltage sag**, so the defaults will produce oscillation. The biggest risk is **Ki** — high integral gain + low inertia + voltage sag = death wobble.

## Control Mode

Use **`controlANGLE()`** (the default). Do not switch to `controlANGLE2` or `controlRATE` until basic hover is solid.

## Gain Changes

### PID Gains — Roll/Pitch Angle Mode (lines 160-167)

| Parameter | Default | Recommended | Change | Rationale |
|-----------|---------|-------------|--------|-----------|
| `Kp_roll_angle` | 0.2 | **0.15** | -25% | Low inertia needs less P to avoid overshoot |
| `Ki_roll_angle` | 0.3 | **0.10** | -67% | **Critical reduction** — #1 crash risk on 1S micros |
| `Kd_roll_angle` | 0.05 | **0.03** | -40% | Less damping needed on light frame; too much D heats 0802 motors |
| `Kp_pitch_angle` | 0.2 | **0.15** | -25% | Match roll (symmetric airframe) |
| `Ki_pitch_angle` | 0.3 | **0.10** | -67% | Match roll |
| `Kd_pitch_angle` | 0.05 | **0.03** | -40% | Match roll |

### PID Gains — Yaw (lines 176-178)

| Parameter | Default | Recommended | Change | Rationale |
|-----------|---------|-------------|--------|-----------|
| `Kp_yaw` | 0.3 | **0.2** | -33% | Yaw authority limited on 75mm; less P avoids jerky yaw |
| `Ki_yaw` | 0.05 | **0.03** | -40% | Reduce integral buildup |
| `Kd_yaw` | 0.00015 | **0.0001** | -33% | Less derivative noise on small props |

### Controller Limits (lines 155-158)

| Parameter | Default | Recommended | Change | Rationale |
|-----------|---------|-------------|--------|-----------|
| `i_limit` | 25.0 | **15.0** | -40% | Tighter integrator saturation prevents wind-up during 1S sag |
| `maxRoll` | 30.0 | **20.0** | -33% | Conservative angle limit for first flight |
| `maxPitch` | 30.0 | **20.0** | -33% | Match roll |
| `maxYaw` | 160.0 | **120.0** | -25% | Less yaw rate for controllability |

### Filters — No Changes (lines 133-135)

Keep `B_madgwick=0.04`, `B_accel=0.14`, `B_gyro=0.1` at defaults. Tune only if needed after first flights.

### Gyro Range — No Change (line 48)

Keep `GYRO_250DPS`. With 20° angle limits, gyro rates won't approach 250 deg/s. Higher resolution gives better PID response.

## Copy-Paste Block

```cpp
//Controller parameters (take note of defaults before modifying!):
float i_limit = 15.0;     //Integrator saturation level (default 25.0, reduced for 75mm)
float maxRoll = 20.0;     //Max roll angle in degrees for angle mode (default 30.0, conservative first flight)
float maxPitch = 20.0;    //Max pitch angle in degrees for angle mode (default 30.0, conservative first flight)
float maxYaw = 120.0;     //Max yaw rate in deg/sec (default 160.0, reduced for 75mm)

float Kp_roll_angle = 0.15;    //Roll P-gain - angle mode (default 0.2)
float Ki_roll_angle = 0.10;    //Roll I-gain - angle mode (default 0.3, critical reduction for 75mm)
float Kd_roll_angle = 0.03;    //Roll D-gain - angle mode (default 0.05)
float B_loop_roll = 0.9;       //Roll damping term for controlANGLE2()
float Kp_pitch_angle = 0.15;   //Pitch P-gain - angle mode (default 0.2)
float Ki_pitch_angle = 0.10;   //Pitch I-gain - angle mode (default 0.3, critical reduction for 75mm)
float Kd_pitch_angle = 0.03;   //Pitch D-gain - angle mode (default 0.05)
float B_loop_pitch = 0.9;      //Pitch damping term for controlANGLE2()

float Kp_roll_rate = 0.15;     //Roll P-gain - rate mode (unchanged)
float Ki_roll_rate = 0.2;      //Roll I-gain - rate mode (unchanged)
float Kd_roll_rate = 0.0002;   //Roll D-gain - rate mode (unchanged)
float Kp_pitch_rate = 0.15;    //Pitch P-gain - rate mode (unchanged)
float Ki_pitch_rate = 0.2;     //Pitch I-gain - rate mode (unchanged)
float Kd_pitch_rate = 0.0002;  //Pitch D-gain - rate mode (unchanged)

float Kp_yaw = 0.2;            //Yaw P-gain (default 0.3)
float Ki_yaw = 0.03;           //Yaw I-gain (default 0.05)
float Kd_yaw = 0.0001;         //Yaw D-gain (default 0.00015)
```

## Flight Test Procedure

1. **Pre-arm bench test**: Verify IMU attitude in Configurator (level = 0/0), verify RC channels respond
2. **Props-off motor test**: Arm, advance throttle slightly, verify all 4 motors spin correct direction
3. **Tethered hover** (if possible): Loosely tether, arm, throttle to ~40-50%, check for oscillation
4. **Free hover**: Gentle throttle up to hover (~50-60% on 1S). Hold 5-10 seconds. Look for:
   - **Fast oscillation** (buzzing) → Kp or Kd too high, reduce by 25%
   - **Slow oscillation** (rocking) → Ki too high, reduce by 50%
   - **Sluggish response** → Kp too low, increase by 25%
   - **Drift not correcting** → Ki too low, increase by 25%
   - **Yaw spin** → Kp_yaw too low or motor direction wrong
5. **Iterate**: Change ONE parameter at a time, retest

## Post-First-Flight Tuning Progression

Once stable hover is achieved:

1. Increase `maxRoll`/`maxPitch` to 30 for more authority
2. Gradually increase `Kp_roll/pitch_angle` toward 0.2 for snappier response
3. Increase `Ki_roll/pitch_angle` toward 0.15-0.2 if drift isn't correcting
4. Increase `Kd_roll/pitch_angle` toward 0.04 if oscillation appears at higher Kp
5. Once angle mode is solid, consider `controlANGLE2()` for better disturbance rejection
