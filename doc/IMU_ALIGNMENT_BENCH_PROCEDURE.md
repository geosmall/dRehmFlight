# IMU Alignment Bench Acceptance Procedure

The gate that promotes a target's IMU alignment to `HW-validated`. One
procedure for every board — nothing here is board-specific, and no expected
value is hard-coded per target.

Satisfies requirement **R5** in `BOARD_ALIGNMENT_REQUIREMENTS.md` (this directory). Run it
before the first flight of any new or changed alignment, and on a known-good
craft before its verdict is trusted on a suspect one.

**Tool:** `tools/imu_align_check.py` — read-only, props off.

---

## What this proves, and what it does not

This checks that a board's **physical** IMU behavior matches the alignment its
firmware believes it has. It is a hardware test, downstream of two things that
are already settled offline:

- the alignment *math* is verified against Betaflight source (requirements
  §5.1) and guarded by unit tests, so a matrix bug is not what you are hunting;
- the alignment *inputs* come from up to three places (requirements §5.2), only
  one of which is compiled in.

So before running this, **read back the board's `align_board_roll/pitch/yaw`
and confirm they are what the board is supposed to have.** A perfect matrix fed
a missing board-align value produces a board that fails this procedure for
reasons that have nothing to do with the sensor. (On builds that predate the
runtime `align_board_*` parameters, the values are compile-time constants and
this check is trivially satisfied.)

What it does **not** cover: sensor noise and vibration behavior, scale-factor
accuracy beyond roughly ±20%, temperature effects, or anything about the
magnetometer.

## Why the method is sound

Two phases, each pinning one sensor against an absolute reference:

**Phase A — static poses.** Gravity is an absolute external reference. Holding
a known face up pins the **accel** frame absolutely: no dynamics, no
integration, no dependence on operator timing or smoothness.

**Phase B — supervised sweeps.** One slow ~90° rotation per axis in a stated
direction, each chosen to be **positive** in the right-handed FLU body frame:

- left side rising → gyro X positive
- nose dropping → gyro Y positive
- nose swinging left → gyro Z positive

The gyro's net per-axis rotation over the sweep must land on the intended axis
with the intended sign. The judge is gyro-only, so translation, pivot offset,
sample rate, and speed are all irrelevant — the **direction** is the reference,
and the operator supplies it. That is exactly the trust Phase A places in
"hold it left side up"; Phase B extends the same trust to "roll it left side
up". Together: A pins the accel frame absolutely, B pins the gyro frame
absolutely.

### Method history — two retired designs

Both earlier Phase B designs were killed by this procedure's own rule that the
known-good craft (Air75) must pass before any verdict is trusted:

1. **Freehand per-axis path integration** (July 2026, predates the tool).
   Rotations do not commute: integrating each gyro axis over a wandering
   hand-path dumps real energy into off-axis integrals and fabricates
   cross-axis coupling on a healthy sensor — 8–15° of phantom rotation,
   which sent a real investigation down a dead end. The tool's self-test
   still demonstrates the artifact.
2. **Gyro-vs-gravity correlation** via the transport theorem
   `d(g_body)/dt = -omega x g_body` (the tool's first version, retired
   2026-07-21). Physically correct, but its verdict was hostage to operator
   ergonomics: hand pivots centimeters-to-forearm-lengths from the IMU inject
   linear acceleration (`alpha x r`, `omega x (omega x r)`) at signal order,
   and slow wrist rates sit near the tremor floor. On the *same healthy
   Air75*, r swung 0.43–0.87 across four sessions. An instrument that only
   works for a perfect operator is not an instrument.

The sweep judge keeps what survived: the only per-axis integral it takes is
over a single supervised **monotone** sweep, judged against coarse bars
(45°/30°) an order above the non-commutativity artifact — and ambiguous
motion earns a REDO prompt, never a fault verdict.

## Before you start

- **Props off.** The tool is read-only — it never writes, saves, reboots, or
  arms — but the board is powered and this is a hard rule.
- Board on USB, running a dRehmFlight build with MSP (`MSP_RAW_IMU`).
- Know which way is **forward** and which side is **left** on the board as
  mounted in the airframe. Every pose and sweep is stated in those terms.
- Body frame under test: **+X forward, +Y left, +Z up**. Values arrive
  post-alignment — straight off `boardAlignMatrix` — so this measures the
  composed result, not the raw chip.

Prove the tool itself first (no hardware needed):

```bash
tools/imu_align_check.py --self-test
```

This simulates supervised sweeps through a realistic acquisition model —
MSP quantization, timing jitter, hand tremor, pivot-offset linear
acceleration — and confirms healthy sweeps pass at both fast and slow link
rates while injected sign flips and axis swaps fail. Expect `SELF-TEST PASS`.

## Running it

```bash
tools/imu_align_check.py                 # default /dev/ttyACM0, both phases
tools/imu_align_check.py /dev/ttyACM1
tools/imu_align_check.py --phase a       # poses only
tools/imu_align_check.py --phase b       # sweeps only
```

`SPACE` advances, `q` aborts. Exit code is 0 only if every executed check
passed.

### Phase A — six static poses

Hold each pose steady; capture is automatic once the board is still. Roughly
level by eye is fine — the thresholds are generous, and stillness is judged on
the gyro so a pose captured mid-motion is rejected rather than averaged in.

| Pose | Expect |
|---|---|
| Level, upright | `+1 g` on **Z** |
| Inverted (belly up) | `-1 g` on **Z** |
| Nose up (standing on its tail) | `+1 g` on **X** |
| Nose down | `-1 g` on **X** |
| Left side up (rolled right) | `+1 g` on **Y** |
| Right side up (rolled left) | `-1 g` on **Y** |

The dominant axis must read at least 0.80 g and every other axis must stay
under 0.28 g.

### Phase B — three supervised sweeps

Each step is **one slow steady sweep of about 90° in the stated direction**,
then hold still. Capture starts when the board moves and ends when it stops.
Rotate in place; speed and smoothness do not matter.

| Step | Motion | Gyro must read |
|---|---|---|
| Roll | From level, roll **left side up** ~90° | net **+X** |
| Pitch | From level, tip the **nose down** ~90° | net **+Y** |
| Yaw | Stay level, swing the **nose left** ~90° | net **+Z** |

The tool prints the net rotation it saw on all three axes for every sweep, so
a healthy board shows ~±90° on one axis and near-zero on the others.

**The direction is the reference.** A sweep done the wrong way is
indistinguishable from a sign fault — if you get a FAIL, first confirm you
swept the stated direction, then rerun once before believing it.

## Reading the result

| Report | Meaning |
|---|---|
| `PASS` on a pose | That body axis reads gravity with the right sign |
| `FAIL` on a pose | The accel frame is wrong — wrong axis dominant, wrong sign, or too much bleed into other axes |
| `PASS` on a sweep | The gyro put the rotation on the intended axis with the intended sign |
| `FAIL: ... axis mapping fault` | The sweep registered on a different gyro axis — a permutation error |
| `FAIL: ... sign fault` | Right axis, wrong direction — a sign error (or a wrong-direction sweep; confirm and rerun once) |
| `REDO: only N deg` | Sweep too small — go closer to 90° and try again; no verdict recorded |
| `REDO: N deg of off-axis rotation` | Sweep too diagonal — keep it about one axis; no verdict recorded |

A REDO is never a finding — the tool re-prompts in place until it gets a
judgeable sweep or you quit.

## Promotion

A target may be marked `HW-validated` in the requirements §5 ledger when:

1. `--self-test` passes on the host;
2. the board's `align_board_*` values were read back and match intent;
3. all six poses pass;
4. all three sweeps pass — each landing ~90° on the intended axis with the
   intended sign.

**Order matters across boards.** Run the whole procedure on a known-good craft
(the Air75) before trusting its verdict on a suspect one (the Pavo). An
unproven test aimed at a suspect board produces hypotheses, not evidence — if
the Air75 fails, fix the procedure, not the firmware. This rule has now
retired two Phase B designs (see Method history); it is the most valuable
sentence in this document.

## If a board fails

Work outward from the cheapest cause:

1. **Operator.** For a sweep sign FAIL: confirm the sweep direction matched
   the prompt, rerun once.
2. **Config.** Re-read `align_board_roll/pitch/yaw`; `save`, power-cycle, and
   read them back again. A value that is right until the power-cycle and wrong
   after is a persistence fault, not a sensor fault.
3. **Mounting.** Confirm the physical chip orientation matches the target's
   `IMUAlignment`, and that "forward" in the airframe is what the target
   assumes.
4. **Sensor path.** Only once 1–3 are clean does a failure implicate the
   driver or the chip. Both sensors failing together points at the shared
   alignment; one sensor failing alone points below it (requirement R2).

Do not fly a board that fails this procedure.

## Validation record

- **2026-07-21 — Air75 (BEFH-BETAFPVG473, CW180, anchor build `90f0ebcce`):
  PASS 9/9.** Poses 6/6 (three consecutive sessions); sweeps X +81°, Y +90°,
  Z +94°, off-axis ≤6°. The procedure is proven on a known-good craft and its
  verdict may now be trusted on suspect boards.
