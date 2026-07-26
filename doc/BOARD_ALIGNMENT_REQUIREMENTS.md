# Board IMU Alignment — Correctness & Validation Requirements

> This is the living per-target validation ledger. Its evidence trail cites
> development-history identifiers — build/commit SHAs, HIL rig IDs
> (`HIL-00x`), and CI scripts (`./ci/...`) — that are not part of this
> repository; they record how each verdict was earned. What matters to a
> builder is the **verdict per target**: fly only what is `HW-validated`,
> and run the bench gate (`IMU_ALIGNMENT_BENCH_PROCEDURE.md`,
> `tools/imu_align_check.py`) before the first flight of any new or changed
> alignment.

Fleet-wide requirements for the `BoardAlignment` system (`IMUAlignment` enum +
`makeSensorToVehicleMatrix`) that maps raw IMU samples into the vehicle FLU
body frame. **Scope is every target and every alignment case, not any single
board.** The BetaFPV Pavo Pico II (F405, `CW270_DEG`) is the first board to
exercise an asymmetric alignment on real hardware and is used here as the
worked example / forcing function — it is not the subject.

Status: **Campaign complete (2026-07-21)** — R1 verified (§5.1), R3 green on
target, R5 satisfied and proven on both fleet boards. Still open: R4 ledger
backfill for the remaining targets (§4 task 6) and the R6 converter gap
(§5.2). Execution record lives in `BOARD_ALIGNMENT_VALIDATION_PLAN.md`.

---

## 1. Why this exists

Different flight controllers mount the IMU in different orientations.
Betaflight handles this with `sensor_align` (CW0/90/180/270 ± flip) plus a
board-align Euler (`align_board_{roll,pitch,yaw}`). Our target-generation
pipeline gained an equivalent in May 2026:

- `IMUAlignment` enum + `IMUConfig::alignment` field (commit `e2ecc44c5`).
- `BoardAlignment` library — `makeTargetSensorAlignMatrix`,
  `makeBetaflightBoardAlignMatrix`, `makeSensorToVehicleMatrix`
  (commits `55db79111`, `667f02905`).
- Converter emits the alignment per target (`09f747f59`); an audit made it
  explicit on every hand-edited target after finding stale defaults
  (`750231796` — e.g. `MATEKH743` was defaulting to `CW0_DEG` instead of
  `CW0_DEG_FLIP`, which would self-level inverted).

The runtime path: raw sensor → `sensor→board` (chip enum) →
`board→vehicle` (Euler) → attitude/control. The same matrix is applied in
`getIMUdata()` **identically to gyro, accel, and mag**.

## 2. Current state — honest ledger (the gap this closes)

- The alignment **values** were transcribed from Betaflight config sources and
  **build-verified only**. Commit `750231796` states the defects were "latent
  because none of these boards is in the current rig registry for HIL
  validation." `doc/DF_EVOLUTION.md:27` carries the same open item from the
  other direction: the composition is now verified and unit-guarded, but
  hardware validation of any non-identity orientation is still outstanding.
- ~~Only **CW0 / CW180** (symmetric, self-inverse) cases have run on hardware
  (F411RE, G474, G473/Air75). **No asymmetric case (CW90/CW270, or any
  `_FLIP`) has been hardware- or flight-validated.**~~ **Closed 2026-07-21:**
  the Pavo's CW270+roll180 passed the R5 bench on both sensors (first
  asymmetric case), settling on hardware the composition/ordering classes
  CW180 masks (transpose, multiply-order, Euler-order). `_FLIP` cases remain
  bench-unvalidated (MATEKH743 is build-only). Flight validation of an
  asymmetric case is still outstanding.
- ~~Even the CW180 case is only validated on an **older load path**: Air75's
  validating flight build (`90f0ebc`, 2026-06-29) predates the
  runtime `align_board_*` rework (`e500a42`, 2026-07-09) and the accel-cal
  gravity fix (`0a86aad`, 2026-07-12). No flight has exercised master's
  current alignment load/config path on any board.~~ **Closed 2026-07-21:**
  Flight 1 flew master `d2c1ffc` clean on the Air75 with the runtime path
  live-verified on the bench (`align_board_*` readback), and the Pavo's
  180/0/0 was verified through the same path including save/power-cycle
  persistence.
- The **matrix math itself is settled**: `makeSensorToVehicleMatrix` is verified
  element-wise against Betaflight source for all 8 enums × 4 board-align cases
  (§5.1), and an AUnit guard encodes that ground truth. What remains unverified
  is everything *around* the math — whether the intended alignment values reach
  it at runtime, and whether the result is correct on physical hardware.
- The alignment **inputs** come from two different places, and only one of them
  is compile-time (§5.2). A target's `IMUAlignment` is baked in, but the
  board-align Euler arrives at runtime through CLI/INI persistence — so a
  correct matrix can still be fed wrong values.
- Consequence: the Pavo "gyro fault" investigation chased a *gyro-only* rotation
  in the IMU driver. Because the alignment matrix is applied identically to all
  sensors, a real alignment error must show on **accel too** — so the
  "gyro-only, accel-clean" premise is unconfirmed and likely a measurement
  artifact. Any validation MUST check accel and gyro together.

## 3. Requirements

**R1 — Conformance to the Betaflight alignment convention.** For every
`IMUAlignment` enum and representative board-align Euler angles,
`makeSensorToVehicleMatrix(chip, yaw, pitch, roll)` applied to a vector MUST
equal Betaflight's net result: `alignSensorViaRotation(chip)` composed with
`initBoardAlignment` / `buildRotationMatrix`. Verified against **Betaflight
source** (`sensors/boardalignment.c`, `common/vector.c`), not our design doc.
Any disagreement is a defect in `BoardAlignment`, not in the target.

*What Betaflight is authoritative for — and what it is not.* Betaflight defines
what the tokens mean (`CW270_DEG`, `align_board_roll = 180`), because our
alignment values are transcribed from Betaflight configs, on Betaflight-designed
boards, and stock Betaflight flies both airframes. That makes it the correct
reference for **this** requirement. It does not make R1 a claim about whether a
given aircraft's sensor data matches its physical motion. **Conformance is
necessary, not sufficient — R1 can pass in full while a board still flies
wrong**, which is exactly the present Pavo state.

R1 is silent on, and cannot detect:

- whether the intended values reach the matrix at runtime (§5.2);
- whether the chip physically installed matches the target's enum, or is
  mounted the way the target assumes;
- a defect below the alignment — driver axis mapping, or a per-axis sensor
  fault (note that an asymmetric alignment *routes* such a fault to a different
  vehicle axis than a symmetric one does, so the same defect presents
  differently on the Pavo than on the Air75);
- whether the two projects' axis *ordering* conventions agree — see the body
  frame note below, which resolves the sign question but not this one.

Every item on that list is settled by physical motion, not by source
comparison. That is R5's job, and it is why both requirements exist.

*Body frame — resolved, both projects are FLU.* R1 proves the two matrices are
identical; it does not by itself prove their outputs mean the same thing
downstream. That question is settled independently, and the answer is that
Betaflight's IMU body frame is **FLU (+X forward, +Y left, +Z up)** — the same
as dRehmFlight's, and the same as ROS REP-103. Evidence, in decreasing order of
directness:

- **Accel calibration.** `sensors/acceleration_init.c:449` sets the Z trim to
  `average - acc_1G` ("shift Z down by acc_1G"), so a trimmed level board reads
  **+1 g on Z**. Betaflight's +Z is up.
- **Handedness.** `flight/imu.c` builds rotation matrices and takes cross
  products throughout, so the frame is right-handed. Right-handed with X
  forward and Z up forces **Y left**.
- **Attitude math agrees.** `imu.c:311` derives roll as
  `atan2(rMat.m[2][1], rMat.m[2][2])` where row 2 is world-up in body
  coordinates; that yields 0 when level and `+phi` for a right roll only if a
  positive rotation about +X lifts +Y, i.e. only if +Y is the left side.
- **Flight evidence.** The Air75 flies with `CW180` = `diag(-1,-1,+1)` copied
  verbatim. A Y-sign mismatch would invert roll self-leveling; a Z-sign
  mismatch would make the craft believe it is inverted. Neither occurs.

**The `axisNED_e` enum in `common/axis.h` is not a counterexample.** It is the
geographic/navigation earth frame, not the sensor body frame, and in the pinned
Betaflight (`01fe1e369`) it is unused — declared and referenced nowhere else in
`src/main`. Body frame and earth frame are separate questions, and only the
former bears on alignment. Betaflight's *Magnetometer* page settles it in as
many words — having introduced NED as the frame the earth's field is usually
expressed in, it adds:

> This representation is not at all relevant for Betaflight's mag orientation,
> but may be useful when interpreting the magnetic field strength and direction
> values reported for your location.

The same page requires magnetometer data be returned X-forward, Y-left, Z-up,
independently reconfirming FLU from a second vendor page.

`doc/BF_BODY_FRAME.md` reaches the same FLU conclusion against **the same
pinned commit**, from Betaflight's published documentation together with the
source. Betaflight's *Flight Controller Orientation* page states the
board-align convention directly:

> R = Rz(-Yaw) \* Ry(-Pitch) \* Rx(-Roll)

> Due to the minus sign, positive angle direction follows the left-hand rule —
> e.g. to yaw positive, grab the z axis with your left hand and rotate towards
> the direction of your fingers.

That is character-for-character what `makeBetaflightBoardAlignMatrix`
implements, and the left-hand sense is what its header comment already
documents (`BoardAlignment.h:41-44`, `BoardAlignment.cpp:13-17`) — written from
source inference before the vendor text was consulted. Our convention is
therefore confirmed three independent ways: numerically against source (§5.1),
by the published formula, and by the published handedness rule.

That document also derives Betaflight's *earth* frame at this revision as
**North-West-Up**, inferred from the GPS course-over-ground conversion
(`imu.c:448-456`: course 0 is north and clockwise, converted through
`sincosf_approx(-courseOverGround, ...)`, so east maps to earth `-Y`) rather
than named anywhere in the source. The earth frame is downstream of alignment
and out of scope for this campaign — but it is the first thing to check if we
ever port Betaflight heading or navigation code, where NWU-versus-NED would
matter directly.

**Residual.** All three *sign* conventions are confirmed above. Axis
*permutation* is not: `CW180` negates X and Y but permutes nothing, so the
Air75 cannot distinguish our axis ordering from a swapped one. It is an
implausible failure given the source evidence, but only an asymmetric alignment
exercises it — and the Pavo's `CW270` (`vX = -sY`, `vY = -sX`) is exactly that
case. Phase A of `doc/IMU_ALIGNMENT_BENCH_PROCEDURE.md` settles it on any board
in about two minutes.

**R2 — Sensor symmetry is explicit.** The alignment is one matrix applied to
gyro, accel, and mag. Therefore an alignment error manifests on ALL sensors;
validation and debugging MUST treat "gyro looks wrong but accel looks right" as
evidence *against* an alignment cause, not for it.

**R3 — Unit-test coverage.** AUnit tests assert `makeSensorToVehicleMatrix`
against precomputed Betaflight ground-truth matrices for each of the 8 enums,
each combined with (a) zero board-align, (b) a non-trivial board-align, and
(c) an **asymmetric, non-180° board-align** (e.g. `roll=90`, `yaw=-45`).
Case (c) is mandatory: any 180° rotation has a symmetric matrix (`B^T = B`),
so cases built only from 180° rotations — including the once-suggested
`roll=180, yaw=-45`, which is itself a net 180° rotation — cannot detect a
transposed board-align convention (found and mutation-verified during Stage 3,
see §5.1). Tests are offline and hardware-independent, so a convention
regression is catchable fast — but there is no automated CI, so the
guard is only as good as the discipline of running
`./ci/saflash.sh tests/BoardAlignment_UT` when `BoardAlignment` changes.

**R4 — Per-target validation ledger.** A maintained table (§5) records, for
each shipped target: alignment enum, its Betaflight source (`file:line`), and
validation status. **No target may be flown on an alignment whose status is not
`HW-validated`** — build-only/transcribed alignments are provisional.
*Bootstrap exception:* the §7 baseline ladder (Flights 0/1) re-flies the
Air75's previously flight-validated configuration to establish the trust
anchor; those two flights are the one permitted case of flying ahead of
`HW-validated` status, and they are what promotes it.

**R5 — Fleet-reusable bench acceptance procedure.** A single documented
procedure (not per-board scripts) that, for any board on USB: drives each axis
to a known ±90° pose and asserts the **correct axis and sign on both accel and
gyro** over MSP. This is the gate that promotes a target to `HW-validated` and
must pass before first flight of any new/changed alignment.
*Satisfied by* `doc/IMU_ALIGNMENT_BENCH_PROCEDURE.md` + `tools/imu_align_check.py`
(read-only, props off), **proven on the Air75 2026-07-21 (9/9)**; the tool's
`--self-test` discharges the offline half of the "prove the procedure first"
rule below. Three constraints, learned the hard way — the first two from the
July 2026 Pavo investigation, the third from this tool's own first version:
- **The procedure itself must be validated on a known-good craft** (Air75)
  before its verdict is trusted on a suspect one. An unproven test aimed at a
  suspect board produces hypotheses, not evidence. This rule has now retired
  two Phase B designs; it is doing exactly its job.
- **No freehand-integration verdicts.** Naive per-axis integration of a
  hand-rotated board dumps real energy into off-axis integrals (rotation
  non-commutativity) and fakes cross-axis coupling on a healthy sensor. The
  accepted form is a *supervised monotone single-axis sweep* judged against
  coarse bars (45°/30°) an order above that artifact, with ambiguous motion
  earning a redo rather than a verdict.
- **The instrument must be operator-robust.** The tool's first Phase B
  (gyro-vs-gravity transport correlation) was physically correct yet swung
  r = 0.43–0.87 across sessions on the same healthy board — pivot-offset
  linear acceleration and near-tremor rates made its verdict a function of
  the operator's hands. A verdict that depends on operator ergonomics is not
  acceptance-grade; and any self-test must model the real acquisition
  (quantization, timing jitter, tremor, pivot offset), since a float-only
  simulation happily passed the judge that hardware then rejected.

**R6 — Regression guard.** The converter remains the single source of the
alignment field; hand-edited target drift is caught by the existing audit
(`750231796`). New targets get their alignment from the converter, not by hand.
The converter currently satisfies this for the chip enum only — it does not
parse `DEFAULT_ALIGN_BOARD_*`, so a board that bakes a board-align default into
its Betaflight config generates a target that is silently missing half its
alignment (§5.2). Closing that is a prerequisite for porting any such board.

## 4. Verification tasks (ordered)

1. ~~Transcribe Betaflight's `alignSensorViaRotation` + `initBoardAlignment` /
   `buildRotationMatrix`; derive ground-truth and compare to
   `makeSensorToVehicleMatrix`.~~ **DONE — passed, see §5.1.** No composition
   bug exists.
2. ~~If a bug is found, fix `BoardAlignment`.~~ **N/A — no bug found.** The
   targets and the library are both left untouched.
3. **DONE** — the R3 AUnit tests in
   `tests/BoardAlignment_UT/BoardAlignment_UT.ino` (`betaflight_ground_truth_*`)
   compile clean, are mutation-validated on host, and **now pass on target**:
   24/24 on a NUCLEO-G474RE (HIL-008), 2026-07-20. The guard is
   hardware-executed, not merely build-verified.
4. ~~Run the R5 bench acceptance on the **Air75 first** — a known-good craft
   proves the procedure itself (per R5) before it is trusted anywhere else.~~
   **DONE 2026-07-21 — 9/9 on the resident anchor build.** The known-good
   gate rejected the tool's first two Phase B designs before passing the
   third (see R5's constraints and the procedure doc's method history).
5. ~~Run the R5 bench acceptance on the Pavo (both sensors) → promote or
   reject.~~ **DONE 2026-07-21 — promoted** (9/9 on master; the resident
   branch binary first failed with a gyro-only Y ≈ −X signature, root-caused
   to an untrusted build, not the board — plan Stage 6 outcome).
6. Backfill the R4 ledger for every target; re-validate any asymmetric case.
7. Close the converter gap in §5.2 so board-align defaults stop being dropped
   silently (R6).

## 5. Per-target alignment ledger (partial — complete during task 6)

`GYRO_1_ALIGN` sources are cited against the tracked converter inputs under
`Arduino_Core_STM32/extras/betaflight_converter/bf_configs/`. The **board-align**
column is the second, runtime half of the alignment (§5.2) — a blank there means
the target expects 0/0/0.

| Target | Enum | `GYRO_1_ALIGN` source (`file:line`) | Board-align (source) | Validation status |
|---|---|---|---|---|
| BEFH-BETAFPVG473 (Air75) | CW180_DEG | `BETAFPVG473/config.h:109` | none — 0/0/0 | **HW-validated + flight-validated on master 2026-07-21** — R5 bench 9/9 on the anchor build AND on master `d2c1ffcf3` (runtime `e500a42` path, `align_board_*` 0/0/0 live), Flight 0 (anchor, 2026-07-19) and Flight 1 (master, 2026-07-21) both clean. |
| BEFH-BETAFPVF405 (Pavo) | CW270_DEG | `BETAFPVF405_ELRS/config.h:115` (not in `bf_configs/`; from `BETAFPV_FCs/`) | **roll=180 — CLI/INI only**, in OEM dump and device diff | **HW-validated 2026-07-21** on master `d2c1ffcf3` — R5 bench 9/9 (poses 6/6; sweeps +92/+100/+87°, off-axis ≤7°); config readback 180/0/0 verified incl. save+power-cycle persistence. **First asymmetric alignment validated in the fleet.** Fly master-lineage builds only; the 2026-07-16 branch binary is condemned (plan Stage 6). |
| MTKS-MATEKH743 | CW0_DEG_FLIP | `MATEKH743/config.h:160` | none — 0/0/0 | build-only |
| JHEF-JHEF411 | CW180_DEG | `JHEF411/config.h:104` | none — 0/0/0 | build-only |
| WEACT_G474_HIL007 | CW180_DEG | n/a (bench rig, mirrors BETAFPVG473) | none — 0/0/0 | build-only |
| _…all remaining targets…_ | | | | |

Two rows carry asymmetry risk if they are ever ported: BetaFPV **G473_V2**
(board-align `yaw = -45`, CLI-borne) and **G473_V3** (`DEFAULT_ALIGN_BOARD_YAW
-45`, baked into its config.h). Neither is a shipped target today; both would
need the §5.2 handling before they could be.

### 5.1 Appendix: R1 ground-truth matrix comparison — **PASSED 2026-07-18**

**Verdict: 32/32 cases match element-wise. The alignment math is exonerated.**
Worst element error 8.74e-8 (float `sinf(pi)` residual; tolerance 1e-6).

*Scope.* This is a conformance result, not a statement that any board is
correctly aligned in flight — see R1 for what it does and does not cover.

**Method.** Host harness compiled the production
`Arduino_Core_STM32/libraries/BoardAlignment/src/BoardAlignment.cpp`
**unmodified** (only `ConfigTypes.h` shimmed to the bare enum) against a
literal transcription of Betaflight source at `01fe1e369`
(4.5.0-1069, local clone `betaflight/`), ground truth in double precision:

- `alignSensorViaRotation` switch — `src/main/sensors/boardalignment.c:94-145`
- `initBoardAlignment` (early-returns on all-zero board align) —
  `src/main/sensors/boardalignment.c:64-78`
- `buildRotationMatrix` — `src/main/common/vector.c:214-236`
- `applyRotationMatrix` = `matrixTrnVectorMul` — `src/main/common/vector.c:203-212,238-241`

Net ground-truth matrix per case obtained by pushing basis vectors through the
BF pipeline (sensor switch first, then board align — order per
`boardalignment.c:142-144`); compared element-wise to
`makeSensorToVehicleMatrix(chip, yaw, pitch, roll)`.

**Path confirmation.** For all 8 standard enums Betaflight executes the
`alignSensorViaRotation` *switch*, not the decidegree matrix path:
`gyro.c:414-418`, `acceleration.c:59-68`, `compass.c:479-483` all gate on
`ALIGN_CUSTOM` and fall through to the switch otherwise. `ALIGN_CUSTOM` /
`buildRotationMatrixFromAngles` is out of our scope (per `ConfigTypes.h`).

**Convention equivalence (the suspected composition bug — cleared).**
Betaflight applies the **transpose** of `buildRotationMatrix`'s output
(`applyRotationMatrix` delegates to `matrixTrnVectorMul`, `vector.c:238-241`),
where `buildRotationMatrix` builds `M = Rx(roll)*Ry(pitch)*Rz(yaw)`. Our
`makeBetaflightBoardAlignMatrix` instead negates the angles and builds the
forward ZYX matrix `Rz(-yaw)*Ry(-pitch)*Rx(-roll)`. These are identical:
`M^T = (Rx(r)*Ry(p)*Rz(y))^T = Rz(-y)*Ry(-p)*Rx(-r)`. Verified symbolically
(all 9 elements) and numerically. The archived design doc's convention was
therefore faithful to Betaflight after all.

**Cases and ground truth** (for the R3 AUnit tests to encode; `s = sqrt(2)/2`).
Net matrix `N = B * S` where `S` is the sensor permutation (rows below map
`(x,y,z)` per the BF switch) and `B` is the applied board-align matrix:

| Board-align case | Applied matrix `B` |
|---|---|
| zero (0/0/0) | identity (BF skips board align entirely) |
| roll=180, yaw=-45 (net 180° rotation — symmetric) | `[[ s, s, 0], [ s,-s, 0], [ 0, 0,-1]]` |
| roll=180 (Pavo runtime value — symmetric) | `diag(1, -1, -1)` |
| roll=90, yaw=-45 (asymmetric, transpose-sensitive) | `[[ s, 0,-s], [ s, 0, s], [ 0,-1, 0]]` |

| Enum | `S`: `(x,y,z) ->` | zero | r180+y-45 | r180 | r90+y-45 |
|---|---|---|---|---|---|
| CW0_DEG | `( x,  y,  z)` | MATCH | MATCH | MATCH | MATCH |
| CW90_DEG | `( y, -x,  z)` | MATCH | MATCH | MATCH | MATCH |
| CW180_DEG | `(-x, -y,  z)` | MATCH | MATCH | MATCH | MATCH |
| CW270_DEG | `(-y,  x,  z)` | MATCH | MATCH | MATCH | MATCH |
| CW0_DEG_FLIP | `(-x,  y, -z)` | MATCH | MATCH | MATCH | MATCH |
| CW90_DEG_FLIP | `( y,  x, -z)` | MATCH | MATCH | MATCH | MATCH |
| CW180_DEG_FLIP | `( x, -y, -z)` | MATCH | MATCH | MATCH | MATCH |
| CW270_DEG_FLIP | `(-y, -x, -z)` | MATCH | MATCH | MATCH | MATCH |

**Symmetric-case blindness (found during this verification).** Every 180°
rotation matrix is symmetric, so the first three board-align cases satisfy
`B^T = B` and cannot distinguish our convention from its transpose. The
`roll=90, yaw=-45` case was added specifically because it is asymmetric with
two non-commuting angles. Mutation runs against the production code confirmed
the coverage split:

| Deliberate break injected | zero | r180+y-45 | r180 | r90+y-45 |
|---|---|---|---|---|
| drop angle negation (negation + effective Euler-order flip) | pass | **FAIL** | pass | **FAIL** |
| transpose board-align matrix | pass | pass | pass | **FAIL** (all 8 enums) |
| swap composition order (`S*B` for `B*S`) | pass | **FAIL** (6/8) | **FAIL** (4/8) | **FAIL** (7/8) |

Without the asymmetric case, a transposed board-align convention would have
passed every check — the exact class of error R1 exists to rule out.

`makeTargetSensorAlignMatrix`'s 8 matrices reproduce the switch rows exactly
(compare `BoardAlignment.cpp:38-70` to `boardalignment.c:94-145` — same
mapping, case for case), and the composition order (`sensor first, board
second`, `BoardAlignment.cpp:76-79`) matches `boardalignment.c:142-144`.

**Durable executable record (R3).** The ground-truth matrices above are
encoded as hard-coded constants in `tests/BoardAlignment_UT/BoardAlignment_UT.ino`
(tests `betaflight_ground_truth_*`, all four board-align cases x 8 enums),
independent of the library's own builders. The pre-existing tests in that file
check internal consistency plus the sensor switch against BF; none of them
asserted the *net* `makeSensorToVehicleMatrix` output against externally
derived Betaflight ground truth under a non-trivial board-align — that is the
R1 gap the new tests close. The host harness that produced and verified these
numbers was ephemeral (scratchpad); the AUnit tests are the permanent guard.
Those tests are build-verified, host-mutation-validated, and **hardware-executed
— 24/24 on a NUCLEO-G474RE (HIL-008), 2026-07-20**, closing the last gap. (The
run used `./ci/saflash_stlink.sh` on the ST-Link rig; the test is pure software
with no peripherals, so it is not restricted to J-Link rigs.)

**Adjacent check (config path, not math).** Both runtime call sites pass
arguments in the declared `(chip, yaw, pitch, roll)` order from the
`board_align_{yaw,pitch,roll}_degrees` globals:
`../Versions/dRehmFlight_STM32_SCHED/dRehmFlight_STM32_SCHED.ino:485` and
`../Versions/dRehmFlight_STM32_SCHED_MSP_INI/dRehmFlight_STM32_SCHED_MSP_INI.ino:454`.
Whether those globals hold the intended *values* at call time (INI load
ordering, `e500a42` path) remains a Stage 6 question — the math no longer is.

### 5.2 Appendix: where alignment values come from (three sources, one parser)

A target's *effective* alignment is the composition of a chip enum and a
board-align Euler (§1). Those two halves are authored in **three** different
places, and the converter reads only the first:

| # | Source | Example | Reaches our target header? |
|---|---|---|---|
| 1 | `GYRO_1_ALIGN` in the BF unified-target `config.h` | `BETAFPVF405/config.h:122` → `CW270_DEG` | **Yes** — parsed by `code_generator.py:190-204` |
| 2 | `DEFAULT_ALIGN_BOARD_*` in the same `config.h` | `BETAFPVG473_V3/config.h:124` → `yaw = -45` | **No — silently dropped.** Not parsed at all. |
| 3 | `set align_board_*` in the board's CLI dump / diff | Pavo OEM dump → `roll = 180` | **No** — the converter reads `config.h`, never a dump. Must be set at runtime on the craft. |

Consequences that matter operationally:

- **A target header alone does not define a board's alignment.** For the Pavo,
  `CW270_DEG` is compiled in but `roll = 180` must arrive through CLI/INI
  persistence on every unit. If that value is absent — never set, failed to
  save, or lost to a load-ordering bug — the craft silently flies a
  CW270-only alignment. This is not hypothetical: campaign history records
  `align_board_roll = 180` as the fix that turned instant-crash into
  controlled hover (`doc/pavo/RESEARCH_PAVO_BF_VS_DF_PROMPT.md:53`).
- **Source 2 is a live trap for future ports.** A V3-class board would generate
  a target that looks complete and compiles clean while missing half its
  alignment. Fix the generator (`code_generator.py`), never the emitted header.
- **Bench rule:** verifying alignment on any board starts with reading back
  `align_board_roll/pitch/yaw` and confirming they match the intended values
  from sources 2 and 3 — before drawing any conclusion from sensor behavior.
- 45° mounts belong to the BetaFPV **G473_V2/V3** boards, not to the Pavo or
  the Air75; neither of those two has any 45° term in any of the three
  sources. A 45°-looking signature on Pavo hardware would therefore indicate
  contaminated config (e.g. a stray `align_board_yaw`), not a legitimate
  board-align value.

## 6. Exit criteria

- [x] R1 verified against Betaflight source for all 8 enums (§5.1).
- [x] R3 tests green **on target** — 24/24 on a NUCLEO-G474RE (HIL-008),
      2026-07-20 (§4 task 3).
- [x] Baseline ladder (§7), Flight 0 (anchor): clean 2026-07-19.
- [x] Baseline ladder (§7), Flight 1 (master): clean 2026-07-21 — master
      `d2c1ffc` is the trusted base. One non-flight-critical defect found in
      the delta (`bb dump` TX flow control; plan Stage 2 outcome).
- [x] Air75 passes R5 (proves the procedure) — 9/9, 2026-07-21.
- [x] Pavo passes R5 on both sensors — 9/9 on master `d2c1ffcf3`, 2026-07-21;
      `HW-validated`; cleared to resume rate-mode bring-up
      (`PAVO_PICO_II_BRINGUP_PLAN.md`, master-lineage builds only). The July
      "gyro anomaly" was a defective mid-investigation binary wearing a clean
      banner — see plan Stage 6 outcome.
- [ ] Ledger (R4) complete; every flown target is `HW-validated` or explicitly
      quarantined.
- [ ] Converter parses `DEFAULT_ALIGN_BOARD_*` (R6, §5.2) — required before any
      board that uses it can be ported.

## 7. Baseline, trust anchor, and the rejected revert

**Trust anchor** (deepest point provably flight-good): firmware `90f0ebc` +
core `d0a6aac` — the Air75 flight-validated build of 2026-06-29
(`tunes/AIR75_BEFH-G473.txt` provenance header).

**Core fact:** `git diff d0a6aac..ac35614 -- libraries/BoardAlignment
libraries/imu` is **empty** — the alignment math and IMU driver on master are
byte-identical to what Air75 flew. Any CW270 defect is therefore *latent* in
flight-validated code, not a regression. Rewinding the core buys nothing.

**Flight-relevant firmware delta** since the anchor (out of 30 commits on
master, linear, no merges): exactly five —
`e500a42` (runtime align_board + load ordering), `fd1cbfa` (motor reorder),
`0a86aad` (accel-cal gravity fix), `df8dabb` (blackbox recorder),
`89cef97` (motor bench test). Everything else is docs/bootloader/tooling/data.

**Baseline validation ladder** (no history rewrite; master untouched):
- *Flight 0 (control):* Air75 @ the anchor build — isolates craft state from
  software. **PASSED 2026-07-19.** The board was found still carrying the
  original validated binary (`fw_git 90f0ebcce`, built 2026-06-29T18:43:23Z),
  so it flew as-is with no reflash — the control flight used the same bytes
  that earned the anchor, not a reproduction. Bench pre-checks: `diff all`
  values byte-identical to `tunes/AIR75_BEFH-G473.txt`. Result: stable hover,
  crisp attitude response, "flies like remembered." **Craft state is therefore
  exonerated** — any Flight 1 anomaly is attributable to the five-commit delta,
  not to props/battery/frame/tune.
- *Flight 1 (baseline):* Air75 @ master `d2c1ffc` — validates the five-commit
  delta. Pass ⇒ master is the trusted base for all further work; fail ⇒ bisect
  the five. **Pending.** Binary is built and staged with clean provenance
  (`fw_git d2c1ffcf3`, core `ac35614`); the anchor build remains available as
  the rollback.

**"Revert to before Pavo" rejected** as the reset strategy: `BoardAlignment`
predates Pavo support by six weeks (`55db79111` 2026-05-22 vs `d94b1d507`
2026-07-02) and is shared with the working Air75. Reverting Pavo commits leaves
the suspect untouched and gives false comfort. The reset is to a *verified*
foundation (R1 + this ladder), not to a git state.
