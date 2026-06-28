# gridfin_test — aero-glide MPC + attitude PID landing guidance (kRPC, C++)

Steers a descending, **unpowered** Falcon 9 first stage to a target lat/lon/alt
using only the four grid fins (90° apart) as actuators, over kRPC on ports
**30000 / 30001**. The goal is to arrive **over the target at the target
altitude (default 1 km) with horizontal velocity → 0 and the nose vertical**.

Controls the active vessel — load the booster's first stage and switch to it.
Grid fins are deployed via the **BRAKES** action group (remap brakes → grid-fin
deploy in-game), matching the kOS reference setup.

## Build

```
cmake --preset vcpkg
cmake --build build --target gridfin_test --config Release
```

Output: `build/gridfin_test/Release/gridfin_test.exe`. Self-contained at runtime
— links only `krpc::krpc`; the parent aero-table reader is compiled in, no
Clarabel/post2_core DLL needed.

## Run

```
gridfin_test.exe                       # connect 127.0.0.1:30000/30001, target LZ-1, 1 km
gridfin_test.exe --alt 1000 --aoa-max 10
gridfin_test.exe --mass 25000 --sref 10.52
gridfin_test.exe --dry-run             # offline sanity sim, no KSP needed
```

The default aero table is **`aero_table_booster.csv`** — the *booster flying alone
with its grid fins deployed* (S_ref ≈ 10.52 m², the stage-1 recovery config). It
is copied next to `gridfin_test.exe` at build time and resolved relative to the
executable, so the working directory does not matter. Do **not** point `--table`
at `aero_table_full.csv` (that is the full stack, no fins). Override only to test
a different table.

Keys (live): **`1`** toggle grid fins (brakes) · **`c`** control on/off
(auto-engages once fins are deployed) · **`q`** quit.

## How it works — two nested loops

- **Outer loop** (`aero_glide_mpc.hpp/.cpp`): a sampling-based (cross-entropy,
  CEM) **nonlinear MPC**. Each replan it rolls out many candidate
  angle-of-attack steering profiles through the point-mass aero-glide dynamics —
  using the parent project's **real CD/CL aero table** (`post2::aero::AeroTable`,
  vs Mach & α) plus a density profile sampled from kRPC — scores them on a
  terminal cost (horizontal miss + horizontal speed + terminal AoA, with effort
  and smoothness penalties), keeps the elite fraction, and refits the sampling
  distribution. The first node of the converged mean becomes the desired nose
  direction. Re-planned every `--mpc-period` (default 1 s), so model error is
  continually corrected. Below the target altitude it concludes the arrival and
  holds vertical (a booster can't hover).

  When the booster is essentially over the target (horizontal miss < ~60 m, with
  hysteresis) it stops steering and **holds vertical** — borrowed from the kOS
  reference guidance — so it does not keep disturbing the attitude chasing a
  sub-metre miss. If a plan does not reach the target altitude within its horizon
  it falls back to a simple **proportional lift-steering law** (the kOS
  predictor-corrector: tilt retrograde toward the miss-correcting direction by an
  AoA ∝ miss).

- **Inner loop**: the **kRPC autopilot** (`vessel.auto_pilot()`), fed the MPC's
  desired nose direction in the body-fixed frame via `set_target_direction`. KSP's
  own attitude controller knows the correct control-sign mapping and reads its own
  body rates, so this sidesteps the roll-sign / measurement-frame pitfalls of a
  hand-rolled PID (an earlier own-PID version diverged in roll on this low-roll-MOI
  airframe). Roll is left uncontrolled (`target_roll = NaN`) so the autopilot never
  fights the very responsive roll axis.

CSV telemetry → `gridfin_log.csv` (one flushed row per step): control flag, aim
mode (MPC / PROP / CONVRG / ARRIVED / LOWSPD), altitude, horizontal miss/speed, q,
Mach, commanded AoA, autopilot heading error, and the MPC's predicted terminal
miss/speed.

> Tuning still required live: the CD/CL table, S_ref and mass set the achievable
> steering authority; the MPC cost weights (`pos_scale`, `vh_scale`, AoA cap) and
> the PID gains have only been exercised in the offline `--dry-run` sim.
