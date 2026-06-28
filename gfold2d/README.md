# gfold2d — 2D fuel-optimal powered-descent (GFOLD) solver

Stand-alone study tool. **Not** wired into post2-lite. It solves the 2D
soft-landing optimal-control problem as a second-order cone program (SOCP)
with [Clarabel](https://github.com/oxfordcontrol/Clarabel.cpp), using the
lossless-convexification formulation of

> B. Açıkmeşe, J. M. Carson, L. Blackmore, *"Lossless Convexification of
> Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal
> Control Problem,"* IEEE TCST, 2013 — http://www.larsblackmore.com/iee_tcst13.pdf

## What it does

- Minimum-fuel powered descent (paper "Problem 3"), maximising final mass.
- Non-convex thrust **lower** bound `ρ1 ≤ ‖T‖ ≤ ρ2` convexified with slack
  `σ ≥ ‖u‖`; mass dynamics linearised via `z = ln m`.
- Constraints: thrust magnitude (SOC), throttle bounds (convexified upper =
  linear, lower = 2nd-order → rotated→standard SOC), thrust-pointing cone,
  glide-slope cone, dry-mass limit, fixed boundary conditions.
- Trapezoidal collocation on `N` nodes (default 60 → 480 variables).
- Sweeps the time-of-flight `t_f` (the one remaining non-convex parameter) to
  recover the global fuel optimum, and **reports the per-solve time**.

## Formulation → Clarabel

Decision vector per node `k` (stride 8): `[rx, ry, vx, vy, z, ux, uy, σ]`.
Clarabel solves `min ½xᵀPx + qᵀx  s.t.  Ax + s = b, s ∈ K` with `P = 0`,
`q` selecting `−z_N` (maximise landing mass). Cones, in order: one `ZeroCone`
(dynamics + boundary equalities), one `NonnegativeCone` (linear inequalities),
then `2N` `SecondOrderCone(3)` blocks (thrust magnitude + throttle lower bound).

## Build

Prereqs: MSVC, CMake ≥ 3.20, Rust toolchain, vcpkg (for Eigen3).

```sh
# 1. Clarabel.cpp + its Rust submodule (already vendored under external/):
#    git clone --recurse-submodules https://github.com/oxfordcontrol/Clarabel.cpp.git external/Clarabel.cpp

# 2. Build the Clarabel Rust C-wrapper once (produces clarabel_c.dll[.lib]):
cd external/Clarabel.cpp/rust_wrapper && cargo build --release && cd -

# 3. Configure + build this tool:
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 \
      -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake \
      -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build --config Release
```

## Run

```sh
cd build/Release
./gfold2d.exe            # or:  ./gfold2d.exe <N_nodes>
python ../../plot.py     # optional: writes gfold2d_trajectory.png
```

Writes `gfold2d_trajectory.csv` for the optimal `t_f`.

## Result (reference run, N=60, 480 vars, ~1025 constraints)

- Feasibility boundary near `t_f ≈ 31 s`; fuel optimum at **`t_f = 35 s`,
  ~190.6 kg propellant** for the default Mars-lander parameters.
- Classic **max–min–max** throttle profile (0.8 → 0.2 → 0.8); touchdown at the
  target with zero velocity; trajectory stays above the 10° glide-slope cone.
- **Solve time ≈ 3–5 ms per SOCP** (Clarabel internal), ~18 interior-point
  iterations; infeasible `t_f` detected in ~3 ms. The 25-point `t_f` sweep
  totals ~0.1 s.

## Ignition time & t_f search (low-overhead)

The two online decisions — **when to ignite** and **what t_f to fly** — are found
without a brute-force 2-D grid, by exploiting monotonicity / unimodality:

- **t_f (one state):** feasibility in t_f is an interval, so a *bisection* finds
  the minimum feasible t_f; `fuel(t_f)` is unimodal, so *golden section* finds
  the optimum. ~20 solves to ~0.01 s tolerance (`find_tf_optimal`).
- **Latest-safe ignition (the "point of no return"):** feasibility decreases
  monotonically along the ballistic coast, so *bisection on the coast time*
  finds the last instant a feasible landing still exists. ~25 solves
  (`find_ignition_time`). The coast state is propagated analytically and a free
  kinematic screen (`kinematic_possible`) rejects hopeless states before any SOCP.
- **Fuel-optimal ignition (joint t_ig × t_f):** nested golden(t_ig) × golden(t_f).
  ~180 solves vs **625** for a 25×25 grid (`fuel_optimal_ignition`).

Note: with a non-zero min-throttle (ρ1>0) the engine cannot coast once lit, so
the *fuel-optimal* ignition is **early/high** (less impact speed to cancel), not
the latest-feasible one — the latter is a safety deadline, not the fuel optimum.

For real flight: run the plan once high up, then **trigger ignition on a
precomputed threshold crossing** (zero online SOCP), and re-plan periodically
warm-started. Fixing the sparsity of `A` and updating only `b` between re-plans
pushes a single solve toward sub-millisecond.

Parameters live in the `Params` struct at the top of `src/gfold2d.cpp`.
