"""Test whether the orbit-feasibility (target-only) solve succeeds at a fixed
payload, with optionally widened steering authority. Used to find a feasible
warm-start for the max-payload continuation.

Usage: python try_target_solve.py <payload_kg> [c1_lo] [qp]
"""
import os
import json
import subprocess
import sys

SHELL = os.path.abspath("build/Release/post2_shell.exe")
CASE = "cases/ksp_falcon9_max_payload.json"
TMP = "build/mp_target.json"

payload = float(sys.argv[1]) if len(sys.argv) > 1 else 20000.0
c1_lo = float(sys.argv[2]) if len(sys.argv) > 2 else -0.5
qp = sys.argv[3] if len(sys.argv) > 3 else "active-set"

c = json.load(open(CASE))
c["vehicle"]["stages"][2]["dry_mass_kg"] = payload

# Widen elevation c1/c2 authority on the powered ascent phases and disable the
# payload variable so the solve is pure orbit-feasibility at fixed payload.
for v in c["optimization"]["variables"]:
    p = v["path"]
    if p.endswith("steering_model.elevation.c1"):
        v["min_value"] = c1_lo
        v["max_value"] = 0.3
    if p.endswith("steering_model.elevation.c2"):
        v["min_value"] = -0.01
        v["max_value"] = 0.01
    if p == "vehicle.stages[2].dry_mass_kg":
        v["enabled"] = False

json.dump(c, open(TMP, "w"), indent=2, sort_keys=True)

r = subprocess.run(
    [SHELL, "optimize", "--case", TMP, "--qp-solver", qp,
     "--max-iterations", "150", "--no-csv", "--no-svg"],
    capture_output=True, text=True)
out = r.stdout + r.stderr
for line in out.splitlines():
    if any(k in line for k in ("found_feasible", "max_constraint_violation",
            "metric.phases[7].peri", "metric.phases[7].apoap",
            "metric.phases[7].incl", "iters=")):
        print(line)
