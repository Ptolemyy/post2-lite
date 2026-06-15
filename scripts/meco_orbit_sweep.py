"""Report MECO (end of ascent6) orbit vs payload for the max-payload case.

Simulates the case at a list of payload values and computes apoapsis/periapsis
altitude at the last phase-7 (ascent6) trajectory row, so we can see the exact
feasible-payload window without involving the optimizer.
"""
import os
import csv
import json
import math
import subprocess
import sys

MU = 3.986004418e14
RE = 6371000.0  # case earth_radius_m
SHELL = os.path.abspath("build/Release/post2_shell.exe")
CASE = "cases/ksp_falcon9_max_payload.json"
TMP = "build/mp_test.json"
TMP_CSV = "build/mp.csv"

base = json.load(open(CASE))
RE_case = base.get("earth_radius_m", RE)
MU_case = base.get("earth_mu_m3s2", MU)

def orbit_at_meco(payload):
    base["vehicle"]["stages"][2]["dry_mass_kg"] = float(payload)
    json.dump(base, open(TMP, "w"), indent=2, sort_keys=True)
    r = subprocess.run([SHELL, "run", "--case", TMP, "--csv", TMP_CSV, "--no-svg"],
                       capture_output=True, text=True)
    if "Simulation failed" in (r.stdout + r.stderr):
        return None
    last7 = None
    with open(TMP_CSV) as f:
        for row in csv.DictReader(f):
            if int(row["phase_index"]) == 7:
                last7 = row
    if last7 is None:
        return None
    rx = [float(last7["x_m"]), float(last7["y_m"]), float(last7["z_m"])]
    vx = [float(last7["vx_mps"]), float(last7["vy_mps"]), float(last7["vz_mps"])]
    rn = math.sqrt(sum(c*c for c in rx))
    vn = math.sqrt(sum(c*c for c in vx))
    h = [rx[1]*vx[2]-rx[2]*vx[1], rx[2]*vx[0]-rx[0]*vx[2], rx[0]*vx[1]-rx[1]*vx[0]]
    hn = math.sqrt(sum(c*c for c in h))
    rv = sum(rx[i]*vx[i] for i in range(3))
    ev = [((vn*vn - MU_case/rn)*rx[i] - rv*vx[i])/MU_case for i in range(3)]
    e = math.sqrt(sum(c*c for c in ev))
    p = hn*hn/MU_case
    peri = p/(1+e) - RE_case
    apo = (p/(1-e) - RE_case) if e < 1 else float("inf")
    incl = math.degrees(math.acos(max(-1, min(1, h[2]/hn))))
    return apo/1000, peri/1000, e, incl

for pl in [int(x) for x in sys.argv[1:]] or [19000, 20000, 21000, 21300, 21500, 21700]:
    res = orbit_at_meco(pl)
    if res is None:
        print(f"payload={pl:6d}  SUBORBITAL/impact")
    else:
        apo, peri, e, incl = res
        print(f"payload={pl:6d}  apo={apo:8.1f}km  peri={peri:8.1f}km  e={e:.4f}  incl={incl:.3f}")
