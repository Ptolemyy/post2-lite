"""Reformulate the Falcon 9 max-payload case to the burn-to-depletion model.

Task-1 formulation change:
  * The final powered phase (ascent6 = phases[7]) terminates on upper-stage
    propellant depletion instead of a free duration. All propellant is burned
    by construction, so insertion and depletion coincide and the ~23 t residual
    disappears.
  * Powered ascent phases run at full throttle (max payload wants minimum
    gravity loss; max-Q is not constrained here).
  * Payload (vehicle.stages[2].dry_mass_kg) is the objective variable, seeded at
    a feasible guess and allowed up to 30 t.
  * The insertion orbit is pinned by apoapsis / periapsis / inclination range
    constraints; the redundant terminal altitude/speed targets are dropped.
"""
import json
import sys

src = "cases/ksp_falcon9_max_payload.json"
case = json.load(open(src, encoding="utf-8"))

DEPLETION_EPS_KG = 50.0
PAYLOAD_PATH = "vehicle.stages[2].dry_mass_kg"
PAYLOAD_SEED_KG = 16000.0
PAYLOAD_MAX_KG = 30000.0
FINAL_PHASE = 7  # ascent6

phases = case["phases"]

# 1. Final powered phase burns to depletion.
phases[FINAL_PHASE]["termination"] = {
    "type": "propellant_mass_kg",
    "comparison": "<=",
    "value": DEPLETION_EPS_KG,
}

# 2. Full throttle on every powered ascent phase (ascent1..ascent6 = 2..7).
for i in range(2, 8):
    phases[i]["throttle_model"]["c0"] = 1.0
    phases[i]["throttle_model"]["c1"] = 0.0
    phases[i]["throttle_model"]["c2"] = 0.0

# 3. Seed payload at a feasible guess.
case["vehicle"]["stages"][2]["dry_mass_kg"] = PAYLOAD_SEED_KG

opt = case["optimization"]

# 4. Variables: drop the final-phase duration var, widen the payload bound.
new_vars = []
for v in opt["variables"]:
    if v["path"] == f"phases[{FINAL_PHASE}].duration_s":
        continue  # final phase no longer time-terminated
    if v["path"] == PAYLOAD_PATH:
        v = dict(v)
        v["enabled"] = True
        v["min_value"] = 200.0
        v["max_value"] = PAYLOAD_MAX_KG
    new_vars.append(v)
opt["variables"] = new_vars

# 5. Targets: pin the insertion orbit with apo/peri/inclination ranges only.
def target(metric, lo, hi, nominal, weight):
    return {
        "metric": metric,
        "mode": "range",
        "min_value": lo,
        "max_value": hi,
        "value": nominal,
        "weight": weight,
        "scope": "terminal",
        "phase_index": -1,
    }

opt["targets"] = [
    target(f"phases[{FINAL_PHASE}].periapsis_altitude_m", 180000.0, 250000.0, 200000.0, 2),
    target(f"phases[{FINAL_PHASE}].apoapsis_altitude_m", 180000.0, 280000.0, 200000.0, 2),
    target(f"phases[{FINAL_PHASE}].inclination_deg", 28.3, 28.8, 28.562, 1),
]

case["name"] = "Falcon 9 ascent - maximize payload (burn to depletion)"

json.dump(case, open(src, "w", encoding="utf-8"), indent=2, sort_keys=True)
print(f"wrote {src}")
print(f"  final phase termination: {phases[FINAL_PHASE]['termination']}")
print(f"  payload seed: {PAYLOAD_SEED_KG} kg, bound [200, {PAYLOAD_MAX_KG}]")
print(f"  variables: {len(opt['variables'])}, targets: {len(opt['targets'])}")
