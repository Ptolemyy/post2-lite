"""Build a bilinear-tangent steering variant of the optimized max-payload case.

Converts the vacuum upper-stage phases (ascent4..ascent6 = phases 5,6,7) from
generic_poly to bilinear_tangent steering, seeding each phase's tan-slope `a`
from the optimized trajectory's actual elevation profile so the starting point
is near the generic_poly optimum. Tangent `b` is re-anchored by continuity at
each phase boundary. Optimization variables for those phases switch from
elevation c1/c2 to tangent a/a_dot.
"""
import os, csv, json, math, subprocess

SHELL = os.path.abspath("build/Release/post2_shell.exe")
OPT = "cases/ksp_falcon9_max_payload_optimized.json"
OUT = "cases/ksp_falcon9_max_payload_tangent.json"
TAN_PHASES = [5, 6, 7]  # ascent4, ascent5, ascent6 (vacuum, pitched over)

c = json.load(open(OPT))

# Sim the optimized case to read the elevation profile per phase.
subprocess.run([SHELL, "run", "--case", OPT, "--csv", "build/optprof.csv", "--no-svg"],
               capture_output=True, text=True)
rows = list(csv.DictReader(open("build/optprof.csv")))

def elevation_deg(row):
    r = [float(row['x_m']), float(row['y_m']), float(row['z_m'])]
    d = [float(row['engine_direction_eci_x']), float(row['engine_direction_eci_y']),
         float(row['engine_direction_eci_z'])]
    rn = math.sqrt(sum(x*x for x in r))
    up = [x/rn for x in r]
    return math.degrees(math.asin(max(-1, min(1, sum(d[i]*up[i] for i in range(3))))))

def phase_rows(idx):
    return [row for row in rows if int(row['phase_index']) == idx]

for idx in TAN_PHASES:
    pr = phase_rows(idx)
    if len(pr) < 2:
        continue
    t0, t1 = float(pr[0]['time_s']), float(pr[-1]['time_s'])
    e0, e1 = elevation_deg(pr[0]), elevation_deg(pr[-1])
    dur = max(1.0, t1 - t0)
    a_seed = (math.tan(math.radians(e1)) - math.tan(math.radians(e0))) / dur
    ph = c['phases'][idx]
    ph['steering_model']['type'] = 'bilinear_tangent'
    ph['steering_model']['tangent'] = {
        'a': a_seed, 'a_dot': 0.0,
        'b': math.tan(math.radians(e0)), 'b_dot': 0.0,
        't_offset_s': 0.0, 'continuity': True,
    }
    # Drop continuity on the (now-unused) elevation poly to avoid double-anchor.
    ph['steering_model']['elevation']['continuity'] = False
    print(f"phase {idx} ({ph['name']}): elev {e0:.1f}->{e1:.1f}deg over {dur:.0f}s, a_seed={a_seed:.5f}")

# Swap optimization variables: elevation c1/c2 -> tangent a/a_dot on tan phases.
newvars = []
for v in c['optimization']['variables']:
    p = v['path']
    ph_idx = None
    if p.startswith('phases['):
        ph_idx = int(p[len('phases['):p.index(']')])
    if ph_idx in TAN_PHASES and '.elevation.' in p:
        continue  # remove elevation poly vars for tangent phases
    newvars.append(v)
for idx in TAN_PHASES:
    newvars.append({'path': f'phases[{idx}].steering_model.tangent.a',
                    'enabled': True, 'min_value': -0.02, 'max_value': 0.005})
    newvars.append({'path': f'phases[{idx}].steering_model.tangent.a_dot',
                    'enabled': True, 'min_value': -0.0002, 'max_value': 0.0002})
c['optimization']['variables'] = newvars

# Seed payload a touch below the generic-poly optimum so continuation ramps up.
c['vehicle']['stages'][2]['dry_mass_kg'] = 26000.0
for v in c['optimization']['variables']:
    if v['path'] == 'vehicle.stages[2].dry_mass_kg':
        v['min_value'] = 22000.0; v['max_value'] = 32000.0
c['name'] = 'Falcon 9 ascent - maximize payload (bilinear tangent)'
json.dump(c, open(OUT, 'w'), indent=2, sort_keys=True)
print('wrote', OUT)
