# Plot fuel(t_f) from gfold2d_tf_fuel.csv (the golden-search premise).
import csv, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "gfold2d_tf_fuel.csv"
tf, fuel = [], []
for r in csv.DictReader(open(path)):
    if int(r["feasible"]):
        tf.append(float(r["tf"])); fuel.append(float(r["fuel"]))

imin = min(range(len(fuel)), key=lambda i: fuel[i])
fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(tf, fuel, "-", color="tab:blue", lw=2, label="fuel(t_f)  (fine grid)")
ax.plot(tf[imin], fuel[imin], "r*", ms=16, label=f"minimum  t_f={tf[imin]:.1f}s  {fuel[imin]:.1f}kg")
ax.axvline(tf[0], ls="--", c="gray", lw=1, label=f"feasibility floor  {tf[0]:.1f}s")
ax.set_xlabel("time of flight  t_f  [s]")
ax.set_ylabel("landing fuel  [kg]")
ax.set_title("Min-fuel vs time-of-flight is unimodal\n(Blackmore/Acikmese golden-search premise)")
ax.grid(alpha=.3); ax.legend()
fig.tight_layout(); fig.savefig("gfold2d_tf_fuel.png", dpi=120)
print("wrote gfold2d_tf_fuel.png")
