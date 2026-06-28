# Quick visualisation of a gfold2d_trajectory.csv produced by gfold2d.exe
import csv, sys, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "gfold2d_trajectory.csv"
rows = list(csv.DictReader(open(path)))
t   = [float(r["t"]) for r in rows]
rx  = [float(r["rx"]) for r in rows]
ry  = [float(r["ry"]) for r in rows]
thr = [float(r["throttle"]) for r in rows]
spd = [math.hypot(float(r["vx"]), float(r["vy"])) for r in rows]

fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))

# Trajectory + glide-slope cone (10 deg) + thrust direction quivers
ax[0].plot(rx, ry, "-o", ms=3, color="tab:blue", label="trajectory")
xm = max(rx)
ax[0].plot([0, xm], [0, math.tan(math.radians(10))*xm], "r--", lw=1, label="glide slope 10 deg")
for r in rows[::4]:
    x, y = float(r["rx"]), float(r["ry"])
    ux, uy = float(r["ux"]), float(r["uy"])
    s = 30.0
    ax[0].arrow(x, y, ux*s, uy*s, color="tab:orange", head_width=18, length_includes_head=True)
ax[0].set_xlabel("downrange x [m]"); ax[0].set_ylabel("altitude y [m]")
ax[0].set_title("Powered-descent trajectory"); ax[0].legend(); ax[0].grid(alpha=.3)
ax[0].set_aspect("equal", adjustable="datalim")

ax[1].plot(t, thr, "-o", ms=3, color="tab:green")
ax[1].axhline(0.2, ls="--", c="gray"); ax[1].axhline(0.8, ls="--", c="gray")
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("throttle [-]")
ax[1].set_title("Throttle (bounds 0.2 / 0.8)"); ax[1].grid(alpha=.3); ax[1].set_ylim(0, 1)

ax[2].plot(t, spd, "-o", ms=3, color="tab:purple")
ax[2].set_xlabel("time [s]"); ax[2].set_ylabel("speed [m/s]")
ax[2].set_title("Speed magnitude"); ax[2].grid(alpha=.3)

fig.tight_layout()
out = "gfold2d_trajectory.png"
fig.savefig(out, dpi=120)
print("wrote", out)
