"""Plots the elbow limit that depends on the shoulder angle (the table below is the one measured on the arm).

    python plot_elbow_limit.py   ->  elbow_limit_by_shoulder.png next to this file   (needs matplotlib)
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
# rows: (shoulder, elbow_min, elbow_max); the 110-115 / 115-120 rows are taken as two rows, with 115 using the tighter (100) minimum
T = [(40,135,180),(60,120,180),(70,105,180),(80,100,180),(90,100,180),(100,100,180),(110,100,180),(115,100,180),
     (120,95,180),(130,95,170),(140,95,170),(150,90,160),(160,100,150),(170,120,145)]
xs=[r[0] for r in T]; mn=[r[1] for r in T]; mx=[r[2] for r in T]
# conservative envelope between two rows: the tighter of the two neighbours (stepwise: shoulder x..x' uses max of mins, min of maxes)
ex, emn, emx = [], [], []
for a, b in zip(T, T[1:]):
    lo, hi = max(a[1], b[1]), min(a[2], b[2])
    ex += [a[0], b[0]]; emn += [lo, lo]; emx += [hi, hi]
fig, ax = plt.subplots(figsize=(11, 6.5))
ax.fill_between(ex, emn, emx, color="#2f81f7", alpha=0.25, label="allowed (tighter of the two neighbouring rows) - what the firmware would use")
ax.plot(xs, mn, "o--", color="#d29922", lw=1.2, label="min, straight line between your rows")
ax.plot(xs, mx, "s--", color="#3fb950", lw=1.2, label="max, straight line between your rows")
for x, lo, hi in T:
    ax.annotate(str(lo), (x, lo), textcoords="offset points", xytext=(0, -13), ha="center", fontsize=8, color="#9a6700")
    ax.annotate(str(hi), (x, hi), textcoords="offset points", xytext=(0, 6), ha="center", fontsize=8, color="#1a7f37")
ax.axvspan(30, 40, color="#cf222e", alpha=0.08); ax.axvspan(170, 180, color="#cf222e", alpha=0.08)
ax.axhline(80, color="#cf222e", lw=0.8, ls=":"); ax.axhline(180, color="#cf222e", lw=0.8, ls=":")
ax.set_xlim(30, 180); ax.set_ylim(70, 190)
ax.set_xlabel("Shoulder angle (deg)"); ax.set_ylabel("Elbow angle (deg)")
ax.set_title("Elbow limit depending on the shoulder angle (outside the blue area the elbow can break)")
ax.set_xticks(range(40, 171, 10)); ax.grid(alpha=0.3); ax.legend(loc="lower left", fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "elbow_limit_by_shoulder.png"), dpi=110)
print("ok")
