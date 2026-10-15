# Q2.1: flow vs BV-02 position, from runs/q21_valve_sweep.csv
#
#   python3 tools/plot_valve.py q21_valve.png
#
# filled markers = flow by weighing TK-01, open markers = FT-01 with the final calibration

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

A, B = 0.009093, 0.06022  # FT-01 calibration, Q = A*f + B

here = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.DictReader(open(os.path.join(here, "..", "runs", "q21_valve_sweep.csv"))))
out = sys.argv[1] if len(sys.argv) > 1 else "q21_valve.png"

fig, ax = plt.subplots(figsize=(7, 4.5))
for duty, marker, color in ((1.0, "o", "C0"), (0.6, "s", "C1")):
    pts = sorted([r for r in rows if float(r["p02_duty"]) == duty], key=lambda r: float(r["valve_pct"]))
    x = [float(r["valve_pct"]) for r in pts]
    weighed = [float(r["scale_q_lpm"]) for r in pts]
    ft = [A * float(r["f_hz"]) + B if float(r["f_hz"]) > 0 else 0 for r in pts]
    ax.plot(x, weighed, marker + "-", color=color, label=f"P-02 at {duty * 100:.0f}% duty, weighed")
    ax.plot(x, ft, marker, color=color, mfc="none", ms=11, label=f"P-02 at {duty * 100:.0f}% duty, FT-01")
    for xi, yi in zip(x, weighed):
        if yi > 0:
            offset = (-12, 16) if xi == 70 else (6, -14)  # 70% and 75% are close together
            ax.annotate(f"{yi:.2f}", (xi, yi), xytext=offset, textcoords="offset points")

ax.annotate("no flow at 25%", (25, 0), xytext=(8, 4), textcoords="offset points")
ax.set_xlabel("BV-02 position (% open)")
ax.set_ylabel("Flow (L/min)")
ax.set_title("Q2.1 Valve position vs flow rate")
ax.set_xlim(15, 108)
ax.set_ylim(-0.1, 2.6)
ax.grid(True)
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
fig.savefig(out, dpi=150)
print("saved", out)
