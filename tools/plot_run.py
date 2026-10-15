# Flow vs time plot for one recorded run (Case 1 + Case 2)
#
#   python3 tools/plot_run.py runs/20261011_203927_paper_duty_take_2 case_trend.png
#
# top: FT-01 flow in mL/s (L/min on the right axis)
# middle: litres returned to TK-01
# bottom: pump duty, with the time the float switch was high shaded

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

run_dir = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else "case_trend.png"

rows = list(csv.DictReader(open(os.path.join(run_dir, "telemetry.csv"))))

# time zero = switch press (first row that is not IDLE)
start = 0
while rows[start]["state"] == "IDLE":
    start += 1
t0 = float(rows[start]["t_ms"])
rows = rows[max(start - 10, 0):]

t = [(float(r["t_ms"]) - t0) / 1000 for r in rows]
q = [float(r["q_lpm"]) for r in rows]
vol = [float(r["v_tot_l"]) for r in rows]
d1 = [float(r["d1"]) * 100 for r in rows]
d2 = [float(r["d2"]) * 100 for r in rows]
d3 = [float(r["d3"]) * 100 for r in rows]
lsh = [int(r["lsh"]) for r in rows]
state = [r["state"] for r in rows]

# where each part of the sequence starts
labels = {"C1_START": "Case 1", "C2_OPEN_BV01": "BV-01", "C2_FLUSH": "Flush", "COMPLETE": "OFF"}
marks = []
for i in range(1, len(rows)):
    if state[i] != state[i - 1] and state[i] in labels:
        marks.append((t[i], labels[state[i]]))

i_c1_end = state.index("C2_ISOLATE")
i_stop = state.index("COMPLETE")
c1 = [q[i] for i in range(len(rows)) if state[i] == "C1_RUN"][60:]  # skip the first 6 s (pump ramp)
c1_mean = sum(c1) / len(c1)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

ax1.plot(t, [x * 1000 / 60 for x in q])
ax1.set_ylabel("FT-01 flow (mL/s)")
ax1.set_ylim(0, 50)
ax1r = ax1.twinx()
ax1r.set_ylim(0, 50 * 0.06)
ax1r.set_ylabel("L/min")
ax1.set_title("Case 1 + Case 2 run, 11 Oct 2026 (take 2)")

ax2.plot(t, vol)
ax2.set_ylabel("Returned to TK-01 (L)")
ax2.set_ylim(-0.3, 7)
ax2.annotate(f"Case 1: {vol[i_c1_end]:.2f} L", (t[i_c1_end], vol[i_c1_end]), xytext=(-90, 10), textcoords="offset points")
ax2.annotate(f"total: {vol[-1]:.2f} L", (t[i_stop], vol[-1]), xytext=(-85, 8), textcoords="offset points")

ax3.plot(t, d1, label="P-01")
ax3.plot(t, d2, "--", label="P-02")
ax3.plot(t, d3, "-.", label="P-03")
high = [t[i] for i in range(len(rows)) if lsh[i]]
if high:
    ax3.axvspan(high[0], high[-1], color="gray", alpha=0.3, label="LS-01 high")
ax3.set_ylabel("Pump duty (%)")
ax3.set_ylim(-5, 110)
ax3.set_xlabel("Time from switch press (s)")
ax3.legend(loc="center right")

for ax in (ax1, ax2, ax3):
    ax.grid(True)
    for x, _ in marks[1:]:
        ax.axvline(x, color="k", ls=":", lw=1)
for k, (x, name) in enumerate(marks):
    ax1.text(x + 1, 46 - 4 * (k % 2), name)

fig.tight_layout()
fig.savefig(out, dpi=150)
print("saved", out)
print(f"Case 1 mean flow {c1_mean:.3f} L/min, Case 1 volume {vol[i_c1_end]:.3f} L, total {vol[-1]:.3f} L")
