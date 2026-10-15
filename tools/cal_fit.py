# FT-01 calibration: straight line fit Q = a*f + b from the weighed runs
#
#   python3 tools/cal_fit.py runs/cal_runs_20261011.csv cal.png
#
# csv columns: run, pulses, time_s, mass_g
# mass_g is the water collected in TK-01 (scale after - scale before), 1 g = 1 mL

import csv
import sys

import numpy as np

runs = []
for r in csv.DictReader(open(sys.argv[1])):
    if not r["pulses"] or not r["mass_g"]:
        continue  # aborted run
    runs.append((r["run"], float(r["pulses"]), float(r["time_s"]), float(r["mass_g"])))

name = [r[0] for r in runs]
N = np.array([r[1] for r in runs])
t = np.array([r[2] for r in runs])
V = np.array([r[3] for r in runs]) / 1000.0  # litres

f = N / t        # Hz
Q = V / t * 60   # L/min

a, b = np.polyfit(f, Q, 1)
Qfit = a * f + b
err = (Qfit - Q) / Q * 100
r2 = 1 - np.sum((Q - Qfit) ** 2) / np.sum((Q - Q.mean()) ** 2)

print("run  pulses  time_s   f_Hz    V_L    Q_Lmin  Qfit   err_%")
for i in range(len(runs)):
    print(f"{name[i]:<4} {N[i]:6.0f}  {t[i]:6.3f}  {f[i]:6.2f}  {V[i]:.3f}  {Q[i]:.3f}   {Qfit[i]:.3f}  {err[i]:+.2f}")

print()
print(f"a = {a:.6f} L/min per Hz")
print(f"b = {b:.5f} L/min")
print(f"R^2 = {r2:.4f}")
print(f"max error = {np.max(np.abs(err)):.2f} %")
print(f"avg error = {np.mean(np.abs(err)):.2f} %")
print(f"pulses per litre from the slope = {60 / a:.0f} (datasheet 5880)")
print()
print("to load into the ESP32:")
print(f"SET a {a:.6f}")
print(f"SET b {b:.5f}")
print("SAVE")

if len(sys.argv) > 2:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x = np.linspace(0, f.max() * 1.1, 50)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7), gridspec_kw={"height_ratios": [3, 1.3]})
    ax1.plot(f, Q, "o", label="weighed runs")
    ax1.plot(x, a * x + b, label=f"fit: Q = {a:.6f} f + {b:.4f}")
    ax1.plot(x, x / 98, "--", label="datasheet: Q = f / 98")
    ax1.set_xlabel("FT-01 pulse frequency f (Hz)")
    ax1.set_ylabel("Actual flow Q (L/min)")
    ax1.set_title(f"FT-01 calibration, $R^2$ = {r2:.4f}")
    ax1.grid(True)
    ax1.legend()

    ax2.plot(Q, err, "o")
    ax2.axhline(0, color="k", lw=0.8)
    ax2.set_ylim(-1, 1)
    ax2.set_xlabel("Actual flow Q (L/min)")
    ax2.set_ylabel("Fit error (%)")
    ax2.grid(True)

    fig.tight_layout()
    fig.savefig(sys.argv[2], dpi=150)
    print("saved", sys.argv[2])
