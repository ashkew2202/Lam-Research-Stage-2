# Step by step FT-01 calibration. Needs rig_daq.py running. Nothing moves until Enter is pressed.
#
#   python3 tools/cal_session.py runs/cal_runs_20261011.csv
#
# For each run:
#   1. refill the sump from TK-01 until the float trips
#   2. type the TK-01 scale reading
#   3. timed P-02 run (firmware CAL command counts the pulses)
#   4. type the scale reading again
# One row is added to the csv per run and the fit is printed after each run.

import csv
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from daq_client import Daq

PLAN = [(100, 40), (85, 47), (70, 55), (60, 65), (50, 85)]  # P-02 duty %, seconds. about 1.5 kg each
COLS = ["run", "pulses", "time_s", "mass_g", "duty", "before_g", "after_g", "note"]
SCALE_MAX_G = 5000  # our scale is not accurate above 5 kg


def ask(prompt):
    while True:
        s = input(prompt).strip()
        if not s:
            return None
        try:
            return float(s)
        except ValueError:
            print("  type a number in grams, or leave blank")


def refill(d):
    d.send("RESET", "MANUAL", "SOV 1", "P 1 100")
    t0 = time.time()
    tripped = False
    try:
        while time.time() - t0 < 100 and not tripped:
            time.sleep(0.2)
            tripped = d.frame().get("lsh") == "1"
    finally:
        d.send("P 1 0", "SOV 0", "RESET")
    print(f"  float {'tripped' if tripped else 'DID NOT TRIP in 100 s'} after {time.time() - t0:.0f} s. P-01 off, SOV-01 closed")
    time.sleep(3)


def cal_run(d, secs, duty):
    path = os.path.join(d.dir, "cal.csv")
    n0 = len(list(csv.DictReader(open(path))))
    d.send("RESET", f"CAL {secs:g} {duty:g}")
    t0 = time.time()
    while time.time() - t0 < secs + 30:
        rows = list(csv.DictReader(open(path)))
        if len(rows) > n0:
            return rows[-1]
        time.sleep(0.5)
    return None


def main():
    out = sys.argv[1]
    fresh = not os.path.exists(out) or os.path.getsize(out) == 0
    done = 0 if fresh else len(list(csv.DictReader(open(out))))
    f = open(out, "a", newline="")
    w = csv.writer(f)
    if fresh:
        w.writerow(COLS)
        f.flush()
    d = Daq()
    print("logger session:", d.dir)
    i = 0
    while True:
        duty, secs = PLAN[i % len(PLAN)]
        s = input(f"\nRun {done + 1}: P-02 {duty}% for {secs} s.  Press Enter to accept (or type q to finish): ").strip().lower()
        if s in ("q", "quit"):
            break
        try:
            duty, secs = (float(x) for x in s.split())  # can type other values, e.g. "100 40"
        except ValueError:
            i += 1
        input("  Enter to refill the sump (SOV-01 opens, P-01 100% until the float trips): ")
        refill(d)
        before = ask("  TK-01 scale BEFORE, g: ")
        while before is not None and before > SCALE_MAX_G - 1700:
            before = ask(f"  {before:g} g is too full to end under {SCALE_MAX_G} g. Take some water out of TK-01, then BEFORE, g: ")
        input(f"  Enter to run P-02 at {duty:g}% for {secs:g} s: ")
        c = cal_run(d, secs, duty)
        after = ask("  Wait for the return line to stop dripping. TK-01 scale AFTER, g (blank = no reading): ")
        done += 1
        note = ""
        if c is None:
            note = "INVALID: no result from the ESP32"
        elif before is None or after is None:
            note = "INVALID: no scale reading"
        elif after > SCALE_MAX_G:
            note = f"INVALID: scale ended above {SCALE_MAX_G} g"
        mass = "" if note else f"{after - before:g}"
        w.writerow([f"R{done}", c["pulses"] if c else "", c["time_s"] if c else "", mass, f"{duty / 100:g}",
                    "" if before is None else f"{before:g}", "" if after is None else f"{after:g}", note])
        f.flush()
        if note:
            print("  " + note)
            continue
        t, n = float(c["time_s"]), float(c["pulses"])
        print(f"  {n:.0f} pulses in {t:.3f} s = {n / t:.2f} Hz.  {mass} g = {float(mass) / t * 0.06:.3f} L/min")
        fit = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "cal_fit.py"), out],
                             capture_output=True, text=True).stdout
        print(fit.split("to load")[0])


if __name__ == "__main__":
    main()
