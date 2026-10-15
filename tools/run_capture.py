# Copies one run out of the logger session into its own folder runs/<time>_<name>/
#
#   python3 tools/run_capture.py take_2
#
# Start it before pressing the switch. It waits for the run to start, follows it until
# COMPLETE or STOPPED, then copies that part of raw.log, telemetry.csv and events.csv.
# The logger keeps running.

import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from daq_client import RUNS, Daq

name = sys.argv[1]
d = Daq()
while d.frame().get("state") in ("IDLE", "MANUAL", "CAL"):
    time.sleep(0.2)
t0 = time.time() - 2
print("started", d.frame().get("state"), flush=True)
t_end = None
while time.time() - t0 < 900:
    st = d.frame().get("state")
    if st in ("COMPLETE", "STOPPED") and t_end is None:
        t_end = time.time()
    if t_end and time.time() - t_end > 10:
        break
    time.sleep(0.2)
t1 = time.time()
out = os.path.join(RUNS, time.strftime("%Y%m%d_%H%M%S", time.localtime(t0)) + "_" + name)
os.makedirs(out)
with open(os.path.join(d.dir, "raw.log"), errors="replace") as f, open(os.path.join(out, "raw.log"), "w") as o:
    for ln in f:
        try:
            if t0 - 5 <= float(ln.split(" ", 1)[0]) <= t1:
                o.write(ln)
        except ValueError:
            pass
for fn in ("telemetry.csv", "events.csv"):
    rows = list(csv.reader(open(os.path.join(d.dir, fn))))
    with open(os.path.join(out, fn), "w", newline="") as o:
        w = csv.writer(o)
        w.writerow([r for r in rows if r and r[0] == "host_time"][-1])  # use the last header, the first one can be garbled
        w.writerows(r for r in rows[1:] if r and r[0].replace(".", "", 1).isdigit() and t0 - 5 <= float(r[0]) <= t1)
print(out, "state at end:", d.frame().get("state"))
