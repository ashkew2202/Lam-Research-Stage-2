# Small helper to talk to a running rig_daq.py session: send commands and read the latest values

import glob
import json
import os
import time

RUNS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs"))


def session():
    # newest session folder
    return os.path.dirname(max(glob.glob(os.path.join(RUNS, "2*_*", "raw.log")), key=os.path.getmtime))


class Daq:
    def __init__(self, path=None):
        self.dir = path or session()

    def send(self, *cmds):
        with open(os.path.join(self.dir, "cmd.txt"), "a") as f:
            for c in cmds:
                f.write(c + "\n")

    def frame(self):
        with open(os.path.join(self.dir, "latest.json")) as f:
            return json.load(f)

    def watch(self, secs, keys=("state", "q_lpm", "f_hz", "n_ft", "d1", "d2", "d3", "sov", "lsh"), every=1.0):
        end = time.time() + secs
        while time.time() < end:
            fr = self.frame()
            print(" ".join(f"{k}={fr.get(k)}" for k in keys), flush=True)
            time.sleep(every)
