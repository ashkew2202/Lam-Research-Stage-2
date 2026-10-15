# Live plot while the rig is running. It only reads telemetry.csv from the logger's
# session folder, it does not open the serial port.
#
#   python3 tools/live_plot.py [session_dir]     (rig_daq.py starts it automatically)
#
# Shows FT-01 flow in L/min and mL/s, litres returned to TK-01, pump duties,
# the float switch and the solenoid valve against time from the switch press.
# Saves a PNG in the session folder at the end of Case 1 and when the run stops.

import os
import sys
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from daq_client import session

PUMPS = (("d1", "P-01"), ("d2", "P-02"), ("d3", "P-03"))
PHASES = {"C1_START": "Case 1", "C2_ISOLATE": "", "C2_OPEN_BV01": "open BV-01", "C2_FLUSH": "Flush",
          "COMPLETE": "OFF", "STOPPED": "STOPPED"}
ENDS = ("COMPLETE", "STOPPED")


class Tail:
    # reads the new lines added to the csv since the last call
    def __init__(self, path):
        self.path, self.pos, self.buf, self.cols = path, 0, "", None

    def rows(self):
        with open(self.path, newline="") as f:
            f.seek(self.pos)
            self.buf += f.read()
            self.pos = f.tell()
        *lines, self.buf = self.buf.split("\n")
        out = []
        for ln in lines:
            p = ln.rstrip("\r").split(",")
            if p[0] == "host_time":
                self.cols = p
            elif self.cols and len(p) == len(self.cols):
                out.append(dict(zip(self.cols, p)))
        return out


def main():
    sdir = sys.argv[1] if len(sys.argv) > 1 else session()
    path = os.path.join(sdir, "telemetry.csv")
    while not os.path.exists(path):
        time.sleep(0.5)
    tail = Tail(path)
    d = {k: [] for k in ("t", "q", "v", "d1", "d2", "d3", "lsh", "sov")}
    marks = []  # (time, state) at each phase change
    st = {"t0": None, "state": "", "run": False, "row": None}

    fig, (axq, axv, axd, axs) = plt.subplots(4, 1, figsize=(14, 8), sharex=True,
                                             gridspec_kw={"height_ratios": [3, 1.6, 1.6, 0.8]})
    fig.canvas.manager.set_window_title("Flow rig live plot")
    fig.subplots_adjust(left=0.09, right=0.93, top=0.88, bottom=0.07, hspace=0.2)
    title = fig.suptitle("", fontsize=13)

    qline, = axq.plot([], [])
    axq.set_ylabel("FT-01 flow (L/min)")
    axq.secondary_yaxis("right", functions=(lambda x: x * 1000 / 60, lambda x: x * 60 / 1000)).set_ylabel("mL/s")
    vline, = axv.plot([], [])
    axv.set_ylabel("Returned to\nTK-01 (L)")
    dlines = [axd.plot([], [], label=name, drawstyle="steps-post")[0] for _, name in PUMPS]
    axd.set_ylabel("Duty (%)")
    axd.set_ylim(-5, 110)
    axd.legend(loc="center right")
    axs.set_ylim(-0.2, 2.2)
    axs.set_yticks([0.5, 1.5], ["SOV-01 open", "LS-01 high"])
    axs.set_xlabel("Time from switch press (s)")
    for ax in (axq, axv, axd):
        ax.grid(True)
    dyn = []

    def save(tag):
        fig.savefig(os.path.join(sdir, f"live_{time.strftime('%H%M%S')}_{tag}.png"), dpi=120)

    def update(_):
        for r in tail.rows():
            t, s = float(r["host_time"]), r["state"]
            if s == "C1_START" and st["state"] != "C1_START":
                for k in d:
                    d[k].clear()
                marks.clear()
                st["t0"], st["run"] = t, True
            if st["t0"] is None:
                st["t0"] = t
            x = t - st["t0"]
            if st["run"] and s != st["state"] and s in PHASES:
                marks.append((x, s))
                if s == "C2_ISOLATE":
                    st["save"] = "end_case1"
                elif s in ENDS:
                    st["save"] = s.lower()
            st["state"], st["row"] = s, r
            d["t"].append(x)
            d["q"].append(float(r["q_lpm"]))
            d["v"].append(float(r["v_c1_l"]) + float(r["v_c2_l"]))
            for k in ("d1", "d2", "d3"):
                d[k].append(100 * float(r[k]))
            d["lsh"].append(int(r["lsh"]))
            d["sov"].append(int(r["sov"]))
        # before a run starts just show the last 60 s
        if not st["run"] and d["t"] and d["t"][-1] - d["t"][0] > 60:
            n = next(i for i, x in enumerate(d["t"]) if x >= d["t"][-1] - 60)
            for k in d:
                del d[k][:n]
        if not d["t"]:
            return
        r = st["row"]
        qline.set_data(d["t"], d["q"])
        vline.set_data(d["t"], d["v"])
        for ln, (k, _) in zip(dlines, PUMPS):
            ln.set_data(d["t"], d[k])
        for a in dyn:
            a.remove()
        dyn.clear()
        dyn.append(axs.fill_between(d["t"], 1.1, [1.1 + 0.8 * v for v in d["lsh"]], step="post", color="gray"))
        dyn.append(axs.fill_between(d["t"], 0.1, [0.1 + 0.8 * v for v in d["sov"]], step="post", color="gray"))
        x0 = d["t"][0] if not st["run"] else 0
        x1 = max(x0 + 60, d["t"][-1] * 1.03 + 2)
        axq.set_xlim(x0, x1)
        axq.set_ylim(0, max(3.0, max(d["q"]) * 1.15))
        axv.set_ylim(0, max(1.0, max(d["v"]) * 1.15))
        for x, s in marks:
            if not PHASES[s]:
                continue
            for ax in (axq, axv, axd, axs):
                dyn.append(ax.axvline(x, color="k", linewidth=1, linestyle=":"))
            dyn.append(axq.text(x, 1.01, " " + PHASES[s], transform=axq.get_xaxis_transform(), va="bottom", ha="left"))
        q = float(r["q_lpm"])
        tc = d["t"][-1]
        c2 = next((x for x, s in marks if s == "C2_ISOLATE"), None)
        if st["state"] in ENDS or not st["run"]:
            clock = ""
        elif c2 is not None:
            clock = f"Case 2 +{tc - c2:.0f} s"
        else:
            clock = f"Case 1 +{tc:.0f} s"
        title.set_text(f"{st['state']}   {clock}\n"
                       f"FT-01 {q:.2f} L/min = {q * 1000 / 60:.1f} mL/s     Returned to TK-01: "
                       f"Case 1 {float(r['v_c1_l']):.2f} L, Case 2 {float(r['v_c2_l']):.2f} L     "
                       f"P-01 {d['d1'][-1]:.0f}%  P-02 {d['d2'][-1]:.0f}%  P-03 {d['d3'][-1]:.0f}%")
        if st.get("save"):
            fig.canvas.draw()
            save(st.pop("save"))

    anim = FuncAnimation(fig, update, interval=500, cache_frame_data=False)  # keep a reference or the timer stops
    plt.show()


if __name__ == "__main__":
    main()
