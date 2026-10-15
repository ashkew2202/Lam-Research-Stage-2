# Data logger for the flow rig. Keeps the serial port open for the whole session.
#
#   python3 tools/rig_daq.py --session take_1 [--mwd 20] [--no-plot]
#
# Opening the port resets the ESP32, so only this script opens it. Everything else
# goes through files in the session folder (runs/<date>_<time>_<session>/):
#   raw.log        every line from the ESP32 and every command sent, with PC time
#   telemetry.csv  the 10 Hz data lines (D)
#   events.csv     state changes, alarms and so on (E)
#   cal.csv        calibration run results (C)
#   latest.json    the newest data line
#   cmd.txt        add a line here to send it to the ESP32, e.g. echo "P 2 40" >> cmd.txt
#
# --mwd turns on the manual-mode watchdog in the firmware and sends PING every 2 s,
# so if this logger dies the pumps switch off after mwd seconds.

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time

import serial

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mwd", type=float, default=0)
    ap.add_argument("--no-plot", action="store_true", help="do not open the live plot")
    args = ap.parse_args()

    out = os.path.join(ROOT, time.strftime("%Y%m%d_%H%M%S") + "_" + args.session)
    os.makedirs(out)
    raw = open(os.path.join(out, "raw.log"), "a", buffering=1)
    tele_f = open(os.path.join(out, "telemetry.csv"), "a", newline="")
    ev_f = open(os.path.join(out, "events.csv"), "a", newline="")
    cal_f = open(os.path.join(out, "cal.csv"), "a", newline="")
    tele, ev, cal = csv.writer(tele_f), csv.writer(ev_f), csv.writer(cal_f)
    ev.writerow(["host_time", "t_ms", "tag", "text"])
    cal.writerow(["host_time", "t_ms", "run", "pulses", "time_s", "f_hz", "duty"])
    cmd_path = os.path.join(out, "cmd.txt")
    open(cmd_path, "w").close()
    cmd_pos = 0
    fields = None

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = args.port, args.baud, 0.05
    ser.dtr = ser.rts = False
    ser.exclusive = True  # a second open would reset the ESP32 in the middle of a run
    ser.open()
    print(out, flush=True)
    if not args.no_plot:
        subprocess.Popen([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "live_plot.py"), out],
                         start_new_session=True)

    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def send(cmd):
        raw.write(f"{time.time():.3f} >> {cmd}\n")
        ser.write((cmd + "\n").encode())

    time.sleep(1.5)  # let the board boot
    send("HDR")
    send("GET")
    if args.mwd:
        send(f"SET mwd {args.mwd:g}")
    last_ping = time.time()
    buf = b""
    while running:
        buf += ser.read(4096)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.decode("utf-8", "replace").strip()
            if not line:
                continue
            now = time.time()
            raw.write(f"{now:.3f} {line}\n")
            p = line.split(",")
            if p[0] == "H":
                if p[1:] != fields:
                    fields = p[1:]
                    tele.writerow(["host_time"] + fields)
            elif p[0] == "D" and fields and len(p) - 1 == len(fields):
                tele.writerow([f"{now:.3f}"] + p[1:])
                with open(os.path.join(out, "latest.json.tmp"), "w") as f:
                    json.dump(dict(zip(["host_time"] + fields, [now] + p[1:])), f)
                os.replace(os.path.join(out, "latest.json.tmp"), os.path.join(out, "latest.json"))
            elif p[0] == "E" and len(p) >= 4:
                ev.writerow([f"{now:.3f}", p[1], p[2], ",".join(p[3:])])
                ev_f.flush()
            elif p[0] == "C" and len(p) >= 7:
                cal.writerow([f"{now:.3f}"] + p[1:7])
                cal_f.flush()
            elif p[0] == "B":
                with open(os.path.join(out, "burst.txt"), "a") as f:
                    f.write("\n".join(p[1:]) + "\n")
        # commands added to cmd.txt
        with open(cmd_path) as f:
            f.seek(cmd_pos)
            new = f.read()
            if new.endswith("\n"):
                cmd_pos += len(new)
                for c in new.splitlines():
                    if c.strip():
                        send(c.strip())
        if args.mwd and time.time() - last_ping > 2:
            send("PING")
            last_ping = time.time()
        tele_f.flush()

    # everything off before closing
    for c in ("SOV 0", "P 1 0", "P 2 0", "P 3 0", "STOP"):
        send(c)
    time.sleep(0.3)
    raw.write(f"{time.time():.3f} # logger closed\n")
    ser.close()


if __name__ == "__main__":
    main()
