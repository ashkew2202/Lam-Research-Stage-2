# Flow rig code - Team Catalysts (LRC-26-0146)

Lam Research Challenge 3.0, Stage 2.

This is the ESP32 firmware for our three pump flow rig, the laptop scripts we used for logging and plotting, and the data we measured on the rig.

## Files

| Path | What it is |
|---|---|
| firmware/lam_flow_rig/src/main.cpp | The controller (firmware 1.2, Arduino-ESP32 core 2.0.17) |
| firmware/lam_flow_rig/platformio.ini | PlatformIO build file. Upload with `pio run -e upesy_wroom -t upload` |
| firmware/lam_flow_rig/lam_flow_rig.ino | Empty sketch file so the folder also opens in the Arduino IDE |
| tools/rig_daq.py | Logger. Holds the serial port and writes telemetry.csv, events.csv and cal.csv |
| tools/live_plot.py | Live plot of FT-01 flow (L/min and mL/s), litres returned, pump duties, float switch and solenoid valve |
| tools/daq_client.py | Helper the other scripts use to send commands through the logger |
| tools/run_capture.py | Copies one run out of a logging session into its own folder |
| tools/cal_session.py | Walks through the weighed calibration runs |
| tools/cal_fit.py | Fits Q = a*f + b, prints R^2 and the errors, plots the fit |
| tools/plot_run.py | Flow vs time plot of a recorded run |
| tools/plot_valve.py | Valve position vs flow plot (Q2.1) |
| tools/theory.py | Velocity, Reynolds number and residence time numbers for the hand calculations |
| runs/ | Measured data: calibration runs, valve sweep, pump tests and the recorded Case 1 + Case 2 run |

## How the controller works

Case 1. One press of the on/off switch (HS-01) starts P-02, opens SOV-01 and starts P-01. When the sump float switch LS-01 goes high, P-01 stops and SOV-01 closes while P-02 keeps running. When the level drops and the float clears, P-01 starts again.

Case 2. When the Case 1 time is over, SOV-01 closes to isolate the 10 L tank. The operator gets a fixed time window to open BV-01. Then P-01, P-02 and P-03 run together. There is no level sensor on the flush tank, so the flush stops on a P-03 run timer. We set the timer from the measured flush rate and the volume we load into the flush tank. After that every output is switched off and stays off.

Flow. FT-01 pulses are counted with an interrupt. Flow is Q = a*f + b, taken as zero below a cutoff frequency, and it is totalised over the whole run.

Protection. Sump high stops P-01 and P-03 and closes SOV-01 in every mode. P-02 trips after 5 s without flow.

## Results from our rig

- Calibration from five weighed runs between 1.2 and 2.25 L/min (runs/cal_runs_20261011.csv): a = 0.009093 L/min per Hz, b = 0.0602 L/min, R^2 = 0.9998, maximum error 0.46%, average error 0.27%.
- Maximum flow through the loop with P-02 at 100% duty is 2.3 L/min (2.74 L/min with a free outlet). The organisers accepted this limit in place of the 3 L/min target.
- Recorded run (runs/20261011_203927_paper_duty_take_2) with P-01 at 50% and P-02 at 100%: Case 1 for 75 s at 2.30 L/min, 2.85 L returned. P-01 stopped on the float at +30.3 s and restarted at +48.8 s. Flush with all three pumps for 65.6 s. Automatic stop at +157 s with the flush tank empty. 5.83 L returned in total.

## Running the scripts

```
pip install pyserial matplotlib numpy

python3 tools/cal_fit.py runs/cal_runs_20261011.csv cal.png
python3 tools/plot_run.py runs/20261011_203927_paper_duty_take_2 case_trend.png
python3 tools/plot_valve.py q21_valve.png
python3 tools/theory.py
```

Logging a run on the rig:

```
python3 tools/rig_daq.py --session take_1
python3 tools/run_capture.py take_1
```

The first command starts the logger and the live plot. The second one is run in another terminal before pressing the switch.
