# Lam-Research-Stage-2

Three-pump flow rig on an ESP32 and the Stage 2 PCB. This file covers installing the USB driver, building and flashing the test sketches, wiring notes for this specific rig, and the test order from a dry bring-up to the full single take.

## 1. What you need

- Windows laptop, a USB cable that carries data (not a charge-only cable), VS Code
- ESP32 board (uPesy WROOM) on the Stage 2 PCB, 12 V SMPS, three IBT-2 drivers, SOV-01, FT-01, LS-01 float
- Scale (1 g), a 0.8 L jug, a drip tray, a multimeter

## 2. USB driver (do this first)

The board talks to the laptop through a USB-to-serial chip. Without its driver the board does not appear as a COM port and uploads fail with "port is busy or doesn't exist".

1. Plug the board in with USB only. In Device Manager, open Ports (COM & LPT) and note what is there. Unplug the board and see which entry disappears. Ignore ports that stay (usually Bluetooth).
2. If no new COM port appears, install the driver for your board's chip. The uPesy WROOM normally uses a Silicon Labs CP210x chip:
   - Download the Windows driver from https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers
   - Unzip it, right-click the `.inf` file (or run the installer), choose Install
3. If the chip is a CH340 instead, use the WCH driver from https://www.wch-ic.com/downloads/CH341SER_EXE.html
4. Unplug and replug the board. Device Manager should now show a new entry such as "Silicon Labs CP210x USB to UART Bridge (COM7)". Use that COM number below.
5. Still nothing: try another cable (use one that moves files from a phone), another USB port, and plug the board in alone with nothing on the PCB socket.

To tell which chip you have: Device Manager, right-click the port, Properties, Details, Hardware Ids. `10C4:EA60` is CP210x, `1A86:7523` is CH340.

## 3. Project setup (PlatformIO)

Install VS Code and the PlatformIO IDE extension. Open the project folder (the one that holds `platformio.ini`).

Layout:

```
platformio.ini
src/
  main.cpp              rig firmware (leave as it is)
  bringup/main.cpp      component test (valve, button, float, flow sensor, pumps)
  motordebug/main.cpp   single motor debug tool
  watertest/main.cpp    staged water test and full single take
```

`platformio.ini`:

```ini
[env:upesy_wroom]
platform = espressif32
board = upesy_wroom
framework = arduino
monitor_speed = 115200
monitor_filters = time
build_flags = -Wall -Wextra -Wno-unused-parameter
build_src_filter = +<*> -<bringup/> -<motordebug/> -<watertest/>

[env:bringup]
platform = espressif32
board = upesy_wroom
framework = arduino
monitor_speed = 115200
build_flags = -Wall -Wextra -Wno-unused-parameter
build_src_filter = -<*> +<bringup/>

[env:motordebug]
platform = espressif32
board = upesy_wroom
framework = arduino
monitor_speed = 115200
build_src_filter = -<*> +<motordebug/>

[env:watertest]
platform = espressif32
board = upesy_wroom
framework = arduino
monitor_speed = 115200
build_flags = -Wall -Wextra -Wno-unused-parameter
build_src_filter = -<*> +<watertest/>
```

If auto-detect picks the wrong port, add `upload_port = COM7` (your number) to the env.

### Flash and monitor

Close any other program using the port first (serial monitor, Python console). Switch the 12 V supply off, keep USB connected, then:

```
pio run -e bringup -t upload
pio device monitor -e bringup
```

Replace `bringup` with `motordebug` or `watertest`. In VS Code you can use the PlatformIO sidebar: Project Tasks, choose the env, Upload, then Monitor. Send each command with Enter.

If upload hangs on "Connecting...", hold the BOOT button until it starts writing.

To go back to the rig firmware: `pio run -e upesy_wroom -t upload`.

## 4. Wiring notes for this rig

| Item | State |
| --- | --- |
| LSL-03 probe (G4), ZSO-01 (G5), IBT-2 current sense R_IS / L_IS (G15, G36, G39) | Not connected. The sketches never read them. |
| Pump 2 drive pins | Swapped in code: RPWM = G18, LPWM = G19 (G19 stays low). The signal wire from G19 does not reach the module. Wires stay in their original positions and the pump's motor leads are swapped so it turns the right way. |
| Float (LS-01, G35) | Reads LOW when the water is high. The sketches flip it (`FLOAT_INVERTED = true`). A broken float wire reads as "level OK", so overfill protection is lost if the wire breaks. Turning the float over and setting `FLOAT_INVERTED = false` restores that. |
| Flush end | No empty-tank sensor and no pump current. Case 2 ends on a timer of P-03 running time (`tfl`), measured in stage 4. |

12 V supply: switch it on first, then connect USB (or press `g` / start the test after it is on). Never plug or unplug motor leads with the supply on. Mains side wiring needs a qualified person's check: close the SMPS terminal cover and bond its earth terminal.

## 5. Test order

### 5.1 Bring-up (`bringup`)

No water needed. `a` runs everything; a number runs one test.

1. SOV-01 valve: it clicks open and shut three times (listen)
2. HS-01 button: reads 1 at rest, 0 when pressed
3. LS-01 float: moves between both levels
4. FT-01 signal: blow through the meter, pulses appear
5. P-01, P-02, P-03 and all three pumps: dry run mode spins each pump 5 s. Confirm by sound or vibration.
6. Status LED: blinks five times

### 5.2 If a motor does not turn (`motordebug`)

Select one pump (`sel 2`) and run `d 60`. Measure at the IBT-2 with a multimeter: B+ to B− about 12 V, VCC 3.3 V, R_EN and L_EN about 3.3 V, RPWM averaging about 2 V. `r 60` tests the other half of the driver, `pin rp 1` holds the forward pin high, `sweep` finds the starting duty. Outputs stop on their own after 15 s. `stop` shuts everything off.

### 5.3 Water test (`watertest`)

Fill TK-01 about 3.4 L, TK-03 4.2 L, TK-02 empty. Valves: BV-01 shut, BV-02 fully open, BV-03 open. Float jumper removed.

| Stage | Command | What it does | Pass |
| --- | --- | --- | --- |
| 1 | `1` | P-01 moves water TK-01 to the sump (20 s) | Water arrives in TK-02 |
| 2 | `2` | P-02 loop through FT-01 (30 s). TK-02 needs at least 1.9 L. | Pulses and a mean near 3 L/min |
| 4 | `4` | P-01 and P-03 flush-rate run. Open BV-01, shut it afterwards, weigh TK-03, then `lit <litres>` and `save`. | Flush timer `tfl` is set |
| 3 | `3` | Case 1 for 180 s. Pour 0.8 L at the POUR NOW cue. | P-01 stops, P-02 keeps running, P-01 restarts, mean flow about 3.0 L/min |
| 5 | `5` | Case 2. Open BV-01 when the LED blinks fast. | All pumps run, then stop on the timer |
| 6 | `6` or the button | Full take: Case 1 then Case 2 | Whole run with only BV-01 opened by hand |

`a` runs stages 1 to 5 with a pause for refilling. `x` stops everything. `st` shows status. `get`, `set <key> <value>` and `save` manage parameters. `cal <seconds> <duty%>` makes a timed P-02 run for calibration.

Full take timeline: P-02 starts, then SOV-01 opens and P-01 starts. The pour cue appears at 60 s, the float trips, P-01 stops, P-02 carries on, then P-01 restarts. At 180 s Case 2 starts: SOV-01 closes, the LED blinks fast, BV-01 is opened, then P-01 and P-03 flush with P-02 running. When the flush timer ends, everything stops. About six minutes in total.

## 6. Known risks

- The sump can drain at the start of Case 1: P-02 at 100% can draw more than P-01 at 50% refills through SOV-01. Measure both flows with stages 1 and 2. If needed, throttle BV-02 to about 3 L/min or raise P-01's duty (`set d1 0.8`) and justify the change with your data. Watch the sump and type `x` if it runs low.
- There is no dry-run protection (no current sensing). Pump runs are short and stop only on timers, the float, or `x`.
- A flush timer set too long runs P-03 dry. Set it from stage 4.

## 7. Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| "Could not open COM, semaphore timeout" | No driver or a charge-only cable. See section 2. |
| Linker error "undefined reference to setup() / loop()" | The sketch is outside the filter. Put it in `src/<name>/main.cpp` and check `build_src_filter`. |
| `ledcAttach was not declared` | An older core with a sketch written for core 3. Use the supplied sketches, which pick the right call. |
| `REFUSED: LS-01 ... level high` | The float reports high. Lower it, or check the wiring and `FLOAT_INVERTED`. |
| Pump silent in the pump tests | Check B+ and B− at the module, VCC 3.3 V, enables, then use the motor debug tool. |
| No pulses from FT-01 | Air in the meter, missing 5 V on its red wire, or the G14 divider. |