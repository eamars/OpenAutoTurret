# The one-shot actuator: fire it, calibrate it, recover it

## What this is for

Operating the GPIO15 hobby servo (ADR-007): ACTUATE moves it from rest (A) to active (B), waits the
dwell, and brings it back to A. This card also covers finding A, B and the dwell, and recovering the
actuator when it reports itself unavailable. It is not for wiring or choosing a pin: why GPIO15 and
not GPIO18 is in [ADR-007](../ADR-007/ADR-007.md) section 1.

## Where the work happens

- **On the page:** firing it, and calibrating it with the stack up.
- **On the station, over ssh:** the bench tool, for a stopped stack only.
- **The owner, with sudo:** the PWM overlay.
- **controld:** owns the pin while it runs. The launcher sets the pin's function at every start.

Nothing here is compiled. The overlay is the one root step.

## The command

**Fire it.** Press the red **ACTUATE** button under the D-pad. It stays in that place whether or not
the D-pad shows. One press fires one shot; a second press during the shot is refused. Over the JSON
API (an agent's route; the page is the operator's):

```bash
curl -s -X POST http://<station>:8080/api/command -H 'content-type: application/json' -d '{"command":"actuate","arg":""}'
```

**Calibrate it, stack up.**
1. Open MENU › CALIBRATION and scroll to ACTUATOR, then press ADJUST A / B…. The servo goes to A
   (1500 µs if it was never calibrated) and stays powered.
2. Move it with PULSE − / + at a STEP of 50, 10 or 2 µs. Near an end stop, use small steps and back
   off at the first buzz: a stalled servo strips its gears.
3. Press MARK REST (A) and MARK ACTIVE (B) at the two positions, and set the DWELL.
4. TEST runs a shot with the marks. SAVE puts them in force and keeps them across restarts.
5. DONE, or leaving calibration any other way, closes the session: the servo returns to A and goes
   limp.

**Calibrate it, stack stopped** (bench tool; it refuses to start while controld runs):

```bash
ssh -t eamars@rpi-turret python3 ~/workspace/OpenAutoTurret/run/current/Firmware/tools/actuator_calibrate.py
```

**Load the overlay** (owner, sudo). It is lost at every reboot until this line is in
`/boot/firmware/config.txt`:

```bash
sudo dtoverlay pwm pin=15 func=4
```

The persistent form, in `config.txt`, is `dtoverlay=pwm,pin=15,func=4`. After loading it at runtime,
restart the stack so that controld opens the channel.

**Check it.** Telemetry carries `actuator` (`/api/state`): `state` (`off`, `rest`, `active`,
`returning`, `calibrating`, `unconfigured`, `unavailable`), `reason`, `pulse_us` (what is on the wire,
null when off), the saved values and `shots`.

## What it proves

- An accepted ACTUATE wrote B to the PWM, held it for the dwell (at least one 20 ms period), wrote A,
  and switched the output off 0.5 s later. That is what `shots` and the state sequence report.
- A saved calibration is on disk (`run/state/actuator.json`, fsynced) when `saved` reads 1.

## What it does not prove

- **That the servo moved.** The PWM has no feedback: a servo without power, unplugged, or stalled on
  an end stop reports exactly the same telemetry. Look at it, or at a camera when that is allowed.
- **That the station's supply is fine.** Until the servo has a supply of its own, its moves pull the
  Pi's 5 V down (ADR-007 section 2, "Finding"). After any actuator work, run
  `journalctl -k | grep -iE "voltage|SPI transfer"` and check both CAN buses before calling the
  station healthy.

## When it fails

| It says | Why | What to do |
|---|---|---|
| `UNAVAILABLE`: `no PWM chip at 1f00098000: the pwm overlay is not loaded` | a reboot without the `config.txt` line | the owner loads the overlay; restart the stack |
| `UNAVAILABLE`: `GPIO15 is not on PWM0` | the pin's function did not take | restart the stack, which sets it again; `pinctrl get 15` should read `a0 … PWM0_CHAN3` |
| `UNAVAILABLE`: `the bench tool actuator_calibrate.py is running` | two writers would fight over the channel | stop the tool, then restart the stack |
| `UNAVAILABLE`: `PWM write failed: … the output is off` | sysfs refused a write; the output was switched off | nothing: controld tries again every 5 s and returns to A when the write works |
| `NOT CALIBRATED`, or `actuator.json refused: …` | no file, or a value outside 500..2500 µs or 0..5000 ms | calibrate it (above); the reason names the bad value |
| `FAULT` | the station is faulted; nothing actuates beside a fault | HOME recovers first |
| `IN CALIBRATION` | a session is open, possibly on another page | DONE there, or leave calibration |
| the launcher logs `controld left the PWM on; the launcher returned it to rest …` | controld ended without switching off (a crash) | the launcher already drove A and switched it off; read controld's log for the crash |
