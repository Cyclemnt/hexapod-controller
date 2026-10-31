# ST3215 servo motor test scripts

Standalone Python scripts, **independent of the ROS 2 workspace**, for
hands-on hardware validation of the Feetech ST3215 servos via the
*Bus Servo Driver ST & SC for XIAO* (105990190).

## Prerequisites

```bash
pip install feetech-servo-sdk
```

Identify the serial port exposed by the XIAO driver:

```bash
ls /dev/tty{USB,ACM}*
# Update SERIAL_PORT in servo_config.py
```

## Files

| File | Purpose |
|---|---|
| `servo_config.py` | Shared constants (port, baudrate, register addresses, thresholds) |
| `minimal_move.py` | Absolute minimum — one servo, one angle, no imports except SDK |
| `move_single_servo.py` | One servo to a target angle, waits for completion |
| `move_leg.py` | 3 servos simultaneously via REG_WRITE + ACTION broadcast |
| `read_feedback.py` | Live telemetry loop with safety warnings; `--latency` for RTT stats |

Edit the constants at the top of each script, then run.

## Commissioning procedure

### 0 — Assign unique IDs (one servo at a time)
Factory default ID = **1** for every servo. Connect them **one at a time** and
assign IDs using Feetech Debugger (FD) or SCServo_Tool.

Suggested mapping:

| Leg | Coxa | Femur | Tibia |
|-----|------|-------|-------|
| 0   | 1    | 2     | 3     |
| 1   | 4    | 5     | 6     |
| 2   | 7    | 8     | 9     |
| 3   | 10   | 11    | 12    |
| 4   | 13   | 14    | 15    |
| 5   | 16   | 17    | 18    |

### 1 — Identify the serial port
```bash
ls /dev/tty{USB,ACM}*   # plug/unplug to spot the new entry
# Update PORT in servo_config.py
```

### 2 — Check supply voltage
```bash
python read_feedback.py   # IDS = [1]
```
`voltage_v` must be **7.4–8.4 V**. Below 6.5 V the servo protection triggers
silently. A 3S LiPo (12.6 V) is too high — use a DC-DC step-down.

### 3 — Calibrate ZERO_TICKS per joint
1. Disable torque (FD software), place joint at your mechanical 0°.
2. `python read_feedback.py` → note the `ticks` value.
3. Set `ZERO_TICKS` in `servo_config.py`. Maps to `JointCalibration::zeroOffsetTicks`.

### 4 — Validate DIRECTION per joint
With torque disabled, rotate the joint by hand in the positive direction of
your kinematic model. If ticks **decrease**, set `DIRECTION = -1`.
Maps to `JointCalibration::direction`.

### 5 — First move (speed-limited)
```bash
# Edit SERVO_ID, ANGLE_DEG, SPEED=200 in move_single_servo.py
python move_single_servo.py
```
> **Use SPEED 200–300** until the mechanical range inside the 3D-printed leg
> is validated. Full speed on an unloaded horn can hit end-stops hard.

### 6 — Measure bus latency (before sizing the ROS 2 control loop)
```bash
python read_feedback.py --latency   # IDS = [1, 2, 3], INTERVAL = 0
```
Expect 1–3 ms/servo at 1 Mbit/s. 18 servos sequential → 18–54 ms → 20–50 Hz max.

### 7 — First leg move
Validate all three joints individually (steps 3–5) before this step.
```bash
# Edit IDS, ANGLES, SPEED in move_leg.py
python move_leg.py
```

### 8 — Monitor temperature under sustained load
```bash
python read_feedback.py   # keep open in a second terminal
```
Script prints `⚠ TEMP` when approaching the hardware cutoff (~72 °C).
Stop and let the servo cool if temperature exceeds **65 °C**.
