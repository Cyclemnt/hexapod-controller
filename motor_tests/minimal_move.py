"""
minimal_move.py — Bare-minimum script to move one ST3215 servo to a target angle.

No argument parsing, no helpers, no config file.
Edit the constants below and run.

    pip install feetech-servo-sdk
    python minimal_move.py
"""

from scservo_sdk import PortHandler, PacketHandler

# ── Edit these ────────────────────────────────────────────────────────────────
PORT      = "/dev/ttyUSB0"
BAUDRATE  = 1_000_000
SERVO_ID  = 1
ANGLE_DEG = 45.0        # target angle in degrees  (0° = tick 2048)
SPEED     = 300         # 0 = hardware maximum — keep low on first runs
# ──────────────────────────────────────────────────────────────────────────────

TARGET_TICKS = max(0, min(4095, round(2048 + ANGLE_DEG * 4096 / 360)))

port   = PortHandler(PORT)
packet = PacketHandler(port)

port.openPort()
port.setBaudRate(BAUDRATE)

packet.write1ByteTxRx(SERVO_ID, 40, 1)                    # torque enable
packet.write2ByteTxRx(SERVO_ID, 46, SPEED)                # goal speed
packet.write2ByteTxRx(SERVO_ID, 42, TARGET_TICKS)         # goal position

print(f"Servo {SERVO_ID} → {ANGLE_DEG}° ({TARGET_TICKS} ticks)")

port.closePort()

