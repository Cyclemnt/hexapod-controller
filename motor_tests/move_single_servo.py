"""
move_single_servo.py — Move one ST3215 servo to a target angle.

    python move_single_servo.py

Edit the constants below before running.
Use SPEED 200-300 on first runs until mechanical range is validated.

    pip install feetech-servo-sdk
"""

import time
from scservo_sdk import PortHandler, PacketHandler
import servo_config as cfg

# ── Edit these ──────────────────────────────────────────────────────────────
SERVO_ID  = 1
ANGLE_DEG = 45.0
SPEED     = 300     # 0 = hardware max — keep low on first runs
# ────────────────────────────────────────────────────────────────────────────

ticks = max(0, min(4095, round(cfg.ZERO_TICKS + cfg.DIRECTION * ANGLE_DEG * 4096 / 360)))

port   = PortHandler(cfg.PORT)
packet = PacketHandler(port)
port.openPort()
port.setBaudRate(cfg.BAUDRATE)

packet.write1ByteTxRx(SERVO_ID, cfg.TORQUE_EN,  1)
packet.write1ByteTxRx(SERVO_ID, cfg.ACCEL,      0)
packet.write2ByteTxRx(SERVO_ID, cfg.GOAL_SPEED, SPEED)
packet.write2ByteTxRx(SERVO_ID, cfg.GOAL_POS,   ticks)

print(f"Servo {SERVO_ID} → {ANGLE_DEG}° ({ticks} ticks) — waiting...", end="", flush=True)
while True:
    moving, _, _ = packet.read1ByteTxRx(SERVO_ID, cfg.MOVING)
    if not moving:
        break
    print(".", end="", flush=True)
    time.sleep(0.05)
print(" done.")

packet.write1ByteTxRx(SERVO_ID, cfg.TORQUE_EN, 0)
port.closePort()
