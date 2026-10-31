"""
move_leg.py — Move 3 servos simultaneously (coxa, femur, tibia).

Uses REG_WRITE per servo then ACTION broadcast so all joints start at once.

    python move_leg.py

Edit the constants below before running.
Validate each joint individually with move_single_servo.py first.

    pip install feetech-servo-sdk
"""

import time
from scservo_sdk import PortHandler, PacketHandler, BROADCAST_ID
import servo_config as cfg

# ── Edit these ──────────────────────────────────────────────────────────────
IDS    = [1,    2,     3   ]   # coxa, femur, tibia
ANGLES = [0.0,  30.0, -60.0]  # degrees
SPEED  = 300
# ────────────────────────────────────────────────────────────────────────────

def to_ticks(angle):
    return max(0, min(4095, round(cfg.ZERO_TICKS + cfg.DIRECTION * angle * 4096 / 360)))

port   = PortHandler(cfg.PORT)
packet = PacketHandler(port)
port.openPort()
port.setBaudRate(cfg.BAUDRATE)

for sid, ang in zip(IDS, ANGLES):
    ticks = to_ticks(ang)
    print(f"  ID {sid}: {ang:+.1f}° → {ticks} ticks")
    packet.write1ByteTxRx(sid, cfg.TORQUE_EN,  1)
    packet.write1ByteTxRx(sid, cfg.ACCEL,      0)
    packet.write2ByteTxRx(sid, cfg.GOAL_SPEED, SPEED)
    packet.regWrite2ByteTxRx(sid, cfg.GOAL_POS, ticks)   # buffered

packet.action(BROADCAST_ID)   # fire all at once
print("ACTION sent — moving...", end="", flush=True)

while True:
    if all(packet.read1ByteTxRx(sid, cfg.MOVING)[0] == 0 for sid in IDS):
        break
    print(".", end="", flush=True)
    time.sleep(0.05)
print(" done.")

for sid in IDS:
    packet.write1ByteTxRx(sid, cfg.TORQUE_EN, 0)
port.closePort()
