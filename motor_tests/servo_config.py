"""
servo_config.py — Shared constants for all test scripts.
Edit here, changes apply everywhere.
"""

# Serial
PORT     = "/dev/ttyUSB0"
BAUDRATE = 1_000_000

# Encoder (12-bit absolute, 4096 steps/rev)
ZERO_TICKS = 2048          # tick that equals 0° — calibrate per joint
DIRECTION  = 1             # +1 standard, -1 if servo mounted inverted

# RAM register addresses
TORQUE_EN   = 40
ACCEL       = 41
GOAL_POS    = 42
GOAL_SPEED  = 46
PRES_POS    = 56           # 2B — present position
PRES_SPEED  = 58           # 2B — present speed  (signed)
PRES_LOAD   = 60           # 2B — load magnitude + direction
PRES_VOLT   = 62           # 1B — voltage ×0.1 V
PRES_TEMP   = 63           # 1B — temperature °C
STATUS      = 65           # 1B — error bitmask
MOVING      = 66           # 1B — 1 if moving
PRES_CURR   = 69           # 2B — current ×6.5 mA

# Safety thresholds (used by read_feedback.py)
TEMP_WARN_C  = 65          # hardware cutoff ~72 °C
VOLT_MIN_V   = 6.5
VOLT_MAX_V   = 8.4
