"""
read_feedback.py — Poll telemetry from one or more ST3215 servos.

    python read_feedback.py

Edit IDS and INTERVAL below. Ctrl+C to stop.
Use --latency flag: python read_feedback.py --latency

    pip install feetech-servo-sdk
"""

import struct
import sys
import time
from scservo_sdk import PortHandler, PacketHandler
import servo_config as cfg

# ── Edit these ───────────────────────────────────────────────────────────────
IDS      = [1, 2, 3]
INTERVAL = 0.2          # seconds between iterations
# ─────────────────────────────────────────────────────────────────────────────

latency_mode = "--latency" in sys.argv

port   = PortHandler(cfg.PORT)
packet = PacketHandler(port)
port.openPort()
port.setBaudRate(cfg.BAUDRATE)

latency_samples = {sid: [] for sid in IDS}

try:
    i = 0
    while True:
        i += 1
        print(f"--- {i} ---")
        for sid in IDS:
            t0 = time.perf_counter()
            data, res, err = packet.readTxRx(sid, cfg.PRES_POS, 15)
            rtt = (time.perf_counter() - t0) * 1000

            if res != 0 or err != 0:
                print(f"  ID {sid}: comm error")
                continue

            pos   = struct.unpack_from("<H", bytes(data), 0)[0]
            spd   = struct.unpack_from("<h", bytes(data), 2)[0]
            load  = struct.unpack_from("<H", bytes(data), 4)[0]
            volt  = data[6] * 0.1
            temp  = data[7]
            stat  = data[9]
            mov   = data[10]
            curr  = struct.unpack_from("<H", bytes(data), 13)[0] * 6.5

            deg   = (pos - cfg.ZERO_TICKS) * 360 / 4096
            warns = []
            if temp  >= cfg.TEMP_WARN_C:  warns.append(f"TEMP {temp}°C")
            if volt  <  cfg.VOLT_MIN_V:   warns.append(f"VOLT LOW {volt:.1f}V")
            if volt  >  cfg.VOLT_MAX_V:   warns.append(f"VOLT HIGH {volt:.1f}V")
            if stat  != 0:                warns.append(f"STATUS 0x{stat:02X}")

            rtt_str  = f" rtt={rtt:.1f}ms" if latency_mode else ""
            warn_str = "  ⚠ " + " | ".join(warns) if warns else ""
            print(
                f"  ID {sid}: {pos:4}t {deg:+7.2f}° | spd {spd:5} | "
                f"load {load & 0x3FF:4} | {volt:.1f}V | {temp}°C | "
                f"{curr:.0f}mA | mov={mov}{rtt_str}{warn_str}"
            )
            if latency_mode:
                latency_samples[sid].append(rtt)

        print()
        time.sleep(INTERVAL)

except KeyboardInterrupt:
    pass

port.closePort()

if latency_mode and any(latency_samples.values()):
    print("\n── Latency ──")
    for sid, s in latency_samples.items():
        if s:
            avg = sum(s) / len(s)
            print(f"  ID {sid}: min={min(s):.1f} avg={avg:.1f} max={max(s):.1f} ms")
    total_avg = sum(sum(s) for s in latency_samples.values()) / max(sum(len(s) for s in latency_samples.values()), 1)
    print(f"  Sequential budget for {len(IDS)} servo(s): ~{total_avg * len(IDS):.0f} ms → ~{1000 / (total_avg * len(IDS)):.0f} Hz max")

