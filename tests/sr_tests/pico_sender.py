"""
Pico -> Pi telemetry sender, synthetic data version.

Emits fixed-rate packets over USB serial so the transport and parsing can be
built and trusted before any real sensor exists. Every value here is fake but
shaped like the real thing: same ranges, same update rate, same field order.

When the LSM6DSOX and GPS arrive, only build_packet() changes. The framing,
timing, and sequence logic stay exactly as they are.

PACKET FORMAT
-------------
    $seq,t_ms,ax,ay,az,rpm,afr,map*CS\r\n

  seq    packet counter, wraps at 65536 -- lets the Pi detect dropped packets
  t_ms   Pico's monotonic clock in ms -- THE timebase for every channel
  ax/y/z acceleration in g
  rpm    engine speed
  afr    air/fuel ratio
  map    manifold absolute pressure in kPa
  CS     XOR of every byte between $ and *, as two hex digits (NMEA style)

Text rather than binary on purpose. You can read it with your eyes, and at
these rates the bandwidth cost is irrelevant. Switch to binary only when
something actually demands it.

RUNNING IT
----------
Disconnect Thonny first. The REPL and this share one USB serial channel, and
they will fight over it.

Save to the Pico as main.py to have it start on power-up.
"""

import math
import sys
import time

# 50 Hz is a deliberate compromise: fast enough to be interesting, slow enough
# that a text format fits comfortably in the link.
SAMPLE_HZ = 50

# Field order must match the receiver. Change one, change both.
FIELDS = ("seq", "t_ms", "ax", "ay", "az", "rpm", "afr", "map")


def checksum(payload):
    """XOR every byte, NMEA style. Cheap, and catches most line corruption."""
    cs = 0
    for ch in payload:
        cs ^= ord(ch)
    return cs


def build_packet(seq, t_ms):
    """Fake a plausible-looking sample set.

    Replace this with real sensor reads later. Nothing else needs to change.
    """
    t = t_ms / 1000.0

    # Gentle oscillation, as if the car were weaving through corners.
    ax = 0.35 * math.sin(t * 0.7)
    ay = 0.60 * math.sin(t * 0.4)
    az = 1.0 + 0.08 * math.sin(t * 3.1)  # 1g down, plus road texture

    # A slow rev sweep between idle and redline.
    rpm = 3500 + 2800 * math.sin(t * 0.25)

    # Mixture wanders around stoich, richer as revs climb.
    afr = 14.4 - 1.6 * (rpm - 700) / 6300.0

    # Manifold pressure roughly inverse to load.
    map_kpa = 45 + 45 * (0.5 + 0.5 * math.sin(t * 0.25))

    body = "%d,%d,%.3f,%.3f,%.3f,%.0f,%.2f,%.1f" % (
        seq, t_ms, ax, ay, az, rpm, afr, map_kpa
    )
    return "$%s*%02X\r\n" % (body, checksum(body))


def main():
    period_ms = 1000 // SAMPLE_HZ
    next_due = time.ticks_ms()
    seq = 0

    while True:
        t_ms = time.ticks_ms()
        sys.stdout.write(build_packet(seq, t_ms))

        seq = (seq + 1) & 0xFFFF

        # Absolute scheduling, not sleep(period). Sleeping a fixed amount lets
        # the time spent building each packet accumulate as drift; anchoring to
        # next_due keeps the average rate exact.
        next_due = time.ticks_add(next_due, period_ms)
        delay = time.ticks_diff(next_due, time.ticks_ms())
        if delay > 0:
            time.sleep_ms(delay)
        else:
            # Fell behind. Resync rather than sprinting to catch up -- the Pi
            # will see the gap in the sequence numbers, which is what you want.
            next_due = time.ticks_ms()


if __name__ == "__main__":
    main()
