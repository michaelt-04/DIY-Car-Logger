#!/usr/bin/env python3
"""
Pi-side receiver for Pico telemetry packets.

Reads framed packets over serial, validates them, logs to CSV, and reports link
health. Survives the Pico disconnecting and reconnecting, because in a car that
is a normal event rather than a fatal one.

Needs pyserial:
    sudo apt install python3-serial      # on the Pi
    pip install pyserial                 # on macOS

Usage:
    python3 pi_receiver.py                  # auto-detect port
    python3 pi_receiver.py --port /dev/ttyACM0
    python3 pi_receiver.py --quiet
"""

import argparse
import csv
import glob
import os
import sys
import time
from collections import deque
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial missing. Try: sudo apt install python3-serial")


# Must match the Pico's build_packet() exactly. Change one, change both.
FIELDS = ("seq", "t_ms", "ax", "ay", "az", "rpm", "afr", "map",
          "fix", "lat", "lon", "mph", "hdg", "sats", "hdop")
FIELD_TYPES = (int, int, float, float, float, float, float, float,
               int, float, float, float, float, int, float)

# fix: 0 = no fix, 1 = repeated last solution, 2 = fresh solution this packet
FIX_NONE, FIX_STALE, FIX_FRESH = 0, 1, 2

SEQ_MODULO = 65536
RECONNECT_DELAY_S = 1.0


def find_port():
    """Pico appears as /dev/ttyACM* on Linux, /dev/tty.usbmodem* on macOS.

    Never hardcode the number -- it changes between reboots and USB ports.
    Returns None rather than exiting, so the reconnect loop can keep waiting
    for hardware that hasn't been plugged back in yet.
    """
    ports = sorted(
        glob.glob("/dev/ttyACM*") + glob.glob("/dev/tty.usbmodem*")
    )
    return ports[0] if ports else None


def checksum(payload):
    cs = 0
    for b in payload.encode():
        cs ^= b
    return cs


def parse(line):
    """Return a dict, or None if the line is malformed.

    Returning None rather than raising is deliberate: corrupt lines are normal
    on a serial link, especially the partial one you catch when connecting
    mid-stream. They should be counted, not treated as errors.
    """
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None

    body, _, cs_text = line[1:].partition("*")

    try:
        if int(cs_text, 16) != checksum(body):
            return None
    except ValueError:
        return None

    parts = body.split(",")
    if len(parts) != len(FIELDS):
        return None

    try:
        return {k: t(v) for k, t, v in zip(FIELDS, FIELD_TYPES, parts)}
    except ValueError:
        return None


class LinkStats:
    """Tracks whether the link is actually healthy.

    Dropped packets are invisible unless you look for them, which is the whole
    reason the sequence number exists in the packet.
    """

    def __init__(self):
        self.received = 0
        self.bad = 0
        self.dropped = 0
        self.reconnects = 0
        self.last_seq = None
        self.gps_fresh = 0
        self.started = time.monotonic()
        # Rolling window. A lifetime average lets early dead time depress the
        # number for the rest of the session, masking real degradation later.
        self._recent = deque(maxlen=200)

    def note(self, pkt):
        self._recent.append(time.monotonic())
        self.received += 1
        # Count genuinely new solutions, not repeats. Should land near 10 Hz.
        if pkt.get("fix") == FIX_FRESH:
            self.gps_fresh += 1
        seq = pkt["seq"]

        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) % SEQ_MODULO
            # A huge "gap" means the Pico restarted and its counter reset, not
            # that we lost 60,000 packets. Treat that as a resync.
            if 0 < gap < 1000:
                self.dropped += gap
        self.last_seq = seq

    def note_bad(self):
        self.bad += 1

    def note_reconnect(self):
        self.reconnects += 1
        # The Pico's sequence counter restarts from zero, so the next packet
        # would otherwise look like a colossal gap.
        self.last_seq = None
        # Drop pre-disconnect timestamps too, or the window spans the gap and
        # reads low for several seconds after recovery.
        self._recent.clear()

    def rate_now(self):
        """Packet rate over the recent window -- what the link is doing now."""
        if len(self._recent) < 2:
            return 0.0
        span = self._recent[-1] - self._recent[0]
        return (len(self._recent) - 1) / span if span > 0 else 0.0

    def summary(self, live=True):
        # live: what the link is doing right now.
        # not live: how the whole session went. Different questions.
        if live:
            rate = self.rate_now()
        else:
            elapsed = time.monotonic() - self.started
            rate = self.received / elapsed if elapsed else 0

        total = self.received + self.dropped
        loss = (self.dropped / total * 100) if total else 0
        text = "%d packets  %.1f Hz  %d dropped (%.2f%%)  %d malformed" % (
            self.received, rate, self.dropped, loss, self.bad
        )
        if self.reconnects:
            text += "  %d reconnects" % self.reconnects
        return text


def stream(ser, writer, fh, stats, quiet):
    """Read packets until the link fails. Raises SerialException on unplug."""
    last_report = time.monotonic()

    # Discard whatever partial line we caught by connecting mid-stream.
    ser.readline()

    while True:
        raw = ser.readline().decode("ascii", errors="replace")
        if not raw:
            continue

        pkt = parse(raw)
        if pkt is None:
            stats.note_bad()
            continue

        stats.note(pkt)
        writer.writerow(pkt)

        # Flush periodically. Every row is hard on the SD card; never is asking
        # to lose data when power cuts.
        if stats.received % 100 == 0:
            fh.flush()

        now = time.monotonic()
        if not quiet and now - last_report >= 1.0:
            last_report = now
            if pkt["fix"] == FIX_NONE:
                gps = "no fix (%d sats)" % pkt["sats"]
            else:
                gps = "%.5f,%.5f %5.1f mph  %2d sats  hdop %.2f" % (
                    pkt["lat"], pkt["lon"], pkt["mph"],
                    pkt["sats"], pkt["hdop"])
            print("%-58s  %s" % (stats.summary(), gps))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None,
                    help="serial port; auto-detected if omitted")
    ap.add_argument("--baud", type=int, default=115200,
                    help="ignored over USB, matters on GPIO UART")
    ap.add_argument("--outdir", default="logs")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = os.path.join(args.outdir, "log-%s.csv" % stamp)

    print("Log:   %s" % path)
    print("Ctrl-C to stop.\n")

    stats = LinkStats()

    # One file for the whole session, spanning any number of reconnects. The
    # sequence resync in note_reconnect keeps the drop count honest across them.
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()

        try:
            waiting = False
            while True:
                port = args.port or find_port()

                if port is None or not os.path.exists(port):
                    if not waiting:
                        print("Waiting for device...")
                        waiting = True
                    time.sleep(RECONNECT_DELAY_S)
                    continue

                try:
                    with serial.Serial(port, args.baud, timeout=1) as ser:
                        print("Connected: %s" % port)
                        waiting = False
                        stream(ser, writer, fh, stats, args.quiet)

                except serial.SerialException as exc:
                    # Unplugged, or the port vanished mid-read. Not fatal.
                    fh.flush()
                    print("\nLink lost (%s). Reconnecting...\n"
                          % str(exc).split(":")[0])
                    stats.note_reconnect()
                    time.sleep(RECONNECT_DELAY_S)

        except KeyboardInterrupt:
            fh.flush()
            print("\n\nStopped.")
            print(stats.summary(live=False))
            print("%d fresh GPS solutions" % stats.gps_fresh)
            print("Wrote %s" % path)


if __name__ == "__main__":
    main()