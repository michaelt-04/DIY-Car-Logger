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
from datetime import datetime
from collections import deque

try:
    import serial
except ImportError:
    sys.exit("pyserial missing. Try: sudo apt install python3-serial")


FIELDS = ("seq", "t_ms", "ax", "ay", "az", "rpm", "afr", "map")
FIELD_TYPES = (int, int, float, float, float, float, float, float)

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
        self.started = time.monotonic()
        self._recent = deque(maxlen=200)

    def note(self, pkt):
        self._recent.append(time.monotonic())
        self.received += 1
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
        self._recent.clear()
        self.reconnects += 1
        # The Pico's sequence counter restarts from zero, so the next packet
        # would otherwise look like a colossal gap.
        self.last_seq = None

    def summary(self, live=True):
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

    def rate_now(self):
        if len(self._recent) < 2:
            return 0.0
        span = self._recent[-1] - self._recent[0]
        return (len(self._recent) - 1) / span if span > 0 else 0.0


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
            print("%-62s  rpm %5.0f  afr %5.2f"
                  % (stats.summary(), pkt["rpm"], pkt["afr"]))


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
            print("Wrote %s" % path)


if __name__ == "__main__":
    main()