"""
BN-880 (u-blox NEO-M8N) test for Pico 2 W.

WIRING -- verify against the SILKSCREEN on the module, not the cable colours.
The BN-880 labels its pads D G T R V C = SDA, GND, TX, RX, VCC, SCL.

On one common harness that maps to:

    yellow (V, VCC) -> Pico 3V3 OUT (pin 36)   NOT VBUS. The module's TX level
                                               follows its supply and RP2350
                                               GPIO is not 5V tolerant.
    white  (G, GND) -> Pico GND (pin 8 or 38)
    green  (T, TX)  -> Pico GP1 (pin 2)        UART0 RX
    red    (R, RX)  -> Pico GP0 (pin 1)        UART0 TX  -- optional, see below

Black (SDA) and blue (SCL) are the compass. Leave them disconnected.

ABOUT THE MODULE'S RX LINE
-------------------------
If you see "$GNTXT,...,More than 100 frame errors, UART RX was disabled", the
module is complaining about noise on ITS receive pin. An unconnected or idle
GPIO floats, the module reads garbage, and it shuts its receiver off.

It is harmless while you are only reading. You only need that line to send UBX
configuration commands later. Two options:

  - Disconnect the red wire for now. Cleanest.
  - Or leave it: initialising the UART drives TX to a proper idle-high state,
    which stops new errors accruing.

FIRST FIX
---------
Needs sky view. Indoors usually fails. A cold start with no almanac downloads
orbital data at 50 bits/sec and can take several minutes.
"""

import sys
import time
from machine import UART, Pin

BAUD = 9600  # factory default; becomes 115200 after reconfiguration

# timeout matters: without it readline() returns partial lines. This code does
# its own line assembly anyway, which is the robust approach.
uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1), timeout=50)


# --- Line assembly --------------------------------------------------------
# Never trust readline() on a stream like this. Read whatever bytes are
# available, split on newline, and carry the incomplete tail into the next
# pass. A fragment scored as a bad sentence is a bug, not a bad sentence.

class LineReader:
    def __init__(self, uart, max_buffer=1024):
        self._uart = uart
        self._buf = b""
        self._max = max_buffer

    def lines(self):
        """Yield complete lines. Returns nothing if none are ready yet."""
        data = self._uart.read()
        if data:
            self._buf += data

        # Runaway guard: if we somehow never see a newline, don't grow forever.
        if len(self._buf) > self._max:
            self._buf = self._buf[-self._max:]

        while b"\n" in self._buf:
            raw, _, self._buf = self._buf.partition(b"\n")
            try:
                yield raw.decode("ascii").strip()
            except Exception:
                continue


# --- Step 1: confirm the wiring ------------------------------------------

def raw(seconds=20):
    """Dump complete lines only. Run this first."""
    reader = LineReader(uart)
    end = time.ticks_add(time.ticks_ms(), seconds * 1000)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        for line in reader.lines():
            print(line)
        time.sleep_ms(10)


# --- NMEA parsing ---------------------------------------------------------

def valid(sentence):
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, _, cs_text = sentence[1:].strip().partition("*")
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    try:
        return cs == int(cs_text[:2], 16)
    except ValueError:
        return False


def nmea_degrees(value, hemi):
    """NMEA packs coordinates as ddmm.mmmm -- degrees and minutes glued
    together, not decimal degrees. Treating 3721.5 as 37.215 puts you a few
    hundred miles away."""
    if not value:
        return None
    dot = value.find(".")
    deg = float(value[:dot - 2])
    minutes = float(value[dot - 2:])
    result = deg + minutes / 60.0
    return -result if hemi in ("S", "W") else result


class Fix:
    def __init__(self):
        self.valid = False
        self.lat = None
        self.lon = None
        self.speed_mph = 0.0
        self.course = None
        self.utc = None
        self.sats = 0
        self.hdop = None
        self.alt_m = None
        self.sats_visible = 0

    def __repr__(self):
        if not self.valid:
            return "no fix | %d used, %d visible" % (
                self.sats, self.sats_visible
            )
        return "%.6f, %.6f | %.1f mph | %d sats | hdop %s" % (
            self.lat, self.lon, self.speed_mph, self.sats, self.hdop
        )


def update(fix, sentence):
    """Fold one sentence into the running fix."""
    f = sentence.split("*")[0].split(",")
    kind = f[0][3:]  # strip $GN / $GP -- the prefix changes with GNSS config

    if kind == "RMC" and len(f) >= 10:
        fix.valid = (f[2] == "A")
        if fix.valid:
            fix.utc = f[1]
            fix.lat = nmea_degrees(f[3], f[4])
            fix.lon = nmea_degrees(f[5], f[6])
            fix.speed_mph = float(f[7]) * 1.15078 if f[7] else 0.0
            fix.course = float(f[8]) if f[8] else None
        return True

    if kind == "GGA" and len(f) >= 10:
        fix.sats = int(f[7]) if f[7] else 0
        fix.hdop = float(f[8]) if f[8] else None
        fix.alt_m = float(f[9]) if f[9] else None
        return True

    if kind == "GSV" and len(f) >= 4:
        # Field 3 is satellites in view for this constellation.
        try:
            fix.sats_visible = int(f[3])
        except ValueError:
            pass
        return True

    return False


# --- Step 2: watch a live fix --------------------------------------------

def main():
    fix = Fix()
    reader = LineReader(uart)
    last_print = time.ticks_ms()
    ok = 0
    bad = 0

    print("Waiting for satellites. Go outside.\n")

    while True:
        for line in reader.lines():
            if not line:
                continue
            if valid(line):
                ok += 1
                update(fix, line)
            else:
                bad += 1

        if time.ticks_diff(time.ticks_ms(), last_print) >= 1000:
            last_print = time.ticks_ms()
            print("%-52s  %d ok / %d bad" % (repr(fix), ok, bad))

        time.sleep_ms(10)


if __name__ == "__main__":
    main()