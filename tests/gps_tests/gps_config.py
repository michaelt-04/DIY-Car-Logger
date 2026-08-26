"""
Configure a BN-880 (u-blox NEO-M8N) from the Pico. No Windows or u-center
required.

Target configuration:
    10 Hz navigation rate
    GPS + SBAS + QZSS only (GLONASS and BeiDou disabled)
    115200 baud
    RMC and GGA every solution, GSV once per second, everything else off

WHY GPS-ONLY
------------
The M8 series does 10 Hz on a single constellation but only 5 Hz with
concurrent GNSS. For a car logger, temporal resolution beats satellite count:
you care about resolving what happened during a corner, not about holding a
fix in a parking garage. QZSS stays enabled because u-blox specifies it should
be enabled or disabled together with GPS.

WIRING
------
Reconnect the module's RX line for this -- Pico GP0 (pin 1) -> module R.
You cannot configure a receiver you can only listen to.

SETTINGS ARE NOT SAVED
----------------------
This deliberately does not issue CFG-CFG to persist anything. Re-sending the
configuration at every boot is more robust than relying on a backup battery
that may be flat after the car sits for a month: the module's state is then
always known, and the configuration lives in version control rather than in a
capacitor.
"""

import time
from machine import UART, Pin

TX_PIN = 0   # Pico GP0 -> module RX
RX_PIN = 1   # Pico GP1 <- module TX

TARGET_BAUD = 115200
NAV_RATE_MS = 100          # 100 ms == 10 Hz


# --- UBX framing ----------------------------------------------------------

def ubx(cls, msg_id, payload=b""):
    """Build a UBX frame: sync, header, payload, Fletcher-8 checksum."""
    body = bytes([cls, msg_id]) + len(payload).to_bytes(2, "little") + payload
    a = b = 0
    for ch in body:
        a = (a + ch) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


# Message classes/IDs used below
CFG = 0x06
CFG_PRT, CFG_MSG, CFG_RATE, CFG_GNSS = 0x00, 0x01, 0x08, 0x3E
ACK_CLS, ACK_ACK, ACK_NAK = 0x05, 0x01, 0x00

# NMEA standard message IDs (class 0xF0)
NMEA = 0xF0
GGA, GLL, GSA, GSV, RMC, VTG = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05


def wait_ack(uart, cls, msg_id, timeout_ms=1500):
    """Look for UBX-ACK-ACK matching this message.

    Worth doing: without it you have no idea whether the receiver accepted the
    configuration or silently ignored it.
    """
    want = bytes([0xB5, 0x62, ACK_CLS, ACK_ACK])
    nak = bytes([0xB5, 0x62, ACK_CLS, ACK_NAK])
    buf = b""
    end = time.ticks_add(time.ticks_ms(), timeout_ms)

    while time.ticks_diff(end, time.ticks_ms()) > 0:
        data = uart.read()
        if data:
            buf += data
            buf = buf[-256:]          # ACKs are tiny; don't hoard NMEA
            for marker, result in ((want, True), (nak, False)):
                i = buf.find(marker)
                # payload is clsID, msgID at offset 6,7
                if i >= 0 and len(buf) >= i + 8:
                    if buf[i + 6] == cls and buf[i + 7] == msg_id:
                        return result
        time.sleep_ms(10)
    return None                        # timed out


def send(uart, cls, msg_id, payload=b"", label="", expect_ack=True):
    uart.read()                        # clear anything pending
    uart.write(ubx(cls, msg_id, payload))
    if not expect_ack:
        print("  %-28s sent (no ack expected)" % label)
        return True
    result = wait_ack(uart, cls, msg_id)
    print("  %-28s %s" % (
        label,
        "ok" if result is True else ("REJECTED" if result is False else "no ack")
    ))
    return result is True


# --- Payload builders -----------------------------------------------------

def cfg_msg(msg_id, rate):
    """Set output rate for an NMEA sentence on the current port.

    rate is per navigation solution: 1 = every solution, 10 = every tenth,
    0 = off.
    """
    return bytes([NMEA, msg_id, rate])


def cfg_rate(ms):
    return (ms).to_bytes(2, "little") + (1).to_bytes(2, "little") \
        + (1).to_bytes(2, "little")


def cfg_prt_uart(baud):
    """UART1 config: 8N1, UBX+NMEA in and out, at the given baud."""
    return (
        bytes([0x01, 0x00])            # portID 1, reserved
        + (0).to_bytes(2, "little")    # txReady off
        + (0x000008D0).to_bytes(4, "little")   # 8 bits, no parity, 1 stop
        + baud.to_bytes(4, "little")
        + (0x0003).to_bytes(2, "little")       # inProtoMask:  UBX + NMEA
        + (0x0003).to_bytes(2, "little")       # outProtoMask: UBX + NMEA
        + (0).to_bytes(2, "little")            # flags
        + (0).to_bytes(2, "little")            # reserved
    )


def _gnss_block(gnss_id, res_ch, max_ch, enable):
    flags = (0x01010000 if enable else 0x00010000) | (1 if enable else 0)
    return bytes([gnss_id, res_ch, max_ch, 0]) + flags.to_bytes(4, "little")


def cfg_gnss_gps_only():
    """GPS + SBAS + QZSS enabled; GLONASS, BeiDou, Galileo off.

    Required for 10 Hz -- concurrent GNSS caps the M8 at 5 Hz.
    """
    blocks = (
        _gnss_block(0, 8, 16, True)    # GPS
        + _gnss_block(1, 1, 3, True)   # SBAS
        + _gnss_block(2, 0, 0, False)  # Galileo
        + _gnss_block(3, 0, 0, False)  # BeiDou
        + _gnss_block(5, 0, 3, True)   # QZSS -- must match GPS
        + _gnss_block(6, 0, 0, False)  # GLONASS
    )
    return bytes([0x00, 0x00, 0xFF, 6]) + blocks


# --- Baud detection -------------------------------------------------------

def detect_baud(candidates=(9600, 115200), listen_ms=1500):
    """Find the module's current baud by listening for valid NMEA.

    Needed because this runs at every boot: the module may already be
    configured from a previous session, or freshly reset to factory 9600.
    """
    for baud in candidates:
        uart = UART(0, baudrate=baud, tx=Pin(TX_PIN), rx=Pin(RX_PIN),
                    timeout=50)
        time.sleep_ms(200)
        uart.read()

        buf = b""
        end = time.ticks_add(time.ticks_ms(), listen_ms)
        while time.ticks_diff(end, time.ticks_ms()) > 0:
            data = uart.read()
            if data:
                buf += data
            if b"$G" in buf and b"\n" in buf:
                print("Module is at %d baud" % baud)
                return uart, baud
            time.sleep_ms(20)

    return None, None


# --- Main -----------------------------------------------------------------

def configure():
    print("Detecting current baud rate...")
    uart, baud = detect_baud()
    if uart is None:
        print("No NMEA seen at any baud. Check wiring and power.")
        return None

    print("\nTrimming NMEA output:")
    # Do this first -- less traffic makes everything after it more reliable.
    send(uart, CFG, CFG_MSG, cfg_msg(GLL, 0), "GLL off")
    send(uart, CFG, CFG_MSG, cfg_msg(GSA, 0), "GSA off")
    send(uart, CFG, CFG_MSG, cfg_msg(VTG, 0), "VTG off")
    send(uart, CFG, CFG_MSG, cfg_msg(GSV, 10), "GSV every 10th")
    send(uart, CFG, CFG_MSG, cfg_msg(RMC, 1), "RMC every solution")
    send(uart, CFG, CFG_MSG, cfg_msg(GGA, 1), "GGA every solution")

    print("\nGNSS constellations:")
    # Restarts the GNSS subsystem, so allow time and expect a brief signal gap.
    send(uart, CFG, CFG_GNSS, cfg_gnss_gps_only(), "GPS + SBAS + QZSS only")
    time.sleep_ms(1500)

    if baud != TARGET_BAUD:
        print("\nBaud rate:")
        # No ACK wait here: the module switches immediately and the reply race
        # is not worth fighting.
        send(uart, CFG, CFG_PRT, cfg_prt_uart(TARGET_BAUD),
             "switch to %d" % TARGET_BAUD, expect_ack=False)
        time.sleep_ms(300)
        uart = UART(0, baudrate=TARGET_BAUD, tx=Pin(TX_PIN), rx=Pin(RX_PIN),
                    timeout=50)
        time.sleep_ms(200)
        uart.read()

    print("\nNavigation rate:")
    send(uart, CFG, CFG_RATE, cfg_rate(NAV_RATE_MS),
         "%d ms (%d Hz)" % (NAV_RATE_MS, 1000 // NAV_RATE_MS))

    return uart


def verify(uart, seconds=10):
    """Count RMC sentences to confirm the actual achieved rate.

    The ACKs say the receiver accepted the settings. This says it is doing
    what was asked -- not the same claim.
    """
    print("\nMeasuring actual rate for %d seconds..." % seconds)
    buf = b""
    rmc = 0
    total = 0
    start = time.ticks_ms()
    end = time.ticks_add(start, seconds * 1000)

    while time.ticks_diff(end, time.ticks_ms()) > 0:
        data = uart.read()
        if data:
            buf += data
        while b"\n" in buf:
            raw, _, buf = buf.partition(b"\n")
            try:
                line = raw.decode("ascii").strip()
            except Exception:
                continue
            if line.startswith("$"):
                total += 1
                if line[3:6] == "RMC":
                    rmc += 1
        time.sleep_ms(5)

    elapsed = time.ticks_diff(time.ticks_ms(), start) / 1000.0
    print("  RMC:  %.1f Hz   (%d in %.1f s)" % (rmc / elapsed, rmc, elapsed))
    print("  All sentences: %.1f Hz" % (total / elapsed))
    if rmc / elapsed < 8:
        print("  Below 10 Hz -- check the GNSS config was accepted.")


if __name__ == "__main__":
    uart = configure()
    if uart:
        verify(uart)