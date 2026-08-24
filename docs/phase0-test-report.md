# Phase 0 — Transport Layer Validation

**Date:** 24 August 2026
**Goal:** prove the Pico-to-Pi data pipeline works and characterise its limits,
using synthetic data, before any real sensor exists.

---

## Why synthetic data

With a real sensor in the loop, a wrong number has five possible causes: the
sensor, the wiring, the bus configuration, the parsing, or the logger. With
generated data the correct answer is known in advance, so any discrepancy can
only be the transport and parsing code.

Debugging the hard part once, in isolation, means every sensor added later slots
into a pipeline that's already trusted.

---

## Test environment

| | |
|---|---|
| Front end | Raspberry Pi Pico 2 W, MicroPython v1.28.0 (RP2350 ARM build) |
| Logger | Raspberry Pi Zero 2 W, Raspberry Pi OS Lite 64-bit, headless |
| Dev machine | MacBook Air M1, macOS |
| Link | USB CDC serial; OTG adapter on the Pi side |
| Packet format | `$seq,t_ms,ax,ay,az,rpm,afr,map*CS\r\n`, NMEA-style XOR checksum |

---

## Results

### Baseline — macOS, 50 Hz

| Metric | Result |
|---|---|
| Packets | 1,315 |
| Rate | 49.9 Hz (target 50) |
| Dropped | 0 |
| Malformed | 0 |

Rate converged on target rather than drifting below it, confirming the absolute
scheduling in the sender works. A naive `sleep(period)` would have shown a
persistent shortfall as per-packet work accumulated.

### Disconnect behaviour — first attempt

Unplugging the Pico mid-stream raised an unhandled `SerialException` and killed
the process.

Data was **not** lost — the `with open(...)` block closed and flushed on the way
out. Verified: 560 lines on disk (559 rows + header) against ~556 packets counted
before the crash.

But process death is the wrong response. In the car, one connector losing contact
over a bump would stop logging for the remainder of the drive with no indication
until afterward.

### Sequence integrity

```
rows 2103   gaps 0
```

Every packet the receiver reported landed on disk with an unbroken sequence. The
pipeline is lossless end to end, not merely lossless in flight.

### Rate ceiling

`SAMPLE_HZ` swept on the sender, macOS receiving:

| Target | Achieved | Dropped | Malformed | Behaviour |
|---|---|---|---|---|
| 50 Hz | 49.9–50.0 | 0 | 0 | Locked to target |
| 200 Hz | 199.4–200.0 | 0 | 0 | Locked to target |
| 500 Hz | 439.8–500.1 | 0 | 0 | At the edge, some variance |
| 1000 Hz | 437.1–650.3 | 0 | 0 | Free-running, ~530 Hz average |

**The ceiling is the Pico, not the link.** At 1000 Hz the requested period
(`1000 // 1000` = 1 ms) falls below what `time.sleep_ms()` can resolve, so the
scheduler stops governing and the loop free-runs. What's being measured at that
point is how fast MicroPython can format a string and push it to USB — roughly
1.9 ms per packet.

Zero drops at every rate, including while free-running. The transport never
became the limiting factor.

### Raspberry Pi — throughput

| Metric | Result |
|---|---|
| Packets | 169,569 |
| Duration | ~6.5 minutes |
| Rate | ~438 Hz |
| Dropped | 0 |
| Malformed | 0 |
| Reconnects survived | 1 (earlier run) |

The Pi sustained ~438–455 Hz against the Mac's ~530 Hz on identical Pico
firmware, so **on this pairing the Pi is the bottleneck**, not the
microcontroller. Still roughly 4× the Phase 1 requirement.

### Raspberry Pi — thermal

Five-minute soak at ~430 packets/sec, open bench, ~22 °C ambient:

```
41.9 → 45.1 → 47.8 → 49.4 → 51.5 → 53.7 °C
```

Still climbing slowly at cutoff; a longer run would likely have settled in the
mid-50s.

**The number that matters is the rise above ambient: roughly 30 °C.** The Zero
2 W soft-throttles at 80 °C. On a bench that's enormous margin. In a closed car
interior reaching 60–70 °C on a summer day, 65 + 30 lands at 95 °C — past
throttling, before the engine has even started.

Mitigating factors: real Phase 1 load is 100 Hz, not 430, so the delta should be
smaller — perhaps 15–20 °C. But this is a design constraint for the enclosure,
not a solved problem.

---

## Bugs found and fixed

Seven, all in the receiver. Every one of them would have been significantly
harder to diagnose with real sensors attached.

**1. Disconnect killed the process.**
Unhandled `SerialException`. Fixed with an outer reconnect loop that treats link
loss as a normal event.

**2. Sequence resync on reconnect.**
When the Pico restarts, its counter returns to zero. Without resetting
`last_seq`, the next packet registered as a ~65,000-packet gap, permanently
poisoning the loss statistics. Fixed in `note_reconnect()`.

**3. `find_port()` exited when no device present.**
Wrong for something that should start at ignition-on and wait for hardware to
come up. Now returns `None` so the reconnect loop can keep waiting.

**4. macOS port pattern not matched.**
Only globbed `/dev/ttyACM*`. Added `/dev/tty.usbmodem*` so the same script runs
unmodified on both machines.

**5. Rate was a lifetime average.**
Time spent disconnected counted as elapsed time with no packets, so a single
early hiccup would depress the displayed rate for the rest of the session and
mask genuine degradation later. Fixed with a rolling 200-sample window.

**6. Rolling window spanned the disconnect.**
After reconnecting, the deque still held pre-unplug timestamps, showing ~20 Hz
for four seconds before correcting. Fixed by clearing the window in
`note_reconnect()`.

**7. Final summary showed the rolling rate.**
On exit, the session total should be a lifetime average, not the last four
seconds. Fixed with a `live=` flag — live display answers "is the link healthy
now," end-of-session answers "how did this drive go."

---

## What this proves

- Framing, checksums, and parsing are correct — 0 malformed across ~200,000
  packets
- Nothing is lost in transit — 0 dropped at 50, 200, 500, and 1000 Hz
- Disconnection is survivable and does not corrupt statistics
- Data on disk matches data received
- Headroom is roughly 4× the Phase 1 requirement
- The OTG adapter and cable chain work under sustained load

---

## What remains untested

Worth being explicit, because the results above are easy to over-read.

**The workload is not representative.** The sender currently does eight
`sin()` calls and a string format. Real firmware will do I2C transactions, NMEA
parsing, and ADC reads — all more expensive per cycle. Re-measure once real
sensors are running. A rate that *wanders* rather than dropping packets is the
signature of the Pico saturating.

**Everything physical is unproven.** All testing was on a bench at room
temperature with clean USB power:

- Power behaviour under cranking dips and load dump
- Vibration, particularly connector fatigue
- Thermal performance inside an enclosure in a hot car
- EMI from a points ignition system
- The GPIO UART path — only USB has been exercised, and baud rate is meaningless
  over USB CDC

**No watchdog.** If the Pico stops sending while remaining enumerated, the
receiver waits indefinitely with no complaint. A "no valid packet in 2 seconds"
check is the eventual answer, needed before the system runs unattended.

---

## Actions arising

- [ ] Add Pi core temperature as a logged channel — free via
      `/sys/class/thermal/thermal_zone0/temp`, and turns future thermal
      misbehaviour into evidence rather than guesswork
- [ ] Design the enclosure with real venting; do not seal it
- [ ] Add a stall watchdog before unattended operation
- [ ] Decide GPS/IMU rate mismatch handling (10 Hz vs 100 Hz) before writing
      sensor code — likely a fixed layout with a freshness flag
- [ ] Re-measure achieved rate once real sensor reads replace `build_packet()`

---

## Conclusion

The transport layer is finished and characterised. It will not be the limiting
factor in this project.

What has *not* been demonstrated is anything about the physical environment,
which is where builds like this actually fail. That was always Phase 1's real
purpose: get something boring working in the car before adding channels. The
software half is now ahead of schedule.
