# GD188a – Measurement chip UART protocol v0.2

What is known so far about the communication between the main MCU (STM32F410RB) and the
measurement chip in the GD188a. Everything here was reverse-engineered from sigrok captures
of the original firmware and from tests on the device with the custom firmware.

Legend used below:

- **Verified** – seen in sigrok captures of the original firmware and/or confirmed by measurement.
- **Assumed** – consistent with the data so far, but not yet confirmed.
- **Unknown** – not investigated yet.

---

## 1. The chip

- The measurement chip has no readable marking. It is most likely a DTM0660 clone or a
  DTM0660-class chip with manufacturer-specific firmware ("fake DTM0660").
- It has its own 24C02 EEPROM, which very likely holds the factory calibration.
  **Do not write to that EEPROM** – the calibration would be lost.
  (This is a different EEPROM from the BL24C256A used by the MCU for settings.)
- A standard DTM0660 does not accept commands like these; the protocol below was designed by
  the device manufacturer. Similar-looking devices (MDS9208, ET928) may use the same protocol,
  but this is not verified.

## 2. Physical layer

| Item | Value | Status |
| --- | --- | --- |
| MCU peripheral | USART6 | Verified |
| MCU TX → chip RX | PA11 | Verified |
| Chip TX → MCU RX | PA12 | Verified |
| Format | 9600 Bd, 8 data bits, no parity, 1 stop bit | Verified |
| Chip power | Switched together with the display by PA2 (A3V) | Verified |
| Sigrok channels | D1 = MCU → chip, D0 = chip → MCU | Verified |

## 3. Packet format

Every packet (both directions) is:

```
<header> <payload bytes ...> <checksum>
```

- **Checksum** = sum of all preceding bytes of the packet, modulo 256. **Verified**
- Single-byte commands are therefore sent twice, e.g. `5A 5A`, `60 60` (header + checksum).
- Multi-byte values are **little-endian** (least significant byte first). **Verified**
- Bytes of one packet follow each other without gaps; a reply arrives roughly 5–30 ms after
  the request. The custom firmware treats a gap longer than 5 ms inside a packet as the start
  of a new packet.

### Packet lengths by header

| Header | Direction | Length (incl. checksum) | Meaning |
| --- | --- | --- | --- |
| `A0` | chip → MCU | 3 | Boot message |
| `58` | both | 3 | Set function (mode) / echo |
| `5A` | both | MCU: 2, chip: 3 | Auto range / confirmation |
| `5B` | both | MCU: 2, chip: 3 | Manual range step / confirmation |
| `60` | both | MCU: 2, chip: 6 | Read main measurement |
| `61` | both | MCU: 2, chip: 7 | Read frequency (40-bit value) |
| `62` | both | MCU: 2, chip: 4 | Read duty cycle |

## 4. Messages

### 4.1 Boot message (chip → MCU) – Verified

After power-up the chip sends:

```
A0 01 A1
```

If the chip sends it again while running, it has reset itself and the function must be set again.

### 4.2 Set function – `58` – Verified

```
MCU:  58 mm CS
chip: 58 mm CS      (echo of the same packet)
```

`mm` = function code (see section 5). After the echo the chip is in AUTO range.
The custom firmware always follows the echo with `5A` (auto range), as the original firmware does.

### 4.3 Auto range – `5A` – Verified

```
MCU:  5A 5A
chip: 5A rr CS      rr = current range index
```

### 4.4 Manual range – `5B` – Verified

```
MCU:  5B 5B
chip: 5B rr CS      rr = range index now active
```

- Each `5B` switches to manual range and steps to the next range; after the highest range it
  wraps around to range 0. **Verified** (resistance: 05 → 5B → 00).
- Whether the first `5B` after AUTO keeps the current range or already steps is not fully
  clear. The custom firmware therefore restores a stored manual range by sending `5B`
  repeatedly until the reply `rr` equals the wanted range (max. 8 attempts). **Assumed**
- `5B` is not repeated on timeout during a user key press, because a lost reply could mean the
  chip already stepped.

### 4.5 Read main measurement – `60` – Verified

```
MCU:  60 60
chip: 60 v0 v1 v2 rr CS
```

| Field | Meaning |
| --- | --- |
| `v0 v1 v2` | 24-bit **signed** value, little-endian (two's complement) |
| `rr` | Current range index |
| `FF FF 7F` | Overload (value 0x7FFFFF) – shown as 0L |

Used for voltage, current, resistance, diode, continuity and capacitance.

### 4.6 Read frequency – `61` – Verified

```
MCU:  61 61
chip: 61 v0 v1 v2 v3 v4 CS
```

- `v0..v4` = **40-bit unsigned value in mHz**, little-endian (`v4` = most significant byte).
  **Verified**
- There is no range byte. Earlier the fifth byte was taken as a range index because it was always
  00 – that holds only below 4.29 MHz (2^32 mHz). 20 MHz = 2·10^10 mHz needs 35 bits.
- Measured with an external signal generator (custom firmware v0.52):

| Signal | Bytes `v0..v4` | Value (mHz) | Displayed |
| --- | --- | --- | --- |
| 1 kHz | `40 42 0F 00 00` | 1 000 000 | 1.000 kHz |
| 1 MHz | `0F F1 9A 3B 00` | 1 000 009 999 | 1.000 MHz |
| 4 MHz | `A0 C4 6B EE 00` | 4 000 040 096 | 4.000 MHz |
| 5 MHz | `80 AD 06 2A 01` | 5 000 048 000 | 5.000 MHz |
| 12 MHz | `C0 4C 43 CB 02` | 12 000 120 000 | 12.00 MHz |

- The offset of about +10 ppm comes from the crystal tolerance of the generator or the meter.
- Mains (50 Hz) reads as 50000 (50.000 Hz).
- Manufacturer specification: 1 Hz – 20 MHz; 1 Hz – 5 MHz needs more than 500 mVpp,
  5–10 MHz more than 1 Vpp, 10–20 MHz more than 3 Vpp (above 20 MHz indicative only).
- The custom firmware shows 4 digits with an automatic unit: Hz below 1 kHz, kHz below 1 MHz,
  MHz above (e.g. 50.00 Hz, 999.9 kHz, 12.00 MHz).

### 4.7 Read duty cycle – `62` – Verified

```
MCU:  62 62
chip: 62 v0 v1 CS
```

- `v0 v1` = 16-bit unsigned value, little-endian. **Verified**
- One digit = **0.01 %** (6959 = 69.59 %). **Verified** – an external generator set to 55 %
  reads 55.06 %.
- There is no range byte.

## 5. Function codes (`58 mm`)

| Code | Function | Read with | Status |
| --- | --- | --- | --- |
| `01` | DC mV | `60` | Verified |
| `02` | AC mV | `60` | Verified |
| `05` | DC V | `60` | Verified |
| `06` | AC V | `60` (also `61`/`62` for Hz/% in the original FW) | Verified |
| `07` | Resistance | `60` | Verified |
| `09` | Continuity | `60` | Verified |
| `0A` | Diode | `60` | Verified |
| `0B` | Capacitance | `60` | Verified |
| `0C` | DC µA | `60` | Verified |
| `0D` | AC µA | `60` | Verified |
| `0E` | DC mA | `60` | Verified |
| `0F` | AC mA | `60` | Verified |
| `10` | DC 10 A | `60` | Verified |
| `11` | AC 10 A | `60` | Verified |
| `12` | Frequency / duty (Hz/% position) | `61` = Hz, `62` = % | Verified |
| `00`, `03`, `04`, `08`, `13`+ | – | – | Unknown (possibly temperature, hFE, NCV …) |

Notes from the original firmware:

- In the Hz/% position it sends `58 12` and then polls `61` (Hz) or `62` (%). Switching between
  Hz and % sends `58 12` again.
- In the AC V position, Hz and % are also available: `58 06` followed by `61` or `62`.
- The original firmware polls the selected read command about every **200 ms**.

## 6. Range indices (`rr`) and value scaling

The displayed value = raw value × weight of one digit for the given range.

### Voltage and current – Verified

| Function | rr = 0 | 1 | 2 | 3 | 4 |
| --- | --- | --- | --- | --- | --- |
| V (`05`/`06`) | 600.0 mV | 6.000 V | 60.00 V | 600.0 V | 1000 V |
| mV (`01`/`02`) | 60.00 mV | 600.0 mV | – | – | – |
| µA (`0C`/`0D`) | 600.0 µA | 6000 µA | – | – | – |
| mA (`0E`/`0F`) | 60.00 mA | 600.0 mA | – | – | – |
| 10 A (`10`/`11`) | 6.000 A | 10.00 A | – | – | – |

In mV the chip may briefly report a range index above 1 while auto-ranging; such samples are
ignored.

### Resistance (`07`) – Verified

| rr | 0 | 1 | 2 | 3 | 4 | 5 |
| --- | --- | --- | --- | --- | --- | --- |
| Range | 600.0 Ω | 6.000 kΩ | 60.00 kΩ | 600.0 kΩ | 6.000 MΩ | 60.00 MΩ |

- In AUTO the chip starts at range 5 (MΩ).
- With open probes the chip reports values above 6000 on range 5 (e.g. 10450) – treated as 0L.

### Diode (`0A`) – Verified

- Range 0, value in mV → x.xxx V.
- Open probes read about 3237 (3.237 V); values above 3200 are treated as 0L.

### Continuity (`09`) – Verified

- Range 0 only, 600.0 Ω (0.1 Ω per digit).
- Open probes read e.g. 10876; values above 6000 (600 Ω) are treated as 0L.

### Capacitance (`0B`) – Verified (ranges 0–3)

The range index does **not** follow a simple ×10 pattern, and the raw value can exceed 9999.

| rr | Weight of one digit | Measured example |
| --- | --- | --- |
| 0 | 1 pF | ~100 nF → 103325; open leads → 28–33 |
| 1 | 1 nF | ~4.7 µF → 5062 |
| 2 | 10 nF | ~210 µF → 20749 |
| 3 | 1 µF | ~3300 µF → 3357 |
| 4 | 10 µF | Assumed (manufacturer spec, not measured) |

Manufacturer specification for comparison: 99.99 nF / 999.9 nF / 9.999 µF / 99.99 µF /
999.9 µF / 9.999 mF / 99.99 mF, resolution 10 pF … 10 µF, accuracy ±(3.0 % + 10 dgt) to
±(5.0 % + 15 dgt).

The custom firmware converts the value to pF and chooses the display range itself (9999 counts,
with hysteresis).

## 7. Recommended sequence

```
power on (PA2 = 1)
  chip → A0 01 A1                       boot message
MCU  → 58 mm CS                          set function
  chip → 58 mm CS                       echo
MCU  → 5A 5A                             auto range
  chip → 5A rr CS
(optional) MCU → 5B 5B  ... until rr = wanted range
loop every ~200 ms:
  MCU  → 60 60 / 61 61 / 62 62
  chip → data packet
```

Robustness rules used by the custom firmware:

- Never transmit from the UART receive interrupt; send from the main loop only.
- Resend `58` or `5A` if no reply arrives within 500 ms; after 3 failures show
  "chip not responding".
- If no data packet arrives for 2 s, set the function again.
- If `A0 01 A1` arrives during operation, the chip has reset – set the function again.
- Discard data packets that arrive while a mode change is still in progress (they may belong
  to the previous function).
- On UART overrun, restart reception (otherwise HAL stops receiving for good).

## 8. Open questions

- Capacitance range 4 (10 µF per digit?) – needs a capacitor above ~10 000 µF.
- Unused function codes (`00`, `03`, `04`, `08`, `13`+) – temperature, hFE, NCV, …?
- Whether other commands exist (e.g. reading or writing the chip's 24C02 calibration).
- Whether MDS9208 and ET928 use the same chip and protocol.
