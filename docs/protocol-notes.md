# SpyderX-family protocol notes

Sources: independent reverse-engineering by the PsyCalibrator project
(https://github.com/yangzhangpsy/PsyCalibrator, see spyderX.m / spyderX2.m),
corroborated against observed device descriptors. Reimplemented, not
ported — Yutani is Apache-2.0 and contains no code from these sources.

Observed devices: SpyderX2 (085c:0a0a), Spyder/SpyderPro 2024 (085c:0a0b).

## Transport
- Vendor-specific interface (class 0xff), bulk OUT 0x01, bulk IN 0x81
- Command frame: `[cmd, nonce_lo, nonce_hi, size_hi, size_lo, payload…]`
- A vendor control transfer precedes every measurement:
  `bmRequestType 0x41, bRequest 2, wValue 2, wIndex 0, wLength 0`
- Response prefix: 5-byte header (`[nonce_lo, nonce_hi, status, size_hi, size_lo]`), then payload
- Channel measurement data: 16-bit big-endian (`u16`)
- Calibration response matrices: IEEE754 32-bit floats (`f32`, little-endian)

## Command set
| op            | SpyderX                   | SpyderX2/X2-family                    | resp    |
|---------------|---------------------------|---------------------------------------|---------|
| init / HW ver | `d9 42 33 00 00`          | `c2 42 33 00 00`                      | 28/42 B |
| serial        | `c2 5c 37 00 00`          | (from init)                           | 42 B    |
| factory cal   | `cb 05 73 00 01 00`       | `f6 5c 37 00 01 00`                   | 47/113 B|
| ambient       | `d4 a1 c5 00 02 65 10`    | `d4 05 73 00 02 65 10`                | 11 B    |
| setup (v1)    | `c3 29 27 00 01 v1`       | `f7 a1 c5 00 01 v1`                   | 15/27 B |
| measure       | `d2 3f b9 00 07 [v2,s1,s2]`| `f2 29 27 00 0f [s2,s1,s3,s4]`       | 13/17 B |

Ambient uses fixed integration time `0x65` and gain `0x10`.

## Confirmed on hardware (2026-10-22, SpyderX2 085c:0a0a)
- **Init (`c2 42 33 00 00`)**: 42-byte response.
  - Bytes `0..5`: `42 33 00 00 25` (nonce echo + 37-byte payload length)
  - Bytes `5..17`: ASCII `"5.5080005681"` (FW `5.50`, Serial `80005681`)
- **Factory Cal (`f6 5c 37 00 01 00`)**: 113-byte response (`5` header + `108` payload).
  - Byte `5`: `0x00` (cal index)
  - Byte `6`: `v1` (`0x03`)
  - Bytes `7..9`: `v2` (`0x02ca` = 714 ms integration time, BE `u16`)
  - Bytes `9..15`: `00 01 02 03 04 05` (channel indices)
  - Bytes `15..51`: Primary `3x3` XYZ calibration matrix (9 × `f32` LE, row-major X, Y, Z)
  - Bytes `51..87`: Secondary `3x3` matrix (9 × `f32` LE)
  - Bytes `87..111`: 3 × `(gain, offset)` pairs (6 × `f32` LE: `1.0, 0.0, 1.0, 0.0, 1.0, 0.0`)
  - Bytes `111..113`: `v3` (`0x01`), `v4` (`0xba`)
- **Setup (`f7 a1 c5 00 01 v1`)**: 27-byte response (`5` header + `22` payload).
  - Bytes `5..20` (15 bytes): `[s2 (2B = 02 ca), s1 (1B = 03), s3 (6B = 00..05), s4 (6B = 9f bf 9f bf bf 9f)]` — passed directly as the 15-byte payload of `measure` (`f2 29 27 00 0f`).
  - Bytes `20..26` (6 bytes): `setup_off` per-channel baseline offset (`01 01 01 01 01 02`).
  - Byte `26`: `0xff`.
- **Measure (`f2 29 27 00 0f` + `setup[5..20]`)**: Preceded by vendor reset control transfer (`0x41, 0x02, 0x0002, 0`). Returns 17 bytes (`5` header + `12` payload = six `u16` BE channel counts). First 3 channels (`ch[0..3]`) map via the `3x3` factory calibration matrix to CIE `XYZ` (in cd/m² nits).

