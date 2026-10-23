// Copyright 2026 <Visitant Media LLC> — Apache License, Version 2.0 (same terms as mod.rs)

//! SpyderX-family protocol, independently reimplemented in Rust from
//! publicly documented device behaviour (see docs/protocol-notes.md).

use std::time::Duration;

use anyhow::{bail, Context, Result};
use rusb::{Context as LibUsb, DeviceHandle, UsbContext};

use super::known_model;

/// Bulk endpoints, per device descriptors.
const EP_OUT: u8 = 0x01;
const EP_IN: u8 = 0x81;

/// Control transfer sent before measurements ("spyder reset").
const RESET_REQUEST_TYPE: u8 = 0x41;
const RESET_REQUEST: u8 = 0x02;
const RESET_VALUE: u16 = 0x0002;

const READ_TIMEOUT: Duration = Duration::from_secs(10);

/// Read buffer size. Must be >= the endpoint's max packet size (64 B);
/// libusb reports `Overflow` rather than truncating if a packet is larger
/// than the buffer we hand it.
const READ_BUF_SIZE: usize = 512;

/// A command set variant. The 2024-era devices share firmware lineage;
/// which set a given PID accepts is probed empirically.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CmdSet {
    SpyderX,
    SpyderX2,
}

impl CmdSet {
    pub fn name(self) -> &'static str {
        match self {
            CmdSet::SpyderX => "SpyderX",
            CmdSet::SpyderX2 => "SpyderX2",
        }
    }

    /// Init / hardware-version command, observed byte frames.
    fn init_cmd(self) -> Vec<u8> {
        match self {
            CmdSet::SpyderX => vec![0xd9, 0x42, 0x33, 0x00, 0x00],
            CmdSet::SpyderX2 => vec![0xc2, 0x42, 0x33, 0x00, 0x00],
        }
    }

    /// Nominal init response length. Advisory only: the SpyderX2 has been
    /// observed returning 42 bytes where the public notes suggest 28.
    fn init_resp_len(self) -> usize {
        match self {
            CmdSet::SpyderX => 28,
            CmdSet::SpyderX2 => 42,
        }
    }

    /// Factory calibration read command.
    fn cal_cmd(self) -> Vec<u8> {
        match self {
            CmdSet::SpyderX => vec![0xcb, 0x05, 0x73, 0x00, 0x01, 0x00],
            CmdSet::SpyderX2 => vec![0xf6, 0x5c, 0x37, 0x00, 0x01, 0x00],
        }
    }

    fn cal_resp_len(self) -> usize {
        match self {
            CmdSet::SpyderX => 47,
            CmdSet::SpyderX2 => 113,
        }
    }

    /// Sensor setup command (takes `v1` from factory calibration).
    fn setup_cmd(self, v1: u8) -> Vec<u8> {
        match self {
            CmdSet::SpyderX => vec![0xc3, 0x29, 0x27, 0x00, 0x01, v1],
            CmdSet::SpyderX2 => vec![0xf7, 0xa1, 0xc5, 0x00, 0x01, v1],
        }
    }

    fn setup_resp_len(self) -> usize {
        match self {
            CmdSet::SpyderX => 15,
            CmdSet::SpyderX2 => 27,
        }
    }
}

/// Decoded CIE 1931 XYZ & xyY reading from a Spyder measurement.
#[derive(Clone, Copy, Debug, Default)]
pub struct XyzReading {
    /// CIE X tristimulus value (scaled in cd/m²).
    pub x: f64,
    /// CIE Y luminance in cd/m² (nits).
    pub y: f64,
    /// CIE Z tristimulus value (scaled in cd/m²).
    pub z: f64,
    /// Raw 16-bit sensor channel counts (first 3 optical channels).
    pub raw_counts: [u16; 3],
    /// Net counts after subtracting setup_off and black offset.
    pub net_counts: [f64; 3],
}

impl XyzReading {
    /// Convert CIE XYZ to CIE 1931 xyY `(x, y, Y_nits)`.
    pub fn to_xyy(&self) -> (f64, f64, f64) {
        let sum = self.x + self.y + self.z;
        if sum <= 1e-9 {
            (0.3127, 0.3290, 0.0)
        } else {
            (self.x / sum, self.y / sum, self.y)
        }
    }
}

/// Active, initialized session with a connected SpyderX / SpyderX2 / SpyderPro colorimeter.
pub struct SpyderSession {
    handle: DeviceHandle<LibUsb>,
    pub vid_pid: (u16, u16),
    pub cmd_set: CmdSet,
    pub model_name: &'static str,
    pub firmware_version: String,
    pub serial_number: String,
    pub integration_ms: u16,
    /// 3x3 sensor-to-XYZ calibration matrix (row-major: X, Y, Z).
    pub matrix: [[f64; 3]; 3],
    pub gain: [f64; 3],
    pub offset: [f64; 3],
    /// Pre-built measurement command frame (`d2 ...` or `f2 ...`).
    measure_cmd: Vec<u8>,
    measure_resp_len: usize,
    /// Per-channel baseline offset returned by `setup(v1)`.
    pub setup_off: [f64; 3],
    /// Measured dark-current calibration offset (`bcal`, with lens cap on).
    pub black_off: [f64; 3],
    /// Whether `calibrate_black` (`bcal`) has been performed in this session.
    pub black_calibrated: bool,
}

impl SpyderSession {
    /// Open the first recognised Datacolor instrument, run `init`, `factory_cal`,
    /// and `setup(v1)`, and return a ready-to-measure session.
    pub fn open() -> Result<Self> {
        let (handle, vid_pid, model_name) = open_first_with_name()?;

        for cmd_set in [CmdSet::SpyderX2, CmdSet::SpyderX] {
            let init_cmd = cmd_set.init_cmd();
            let init_resp = match transact(&handle, &init_cmd, cmd_set.init_resp_len()) {
                Ok(r) => r,
                Err(_) => continue,
            };

            let (firmware_version, serial_number) = parse_fw_and_serial(&init_resp);

            // Read factory calibration
            let cal_cmd = cmd_set.cal_cmd();
            let cal = transact(&handle, &cal_cmd, cmd_set.cal_resp_len())
                .context("factory calibration read failed")?;

            let (v1, integration_ms, matrix, gain, offset) = parse_calibration(cmd_set, &cal)?;

            // Run sensor setup(v1)
            let setup_cmd = cmd_set.setup_cmd(v1);
            let setup = transact(&handle, &setup_cmd, cmd_set.setup_resp_len())
                .context("sensor setup(v1) failed")?;

            let (measure_cmd, measure_resp_len, setup_off) =
                build_measure_frame(cmd_set, integration_ms, &setup)?;

            tracing::info!(
                "initialised {} ({:04x}:{:04x}, {}) FW={} SN={} v1={:#04x} int={}ms",
                model_name,
                vid_pid.0,
                vid_pid.1,
                cmd_set.name(),
                firmware_version,
                serial_number,
                v1,
                integration_ms
            );

            return Ok(Self {
                handle,
                vid_pid,
                cmd_set,
                model_name,
                firmware_version,
                serial_number,
                integration_ms,
                matrix,
                gain,
                offset,
                measure_cmd,
                measure_resp_len,
                setup_off,
                black_off: [0.0; 3],
                black_calibrated: false,
            });
        }

        bail!("instrument did not respond to SpyderX2 or SpyderX init frames (try unplugging and replugging)")
    }

    /// Perform dark-current zeroing (`bcal`) with the sensor lens cap closed.
    ///
    /// Takes two consecutive dark measurements, verifies that no ambient light is
    /// leaking into the sensor (rejecting if raw counts > 50), and records the
    /// residual thermal offset above `setup_off` in `self.black_off`.
    pub fn calibrate_black(&mut self) -> Result<[f64; 3]> {
        let mut accum = [0.0f64; 3];
        const SAMPLES: usize = 2;

        for _ in 0..SAMPLES {
            reset(&self.handle)?;
            let resp = transact(&self.handle, &self.measure_cmd, self.measure_resp_len)
                .context("dark-current (bcal) bulk transfer failed")?;
            if resp.len() < 11 {
                bail!("dark-current response too short ({} bytes)", resp.len());
            }

            let mut raw = [0u16; 3];
            for i in 0..3 {
                raw[i] = u16::from_be_bytes([resp[5 + i * 2], resp[6 + i * 2]]);
            }

            // Reject if light is hitting the sensor (cap off)
            if raw[0] > 50 || raw[1] > 50 || raw[2] > 50 {
                bail!(
                    "optical light detected (raw counts {:?}) — place the Spyder in its lens cap before zeroing dark current",
                    raw
                );
            }

            for i in 0..3 {
                let net_dark = (raw[i] as f64 - self.setup_off[i]).max(0.0);
                accum[i] += net_dark;
            }
        }

        let bcal = [
            accum[0] / SAMPLES as f64,
            accum[1] / SAMPLES as f64,
            accum[2] / SAMPLES as f64,
        ];
        self.black_off = bcal;
        self.black_calibrated = true;

        tracing::info!("dark-current bcal zeroed: {:?}", bcal);
        Ok(bcal)
    }

    /// Take a single CIE XYZ measurement from the sensor.
    pub fn measure(&self) -> Result<XyzReading> {
        reset(&self.handle)?;

        let resp = transact(&self.handle, &self.measure_cmd, self.measure_resp_len)
            .context("sensor measurement bulk transfer failed")?;

        if resp.len() < 11 {
            bail!("measurement response too short ({} bytes)", resp.len());
        }

        let mut raw_counts = [0u16; 3];
        let mut net_counts = [0.0f64; 3];
        let mut calibrated_ch = [0.0f64; 3];

        for i in 0..3 {
            let raw = u16::from_be_bytes([resp[5 + i * 2], resp[6 + i * 2]]);
            raw_counts[i] = raw;

            let net = (raw as f64 - self.setup_off[i] - self.black_off[i]).max(0.0);
            net_counts[i] = net;

            let g = if self.gain[i].abs() > 1e-9 {
                self.gain[i]
            } else {
                1.0
            };
            calibrated_ch[i] = (net / g) + self.offset[i];
        }

        let x = (self.matrix[0][0] * calibrated_ch[0]
            + self.matrix[0][1] * calibrated_ch[1]
            + self.matrix[0][2] * calibrated_ch[2])
            .max(0.0);
        let y = (self.matrix[1][0] * calibrated_ch[0]
            + self.matrix[1][1] * calibrated_ch[1]
            + self.matrix[1][2] * calibrated_ch[2])
            .max(0.0);
        let z = (self.matrix[2][0] * calibrated_ch[0]
            + self.matrix[2][1] * calibrated_ch[1]
            + self.matrix[2][2] * calibrated_ch[2])
            .max(0.0);

        Ok(XyzReading {
            x,
            y,
            z,
            raw_counts,
            net_counts,
        })
    }
}

fn parse_fw_and_serial(init_resp: &[u8]) -> (String, String) {
    if init_resp.len() >= 17 {
        let raw = String::from_utf8_lossy(&init_resp[5..17]).trim().to_string();
        if raw.len() >= 12 {
            return (raw[..4].to_string(), raw[4..12].to_string());
        }
        return (raw, String::from("—"));
    }
    (String::from("unknown"), String::from("unknown"))
}

fn read_f32_le(bytes: &[u8], offset: usize) -> f64 {
    let arr = [
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ];
    f32::from_le_bytes(arr) as f64
}

fn parse_calibration(
    cmd_set: CmdSet,
    cal: &[u8],
) -> Result<(u8, u16, [[f64; 3]; 3], [f64; 3], [f64; 3])> {
    match cmd_set {
        CmdSet::SpyderX2 => {
            if cal.len() < 113 {
                bail!("SpyderX2 calibration response too short: {} B", cal.len());
            }
            let v1 = cal[6];
            let v2 = u16::from_be_bytes([cal[7], cal[8]]);
            // 9 IEEE754 LE floats starting at byte 15 (after 5B header + 1B idx + 1B v1 + 2B v2 + 6B chan map)
            let mut matrix = [[0.0f64; 3]; 3];
            for r in 0..3 {
                for c in 0..3 {
                    let idx = 15 + (r * 3 + c) * 4;
                    matrix[r][c] = read_f32_le(cal, idx);
                }
            }
            // 3 (gain, off) pairs starting at byte 87
            let mut gain = [1.0f64; 3];
            let mut offset = [0.0f64; 3];
            for i in 0..3 {
                gain[i] = read_f32_le(cal, 87 + i * 8);
                offset[i] = read_f32_le(cal, 87 + i * 8 + 4);
            }
            Ok((v1, v2, matrix, gain, offset))
        }
        CmdSet::SpyderX => {
            if cal.len() < 47 {
                bail!("SpyderX calibration response too short: {} B", cal.len());
            }
            let v1 = cal[6];
            let v2 = u16::from_be_bytes([cal[7], cal[8]]);
            let mut matrix = [[0.0f64; 3]; 3];
            for r in 0..3 {
                for c in 0..3 {
                    let idx = 9 + (r * 3 + c) * 4;
                    matrix[r][c] = read_f32_le(cal, idx);
                }
            }
            Ok((v1, v2, matrix, [1.0; 3], [0.0; 3]))
        }
    }
}

fn build_measure_frame(
    cmd_set: CmdSet,
    v2: u16,
    setup: &[u8],
) -> Result<(Vec<u8>, usize, [f64; 3])> {
    match cmd_set {
        CmdSet::SpyderX2 => {
            if setup.len() < 27 {
                bail!("SpyderX2 setup response too short: {} B", setup.len());
            }
            // Bytes 5..20 (15 bytes) are [s2 (2B), s1 (1B), s3 (6B), s4 (6B)]
            let mut cmd = vec![0xf2, 0x29, 0x27, 0x00, 0x0f];
            cmd.extend_from_slice(&setup[5..20]);
            let setup_off = [
                setup[20] as f64,
                setup[21] as f64,
                setup[22] as f64,
            ];
            Ok((cmd, 17, setup_off))
        }
        CmdSet::SpyderX => {
            if setup.len() < 15 {
                bail!("SpyderX setup response too short: {} B", setup.len());
            }
            let v2_bytes = v2.to_be_bytes();
            let mut cmd = vec![0xd2, 0x3f, 0xb9, 0x00, 0x07, v2_bytes[0], v2_bytes[1]];
            // s1 (1B) + s2 (4B) = setup[5..10]
            cmd.extend_from_slice(&setup[5..10]);
            let setup_off = [
                setup[10] as f64,
                setup[11] as f64,
                setup[12] as f64,
            ];
            Ok((cmd, 13, setup_off))
        }
    }
}

/// Find and open the first recognised Datacolor device on the bus.
fn open_first_with_name() -> Result<(DeviceHandle<LibUsb>, (u16, u16), &'static str)> {
    let ctx = LibUsb::new().context("could not initialise libusb")?;
    let devices = ctx.devices().context("could not enumerate USB bus")?;

    for device in devices.iter() {
        let desc = match device.device_descriptor() {
            Ok(d) => d,
            Err(_) => continue,
        };
        let (vid, pid) = (desc.vendor_id(), desc.product_id());
        let Some(model_name) = known_model(vid, pid) else {
            continue;
        };

        let handle = device.open().with_context(|| {
            format!(
                "could not open {vid:04x}:{pid:04x} \
                 (udev rule applied? try replugging the meter)"
            )
        })?;

        // Detach any kernel driver (e.g. usbhid) that grabbed the interface.
        handle
            .set_auto_detach_kernel_driver(true)
            .context("could not set kernel-driver auto-detach")?;

        handle
            .claim_interface(0)
            .context("could not claim interface 0")?;

        tracing::info!("opened {} ({vid:04x}:{pid:04x})", model_name);
        return Ok((handle, (vid, pid), model_name));
    }

    bail!("no recognised Datacolor colorimeter found — run `yutani detect` first")
}

/// Send a command frame and read the response.
///
/// `expected_len` is advisory: it is used only to note a mismatch, since
/// the device's actual response length is authoritative.
fn transact(
    handle: &DeviceHandle<LibUsb>,
    cmd: &[u8],
    expected_len: usize,
) -> Result<Vec<u8>> {
    let written = handle
        .write_bulk(EP_OUT, cmd, READ_TIMEOUT)
        .with_context(|| format!("bulk write failed for {cmd:02x?}"))?;
    if written != cmd.len() {
        bail!("short write: {written}/{} bytes", cmd.len());
    }

    let mut buf = vec![0u8; READ_BUF_SIZE];
    let read = handle
        .read_bulk(EP_IN, &mut buf, READ_TIMEOUT)
        .context("bulk read failed")?;
    buf.truncate(read);

    if read != expected_len {
        tracing::debug!(
            "response length {read} differs from documented {expected_len}"
        );
    }

    Ok(buf)
}

/// Print a byte slice as an offset-addressed hex dump.
fn hexdump(label: &str, data: &[u8]) {
    println!("{label} ({} bytes):", data.len());
    for (i, chunk) in data.chunks(16).enumerate() {
        let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02x}")).collect();
        let ascii: String = chunk
            .iter()
            .map(|&b| {
                if (0x20..0x7f).contains(&b) {
                    b as char
                } else {
                    '.'
                }
            })
            .collect();
        println!("  {:04x}  {:<47}  {}", i * 16, hex.join(" "), ascii);
    }
    println!();
}

/// Open the meter, run the full init -> factory_cal -> setup -> measure handshake,
/// and dump both raw frames and decoded colorimetric values.
pub fn probe() -> Result<()> {
    let (handle, (vid, pid), model_name) = open_first_with_name()?;

    for cmd_set in [CmdSet::SpyderX2, CmdSet::SpyderX] {
        println!(
            "── probing {model_name} ({vid:04x}:{pid:04x}) with {} command set ──",
            cmd_set.name()
        );

        let cmd = cmd_set.init_cmd();
        hexdump(&format!("→ EP {EP_OUT:#04x} init write"), &cmd);

        match transact(&handle, &cmd, cmd_set.init_resp_len()) {
            Ok(resp) => {
                hexdump(&format!("← EP {EP_IN:#04x} init read"), &resp);
                let (fw, sn) = parse_fw_and_serial(&resp);
                println!(
                    "✅ {} responded — locking in {} protocol (FW: {fw}, SN: {sn})",
                    cmd_set.name(),
                    cmd_set.name()
                );

                let cal_cmd = cmd_set.cal_cmd();
                hexdump(&format!("→ EP {EP_OUT:#04x} cal write"), &cal_cmd);
                let cal = transact(&handle, &cal_cmd, cmd_set.cal_resp_len())?;
                hexdump(&format!("← EP {EP_IN:#04x} factory calibration"), &cal);

                let (v1, v2, matrix, gain, offset) = parse_calibration(cmd_set, &cal)?;
                println!("Decoded Factory Calibration:");
                println!("  v1 = {v1:#04x}, v2 (integration) = {v2} ms");
                println!("  3x3 Matrix (sensor -> XYZ):");
                for (row_label, row) in ["X", "Y", "Z"].iter().zip(matrix.iter()) {
                    println!(
                        "    {row_label}: [{:+.8}, {:+.8}, {:+.8}]",
                        row[0], row[1], row[2]
                    );
                }
                println!("  Gain:   {gain:?}");
                println!("  Offset: {offset:?}\n");

                let setup_cmd = cmd_set.setup_cmd(v1);
                hexdump(&format!("→ EP {EP_OUT:#04x} setup write"), &setup_cmd);
                let setup = transact(&handle, &setup_cmd, cmd_set.setup_resp_len())?;
                hexdump(&format!("← EP {EP_IN:#04x} setup read"), &setup);

                let (meas_cmd, meas_resp_len, setup_off) =
                    build_measure_frame(cmd_set, v2, &setup)?;
                println!("  setup_off: {setup_off:?}\n");

                reset(&handle)?;
                hexdump(&format!("→ EP {EP_OUT:#04x} measure write"), &meas_cmd);
                let meas = transact(&handle, &meas_cmd, meas_resp_len)?;
                hexdump(&format!("← EP {EP_IN:#04x} measure read"), &meas);

                let mut raw = [0u16; 3];
                let mut ch = [0.0f64; 3];
                for i in 0..3 {
                    raw[i] = u16::from_be_bytes([meas[5 + i * 2], meas[6 + i * 2]]);
                    let net = (raw[i] as f64 - setup_off[i]).max(0.0);
                    ch[i] = net / gain[i] + offset[i];
                }
                let x = (matrix[0][0] * ch[0] + matrix[0][1] * ch[1] + matrix[0][2] * ch[2]).max(0.0);
                let y = (matrix[1][0] * ch[0] + matrix[1][1] * ch[1] + matrix[1][2] * ch[2]).max(0.0);
                let z = (matrix[2][0] * ch[0] + matrix[2][1] * ch[1] + matrix[2][2] * ch[2]).max(0.0);
                let reading = XyzReading {
                    x,
                    y,
                    z,
                    raw_counts: raw,
                    net_counts: ch,
                };
                let (cx, cy, lum) = reading.to_xyy();
                println!(
                    "📊 Sample Reading: raw={raw:?} -> XYZ=({x:.4}, {y:.4}, {z:.4}) | xyY=({cx:.4}, {cy:.4}, {lum:.3} cd/m²)"
                );

                return Ok(());
            }
            Err(e) => {
                println!("❌ {} command set rejected: {e:#}", cmd_set.name());
                println!("   (if this persists, unplug and replug the meter)\n");
            }
        }
    }

    println!("No command set answered. Record findings in docs/protocol-notes.md.");
    Ok(())
}

/// "Spyder reset" — vendor control transfer sent before each measurement.
pub fn reset(handle: &DeviceHandle<LibUsb>) -> Result<()> {
    handle
        .write_control(
            RESET_REQUEST_TYPE,
            RESET_REQUEST,
            RESET_VALUE,
            0,
            &[],
            READ_TIMEOUT,
        )
        .context("spyder reset control transfer failed")?;
    Ok(())
}

