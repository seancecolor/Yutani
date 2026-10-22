// Copyright 2026 <you> — Apache License, Version 2.0 (same terms as mod.rs)

//! SpyderX-family protocol, independently reimplemented in Rust from
//! publicly documented device behaviour (see docs/protocol-notes.md).

use std::time::Duration;

use anyhow::{bail, Context, Result};
use rusb::{Context as LibUsb, DeviceHandle, UsbContext};

use super::{known_model, DATACOLOR_VID};

/// Bulk endpoints, per device descriptors.
const EP_OUT: u8 = 0x01;
const EP_IN: u8 = 0x81;

/// Control transfer sent before measurements ("spyder reset").
/// Not needed for init, but implemented here for completeness.
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
#[derive(Clone, Copy)]
pub enum CmdSet {
    SpyderX,
    SpyderX2,
}

impl CmdSet {
    fn name(self) -> &'static str {
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
            CmdSet::SpyderX | CmdSet::SpyderX2 => 28,
        }
    }
}

/// Find and open the first recognised Datacolor device on the bus.
fn open_first() -> Result<DeviceHandle<LibUsb>> {
    let ctx = LibUsb::new().context("could not initialise libusb")?;
    let devices = ctx.devices().context("could not enumerate USB bus")?;

    for device in devices.iter() {
        let desc = match device.device_descriptor() {
            Ok(d) => d,
            Err(_) => continue,
        };
        let (vid, pid) = (desc.vendor_id(), desc.product_id());
        if vid != DATACOLOR_VID || known_model(vid, pid).is_none() {
            continue;
        }

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

        tracing::info!(
            "opened {} ({vid:04x}:{pid:04x})",
            known_model(vid, pid).unwrap_or("Datacolor device")
        );
        return Ok(handle);
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

/// Open the meter, try each known init command, and dump what comes back.
///
/// Stops at the first command set the device answers: these meters are
/// stateful, and sending a second, unmatched frame can wedge them until
/// they are unplugged and replugged.
pub fn probe() -> Result<()> {
    let handle = open_first()?;

    for cmd_set in [CmdSet::SpyderX2, CmdSet::SpyderX] {
        println!("── probing with {} command set ──", cmd_set.name());

        let cmd = cmd_set.init_cmd();
        hexdump(&format!("→ EP {EP_OUT:#04x} write"), &cmd);

        match transact(&handle, &cmd, cmd_set.init_resp_len()) {
            Ok(resp) => {
    hexdump(&format!("← EP {EP_IN:#04x} init read"), &resp);
    println!(
        "✅ {} responded — locking in {} protocol",
        cmd_set.name(),
        cmd_set.name()
    );

    // Follow the init with the factory-calibration read.
    // Same command-set dispatch; X2 frame documented as
    // f6 5c 37 00 01 00, response ~113 bytes (108+5).
    let cal_cmd = match cmd_set {
        CmdSet::SpyderX2 => vec![0xf6, 0x5c, 0x37, 0x00, 0x01, 0x00],
        CmdSet::SpyderX => vec![0xcb, 0x05, 0x73, 0x00, 0x01, 0x00],
    };
    hexdump(&format!("→ EP {EP_OUT:#04x} cal write"), &cal_cmd);
    match transact(&handle, &cal_cmd, 113) {
        Ok(cal) => {
            hexdump(&format!("← EP {EP_IN:#04x} factory calibration"), &cal);
            println!("Captured {} bytes of calibration data.", cal.len());
        }
        Err(e) => println!("⚠️ calibration read failed: {e:#}"),
    }

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
/// Kept here for Milestone 2; harmless to verify now.
#[allow(dead_code)]
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
