// Copyright 2026 <you>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Instrument discovery for Yutani.

use anyhow::{Context, Result};
use hidapi::{DeviceInfo, HidApi};

pub mod usb;
pub mod spyder;

/// Vendor ID shared by Datacolor / ColorVision instruments.
pub const DATACOLOR_VID: u16 = 0x085c;

/// Product IDs Yutani recognises, paired with a human-readable model name.
///
/// Observed in the wild:
///   - 0x0a00 — SpyderX (original)
///   - 0x0a0a — SpyderX2 (reports as "Datacolor SpyderX" in lsusb, FW 5.50)
///   - 0x0a0b — Spyder / SpyderPro (2024, same silicon as X2, FW 6.00)
///
/// Both expose a vendor-specific bulk interface (class 0xff), so they are
/// enumerated via libusb (`driver::usb`), not hidapi.
pub const KNOWN_PRODUCTS: &[(u16, &str)] = &[
    (0x0a00, "Datacolor SpyderX"),
    (0x0a0a, "Datacolor SpyderX2"),
    (0x0a0b, "Datacolor SpyderPro (2024)"),
];

/// Look up a friendly name for a VID/PID pair, if we know it.
pub fn known_model(vid: u16, pid: u16) -> Option<&'static str> {
    if vid != DATACOLOR_VID {
        return None;
    }
    KNOWN_PRODUCTS
        .iter()
        .find(|(p, _)| *p == pid)
        .map(|(_, name)| *name)
}

/// A flattened view of one HID device, for uniform printing.
struct Summary<'a> {
    vid: u16,
    pid: u16,
    manufacturer: Option<&'a str>,
    product: Option<&'a str>,
    serial: Option<&'a str>,
    path: String,
    model: Option<&'static str>,
}

impl<'a> Summary<'a> {
    fn from_info(info: &'a DeviceInfo) -> Self {
        Self {
            vid: info.vendor_id(),
            pid: info.product_id(),
            manufacturer: info.manufacturer_string(),
            product: info.product_string(),
            serial: info.serial_number(),
            path: info.path().to_string_lossy().into_owned(),
            model: known_model(info.vendor_id(), info.product_id()),
        }
    }

    fn is_known(&self) -> bool {
        self.model.is_some()
    }

    fn print(&self) {
        println!("  {}", self.model.unwrap_or("unrecognised HID device"));
        println!("    VID:PID       {:04x}:{:04x}", self.vid, self.pid);
        println!("    Manufacturer  {}", self.manufacturer.unwrap_or("—"));
        println!("    Product       {}", self.product.unwrap_or("—"));
        println!("    Serial        {}", self.serial.unwrap_or("—"));
        println!("    hidraw path   {}", self.path);
        println!();
    }
}

/// Enumerate instruments across both HID and vendor-class USB transports.
pub fn detect(list_all: bool) -> Result<()> {
    println!("Yutani {} — instrument detection", env!("CARGO_PKG_VERSION"));
    println!();

    let mut found = 0usize;

    // Sweep 1: HID-based instruments (none of ours today, but the
    // framework is here for future supported meters).
    let api = HidApi::new().context("could not initialise hidapi")?;
    for info in api.device_list() {
        let summary = Summary::from_info(&info);
        if summary.is_known() {
            found += 1;
        }
        if summary.is_known() || list_all {
            summary.print();
        }
    }

    // Sweep 2: vendor-class bulk devices — SpyderX2, Spyder (2024).
    // These never appear as HID devices, so hidapi can't see them.
    found += usb::detect_usb()?;

    if found == 0 {
        println!("No recognised colorimeter found.");
        println!();
        println!("Checklist:");
        println!("  • Meter plugged directly into the machine, not through a hub?");
        println!("  • Does `lsusb` list a device with vendor ID 085c?");
        println!("  • Udev rule installed, then meter replugged? (see contrib/udev/)");
        println!("  • Run `yutani detect --all` to dump every HID device.");
    } else {
        println!("Found {found} recognised instrument(s).");
    }

    Ok(())
}
