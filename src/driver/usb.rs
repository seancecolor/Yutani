// Copyright 2026 <you> — Apache License, Version 2.0 (same terms as mod.rs)

//! Discovery of vendor-class USB instruments (non-HID).
//!
//! The SpyderX/SpyderX2/Spyder(2024) family exposes a single
//! vendor-specific interface (class 0xff) with two bulk endpoints
//! (0x01 OUT, 0x81 IN) rather than HID reports, so it must be
//! enumerated and driven via libusb.

use anyhow::{Context as _, Result};
use rusb::{Context as LibUsb, UsbContext};

use super::DATACOLOR_VID;

/// Scan the USB bus for Datacolor vendor-class instruments.
///
/// Returns how many were found. Each is printed with its interface
/// and endpoint layout — useful reference material while mapping the
/// measurement protocol (log it into docs/protocol-notes.md).
pub fn detect_usb() -> Result<usize> {
let ctx = LibUsb::new().context("could not initialise libusb")?;

    let mut found = 0usize;

    for device in ctx.devices()?.iter() {
        let desc = match device.device_descriptor() {
            Ok(d) => d,
            Err(_) => continue,
        };
        if desc.vendor_id() != DATACOLOR_VID {
            continue;
        }

        found += 1;

        let model = super::known_model(desc.vendor_id(), desc.product_id())
            .unwrap_or("unknown Datacolor instrument");

        println!("  {model}");
        println!(
            "    VID:PID       {:04x}:{:04x}",
            desc.vendor_id(),
            desc.product_id()
        );
        println!(
            "    USB bus/dev   {:03}/{:03}",
            device.bus_number(),
            device.address()
        );

        // Dump interface layout. Reading descriptors does not require
        // claiming the interface, so this is safe without opening it.
        let config = device
            .active_config_descriptor()
            .or_else(|_| device.config_descriptor(0));
        if let Ok(cfg) = config {
            for iface in cfg.interfaces() {
                for i in iface.descriptors() {
                    println!(
                        "    Interface {} class {:#04x}, {} endpoints",
                        iface.number(),
                        i.class_code(),
                        i.num_endpoints()
                    );
                    for ep in i.endpoint_descriptors() {
                        println!("      EP {:#04x} ({:?})", ep.address(), ep.direction());
                    }
                }
            }
        } else {
            println!("    (descriptors unavailable — check udev permissions)");
        }
        println!();
    }

    Ok(found)
}
