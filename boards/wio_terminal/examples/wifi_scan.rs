#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use embedded_graphics as eg;
use panic_halt as _;
use wio_terminal as wio;

use wio::hal::clock::GenericClockController;
use wio::hal::delay::Delay;
use wio::pac::{CorePeripherals, Peripherals};
use wio::prelude::*;
use wio::wifi_prelude::*;
use wio::wifi_rpcs as rpc;
use wio::{entry, wifi_singleton};

use core::fmt::Write;
use cortex_m::interrupt::free as disable_interrupts;
use eg::mono_font::{ascii::FONT_6X12, MonoTextStyle};
use eg::pixelcolor::Rgb565;
use eg::prelude::*;
use eg::primitives::{PrimitiveStyleBuilder, Rectangle};
use eg::text::{Baseline, Text};

use heapless::String;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();

    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.gclk,
        &mut peripherals.mclk,
        &mut peripherals.osc32kctrl,
        &mut peripherals.oscctrl,
        &mut peripherals.nvmctrl,
    );
    let mut delay = Delay::new(core.SYST, &mut clocks);
    let sets = wio::Pins::new(peripherals.port).split();

    // Set up the display so we can print out APs.
    let (mut display, _backlight) = sets
        .display
        .init(
            &mut clocks,
            peripherals.sercom7,
            &mut peripherals.mclk,
            58.MHz(),
            &mut delay,
        )
        .unwrap();
    clear(&mut display);
    let mut textbuffer = String::<256>::new();

    let mut user_led = sets.user_led.into_push_pull_output();
    user_led.set_high().unwrap();

    // Initialize the wifi peripheral.
    let nvic = &mut core.NVIC;
    disable_interrupts(|cs| unsafe {
        wifi_init(
            cs,
            sets.wifi,
            peripherals.sercom0,
            &mut clocks,
            &mut peripherals.mclk,
            &mut delay,
        );

        if let Some(wifi) = WIFI.as_mut() {
            wifi.enable(cs, nvic);
        }
    });

    let version = unsafe {
        WIFI.as_mut()
            .map(|wifi| wifi.blocking_rpc(rpc::GetVersion {}).unwrap())
            .unwrap()
    };
    writeln!(textbuffer, "fw: {}", version).unwrap();
    write(
        &mut display,
        textbuffer.as_str(),
        Point::new(320 - (3 + version.len() * 12) as i32, 3),
    );
    textbuffer.truncate(0);

    let mac = unsafe {
        WIFI.as_mut()
            .map(|wifi| wifi.blocking_rpc(rpc::GetMacAddress {}).unwrap())
            .unwrap()
    };
    writeln!(textbuffer, "mac: {}", mac).unwrap();
    write(&mut display, textbuffer.as_str(), Point::new(3, 3));
    textbuffer.truncate(0);

    loop {
        user_led.set_low().ok();
        // Start scanning
        unsafe {
            WIFI.as_mut()
                .map(|wifi| wifi.blocking_rpc(rpc::ScanStart {}).unwrap())
                .unwrap()
        };
        // Block until the scan is complete
        loop {
            let scanning = unsafe {
                WIFI.as_mut()
                    .map(|wifi| wifi.blocking_rpc(rpc::IsScanning {}).unwrap())
                    .unwrap()
            };
            if !scanning {
                break;
            }
        }

        let num = unsafe {
            WIFI.as_mut()
                .map(|wifi| wifi.blocking_rpc(rpc::ScanGetNumAPs {}).unwrap())
                .unwrap()
        };
        let aps = unsafe {
            WIFI.as_mut()
                .map(|wifi| {
                    wifi.blocking_rpc(rpc::ScanGetAP::<generic_array::typenum::consts::U16>::new())
                })
                .unwrap()
        };
        user_led.set_high().ok();

        // Write the information to the screen.
        writeln!(textbuffer, "{:?} APs", num).unwrap();
        write_with_clear(&mut display, textbuffer.as_str(), 3, Point::new(170, 3));
        textbuffer.truncate(0);

        for (i, ap) in aps.unwrap().0.iter().enumerate() {
            if i >= num as usize {
                break;
            }
            writeln!(textbuffer, "{:?}", ap.ssid).unwrap();
            write_with_clear(
                &mut display,
                textbuffer.as_str(),
                150 / 6,
                Point::new(3, 30 + i as i32 * 12),
            );
            textbuffer.truncate(0);

            writeln!(textbuffer, "{:?}", ap.bssid).unwrap();
            write_with_clear(
                &mut display,
                textbuffer.as_str(),
                18,
                Point::new(150, 30 + i as i32 * 12),
            );
            textbuffer.truncate(0);

            writeln!(textbuffer, "{:?}", ap.rssi).unwrap();
            write_with_clear(
                &mut display,
                textbuffer.as_str(),
                4,
                Point::new(290, 30 + i as i32 * 12),
            );
            textbuffer.truncate(0);

            if ap.band as u8 == 1 {
                write_with_clear(&mut display, "5G", 3, Point::new(132, 30 + i as i32 * 12));
                textbuffer.truncate(0);
            }
        }
    }
}

wifi_singleton!(WIFI);

fn clear(display: &mut wio::LCD) {
    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();
    let backdrop =
        Rectangle::with_corners(Point::new(0, 0), Point::new(320, 320)).into_styled(style);
    backdrop.draw(display).ok().unwrap();
}

fn write<'a, T: Into<&'a str>>(display: &mut wio::LCD, text: T, pos: Point) {
    Text::with_baseline(
        text.into(),
        pos,
        MonoTextStyle::new(&FONT_6X12, Rgb565::WHITE),
        Baseline::Top,
    )
    .draw(display)
    .ok()
    .unwrap();
}

fn write_with_clear<'a, T: Into<&'a str>>(
    display: &mut wio::LCD,
    text: T,
    num_clear: i32,
    pos: Point,
) {
    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();
    Rectangle::with_corners(pos, Point::new(pos.x + (6 * num_clear), pos.y + 12))
        .into_styled(style)
        .draw(display)
        .ok()
        .unwrap();

    Text::with_baseline(
        text.into(),
        pos,
        MonoTextStyle::new(&FONT_6X12, Rgb565::WHITE),
        Baseline::Top,
    )
    .draw(display)
    .ok()
    .unwrap();
}
