//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod pio_example;

use rp_pico::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;
use core::{fmt::Write, str};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp2040_hal::{self as bsp};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use rp2040_hal::{gpio::Pins, uart::{DataBits, StopBits, UartConfig, UartPeripheral}};
use rp2040_hal::fugit::RateExtU32;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut peripherals = pac::Peripherals::take().unwrap();
    let sio = Sio::new(peripherals.SIO);
    let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);

    let (mut pio, sm0, _, _, _) = rp2040_hal::pio::PIOExt::split(peripherals.PIO0, &mut peripherals.RESETS);

    pins.gpio15.into_push_pull_output().set_low().unwrap();
    pins.gpio16.into_push_pull_output().set_high().unwrap();
    pins.gpio17.into_push_pull_output().set_low().unwrap();

    pio_example::blink_program_init(
      &mut pio,
      sm0,
      pins.gpio14.into_function(),
    );

    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    let mut clocks = init_clocks_and_plls(XOSC_CRYSTAL_FREQ, peripherals.XOSC, peripherals.CLOCKS, peripherals.PLL_SYS, peripherals.PLL_USB, &mut peripherals.RESETS, &mut watchdog).ok().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let gpio0 = pins.gpio0;
    let mut gpio1 = pins.gpio1;
    gpio1.set_input_override(rp2040_hal::gpio::InputOverride::Invert);
    let uart_pins_0 = (
        gpio0.into_function(),
        gpio1.into_function(),
    );
    let mut uart0 = UartPeripheral::new(peripherals.UART0, uart_pins_0, &mut peripherals.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        ).unwrap();

    let gpio4 = pins.gpio4;
    let mut gpio5 = pins.gpio5;
    gpio5.set_input_override(rp2040_hal::gpio::InputOverride::Invert);
    let uart_pins_1 = (
        gpio4.into_function(),
        gpio5.into_function(),
    );
    let mut uart1 = UartPeripheral::new(peripherals.UART1, uart_pins_1, &mut peripherals.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        ).unwrap();
        
    let mut i = 0;
    loop {
        info!("Loop");
        core::write!(uart0, "Hello World 0 {}!\r\n", i).unwrap();
        core::write!(uart1, "Hello World 1 {}!\r\n", i).unwrap();
        // uart0.write_full_blocking(b"Hello World 0!\r\n");
        let mut buf: [u8;256] = [0;256];
        while uart0.uart_is_readable() {
            match uart0.read_raw(&mut buf) {
                Ok(count) => {
                    let b = &mut buf[0..count];
                    convert_to_lossy_utf8(b);
                    info!("Read 0: {:?}", str::from_utf8(b).unwrap());
                },
                Err(e) => {
                    info!("Read 0 error");
                },
            }
        }
        while uart1.uart_is_readable() {
            match uart1.read_raw(&mut buf) {
                Ok(count) => {
                    let b = &mut buf[0..count];
                    convert_to_lossy_utf8(b);
                    info!("Read 1: {:?}", str::from_utf8(b).unwrap());
                },
                Err(e) => {
                    info!("Read 1 error");
                },
            }
        }
        delay.delay_ms(100);
        i += 1;
    }
}

fn convert_to_lossy_utf8(input: &mut [u8]) {
    for byte in input {
        if !byte.is_ascii() {
            *byte = b'?';
        }
    }
}








// use rp_pico::hal::pio::{PIOExt, SM0};
// use rp_pico::hal::clocks::ClocksManager;
// use rp_pico::hal::gpio::{FunctionPio0, PullUp};
// use rp_pico::pac::PIO0;
// use rp_pico::hal::pio::{PIOBuilder, StateMachine, PIO};

// use pio_proc::pio_file;

// const UART_RX_PIN: u8 = 0;
// const SERIAL_BAUD: u32 = 9600;

// fn blink_program_init<P: PIOExt, SM: StateMachineIndex, T: PinId, U: PullType>(
//   pio: &mut PIO<P>,
//   sm: UninitStateMachine<(P, SM)>,
//   led: Pin<T, FunctionPio0, U>
// ) {
//     let led_pin_id = led.id().num;

//     let program;
//     match 1 {
//       0 => {
//         program = pio_proc::pio_file!("./src/blink.pio").program;
//       },
//       1 => {
//         program = pio_proc::pio_file!("./src/blink2.pio").program;
//       },
//       2 => {
//         const MAX_DELAY: u8 = 31;
//         let mut a = pio::Assembler::<32>::new();
//         let mut wrap_target = a.label();
//         let mut wrap_source = a.label();
//         a.set(pio::SetDestination::PINDIRS, 1);
//         a.bind(&mut wrap_target);
//         a.set_with_delay(pio::SetDestination::PINS, 0, MAX_DELAY);
//         a.set_with_delay(pio::SetDestination::PINS, 1, MAX_DELAY);
//         a.bind(&mut wrap_source);
//         program = a.assemble_with_wrap(wrap_source, wrap_target);
//       },
//       3 => {
//         let mut a = pio::Assembler::<32>::new();
//         let mut wrap_target = a.label();
//         let mut lp1 = a.label();
//         let mut lp2 = a.label();
//         let mut wrap_source = a.label();
//         a.set(pio::SetDestination::PINDIRS, 1);
//         a.pull(false, true);
//         a.out(pio::OutDestination::Y, 32);
//         a.bind(&mut wrap_target);
//         a.mov(pio::MovDestination::X, pio::MovOperation::None, pio::MovSource::Y);
//         a.set(pio::SetDestination::PINS, 1);
//         a.bind(&mut lp1);
//         a.jmp(pio::JmpCondition::XDecNonZero, &mut lp1);
//         a.mov(pio::MovDestination::X, pio::MovOperation::None, pio::MovSource::Y);
//         a.set(pio::SetDestination::PINS, 0);
//         a.bind(&mut lp2);
//         a.jmp(pio::JmpCondition::XDecNonZero, &mut lp2);
//         a.bind(&mut wrap_source);
//         program = a.assemble_with_wrap(wrap_source, wrap_target);    
//       },
//       _ => {
//         core::panic!("Unhandled case");
//       }
//     }

//     let installed = pio.install(&program).unwrap();
//     let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
//     let (mut smx, _rx, mut tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
//         .set_pins(led_pin_id, 1)
//         .clock_divisor_fixed_point(int, frac)
//         .build(sm);
//     smx.set_pindirs([(led_pin_id, rp2040_hal::pio::PinDir::Output)]);
//     smx.start();
//     tx.write(0x000000FF);
// }
