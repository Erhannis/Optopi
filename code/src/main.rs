//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use rp_pico::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;
use core::{fmt::Write, str};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp2040_hal as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use rp2040_hal::{gpio::{Pins, FunctionUart}, uart::{self, DataBits, StopBits, UartConfig, UartPeripheral}};
use rp2040_hal::fugit::RateExtU32;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut peripherals = pac::Peripherals::take().unwrap();
    let sio = Sio::new(peripherals.SIO);
    let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
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








use rp_pico::hal::pio::{PIOExt, SM0, SMConfig};
use rp_pico::hal::clocks::ClocksManager;
use rp_pico::hal::gpio::{FunctionPio0, PullUp};
use rp_pico::pac::PIO0;
use rp_pico::hal::pio::{PIOBuilder, StateMachine, PIO};

use pio_proc::pio_file;
pio_file!("./src/uart_rx.pio");

const UART_RX_PIN: u8 = 0;
const SERIAL_BAUD: u32 = 9600;

fn uart_rx_program_init(
    pio: &mut PIO<PIO0>,
    sm: &mut StateMachine<PIO0, SM0>,
    offset: u8,
    pin: u8,
    baud: u32,
    clocks: &ClocksManager,
) {
    // Configure pin
    let _pin = pio.gpio_init(pin).into_mode::<PullUp>();

    // Create default config
    let mut config = SMConfig::default();
    
    // Set up the pin
    config.set_in_pins(pin);
    config.set_jmp_pin(pin);
    config.set_in_shift(false, true, 32); // shift right, autopush disabled
    config.set_fifo_join_rx(); // Deeper FIFO

    // Set clock divider
    let clk_hz = clocks.system_clock.freq().to_Hz();
    let div = (clk_hz as f32) / (8.0 * baud as f32);
    config.clock_divider(div);

    // Apply the config and start the state machine
    sm.set_config(&config);
    sm.set_enabled(true);
}
