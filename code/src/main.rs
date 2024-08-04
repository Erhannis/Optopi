#![no_std]
#![no_main]

mod pio_run;

use embedded_hal::digital::OutputPin;
use pio_run::uart_tx_putc;
use rp_pico::{entry, pac::clocks};
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use core::{fmt::Write, str};

use rp2040_hal::{self as bsp, gpio::{DynBankId, DynPinId}};

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

    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    let mut clocks = init_clocks_and_plls(XOSC_CRYSTAL_FREQ, peripherals.XOSC, peripherals.CLOCKS, peripherals.PLL_SYS, peripherals.PLL_USB, &mut peripherals.RESETS, &mut watchdog).ok().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // let baud = 9600;
    // let baud = 115200;
    let baud = 921600;

    let (mut pio0, sm00, _, _, _) = rp2040_hal::pio::PIOExt::split(peripherals.PIO0, &mut peripherals.RESETS);
    let mut gpio18 = pins.gpio18;
    gpio18.set_output_override(rp2040_hal::gpio::OutputOverride::Invert);
    let mut uart2_tx = pio_run::uart_tx_program_init(
      &mut pio0,
      sm00,
      gpio18.into_function(),
      baud,
      &clocks
    );

    let (mut pio1, sm10, _, _, _) = rp2040_hal::pio::PIOExt::split(peripherals.PIO1, &mut peripherals.RESETS);
    let mut gpio19 = pins.gpio19;
    // gpio19.set_input_override(rp2040_hal::gpio::InputOverride::Invert);
    let mut uart2_rx = pio_run::uart_rx_program_init(
      &mut pio1,
      sm10,
      gpio19.into_function(),
      baud,
      &clocks
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut gpio0 = pins.gpio0;
    let mut gpio1 = pins.gpio1;
    gpio0.set_output_override(rp2040_hal::gpio::OutputOverride::Invert);
    // gpio1.set_input_override(rp2040_hal::gpio::InputOverride::Invert);
    let uart_pins_0 = (
        gpio0.into_function(),
        gpio1.into_function(),
    );
    let mut uart0 = UartPeripheral::new(peripherals.UART0, uart_pins_0, &mut peripherals.RESETS)
        .enable(
            UartConfig::new(baud.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        ).unwrap();

    let mut gpio4 = pins.gpio4;
    let gpio4_id = gpio4.id();
    let mut gpio5 = pins.gpio5;
    gpio4.set_output_override(rp2040_hal::gpio::OutputOverride::Invert);
    gpio5.set_input_override(rp2040_hal::gpio::InputOverride::Invert);
    let uart_pins_1 = (
        gpio4.into_function(),
        gpio5.into_function(),
    );
    let mut uart1 = UartPeripheral::new(peripherals.UART1, uart_pins_1, &mut peripherals.RESETS)
        .enable(
            UartConfig::new(baud.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        ).unwrap();
        
    let mut i = 0;
    loop {
        info!("Loop");

        toggle_uart_tx_pin(&gpio4_id, &mut delay);
        
        core::write!(uart0, "Hello World 0 {}!\r\n", i).unwrap();
        core::write!(uart1, "Hello World 1 {}!\r\n", i).unwrap();
        uart_tx_putc(&mut uart2_tx, b'x');
        // uart0.write_full_blocking(b"Hello World 0!\r\n");

        delay.delay_ms(100);

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
        loop {
          //DUMMY Inefficient
          match uart2_rx.read() {
            Some(c) => {info!("Read 2: {:?}", (c >> 24) as u8 as char)},
            None => {break;}
          }
        }
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

fn toggle_uart_tx_pin(pin: &DynPinId, delay: &mut cortex_m::delay::Delay) {
  let num = pin.num; // gpio4.id(), back when you had access to it;
  let mask = 1 << pin.num;
  let bank = pin.bank;

  // become push_pull
  unsafe {
    let sio = &*pac::SIO::PTR;
    let gpio_oe_set = sio.gpio_oe_set();
    match bank {
        DynBankId::Bank0 => &gpio_oe_set,
        DynBankId::Qspi => core::mem::transmute(&sio.gpio_hi_oe_set()),
    }.write(|w| { w.gpio_oe_set().bits(mask) });
  }
  // next is: pin.pad_ctrl().modify(|_, w| w.ie().set_bit());
  let gpio = unsafe { &*pac::PADS_BANK0::PTR };
  gpio.gpio(usize::from(num)).modify(|_, w| w.ie().set_bit());
  // next is: pin.io_ctrl().modify(|_, w| w.funcsel().variant(funcsel));
  let gpio = unsafe { &*pac::IO_BANK0::PTR };
  let io_ctrl = &gpio.gpio(usize::from(num)).gpio_ctrl();
  io_ctrl.modify(|_, w| w.funcsel().variant(pac::io_bank0::gpio::gpio_ctrl::FUNCSEL_A::SIO));

  // set_low
  unsafe {
    let sio = &*pac::SIO::PTR;
    let gpio_out_clr = sio.gpio_out_clr();
    match bank {
        DynBankId::Bank0 => &gpio_out_clr,
        DynBankId::Qspi => core::mem::transmute(&sio.gpio_hi_out_clr()),
    }.write(|w| { w.gpio_out_clr().bits(mask) });
  }

  delay.delay_us(3); // 2 or 3 seem to work - 2 shows no extra spike, 3 shows one ~100ns wide

  // set_high
  unsafe {
    let sio = &*pac::SIO::PTR;
    let gpio_out_set = sio.gpio_out_set();
    match bank {
        DynBankId::Bank0 => &gpio_out_set,
        DynBankId::Qspi => core::mem::transmute(&sio.gpio_hi_out_set()),
    }.write(|w| { w.gpio_out_set().bits(mask) });
  }

  // become uart
  // next is: pin.pad_ctrl().modify(|_, w| w.ie().set_bit());
  let gpio = unsafe { &*pac::PADS_BANK0::PTR };
  gpio.gpio(usize::from(num)).modify(|_, w| w.ie().set_bit());
  // next is: pin.io_ctrl().modify(|_, w| w.funcsel().variant(funcsel));
  let gpio = unsafe { &*pac::IO_BANK0::PTR };
  let io_ctrl = &gpio.gpio(usize::from(num)).gpio_ctrl();
  io_ctrl.modify(|_, w| w.funcsel().variant(pac::io_bank0::gpio::gpio_ctrl::FUNCSEL_A::UART));
}