use rp2040_hal::gpio::{DynBankId, DynPinId};
use rp_pico::pac;

pub fn toggle_uart_tx_pin(pin: &DynPinId, delay: &mut cortex_m::delay::Delay) {
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