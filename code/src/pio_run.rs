use rp2040_hal::{clocks::ClocksManager, gpio::{FunctionPio0, Pin, PinId, PinState, PullType}, pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO}, Clock};

pub fn blink_program_init<P: PIOExt, SM: StateMachineIndex, T: PinId, U: PullType>(
  pio: &mut PIO<P>,
  sm: UninitStateMachine<(P, SM)>,
  led: Pin<T, FunctionPio0, U>
) {
    let led_pin_id = led.id().num;

    let program = pio_proc::pio_file!("./src/blink.pio").program;

    let installed = pio.install(&program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (mut smx, mut _rx, mut _tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm);
    smx.set_pindirs([(led_pin_id, rp2040_hal::pio::PinDir::Output)]);
    smx.start();
}

pub fn uart_tx_program_init<P: PIOExt, SM: StateMachineIndex, T: PinId, U: PullType>(
  pio: &mut PIO<P>,
  sm: UninitStateMachine<(P, SM)>,
  tx_pin: Pin<T, FunctionPio0, U>,
  baud: u32,
  clocks: &ClocksManager,
) -> rp2040_hal::pio::Tx<(P, SM)> {
    let tx_pin_id = tx_pin.id().num;

    let program = pio_proc::pio_file!("./src/uart_tx.pio").program;

    // SM transmits 1 bit per 8 execution cycles.
    let divisor: f32 = (clocks.system_clock.freq().to_Hz() as f32) / ((8 * baud) as f32);
    
    let installed = pio.install(&program).unwrap();
    let (mut smx, mut _rx, mut _tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(rp2040_hal::pio::Buffers::OnlyTx) // We only need TX, so get an 8-deep FIFO!
        .out_pins(tx_pin_id, 1) // We are mapping both OUT and side-set to the same pin, because sometimes we need to assert user data onto the pin (with OUT) and sometimes assert constant values (start/stop bit)
        .side_set_pin_base(tx_pin_id)
        .clock_divisor(divisor)
        .out_shift_direction(rp2040_hal::pio::ShiftDirection::Right)
        .autopull(false)
        .pull_threshold(32)
        .build(sm);
    smx.set_pins([(tx_pin_id, rp2040_hal::pio::PinState::High)]);
    smx.set_pindirs([(tx_pin_id, rp2040_hal::pio::PinDir::Output)]);
    smx.start();

    return _tx;
}

//RAINY Can we wrap this and return it?
#[inline(always)]
pub fn uart_tx_putc<P: PIOExt, SM: StateMachineIndex>(tx: &mut rp2040_hal::pio::Tx<(P, SM)>, c: u8) {
    // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
    loop {
      if tx.write(c as u32) {
        break;
      }
    }
}

pub fn uart_rx_program_init<P: PIOExt, SM: StateMachineIndex, T: PinId, U: PullType>(
  pio: &mut PIO<P>,
  sm: UninitStateMachine<(P, SM)>,
  rx_pin: Pin<T, FunctionPio0, U>,
  baud: u32,
  clocks: &ClocksManager,
) -> rp2040_hal::pio::Rx<(P, SM)> {
    // rx_pin.into_pull_up_input(); //DUMMY
    let rx_pin_id = rx_pin.id().num;

    let program = pio_proc::pio_file!("./src/uart_rx.pio").program;

    // SM transmits 1 bit per 8 execution cycles.
    let divisor: f32 = (clocks.system_clock.freq().to_Hz() as f32) / ((8 * baud) as f32);
    
    let installed = pio.install(&program).unwrap();
    let (mut smx, mut _rx, mut _tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
        .buffers(rp2040_hal::pio::Buffers::OnlyRx) // Deeper FIFO as we're not doing any TX
        .in_pin_base(rx_pin_id) // for WAIT, IN
        .jmp_pin(rx_pin_id) // for JMP
        .clock_divisor(divisor)
        .out_shift_direction(rp2040_hal::pio::ShiftDirection::Right)
        .autopush(false)
        .push_threshold(32)
        .build(sm);
    smx.set_pindirs([(rx_pin_id, rp2040_hal::pio::PinDir::Input)]);
    smx.start();
    
    return _rx;
}

//RAINY Can we wrap this and return it?
#[inline(always)]
pub fn uart_rx_getc<P: PIOExt, SM: StateMachineIndex>(rx: &mut rp2040_hal::pio::Rx<(P, SM)>) -> u8 {
    // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
    return loop {
      let r = rx.read();
      match r {
        Some(c) => break (c >> 24) as u8,
        None => {}
      }
    };
}