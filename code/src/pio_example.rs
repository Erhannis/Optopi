use rp2040_hal::{gpio::{FunctionPio0, Pin, PinId, PullType}, pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO}};

pub fn blink_program_init<P: PIOExt, SM: StateMachineIndex, T: PinId, U: PullType>(
  pio: &mut PIO<P>,
  sm: UninitStateMachine<(P, SM)>,
  led: Pin<T, FunctionPio0, U>
) {
    let led_pin_id = led.id().num;

    let program;
    match 1 {
      0 => {
        program = pio_proc::pio_file!("./src/blink.pio").program;
      },
      1 => {
        program = pio_proc::pio_file!("./src/blink2.pio").program;
      },
      2 => {
        const MAX_DELAY: u8 = 31;
        let mut a = pio::Assembler::<32>::new();
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        a.set(pio::SetDestination::PINDIRS, 1);
        a.bind(&mut wrap_target);
        a.set_with_delay(pio::SetDestination::PINS, 0, MAX_DELAY);
        a.set_with_delay(pio::SetDestination::PINS, 1, MAX_DELAY);
        a.bind(&mut wrap_source);
        program = a.assemble_with_wrap(wrap_source, wrap_target);
      },
      3 => {
        let mut a = pio::Assembler::<32>::new();
        let mut wrap_target = a.label();
        let mut lp1 = a.label();
        let mut lp2 = a.label();
        let mut wrap_source = a.label();
        a.set(pio::SetDestination::PINDIRS, 1);
        a.pull(false, true);
        a.out(pio::OutDestination::Y, 32);
        a.bind(&mut wrap_target);
        a.mov(pio::MovDestination::X, pio::MovOperation::None, pio::MovSource::Y);
        a.set(pio::SetDestination::PINS, 1);
        a.bind(&mut lp1);
        a.jmp(pio::JmpCondition::XDecNonZero, &mut lp1);
        a.mov(pio::MovDestination::X, pio::MovOperation::None, pio::MovSource::Y);
        a.set(pio::SetDestination::PINS, 0);
        a.bind(&mut lp2);
        a.jmp(pio::JmpCondition::XDecNonZero, &mut lp2);
        a.bind(&mut wrap_source);
        program = a.assemble_with_wrap(wrap_source, wrap_target);    
      },
      _ => {
        core::panic!("Unhandled case");
      }
    }

    let installed = pio.install(&program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (mut smx, _rx, mut tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm);
    smx.set_pindirs([(led_pin_id, rp2040_hal::pio::PinDir::Output)]);
    smx.start();
    tx.write(0x000000FF);
}
