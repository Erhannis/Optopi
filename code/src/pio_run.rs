use rp2040_hal::{gpio::{FunctionPio0, Pin, PinId, PullType}, pio::{PIOExt, StateMachineIndex, UninitStateMachine, PIO}};

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
