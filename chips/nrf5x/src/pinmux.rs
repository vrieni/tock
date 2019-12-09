//! An abstraction over the pin multiplexer, nRF5X-family
//!
//! Controller drivers should use the `Pinmux` type (instead of a
//! `nrf5x::gpio::Pin`) for GPIO pins that are used by hardware peripherals. The
//! board configuration should create `Pinmux`s and pass them into controller
//! drivers during initialization. Using this type ensures that the same pin is
//! not assigned to multiple hardware peripherals.
//!
//! The `Pinmux` type works by maintaining a static bitfield with a bit per GPIO
//! pin. The bit corresponding to the GPIO pin is set to one if that GPIO pin
//! has been converted to a `Pinmux` and is therefore in use by a peripheral.
//! Any subsequent attempts to create a `Pinmux` for that pin will cause a
//! panic. Since the pin mapping should happen when the chip boots, this panic
//! should be cause immediately and should not affect the board in steady-state
//! execution.

use kernel::common::cells::VolatileCell;

use crate::gpio::Pin;

/// The number of GPIO ports the nRF5x chips can support.
// Note: only the nrf52840 has two ports, but we create two ports to avoid
// gating this code by a feature.
const NUM_PORTS: usize = 2;

/// The number of pins in a port. This is set directly from how Nordic
/// configures there hardware. Luckily for us on all Nordic chips there are (up
/// to) 32 pins / port.
const PIN_PER_PORT: usize = 32;

/// Keeps track of which pins has a `Pinmux` been created for. The corresponding
/// bit is set to 1 if that pin has been assigned to a `Pinmux` and therefore
/// allocated to a hardware peripheral.
static mut USED_PINS: [VolatileCell<u32>; NUM_PORTS] = [VolatileCell::new(0), VolatileCell::new(0)];

/// An opaque wrapper around a configurable pin. Hardware peripherals that need
/// to internally assign GPIO pins to specific functions should require a pin of
/// this type for their configuration.
pub struct Pinmux(Pin);

impl Pinmux {
    /// Creates a new `Pinmux` wrapping the specific GPIO pin.
    ///
    /// # Panics
    ///
    /// If a `Pinmux` for this pin has already been created.
    pub unsafe fn new(pin: Pin) -> Pinmux {
        let port: usize = (pin as usize) / PIN_PER_PORT;
        let pin_idx: usize = (pin as usize) % PIN_PER_PORT;
        let used_pins = USED_PINS[port].get();
        if used_pins & (1 << pin_idx) != 0 {
            panic!("Pin {} is already in use!", pin as usize);
        } else {
            USED_PINS[port].set(used_pins | 1 << pin_idx);
            Pinmux(pin)
        }
    }

    pub fn get_pin_index(self) -> u32 {
        self.0 as u32
    }
}

impl Into<u32> for Pinmux {
    fn into(self) -> u32 {
        self.0 as u32
    }
}
