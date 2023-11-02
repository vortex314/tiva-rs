use crate::limero::Sink;
use core::convert::Infallible;
use cortex_m_semihosting::hprintln;
use embedded_hal::digital::v1::OutputPin;
use futures::FutureExt;
use futures::{future::select, select_biased};

use embassy_time::Duration;
use embassy_time::Timer;

pub struct Led<'a> {
    pin: &'a mut dyn OutputPin,
    led_state: bool,
    active: bool,
    pub active_sink: Sink<bool,3>,
}

impl<'a> Led<'a> {
    pub fn new(pin: &'a mut dyn OutputPin) -> Self {
        Self {
            pin,
            led_state: false,
            active: true,
            active_sink: Sink::<bool,3>::new(),
        }
    }

    pub async fn blink(&mut self) {
        loop {
            self.toggle();
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    fn toggle(&mut self) {
        if self.active {
            if self.led_state {
                self.pin.set_low();
                self.led_state = false;
            } else {
                self.pin.set_high();
                self.led_state = true;
            }
        }
    }

    fn set_active(&mut self, active: bool) {
        self.active = active;
        self.led_state = false;
        self.pin.set_low();
    }

    pub async fn run(&mut self) -> Infallible {
        loop {
            select_biased! {
                msg = self.blink().fuse() => {},
            }
        }
    }
}
