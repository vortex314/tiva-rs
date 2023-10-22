use core::cell::Cell;
use core::convert::Infallible;
use cortex_m_semihosting::hprintln;
use embedded_hal::digital::v1::OutputPin;
use futures::select_biased;
use thingbuf as conn;
use crate::limero::{Sink,TimerClient, TIMER_SERVER, get_timer_server,TimerMsg};
use alloc::vec::Vec;
use futures::FutureExt;


pub struct Led<'a> {
    pin: &'a mut dyn OutputPin,
    led_state: bool,
    active: bool,
    timer_tick:Sink<TimerMsg> ,
    active_sink: Sink<bool>,
}

impl TimerClient for Led<'_> {
    fn on_timer(& self, timer_id: u32) {
    }
}

impl<'a> Led<'a> {
    pub fn new(pin: &'a mut dyn OutputPin) -> Self {     
        Self {
            pin,
            led_state: false,
            active: true,
            timer_tick: Sink::<TimerMsg>::new(10),
            active_sink: Sink::<bool>::new(10),
        }
    }
    fn toggle(&mut self ){
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
        self.led_state=false;
        self.pin.set_low();
    }

    pub async fn run(&mut self) -> Infallible {
        get_timer_server().new_interval(100,  self.timer_tick.sender());
        loop {
            select_biased! {
                msg = self.active_sink.recv_async().fuse() => {
                    self.set_active(msg);
                }
                _ = self.timer_tick.recv_async().fuse() => {
                    self.toggle();
                }
            }
        }   
    }
}
