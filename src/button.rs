use core::cell::RefCell;
use core::pin::pin;
use core::pin::Pin;
use core::task::Context;
use core::task::Poll;
use core::task::Waker;

use alloc::boxed::Box;
use alloc::rc::Rc;
use cortex_m_rt::exception;
use embassy_sync::waitqueue::WakerRegistration;
use embassy_time::with_timeout;
use embedded_hal::can::Error;
use embedded_hal::digital::v2::InputPin;
use futures::Future;
use log::info;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c_hal::gpio;
use void::Void;

use crate::limero::*;
use cortex_m_semihosting::hprintln;
use tm4c123x::interrupt;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Duration;
use embassy_time::Timer;

#[derive(Debug, Clone)]
pub enum ButtonEvent {
    Released,
    Pressed,
}


static mut IRQ_SENDER: Option<Emitter<ButtonEvent>> = None;
pub struct Button {
    pressed: bool,
    emitter: Emitter<ButtonEvent>,
    pin: Box<dyn InputPin<Error = ()>>,
}

impl Button {
    pub fn new(pin: impl InputPin<Error = ()> + 'static) -> Self {
        let reg = WakerRegistration::new();
        Button {
            pressed: false,
            emitter: Emitter::new(),
            pin: Box::new(pin),
        }
    }
    pub async fn run() {
        Timer::after(Duration::from_millis(u64::MAX)).await;
    }
}

impl  Source<ButtonEvent> for Button {
    fn add_handler(& mut self, handler: Box<dyn Handler<ButtonEvent>>) {
        self.emitter.add_handler(handler);
    }
}

#[interrupt]
fn GPIOF() {
    
}
