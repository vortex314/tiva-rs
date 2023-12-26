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


pub struct Button {
    pressed: bool,
    emitter: Rc<RefCell<Emitter<ButtonEvent>>>,
 //   pin: Box<dyn InputPin<Error = ()>>,
}

impl Button {
    pub fn new(/*pin: impl InputPin<Error = ()> + 'static*/) -> Self {
        let reg = WakerRegistration::new();
        Button {
            pressed: false,
            emitter: Rc::new(RefCell::new(Emitter::new())),
       //     pin: Box::new(pin),
        }
    }
    pub async fn run() {
        Timer::after(Duration::from_millis(u64::MAX)).await;
    }
    pub fn emit(&mut self, event: ButtonEvent) {
        self.emitter.borrow().emit(event);
    }
}

impl  Source<ButtonEvent> for Button {
    fn add_handler(& mut self, handler: Box<dyn Handler<ButtonEvent>>) {
        self.emitter.borrow_mut().add_handler(handler);
    }
}

impl Sink<ButtonEvent> for Button {
    fn handler(&self) -> Box<dyn Handler<ButtonEvent>> {
        struct ButtonHandler {
            emitter: Rc<RefCell<Emitter<ButtonEvent>>>,
        }
        impl<'a> Handler<ButtonEvent> for ButtonHandler {
            fn handle(&self, event: ButtonEvent) {
                info!("ButtonHandler {:?}", event);
                let _ = self.emitter.borrow().emit(event) ;
            }
        }
        Box::new(ButtonHandler {
            emitter: self.emitter.clone(),
        })
    }
}

