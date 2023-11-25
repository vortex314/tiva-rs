use core::cell::RefCell;
use core::pin::Pin;
use core::pin::pin;
use core::task::Context;
use core::task::Poll;
use core::task::Waker;

use alloc::boxed::Box;
use alloc::rc::Rc;
use cortex_m_rt::exception;
use embassy_sync::waitqueue::WakerRegistration;
use embedded_hal::can::Error;
use embedded_hal::digital::v2::InputPin;
use futures::Future;
use void::Void;

use crate::limero::*;
use cortex_m_semihosting::hprintln;
use tm4c123x::interrupt;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Duration;
use embassy_time::Timer;

#[derive(Debug, Clone, Default)]
pub enum ButtonEvent {
    #[default]
    Released,
    Pressed,
}

struct ButtonState {
    pressed: bool,
    pin: Box<dyn InputPin<Error = ()>>,
}

static BUTTON_WAKER: AtomicWaker = AtomicWaker::new();
pub struct Button {
    pub actor: Actor<NoCmd, ButtonEvent>,
    state: Rc<RefCell<ButtonState>>,
}
impl Button {
    pub fn new(pin: impl InputPin<Error=()> + 'static) -> Self {
        let reg = WakerRegistration::new();
        Button {
            actor: Actor::new(0), // no input queue neeeded
            state: Rc::new(RefCell::new(ButtonState {
                pressed: false,
                pin: Box::new(pin),
            })),
        }
    }
    pub async fn run(&mut self) {
        self.await;

        /*loop {
            Timer::after(Duration::from_millis(5000)).await;
            self.actor.emit(&ButtonEvent::Pressed);
            Timer::after(Duration::from_millis(5000)).await;
            self.actor.emit(&ButtonEvent::Released);
        }*/
    }
}

impl futures::Future for Button {
    type Output = ButtonEvent;

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        BUTTON_WAKER.register(cx.waker());

        let mut state = self.state.borrow_mut();
        if state.pin.is_low().unwrap() {
            if !state.pressed {
                state.pressed = true;
                self.actor.emit(&ButtonEvent::Pressed);
                Poll::Ready(ButtonEvent::Pressed)
            } else {
                Poll::Pending
            }
        } else {
            if state.pressed {
                state.pressed = false;
                self.actor.emit(&ButtonEvent::Released);
                Poll::Ready(ButtonEvent::Released)
            } else {
                Poll::Pending
            }
        }
    }
}

// interrupt handler wake ButtonActor

#[interrupt]
fn GPIOF() {
    /* let p = unsafe { tm4c123x::Peripherals::steal() };
    let mut portf = p.GPIO_PORTF.split(&p.SYSCTL.power_control);
    let mut switch1 = portf.pf4.into_pull_up_input();
    switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);*/

    hprintln!("GPIOF");
    BUTTON_WAKER.wake();
    //  button_actor.receptor.emit(&ButtonEvent::Pressed);
}

