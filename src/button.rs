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

#[derive(Debug, Clone, Default)]

pub enum ButtonCmd {
    #[default]
    Init,
    Open,
    Close
}

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
static mut BUTTON_STATE: Option<Box<dyn InputPin<Error = ()>>> = None;
pub struct Button {
    state: ButtonState,
}

impl Button {
    pub fn new(pin: impl InputPin<Error = ()> + 'static) -> Self {
        let reg = WakerRegistration::new();
        Button {
            state: ButtonState {
                pressed: false,
                pin: Box::new(pin),
            }
        }
    }
}

impl Actor<ButtonCmd,ButtonEvent> for Button {
    fn init(&mut self, scheduler: &mut TimerScheduler<ButtonCmd>) {
        info!("Button init");
        let p = unsafe { tm4c123x::Peripherals::steal() };
        let mut sysctl = p.SYSCTL.constrain();
        let mut portf = p.GPIO_PORTF.split(&sysctl.power_control);
        let mut switch1 = portf.pf4.into_pull_up_input();
        switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
        unsafe {
            BUTTON_STATE = Some(Box::new(switch1));
        }
    }
    fn on(&mut self, cmd: &ButtonCmd, scheduler: &mut TimerScheduler<ButtonCmd>, emitter: &mut dyn Publisher<ButtonEvent>) {
        match cmd {
            ButtonCmd::Init => {
                let p = unsafe { tm4c123x::Peripherals::steal() };
                let mut sysctl = p.SYSCTL.constrain();
                let mut portf = p.GPIO_PORTF.split(&sysctl.power_control);
                let mut switch1 = portf.pf4.into_pull_up_input();
                switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
                unsafe {
                    BUTTON_STATE = Some(Box::new(switch1));
                }
            }
            ButtonCmd::Open => {
                emitter.emit(&ButtonEvent::Released);
            }
            ButtonCmd::Close => {
                emitter.emit(&ButtonEvent::Pressed)
            }
        }
    }
}


// interrupt handler wake ButtonActor
static mut GPIOF_INTERRUPTS: u32 = 1000;

#[interrupt]
fn GPIOF() {
    /* let p = unsafe { tm4c123x::Peripherals::steal() };
    let mut portf = p.GPIO_PORTF.split(&p.SYSCTL.power_control);
    let mut switch1 = portf.pf4.into_pull_up_input();
    switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);*/
// steal periheral
    let p = unsafe { tm4c123x::Peripherals::steal() };
    let mut sysctl = p.SYSCTL.constrain();
    let mut portf = p.GPIO_PORTF.split(&sysctl.power_control);
    let mut switch1 = portf.pf4.into_pull_up_input();
    switch1.clear_interrupt();
    hprintln!("GPIOF interrupt");
    unsafe {
        GPIOF_INTERRUPTS += 1;
    }

//    BUTTON_WAKER.wake();
}

