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

#[derive(Debug, Clone, Default)]
pub enum ButtonCmd {
    #[default]
    Init,
    Changed,
}

#[derive(Debug, Clone, Default)]
pub enum ButtonEvent {
    #[default]
    Released,
    Pressed,
}

struct ButtonState {}

static mut IRQ_SENDER: Option<Emitter<ButtonEvent>> = None;
pub struct Button {
    pressed: bool,
    pin: Box<dyn InputPin<Error = ()>>,
}

impl Button {
    pub fn new(pin: impl InputPin<Error = ()> + 'static) -> Self {
        let reg = WakerRegistration::new();
        Button {
            pressed: false,
            pin: Box::new(pin),
        }
    }
}

impl Future for Button {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let mut this = self.get_mut();
        this.pressed = ! this.pressed;

        if this.pressed  {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}


impl Actor<ButtonCmd, ButtonEvent> for Button {
    fn init(&mut self, scheduler: &mut TimerScheduler<ButtonCmd>) {
        info!("Button init");
        let p = unsafe { tm4c123x::Peripherals::steal() };
        let mut sysctl = p.SYSCTL.constrain();
        let mut portf = p.GPIO_PORTF.split(&sysctl.power_control);
        let mut switch1 = portf.pf4.into_pull_up_input();
        switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
    }
    fn on(
        &mut self,
        cmd: &ButtonCmd,
        scheduler: &mut TimerScheduler<ButtonCmd>,
        emitter: &mut Emitter<ButtonEvent>,
    ) {
        /*unsafe {
            if IRQ_SENDER.is_none() {
                IRQ_SENDER = Some(*emitter);
            };
        };*/
        match cmd {
            ButtonCmd::Init => {
                let p = unsafe { tm4c123x::Peripherals::steal() };
                let mut sysctl = p.SYSCTL.constrain();
                let mut portf = p.GPIO_PORTF.split(&sysctl.power_control);
                let mut switch1 = portf.pf4.into_pull_up_input();
                switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
            }
            ButtonCmd::Changed => {
                if self.pin.is_low().unwrap() {
                    emitter.emit(&ButtonEvent::Pressed);
                } else {
                    emitter.emit(&ButtonEvent::Released);
                }
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

        if switch1.is_low().unwrap() {
            hprintln!("GPIOF interrupt low");
            if let Some(emitter) = &IRQ_SENDER {
                emitter.emit(&ButtonEvent::Pressed);
            }
        } else {
            hprintln!("GPIOF interrupt high");
            if let Some(emitter) = &IRQ_SENDER {
                emitter.emit(&ButtonEvent::Released);
            }
        }
    }
}
