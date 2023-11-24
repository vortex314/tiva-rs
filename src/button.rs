use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
use log::info;

use embassy_time::Duration;
use embassy_time::Timer;


#[derive(Debug, Clone, Default)]
pub enum ButtonEvent {
    #[default]
    Released,
    Pressed,
}

pub struct Button {
    pub actor: Actor<NoCmd, ButtonEvent>,
}
impl Button {
    pub fn new() -> Self {
        Button {
            actor: Actor::new(0), // no input queue neeeded
        }
    }
    pub async fn run(&mut self) {
        loop {
            Timer::after(Duration::from_millis(1000)).await;
            info!("I Button pressed");
            self.actor.emit(&ButtonEvent::Pressed);
            Timer::after(Duration::from_millis(1000)).await;
            info!("I Button Released");
            self.actor.emit(&ButtonEvent::Released);
        }

    }
}

// interrupt handler wake ButtonActor

use cortex_m_semihosting::hprintln;
use tm4c123x::interrupt;

#[interrupt]
unsafe fn GPIOF() {
    hprintln!("GPIOF");
    //  button_actor.receptor.emit(&ButtonEvent::Pressed);
}