use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use embedded_hal::digital::OutputPin;

use core::cell::RefCell;
use embassy_time::Duration;
use embassy_time::Timer;
//use embassy_futures::join::join;
use embassy_futures::select::select;
use log::info;

#[derive(Debug, Clone, Default)]
pub enum LedCmd {
    #[default]
    On,
    Off,
    Blink(u32),
}
pub struct Led {
    pub actor: Actor<LedCmd, NoEvent>,
    state: Rc<RefCell<LedCmd>>,
    interval_ms: Rc<RefCell<u64>>,
    pin: Box<dyn OutputPin>,
}

impl Led {
    pub fn new(pin: impl OutputPin + 'static) -> Self {
        Led {
            state: Rc::new(RefCell::new(LedCmd::Off)),
            actor: Actor::new(10),
            interval_ms: Rc::new(RefCell::new(100)),
            pin: Box::new(pin),
        }
    }
    pub async fn run(&mut self) {
        select(
            async {
                loop {
                    let state = self.state.borrow().clone();
                    match state {
                        LedCmd::On => {
                            self.pin.set_high();
                            Timer::after(Duration::from_millis(100)).await;
                        }
                        LedCmd::Off => {
                            self.pin.set_low();
                            Timer::after(Duration::from_millis(100)).await;
                        }
                        LedCmd::Blink(intv) => {
                            let interval = Duration::from_millis(intv as u64);
                            Timer::after(interval).await;
                            self.pin.set_high();
                            Timer::after(interval).await;
                            self.pin.set_low();
                        }
                    }
                }
            },
            async {
                loop {
                    let cmd = self.actor.recv().await;
                    self.state.replace(cmd);
                }
            },
        )
        .await;
    }
}
