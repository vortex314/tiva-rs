use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use embassy_time::Instant;
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
    TimerBlink,
}
pub struct Led {
    state: LedCmd,
    interval_ms: u64,
    pin: Box<dyn OutputPin>,
    pin_high: bool,
}

impl Led {
    pub fn new(pin: impl OutputPin + 'static) -> Self {
        Led {
            state: LedCmd::On,
            interval_ms: 100,
            pin: Box::new(pin),
            pin_high: false,
        }
    }
}

impl Actor<LedCmd, NoEvent> for Led {
    fn init(&mut self,wrapper:&mut ActorWrapper<LedCmd,NoEvent>) {
        info!("Led init");
        wrapper.timer_scheduler.interval_timer(
            LedCmd::TimerBlink,
            Duration::from_millis(1000),
        );
        self.pin.set_high();
        self.pin_high = true;
        self.state = LedCmd::On;
    }
    fn on(&mut self, cmd: &LedCmd,wrapper:&mut ActorWrapper<LedCmd,NoEvent>) {
        info!("Led cmd {:?}", cmd);
        self.state = cmd.clone();
        match cmd {
            LedCmd::On => {
                self.pin.set_high();
                self.pin_high = true;
            }
            LedCmd::Off => {
                self.pin.set_low();
                self.pin_high = false;
            }
            LedCmd::Blink(intv) => {
                wrapper.timer_scheduler.set_alarm(
                    LedCmd::TimerBlink,
                    Instant::now() + Duration::from_millis(*intv as u64),
                );
                self.interval_ms = *intv as u64;
            }
            LedCmd::TimerBlink => {
                if let LedCmd::Blink(x) = self.state {
                    if self.pin_high {
                        self.pin.set_low();
                        self.pin_high = false;
                    } else {
                        self.pin.set_high();
                        self.pin_high = true;
                    }
                }
            }
        }
    }
}
