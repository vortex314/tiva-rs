use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::sync::Arc;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::DynamicSender;
use embassy_time::Instant;
use embedded_hal::digital::OutputPin;
use futures::Future;

use core::cell::RefCell;
use core::pin::Pin;
use core::task::Context;
use core::task::Poll;
use core::task::Waker;
use embassy_time::Duration;
use embassy_time::Timer;
//use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_sync::channel::Channel;
use embassy_sync::channel::{self, Receiver, Sender};
use log::info;

#[derive(Debug, Clone)]
pub enum LedCmd {
    On,
    Off,
    Blink(u32),
}
pub struct Led {
    channel: Rc<RefCell<Channel<NoopRawMutex, LedCmd, 3>>>,
    state: LedCmd,
    interval_ms: u64,
    pin: Box<dyn OutputPin>,
    pin_high: bool,
    scheduler: TimerScheduler,
}

impl Led {
    pub fn new(pin: impl OutputPin + 'static, capacity: usize) -> Self {
        Led {
            channel:Rc::new(RefCell::new(Channel::<NoopRawMutex, LedCmd, 3>::new())),
            state: LedCmd::On,
            interval_ms: 100,
            pin: Box::new(pin),
            pin_high: false,
            scheduler: TimerScheduler::new(),
        }
    }
    pub async fn run(&mut self) {
        info!("Led run");
        loop {
            let cmd = self.channel.borrow().receiver().receive().await;
            info!("Led run {:?}", cmd);
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
                    self.interval_ms = intv as u64;
                }
            }
        }
    }
}

impl Handler<LedCmd> for Led {
    fn handle(&self, cmd: LedCmd) {
        let sender = self.channel.borrow().try_send(cmd.clone());
    }
}

impl Sink<LedCmd> for Led {
    fn handler(&self) -> Box<dyn Handler<LedCmd>> {
        struct LedHandler {
            channel: Rc<RefCell<Channel<NoopRawMutex,LedCmd,3>>>,
        }
        impl<'a> Handler<LedCmd> for LedHandler {
            fn handle(&self, cmd: LedCmd) {
                let _ = self.channel.borrow_mut().try_send(cmd.clone());
            }
        }
        Box::new(LedHandler {
            channel: self.channel.clone(),
        })
    }
}

