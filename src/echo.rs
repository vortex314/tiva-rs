use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use embedded_hal::digital::OutputPin;

use core::cell::RefCell;
use embassy_time::Duration;
use embassy_time::Timer;
//use embassy_futures::join::join;
use crate::timer_driver::msec;
use embassy_futures::select::select;
use log::info;

#[derive(Debug, Clone, Default)]
pub enum EchoCmd {
    #[default]
    EchoEmpty,
    EchoMsg(u32, u64),
}

pub struct Echo {
    max_count: u32,
    start_time: u64,
}

impl Echo {
    pub fn new(max_count: u32) -> Self {
        Echo {
            max_count,
            start_time: msec(),
        }
    }
}

impl Actor<EchoCmd, EchoCmd> for Echo {
    fn init(&mut self, wrapper: &mut ActorWrapper<EchoCmd, EchoCmd>) {
        info!("Echo init");
        wrapper.emit(&EchoCmd::EchoMsg(0, self.start_time));
    }
    fn on(&mut self, cmd: &EchoCmd, _me: &mut ActorWrapper<EchoCmd, EchoCmd>) {
        match cmd {
            EchoCmd::EchoEmpty => {}
            EchoCmd::EchoMsg(count, start) => {
                if *count < self.max_count {
                    _me.emit(&EchoCmd::EchoMsg(count + 1, *start));
                } else {
                    let now = msec();
                    info!("Echo {} ms", now - start);
                }
            }
        }
    }
}
