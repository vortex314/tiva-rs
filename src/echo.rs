use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use embassy_time::Instant;
use embedded_hal::digital::OutputPin;

use core::cell::RefCell;
use core::task::Poll;
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
    EchoTimer,
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
    fn init(&mut self, scheduler:&mut TimerScheduler<EchoCmd>) {
        info!("Echo init");
        scheduler.set_alarm(EchoCmd::EchoTimer,Instant::now()+Duration::from_millis(1000));
    }
    fn on(&mut self, cmd: &EchoCmd, _scheduler:&mut TimerScheduler<EchoCmd>,emitter:&mut Emitter<EchoCmd>) {
        match cmd {
            EchoCmd::EchoEmpty => {}
            EchoCmd::EchoMsg(count, start) => {
                if *count < self.max_count {
                    emitter.emit(&EchoCmd::EchoMsg(count + 1, *start));
                } else {
                    let now = msec();
                    info!("Echo {} ms", now - start);
                }
            }
            EchoCmd::EchoTimer => {
                emitter.emit(&EchoCmd::EchoMsg(0, Instant::now().as_millis()));
            }
        }
    }
}
use core::pin::Pin;
use core::task::Context;
use core::task::Waker;
use futures::Future;
static mut WAKER : Option<Waker> = None;

impl Future for Echo {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        unsafe { WAKER = Some(cx.waker().clone());};
        let mut this = self.get_mut();
            Poll::Pending
    }
}
