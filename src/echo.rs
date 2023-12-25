use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use embassy_sync::channel;
use embassy_time::Instant;
use embedded_hal::digital::OutputPin;

use crate::timer_driver::msec;
use core::cell::RefCell;
use core::f32::consts::E;
use core::marker::PhantomData;
use core::task::Poll;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::channel::Receiver;
use embassy_sync::channel::Sender;
use embassy_time::Duration;
use embassy_time::Timer;
use log::info;

#[derive(Debug, Clone)]
pub enum EchoCmd {
    EchoMsg(u32, u64),
}

pub struct Echo<> {
    max_count: u32,
    start_time: u64,
    channel: Channel<NoopRawMutex, EchoCmd, 3>,
    emitter: Emitter<EchoCmd>,
    p: PhantomData<()>,
}

impl Echo {
    pub fn new(max_count: u32) -> Self {
        let channel = Channel::<NoopRawMutex, EchoCmd, 3>::new();
        Echo {
            max_count,
            start_time: msec(),
            channel,
            emitter: Emitter::new(),
            p: PhantomData,
        }
    }

    pub fn handle(&self,cmd:&EchoCmd)  {
        let sender = self.channel.try_send(cmd.clone());
    }

    pub async fn run(&mut self) {
        loop {
            Timer::after(Duration::from_millis(1000)).await;
            self.emitter
                .emit(EchoCmd::EchoMsg(0, Instant::now().as_millis() as u64));
            let cmd = self.channel.receiver().receive().await;
            match cmd {
                EchoCmd::EchoMsg(count, ts) => {
                    if count < self.max_count {
                        let msg = EchoCmd::EchoMsg(count + 1, ts);
                        self.emitter.emit(msg);
                    } else {
                        let delta = Instant::now().as_millis() as u64 - ts;
                        info!("Echo done {} messages in {} ms", count, delta);
                        info!("Echo done {} messages per second", count as u64  * 1000 / delta);
                    }
                }
            }
        }
    }
}



impl Source<EchoCmd> for Echo<> {
    fn add_handler(&mut self, handler: Box<dyn Handler<EchoCmd>> ) {
        self.emitter.add_handler(handler);
    }
}
