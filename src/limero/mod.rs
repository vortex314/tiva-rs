use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec::Vec;
use core::borrow::Borrow;
use core::convert::Infallible;
use core::fmt::Debug;
use core::pin::{pin, Pin};
use cortex_m::interrupt;
use cortex_m_semihosting::hprintln;
/*
use thingbuf::mpsc::{channel, RecvRef};
use thingbuf::mpsc::{Receiver, Sender};*/

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::channel::{self, DynamicReceiver};
use embassy_sync::channel::{Channel, DynamicSender, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::Timer;

pub struct Sink<T: Default + Clone, const SIZE: usize> {
    channel: Channel<NoopRawMutex, T, SIZE>,
}

impl<T: Default + Clone, const SIZE: usize> Sink<T, SIZE> {
    pub fn new() -> Self {
        let channel = Channel::<NoopRawMutex, T, SIZE>::new();
        Sink { channel }
    }
    pub async fn recv(&mut self) -> T {
        self.channel.receiver().receive().await
    }
    pub fn sender(&self) -> Sender<NoopRawMutex, T, SIZE> {
        self.channel.sender()
    }
    pub fn on(&self, item: T) {
        let _ = self.channel.sender().send(item);
    }
}

pub struct Source<'CH, T> {
    senders: Vec<DynamicSender<'CH, T>>,
}

impl<'CH, T: Default + Clone + Debug> Source<'CH, T>
where
    T: Clone + Send + 'static,
{
    pub fn new() -> Self {
        Source {
            senders: Vec::<DynamicSender<T>>::new(),
        }
    }
    pub async fn emit(&self, item: T) {
        hprintln!("Source::emit-1() item={:?}",item);
        for sender in self.senders.iter() {
            hprintln!("Source::emit-loop() sender={:?}",item);
            let f = sender.send(item.clone()).await;
        }
    }
}

use core::ops::Shr;
impl<'a, T: 'static + Default, const N: usize> Shr<&'static Sink<T, N>> for &mut Source<'a, T>
where
    T: Clone + Send + 'static,
{
    type Output = ();

    fn shr(self, rhs: &'static Sink<T, N>) -> Self::Output {
        let d = rhs.channel.sender().clone();
        let dy: DynamicSender<T> = d.into();
        self.senders.push(dy);
    }
}
