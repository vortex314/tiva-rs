#[cfg(all(feature = "std", feature = "no_std"))]
compile_error!("feature \"std\" and feature \"no_std\" cannot be enabled at the same time");

#[cfg(feature = "tokio")]
use {
    std::io::Write,
    std::pin::pin,
    std::rc::Rc,
    std::sync::Arc,
    std::thread::sleep,
    std::time::{Duration, Instant},
    std::{ops::Shr, pin::Pin},
    tokio::task::block_in_place,
};

#[cfg(feature = "embassy")]
use {alloc::boxed::Box, alloc::rc::Rc, alloc::vec::Vec, embassy_futures::block_on};

use core::{cell::RefCell, default, mem, marker::PhantomData};
use core::{
    ops::Shr,
    pin::Pin,
    task::{Context, Poll},
};

use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use embassy_futures::select::*;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel, pubsub::publisher::Pub};
use embassy_time::{with_timeout, Duration, Instant, TimeoutError};
use futures::Future;
use log::{info, warn};

pub trait Handler<T> {
    fn handle(&self, cmd: T);
}

pub trait Sink<T> {
    fn handler(&self) -> Box<dyn Handler<T>>; // cloneable handler
}

pub trait Source<T> {
    fn add_handler(&mut self, handler: Box<dyn Handler<T>>);
}

pub trait Flow<T, U>: Sink<T> + Source<U> {}

pub struct Mapper<'a,T, U> {
    emitter: Rc<RefCell<Emitter<U>>>,
    func: Rc<dyn Fn(T) -> U>,
    phantom: PhantomData<&'a ()>,
}

impl<'a,T,U> Clone for Mapper<'a,T, U> {
    fn clone(&self) -> Self {
        Self {
            emitter: self.emitter.clone(),
            func: self.func.clone(),
            phantom: PhantomData,
        }
    }
}

impl<'a,T, U> Mapper<'a,T, U> {
    pub fn new(func: impl Fn(T) -> U +'static) -> Self {
        Self {
            emitter: Rc::new(RefCell::new(Emitter::new())),
            func: Rc::new(func),
            phantom: PhantomData,
        }
    }
}

impl<'a,T:'static,U:'static> Sink<T> for Mapper<'a,T, U> where U: Clone +'static{
    fn handler(&self) -> Box<dyn Handler<T>> {
        let emitter = self.emitter.clone();
        let func = self.func.clone();
        struct MapperHandler<T, U> {
            emitter: Rc<RefCell<Emitter<U>>>,
            func: Rc<dyn Fn(T) -> U>,
        }
        impl<T, U> Handler<T> for MapperHandler<T, U>
        where
            U: Clone,
        {
            fn handle(&self, cmd: T) {
                let cmd = (self.func)(cmd);
                self.emitter.borrow().emit(cmd);
            }
        }
        Box::new(MapperHandler {
            emitter,
            func,
        })
    }
}

impl<'a,T, U> Source<U> for Mapper<'a,T, U> {
    fn add_handler(&mut self, handler: Box<dyn Handler<U>>) {
        self.emitter.borrow_mut().add_handler(handler);
    }
}

impl<'a,T,U> Handler<T> for Mapper<'a,T, U>
where
    U: Clone,
{
    fn handle(&self, cmd: T) {
        let cmd = (self.func)(cmd);
        self.emitter.borrow().emit(cmd);
    }
}



pub struct Emitter<T> {
    handlers: Vec< Box<dyn Handler<T>>>,
}

impl<T> Emitter<T> {
    pub fn new() -> Self {
        Self {
            handlers: Vec::new(),
        }
    }
    pub fn emit(&self, t: T)
    where
        T: Clone,
    {
        for handler in self.handlers.iter() {
            handler.handle(t.clone());
        }
    }
    pub fn add_handler(&mut self, handler: Box<dyn Handler<T>>) {
        self.handlers.push(handler);
    }
}

/*impl<T> Clone for Emitter<T> {
    fn clone(&self) -> Self {
        Self {
            handlers: self.handlers.clone(),
        }
    }
}*/

pub fn link<T>(source: &mut dyn Source<T>, sink: &dyn Sink<T>) {
    source.add_handler(sink.handler());
}


type TimerId = u32;

#[derive(Debug)]
pub enum TimerCmd {
    Gated(TimerId, Duration),
    Interval(TimerId, Duration),
    Once(TimerId, Instant),
    TimerExpired(TimerId),
}

/// a infinite duration u64::MAX/2
const fn forever() -> Duration {
    Duration::from_millis(1_000_000_000)
}
/// a infinite instant now() + DUration::from_millis(u64::MAX/2
fn infinity() -> Instant {
    Instant::now() + forever()
}

pub enum TimerType {
    Gated,
    Interval,
    Once,
}
pub struct Timer {
    expires_at: Instant,
    timer_type: TimerType,
    interval: Option<Duration>,
    id: u8,
}
impl Timer {
    pub fn once(id: u8, instant: Instant) -> Self {
        Timer {
            expires_at: instant,
            timer_type: TimerType::Once,
            interval: None,
            id,
        }
    }
    /// Creates a new timer that expires at `instant` and then repeats at
    /// the given interval.

    pub fn interval(id: u8, instant: Instant, interval: Duration) -> Self {
        Timer {
            expires_at: instant,
            timer_type: TimerType::Interval,
            interval: Some(interval),
            id,
        }
    }
    /// Creates a new timer that expires at `instant` and then repeats at
    /// the given interval.
    pub fn gated(id: u8, instant: Instant, interval: Duration) -> Self {
        Timer {
            expires_at: instant,
            timer_type: TimerType::Gated,
            interval: Some(interval),
            id,
        }
    }
    /// Returns `true` if the timer has expired.
    fn is_expired(&self) -> bool {
        self.expires_at < Instant::now()
    }
    /// Updates the timer's expiration time based on its current type.
    /// If the timer is repeating, the interval is added to the current
    /// expiration time or the current time
    ///
    fn update_timeout(&mut self) {
        match self.timer_type {
            TimerType::Once => {
                self.expires_at = infinity();
            }
            TimerType::Interval => {
                self.expires_at = Instant::now() + self.interval.unwrap();
            }
            TimerType::Gated => {
                self.expires_at += self.interval.unwrap();
            }
        }
    }
}

pub struct TimerScheduler {
    timers: BTreeMap<u8, Timer>,
}

impl TimerScheduler {
    pub fn new() -> Self {
        TimerScheduler {
            timers: BTreeMap::new(),
        }
    }
    pub fn add_timer(&mut self, timer: Timer) {
        self.timers.insert(timer.id, timer);
    }
    pub fn del_timer(&mut self, id: u8) {
        self.timers.remove(&id);
    }
    pub fn get_timer(&self, id: u8) -> Option<&Timer> {
        self.timers.get(&id)
    }

    pub fn expired_list(&mut self) -> Vec<u8> {
        let mut expired = Vec::new();
        for timer in self.timers.iter_mut() {
            if timer.1.is_expired() {
                expired.push(*timer.0);
            }
        }
        expired
    }
    pub fn reload(&mut self) {
        for timer in self.timers.iter_mut() {
            if timer.1.is_expired() {
                timer.1.update_timeout();
            }
        }
    }

    pub fn soonest(&self) -> Option<Duration> {
        let infinity = infinity();
        let mut soonest = infinity;
        for timer in self.timers.iter() {
            if timer.1.expires_at < soonest {
                soonest = timer.1.expires_at;
            }
        }
        //       info!("soonest={:?} vs infinity() {:?}",soonest,infinity);

        if soonest == infinity {
            None
        } else if soonest < Instant::now() {
            None
        } else {
            Some(soonest-Instant::now())
        }
    }
}
