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

use core::{cell::RefCell, default, mem};
use core::{
    ops::Shr,
    pin::Pin,
    task::{Context, Poll},
};

use alloc::sync::Arc;
use embassy_futures::select::*;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel, pubsub::publisher::Pub};
use embassy_time::{with_timeout, Duration, Instant, TimeoutError};
use futures::Future;
use log::{info, warn};

trait Sink<T> {
    fn on(&mut self, value: &T);
}

trait Source<T> {
    fn add_listener(&mut self, listener: Box<dyn Sink<T>>) -> usize;
    fn remove_listener(&mut self, listener_id: usize);
    fn emit(&mut self, value: &T);
}

trait Flow<T, U>: Sink<T> + Source<U> {}

trait Runner {
    fn run(&mut self);
}

type Callback = dyn FnMut() -> () + 'static;
#[derive(Debug, Clone, Default)]
enum TimerType {
    #[default]
    Undefined,
    Interval,
    Gated,
    Once,
}
struct Timer {
    func: Callback,
    expires_at: Instant,
    interval: Duration,
    timer_type: TimerType,
}
impl Timer {
    fn new(func: Callback) -> Self {
        Timer {
            func,
            expires_at: Instant::MAX,
            interval: Duration::MAX,
            timer_type: TimerType::Undefined,
        }
    }
    fn set_alarm(&mut self, time: Instant) {
        self.expires_at = time;
        self.timer_type = TimerType::Once;
    }
    fn interval_timer(&mut self, interval: Duration) {
        self.expires_at = Instant::now() + interval;
        self.interval = interval;
        self.timer_type = TimerType::Interval;
    }
    fn gated_timer(&mut self, interval: Duration) {
        self.expires_at = Instant::now() + interval;
        self.interval = interval;
        self.timer_type = TimerType::Gated;
    }
    fn cancel(&mut self) {
        self.expires_at = Instant::MAX;
        self.timer_type = TimerType::Undefined;
    }
    fn is_expired(&self) -> bool {
        Instant::now() >= self.expires_at
    }
    fn is_set(&self) -> bool {
        self.expires_at != Instant::MAX
    }
    fn is_undefined(&self) -> bool {
        self.timer_type == TimerType::Undefined
    }
    fn is_interval(&self) -> bool {
        self.timer_type == TimerType::Interval
    }
    fn is_gated(&self) -> bool {
        self.timer_type == TimerType::Gated
    }
    fn is_once(&self) -> bool {
        self.timer_type == TimerType::Once
    }
    fn handle_timeout(&mut self) {
        match self.timer_type {
            TimerType::Interval => {
                self.expires_at += self.interval;
            }
            TimerType::Gated => {
                self.expires_at += self.interval;
            }
            TimerType::Once => {
                self.expires_at = Instant::MAX;
            }
            TimerType::Undefined => {}
        }
        (self.func)();
    }
}
