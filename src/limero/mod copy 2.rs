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

use core::{cell::RefCell, mem};
use core::{
    ops::Shr,
    pin::Pin,
    task::{Context, Poll},
};

use alloc::sync::Arc;
use embassy_futures::select::*;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel, pubsub::publisher::Pub};
use embassy_time::{with_timeout, Duration, Instant, TimeoutError, Timer};
use futures::Future;
use log::{info, warn};
use mini_io_queue::asyncio;
use mini_io_queue::asyncio::queue;
use mini_io_queue::storage::HeapBuffer;

#[derive(Debug, Clone, Default)]
pub enum NoEvent {
    #[default]
    Zero = 0,
}
#[derive(Debug, Clone, Default)]
pub enum NoCmd {
    #[default]
    Zero = 0,
}
pub trait Listener<T> {
    fn on(&self, value: &T);
}

pub trait Runner {
    fn run(&self);
}

pub trait Publisher<T> {
    fn add_listener(&self, listener: Box<dyn Listener<T>>);
    fn remove_listener(&self, listener: &Box<dyn Listener<T>>);
    fn emit(&self, value: &T);
}

pub trait Actor<CMD, EVENT>: Future<Output = ()> + Unpin + 'static {
    fn init(&mut self, scheduler: Rc<TimerScheduler<CMD>>);
    fn on(
        &mut self,
        cmd: &CMD,
        scheduler: &mut TimerScheduler<CMD>,
        publisher: &mut Emitter<EVENT>,
    );
}

fn compare_box<T: ?Sized>(left: &Box<T>, right: &Box<T>) -> bool {
    let left: *const T = left.as_ref();
    let right: *const T = right.as_ref();
    left == right
}

#[derive(Debug, Clone, PartialEq, Copy)]
enum ClockType {
    OneShot,
    Interval,
    Gated,
}

#[derive(Debug, Clone, PartialEq, Copy)]
struct ClockEntry<CMD> {
    expires_at: Instant,
    interval: Duration,
    cmd: CMD,
    clock_type: ClockType,
}

pub struct TimerScheduler<CMD> {
    clock_entries: Vec<ClockEntry<CMD>>,
    next_alarm: Option<ClockEntry<CMD>>,
}

impl<CMD> TimerScheduler<CMD>
where
    CMD: Clone + Default,
{
    fn new() -> TimerScheduler<CMD> {
        TimerScheduler {
            clock_entries: Vec::new(),
            next_alarm: None,
        }
    }
    pub fn set_alarm(&mut self, cmd: CMD, clock: Instant) {
        self.clock_entries.push(ClockEntry {
            expires_at: clock,
            cmd,
            interval: Duration::from_millis(10),
            clock_type: ClockType::OneShot,
        });
        self.next_alarm = self.next_alarm();
    }

    pub fn one_shot(&mut self, cmd: CMD, delta: Duration) {
        self.clock_entries.push(ClockEntry {
            expires_at: Instant::now() + delta,
            cmd,
            interval: Duration::from_millis(10),
            clock_type: ClockType::OneShot,
        });
        self.next_alarm = self.next_alarm();
    }

    pub fn interval_timer(&mut self, cmd: CMD, interval: Duration) {
        info!("set_interval {} ", interval.as_millis());
        self.clock_entries.push(ClockEntry {
            expires_at: Instant::now() + interval,
            interval,
            cmd,
            clock_type: ClockType::Interval,
        });
        self.next_alarm = self.next_alarm();
    }

    pub fn gated(&mut self, cmd: CMD, interval: Duration) {
        self.clock_entries.push(ClockEntry {
            expires_at: Instant::now() + interval,
            interval,
            cmd,
            clock_type: ClockType::Gated,
        });
        self.next_alarm = self.next_alarm();
    }

    // set timer to send cmd after delay

    pub fn cancel_timer(&mut self, cmd: CMD) {
        let mut i = 0;
        while i < self.clock_entries.len() {
            if mem::discriminant(&self.clock_entries[i].cmd) == mem::discriminant(&cmd) {
                self.clock_entries.remove(i);
            } else {
                i += 1;
            }
        }
        self.next_alarm = self.next_alarm();
    }

    fn next_alarm(&self) -> Option<ClockEntry<CMD>> {
        let mut min = Instant::now() + Duration::from_millis(1000000);
        let mut clock_entry: Option<ClockEntry<CMD>> = None;
        for entry in self.clock_entries.iter() {
            if entry.expires_at < min {
                info!(
                    "next_alarm [{}] {} ",
                    self.clock_entries.len(),
                    entry.expires_at.as_millis()
                );
                min = entry.expires_at;
                let ec = entry;
                clock_entry = Some((*ec).clone());
            }
        }
        clock_entry
    }

    fn handle_timeout(&mut self) -> Vec<CMD> {
        info!("handle_timeout()");
        let mut timeouts: Vec<CMD> = Vec::new();
        let mut i = 0;
        while i < self.clock_entries.len() {
            info!("handle_timeout() index [{}]", i);
            if self.clock_entries[i].expires_at <= Instant::now() {
                timeouts.push(self.clock_entries[i].cmd.clone());
                match self.clock_entries[i].clock_type {
                    ClockType::OneShot => {
                        self.clock_entries.remove(i);
                    }
                    ClockType::Gated => {
                        let interval = self.clock_entries[i].interval;
                        self.clock_entries[i].expires_at += interval;
                    }
                    ClockType::Interval => {
                        self.clock_entries[i].expires_at =
                            Instant::now() + self.clock_entries[i].interval;
                    }
                }
            } else {
                i += 1;
            }
        }
        self.next_alarm = self.next_alarm();
        timeouts
    }
}

pub struct Emitter<EVENT> {
    listeners: Vec<Box<dyn Listener<EVENT>>>,
}

impl<EVENT> Emitter<EVENT> {
    fn new() -> Emitter<EVENT> {
        Emitter {
            listeners: Vec::new(),
        }
    }
    fn add_listener(&mut self, listener: Box<dyn Listener<EVENT>>) {
        self.listeners.push(listener);
    }
    fn remove_listener(&mut self, listener: &Box<dyn Listener<EVENT>>) {
        self.listeners.retain(|x| compare_box(x, listener) == false);
    }
    pub fn emit(&self, value: &EVENT) {
        for listener in self.listeners.iter() {
            listener.on(value);
        }
    }
}

struct ActorCell<CMD,EVENT,const SIZE:usize> {
    emitter: Emitter<EVENT>,
    channel: channel::Channel<NoopRawMutex, CMD, SIZE>,
    timer_scheduler: Rc<TimerScheduler<CMD>>,
}
pub struct ActorWrapper<CMD, EVENT,const SIZE:usize> {
    actor: Box<dyn Actor<CMD, EVENT>>,
    actor_cell: Rc<ActorCell<CMD,EVENT,SIZE>>,
}

impl<CMD, EVENT,const SIZE:usize> ActorWrapper<CMD, EVENT,SIZE>
where
    CMD: Clone + Default,
    EVENT: Clone + Default +'static,
{
    fn new(actor: Box<dyn Actor<CMD, EVENT,SIZE>>, capacity: usize) -> ActorWrapper<CMD, EVENT,SIZE> {
        // let (mut reader, mut writer) = queue(capacity);
        let channel = channel::Channel::<NoopRawMutex, CMD, SIZE>::new();
        let timer_scheduler = TimerScheduler::<CMD>::new();
        ActorWrapper {
            actor,
            emitter: Emitter::new(),
            channel,
            timer_scheduler: Rc::new(TimerScheduler::new()),
        }
    }

    fn init(&mut self) {
        self.actor.init(self.timer_scheduler);
    }

    async fn recv(&mut self) -> CMD {
        self.channel.receive().await
    }

    fn on(&mut self, cmd: &CMD) {
        let res = self.channel.try_send(cmd.clone());
        match res {
            Ok(()) => {}
            Err(_) => {
                warn!("ActorWrapper::on() queue full");
            }
        }
    }

    // find next alarm entry
    // if clock is in the past, send cmd and remove entry
    // if clock is in the future, wait for clock and repeat
    // if no entry, wait forever
    async fn process_message(&mut self) {
        //     let cmd = self.recv().await;
        //      self.actor
        //          .on(&cmd, &mut self.timer_scheduler, &mut self.emitter);
    }

    async fn run(&mut self) {
   /*      loop {
            match &self.timer_scheduler.next_alarm {
                Some(clock_entry) => {
                    if clock_entry.expires_at <= Instant::now() {
                        // clock passed
                        self.timer_scheduler
                            .handle_timeout()
                            .iter()
                            .for_each(|cmd| self.on(cmd));
                    } else {
                        let timeout = clock_entry.expires_at - Instant::now();
                        let res = with_timeout(timeout, self.process_message()).await;
                        match res {
                            Err(TimeoutError) => {
                                self.timer_scheduler
                                    .handle_timeout()
                                    .iter()
                                    .for_each(|cmd| self.on(cmd));
                            }
                            Ok(()) => {}
                        }
                    }
                }
                None => {
                    self.process_message().await;
                }
            }
        }*/
    }
}

pub struct ActorRef<CMD, EVENT> {
    actor_wrapper: Rc<RefCell<ActorWrapper<CMD, EVENT>>>,
}

impl<CMD, EVENT> ActorRef<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    pub fn new(mut actor: Box<dyn Actor<CMD, EVENT>>, capacity: usize) -> ActorRef<CMD, EVENT> {
        ActorRef {
            actor_wrapper: Rc::new(RefCell::new(ActorWrapper::new(actor, 3))),
        }
    }

    pub fn tell(&self, cmd: &CMD) {
        let _ = self.actor_wrapper.borrow().channel.try_send(cmd.clone());
    }

    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>) {
        self.actor_wrapper
            .borrow_mut()
            .emitter
            .add_listener(listener);
    }

    fn remove_listener(&self, listener: &Box<dyn Listener<EVENT>>) {
        self.actor_wrapper
            .borrow_mut()
            .emitter
            .remove_listener(listener);
    }
    fn emit(&self, value: &EVENT) {
        self.actor_wrapper.borrow().emitter.emit(value);
    }
}

impl<CMD,EVENT> Future for & ActorRef<CMD,EVENT> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        Poll::Pending
    }
}

impl<CMD, EVENT> Listener<CMD> for ActorRef<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn on(&self, value: &CMD) {
        self.tell(value);
    }
}

impl<CMD, EVENT> Clone for ActorRef<CMD, EVENT> {
    fn clone(&self) -> Self {
        ActorRef {
            actor_wrapper: Rc::clone(&self.actor_wrapper),
        }
    }
}

//======================  Handler  ======================

pub struct Mapper<CMD, EVENT> {
    func: fn(&CMD) -> Option<EVENT>,
    emitter: Rc<RefCell<Emitter<EVENT>>>,
}

impl<CMD, EVENT> Clone for Mapper<CMD, EVENT> {
    fn clone(&self) -> Self {
        Mapper {
            func: self.func,
            emitter: self.emitter.clone(),
        }
    }
}

impl<EVENT, CMD> Mapper<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    pub fn new(func: fn(&CMD) -> Option<EVENT>) -> Self {
        Mapper {
            func,
            emitter: Rc::new(RefCell::new(Emitter::new())),
        }
    }
    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>) {
        self.emitter.borrow_mut().add_listener(listener);
    }

    fn remove_listener(&self, listener: &Box<dyn Listener<EVENT>>) {
        self.emitter.borrow_mut().remove_listener(listener);
    }
    fn emit(&self, value: &EVENT) {
        self.emitter.borrow().emit(value);
    }
}

impl<CMD, EVENT> Listener<CMD> for Mapper<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn on(&self, cmd: &CMD) {
        (self.func)(cmd).map(|e| self.emit(&e));
    }
}

pub fn mapper<CMD, EVENT>(func: fn(&CMD) -> Option<EVENT>) -> Mapper<CMD, EVENT>
where
    CMD: Clone + Default + 'static,
    EVENT: Clone + Default + 'static,
{
    Mapper::new(func)
}

//====================== Sink ======================

pub struct Sink<CMD, F>
where
    F: Fn(&CMD),
{
    func: Box<F>,
    phantom: core::marker::PhantomData<CMD>,
}
impl<F, CMD> Sink<CMD, F>
where
    CMD: Clone + Default,
    F: Fn(&CMD),
{
    pub fn new(func: F) -> Self {
        Sink {
            func: Box::new(func),
            phantom: core::marker::PhantomData,
        }
    }
}

/*impl<CMD,F> Clone for Sink<CMD,F> where F: Fn(&CMD) {
    fn clone(&self) -> Self {
        Sink { func: self.func ,phantom: core::marker::PhantomData }
    }
}*/

impl<CMD, F> Listener<CMD> for Sink<CMD, F>
where
    CMD: Clone + Default,
    F: Fn(&CMD),
{
    fn on(&self, cmd: &CMD) {
        (self.func)(cmd);
    }
}
//======================  &ActorRef >> &ActorRef ======================

impl<'a, T, U, V> Shr<&'a ActorRef<U, V>> for &'a ActorRef<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    V: Clone + Default + 'static,
{
    type Output = &'a ActorRef<U, V>;

    fn shr(self, rhs: &'a ActorRef<U, V>) -> Self::Output {
        self.add_listener(Box::new(rhs.clone()));
        rhs
    }
}

//======================  &ActorRef >> Sink ======================

impl<'a, T, U, F> Shr<Sink<U, F>> for &'a ActorRef<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    F: Fn(&U) + 'static,
{
    type Output = ();

    fn shr(self, rhs: Sink<U, F>) -> Self::Output {
        self.add_listener(Box::new(rhs));
        ()
    }
}
//======================  &ActorRef<T,U>   >>    Listener<U>  ======================

impl<'a, T, U> Shr<Box<dyn Listener<U>>> for &'a ActorRef<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
{
    type Output = ();

    fn shr(self, rhs: Box<dyn Listener<U>>) -> Self::Output {
        self.add_listener(rhs);
        ()
    }
}

//======================  ActorRef >> Mapper >> ... ======================

impl<'a, T, U, V> Shr<&'a Mapper<U, V>> for &ActorRef<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    V: Clone + Default + 'static,
{
    type Output = &'a Mapper<U, V>;

    fn shr(self, rhs: &'a Mapper<U, V>) -> Self::Output {
        self.add_listener(Box::new(rhs.clone()));
        rhs
    }
}
//======================  Mapper >> ActorRef . ======================

impl<'a, T, U, V> Shr<&'a ActorRef<U, V>> for &'a Mapper<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    V: Clone + Default + 'static,
{
    type Output = &'a ActorRef<U, V>;

    fn shr(self, rhs: &'a ActorRef<U, V>) -> Self::Output {
        self.add_listener(Box::new(rhs.clone()));
        rhs
    }
}

/*
fn filter<T>(value: &T) -> Mapper<T, T>
where
    T: Clone + Default + 'static,
{
    Mapper::new(|v| {
        if v == value {
            Some(v.clone())
        } else {
            None
        }
    })
}*/
