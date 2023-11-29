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
use core::ops::Shr;

use embassy_futures::select::*;
use embassy_time::{with_timeout, Duration, Instant, TimeoutError, Timer};
use log::{warn, info};
use mini_io_queue::asyncio;
use mini_io_queue::asyncio::queue;
use mini_io_queue::storage::HeapBuffer;

use nb::block;

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
pub trait Publisher<T> {
    fn add_listener(&self, listener: Box<dyn Listener<T>>);
    fn remove_listener(&self, listener_id:&Box<dyn Listener<T>>);
    fn emit(&self, value: &T);
}
pub trait Actor<CMD, EVENT> {
    fn init(&mut self, wrapper: &mut ActorWrapper<CMD, EVENT>);
    fn on(&mut self, cmd: &CMD, me: &mut ActorWrapper<CMD, EVENT>);
}

fn compare_box<T: ?Sized>(left: &Box<T>, right: &Box<T>) -> bool {
    let left : *const T = left.as_ref();
    let right : *const T = right.as_ref();
    left == right
}

#[derive(Debug, Clone, PartialEq,Copy)]
struct ClockEntry<CMD> {
    expires_at: Instant,
    interval: Duration,
    cmd: CMD,
    repeat: bool,
}

pub struct ActorWrapper<CMD, EVENT> {
    listeners: Vec<Box<dyn Listener<EVENT>>>, // invoked on EVENT
    cmds_reader: asyncio::Reader<HeapBuffer<CMD>>, // used by actor itself
    cmds_writer: asyncio::Writer<HeapBuffer<CMD>>,
    clock_entries: Vec<ClockEntry<CMD>>,
    next_alarm: Option<ClockEntry<CMD>>,
}

impl<CMD, EVENT> ActorWrapper<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn new(capacity: usize) -> ActorWrapper<CMD, EVENT> {
        let (mut reader, mut writer) = queue(capacity);
        ActorWrapper {
            listeners: Vec::new(),
            cmds_reader: reader,
            cmds_writer: writer,
            clock_entries: Vec::new(),
            next_alarm: None,
        }
    }
    async fn recv(&mut self) -> CMD {
        let mut cmd = [CMD::default()];
        self.cmds_reader.read(&mut cmd).await;
        cmd[0].clone()
    }
    fn add_listener(&mut self, listener: Box<dyn Listener<EVENT>>)  {
        self.listeners.push(listener);
    }
    fn remove_listener(&mut self, listener: &Box<dyn Listener<EVENT>>) {
        self.listeners.retain(|x| compare_box(x,listener)==false);
    }
    pub fn emit(&self, value: &EVENT) {
        for listener in self.listeners.iter() {
            listener.on(value);
        }
    }
    fn on(&mut self, cmd: &CMD) {
        let buf = [cmd.clone()];
        if block_on(self.cmds_writer.write(&buf)) == 0 {
            warn!(" cannot write command ")
        };
    }
    fn has_space(&self) -> bool {
        self.cmds_writer.has_space()
    }
    // set timer to send cmd after delay
    pub fn set_alarm(&mut self, cmd: CMD, clock: Instant) {
        info!("set_alarm {} ", clock.as_millis());
        self.clock_entries.push(ClockEntry {
            expires_at: clock,
            cmd,
            interval: Duration::from_millis(10),
            repeat: false,
        });
        self.next_alarm = self.next_alarm();
    }

    pub fn interval_timer(&mut self, cmd: CMD, interval: Duration) {
        info!("set_interval {} ", interval.as_millis());
        self.clock_entries.push(ClockEntry {
            expires_at: Instant::now() + interval,
            interval,
            cmd,
            repeat: true,
        });
        self.next_alarm = self.next_alarm();
    }


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
            if entry.expires_at <= min {
                info!("next_alarm [{}] {} ", self.clock_entries.len(),entry.expires_at.as_millis());
                min = entry.expires_at;
                clock_entry = Some(entry.clone());
            }
        }
        clock_entry
    }

    fn handle_timeout(&mut self) {
        let mut i = 0;
        while i < self.clock_entries.len() {
            if self.clock_entries[i].expires_at <= Instant::now() {
                let cmd = self.clock_entries[i].cmd.clone();
                self.on(&cmd);
                if self.clock_entries[i].repeat {
                    let interval = self.clock_entries[i].interval;
                    self.clock_entries[i].expires_at += interval;
                } else {
                    self.clock_entries.remove(i);
                }
            } else {
                i += 1;
            }
        }
        self.next_alarm = self.next_alarm();
    }
    // find next alarm entry
    // if clock is in the past, send cmd and remove entry
    // if clock is in the future, wait for clock and repeat
    // if no entry, wait forever
    async fn process_message(&mut self) {
        let cmd = self.recv().await;
        self.on(&cmd);
    }
    async fn run(&mut self) {
        let mut buf = [CMD::default(); 1];
        info!("ActorWrapper::run() next_alarm {}",self.next_alarm.is_some());
        match &self.next_alarm {
            Some(clock_entry) => {
                info!("next alarm in {} ms", clock_entry.expires_at.as_millis());
                let timeout = clock_entry.expires_at - Instant::now();
                let res = with_timeout(timeout, self.process_message()).await;
                match res {
                    Err(TimeoutError) => {
                        self.handle_timeout();
                    }
                    Ok(()) => {}
                }
            }
            None => {
                info!("no alarm");
                self.process_message().await;
            }
        }
    }
}

pub struct ActorRef<CMD, EVENT> {
    actor: Rc<RefCell<Box<dyn Actor<CMD, EVENT>>>>,
    actor_wrapper: Rc<RefCell<ActorWrapper<CMD, EVENT>>>,
}

impl<CMD, EVENT> ActorRef<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    pub fn new(actor: Box<dyn Actor<CMD, EVENT>>, capacity: usize) -> ActorRef<CMD, EVENT> {
        let r = ActorRef {
            actor: Rc::new(RefCell::new(actor)),
            actor_wrapper: Rc::new(RefCell::new(ActorWrapper::new(capacity))),
        };
        r.actor.borrow_mut().init(&mut r.actor_wrapper.borrow_mut());
        r
    }

    pub fn on(&self, cmd: &CMD) {
        self.actor_wrapper.borrow_mut().on(cmd);
    }

    pub async fn run(&self) {
        self.actor_wrapper.borrow_mut().run().await;
    }
}

impl<CMD, EVENT> Listener<CMD> for ActorRef<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn on(&self, value: &CMD) {
        self.on(value);
    }
}

impl<CMD, EVENT> Publisher<EVENT> for ActorRef<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>)  {
        self.actor_wrapper.borrow_mut().add_listener(listener);
    }

    fn remove_listener(&self, listener: &Box<dyn Listener<EVENT>>) {
        self.actor_wrapper.borrow_mut().remove_listener(listener);
    }
    fn emit(&self, value: &EVENT) {
        self.actor_wrapper.borrow().emit(value);
    }
}

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

impl<CMD, EVENT> Clone for ActorRef<CMD, EVENT> {
    fn clone(&self) -> Self {
        ActorRef {
            actor_wrapper: Rc::clone(&self.actor_wrapper),
            actor: Rc::clone(&self.actor),
        }
    }
}

//======================  Handler  ======================

pub struct Mapper<CMD, EVENT> {
    func: fn(&CMD) -> Option<EVENT>,
    listeners: Rc<RefCell<Vec<Box<dyn Listener<EVENT>>>>>,
}

impl<CMD, EVENT> Clone for Mapper<CMD, EVENT> {
    fn clone(&self) -> Self {
        Mapper {
            func: self.func,
            listeners: Rc::clone(&self.listeners),
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
            listeners: Rc::new(RefCell::new(Vec::new())),
        }
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

impl<EVENT, CMD> Publisher<EVENT> for Mapper<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>)  {
        self.listeners.borrow_mut().push(listener);
    }

    fn remove_listener(&self, listener: &Box<dyn Listener<EVENT>>) {
        self.listeners.borrow_mut().retain(|x| compare_box(x,listener)==false);
    }
    fn emit(&self, value: &EVENT) {
        for listener in self.listeners.borrow().iter() {
            listener.on(value);
        }
    }
}
//======================  Actor >> Handler >> ... ======================

impl<'a, T, U, V> Shr<&'a Mapper<U, V>> for &'a ActorRef<T, U>
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
//======================  Handler >> Actor >> ... ======================

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
//====================== Sink ======================

pub struct Sink<CMD> {
    func: fn(&CMD),
}
impl<CMD> Sink<CMD>
where
    CMD: Clone + Default,
{
    pub fn new(func: fn(&CMD)) -> Self {
        Sink { func }
    }
}

impl<CMD> Clone for Sink<CMD> {
    fn clone(&self) -> Self {
        Sink { func: self.func }
    }
}

impl<CMD> Listener<CMD> for Sink<CMD>
where
    CMD: Clone + Default,
{
    fn on(&self, cmd: &CMD) {
        (self.func)(cmd);
    }
}

//======================  Actor >> Sink ======================

impl<'a, T, U> Shr<&'a Sink<U>> for &'a ActorRef<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
{
    type Output = &'a Sink<U>;

    fn shr(self, rhs: &'a Sink<U>) -> Self::Output {
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
