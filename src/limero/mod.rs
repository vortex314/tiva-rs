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

use core::ops::Shr;
use core::{cell::RefCell, mem};

use embassy_futures::select::*;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel, pubsub::publisher::Pub};
use embassy_time::{with_timeout, Duration, Instant, TimeoutError, Timer};
use log::{info, warn};
use mini_io_queue::asyncio;
use mini_io_queue::asyncio::queue;
use mini_io_queue::storage::HeapBuffer;

// led.actor.timers.set_interval(LedCmd::TimerBlink, Duration::from_millis(1000));
// led.actor.timers.set_alarm(LedCmd::TimerBlink, Instant::now() + Duration::from_millis(1000));
// actorref = Rc<RefCell< Publisher + Listener >>
// Sink = Listener
// Source = Publisher
// Led = { Sink }, Button = { Source }
// a >> func >> b
// actor.timer_scheduler.set_interval(LedCmd::TimerBlink, Duration::from_millis(1000));
// led.handle(LedCmd::Blink(200))
// Led : Listener<LedCmd>, Publisher<LedEvent>
// button.event >> transform(|x| { Pressed => Blink(100)}) >> led.cmd
/*
use alloc::string::{String, ToString};
use serde::Serialize;
use serde::Deserialize;
enum MqttCmd<'a> {
    Publish(String,String),
    Rxd(dyn Deserialize<'a>),
    Connect,
    Disconnect,
}

enum MqttEvent {
    Publish(String,String),
    Txd(dyn Serialize),
    Connected,
    Disconnected,
}

enum MqttTimer {
    LoopbackTimer
}
struct MqttActor<'a> { // }: Listener<MqttCmd> + Publisher<MqttEvent> + Listener<MqttTimer> {
    cmd : dyn Listener<MqttCmd<'a>>,
    timer : dyn Listener<MqttTimer>,
    event : dyn Publisher<MqttEvent>
}

impl Listener<MqttCmd<'_>> for MqttActor {
    fn on(&mut self,cmd:MqttCmd) {
        match cmd {
            MqttCmd::Publish(topic,value) => { self.emit(["pub",topic.to_string(),value.to_string()])},
            _ => {}
        }
    }
}

//

trait Actor<CMD,EVENT,TIMER_EVENT> : Listener<CMD> + Publisher<EVENT> + Listener<TIMER_EVENT> {
    fn init(&mut self,wrapper : &mut ActorWrapper<CMD, EVENT,TIMER_EVENT>);
    fn on(&mut self, cmd: &CMD,wrapper : &mut ActorWrapper<CMD, EVENT,TIMER_EVENT>) ;
    fn on_timer(&mut self, cmd: &TIMER_EVENT,wrapper : &mut ActorWrapper<CMD, EVENT,TIMER_EVENT>) ;
}
*/

use nb::block;
//use tm4c123x_hal::pwm::Timer;

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
    fn remove_listener(&self, listener: &Box<dyn Listener<T>>);
    fn emit(&self, value: &T);
}
pub trait Actor<CMD, EVENT> {
    fn init(&mut self, wrapper: &mut TimerScheduler<CMD>);
    fn on(&mut self, cmd: &CMD, scheduler: &mut TimerScheduler<CMD>,publisher:&mut dyn Publisher<EVENT>);
}

fn compare_box<T: ?Sized>(left: &Box<T>, right: &Box<T>) -> bool {
    let left: *const T = left.as_ref();
    let right: *const T = right.as_ref();
    left == right
}

#[derive(Debug, Clone, PartialEq, Copy)]
struct ClockEntry<CMD> {
    expires_at: Instant,
    interval: Duration,
    cmd: CMD,
    repeat: bool,
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
        timeouts
    }
}

pub struct ActorWrapper<CMD, EVENT> {
    actor: Option<Box<dyn Actor<CMD, EVENT>>>,
    listeners: Vec<Box<dyn Listener<EVENT>>>, // invoked on EVENT
    channel: channel::Channel<NoopRawMutex, CMD, 3>,
    pub timer_scheduler: TimerScheduler<CMD>,
}

impl<CMD, EVENT> ActorWrapper<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn new(actor: Box<dyn Actor<CMD, EVENT>>, capacity: usize) -> ActorWrapper<CMD, EVENT> {
        // let (mut reader, mut writer) = queue(capacity);
        let channel = channel::Channel::<NoopRawMutex, CMD, 3>::new();
        let timer_scheduler = TimerScheduler::<CMD>::new();
        ActorWrapper {
            actor: Some(actor),
            listeners: Vec::new(),
            //        cmds_reader: r,
            //        cmds_writer: s,
            channel,
            timer_scheduler: TimerScheduler::new(),
        }
    }

    fn init(&mut self) {
        self.actor.unwrap().init(&mut self.timer_scheduler);
    }

    async fn recv(&mut self) -> CMD {
        self.channel.receive().await
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
        let cmd = self.recv().await;
        let mut actor = self.actor.take().unwrap();
        actor.on(&cmd, &mut self.timer_scheduler,&mut self);
    }
    async fn run(&mut self) {
        let mut buf = [CMD::default(); 1];
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
        self.tell(value);
    }
}

impl<CMD, EVENT> Publisher<EVENT> for ActorRef<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>) {
        self.actor_wrapper.borrow_mut().add_listener(listener);
    }

    fn remove_listener(&self, listener: &Box<dyn Listener<EVENT>>) {
        self.actor_wrapper.borrow_mut().remove_listener(listener);
    }
    fn emit(&self, value: &EVENT) {
        self.actor_wrapper.borrow().emit(value);
    }
}

impl<'a, T, U, V> Shr<ActorRef<U, V>> for ActorRef<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    V: Clone + Default + 'static,
{
    type Output = ActorRef<U, V>;

    fn shr(self, rhs: ActorRef<U, V>) -> Self::Output {
        self.add_listener(Box::new(rhs.clone()));
        rhs
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
    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>) {
        self.listeners.borrow_mut().push(listener);
    }

    fn remove_listener(&self, listener: &Box<dyn Listener<EVENT>>) {
        self.listeners
            .borrow_mut()
            .retain(|x| compare_box(x, listener) == false);
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

pub struct Sink<F, CMD>
where
    F: Fn(&CMD),
{
    func: Box<F>,
    phantom: core::marker::PhantomData<CMD>,
}
impl<F, CMD> Sink<F, CMD>
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

/*impl<CMD> Clone for Sink<CMD> {
    fn clone(&self) -> Self {
        Sink { func: self.func }
    }
}*/

impl<F, CMD> Listener<CMD> for Sink<F, CMD>
where
    CMD: Clone + Default,
    F: Fn(&CMD),
{
    fn on(&self, cmd: &CMD) {
        (self.func)(cmd);
    }
}

//======================  Actor >> Sink ======================

impl<'a, T, U, F> Shr<Sink<F, U>> for &'a ActorRef<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    F: Fn(&U) + 'static,
{
    type Output = ();

    fn shr(self, rhs: Sink<F, U>) -> Self::Output {
        self.add_listener(Box::new(rhs));
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
