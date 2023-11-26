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

use core::cell::RefCell;
use core::ops::Shr;

use log::warn;
use mini_io_queue::asyncio;
use mini_io_queue::asyncio::queue;
use mini_io_queue::storage::HeapBuffer;

pub trait Listener<T> {
    fn on(&self, value: &T);
}
pub trait Publisher<T> {
    fn add_listener(&self, listener: Box<dyn Listener<T>>) -> usize;
    fn remove_listener(&self, listener_id: usize);
    fn emit(&self, value: &T);
}
pub struct Actor<CMD, EVENT> {
    cmds_reader: Rc<RefCell<asyncio::Reader<HeapBuffer<CMD>>>>, // used by actor itself
    cmds_writer: Rc<RefCell<asyncio::Writer<HeapBuffer<CMD>>>>, // used by external party talking to actor
    listeners: Rc<RefCell<Vec<Box<dyn Listener<EVENT>>>>>,      // used by actor itself
}
impl<CMD, EVENT> Clone for Actor<CMD, EVENT> {
    fn clone(&self) -> Self {
        Actor {
            cmds_reader: Rc::clone(&self.cmds_reader),
            cmds_writer: Rc::clone(&self.cmds_writer),
            listeners: Rc::clone(&self.listeners),
        }
    }
}
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
impl<CMD, EVENT> Actor<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    pub fn new(capacity: usize) -> Self {
        let (mut reader, mut writer) = queue(capacity);
        Actor {
            cmds_reader: Rc::new(RefCell::new(reader)),
            cmds_writer: Rc::new(RefCell::new(writer)),
            listeners: Rc::new(RefCell::new(Vec::new())),
        }
    }
    pub async fn recv(&self) -> CMD {
        let mut cmd = [CMD::default()];
        self.cmds_reader.borrow_mut().read(&mut cmd).await;
        cmd[0].clone()
    }
}

impl<CMD, EVENT> Listener<CMD> for Actor<CMD, EVENT>
where
    CMD: Clone + Default,
{
    fn on(&self, cmd: &CMD) {
        if self.cmds_writer.borrow().has_space() {
            let buf = [cmd.clone()];
            let fut = async {
                if self.cmds_writer.borrow_mut().write(&[cmd.clone()]).await != 1 {
                    warn!("no reader ");
                }
            };
            #[cfg(feature = "tokio")]
            block_in_place(|| {
                futures::executor::block_on(fut);
            });
            #[cfg(feature = "embassy")]
            block_on(fut);
        } else {
            warn!("no space in actor queue");
        }
    }
}

impl<EVENT, CMD> Publisher<EVENT> for Actor<CMD, EVENT>
where
    EVENT: Clone,
{
    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>) -> usize {
        self.listeners.borrow_mut().push(listener);
        self.listeners.borrow().len() - 1
    }
    fn remove_listener(&self, listener_id: usize) {
        self.listeners.borrow_mut().remove(listener_id);
    }
    fn emit(&self, value: &EVENT) {
        for listener in self.listeners.borrow().iter() {
            listener.on(value);
        }
    }
}
pub struct Flow<EVENT, CMD> {
    pub actor: Actor<EVENT, CMD>,
    func: fn(&EVENT) -> CMD,
}
impl<EVENT, CMD> Flow<EVENT, CMD>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    pub fn new(func: fn(&EVENT) -> CMD) -> Self {
        Flow {
            actor: Actor::new(5),
            func,
        }
    }
    pub async fn run(&mut self) {
        loop {
            let event = self.actor.recv().await;
            let cmd = (self.func)(&event);
            self.actor.emit(&cmd);
        }
    }
}

impl<'a, T, U, V> Shr<&'a Actor<U, V>> for &'a Actor<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    V: Clone + Default + 'static,
{
    type Output = &'a Actor<U, V>;

    fn shr(self, rhs: &'a Actor<U, V>) -> Self::Output {
        self.add_listener(Box::new(rhs.clone()));
        rhs
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
    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>) -> usize {
        self.listeners.borrow_mut().push(listener);
        self.listeners.borrow().len() - 1
    }
    fn remove_listener(&self, listener_id: usize) {
        self.listeners.borrow_mut().remove(listener_id);
    }
    fn emit(&self, value: &EVENT) {
        for listener in self.listeners.borrow().iter() {
            listener.on(value);
        }
    }
}
//======================  Actor >> Handler >> ... ======================

impl<'a, T, U, V> Shr<&'a Mapper<U, V>> for &'a Actor<T, U>
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

impl<'a, T, U, V> Shr<&'a Actor<U, V>> for &'a Mapper<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    V: Clone + Default + 'static,
{
    type Output = &'a Actor<U, V>;

    fn shr(self, rhs: &'a Actor<U, V>) -> Self::Output {
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

impl<'a, T, U> Shr<&'a Sink<U>> for &'a Actor<T, U>
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
