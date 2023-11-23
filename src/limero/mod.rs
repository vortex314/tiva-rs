
#[cfg(all(feature = "std", feature = "no_std"))]
compile_error!("feature \"std\" and feature \"no_std\" cannot be enabled at the same time");

#[cfg(feature = "std")]
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

#[cfg(feature = "no_std")]
use {
    alloc::boxed::Box, 
    alloc::rc::Rc, 
    alloc::vec::Vec, 
    embassy_futures::block_on,
};

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
    CMD: Clone +  Default,
{
    fn on(&self, cmd: &CMD) {
        if self.cmds_writer.borrow().has_space() {
            let buf = [cmd.clone()];
#[cfg(feature = "std")]
            block_in_place(|| {
                futures::executor::block_on(self.cmds_writer.borrow_mut().write(&buf));
            });
#[cfg(feature = "no_std")]
            block_on(async {
                let rc = self.cmds_writer.borrow_mut().write(&[cmd.clone()]).await;
            });
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

type Rhs<T> = Box<dyn Listener<T>>;
type Lhs<T> = Box<dyn Publisher<T>>;

impl<T> Shr<Rhs<T>> for Lhs<T> {
    type Output = usize;

    fn shr(self, rhs: Rhs<T>) -> Self::Output {
        self.add_listener(rhs)
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