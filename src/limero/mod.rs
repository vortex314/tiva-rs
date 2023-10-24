use alloc::boxed::Box;
use alloc::vec::Vec;
use cortex_m::interrupt;
use lilos::create_mutex;
use core::convert::Infallible;
use core::time::Duration;
use cortex_m_semihosting::hprintln;
//use core::fmt::Debug;
use lilos::exec::sleep_until;
use lilos::time::TickTime;
use lilos::mutex::Mutex;
use core::pin::Pin;
use thingbuf::mpsc::{channel, RecvRef};
use thingbuf::mpsc::{Receiver, Sender};
use core::borrow::Borrow;

pub struct Sink<T: Default + Clone> {
    receiver: Receiver<T>,
    sender: Sender<T>,
}

impl<T: Default + Clone> Sink<T> {
    pub fn new(size: usize) -> Self {
        let (sender, receiver) = channel(size);
        Sink { receiver, sender }
    }
    pub async fn recv_ref(&mut self) -> RecvRef<T> {
        self.receiver.recv_ref().await.unwrap()
    }
    pub async fn recv(&mut self) -> T {
        self.receiver.recv().await.unwrap()
    }
    pub fn sender(&self) -> Sender<T> {
        self.sender.clone()
    }
}

pub struct Source<T> {
    senders: Vec<Sender<T>>,
}

impl<T: Default + Clone> Source<T>
where
    T: Clone + Send + 'static,
{
    pub fn new() -> Self {
        Source {
            senders:Vec::<Sender<T>>::new(),
        }
    }
    /*async fn emit_async(&mut self, item: T) -> Result<(), SendError<T>> {
        for sender in self.senders.iter() {
            sender.send_async(item.clone()).await.unwrap()  ;
        }
        Ok(())
    }*/
    pub fn emit(&self, item: T) {
        interrupt::free(|cs| {
           // let mut senders = *self.senders.lock();
            for sender in self.senders.iter() {
                let _ = sender.send(item.clone());
            }
        });
    }
}

struct TimerSource {
    source: Source<()>,
    interval: Duration,
    active: bool,
}

impl TimerSource {
    fn new() -> Self {
        TimerSource {
            source: Source::new(),
            interval: Duration::from_secs(1),
            active: false,
        }
    }
    fn interval(&mut self, interval: Duration) -> &mut Self {
        self.interval = interval;
        self
    }
    async fn run(&mut self) -> () {
        loop {
            let _ = self.source.emit(());
            //           tokio::time::sleep(self.interval).await;
        }
    }
}

pub static mut TIMER_SERVER: Option<TimerServer> = None;

pub fn get_timer_server() -> &'static mut TimerServer {
    unsafe {
        if TIMER_SERVER.is_none() {
            TIMER_SERVER = Some(TimerServer::new());
        };
        TIMER_SERVER.as_mut().unwrap()
    }
}

pub trait TimerClient {
    fn on_timer(&self, timer_id: u32);
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct TimerMsg {}

#[derive(PartialEq, Debug)]
enum TimerType {
    Interval,
    Gate,
    OneShot,
}

#[derive(Debug)]
struct TimerEntry {
    kind: TimerType,
    interval: Duration,
    client: Box<Sender<TimerMsg>>,
    repeat: bool,
    expires_at: TickTime,
}

pub struct TimerServer {
    clients: Pin<&Mutex<Vec<TimerEntry>>>,
}

impl TimerServer {
    pub fn new() -> Self {
        create_mutex!( clients, Vec::<TimerEntry>::new());
        Self {
            clients
        }
    }

    fn add(&mut self,te:TimerEntry ) {
        interrupt::free(|cs| {
            self.clients.perform(|x| {
                x.push(te);
            });
        });
    }

    pub fn new_interval(&mut self, interval: u32, client: Sender<TimerMsg>) {
        let te = TimerEntry {
            kind: TimerType::Interval,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            repeat: true,
            expires_at: TickTime::now() + Duration::from_millis(interval as u64),
        };
        self.add(te);
    }

    pub fn new_gate(&mut self, interval: u32, client: Sender<TimerMsg>) {
        let te = TimerEntry {
            kind: TimerType::Gate,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            repeat: true,
            expires_at: TickTime::now() + Duration::from_millis(interval as u64),
        };
        self.add(te);
    }

    pub fn one_shot(&mut self, interval: u32, client: Sender<TimerMsg>) {
        let te = TimerEntry {
            kind: TimerType::OneShot,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            repeat: false,
            expires_at: TickTime::now() + Duration::from_millis(interval as u64),
        };
        self.add(te);
    }

    pub fn cancel_all(&mut self) {
        // self.clients.clear();
    }


    pub async fn run(&mut self) -> Infallible {
        loop {
            let deadline = self.find_next_expiration();
            sleep_until(deadline).await;
            let mut now = TickTime::now();
            interrupt::free(|cs| {
                self.clients.perform(|timer_entries: &mut Vec<TimerEntry>| {
                    for te in timer_entries.iter_mut() {
                        if te.expires_at <= now {
                            te.client.try_send(TimerMsg {}).expect(" channel overflow ");
                            if te.repeat && te.kind == TimerType::Interval {
                                te.expires_at = now + te.interval;
                            } else if te.repeat && te.kind == TimerType::Gate {
                                te.expires_at += te.interval;
                            } else {
                                te.expires_at = now + Duration::from_secs(1000000000);
                            }
                        }
                    }
                });
            })
            /* 
            for te in self.clients.iter_mut() {
                if te.expires_at <= now {
                    let _r = te.client.send(TimerMsg {}).await.unwrap();
                    if te.repeat && te.kind == TimerType::Interval {
                        te.expires_at = now + te.interval;
                    } else if te.repeat && te.kind == TimerType::Gate {
                        te.expires_at += te.interval;
                    } else {
                        te.expires_at = now + Duration::from_secs(1000000000);
                    }
                }
            }*/
        }
    }

    // search all timerentries for the next expiration time
    pub fn find_next_expiration(&mut self) -> TickTime {
        let mut exp = TickTime::now() + Duration::from_secs(1); // 1 sec default as max start
        for te in self.clients.iter_mut() {
            if te.expires_at < exp {
                exp = te.expires_at;
            }
        }
        exp
    }
}

/*
struct Frame<'a> {
    buffer: &'a mut [u8],
    index: usize,
    max: usize,
    error: Option<Error>,
}

impl Frame {
    fn new(buffer: &mut [u8]) -> Frame {
        Frame { buffer, max:buffer.capacity(),index: 0,None }
    }
    fn sequence_start(&mut self) -> &mut Self {
        self.write(0x7e);
        self
    }
    fn sequence_end(&mut self) -> &mut Self {
        self.buffer[self.index] = 0x7e;
        self.index += 1;
        self
    }

    fn write(&mut self,b:u8) -> &mut Self {
        if self.error.is_some() {
            return self;
        }
        self.buffer[self.index] = b;
        self.index += 1;
        if self.index >= self.max {
            self.error = Some(Error::BufferOverflow);
        }
        self
    }

    fn frame_begin(&mut self) -> Frame {
        self.write(0x7e)
    }

    fn frame_end(&mut self) -> &mut Self {
        self.buffer[self.index] = 0x7e;
        let crc = crc16::State::<crc16::XMODEM>::calculate(&self.buffer[1..self.index]);
        self.write_u16(crc);
        self
    }


}

#[cfg(test)]

fn test_serialization() {

}

*/
