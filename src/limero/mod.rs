use alloc::boxed::Box;
use alloc::vec::Vec;
use core::convert::Infallible;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
//use core::time::Duration;
// use alloc::time::Instant;
use cortex_m::interrupt;
use cortex_m_semihosting::hprintln;
//use core::fmt::Debug;
use alloc::string::String;
use core::borrow::Borrow;
use core::pin::{pin, Pin};

use thingbuf::mpsc::{channel, RecvRef};
use thingbuf::mpsc::{Receiver, Sender};

/*
use lilos::exec::sleep_until;
use lilos::mutex::Mutex;
use lilos::time::TickTime;
use lilos::{create_mutex, create_static_mutex};
 */
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::Timer;

struct PublishMsg {
    topic: String,
    payload: Vec<u8>,
}
struct SubscribeMsg {
    topic: String,
}

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
    pub fn on(&self, item: T) {
        let _ = self.sender.send(item);
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
            senders: Vec::<Sender<T>>::new(),
        }
    }
    pub fn emit(&self, item: T) {
        interrupt::free(|cs| {
            for sender in self.senders.iter() {
                let _ = sender.send(item.clone());
            }
        });
    }
}

use core::ops::Shr;
impl<'a, T: 'static + Default> Shr<&Sink<T>> for &mut Source<T>
where
    T: Clone + Send + 'static,
{
    type Output = ();

    fn shr(self, rhs: &Sink<T>) -> Self::Output {
        self.senders.push(rhs.sender());
    }
}
/*
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
}*/

pub static mut TIMER_SERVER: Option<TimerServer> = None;

pub fn get_timer_server() -> &'static mut TimerServer {
    unsafe {
        if TIMER_SERVER.is_none() {
            TIMER_SERVER = Some(TimerServer::new());
        };
        TIMER_SERVER.as_mut().unwrap()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct TimerMsg {}

#[derive(PartialEq, Debug)]
enum TimerType {
    Interval,
    Gate,
    OneShot,
}

struct TimerEntry {
    kind: TimerType,
    interval: Duration,
    client: Box<Sender<TimerMsg>>,
    repeat: bool,
    expires_at: Instant, // TickTime,
}

pub struct TimerServer {
     clients: Mutex<NoopRawMutex, Vec<TimerEntry>>,
}

impl<'a> TimerServer {
    pub fn new() -> Self {

        //     let mutex = create_static_mutex!(Vec::<TimerEntry>, Vec::<TimerEntry>::new());
        Self { clients: Mutex::<NoopRawMutex, _>::new(Vec::<TimerEntry>::new()) }
    }

    async fn add(&mut self, te: TimerEntry) {
        self.clients.lock().await.push(te);
        /*self.clients
        .perform(|timer_entries: &mut Vec<TimerEntry>| {
            timer_entries.push(te);
        })
        .await*/
    }

    pub async fn new_interval(&mut self, interval: u32, client: Sender<TimerMsg>) {
        let te = TimerEntry {
            kind: TimerType::Interval,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            repeat: true,
            expires_at: Instant::now() + Duration::from_millis(interval as u64),
        };
        self.add(te).await;
    }

    pub async fn new_gate(&mut self, interval: u32, client: Sender<TimerMsg>) {
        hprintln!("TimerServer::new_gate()");
        let te = TimerEntry {
            kind: TimerType::Gate,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            repeat: true,
            expires_at: Instant::now() + Duration::from_millis(interval as u64),
        };
        self.add(te).await;
    }

    pub async fn one_shot(&mut self, interval: u32, client: Sender<TimerMsg>) {
        let te = TimerEntry {
            kind: TimerType::OneShot,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            repeat: false,
            expires_at: Instant::now() + Duration::from_millis(interval as u64),
        };
        self.add(te).await;
    }

    pub fn cancel_all(&mut self) {
        // self.clients.clear();
    }

    pub async fn run(&mut self) -> Infallible {
        loop {
            hprintln!("TimerServer::run() {}", self.clients.lock().await.len());
            let deadline = self.find_next_expiration().await;
            Timer::at(deadline).await;
            let now = Instant::now();
            self.clients.lock().await.iter_mut().for_each(|te| {
                if te.expires_at <= now {
                    if te.client.try_send(TimerMsg {}).is_ok() {
                        if te.repeat && te.kind == TimerType::Interval {
                            te.expires_at = now + te.interval;
                        } else if te.repeat && te.kind == TimerType::Gate {
                            te.expires_at += te.interval;
                        } else {
                            te.expires_at = now + Duration::from_secs(1000000000);
                        }
                    } else {
                        hprintln!("TimerServer::run() client not ready");
                    }
                }
            });
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
    pub async fn find_next_expiration(&mut self) -> Instant {
        let mut exp = Instant::now() + Duration::from_secs(1); // 1 sec default as max start value
                                                               //        hprintln!("TimerServer::find_next_expiration()");
        self.clients.lock().await.iter_mut().for_each(|te| {
            if te.expires_at < exp {
                exp = te.expires_at;
            }
        });
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
