use alloc::boxed::Box;
use alloc::vec::Vec;
use core::time::Duration;
use lilos::time::TickTime;
use lilos::exec::sleep_until;

pub static mut TIME_SERVER: Option<TimerServer>  = None;

pub fn init() {
    unsafe {
        TIME_SERVER = Some(TimerServer::new());
    }
}

pub fn get_time_server() -> &'static mut TimerServer<'static> {
    unsafe { TIME_SERVER.as_mut().unwrap() }
}


pub trait TimeClient {
    fn on_timer(& self, timer_id: u32);
}
pub enum TimeServerMsg {
    WakeMsg,
}

#[derive(Clone, Copy, Debug,PartialEq)]
enum TimerType {
    Interval,
    Gate,
    OneShot,
}

struct TimerEntry<'a> {
    kind: TimerType,
    interval: Duration,
    client: Box<&'a mut dyn TimeClient>,
    timer_id: u32,
    repeat: bool,
    expires_at: TickTime,
}

pub struct TimerServer<'a> {
    clients: Vec<TimerEntry<'a>>,
}

impl<'a> TimerServer<'a> {
    pub fn new() -> Self {
        Self {
            clients: Vec::new(),
        }
    }
    pub fn new_interval(&mut self, interval: u32, client: &'a mut dyn TimeClient) {
        let te = TimerEntry {
            kind: TimerType::Interval,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            timer_id: 0,
            repeat: true,
            expires_at: TickTime::now()+Duration::from_millis(interval as u64),
        };
        self.clients.push(te);
    }

    pub fn new_gate(&mut self, interval: u32, client: &'a mut dyn TimeClient) {
        let te = TimerEntry {
            kind: TimerType::Gate,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            timer_id: 0,
            repeat: true,
            expires_at: TickTime::now()+Duration::from_millis(interval as u64),
        };
        self.clients.push(te);
    }

    pub fn one_shot(&mut self, interval: u32, client: &'a mut dyn TimeClient) {
        let te = TimerEntry {
            kind: TimerType::OneShot,
            interval: Duration::from_millis(interval as u64),
            client: Box::new(client),
            timer_id: 0,
            repeat: false,
            expires_at: TickTime::now()+Duration::from_millis(interval as u64),
        };
        self.clients.push(te);
    }

    pub fn cancel(&mut self, timer_id: u32) {
        self.clients.retain(|te| te.timer_id != timer_id);
    }

    pub fn cancel_all(&mut self) {
        self.clients.clear();
    }

    pub async fn run(&mut self) {
        loop {
            let deadline = self.find_next_expiration();  
            sleep_until(deadline).await;
            let now = TickTime::now();
            for te in self.clients.iter_mut() {
                if te.expires_at < now {
                    te.client.on_timer(te.timer_id);
                    if te.repeat && te.kind == TimerType::Interval {
                        te.expires_at = now + te.interval;
                    } else if te.repeat && te.kind == TimerType::Gate {
                        te.expires_at += te.interval;
                    } else {
                        te.expires_at = now + Duration::from_secs(1000000000);
                    }
                }
            }
        }
    }

    // search all timerentries for the next expiration time
    pub fn find_next_expiration(&mut self) -> TickTime{
        let mut exp = TickTime::now()+Duration::from_secs(1);
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
