use core::convert::Infallible;
use embedded_hal::digital::v1::OutputPin;
use thingbuf as conn;
use alloc::vec::Vec;

trait TimeClient {
    fn on_timer(&mut self,timer_id: u32);
}
enum TimeServerMsg {
    WakeMsg,
    Interval(u32),
    Gate(u32),
}

struct TimerServer<'a> {
    clients : Vec<&'a mut dyn TimeClient>,
}

impl<'a> TimerServer<'a> {
    fn new_interval(&mut self, interval: u32,client:&'a mut dyn TimeClient) {
        self.clients.push(client);
    }

    fn new_gate(&mut self, interval: u32,client:&'a mut dyn TimeClient) {
        self.clients.push(client);
    }

    fn one_shot(&mut self, interval: u32,client:&'a mut dyn TimeClient) {
        self.clients.push(client);
    }
}

#[derive(Clone, Debug)]
pub enum LedMsg {
    On,
    Off,
    Interval(u32),
    BlinkTimer,
}

impl Default for LedMsg {
    fn default() -> Self {
        LedMsg::On
    }
}

pub struct Led<'a> {
    pin: &'a mut dyn OutputPin,
    led_state: bool,
    recv_main: conn::mpsc::Receiver<LedMsg>,
    send_main: conn::mpsc::Sender<LedMsg>,
    active: bool,
}

impl TimeClient for Led<'_> {
    fn on_timer(&mut self, timer_id: u32) {
        let _ = self.send_main.send(LedMsg::BlinkTimer);
    }
}

impl<'a> Led<'a> {
    pub fn new(pin: &'a mut dyn OutputPin) -> Self {
        let (send_main, recv_main) = conn::mpsc::channel::<LedMsg>(10);
        Self {
            pin,
            led_state: false,
            send_main,
            recv_main,
            active: true,
        }
    }

    pub fn on(&mut self, msg: LedMsg) {
        let _ = self.send_main.send(msg);
    }

    fn toggle(&mut self ){
        if self.active {
            if self.led_state {
                self.pin.set_low();
                self.led_state = false;
            } else {
                self.pin.set_high();
                self.led_state = true;
            }
        }
    }

    fn set_active(&mut self, active: bool) {
        self.active = active;
        self.led_state=false;
        self.pin.set_low();
    }

    pub async fn run(&mut self) -> Infallible {
        loop {
            let msg = self.recv_main.recv().await;
            if let Some(m) = msg {
                match m {
                    LedMsg::On => self.set_active(true),
                    LedMsg::Off => self.set_active(false),
                    LedMsg::BlinkTimer => self.toggle(),
                    LedMsg::Interval(ms) => {}
                }
            }
        }
    }
}
