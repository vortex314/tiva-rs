use alloc::sync::Arc;
use core::borrow::BorrowMut;
use core::cell::RefCell;
use core::convert::Infallible;
use cortex_m::interrupt::free;
use cortex_m::interrupt::Mutex;
use embedded_hal::digital::v2::OutputPin;

use core::pin::pin;
use futures::select_biased;
use futures::FutureExt;
use lilos;
use lilos::exec::PeriodicGate;
use thingbuf as conn;
use tm4c123x_hal as hal;
use tm4c123x_hal::gpio::Output;
use tm4c123x_hal::gpio::PushPull;
pub struct Led {
    pin: hal::gpio::gpiof::PF1<Output<PushPull>>,
    gate: PeriodicGate,
    recv_cmd: conn::mpsc::Receiver<bool>,
    send_cmd: conn::mpsc::Sender<bool>,
    active: bool,
}
impl Led {
    pub fn new(pin: hal::gpio::gpiof::PF1<Output<PushPull>>) -> Self {
        let (send_cmd, recv_cmd) = conn::mpsc::channel::<bool>(10);
        let gate = PeriodicGate::from(lilos::time::Millis(500));
        Self {
            pin,
            gate,
            send_cmd,
            recv_cmd,
            active: true,
        }
    }

    async fn cmd(&mut self, v: bool) {
        let _ = self.send_cmd.send(v).await;
    }

    async fn blink(&mut self) -> Infallible {
        loop {
            self.gate.next_time().await;
            let _ = self.pin.set_high();
            self.gate.next_time().await;
            let _ = self.pin.set_low();
        }
    }
    async fn on_cmd(&mut self) -> Infallible {
        loop {
            let cmd = self.recv_cmd.recv().await;
            if let Some(x) = cmd {
                self.active = x;
            }
        }
    }
    pub async fn run(&mut self) -> Infallible {
  //      loop {
        self.blink().await
  //      self.on_cmd().await;
  //      }
    }
}
