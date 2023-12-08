use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::sync::Arc;
use embassy_time::Instant;
use embedded_hal::digital::OutputPin;
use futures::Future;

use core::borrow::BorrowMut;
use core::cell::RefCell;
use core::pin::Pin;
use core::task::Context;
use core::task::Poll;
use core::task::Waker;
use embassy_time::Duration;
use embassy_time::Timer;
//use embassy_futures::join::join;
use embassy_futures::select::select;
use log::info;

#[derive(Debug, Clone, Default)]
pub enum LedCmd {
    #[default]
    On,
    Off,
    Blink(u32),
    TimerBlink,
}
pub struct Led {
    state: LedCmd,
    interval_ms: u64,
    pin: Box<dyn OutputPin>,
    pin_high: bool,
    scheduler: Option<Rc<TimerScheduler<LedCmd>>>,
}

impl Led {
    pub fn new(pin: impl OutputPin + 'static) -> Self {
        Led {
            state: LedCmd::On,
            interval_ms: 100,
            pin: Box::new(pin),
            pin_high: false,
            scheduler:None,
        }
    }
}

impl Actor<LedCmd, NoEvent> for Led {
    fn init(&mut self, scheduler: Rc<TimerScheduler<LedCmd>>) {
        info!("Led init");
        self.pin.set_high();
        self.pin_high = true;
        self.state = LedCmd::On;
        self.scheduler = Some(Rc::clone(&scheduler));
        self.scheduler.unwrap().borrow_mut().interval_timer(LedCmd::TimerBlink, Duration::from_millis(1000));

    }
    fn on(
        &mut self,
        cmd: &LedCmd,
        scheduler: &mut TimerScheduler<LedCmd>,
        publisher: &mut Emitter<NoEvent>,
    ) {
        info!("Led cmd {:?}", cmd);
        self.state = cmd.clone();
        match cmd {
            LedCmd::On => {
                self.pin.set_high();
                self.pin_high = true;
            }
            LedCmd::Off => {
                self.pin.set_low();
                self.pin_high = false;
            }
            LedCmd::Blink(intv) => {
                scheduler.set_alarm(
                    LedCmd::TimerBlink,
                    Instant::now() + Duration::from_millis(*intv as u64),
                );
                self.interval_ms = *intv as u64;
            }
            LedCmd::TimerBlink => {
                if let LedCmd::Blink(x) = self.state {
                    if self.pin_high {
                        self.pin.set_low();
                        self.pin_high = false;
                    } else {
                        self.pin.set_high();
                        self.pin_high = true;
                    }
                }
            }
        }
    }

    async fn wait_for_cmd() -> LedCmd {
        unsafe {
            let waker = WAKER.as_ref().unwrap().clone();
            let mut led = Led::new();
            let mut led = Pin::new_unchecked(&mut led);
            let mut ctx = Context::from_waker(&waker);
            loop {
                if let Poll::Ready(cmd) = led.as_mut().poll(&mut ctx) {
                    return cmd;
                }
                cortex_m::asm::wfi();
            }
        }
    }

}

static mut WAKER : Option<Waker> = None;

impl Future for Led {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        unsafe { WAKER = Some(cx.waker().clone());};
        let mut this = self.get_mut();

            Poll::Pending
    }
}
