use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use embedded_hal::digital::OutputPin;

use core::cell::RefCell;
use embassy_time::Duration;
use embassy_time::Timer;
//use embassy_futures::join::join;
use crate::timer_driver::msec;
use embassy_futures::select::select;
use log::info;

#[derive(Debug, Clone, Default)]
pub enum EchoCmd {
    #[default]
    EchoEmpty,
    EchoMsg(u32, u64),
}

struct EchoState {
    pub max_count: u32,
    pub start_time: u64,
}

pub struct Echo {
    pub actor: Actor<EchoCmd, EchoCmd>,
    state: Rc<RefCell<EchoState>>,
}

impl Echo {
    pub fn new(max_count: u32) -> Self {
        Echo {
            state: Rc::new(RefCell::new(EchoState {
                max_count,
                start_time: msec(),
            })),
            actor: Actor::new(10),
        }
    }
    pub async fn run(&mut self) {
        loop {
            select(
                async {
                        Timer::after(Duration::from_millis(3000)).await;
                        info!(
                            "Echo start for {} max_count ",
                            self.state.borrow().max_count
                        );
                        self.actor.emit(&EchoCmd::EchoMsg(0, msec()));
                },
                async {
                        let cmd = self.actor.recv().await;
                        match cmd {
                            EchoCmd::EchoMsg(count, start) => {
                                if count < self.state.borrow().max_count {
                                    self.actor.emit(&EchoCmd::EchoMsg(count + 1, start));
                                    Timer::after(Duration::from_millis(1)).await;
                                } else {
                                    let now = msec();
                                    info!("Echo {} ms", now - start);
                                }
                            }
                            _ => {}
                        }
                },
            ).await;
        }
        
    }
}
