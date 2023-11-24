use crate::limero::*;
use alloc::rc::Rc;
use core::cell::RefCell;
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
}
pub struct Led {
    pub actor: Actor<LedCmd, NoEvent>,
    state: Rc<RefCell<bool>>,
    interval_ms: Rc<RefCell<u64>>,
}

impl Led {
    pub fn new() -> Self {
        Led {
            state: Rc::new(RefCell::new(false)),
            actor: Actor::new(10),
            interval_ms: Rc::new(RefCell::new(1000)),
        }
    }
    pub async fn run(&mut self) {
        select(
            async {
                loop {
                    let interval = Duration::from_millis(*self.interval_ms.borrow());
                    Timer::after(interval).await;
                    self.state.replace(true);
                    Timer::after(interval).await;
                    self.state.replace(false);
                }
            },
            async {
                loop {
                    let cmd = self.actor.recv().await;
                    info!("received event {:?}", cmd);
                    match cmd {
                        LedCmd::On => {
                            self.state.replace(true);
                        }
                        LedCmd::Off => {
                            self.state.replace(false);
                        }
                        LedCmd::Blink(t) => {
                            self.interval_ms.replace(t as u64) ;
                        }
                    }
                }
            },
        )
        .await;
    }
}
/*
impl Listener<LedCmd> for Led {
    fn on(&self, value: &LedCmd) {
        self.actor.on(value);
    }
}*/
