use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
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
}

impl Led {
    pub fn new() -> Self {
        Led {
            state: Rc::new(RefCell::new(false)),
            actor: Actor::new(10),
        }
    }
    pub async fn run(&mut self) {
        loop {
            let cmd = self.actor.recv().await;
            info!("received event {:?}", cmd);
            match cmd {
                LedCmd::On => {}
                LedCmd::Off => {}
                LedCmd::Blink(t) => {}
            }
        }
    }
}
/*
impl Listener<LedCmd> for Led {
    fn on(&self, value: &LedCmd) {
        self.actor.on(value);
    }
}*/