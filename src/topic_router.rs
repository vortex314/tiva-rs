use crate::limero::*;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::String;
use alloc::vec::Vec;
use embassy_time::Instant;
use embedded_hal::digital::OutputPin;
use serde::de;

use core::any::Any;
use core::cell::RefCell;
use embassy_time::Duration;
use embassy_time::Timer;
//use embassy_futures::join::join;
use embassy_futures::select::select;
use log::info;

pub enum TopicRouterCmd {
    Subscribe(String, Box<dyn Fn(&str)>),
}
struct RouterEntry {
    topic: String,
    func: Box<dyn Fn(&str)>,
}
pub struct TopicRouter {
    entries : Vec<RouterEntry>,
}

impl TopicRouter {
    pub fn new() -> Self {
        TopicRouter {
            entries: Vec::new(),
        }
    }
}

impl TopicRouter {
    fn init(&mut self, wrapper: &mut ActorWrapper<TopicRouterCmd, NoEvent>) {
        
    }
    fn on(&mut self, cmd: &TopicRouterCmd, _me: &mut ActorWrapper<TopicRouterCmd, NoEvent>) {
        match cmd {
            TopicRouterCmd::Subscribe(topic,func_pointer) => {
    //            self.entries.push(RouterEntry{topic:topic.clone(),func:*func_pointer});
            }
        }
    }
}
