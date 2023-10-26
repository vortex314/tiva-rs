use core::convert::Infallible;
use futures::select_biased;
use crate::limero::{Sink, get_timer_server,TimerMsg};
use futures::FutureExt;
use crate::limero::Source;
use embedded_hal::serial;
use embedded_hal::serial::Read as Rx;


const FRAME_DELIMITER: u8 = 0x7e;

use alloc::vec::Vec;
use void::Void;

pub struct Uart<'a> {
    reader: &'a mut dyn serial::Read<u8,Error=Void>,
    writer: &'a mut dyn serial::Write<u8,Error = Void>,
    timer_tick:Sink<TimerMsg> ,
    pub txd_sink: Sink<Vec<u8>>,
    pub rxd_source: Source<Vec<u8>>,
}



impl<'a> Uart<'a> {
    pub fn new(reader : &'a mut dyn Rx<u8,Error=Void>,writer:&'a mut dyn serial::Write<u8,Error = Void>) -> Self {     
        Self {
            reader,
            writer,
            timer_tick: Sink::<TimerMsg>::new(10),
            txd_sink: Sink::<Vec<u8>>::new(10),
            rxd_source: Source::<Vec<u8>>::new(),
        }
    }


    pub async fn run(&mut self) -> Infallible {
        get_timer_server().new_gate(100,  self.timer_tick.sender()).await;
        loop {
            select_biased! {
                msg = self.txd_sink.recv().fuse() => {
                    for b in msg {
                        let _ = self.writer.write(b);
                    }
                }
                _ = self.timer_tick.recv().fuse() => {
                }
            }
        }   
    }
}
