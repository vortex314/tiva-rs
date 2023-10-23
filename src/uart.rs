use core::convert::Infallible;
use embedded_hal::digital::v1::OutputPin;
use futures::select_biased;
use crate::limero::{Sink,TimerClient, get_timer_server,TimerMsg};
use futures::FutureExt;
use crate::limero::Source;
use embedded_hal::serial;

const FRAME_DELIMITER: u8 = 0x7e;

pub struct Uart<'a> {
    reader: &'a mut dyn serial::Read<u8,Error=u32>,
    writer: &'a mut dyn serial::Write<u8,Error = u32>,
    timer_tick:Sink<TimerMsg> ,
    txd_sink: Sink<&'a [u8]>,
    rxd_source: Source<&'a [u8]>,
}



impl<'a> Uart<'a> {
    pub fn new(reader : &mut  dyn  serial::Read<u8,Error = u32>,writer:&mut dyn serial::Write<u8,Error = u32>) -> Self {     
        Self {
            reader,
            writer,
            timer_tick: Sink::<TimerMsg>::new(10),
            txd_sink: Sink::<&[u8]>::new(10),
            rxd_source: Source::<&[u8]>::new(),
        }
    }


    pub async fn run(&mut self) -> Infallible {
        get_timer_server().new_gate(100,  self.timer_tick.sender());
        loop {
            select_biased! {
                msg = self.txd_sink.recv().fuse() => {
                    self.writer.write(msg);
                }
                _ = self.timer_tick.recv().fuse() => {
                }
            }
        }   
    }
}
