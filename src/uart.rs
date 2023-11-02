use core::convert::Infallible;
use futures::select_biased;
use crate::limero::{Sink,};
use futures::FutureExt;
use crate::limero::Source;
use embedded_hal::serial;
use embedded_hal::serial::Read as Rx;
use cortex_m_semihosting::hprintln;

const FRAME_DELIMITER: u8 = '\n' as u8;

use alloc::vec::Vec;
use void::Void;

pub struct Uart<'a> {
    reader: &'a mut dyn serial::Read<u8,Error=Void>,
    writer: &'a mut dyn serial::Write<u8,Error = Void>,
    pub txd_sink: Sink<Vec<u8>,3>,
    pub rxd_source: Source<'a,Vec<u8>>,
}



impl<'a> Uart<'a> {
    pub fn new(reader : &'a mut dyn Rx<u8,Error=Void>,writer:&'a mut dyn serial::Write<u8,Error = Void>) -> Self {     
        Self {
            reader,
            writer,
            txd_sink: Sink::<Vec<u8>,3>::new(),
            rxd_source: Source::<Vec<u8>>::new(),
        }
    }


    pub async fn run(&mut self) -> Infallible {
        hprintln!("Uart::run()");
        loop {
            hprintln!("Uart::run() loop");
            select_biased! {
                msg = self.txd_sink.recv().fuse() => {
                    hprintln!("Uart::run() msg={:?}",msg);
                    for b in msg {
                        let _ = self.writer.write(b);
                    }
                }
               
            }
        }   
    }
}
