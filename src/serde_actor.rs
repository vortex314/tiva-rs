use alloc::boxed::Box;
use alloc::string::String;
#[allow(dead_code)]
use alloc::vec::Vec;
use core::convert::Infallible;
use core::fmt::Debug;
use cortex_m_semihosting::hprintln;
use embassy_time::Duration;
use embassy_time::Timer;
use futures::select_biased;
use futures::FutureExt;
use serde::ser::SerializeSeq;
use serde::Serialize;
use serde::Serializer;
use serde_derive::Serialize;
use serde_json_core::ser::Serializer as Ser;

type Bytes = Vec<u8>;
use crate::limero::Sink;
use crate::limero::Source;
use crate::timer_driver::{usec, CLOCK};
#[derive(Debug, Default, Clone)]
pub enum PubSubMsg {
    #[default]
    None,
    Sub(String),
    Pub(String, Bytes),
}
pub struct SerdeActor<'a> {
    pub txd_source: Source<'a,Bytes>,
    pub rxd_sink: Sink<Bytes,3>,
    pub pubsub_msg: Sink<PubSubMsg,3>,
    buffer: Box<[u8; 100]>,
}
struct BytesWrapper<'a>(&'a Vec<u8>);

impl BytesWrapper<'_> {
    pub fn new<'a>(bytes: &'a Vec<u8>) -> BytesWrapper<'a> {
        BytesWrapper(bytes)
    }
}

impl Serialize for BytesWrapper<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        //todo could be a base64 encoding
        let mut seq = serializer.serialize_seq(Some(self.0.len()))?;
        for e in self.0 {
            seq.serialize_element(e)?;
        }
        seq.end()
    }
}

impl SerdeActor<'_> {
    pub fn new() -> Self {
        Self {
            txd_source: Source::<Vec<u8>>::new(),
            rxd_sink: Sink::<Vec<u8>,3>::new(),
            pubsub_msg: Sink::<PubSubMsg,3>::new(),
            buffer: Box::new([0u8; 100]),
        }
    }

    fn crc_calc(data: &[u8], length: usize) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for index in 0..length {
            crc ^= data[index] as u16;
            for _j in 0..8 {
                if crc & 1 == 1 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        crc
    }

    pub async fn pub_me(&self) {
        loop {
            self.publish("src/tiva/sys/clock", &usec());
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    pub fn publish<T>(&self, topic: &str, payload: &T)
    where
        T: Serialize,
    {
        let mut buffer = Box::new([0u8; 100]);
        let mut serializer: Ser<'_> = Ser::new(buffer.as_mut());
        let mut seq = serializer.serialize_seq(None).unwrap();
        seq.serialize_element("pub").unwrap();
        seq.serialize_element(topic).unwrap();
        seq.serialize_element(payload).unwrap();
        seq.end().unwrap();
        let mut length = serializer.end();
        let crc = Self::crc_calc(buffer.as_mut(), length);
        let crc_str = alloc::format!("{:04X}\r\n", crc);
        buffer[length..length + 6].copy_from_slice(crc_str.as_bytes());
        length += 6;
        self.txd_source
            .emit((buffer.as_slice()[0..length]).to_vec());
    }

    pub async fn run(&mut self) -> Infallible {
        loop {
            select_biased! {
                _ =  self.keep_alive().fuse() => {
                    hprintln!("pubsub keep_alive()");   
                },
                 /*msg = self.rxd_sink.recv().fuse() => {
                    hprintln!("pubsub rxd_sink.recv()");
                },
                msg = self.pubsub_msg.recv().fuse() => {
                    hprintln!("pubsub pubsub_msg.recv()");
                    match  msg  {
                        PubSubMsg::Pub(topic,x) => {
                            let mut serializer: Ser<'_> = Ser::new(self.buffer.as_mut());
                            let mut seq = serializer.serialize_seq(None).unwrap();
                            seq.serialize_element("pub").unwrap();
                            seq.serialize_element(topic.as_str()).unwrap();
                            seq.serialize_element(& BytesWrapper::new(&x)).unwrap();
                            seq.end().unwrap();
                            let mut length = serializer.end();
                            let crc = Self::crc_calc(self.buffer.as_mut(), length);
                            let crc_str = alloc::format!("{:04X}\r\n", crc);
                            self.buffer[length..length+6].copy_from_slice(crc_str.as_bytes());
                            length += 6;
                            self.txd_source.emit((self.buffer.as_slice()[0..length]).to_vec());

                        }
                        _ => {}
                    };
                },*/

            }
        }
    }
    async fn keep_alive(&self) {
        hprintln!("pubsub keep_alive()");
        loop {
            Timer::after(Duration::from_millis(1000)).await;
            hprintln!("pubsub keep_alive() in loop");

            let mut buffer: Box<[u8; 100]> = Box::new([0u8; 100]);
            let mut serializer = Ser::new(buffer.as_mut());
            let mut seq = serializer.serialize_seq(None).unwrap();
            seq.serialize_element("pub").unwrap();
            seq.serialize_element("src/tiva/sys/loopback").unwrap();
            let u = 123456; // self.tick_time.elapsed().0;
            let msec = u % 1000;
            let sec = (u / 1000) % 60;
            let min = (u / 60000) % 60;
            let hour = (u / 3600000) % 24;
            let day = (u / 86400000) % 365;
            seq.serialize_element(&day).unwrap();
            seq.serialize_element(&hour).unwrap();
            seq.serialize_element(&min).unwrap();
            seq.serialize_element(&sec).unwrap();
            seq.serialize_element(&msec).unwrap();
            seq.end().unwrap();
            let length = serializer.end();
            let crc = Self::crc_calc(buffer.as_mut(), length);
            self.txd_source
                .emit((buffer.as_slice()[0..length]).to_vec());
            let crc_str = alloc::format!("{:04X}\r\n", crc);
            self.txd_source.emit(crc_str.as_bytes().to_vec());
            hprintln!("pubsub keep_alive() done");
        }
    }
}
/*
async fn uart_sender(
    uart0: Serial<
        tm4c123x::UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
) -> Infallible {
    let (mut tx, _rx) = uart0.split();
    // let tick_time = TickTime::now();
    let mut buffer: Box<[u8; 100]> = Box::new([0u8; 100]);
    let mut ts = Box::new(Sink::<TimerMsg>::new(2));
    loop {
        //     hprintln!("uart_sender loop");
        let _ = ts.recv().await;
        let mut serializer: Ser<'_> = Ser::new(buffer.as_mut());
        let mut seq = serializer.serialize_seq(None).unwrap();
        seq.serialize_element("pub").unwrap();
        seq.serialize_element("src/tiva/sys/loopback").unwrap();
        let u = CLOCK::now().0;
        let msec = u % 1000;
        let sec = (u / 1000) % 60;
        let min = (u / 60000) % 60;
        let hour = (u / 3600000) % 24;
        let day = (u / 86400000) % 365;
        seq.serialize_element(&day).unwrap();
        seq.serialize_element(&hour).unwrap();
        seq.serialize_element(&min).unwrap();
        seq.serialize_element(&sec).unwrap();
        seq.serialize_element(&msec).unwrap();
        seq.end().unwrap();
        let length = serializer.end();
        let crc = SerdeActor::crc_calc(buffer.as_mut(), length);
        tx.write_all(&buffer.as_slice()[0..length]);
        // tx.write_all(&['\r' as u8, '\n' as u8]);
        //     tx.write_all(&buffer.as_mut());
        tx.write_fmt(format_args!("{:04X}\r\n", crc)).unwrap();
    }
}
*/
