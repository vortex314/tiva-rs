#![no_std]
//#![feature(noop_waker)]
#![no_main]
#![allow(deprecated)]
//#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]

use alloc::boxed::Box;
use serde::de;
use core::any::Any;
use core::convert::Infallible;
use core::fmt::Write;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use futures::select_biased;
// use std::process::Output;
// use alloc::task;

use lilos::time::TickTime;

use tm4c123x_hal::gpio::gpioa::PA0;
use tm4c123x_hal::gpio::gpioa::PA1;
use tm4c123x_hal::gpio::AlternateFunction;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::gpio::PushPull;
use tm4c123x_hal::gpio::AF1;
use tm4c123x_hal::serial::Serial;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c123x_hal::{self as hal, prelude::*};

use alloc::string::String;
use core::panic::PanicInfo;

use serde::ser::SerializeSeq;
use serde::Serializer;
use serde_json_core::ser::Serializer as Ser;

extern crate alloc;
use core::option::Option::Some;

use crate::limero::get_timer_server;
use crate::limero::Sink;
use crate::limero::TimerMsg;
use thingbuf as conn;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    hprintln!("panic {:?}", _info);
    loop {}
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    hprintln!("{:#?}", ef);
    loop {}
}

// HEAP allocation for embedded
use embedded_alloc::Heap;
#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
const HEAP_SIZE: usize = 10240; // in bytes
fn heap_setup() {
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) } // 👈
}

mod led;
mod limero;
mod uart;

use uart::Uart;

#[cortex_m_rt::entry]
fn main() -> ! {
    heap_setup();

    let (tx, rx) = conn::mpsc::channel::<String>(10);
    let mut peripherals = hal::Peripherals::take().unwrap();
    let mut sysctl = peripherals.SYSCTL.constrain();
    let mut porta_parts = peripherals.GPIO_PORTA.split(&sysctl.power_control);
    sysctl.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sysctl.clock_setup.freeze();
    let mut cortex_peripherals = cortex_m::Peripherals::take().unwrap();
    //   let mut tmc = device::CorePeripherals::take().unwrap();
    // systick_setup(&cp);

    let tx_pin = porta_parts
        .pa1
        .into_af_push_pull::<hal::gpio::AF1>(&mut porta_parts.control);
    let rx_pin = porta_parts.pa0.into_af_push_pull(&mut porta_parts.control);

    let uart = hal::serial::Serial::uart0(
        peripherals.UART0,
        tx_pin,
        rx_pin,
        (),
        (),
        115200_u32.bps(),
        hal::serial::NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sysctl.power_control,
    );
    let (mut tx, mut rx) = uart.split();
    let mut uart_actor = Uart::new(&mut rx, &mut tx);
    let mut serializer = SerdeActor::new();
    &mut serializer.txd_source >> & uart_actor.txd_sink;
    &mut uart_actor.rxd_source >> & serializer.rxd_sink;

    let mut portf = peripherals.GPIO_PORTF.split(&sysctl.power_control);
    let porte = peripherals.GPIO_PORTE.split(&sysctl.power_control);

    let mut pin_red = portf.pf1.into_push_pull_output();
    let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();

    let mut pin_uext1 = porte.pe2.into_push_pull_output();
    let mut pin_uext2 = porte.pe3.into_push_pull_output();
    pin_uext1.set_low();
    pin_uext2.set_low();
    //let mut ctrl = portf.control;
    let switcher = core::pin::pin!(async move {
        let switch2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();
        let switch1 = portf.pf4.into_pull_up_input();
        loop {
            if switch1.is_low() {
                pin_blue.set_low();
                pin_green.set_high();
            }
            if switch2.is_low() {
                pin_blue.set_high();
                pin_green.set_low();
            }
        }
    });

    let mut led_actor = led::Led::new(&mut pin_red);
    led_actor.active_sink.on(true);
    let timer_server_task = core::pin::pin!(get_timer_server().run());
    //    let uart_sender = core::pin::pin!(uart_sender(uart));
    lilos::time::initialize_sys_tick(&mut cortex_peripherals.SYST, 80_000_000);
    lilos::exec::run_tasks(
        &mut [
            timer_server_task,
            core::pin::pin!(led_actor.run()),
            core::pin::pin!(uart_actor.run()),
            core::pin::pin!(serializer.run()),
        ], // <-- array of tasks
        lilos::exec::ALL_TASKS, // <-- which to start initially
    );
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

#[allow(dead_code)]
use alloc::vec::Vec;
use limero::Source;
use futures::FutureExt;
use core::fmt::Debug;
type Bytes = Vec<u8>;
use serde::Serialize;
#[derive(Debug,Default ,Clone  )]
enum PubSubMsg {
    #[default]
    None,
    Sub(String),
    Pub(String,Bytes)   ,
}
struct SerdeActor {
    pub txd_source: Source<Bytes>,
    pub rxd_sink: Sink<Bytes>,
    pub pubsub_msg : Sink<PubSubMsg>,
    timer_tick: Sink<TimerMsg>,
    tick_time : TickTime,
    buffer: Box<[u8; 100]>,
}

impl SerdeActor {
    pub fn new() -> Self {
        let timer_tick = Sink::<TimerMsg>::new(3);
        let tick_time = TickTime::now();

        Self {
            txd_source: Source::<Vec<u8>>::new(),
            rxd_sink: Sink::<Vec<u8>>::new(10),
            timer_tick,
            tick_time,
            pubsub_msg : Sink::<PubSubMsg>::new(10),
            buffer: Box::new([0u8; 100]),
        }
    }

    pub fn publish(& self,topic: &str, payload: & dyn Serialize<serde_json_core::ser::Serializer>) {
        let mut serializer: Ser<'_> = Ser::new(self.buffer.as_mut());
        let mut seq = serializer.serialize_seq(None).unwrap();
        serializer.serialize_element("pub").unwrap();
        serializer.serialize_element(topic).unwrap();
        serializer.serialize_element(payload).unwrap();
        seq.end().unwrap();
        let length = serializer.end();
        let crc = crc_calc(self.buffer.as_mut(), length);
        let crc_str = alloc::format!("{:04X}\r\n", crc);
        self.buffer[length..length+6].copy_from_slice(crc_str.as_bytes());
        length += 6;
        self.txd_source.emit((self.buffer.as_slice()[0..length]).to_vec());
    }

    pub async fn run(&mut self) -> Infallible {
        get_timer_server().new_gate(100, self.timer_tick.sender()).await;
        loop {
            select_biased! {
                _ = self.timer_tick.recv().fuse() => {
                    self.keep_alive();
                },
                msg = self.rxd_sink.recv().fuse() => {
                    hprintln!("pubsub msg {:?}", msg);
                },
                msg = self.pubsub_msg.recv().fuse() => {
                     match  msg  {
                        PubSubMsg::Pub(topic,x) => {
                            let mut serializer: Ser<'_> = Ser::new(self.buffer.as_mut());
                            let mut seq = serializer.serialize_seq(None).unwrap();
                            serializer.serialize_element("pub").unwrap();
                            serializer.serialize_element(topic.as_str()).unwrap();
                            serializer.serialize_element(x).unwrap();
                            seq.end().unwrap();
                            let length = serializer.end();
                            let crc = crc_calc(self.buffer.as_mut(), length);
                            let crc_str = alloc::format!("{:04X}\r\n", crc);
                            self.buffer[length..length+6].copy_from_slice(crc_str.as_bytes());
                            length += 6;
                            self.txd_source.emit((self.buffer.as_slice()[0..length]).to_vec());

                        } 
                        _ => {}
                    };
                },
                
            }
        }
    }
    fn keep_alive(&self) {
        let mut buffer: Box<[u8; 100]> = Box::new([0u8; 100]);
        let mut serializer = Ser::new(buffer.as_mut());
        let mut seq = serializer.serialize_seq(None).unwrap();
        seq.serialize_element("pub").unwrap();
        seq.serialize_element("src/tiva/sys/loopback").unwrap();
        let u = self.tick_time.elapsed().0;
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
        let size = ALLOCATOR.free();
        seq.serialize_element(&size).unwrap();
        seq.end().unwrap();
        let length = serializer.end();
        let crc = crc_calc(buffer.as_mut(), length);
        self.txd_source.emit((buffer.as_slice()[0..length]).to_vec());
        let crc_str = alloc::format!("{:04X}\r\n", crc);
        self.txd_source.emit(crc_str.as_bytes().to_vec());
    }
}

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
    let tick_time = TickTime::now();
    let mut buffer: Box<[u8; 100]> = Box::new([0u8; 100]);
    let mut ts = Box::new(Sink::<TimerMsg>::new(2));
    get_timer_server().new_interval(100, ts.sender()).await;
    hprintln!("uart_sender started");
    loop {
        //     hprintln!("uart_sender loop");
        let _ = ts.recv().await;
        let mut serializer: Ser<'_> = Ser::new(buffer.as_mut());
        let mut seq = serializer.serialize_seq(None).unwrap();
        seq.serialize_element("pub").unwrap();
        seq.serialize_element("src/tiva/sys/loopback").unwrap();
        let u = tick_time.elapsed().0;
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
        let size = ALLOCATOR.free();
        seq.serialize_element(&size).unwrap();
        seq.end().unwrap();
        let length = serializer.end();
        let crc = crc_calc(buffer.as_mut(), length);
        tx.write_all(&buffer.as_slice()[0..length]);
        // tx.write_all(&['\r' as u8, '\n' as u8]);
        //     tx.write_all(&buffer.as_mut());
        tx.write_fmt(format_args!("{:04X}\r\n", crc)).unwrap();
    }
}
