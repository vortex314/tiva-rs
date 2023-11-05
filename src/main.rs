#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
//#![feature(noop_waker)]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]
#![feature(const_type_id)]

use alloc::boxed::Box;
use alloc::vec::Vec;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel;
use embassy_sync::channel::Channel;
use embassy_sync::channel::DynamicSender;
use embassy_sync::channel::Sender;
use core::any::Any;
use core::any::TypeId;
use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::fmt::Write;
use core::ops::ShrAssign;
use core::task::Poll;
use cortex_m::peripheral::SYST;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_time::time_driver_impl;
use embassy_time::Duration;
use embassy_time::Timer;
use futures::select_biased;
use futures::FutureExt;
use serde::de;
// use std::process::Output;
// use alloc::task;

// use lilos::time::TickTime;
use embassy_executor::Spawner;
use embassy_sync::mutex::Mutex;
use embassy_time::driver::{AlarmHandle, Driver};
use embassy_time::Instant;

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
use panic_semihosting as _;

use serde::ser::SerializeSeq;
use serde::Serialize;
use serde::Serializer;
use serde_derive::Serialize;
use serde_json_core::ser::Serializer as Ser;

#[derive(Serialize)]
struct Test {
    x: u32,
    b: &'static str,
}

extern crate alloc;
use core::option::Option::Some;

use crate::limero::Sink;

/*
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    hprintln!("panic {:?}", _info);
    loop {}
}
*/

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
mod timer_driver;
use timer_driver::Clock;
mod serde_actor;
use serde_actor::SerdeActor;

#[embassy_executor::task]
async fn timer_server_task() {

    loop {
        hprintln!("timer_server_task");
    }
}

struct Emitter<'a,T> {
    sender:Option<DynamicSender<'a,T>>,
}

impl<'a,T> Emitter<'a,T> {
    pub fn new() -> Self {
        Emitter{sender:None}
    }
    pub fn bind(&mut self, sender : DynamicSender<'a,T>) {
        if self.sender.is_none() {
            self.sender = Some(sender);
        }
    }
    pub fn pipe<const SIZE:usize>(&mut self,pipe:&  'a Pipe<T,SIZE>) {
        self.bind(pipe.sender());
    }
    pub async fn emit(&mut self, value:T) {
        self.sender.as_mut().unwrap().send(value).await; //todo test for some
    }
}

struct Receptor<'a,T> {
    receiver:Option<channel::DynamicReceiver<'a,T>>,
}

impl<'a,T> Receptor<'a,T> {
    pub fn new() -> Self {
        Receptor{receiver:None}
    }
    pub fn bind(&mut self, receiver : channel::DynamicReceiver<'a,T>) {
        if self.receiver.is_none() {
            self.receiver = Some(receiver);
        }
    }
    pub fn pipe<const SIZE:usize>(&mut self,pipe:&  'a Pipe<T,SIZE>) {
        self.bind(pipe.receiver());
    }
    pub async fn poll(&mut self) -> T {
        self.receiver.as_mut().unwrap().receive().await
    }
}




struct Actor1<'a> {
    pub emitter:Emitter<'a,u32>,
}

struct Actor2<'a> {
    pub receptor:Receptor<'a,u32>,
}

impl Actor2<'_> {
    pub async fn run(&mut self) {
        loop {
            let x = self.receptor.poll().await;
            hprintln!("x={}",x);
        }
    }
}

#[embassy_executor::task]
async fn actor1_task(mut actor1: Actor1<'static>) {
    actor1.emitter.emit(1).await;
    actor1.emitter.emit(2).await;
}

#[embassy_executor::task]
async fn actor2_task(mut actor2: Actor2<'static>) {
    loop {
        let x = actor2.receptor.poll().await;
        hprintln!("x={}",x);
    }
}

struct Pipe<T,const SIZE:usize> {
    channel : Channel<NoopRawMutex, T, SIZE>,
}

impl<T,const SIZE:usize> Pipe<T,SIZE> {
    pub fn new( ) -> Self {
        Pipe{channel:Channel::<NoopRawMutex, T, SIZE>::new()}
    }
    pub fn sender(&self) -> DynamicSender<T> {
        self.channel.sender().into()
    }
    pub fn receiver(&self) -> channel::DynamicReceiver<T> {
        self.channel.receiver().into()
    }
}

impl<'a,T,const S:usize> ShrAssign<&'a Pipe<T,S>> for Emitter<'a,T> {
    fn shr_assign(&mut self, rhs: & 'a Pipe<T,S>) {
        self.bind(rhs.sender());
    }
}

impl<'a,T,const S:usize> ShrAssign<&'a Receptor<'a,T>> for Pipe<T,S> {
    fn shr_assign(&mut self, rhs: & 'a Receptor<'a,T>) {
    //    rhs.bind(self.receiver());
    }
}
/* 
fn queue<'a,T,const SIZE:usize>() -> (DynamicSender<'a,T>, channel::DynamicReceiver<'a,T>) {
    let channel: Channel<NoopRawMutex, T, SIZE> = Channel::<NoopRawMutex, T, SIZE>::new();
    (channel.sender().into(),channel.receiver().into())
}*/


// #[cortex_m_rt::entry]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    heap_setup();
    hprintln!("main started"   );
    let mut actor1 = Actor1{emitter:Emitter::new()};
    let mut actor2 = Actor2{receptor:Receptor::new()};
    let mut actor3 = Actor2{receptor:Receptor::new()};
    let channel1 = Channel::<NoopRawMutex, u32, 10>::new();
    let pipe1 = Pipe::<u32,10>::new();
    actor1.emitter >>= &pipe1;
    actor1.emitter.pipe(&pipe1);
    actor2.receptor.pipe(&pipe1);
 //   let mut actor1 = Box::pin(actor1);
 //   let mut actor2 = Box::pin(actor2);
    actor1.emitter.bind(channel1.sender().into());
    actor2.receptor.bind(channel1.receiver().into());
    actor3.receptor.bind(channel1.receiver().into());
    actor1.emitter.emit(1).await   ;
    actor1.emitter.emit(2).await   ;
    actor2.run().await;

  //  actor1.emitter >> queue(10) >> actor2.receptor;
  //  actor1.emitter >> transform(|x| x+1) >> actor2.receptor;

    let mut peripherals = hal::Peripherals::take().unwrap();
    let mut sysctl = peripherals.SYSCTL.constrain();
    let mut porta_parts = peripherals.GPIO_PORTA.split(&sysctl.power_control);
    sysctl.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sysctl.clock_setup.freeze();
    let mut cortex_peripherals = cortex_m::Peripherals::take().unwrap();
    Clock::init_timer_driver(cortex_peripherals.SYST);

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

    let mut portf = peripherals.GPIO_PORTF.split(&sysctl.power_control);

    let mut pin_red = portf.pf1.into_push_pull_output();
    /*    let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();

    let porte = peripherals.GPIO_PORTE.split(&sysctl.power_control);
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
    });*/

    let (mut tx, mut rx) = uart.split();
  //  static mut uart_actor:UnsafeCell<Uart> = UnsafeCell::new(Uart::new(&mut rx, &mut tx));
    let mut serde_actor = SerdeActor::new();
 //   &mut serde_actor.txd_source >> uart_actor.txd_sink;
    //   &mut uart_actor.rxd_source >> &serde_actor.rxd_sink;
    serde_actor.publish("src/tiva/sys/started", &"started");
    serde_actor.publish("src/tiva/sys/heap", &ALLOCATOR.free());
    serde_actor.publish("src/tiva/sys/heap_size", &[1, 2, 3, 4]);
    let t = Test { x: 1, b: "hi" };
    serde_actor.publish("src/tiva/sys/test", &t);
    let mut led_actor = led::Led::new(&mut pin_red);
    led_actor.active_sink.on(true);
    hprintln!("main loop started");
    loop {
        select_biased! {
            /*_ = get_timer_server().run().fuse() => {
                hprintln!("timer_server_task done");
            },
             _ = uart_actor.run().fuse() => {
                hprintln!("uart_actor done");
            },*/
            _ = serde_actor.run().fuse() => {
                hprintln!("serde_actor done");
            },
            _ = led_actor.run().fuse() => {
                hprintln!("led_actor done");
            },
        };
    }
}
