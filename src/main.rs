#![no_std]
#![feature(type_alias_impl_trait)]
//#![feature(noop_waker)]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]
#![feature(const_type_id)]

use alloc::boxed::Box;
use core::cell::Cell;
use core::marker;
use core::marker::PhantomData;
use alloc::vec::Vec;
use core::any::Any;
use core::any::TypeId;
use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::fmt::Write;
use core::ops::ShrAssign;
use core::pin::Pin;
use core::task::Poll;
use cortex_m::peripheral::SYST;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel;
use embassy_sync::channel::Channel;
use embassy_sync::channel::DynamicSender;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::DynPublisher;
use embassy_sync::pubsub::DynSubscriber;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::time_driver_impl;
use embassy_time::Duration;
use embassy_time::Timer;
use embassy_futures::block_on;
use futures::future::Lazy;
use futures::select_biased;
use futures::stream::Once;
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

trait Publisher<T> {
    fn publish(&mut self, value: T);
}

trait Subscriber<T> {
    async fn next_message(&mut self) -> Option<T>;
}

struct Emitter<'a, T>
where
    T: Clone + Send,
{
    sender: Option<DynPublisher<'a, T>>,
    publisher : Option<&'a dyn Publisher<T>>,
}

impl<'a, T> Emitter<'a, T>
where
    T: Clone + Send,
{
    pub fn new() -> Self {
        Emitter { sender: None,publisher:None }
    }
    pub fn bind(&mut self, sender: DynPublisher<'a, T>) {
        if self.sender.is_none() {
            self.sender = Some(sender);
        } else {
            panic!(
                "Emitter already bound for type {}",
                core::any::type_name::<T>()
            );
        }
    }
    pub fn bind_publisher(&mut self, publisher: &'a dyn Publisher<T>) {
        if self.publisher.is_none() {
            self.publisher = Some(publisher);
        } else {
            panic!(
                "Emitter already bound for type {}",
                core::any::type_name::<T>()
            );
        }
    }

    pub fn to<const SIZE: usize>(&mut self, mut pipe:  &'a Pipe<'a,T, SIZE>) {
        self.bind(pipe.input().into());
    }
    pub async fn emit(&mut self, value: T) {
        if self.sender.is_none() {
            return;
        }
        if let Some(sndr) = self.sender.as_mut(){
            sndr.publish(value).await; //todo test for some
        }
    }
}

struct Receptor<'a, T>
where
    T:  Clone + Send,
{
    receiver: Option<DynSubscriber<'a, T>>,
}

impl<'a, T> Receptor<'a, T>
where
    T:  Clone + Send,
{
    pub fn new() -> Self {
        Receptor { receiver: None }
    }
    pub fn bind(&mut self, receiver: DynSubscriber<'a, T>) {
        if self.receiver.is_none() {
            self.receiver = Some(receiver);
        } else {
            panic!("Receptor already bound");
        }
    }
    pub fn from<const SIZE: usize>(&mut self, pipe: &'a Pipe<T, SIZE>) {
        self.bind(pipe.output().into());
    }

    pub async fn receive(&mut self) -> T {
        loop {
            if self.receiver.is_none() {
                Timer::after(Duration::from_millis(100)).await;
            } else {
                if let Some(rcvr) = self.receiver.as_mut(){
                    let x = rcvr.next_message_pure().await;
                    return x;
                }
            }
        }
    }
}

struct Pipe<'a,T, const CAP: usize = 1, const SUBS: usize = 1, const PUBS: usize = 1>
where
    T : Clone + Send,
{
    channel: PubSubChannel<NoopRawMutex, T, CAP, SUBS, PUBS>,
    _marker: marker::PhantomData<&'a T>
}

impl<'a,T, const CAP: usize, const SUBS: usize, const PUBS: usize> Pipe<'a,T, CAP, SUBS, PUBS>
where
    T:  Clone + Send,
{
    pub fn new() -> Self {
        Pipe {
            channel: PubSubChannel::<NoopRawMutex, T, CAP, SUBS, PUBS>::new(),
            _marker: marker::PhantomData,
        }
    }
    pub fn input(&self) -> DynPublisher<T> {
        self.channel.dyn_publisher().unwrap()
    }
    pub fn output(&self) -> DynSubscriber<T> {
        self.channel.dyn_subscriber().unwrap()
    }
}

impl<'a,T,const CAP:usize> Publisher<T> for Pipe<'a,T,CAP> where T:Clone+Send{
     fn publish(&mut self, value: T) {
       block_on( self.channel.publisher().unwrap().publish(value));
    }
}

impl<'a,T> Subscriber<T> for Pipe<'a,T> where T:Clone+Send{
    async fn next_message(&mut self) -> Option<T> {
        Some(self.channel.subscriber().unwrap().next_message_pure().await)
    }
}

struct Transformer<'a,T, U>
where
    T:  Clone + Send,
    U:  Clone + Send,
{
    f: Box<dyn FnMut(T) -> U>,
    receptor: Receptor<'a, T>,
    emitter: Emitter<'a, U>,
}

impl<'a,T, U> Transformer<'a,T, U>
where
    T:  Clone + Send,
    U:  Clone + Send,
{
    pub fn new(f: Box<dyn FnMut(T) -> U>) -> Self {
        Transformer {
            f,
            emitter: Emitter::new(),
            receptor: Receptor::new(),
        }
    }
    pub fn transform(&mut self, value: T) -> U {
        (self.f)(value)
    }
    pub async fn receive(&mut self) {
        loop {
            let x = self.receptor.receive().await;
            let y = self.transform(x);
            self.emitter.emit(y).await;
        }
    }
}

impl<'a, T> ShrAssign< DynPublisher<'a,T>> for Emitter<'a, T>
where
    T: Clone + Send,
{
    fn shr_assign(&mut self, mut rhs:  DynPublisher<'a,T>) {
        self.bind(rhs);
    }
}

impl<'a, T, const S: usize> ShrAssign<&'a Receptor<'a, T>> for Pipe<'a,T, S>
where
    T:  Clone + Send,
{
    fn shr_assign(&mut self, rhs: &'a Receptor<'a, T>) {
        //    rhs.bind(self.receiver());
    }
}
/*
fn queue<'a,T,const SIZE:usize>() -> (DynamicSender<'a,T>, channel::DynamicReceiver<'a,T>) {
    let channel: Channel<NoopRawMutex, T, SIZE> = Channel::<NoopRawMutex, T, SIZE>::new();
    (channel.sender().into(),channel.receiver().into())
}*/

fn transform<'a, T, F>(f: F) -> impl FnMut(T) -> T + 'a
where
    F: FnMut(T) -> T + 'a,
{
    f
}

struct Wifi<'a> {
    pub connected: Emitter<'a, bool>,
}

impl Wifi<'_> {
    fn new() -> Self {
        Wifi {
            connected: Emitter::new(),
        }
    }
}

struct LedBlinker<'a> {
    pub blink_fast: Receptor<'a, bool>,
}

impl LedBlinker<'_> {
    fn new() -> Self {
        LedBlinker {
            blink_fast: Receptor::new(),
        }
    }
    pub async fn run(&mut self) {
        loop {
            let x = self.blink_fast.receive().await;
            hprintln!("x={}", x);
        }
    }
}

// #[cortex_m_rt::entry]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    heap_setup();
    hprintln!("main started");
// pipes defined here so they will be destroyed after actors are destroyed, stupid...
    let channel1 = PubSubChannel::<NoopRawMutex, bool, 10,3,3>::new();
    let pipe1 = Pipe::<bool, 10>::new();

    let mut wifi = Wifi::new();
    let mut led_1 = LedBlinker::new();
    let mut led_2 = LedBlinker::new();

    wifi.connected.to(&pipe1);

    wifi.connected.bind_publisher(&pipe1);
    wifi.connected.bind(channel1.dyn_publisher().unwrap());
    wifi.connected >>= pipe1.input();
 //   led_1.blink_fast.from(pipe1);
 //   led_2.blink_fast.from(pipe1);

    let negate_transformer = Transformer::new(Box::new(|x: bool| !x));

    wifi.connected.emit(true).await;
    wifi.connected.emit(false).await;
    led_1.run().await;

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
    /*loop {
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
    }*/
}
