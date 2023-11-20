#![no_std]
#![feature(type_alias_impl_trait)]
//#![feature(noop_waker)]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]
#![feature(const_type_id)]
#![deny(elided_lifetimes_in_paths)]

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;
use cortex_m::interrupt;
use core::any::Any;
use core::any::TypeId;
use core::cell::Cell;
use core::cell::RefCell;
use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::fmt::Error;
use core::fmt::Write;
use core::marker;
use core::marker::PhantomData;
use core::ops::ShrAssign;
use core::pin::Pin;
use core::task::Poll;
use cortex_m::peripheral::SYST;
use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_futures::block_on;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel;
use embassy_sync::channel::Channel;
use embassy_sync::channel::DynamicReceiver;
use embassy_sync::channel::DynamicSender;
use embassy_sync::channel::Receiver;
use embassy_sync::channel::Sender;
use embassy_sync::pubsub::DynPublisher;
use embassy_sync::pubsub::DynSubscriber;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::pubsub::Subscriber;
use embassy_time::time_driver_impl;
use embassy_time::Duration;
use embassy_time::Timer;
use futures::future::Lazy;
use futures::select_biased;
use futures::stream::Once;
use futures::Future;
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
use alloc::sync::Arc;
use core::option::Option::Some;

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
/*

Source : property that emits values / messages if somebody is listening or not
Publisher : the one that is used by Source to send really the events
Subscriber : creates a handle to a function that can be used to async receiev events
Receptor ; property of an actor that receives events and acts upon it, if it is not connected it will never receive a message

wifi >> map(|x| match x {
    WifiMsg::Connected => LedMsg::On,
    WifiMsg::Disconnected => LedMsg::Off
}) >> led
*/

//==============================================================================
trait Sink<T> {
    fn on(&self, value: &T);
}
struct Source<T> {
    sinks: Vec<Box<dyn Fn(&T)>>,
}

impl<T> Source<T> {
    pub fn new() -> Self {
        Source { sinks: Vec::new() }
    }

    pub fn bind(&mut self, sink: Box<dyn Fn(&T)>) {
        self.sinks.push(sink)
    }

    pub fn emit(&mut self, value: &T) {
        for sink in self.sinks.iter() {
            sink(&value);
        }
    }
}

struct Led {
    state: bool,
}

impl Led {
    pub fn new() -> Self {
        Led { state: false }
    }
}

enum LedMsg {
    On,
    Off,
    Blink(u32),
}

impl Sink<LedMsg> for Rc<RefCell<Led>> {
    fn on(&self, value: &LedMsg) {
        let mut this = self.borrow_mut();
        match value {
            LedMsg::On => {
                this.state = true;
            }
            LedMsg::Off => {
                this.state = false;
            }
            LedMsg::Blink(t) => {
                hprintln!("blink {}", t);
                this.state = true;
                // wait for t
                this.state = false;
            }
        }
    }
}

#[derive(PartialEq)]
enum ButtonEvent {
    Pressed,
    Released,
}
enum ButtonAction {
    Idle,
    Active,
}
struct Button {
    source: Source<ButtonEvent>,
}
impl Button {
    pub fn new() -> Self {
        Button {
            source: Source::new(),
        }
    }
}

struct Pipe<T> {
    source: Source<T>,
}

impl<T> Sink<T> for Rc<RefCell<Pipe<T>>> {
    fn on(&self, value: &T) {
        self.borrow_mut().source.emit(value);
    }
}

// interrupt handler wake ButtonActor
struct ButtonActor {
    action: ButtonAction,
    receptor: Pipe<ButtonEvent>,
    emitter: Source<ButtonEvent>,
}
#[interrupt]
fn GPIOF() {
    hprintln!("GPIOF");
    //  button_actor.receptor.emit(&ButtonEvent::Pressed);
}

fn test() {
    let mut button = Button::new();
    let mut led = Rc::new(RefCell::new(Led::new()));
    button.source.bind(Box::new(move |x| match x {
        ButtonEvent::Pressed => led.on(&LedMsg::Blink(300)),
        ButtonEvent::Released => led.on(&LedMsg::Blink(1000)),
    }));
    button.source.emit(&ButtonEvent::Pressed);
    button.source.emit(&ButtonEvent::Released);
}

// #[cortex_m_rt::entry]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    heap_setup();
    hprintln!("main started");
    // pipes defined here so they will be destroyed after actors are destroyed, stupid...
    test();
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
    let switch2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();
    let switch1 = portf.pf4.into_pull_up_input();
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
