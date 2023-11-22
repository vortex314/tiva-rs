#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(noop_waker)]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_mut)]
#![feature(const_type_id)]
#![deny(elided_lifetimes_in_paths)]

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::ToString;
use alloc::vec;
use alloc::vec::Vec;
use core::any::Any;
use core::any::TypeId;
use core::cell::Cell;
use core::cell::RefCell;
use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::default;
use core::fmt::Error;
use core::fmt::Write;
use core::marker;
use core::marker::PhantomData;
use core::ops::ShrAssign;
use core::pin::Pin;
use core::task::Poll;
use cortex_m::peripheral::SYST;
use cortex_m_rt::entry;
use cortex_m_rt::exception;
use tm4c123x::interrupt;
use tm4c_hal::gpio;

use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_futures::block_on;
use embassy_futures::select::select;
use embassy_futures::select::select4;

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

use core::ops::Shr;
use serde::ser::SerializeSeq;
use serde::Serialize;
use serde::Serializer;
use serde_derive::Serialize;
use serde_json_core::ser::Serializer as Ser;

mod led;
mod limero;
mod uart;
use uart::Uart;
mod timer_driver;
use timer_driver::Clock;
mod serde_actor;
use serde_actor::SerdeActor;

#[derive(Serialize)]
struct Test {
    x: u32,
    b: &'static str,
}

extern crate alloc;
use alloc::sync::Arc;
use core::option::Option::Some;
use mini_io_queue::asyncio;
use mini_io_queue::storage::HeapBuffer;

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
trait Listener<T> {
    fn on(&self, value: &T);
}
trait Publisher<T> {
    fn add_listener(&self, listener: Box<dyn Listener<T>>) -> usize;
    fn remove_listener(&self, listener_id: usize);
    fn emit(&self, value: &T);
}

struct ActorData<CMD, EVENT> {
    cmds_reader: asyncio::Reader<HeapBuffer<CMD>>,
    cmds_writer: asyncio::Writer<HeapBuffer<CMD>>,
    // events_reader: asyncio::Reader<HeapBuffer<EVENT>>,
    // events_writer: asyncio::Writer<HeapBuffer<EVENT>>,
    listeners: Vec<Box<dyn Listener<EVENT>>>,
}
impl<CMD, EVENT> Clone for Actor<CMD, EVENT> {
    fn clone(&self) -> Self {
        Actor {
            data: Rc::clone(&self.data),
        }
    }
}
impl<CMD, EVENT> ActorData<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn new(in_capacity: usize, out_capacity: usize) -> Self {
        let (cmds_reader, cmds_writer) = asyncio::queue(in_capacity);
        //       let (events_reader, events_writer) = asyncio::queue(out_capacity);
        ActorData {
            cmds_reader,
            cmds_writer,
            //            events_reader,
            //           events_writer,
            listeners: Vec::new(),
        }
    }
}
struct Actor<CMD, EVENT> {
    data: Rc<RefCell<ActorData<CMD, EVENT>>>,
}
#[derive(Debug, Clone, Default)]
enum NoEvent {
    #[default]
    Zero = 0,
}
#[derive(Debug, Clone, Default)]
enum NoCmd {
    #[default]
    Zero = 0,
}
impl<CMD, EVENT> Actor<CMD, EVENT>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    fn new(in_capacity: usize, out_capacity: usize) -> Self {
        Actor {
            data: Rc::new(RefCell::new(ActorData::new(in_capacity, out_capacity))),
        }
    }
    async fn recv(&self) -> CMD {
        let mut cmd = [CMD::default()];
        hprintln!("I Actor waiting recv....");
        let cnt = self.data.borrow_mut().cmds_reader.read(&mut cmd).await;
        if cnt == 0 {
            hprintln!("I Actor no cmd");
        }
        hprintln!("I Actor received event ");
        cmd[0].clone()
    }
}

impl<CMD, EVENT> Listener<CMD> for Actor<CMD, EVENT>
where
    CMD: Clone,
{
    fn on(&self, cmd: &CMD) {
        hprintln!("I Listener on ");
        let mut data = self.data.borrow_mut();
        if data.cmds_writer.has_space() {
            hprintln!("I Listener write ....");
            block_on(async {
                let rc = data.cmds_writer.write(&[cmd.clone()]).await;
                hprintln!("I Listener write done : {} ", rc);
            });
        } else {
            hprintln!("W Listener no space");
        };
    }
}

impl<EVENT, CMD> Publisher<EVENT> for Actor<CMD, EVENT>
where
    EVENT: Clone,
{
    fn add_listener(&self, listener: Box<dyn Listener<EVENT>>) -> usize {
        self.data.borrow_mut().listeners.push(listener);
        hprintln!("add_listener");
        self.data.borrow_mut().listeners.len() - 1
    }
    fn remove_listener(&self, listener_id: usize) {
        self.data.borrow_mut().listeners.remove(listener_id);
    }
    fn emit(&self, value: &EVENT) {
        hprintln!("I emit()");
        for listener in self.data.borrow().listeners.iter() {
            hprintln!("I emit to listener");
            listener.on(value);
        }
    }
}
struct Flow<EVENT, CMD> {
    pub actor: Actor<EVENT, CMD>,
    func: fn(&EVENT) -> CMD,
}
impl<EVENT, CMD> Flow<EVENT, CMD>
where
    CMD: Clone + Default,
    EVENT: Clone + Default,
{
    pub fn new(func: fn(&EVENT) -> CMD) -> Self {
        Flow {
            actor: Actor::new(3, 3),
            func,
        }
    }
    async fn run(&mut self) {
        loop {
            hprintln!("I Flow waiting recv....");
            let event = self.actor.recv().await;
            hprintln!("I Flow received event ");
            let cmd = (self.func)(&event);
            self.actor.emit(&cmd);
        }
    }
}
#[derive(Debug, Clone, Default)]
enum MqttCmd {
    #[default]
    Connect,
    Disconnect,
    Publish(String, Vec<u8>),
    Subscribe(String),
    Unsubscribe(String),
}
#[derive(Debug, Clone, Default)]
enum MqttEvent {
    #[default]
    Connected,
    Disconnected,
    Message(String, Vec<u8>),
}
struct Mqtt {
    pub actor: Actor<MqttCmd, MqttEvent>,
}

impl Mqtt {
    fn new() -> Self {
        Mqtt {
            actor: Actor::new(10, 10),
        }
    }
    async fn run(&mut self) {
        loop {
            let mut cmd = [MqttCmd::default()];
            self.actor
                .data
                .borrow_mut()
                .cmds_reader
                .read(&mut cmd)
                .await;
            match cmd[0].clone() {
                MqttCmd::Connect => {
                    self.actor.emit(&MqttEvent::Connected);
                    // connect
                    // emit event
                }
                MqttCmd::Disconnect => {
                    // disconnect
                    // emit event
                }
                MqttCmd::Publish(topic, payload) => {
                    // publish
                    // emit event
                }
                MqttCmd::Subscribe(topic) => {
                    // subscribe
                    // emit event
                }
                MqttCmd::Unsubscribe(topic) => {
                    // unsubscribe
                    // emit event
                }
            }
        }
    }
}

type Rhs<T> = Box<dyn Listener<T>>;
type Lhs<T> = Box<dyn Publisher<T>>;

impl<T> Shr<Rhs<T>> for Lhs<T> {
    type Output = usize;

    fn shr(self, rhs: Rhs<T>) -> Self::Output {
        self.add_listener(rhs)
    }
}

#[derive(Debug, Clone, Default)]
enum LedCmd {
    #[default]
    On,
    Off,
    Blink(u32),
}
struct Led {
    actor: Actor<LedCmd, NoEvent>,
    state: Rc<RefCell<bool>>,
}

impl Led {
    pub fn new() -> Self {
        Led {
            state: Rc::new(RefCell::new(false)),
            actor: Actor::new(10, 0),
        }
    }
    async fn run(&mut self) {
        loop {
            let cmd = self.actor.recv().await;
            hprintln!("received event {:?}", cmd);
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

#[derive(Debug, Clone, Default)]
enum ButtonEvent {
    #[default]
    Released,
    Pressed,
}

struct Button {
    actor: Actor<NoCmd, ButtonEvent>,
}
impl Button {
    pub fn new() -> Self {
        Button {
            actor: Actor::new(0, 10),
        }
    }
    pub async fn run(&mut self) {
        hprintln!("I Button sleep");
        //           Timer::after(Duration::from_millis(5000)).await;
        hprintln!("I Button pressed");
        self.actor.emit(&ButtonEvent::Pressed);
    }
}

// interrupt handler wake ButtonActor

#[interrupt]
unsafe fn GPIOF() {
    hprintln!("GPIOF");
    //  button_actor.receptor.emit(&ButtonEvent::Pressed);
}

impl<'a, T, U, V> Shr<&'a Actor<U, V>> for &'a Actor<T, U>
where
    T: Clone + Default + 'static,
    U: Clone + Default + 'static,
    V: Clone + Default + 'static,
{
    type Output = &'a Actor<U, V>;

    fn shr(self, rhs: &'a Actor<U, V>) -> Self::Output {
        self.add_listener(Box::new(rhs.clone()));
        rhs
    }
}

async fn test() {
    let mut mqtt = Mqtt::new();
    let mut button = Button::new();
    let mut led = Led::new();
    let mut pressed_led_on = Flow::new(|x| match x {
        ButtonEvent::Pressed => LedCmd::On,
        ButtonEvent::Released => LedCmd::Off,
    });
    let mut log_button = Flow::new(|x| match x {
        ButtonEvent::Pressed => hprintln!("==> pressed"),
        ButtonEvent::Released => hprintln!("==> released"),
    });

    button.actor.emit(&ButtonEvent::Pressed);
    button.actor.emit(&ButtonEvent::Released);
    let _ = &button.actor >> &log_button.actor;
    //   let _ = &button.actor >> &pressed_led_on.actor >> &led.actor;
    loop {
        // hprintln!("loop");
        loop {
            select(button.run(), log_button.run()).await;
        }
        /*let _ = select4(
            pressed_led_on.run(),
            led.run(),
            select(mqtt.run(), log_button.run()),
            button.run(),
        )
        .await;*/
    }
}

// #[cortex_m_rt::entry]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    heap_setup();
    hprintln!("main started");
    // pipes defined here so they will be destroyed after actors are destroyed, stupid...
    block_on(test());
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
    let mut switch2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();
    let mut switch1 = portf.pf4.into_pull_up_input();
    switch1.set_interrupt_mode(gpio::InterruptMode::LevelLow);
    switch2.set_interrupt_mode(gpio::InterruptMode::LevelLow);

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
    // let mut led_actor = led::Led::new(&mut pin_red);
    //  led_actor.active_sink.on(true);
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
