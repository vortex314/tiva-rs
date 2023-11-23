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

mod timer_driver;
use timer_driver::Clock;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
use cortex_m_rt::exception;
use tm4c123x::interrupt;
use tm4c_hal::gpio;

use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_futures::block_on;
use embassy_executor::Spawner;
use embassy_time::driver::{AlarmHandle, Driver};

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
use panic_semihosting as _;
use log::info;
use serde_derive::Serialize;

mod limero;
use limero::Actor;
use limero::Listener;
use limero::Publisher;
use limero::Flow;
use limero::NoEvent;
use limero::NoCmd;


#[derive(Serialize)]
struct Test {
    x: u32,
    b: &'static str,
}

extern crate alloc;

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
            actor: Actor::new(10),
        }
    }
    async fn run(&mut self) {
        loop {
            let mut cmd = self.actor.recv().await;
            match cmd {
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
            actor: Actor::new(10),
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
            actor: Actor::new(0),
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

async fn test() {
    let mut mqtt = Mqtt::new();
    let mut button = Button::new();
    let mut led = Led::new();
    let mut pressed_led_on = Flow::new(|x| match x {
        ButtonEvent::Pressed => LedCmd::On,
        ButtonEvent::Released => LedCmd::Off,
    });
    let mut log_button = Flow::new(|x| match x {
        ButtonEvent::Pressed => info!("==> pressed"),
        ButtonEvent::Released => info!("==> released"),
    });

    /*let _z = &button.actor
    >> &map(|x| match x {
        ButtonEvent::Pressed => LedCmd::On,
        ButtonEvent::Released => LedCmd::Off,
    })
    >> &led.actor; */

    button.actor.emit(&ButtonEvent::Pressed);
    button.actor.emit(&ButtonEvent::Released);
    let _y = &button.actor >> &log_button.actor;
    let _x = &button.actor >> &pressed_led_on.actor >> &led.actor;
    loop {
        embassy_futures::join::join4(
            button.run(),
            log_button.run(),
            pressed_led_on.run(),
            led.run(),
        ).await;
    }
}


// #[cortex_m_rt::entry]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    heap_setup();
    hprintln!("main started");
    block_on(test());

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
    hprintln!("main loop started");

}
