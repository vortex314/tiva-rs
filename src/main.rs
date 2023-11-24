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
use alloc::fmt::format;
use cortex_m_semihosting::hprint;
use embassy_time::Duration;
use embassy_time::Timer;
use log::Level;
use log::Record;
use log::SetLoggerError;
use timer_driver::Clock;
extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;
use core::cell::RefCell;
use cortex_m_rt::exception;
use tm4c123x::interrupt;
use tm4c_hal::gpio;

use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_executor::Spawner;
use embassy_futures::block_on;
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
use log::info;
use log::LevelFilter;
use log::Metadata;
use panic_semihosting as _;
use serde_derive::Serialize;

mod limero;
use limero::Actor;
use limero::Flow;
use limero::Listener;
use limero::Mapper;
use limero::NoCmd;
use limero::NoEvent;
use limero::Publisher;
use limero::Sink;

mod led;
use led::*;
mod button;
use button::*;

#[derive(Debug, Clone, Default)]
enum SysMsg {
    #[default]
    Start,
    Stop,
    Restart,
}

async fn test() {
    let mut button = Button::new();
    let mut led = Led::new();
    let mut pressed_led_on = Mapper::new(|x| match x {
        ButtonEvent::Pressed => Some(LedCmd::Blink(1000)),
        ButtonEvent::Released => Some(LedCmd::Blink(100)),
    });
    let mut log_button = Sink::new(|x| match x {
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

    let _ = &button.actor >> &log_button;
    let _ = &button.actor >> &pressed_led_on >> &led.actor;

    button.actor.add_listener(Box::new(pressed_led_on));
    loop {
        embassy_futures::join::join(button.run(), led.run()).await;
    }
}

// #[cortex_m_rt::entry]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    heap_setup();
    hprintln!("main started");
    logger_init().unwrap();

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

    test().await;

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

struct SimpleLogger;

impl log::Log for SimpleLogger {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        metadata.level() >= Level::Info
    }

    fn log(&self, record: &Record<'_>) {
        if self.enabled(record.metadata()) {
            let s = record.args().as_str().unwrap();
            hprint!("[{}] ", record.level().as_str());
            let t = msec();
            hprint!("{} : ", t);
            hprint!(s);
            hprint!("\r\n");
        }
    }

    fn flush(&self) {}
}

static LOGGER: SimpleLogger = SimpleLogger;

pub fn logger_init() -> Result<(), SetLoggerError> {
    log::set_logger(&LOGGER).map(|()| log::set_max_level(LevelFilter::Info))
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    hprintln!("{:#?}", ef);
    loop {}
}

// HEAP allocation for embedded
use embedded_alloc::Heap;

use crate::timer_driver::msec;
#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
const HEAP_SIZE: usize = 10240; // in bytes
fn heap_setup() {
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) } // 👈
}
