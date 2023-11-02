#![no_std]
#![feature(type_alias_impl_trait)]
//#![feature(noop_waker)]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]

use alloc::boxed::Box;
use core::any::Any;
use core::cell::UnsafeCell;
use core::convert::Infallible;
use core::fmt::Write;
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
use thingbuf as conn;

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

// #[cortex_m_rt::entry]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
    static mut uart_actor:UnsafeCell<Uart> = UnsafeCell::new(Uart::new(&mut rx, &mut tx));
    let mut serde_actor = SerdeActor::new();
    &mut serde_actor.txd_source >> uart_actor.txd_sink;
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
