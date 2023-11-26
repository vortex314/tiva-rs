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

extern crate alloc;


use alloc::boxed::Box;
use cortex_m_rt::exception;
use tm4c_hal::{gpio, serial};

use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_executor::Spawner;

use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c123x_hal::{self as hal, prelude::*};

use log::{info, Level};
use panic_semihosting as _;

mod limero;
use limero::*;

mod led;
use led::*;
mod button;
use button::*;
mod semi_logger;
use semi_logger::*;
mod timer_driver;
use timer_driver::Clock;
mod echo;
use echo::*;

mod serial_logger;

use embassy_executor::Executor;
// use serial_logger::serial_logger_init;

#[embassy_executor::task]
async fn test_task() {
    let mut echo = Echo::new(1000000);
   let _ = &echo.actor >> &echo.actor;
   echo.run().await;
}
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    heap_setup();
    hprintln!("main started");
 //   semi_logger_init().unwrap();

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
    let (mut tx, mut rx) = uart.split();
    hprintln!("switching to serial logger");
    let _x = SerialLogger::init(Level::Info,Box::new(tx));
    let _r = spawner.spawn(test_task());


    let mut portf = peripherals.GPIO_PORTF.split(&sysctl.power_control);

    let mut pin_red = portf.pf1.into_push_pull_output();
    let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();
    let mut switch2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();
    let mut switch1 = portf.pf4.into_pull_up_input();
    switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
    switch2.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
    info!(" switch1 interrupt status : {}",switch1.get_interrupt_status());

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
    let mut button = Button::new(switch1);
    let mut button2 = Button::new(switch2);

    let mut led_red = Led::new(pin_red);
    let mut led_blue = Led::new(pin_blue);
    let mut led_green = Led::new(pin_green);

    let mut pressed_led_on = Mapper::new(|x| match x {
        ButtonEvent::Pressed => Some(LedCmd::Blink(500)),
        ButtonEvent::Released => Some(LedCmd::Blink(50)),
    });
    let mut log_button = Sink::new(|x| match x {
        ButtonEvent::Pressed => info!("button pressed"),
        ButtonEvent::Released => info!("button released"),
    });

    let mut log_button2 = Sink::new(|x| match x {
        ButtonEvent::Pressed => info!("button2 pressed"),
        ButtonEvent::Released => info!("button2 released"),
    });

    let mut pressed2_led_on = Mapper::new(|x| match x {
        ButtonEvent::Pressed => Some(LedCmd::On),
        ButtonEvent::Released => Some(LedCmd::Off),
    });

    let _ = &button.actor >> &log_button;
    let _ = &button2.actor >> &log_button2;
    let _ = &button2.actor >> &pressed2_led_on >> &led_green.actor;
    let _ = &button.actor >> &pressed_led_on >> &led_red.actor;
    info!("main loop started");
    loop {
        embassy_futures::select::select4(button.run(), led_red.run(),button2.run(),led_green.run()).await;
    }
}

//use alloc::fmt::format;

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    hprintln!("{:#?}", ef);
    loop {}
}

// HEAP allocation for embedded
use embedded_alloc::Heap;
use void::Void;

use crate::serial_logger::*;

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
const HEAP_SIZE: usize = 10240; // in bytes
fn heap_setup() {
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) } // 👈
}
