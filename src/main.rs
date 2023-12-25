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
//#![deny(elided_lifetimes_in_paths)]

extern crate alloc;

use core::cell::RefCell;
use core::default;
use core::pin::{pin, Pin};
use core::task::{Context, Poll};

use alloc::borrow;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::String;
use alloc::vec::{self, Vec};
use cortex_m::interrupt;
use cortex_m::interrupt::enable;
use cortex_m::peripheral::{nvic, NVIC};
use cortex_m_rt::exception;

use cortex_m_semihosting::nr::open::R;
use embassy_sync::pubsub::publisher;
use futures::Future;
use tm4c123x::Interrupt;
use tm4c_hal::{gpio, serial};

use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_executor::Spawner;

use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c123x_hal::{self as hal, prelude::*};

use log::{info, warn, Level};
use panic_semihosting as _;

mod limero;
use limero::*;

mod led;
use led::*;
mod button;
use button::*;

mod echo;
use echo::*;
mod semi_logger;
use semi_logger::*;
mod timer_driver;
use timer_driver::Clock;
mod serial_logger;

use embassy_executor::Executor;
// use serial_logger::serial_logger_init;

/*#[embassy_executor::task]
async fn test_task() {
    let mut echo = Echo::new(1000000);
    let _ = &echo.actor >> &echo.actor;
    echo.run().await;
}
*/
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
    let _x = SerialLogger::init(Level::Info, Box::new(tx));
    //  let _r = spawner.spawn(test_task());

    let mut portf = peripherals.GPIO_PORTF.split(&sysctl.power_control);

    let mut pin_red = portf.pf1.into_push_pull_output();
    let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();
    let mut switch2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();
    let mut switch1 = portf.pf4.into_pull_up_input();
    switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
    let mut reg = portf.control;
    unsafe {
        enable();
    };
    unsafe {
        NVIC::unmask(Interrupt::GPIOF);
    };
    switch2.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
    info!(
        " switch1 interrupt status : {}",
        switch1.get_interrupt_status()
    );

    /*let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();

    let porte = peripherals.GPIO_PORTE.split(&sysctl.power_control);
    let mut pin_uext1 = porte.pe2.into_push_pull_output();
    let mut pin_uext2 = porte.pe3.into_push_pull_output();
    pin_uext1.set_low();
    pin_uext2.set_low();
    */
    let mut pressed_led_on = Mapper::new(move |x| match x {
        ButtonEvent::Pressed => LedCmd::Blink(500),
        ButtonEvent::Released => LedCmd::Blink(50),
    });
    {
        let mut button_1 = Button::new(switch1);
        let mut button_2 = Button::new(switch2);

        let mut led_red = Led::new(pin_red, 1);
        let mut led_blue = Led::new(pin_blue, 1);
        let mut led_green = Led::new(pin_green, 1);

        /*let mut log_button = Sink::new(|x| match x {
            ButtonEvent::Pressed => info!("button pressed"),
            ButtonEvent::Released => info!("button released"),
        });*/
        unsafe {
            cortex_m::interrupt::enable();
        };

        let mapper2 = pressed_led_on.clone();

        pressed_led_on.add_handler(led_red.handler());
        button_1.add_handler(Box::new(pressed_led_on));
        link(&mut button_1, &mapper2);
        //link(&mut button_2, & pressed_led_on);

        info!("main loop started");
        embassy_futures::select::select(led_blue.run(), led_green.run()).await;
    }
    warn!("Stopped. Shouldn't have happened");
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
