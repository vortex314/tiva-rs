#![no_std]
//#![feature(noop_waker)]
#![no_main]
#![allow(deprecated)]
//#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]

use core::convert::Infallible;
use core::fmt::Write;
// use std::process::Output;
// use alloc::task;

use lilos::time::TickTime;
use tm4c123x as device;

use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c123x_hal::{self as hal, prelude::*};
use tm4c123x_hal::gpio::gpioa::PA0;
use tm4c123x_hal::gpio::gpioa::PA1;
use tm4c123x_hal::gpio::AlternateFunction;
use tm4c123x_hal::gpio::PushPull;
use tm4c123x_hal::gpio::Output;
use tm4c123x_hal::gpio::AF1;
use tm4c123x_hal::serial::Serial;

use core::panic::PanicInfo;
use alloc::string::String;

use serde::ser::SerializeSeq;
use serde::Serializer;
use serde_json_core::ser::Serializer as Ser;

extern crate alloc;
use core::option::Option::Some;


use lilos::exec::PeriodicGate;
use thingbuf as conn;


#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
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
use limero::{TimeClient, TIME_SERVER};
use limero::TimerServer;



#[cortex_m_rt::entry]
fn main() -> ! {
    heap_setup();
    unsafe { TIME_SERVER= Some(TimerServer::new()); };

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
    lilos::time::initialize_sys_tick(&mut cortex_peripherals.SYST, 80_000_000);

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

    let mut led = led::Led::new(& mut pin_red);
    let led_task = core::pin::pin!(led.run());

 //   let blink = core::pin::pin!(blink_led(pin_red));

    let uart_sender = core::pin::pin!(uart_sender(uart));
    lilos::exec::run_tasks(
        &mut [ led_task], // <-- array of tasks
        lilos::exec::ALL_TASKS,              // <-- which to start initially
    );
}

fn crc_calc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for b  in data.iter() {
        crc ^= *b as u16;
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
    const PERIOD: lilos::time::Millis = lilos::time::Millis(500);
    let mut gate = PeriodicGate::from(PERIOD);
    let tick_time = TickTime::now();

    loop {
        let buffer: &mut [u8; 100] = &mut [0u8; 100];
        let mut serializer = Ser::new(buffer);
        let mut seq = serializer.serialize_seq(Some(3)).unwrap();
        let _s = String::from("pub");
        seq.serialize_element("pub").unwrap();
        seq.serialize_element("src/tiva/sys/loopback").unwrap();
        let u = tick_time.elapsed().0 ;
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
        let _ = seq.end();
        serializer.end();
        let crc = crc_calc(buffer);
        tx.write_all(&buffer);
        tx.write_fmt(format_args!("{:04X}\r\n", crc)).unwrap();
        gate.next_time().await;
    }
}

async fn blink_led(mut pin_red: hal::gpio::gpiof::PF1<Output<PushPull>>) -> Infallible {
    const PERIOD: lilos::time::Millis = lilos::time::Millis(500);
    let mut gate = PeriodicGate::from(PERIOD);
    loop {
        gate.next_time().await;
        pin_red.set_high();
        gate.next_time().await;
        pin_red.set_low();
    }
}