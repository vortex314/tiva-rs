#![no_std]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
#![feature(type_alias_impl_trait)]

mod limero;

use core::fmt::Write;
use core::ops::Deref;
use cortex_m::delay;
use cortex_m::peripheral::{DWT, SCB};
use cortex_m_rt::entry;
use embedded_alloc::Heap;

use heapless::pool::Box;
use serde::ser::SerializeSeq;

use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::eeprom::{
    Blocks, Eeprom, EepromAddress, EepromError, Erase, Read, Write as EepromWrite,
};
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::interrupt::*;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c123x_hal::time::{Instant, MonoTimer};
use tm4c123x_hal::Peripherals;
use tm4c123x_hal::{self as hal, prelude::*};

use core::alloc::GlobalAlloc;
use core::alloc::Layout;
use core::ffi::c_void;
use core::panic::PanicInfo;


use serde::Serializer;
use serde_json_core::ser::Serializer as Ser;

use tm4c123x_hal::prelude::*;
mod tm4c123x;

use nb_executor::{Executor,Signals,Events,EventMask,Mpmc,Park,Parked};
use futures::{FutureExt,Future,};
use futures::task::{Context, Poll,ArcWake};
use futures::join;
use bitflags::bitflags;
// use futures::select;
//use core::future::join;
use tm4c123x::*;
use heapless::{Arc,Vec};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

fn park_test(park: Park<'_>) -> Parked {
    let parked = park.race_free();
    assert!(!parked.is_idle());
    parked
}

bitflags! {
    struct Ev: u32 {
        const A = 1 << 3;
        const B = 1 << 11;
        const C = 1 << 17;
        const ALL = Self::A.bits | Self::B.bits | Self::C.bits;
    }
}

impl EventMask for Ev {
    fn as_bits(self) -> u32 {
        self.bits()
    }
}

impl ArcWake for Events<Ev> {
    fn wake_by_ref(arc_self: &Arc<Self>) {
        arc_self.raise(Ev::ALL)
    }
}

#[entry]
fn main() -> ! {
    let events = Events::default();
    let signals = events.watch();

    let queue = Mpmc::<_, 4>::new();

    let producer = async {
        for n in 0..32 {
            queue.enqueue(n, &signals, Ev::A).await;
        }
    };

    let consumer = async {
        for n in 0..32 {
            assert_eq!(queue.dequeue(&signals, Ev::A).await, n);
        }

        assert_eq!(queue.inner().dequeue(), None);
    };

    let future = async { join!(producer, consumer) };
    signals.bind().block_on(future, park_test);

    let p = hal::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut sysctl = p.SYSCTL.constrain();
    sysctl.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sysctl.clock_setup.freeze();

    let mut portf = p.GPIO_PORTF.split(&sysctl.power_control);
    let porte = p.GPIO_PORTE.split(&sysctl.power_control);
    let mut pin_red = portf.pf1.into_push_pull_output();
    let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();
    let mut pin_uext1 = porte.pe2.into_push_pull_output();
    let mut pin_uext2 = porte.pe3.into_push_pull_output();
    pin_uext1.set_low();
    pin_uext2.set_low();
    //let mut ctrl = portf.control;
    let switch2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();
    let switch1 = portf.pf4.into_pull_up_input();

    let mut porta = p.GPIO_PORTA.split(&sysctl.power_control);
    let mut timer = Delay::new(cp.SYST, &clocks);
    let buffer = &mut [0u8; 100];

    // Activate UART
    let uart = hal::serial::Serial::uart0(
        p.UART0,
        porta
            .pa1
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        porta
            .pa0
            .into_af_push_pull::<hal::gpio::AF1>(&mut porta.control),
        (),
        (),
        115200_u32.bps(),
        hal::serial::NewlineMode::SwapLFtoCRLF,
        &clocks,
        &sysctl.power_control,
    );

    let (mut tx, _rx) = uart.split();

    let mut counter = 0u32;
    loop {
        let mut serializer = Ser::new(buffer);
        let mut seq = serializer.serialize_seq(Some(3)).unwrap();
        seq.serialize_element("pub").unwrap();
        seq.serialize_element("src/tiva/sys/loopback").unwrap();
        let u = counter;
        seq.serialize_element(&u).unwrap();
        let _ = seq.end();
        serializer.end();
        let crc = crc_calc(buffer);
        tx.write_all(&buffer);
        tx.write_fmt(format_args!("{:X}\r\n", crc)).unwrap();

        timer.delay_ms(1000u32);

        counter = counter.wrapping_add(1);
        if counter > 100000 {
            SCB::sys_reset();
        }

        if counter % 100 < 1 {
            pin_red.set_low();
            pin_blue.set_low();
            pin_green.set_high();
        };
        if switch1.is_low() {
            pin_red.set_high();
            pin_blue.set_low();
            pin_green.set_low();
        }
        if switch2.is_low() {
            pin_red.set_low();
            pin_blue.set_high();
            pin_green.set_low();
        }
    }
}

fn crc_calc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for i in 0..data.len() {
        crc ^= data[i] as u16;
        for _j in 0..8 {
            if crc & 1 == 1 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    crc
}
