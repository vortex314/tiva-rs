#![no_std]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
#![feature(type_alias_impl_trait)]

// #![feature(type_alias_impl_trait)]
/*
Cargo.toml
embassy-executor={version="0.3.0", features = ["arch-cortex-m","executor-thread", "executor-interrupt","nightly"]}
# embassy-time = {version="0.1.3", features = ["tick-hz-1_000"]}
embassy-sync = {version="0.3.0", features = []}
 */

mod limero;

use core::fmt::Write;
use core::ops::Deref;
use cortex_m::delay;
use cortex_m::peripheral::{DWT, SCB};
use cortex_m_rt::entry;
use embedded_alloc::Heap;

use heapless::pool::Box;
use panic_halt as _;
use serde::ser::SerializeSeq;

use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::eeprom::{
    Blocks, Eeprom, EepromAddress, EepromError, Erase, Read, Write as EepromWrite,
};
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c123x_hal::time::{Instant, MonoTimer};
use tm4c123x_hal::{self as hal, prelude::*};

use core::alloc::GlobalAlloc;
use core::alloc::Layout;
use core::ffi::c_void;
use core::panic::PanicInfo;
use heapless::LinearMap;
use heapless::String;
use heapless::Vec;

use serde::Serializer;
use serde_json_core::ser::Serializer as Ser;

#[macro_use]
extern crate alloc;

#[global_allocator]
static HEAP: Heap = Heap::empty();

fn heap_alloc() {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 10240;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
}
/*#[panic_handler]
fn panic(_x: &PanicInfo) -> ! {
    loop {}
}*/

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

use embassy_executor::*;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;

#[embassy_executor::task]
async fn run() -> ! {
    loop {}
}
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    heap_alloc();
    //   _spawner.spawn(run()).unwrap();
    let mut channel = Channel::<NoopRawMutex, u32, 3>::new();
    let mut tx = channel.sender();
    let mut rx = channel.receiver();

    async {
        let mut x = rx.receive().await;
        assert_eq!(x, 123);
        x = rx.receive().await;
        assert_eq!(x, 456);
    }

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

/*   let mut eeprom = Eeprom::new(p.EEPROM, &sc.power_control);

match eeprom_test_all(&mut eeprom) {
    Ok(_) => {
        // Huzzah!
    }
    Err(code) => {
        panic!("Error detected while testing EEPROM: {}", code);
    }
}*/
/*
pub fn eeprom_test_write_read(
    eeprom: &mut Eeprom,
    address: &EepromAddress,
    data_to_write: &[u8],
    read_buffer: &mut [u8],
) -> Result<(), EepromError> {
    eeprom.write(address, &data_to_write)?;
    eeprom.read(address, data_to_write.len(), read_buffer)?;

    for (i, byte) in data_to_write.iter().enumerate() {
        assert_eq!(*byte, read_buffer[i], "Read data differs from written data");
    }

    Ok(())
}

pub fn eeprom_test_all(eeprom: &mut Eeprom) -> Result<(), EepromError> {
    let mut buffer = [0 as u8; 64]; // 64 byte read buffer

    // Sanity check for simple mapping from word offset to an EepromAddress
    let mut address = eeprom.word_index_to_address(52).unwrap();
    assert_eq!(address.block(), 3, "Word 52 should be in block 3, offset 4");
    assert_eq!(
        address.offset(),
        4,
        "Word 52 should be in block 3, offset 4"
    );

    // Sanity check for EepromAddress to word offset
    let word_index = eeprom.address_to_word_index(&address).unwrap();
    assert_eq!(
        word_index, 52,
        "Word index for block 3, offset 4 should be 52"
    );

    // Simplest case, middle of a block, no straddle
    let test_array_1: [u8; 4] = [1, 2, 3, 4];
    eeprom_test_write_read(eeprom, &mut address, &test_array_1, &mut buffer)?;

    // Test boundry conditions for access that straddles a block
    let test_array_2: [u8; 10] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    let address_straddle_block = EepromAddress::new(0, 15);
    eeprom_test_write_read(eeprom, &address_straddle_block, &test_array_2, &mut buffer)?;

    eeprom.erase(&address_straddle_block, test_array_2.len())?;
    eeprom.read(&address_straddle_block, test_array_2.len(), &mut buffer)?;
    for i in 0..test_array_2.len() {
        assert_eq!(buffer[i], 0, "Buffer should be all 0's")
    }

    // Test the block erase using the straddle address and data
    eeprom.write(&address_straddle_block, &test_array_2)?;
    eeprom.erase_block(0)?;
    eeprom.read(&address_straddle_block, test_array_2.len(), &mut buffer)?;
    for i in 0..test_array_2.len() {
        match i {
            0..=3 => {
                assert_eq!(buffer[i], 0, "Buffer[0..3] should be all 0's");
            }
            _ => {
                assert_eq!(
                    buffer[i], test_array_2[i],
                    "Buffer[4..9] should match test_array_2"
                )
            }
        }
    }

    Ok(())
}

*/
