#![no_std]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
use core::fmt::Write;
use core::ops::Deref;
use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use embedded_alloc::Heap;

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use tm4c123x_hal::eeprom::{
    Blocks, Eeprom, EepromAddress, EepromError, Erase, Read, Write as EepromWrite,
};
use tm4c123x_hal::{self as hal, prelude::*};

// use postcard::{from_bytes, to_vec};
use core::alloc::GlobalAlloc;
use core::alloc::Layout;
use core::ffi::c_void;
use heapless::LinearMap;
use heapless::String;
use heapless::Vec;
use core::panic::PanicInfo;

use serde::{Deserialize, Serialize};
use serde_json_core::{from_slice, to_vec};
#[macro_use]
extern crate alloc;

#[global_allocator]
static HEAP: Heap = Heap::empty();
/*#[panic_handler]
fn panic(_x: &PanicInfo) -> ! {
    loop {}
}*/

#[derive(Serialize, Deserialize, Debug)]
enum WireMsg {
    Publish {
        topic: &'static str,
        payload: &'static str,
    },
    Subscribe {
        topic: &'static str,
    },
    Unsubscribe {
        topic: &'static str,
    },
}

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 10240;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
    let p = hal::Peripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    let mut portf = p.GPIO_PORTF.split(&sc.power_control);
    let mut pin_red = portf.pf1.into_push_pull_output();
    let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();
    //let mut ctrl = portf.control;
    let switch2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();
    let switch1 = portf.pf4.into_pull_up_input();

    let mut porta = p.GPIO_PORTA.split(&sc.power_control);
    let publish_msg = WireMsg::Publish {
        topic: "test",
        payload: "test",
    };

    let mut array = Vec::<&str,3>::new();

    let buffer:Vec<u8,100> = to_vec(&["pub","topic","value"]).unwrap();

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
        &sc.power_control,
    );

    let (mut tx, _rx) = uart.split();

    let mut counter = 0u32;
    loop {
        tx.write_fmt(format_args!("Hello, world! {}  \r\n", counter))
            .unwrap();
        //       tx.write_all(&msg);
        tx.write_char('\n' as u8 as char).unwrap();
        tx.write_all(&buffer);

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
