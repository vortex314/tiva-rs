use alloc::boxed::Box;
use core::fmt::Write;

use cortex_m_semihosting::hprintln;
use embedded_hal::serial::Write as SerialWrite;
use log::{Level, Log, Metadata, Record};
use tm4c123x_hal::prelude::_embedded_hal_serial_Write;
use void::Void;
use crate::timer_driver::msec;
use crate::alloc::string::ToString;
use nb::block;

pub struct SerialLogger<E> {
    pub level: Level,
    pub writer: Option<Box<dyn SerialWrite<u8, Error = E>>>,
}

unsafe impl<E> Sync for SerialLogger<E> {}
unsafe impl<E> Send for SerialLogger<E> {}

static mut SERIAL_LOGGER: SerialLogger<Void> = SerialLogger {
    level: Level::Info,
    writer: None,
};

impl SerialLogger<Void> {
    pub fn init(level: Level, mut w: Box<dyn SerialWrite<u8, Error = Void>>) {
        unsafe {
            w.write_str("[INFO] Serial logger initialized\r\n").unwrap();

            // Set global SERIAL_LOGGER log context
            SERIAL_LOGGER.level = level;
            SERIAL_LOGGER.writer = Some(w);

            // Attach to logger
            let x = log::set_logger(&SERIAL_LOGGER);
            hprintln!("log::set_logger: {:?}", x);
            log::set_max_level(SERIAL_LOGGER.level.to_level_filter());
        }
    }
}

impl<E> Log for SerialLogger<E> {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        metadata.level() >= self.level
    }

    fn log(&self, record: &Record<'_>) {
        if self.enabled(record.metadata()) {
            let _ = unsafe {
                match record.module_path() {
                    Some(p) => {
                        let s = record.args().to_string();
                        let s1 = s.as_str();
                        // write!(SERIAL_LOGGER, "[{}] {} - {}\r\n", record.level(), p, record.args()),
                        write!(
                            SERIAL_LOGGER,
                            "[{:1.1}] {:6.6} {:10.10}:{:4.4}|{}\r\n",
                            record.level().as_str(),
                            msec(),
                            record.file().unwrap_or("unknown"),
                            record.line().unwrap_or(0),
                            s
                        )
                    },
                    None => write!(
                        SERIAL_LOGGER,
                        "[{}] - {}\r\n",
                        record.level(),
                        record.args()
                    ),
                }
            };
        }
    }

    fn flush(&self) {}
}

impl<E> core::fmt::Write for SerialLogger<E> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Fetch writer (if bound)
        let w = match &mut self.writer {
            Some(w) => w,
            None => return Err(core::fmt::Error),
        };
        // Output string as bytes
        for b in s.as_bytes() {
            block!(w.write(*b)).map_err(|_| core::fmt::Error)?;
        }
        // Flush output buffers
        block!(w.flush()).map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}
