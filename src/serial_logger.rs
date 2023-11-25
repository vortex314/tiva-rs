use core::{borrow::BorrowMut, cell::RefCell};

use crate::{semi_logger::SEMI_LOGGER, timer_driver::msec};
use alloc::{boxed::Box, rc::Rc, string::ToString, sync::Arc};
use embassy_sync::{
    blocking_mutex::{raw::NoopRawMutex, NoopMutex},
    mutex::Mutex,
};
use log::{Level, LevelFilter, Log, Metadata, Record, SetLoggerError};
use tm4c123x_hal::prelude::_embedded_hal_serial_Write;
use void::Void;

use embedded_hal::serial::Write as SerialWrite;

use lazy_static::lazy_static;

lazy_static! {
    static ref LOGGER: Mutex<NoopRawMutex, Box<dyn Log>> =
        Mutex::new(NoopMutex::new(Box::new(SEMI_LOGGER)));
}
struct SerialLogger {
    pub writer: Option<Box<dyn SerialWrite<u8, Error = Void>>>,
}
unsafe impl Sync for SerialLogger {}
unsafe impl Send for SerialLogger {}

pub fn serial_logger_init(
    writer: Box<dyn SerialWrite<u8, Error = Void>>,
) -> Result<(), SetLoggerError> {
    /*SERIAL_LOGGER.set(Mutex::new(SerialLogger {
        writer: Some(writer),
    }));
    log::set_logger(&SERIAL_LOGGER).map(|()| log::set_max_level(LevelFilter::Info))*/
Ok(())
}

impl log::Log for SerialLogger {
    fn enabled(&self, _: &Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &Record<'_>) {
        let line = alloc::format!(
            "[{}] {} {}:{}|{}",
            record.level().as_str(),
            msec(),
            record.file().unwrap_or("unknown"),
            record.line().unwrap_or(0),
            record.args()
        );

        let mut writer = self.writer.try_lock().unwrap();
        for byte in line.as_bytes() {
            writer.as_mut().unwrap().write(*byte);
        }
    }

    fn flush(&self) {}
}
