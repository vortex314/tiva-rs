/*
SemiHosting logger to debug port of probe 
*/

use alloc::string::ToString;
use cortex_m_semihosting::hprintln;

use crate::timer_driver::msec;
use log::{Level, LevelFilter, Metadata, Record, SetLoggerError};

pub struct SemiLogger;

pub static SEMI_LOGGER: SemiLogger = SemiLogger;

pub fn semi_logger_init() -> Result<(), SetLoggerError> {
    log::set_logger(&SEMI_LOGGER).map(|()| log::set_max_level(LevelFilter::Info))
}

impl log::Log for SemiLogger {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        metadata.level() >= Level::Info
    }

    fn log(&self, record: &Record<'_>) {
        if self.enabled(record.metadata()) {
            let s = record.args().to_string();
            let s1 = s.as_str();
            hprintln!(
                "[{:1.1}] {:6.6} {:10.10}:{:4.4}|{}",
                record.level().as_str(),
                msec(),
                record.file().unwrap_or("unknown"),
                record.line().unwrap_or(0),
                s
            );
        }
    }

    fn flush(&self) {}
}
