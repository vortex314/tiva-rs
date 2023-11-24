use alloc::string::ToString;
use cortex_m_semihosting::hprintln;

use log::{Level, LevelFilter, Metadata, Record, SetLoggerError};
use crate::timer_driver::msec;


struct SimpleLogger;

impl log::Log for SimpleLogger {
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

static LOGGER: SimpleLogger = SimpleLogger;

pub fn logger_init() -> Result<(), SetLoggerError> {
    log::set_logger(&LOGGER).map(|()| log::set_max_level(LevelFilter::Info))
}
