use core::cell::Cell;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use core::{mem, ptr};

use cortex_m::peripheral::{syst::SystClkSource, SYST};

use cortex_m_rt::exception;

use cortex_m_semihosting::hprintln;
use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::driver::{AlarmHandle, Driver};


const RELOAD_VALUE: u32 = 80_000 - 1;

pub struct Clock {
    msec: u64,
}

pub static mut CLOCK: Clock = Clock { msec: 0 };
//use volatile_register::{RW, RO};
/* 
#[repr(C)]
struct SysTick {
    pub csr: RW<u32>,
    pub rvr: RW<u32>,
    pub cvr: RW<u32>,
    pub calib: RO<u32>,
}

fn get_systick() -> &'static mut SysTick {
    unsafe { &mut *(0xE000_E010 as *mut SysTick) }
}

fn get_time() -> u32 {
    let systick = get_systick();
    systick.cvr.read()
}
*/
pub fn usec() -> u64 {
    unsafe { CLOCK.now() }
}

pub fn msec() -> u64 {
    unsafe { CLOCK.msec }
}


impl Clock {
    fn now(&self) -> u64 {
        self.msec * 1000u64 + Self::get_current_usec() as u64
    }

    pub fn get_current_usec() -> u32 {
            let reg = unsafe { (*SYST::PTR).cvr.read() };
            (RELOAD_VALUE - reg) / 80
    }

    pub fn init_timer_driver(mut systick: SYST) {
        systick.disable_interrupt();
        systick.set_clock_source(SystClkSource::Core);
        systick.set_reload(RELOAD_VALUE);
        systick.enable_counter();
        systick.enable_interrupt();
    }
}

#[exception]
fn SysTick() {
    unsafe {
        CLOCK.msec += 1;
    }
    DRIVER.on_interrupt();
}

const ALARM_COUNT: usize = 1;

struct AlarmState {
    timestamp: Cell<u64>,

    // This is really a Option<(fn(*mut ()), *mut ())>
    // but fn pointers aren't allowed in const yet
    callback: Cell<*const ()>,
    ctx: Cell<*mut ()>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
            callback: Cell::new(ptr::null()),
            ctx: Cell::new(ptr::null_mut()),
        }
    }
}
pub(crate) struct SystickDriver {
    /// Number of 2^15 periods elapsed since boot.
    period: AtomicU32,
    alarm_count: AtomicU8,
    /// Timestamp at which to fire alarm. u64::MAX if no alarm is scheduled.
    alarms: Mutex<CriticalSectionRawMutex, [AlarmState; ALARM_COUNT]>,
}

const ALARM_STATE_NEW: AlarmState = AlarmState::new();

embassy_time::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    period: AtomicU32::new(0),
    alarm_count: AtomicU8::new(0),
    alarms: Mutex::const_new(CriticalSectionRawMutex::new(), [ALARM_STATE_NEW; ALARM_COUNT]),
});

impl SystickDriver {
    fn init(&'static self, cs: critical_section::CriticalSection<'static>) {}

    fn on_interrupt(&self) {
        let nu = self.now();

        critical_section::with(|cs| {
            for n in 0..ALARM_COUNT {
                let alarm = &self.alarms.borrow(cs)[n];
                let at = alarm.timestamp.get();
                if at != u64::MAX {
                    if nu >= at {
                        self.trigger_alarm(n, cs);
                    }
                }
            }
        })
    }

    fn next_period(&self) {
        hprintln!("next_period({} )", unsafe { CLOCK.now() });
        // We only modify the period from the timer interrupt, so we know this can't race.
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);
        let t = (period as u64) << 15;

        critical_section::with(move |cs| {
            for n in 0..ALARM_COUNT {
                let alarm = &self.alarms.borrow(cs)[n];
                let at = alarm.timestamp.get();
            }
        })
    }

    fn get_alarm<'a>(&'a self, cs: CriticalSection<'a>, alarm: AlarmHandle) -> &'a AlarmState {
        // safety: we're allowed to assume the AlarmState is created by us, and
        // we never create one that's out of bounds.
        unsafe { self.alarms.borrow(cs).get_unchecked(alarm.id() as usize) }
    }

    fn trigger_alarm(&self, n: usize, cs: CriticalSection<'_>) {
        let alarm = &self.alarms.borrow(cs)[n];
        alarm.timestamp.set(u64::MAX);

        // Call after clearing alarm, so the callback can set another alarm.

        // safety:
        // - we can ignore the possibility of `f` being unset (null) because of the safety contract of `allocate_alarm`.
        // - other than that we only store valid function pointers into alarm.callback
        let f: fn(*mut ()) = unsafe { mem::transmute(alarm.callback.get()) };
        f(alarm.ctx.get());
    }
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        /*let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        calc_now(period, 1)*/
        unsafe { CLOCK.now() }
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        hprintln!("allocate_alarm()");
        critical_section::with(|_| {
            let id = self.alarm_count.load(Ordering::Relaxed);
            if id < ALARM_COUNT as u8 {
                self.alarm_count.store(id + 1, Ordering::Relaxed);
                Some(AlarmHandle::new(id))
            } else {
                None
            }
        })
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
 //       hprintln!("set_alarm_callback({} ,{} )", callback as usize,alarm.id());
        critical_section::with(|cs| {
            let alarm = self.get_alarm(cs, alarm);
            alarm.callback.set(callback as *const ());
            alarm.ctx.set(ctx);
        })
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
 //       hprintln!("set_alarm({} ,{} )", timestamp,alarm.id());
        let t = self.now();
        critical_section::with(|cs| {
            let n = alarm.id() as usize;
            let alarm = self.get_alarm(cs, alarm);
            alarm.timestamp.set(timestamp);
            if timestamp <= t {
                // If alarm timestamp has passed the alarm will not fire.
                // Disarm the alarm and return `false` to indicate that.
                alarm.timestamp.set(u64::MAX);
                return false;
            }

            let safe_timestamp = timestamp.max(t + 3);
            true
        })
    }
}

pub(crate) fn init(cs: CriticalSection<'static>) {
    DRIVER.init(cs)
}
