use core::cell::Cell;
use core::convert::TryInto;
use core::sync::atomic::{compiler_fence, AtomicU32, AtomicU8, Ordering};
use core::{mem, ptr};
use core::cell::UnsafeCell;
use core::num::Wrapping;

use cortex_m::interrupt;
use cortex_m::peripheral::syst;
use cortex_m::peripheral::{syst::SystClkSource, SYST};

use cortex_m_rt::exception;

use cortex_m_semihosting::hprintln;
use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::driver::{AlarmHandle, Driver};
use embassy_time::Instant;
use embassy_time::TICK_HZ;

pub struct PollingSysTick {
    syst: UnsafeCell<SYST>,
    counter: UnsafeCell<u32>,
}
static mut POLLING_SYSTICK: PollingSysTick = PollingSysTick { syst: UnsafeCell,counter: UnsafeCell };


#[exception]
fn SysTick() {
    unsafe {
        POLLING_SYSTICK.on_interrupt();
    }
    DRIVER.on_interrupt();
}


impl PollingSysTick {
    /// Configures SysTick based on the values provided in the calibration.
    pub fn new(mut syst: SYST, calibration: u32) -> Self {
        syst.disable_interrupt();
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload(calibration - 1);
        syst.enable_counter();

        PollingSysTick {
            syst: UnsafeCell::new(syst),
            counter: UnsafeCell::default(),
        }
    }

    /// Turns this value back into the underlying SysTick.
    pub fn free(self) -> SYST {
        self.syst.into_inner()
    }

    pub fn init(mut systick: SYST, sysclk: u32,tick_rate:u32)  {
        // + TIMER_HZ / 2 provides round to nearest instead of round to 0.
        // - 1 as the counter range is inclusive [0, reload]
        let reload = (sysclk + tick_rate / 2) / tick_rate - 1;

        assert!(reload <= 0x00ff_ffff);
        assert!(reload > 0);

        systick.disable_counter();
        systick.set_clock_source(SystClkSource::Core);
        systick.set_reload(reload);
        systick.enable_interrupt();
        systick.enable_counter();

        unsafe { POLLING_SYSTICK.syst = UnsafeCell::new(systick);};
    }

    pub fn now(& self) -> Instant {
        if self.syst.into_inner().has_wrapped() {
            self.counter.get_mut().0 += 1;
        }
        let ts = (unsafe { (*SYST::PTR).cvr.read() } / 80_000) as u64;
        Instant::from_ticks(self.counter.into_inner() as u64 * TICK_HZ + ts)
    }

    unsafe fn reset(&mut self) {
        *(self.syst.get()).clear_current();
        self.systick.unwrap().enable_counter();
    }

    #[inline(always)]
    fn set_compare(&mut self, _val: Instant) {
        // No need to do something here, we get interrupts anyway.
    }

    #[inline(always)]
    fn clear_compare_flag(&mut self) {
        // NOOP with SysTick interrupt
    }

    #[inline(always)]
    fn zero() -> Instant {
        Instant::from_ticks(0)
    }

    #[inline(always)]
    fn on_interrupt(&mut self) {
        if self.systick.unwrap().has_wrapped() {
            self.clock = self.clock.wrapping_add(1);
        }
    }
}

const ALARM_COUNT: usize = 3;

fn calc_now(period: u32, counter: u16) -> u64 {
    ((period as u64) << 15) + ((counter as u32 ^ ((period & 1) << 15)) as u64)
}

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
    fn init(&'static self, cs: critical_section::CriticalSection) {}

    fn on_interrupt(&self) {
        hprintln!("SystickDriver::on_interrupt()");
        critical_section::with(|cs| {
            for n in 0..ALARM_COUNT {
                let alarm = &self.alarms.borrow(cs)[n];
                let at = alarm.timestamp.get();
                if at != u64::MAX {
                    let t = self.now();
                    if t >= at {
                        self.trigger_alarm(n, cs);
                    }
                }
            }
        })
    }

    fn next_period(&self) {
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

    fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
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
        CLOCK_COUNTER.now().as_ticks()
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
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
        critical_section::with(|cs| {
            let alarm = self.get_alarm(cs, alarm);
            alarm.callback.set(callback as *const ());
            alarm.ctx.set(ctx);
        })
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        critical_section::with(|cs| {
            let n = alarm.id() as usize;
            let alarm = self.get_alarm(cs, alarm);
            alarm.timestamp.set(timestamp);
            let t = self.now();
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

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs)
}
