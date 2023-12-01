#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(noop_waker)]
#![no_main]
#![allow(deprecated)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_mut)]
#![feature(const_type_id)]
#![deny(elided_lifetimes_in_paths)]

extern crate alloc;

use core::cell::RefCell;
use core::default;
use core::pin::Pin;
use core::task::{Context, Poll};

use alloc::borrow;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::String;
use alloc::vec::{self, Vec};
use cortex_m::interrupt;
use cortex_m::interrupt::enable;
use cortex_m::peripheral::{nvic, NVIC};
use cortex_m_rt::exception;

use cortex_m_semihosting::nr::open::R;
use embassy_sync::pubsub::publisher;
use futures::Future;
use tm4c123x::Interrupt;
use tm4c_hal::{gpio, serial};

use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hprintln;
use embassy_executor::Spawner;

use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::sysctl::SysctlExt;
use tm4c123x_hal::{self as hal, prelude::*};

use log::{info, Level};
use panic_semihosting as _;

mod limero;
use limero::*;

mod led;
use led::*;
mod button;
use button::*;

mod echo;
use echo::*;
mod semi_logger;
use semi_logger::*;
mod timer_driver;
use timer_driver::Clock;
mod serial_logger;
mod topic_router;
use topic_router::*;

use embassy_executor::Executor;
// use serial_logger::serial_logger_init;
/*

ActorRef -> ActorWrapper -> Actor
tell -> on -> queue
ask -> on -> queue -> poll -> future
emit -> on -> queue -> poll -> future

CMD<Messages> = enum { Message(Messages),sender:ActorRef,reply:ActorRef,context:Context }

let led = ActorRef::new(Led::new(pinA1));
let button = ActorRef::new(Button::new(pinA2));
let mqtt = ActorRef::new(Mqtt::new());

led >> button ;
mqtt.tell(RouteReq("dst/+/object/prop",object_actor));
mqtt >> filter(MqttMsg::Publish("dst/+/led/left",x))

*/

/*#[embassy_executor::task]
async fn test_task() {
    let mut echo = Echo::new(1000000);
    let _ = &echo.actor >> &echo.actor;
    echo.run().await;
}
*/
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    heap_setup();

    hprintln!("main started");
    //   semi_logger_init().unwrap();

    let mut peripherals = hal::Peripherals::take().unwrap();
    let mut sysctl = peripherals.SYSCTL.constrain();
    let mut porta_parts = peripherals.GPIO_PORTA.split(&sysctl.power_control);
    sysctl.clock_setup.oscillator = hal::sysctl::Oscillator::Main(
        hal::sysctl::CrystalFrequency::_16mhz,
        hal::sysctl::SystemClock::UsePll(hal::sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sysctl.clock_setup.freeze();
    let mut cortex_peripherals = cortex_m::Peripherals::take().unwrap();
    Clock::init_timer_driver(cortex_peripherals.SYST);

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
    let (mut tx, mut rx) = uart.split();
    hprintln!("switching to serial logger");
    let _x = SerialLogger::init(Level::Info, Box::new(tx));
    //  let _r = spawner.spawn(test_task());

    let mut portf = peripherals.GPIO_PORTF.split(&sysctl.power_control);

    let mut pin_red = portf.pf1.into_push_pull_output();
    let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();
    let mut switch2 = portf.pf0.unlock(&mut portf.control).into_pull_up_input();
    let mut switch1 = portf.pf4.into_pull_up_input();
    switch1.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
    let mut reg = portf.control;
    unsafe {
        enable();
    };
    unsafe {
        NVIC::unmask(Interrupt::GPIOF);
    };
    switch2.set_interrupt_mode(gpio::InterruptMode::EdgeBoth);
    info!(
        " switch1 interrupt status : {}",
        switch1.get_interrupt_status()
    );

    /*let mut pin_blue = portf.pf2.into_push_pull_output();
    let mut pin_green = portf.pf3.into_push_pull_output();

    let porte = peripherals.GPIO_PORTE.split(&sysctl.power_control);
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
    });*/
    let mut button = ActorRef::new(Box::new(Button::new(switch1)), 2);
    let mut button2 = ActorRef::new(Box::new(Button::new(switch2)), 2);

    let mut led_red = ActorRef::new(Box::new(Led::new(pin_red)), 3);
    let mut led_blue = ActorRef::new(Box::new(Led::new(pin_blue)), 3);
    let mut led_green = ActorRef::new(Box::new(Led::new(pin_green)), 3);

    // let mut mqtt = ActorRef::new(Box::new(Mqtt::new()),1);
    // let mut topic_router = ActorRef::new(Box::new(TopicRouter::new()),1);
    // mqtt >> map(|x| match x { MqttMsg::Publish(topic,msg) => Some(RouteReq(topic,msg)) } ) >> topic_router;
    #[derive(Clone, Default)]
    enum Msg<'a> {
        #[default]
        Start,
        Publish(&'a str, &'a str),
    }

    let msg = Msg::Publish("topic", "aaaaa");

    if let Msg::Publish("str", msg) = msg {
        info!("subscribe {}", msg);
    }

    let led_blue_clone = led_blue.clone();
    let led_red_clone = led_red.clone();
    let f =   move |msg | match msg {
        Msg::Publish("led_blue", msg) => {
            led_blue_clone.tell(&LedCmd::On);
        }
        Msg::Publish("led_red", msg) => {
            led_red_clone.tell(&LedCmd::On);
        }
        _ => {}
    };

   
    let z  = |msg:&_| -> () { match msg {
        Msg::Publish("led_blue", msg) => {
            led_blue.tell(&LedCmd::On);
        }
        Msg::Publish("led_red", msg) => {
            led_red.tell(&LedCmd::On);
        }
        _ => {}
    }};

    let sink = Sink::new(z );

    let func =  |msg| match msg {
        Msg::Publish("led_blue", msg) => {
            led_blue.tell(&LedCmd::On);
        }
        Msg::Publish("led_red", msg) => {
            led_red.tell(&LedCmd::On);
        }
        _ => {}
    };

    let mut pressed_led_on = Mapper::new(|x| match x {
        ButtonEvent::Pressed => Some(LedCmd::Blink(500)),
        ButtonEvent::Released => Some(LedCmd::Blink(50)),
    });
    let mut log_button = Sink::new(|x| match x {
        ButtonEvent::Pressed => info!("button pressed"),
        ButtonEvent::Released => info!("button released"),
    });

    let mut log_button2 = Sink::new(|x| match x {
        ButtonEvent::Pressed => info!("button2 pressed"),
        ButtonEvent::Released => info!("button2 released"),
    });

    let mut pressed2_led_on = Mapper::new(|x| match x {
        ButtonEvent::Pressed => Some(LedCmd::On),
        ButtonEvent::Released => Some(LedCmd::Off),
    });
    // &mqtt.actor >> Mapper::new( |event| match event { Message("led_interval",x) => Some(LedCmd::Blink(x))} ) >> &led.actor; ));
    // let led_actor = ActorRef::<LedCmd,NoEvent>::new(led);
    // impl Actor for Led {
    //     type Cmd = LedCmd;
    //     type Event = NoEvent;
    //     fn on(&mut self, cmd: &LedCmd) -> Option<NoEvent> {
    //         match cmd {
    //             LedCmd::On => {
    //                 self.pin.set_high();
    //                 Some(NoEvent)
    //             }

    unsafe {
        cortex_m::interrupt::enable();
    };

    let _ = &button >> log_button;
    let _ = &button2 >> log_button2;
    let _ = &button2 >> &pressed2_led_on >> &led_green;
    let _ = &button >> &pressed_led_on >> &led_red;
    info!("main loop started");
    loop {
        embassy_futures::select::select4(
            button.run(),
            led_red.run(),
            button2.run(),
            led_green.run(),
        )
        .await;
    }
}

//use alloc::fmt::format;

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    hprintln!("{:#?}", ef);
    loop {}
}

// HEAP allocation for embedded
use embedded_alloc::Heap;
use void::Void;

use crate::serial_logger::*;

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();
const HEAP_SIZE: usize = 10240; // in bytes
fn heap_setup() {
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) } // 👈
}
