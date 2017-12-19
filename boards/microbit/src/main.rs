//! Tock kernel for the BBC microbit
//! This is an nRF51822 Soc (a Cortex M0 core with a BLE transceiver)
//!
//! ### Pin configuration
//! ## Edge connector
//! * 0 -> (pin 3)
//! * 1 -> (pin 2)
//! * 2 -> (pin 1)
//! * 3 -> (pin 4)
//! * 4 -> BUTTON0 (pin 5)
//! * 5 -> (pin 17)
//! * 6 -> (pin 12)
//! * 7 -> (pin 11)
//! * 8 -> (pin 18)
//! * 9 -> (pin 10)
//! * 10 -> (pin 6)
//! * 11 -> BUTTON1 (pin 26)
//! * 12 -> (pin 20)
//! * 13 -> SCK (pin 23)
//! * 14 -> MISO (pin 22)
//! * 15 -> MOSI (pin 21)
//! * 16 -> (pin 16)
//! * 19 -> SCL (pin 0)
//! * 20 -> SDA (pin 30)
//! 
//! ### Authors
//! * Michael E. Craggs <m.e.craggs@gmail.com>
//! * Date: December 07, 2017

#![no_std]
#![no_main]
#![feature(lang_items,compiler_builtins_lib)]

extern crate capsules;
extern crate compiler_builtins;
#[macro_use(debug, static_init)]
extern crate kernel;
extern crate nrf51;
extern crate nrf5x;

use kernel::{Chip, SysTick};
use kernel::hil::symmetric_encryption::SymmetricEncryption;
use kernel::hil::uart::UART;
use nrf5x::pinmux::Pinmux;


#[macro_use]
pub mod io;

const BUTTON1_PIN: usize = 17;
const BUTTON2_PIN: usize = 21;

const FAULT_RESPONSE: kernel::process::FaultResponse = kernel::process::FaultResponse::Panic;

const NUM_PROCS: usize = 2;

#[link_section = ".app_memory"]
static mut APP_MEMORY: [u8; 8192] = [0; 8192];

static mut PROCESSES: [Option<kernel::Process<'static>>; NUM_PROCS] = [None, None];

pub struct Platform {
    aes: &'static capsules::symmetric_encryption::Crypto<'static, nrf5x::aes::AesECB>,
    button: &'static capsules::button::Button<'static, nrf5x::gpio::GPIOPin>,
    console: &'static capsules::console::Console<'static, nrf51::uart::UART>,
    gpio: &'static capsules::gpio::GPIO<'static, nrf5x::gpio::GPIOPin>,
    rng: &'static capsules::rng::SimpleRng<'static, nrf5x::trng::Trng<'static>>,
}

impl kernel::Platform for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&kernel::Driver>) -> R,
    {
        match driver_num {
            capsules::symmetric_encryption::DRIVER_NUM => f(Some(self.aes)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::rng::DRIVER_NUM => f(Some(self.rng)),
            _ => f(None),
        }
    }
}

#[no_mangle]
pub unsafe fn reset_handler() {
    nrf51::init();

    let button_pins = static_init!(
        [(&'static nrf5x::gpio::GPIOPin, capsules::button::GpioMode); 2],
        [
            (&nrf5x::gpio::PORT[BUTTON1_PIN], capsules::button::GpioMode::LowWhenPressed),
            (&nrf5x::gpio::PORT[BUTTON2_PIN], capsules::button::GpioMode::LowWhenPressed),
        ],
        2 * 4);
    let button = static_init!(
        capsules::button::Button<'static, nrf5x::gpio::GPIOPin>,
        capsules::button::Button::new(button_pins, kernel::Grant::create()),
        96/8);
    for &(btn, _) in button_pins.iter() {
        use kernel::hil::gpio::PinCtl;
        btn.set_input_mode(kernel::hil::gpio::InputMode::PullUp);
        btn.set_client(button);
    }

    let gpio_pins = static_init!(
        [&'static nrf5x::gpio::GPIOPin; 30],
        [
            &nrf5x::gpio::PORT[1],
            &nrf5x::gpio::PORT[2],
            &nrf5x::gpio::PORT[3],
            &nrf5x::gpio::PORT[4],
            &nrf5x::gpio::PORT[5],
            &nrf5x::gpio::PORT[6],
            &nrf5x::gpio::PORT[7],
            &nrf5x::gpio::PORT[8],
            &nrf5x::gpio::PORT[9],
            &nrf5x::gpio::PORT[10],
            &nrf5x::gpio::PORT[11],
            &nrf5x::gpio::PORT[12],
            &nrf5x::gpio::PORT[13],
            &nrf5x::gpio::PORT[14],
            &nrf5x::gpio::PORT[15],
            &nrf5x::gpio::PORT[16],
            &nrf5x::gpio::PORT[17],
            &nrf5x::gpio::PORT[18],
            &nrf5x::gpio::PORT[19],
            &nrf5x::gpio::PORT[20],
            &nrf5x::gpio::PORT[21],
            &nrf5x::gpio::PORT[22],
            &nrf5x::gpio::PORT[23],
            &nrf5x::gpio::PORT[24],
            &nrf5x::gpio::PORT[25],
            &nrf5x::gpio::PORT[26],
            &nrf5x::gpio::PORT[27],
            &nrf5x::gpio::PORT[28],
            &nrf5x::gpio::PORT[29],
            &nrf5x::gpio::PORT[30],
        ],
        4 * 30);

    let gpio = static_init!(
        capsules::gpio::GPIO<'static, nrf5x::gpio::GPIOPin>,
        capsules::gpio::GPIO::new(gpio_pins),
        224/8);
    for pin in gpio_pins.iter() {
        pin.set_client(gpio);
    }
    nrf51::uart::UART0.configure(
        Pinmux::new(25),
        Pinmux::new(24),
        Pinmux::new(31), //unused  ** REQUIRES A PROPER FIX **
        Pinmux::new(31),
    ); //unused  ** REQUIRES A PROPER FIX **
    let console = static_init!(
        capsules::console::Console<nrf51::uart::UART>,
        capsules::console::Console::new(
            &nrf51::uart::UART0,
            115200,
            &mut capsules::console::WRITE_BUF,
            kernel::Grant::create()),
        224/8);
    UART::set_client(&nrf51::uart::UART0, console);
    console.initialize();

    let kc = static_init!(
        capsules::console::App,
        capsules::console::App::default(),
        480/8);
    kernel::debug::assign_console_driver(Some(console), kc);

    let rtc = &nrf5x::rtc::RTC;
    rtc.start();

    let rng = static_init!(
        capsules::rng::SimpleRng<'static, nrf5x::trng::Trng>,
        capsules::rng::SimpleRng::new(&mut nrf5x::trng::TRNG, kernel::Grant::create()),
        96/8);
    nrf5x::trng::TRNG.set_client(rng);

    let aes = static_init!(
        capsules::symmetric_encryption::Crypto<'static, nrf5x::aes::AesECB>,
        capsules::symmetric_encryption::Crypto::new(&mut nrf5x::aes::AESECB,
                                                    kernel::Grant::create(),
                                                    &mut capsules::symmetric_encryption::KEY,
                                                    &mut capsules::symmetric_encryption::BUF,
                                                    &mut capsules::symmetric_encryption::IV),
        288/8);
    nrf5x::aes::AESECB.ecb_init();
    SymmetricEncryption::set_client(&nrf5x::aes::AESECB, aes);

    nrf5x::clock::CLOCK.low_stop();
    nrf5x::clock::CLOCK.high_stop();

    nrf5x::clock::CLOCK.low_set_source(nrf5x::clock::LowClockSource::XTAL);
    nrf5x::clock::CLOCK.low_start();
    nrf5x::clock::CLOCK.high_start();
    while !nrf5x::clock::CLOCK.low_started() {}
    while !nrf5x::clock::CLOCK.high_started() {}

    let platform = Platform {
        aes: aes,
        button: button,
        console: console,
        gpio: gpio,
        rng: rng,
    };

    rtc.start();

    let mut chip = nrf51::chip::NRF51::new();
    chip.systick().reset();
    chip.systick().enable(true);

    debug!("Initialization complete. Entering main loop");
    extern "C" {
        static _sapps: u8;
    }
    kernel::process::load_processes(
        &_sapps as *const u8,
        &mut APP_MEMORY,
        &mut PROCESSES,
        FAULT_RESPONSE,
    );
    kernel::main(
        &platform,
        &mut chip,
        &mut PROCESSES,
        &kernel::ipc::IPC::new(),
    );

}
