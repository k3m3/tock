use core::fmt::{Write, write, Arguments};
use kernel::hil::uart::{self, UART};
use nrf51;

pub struct Writer {
    initialized: bool,
}

pub static mut WRITER: Writer = Writer { initialized: false };

impl Write for Writer {
    fn write_str(&mut self, s: &str) -> ::core::fmt::Result {
        let uart = unsafe { &mut nrf51::uart::UART0 };
        if !self.initialized {
            self.initialized = true;
            uart.init(uart::UARTParams {
                baud_rate: 115200,
                stop_bits: uart::StopBits::One,
                parity: uart::Parity::None,
                hw_flow_control: false,
            });
            for c in s.bytes() {
                unsafe {
                    uart.send_byte(c);
                }
                while !uart.tx_ready() {}
            }
        }
        Ok(())
    }
}

#[macro_export]
macro_rules! print {
     () => {
         use core::fmt::Write;
         let writer = &mut $crate::io::WRITER;
         let _ = write(writer, format_args!($($arg)*));
     };
 }

#[cfg(not(test))]
#[lang = "panic_fmt"]
#[no_mangle]
pub unsafe extern "C" fn rust_begin_unwind(
    _args: Arguments,
    _file: &'static str,
    _line: usize,
) -> ! {
    loop {
        use kernel::process;
        let writer = &mut WRITER;
        let _ = writer.write_fmt(format_args!(
            "\r\nKernel panic at {}:{}:\r\n\t\"",
            _file,
            _line
        ));
        let _ = write(writer, _args);
        let _ = writer.write_str("\"\r\n");

        // Print version of the kernel
        let _ = writer.write_fmt(format_args!(
            "\tKernel version {}\r\n",
            env!("TOCK_KERNEL_VERSION")
        ));

        // Print fault status once
        let procs = &mut process::PROCS;
        if procs.len() > 0 {
            procs[0].as_mut().map(
                |process| { process.fault_str(writer); },
            );
        }

        // print data about each process
        let _ = writer.write_fmt(format_args!("\r\n---| App Status |---\r\n"));
        let procs = &mut process::PROCS;
        for idx in 0..procs.len() {
            procs[idx].as_mut().map(|process| {
                process.statistics_str(writer);
            });
        }

    }
}
