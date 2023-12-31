# First project on Stellaris Launchpad using Rust

This is a simple  project for the [Stellaris Launchpad](http://www.ti.com/tool/ek-lm4f120xl) using [Rust](http://www.rust-lang.org/).
<p>[tm4c123x-hal](https://github.com/rust-embedded-community/tm4c-hal) is used to access the hardware.
## Requirements for an embedded actor model
- async/await and timers
- usable on std and no_std
- actors shouldn't be aware of each other existance
- wiring happens outside actor
- actors should be able to be used in a non-embedded environment
- actors should be able to be used in a multi-threaded environment
- fast and efficient use of cpu resources
- suport buffering of messages
- actor should be internally handling, one message at a time
## Requirements
- communicate via serial to serial2redis
- serial protocol is JSON or CBOR 
- serial protocol should be consumable by a USB client inside browser

## References 
- [Tutorial stellaris - pinout](https://embeddedcomputing.weebly.com/stellaris-lm4f120-launchpad.html)
- [Datasheet](https://www.ti.com/lit/gpn/tm4c123gh6pm)
- [Product Sheet](https://www.ti.com/product/TM4C123GH6PM)
- [Stellaris Launchpad board](http://www.ti.com/tool/sw-ek-lm4f120xl)
- [StellarisWare User Guide](http://www.ti.com/lit/ug/spmu298a/spmu298a.pdf)
- [Olimex UEXT extension board](https://www.olimex.com/Products/Modules/UEXT/resources/UEXT.pdf)
- [Olimex Stellaris UEXT product page](https://www.olimex.com/Products/Modules/Adapters/STELLARIS-UEXT/open-source-hardware)
- [Olimex Stellaris UEXT Schematic](https://www.olimex.com/Products/Modules/Adapters/STELLARIS-UEXT/resources/STELLARIS-UEXT_sch.pdf)

## Too look at
- [probe usage and embassy example](https://github.com/dkhayes117/propane-monitor-embassy)
- [embassy on ESP32](https://github.com/ivmarkov/ruwm/blob/master/.cargo/config.toml)
- [ async in embedded driver](https://cliffle.com/blog/composed-concurrency-in-drivers/)
- [ general rust async](https://cliffle.com/blog/async-inversion/)



# Embassy ( on hold ) 

## RUST setup embassy 
```sh
rustup target add thumbv7em-none-eabihf
rustup toolchain install nightly
rustup default nightly
```

## Encountered issues 
- most JSON serializer use std
- embassy on a not officially supported board is a bit of a pain to get working
- embassy timers drags in a lot of dependencies , if you don't have a stm32 or nrf52 board you are basically out of luck

# RTIC
- stopped this one,as the programming model just deviates too much from the normal rust async model

# spin_on
- very simple, but also very limited

## std on microcontroller
- [Excellent article](https://blog.timhutt.co.uk/std-embedded-rust/index.html)

```shell
rustup target add thumbv7em-none-eabihf
rustup +nightly target add thumbv7m-none-eabihf
# build
cargo +nightly run --release
# list all targets
rustc --print target-list
cargo +nightly build -Z build-std=std,panic_abort --target thumbv7em-none-eabihf
```
This works fine, but the resulting binary is 247kB in size for an empty program, which is a bit too much for a microcontroller with 256KB of flash.
- Build with std and panic_abort
```shell
   text    data     bss     dec     hex filename
 247692      28      56  247776   3c7e0 target/thumbv7em-none-eabihf/debug/tiva-rs
```
- Build with no_std
```shell
   text    data     bss     dec     hex filename
  36480       0       4   36484    8e84 target/thumbv7em-none-eabihf/debug/tiva-rs
```
Build with 

## GDB cheat sheet

- (gdb) l - list source code
- (gdb) b - set breakpoint
- (gdb) r - run
- (gdb) c - continue
- (gdb) n - next - code line by line
- (gdb) s - step - step into function and next instruction cpu
- (gdb) p - print
- (gdb) bt - backtrace
- (gdb) i r - info registers
- (gdb) i r r0 - info register r0
- (gdb) i r pc - info register pc
- (gdb) i r sp - info register sp
- (gdb) i r lr - info register lr
- (gdb) jump reset - jump to reset
- (gdb) jump main - jump to main
- (gdb) x/10x $sp - examine stack
- (gdb) x/10x $pc - examine program counter
- (gdb) info break - list breakpoints
- (gdb) delete breakpoints - delete all breakpoints
- (gdb) x/i $pc - examine instruction at program counter
- (gdb) info functions - list all functions
- (gdb) info variables - list all variables