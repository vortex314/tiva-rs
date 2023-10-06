# First project on Stellaris Launchpad using Rust

This is a simple  project for the [Stellaris Launchpad](http://www.ti.com/tool/ek-lm4f120xl) using [Rust](http://www.rust-lang.org/).
<p>[tm4c123x-hal](https://github.com/rust-embedded-community/tm4c-hal) is used to access the hardware.

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



