#!/bin/bash
PAR1=$1
VERSION="${PAR1:=debug}"
echo "=========> VERSION: $VERSION"
ELF="target/thumbv7em-none-eabihf/$VERSION/tiva-rs"
echo "=========> Using ELF: $ELF"
arm-none-eabi-gdb --command=gdb.cmd target/thumbv7em-none-eabihf/debug/tiva-rs 

