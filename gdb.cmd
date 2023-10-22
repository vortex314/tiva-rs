set confirm off
target remote 127.0.0.1:3333
monitor reset
monitor halt
monitor arm semihosting enable
load
