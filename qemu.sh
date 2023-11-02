qemu-system-arm -cpu cortex-m4 -machine lm3s6965evb -nographic \
  -semihosting-config enable=on,target=native \
  -gdb tcp::3333 \
  -S \
  -kernel  target/thumbv7em-none-eabihf/debug/tiva-rs