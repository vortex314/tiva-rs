PAR1=$1
VERSION="${PAR1:=debug}"
echo "=========> VERSION: $VERSION"
ELF="target/thumbv7em-none-eabihf/$VERSION/tiva-rs"
// echo "=========> Using ELF: $ELF"
// nm -C --print-size --size-sort --radix=d $ELF 
size $ELF
arm-linux-gnueabihf-size $ELF
