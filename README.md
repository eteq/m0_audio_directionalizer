Magic incantation to flash from arch:
```
cargo objcopy --release -- -O binary toflash.bin;uf2conv toflash.bin --base 0x2000 --output toflash.uf2; cp toflash.uf2 /run/media/<username>/CPLAYBOOT/
```
