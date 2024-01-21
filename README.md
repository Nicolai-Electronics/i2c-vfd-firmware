# Firmware for I2C controlled HCS-12SS59T VFD

This repository contains the firmware for the I2C controlled NE-HCS12SS59T-R01 Vacuum Fluorescent Display board.

## HCS-12SS59T

The VFD attached to the board is a Samsung HCS-12SS59T VFD controlled via SPI.

## I2C interface

The default I2C address is 0x10. The address can be changed by bridging the address jumpers on the board. This allows for modifying the I2C address in the range 0x10 up to 0x2F.

### Register map

| Register | Name            | Bit 7            | Bit 6            | Bit 5            | Bit 4            | Bit 3            | Bit 2            | Bit 1            | Bit 0            |
|----------|-----------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|
| 0        | System control  | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | LED state        | Test mode        | Display enable   |
| 1        | Display offset  | Offset bit 7     | Offset bit 6     | Offset bit 5     | Offset bit 4     | Offset bit 3     | Offset bit 2     | Offset bit 1     | Offset bit 0     |
| 2        | Scroll length   | Length bit 7     | Length bit 6     | Length bit 5     | Length bit 4     | Length bit 3     | Length bit 2     | Length bit 1     | Length bit 0     |
| 3        | Scroll mode     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Loop enable      | Mode bit 3       | Mode bit 2       | Mode bit 1       | Mode bit 0       |
| 4        | Scroll speed HI | Speed bit 15     | Speed bit 14     | Speed bit 13     | Speed bit 12     | Speed bit 11     | Speed bit 10     | Speed bit 9      | Speed bit 8      |
| 5        | Scroll speed LO | Speed bit 7      | Speed bit 6      | Speed bit 5      | Speed bit 4      | Speed bit 3      | Speed bit 2      | Speed bit 1      | Speed bit 0      |
| 6        | Brightness      | Brightness bit 7 | Brightness bit 6 | Brightness bit 5 | Brightness bit 4 | Brightness bit 3 | Brightness bit 2 | Brightness bit 1 | Brightness bit 0 |
| 7        | Reserved (0)    | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     |
| 8        | Reserved (0)    | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     |
| 9        | Reserved (0)    | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     | Reserved (0)     |
| 10 - 255 | Data (ASCII)    | Reserved (0)     | ASCII bit 6      | ASCII bit 5      | ASCII bit 4      | ASCII bit 3      | ASCII bit 2      | ASCII bit 1      | ASCII bit 0      |

### System control

To enable the display write `0x01` to register `0x00`. If you also want to turn on the amber LED on the back of the board write `0x05` (0x04 + 0x01) instead.

The display is enabled by default when power is applied. This register allows disabling the VFD and the voltage boosters on the VFD board for saving power.

### Brightness

This register controls the filament current, the default value is `110`. Setting the filament current too low may result in no or a dimmed display, setting the filament current too high will shorten the lifespan of the VFD.

### More information

More information is available on our [website](https://nicolaielectronics.nl/docs/ne-hcs12ss59t-r1_i2c_vfd/)
