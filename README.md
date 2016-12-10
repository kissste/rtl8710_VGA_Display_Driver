# rtl8710_VGA_Display_Driver
VGA Driver for RTL8710, RTL8711 and RTL8195 SoC

Runs resolution 800x600 @63Hz at Pixel frequency 41.33MHz.

Using 2 SPIs channels - one for video signal, second one for H-Sync
And, one GPIO for V-Sync



**Console command (RX/TX GB1/GB0 38400 baud):**<br>
ATVG - Will start the VGA Display<br>

**Pictures:**<br>
https://goo.gl/photos/ztxVsQsS6xFEcsJM6

**Wiring:**<br>
GPIO_A1 - VGA:Video via a resistor<br>
GPIO_C2 - VGA:H-Synch directly<br>
GPIO_A5 - VGA:V-Sync directly<br>
