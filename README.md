# rtl8710_VGA_Display_Driver
VGA Driver for RTL8710, RTL8711 and RTL8195 SoC

Runs resolution 800x600 @63Hz at Pixel frequency 41.33MHz.

Using 2 SPIs channels - one for video signal, second on for H-Sync
One GPIO for V-Sync

https://goo.gl/photos/ztxVsQsS6xFEcsJM6

Wiring:

GPIO_A1 - Video

GPIO_C2 - H-Synch

GPIO_A5 - V-Sync
