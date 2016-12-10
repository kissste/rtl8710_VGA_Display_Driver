# rtl8710_VGA_Display_Driver
VGA Driver for RTL8710, RTL8711 and RTL8195 SoC

Runs resolution 800x600 @63Hz at Pixel frequency 41.33MHz.

Using 2 SPIs channels - one for video signal, second one for H-Sync
One GPIO for V-Sync



<u>Console command (RX/TX GB1/GB0 38400 baud):</u><br>
ATVG - Will start the VGA Display<br>

<u>Pictures:</u>
https://goo.gl/photos/ztxVsQsS6xFEcsJM6

<u>Wiring:</u>
GPIO_A1 - Video<br>
GPIO_C2 - H-Synch<br>
GPIO_A5 - V-Sync<br>
