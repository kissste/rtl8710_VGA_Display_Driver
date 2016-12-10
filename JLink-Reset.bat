@echo off
PATH=c:\arduino\SEGGER\JLink_V610n;%PATH%
start JLink.exe -Device CORTEX-M3 -If SWD -Speed 1000 flasher\RTL_Reset.JLinkScript
