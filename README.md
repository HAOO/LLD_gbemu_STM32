# LLD_gbemu_STM32
STM32 port of LLD gbemu https://github.com/rockytriton/LLD_gbemu
I am using the STM32F769I-discovery development board, HAL and BSP provided by STM, the progect can be compiled using STMcube IDE,
just need to open the project file.


Working:
Load rom from SD card, the file name is hard coded.
rom instruction decode.

Missing:
graphics to be displayed
sound not working

Misc.
performace is aceptable, the UART comunication for debugging needs to be remove, it is too slow
