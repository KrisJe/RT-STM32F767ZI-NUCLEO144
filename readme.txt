*****************************************************************************
** ChibiOS/RT HAL - ADC driver demo for STM32F767ZI.                         **
*****************************************************************************

** TARGET **

The demo runs on an STMicroelectronics STM32F767ZI NUCLEO board.

** The Demo **

The application demonstrates the use of the STM32F7xx multi ADC mode using a
hacked together extra driver in adc_multi.c.

PA0, PA1, PA2, PA3, PC0 and PC1 are continuously read and output to the usb-serial device.
