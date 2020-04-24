#ifndef GBLINK_H
#define GBLINK_H

#include <libopencm3/stm32/gpio.h>

#define MODE_SNIFF 's'
#define MODE_SLAVE 'b'
#define MODE_MASTER 'm'
#define PRINTER_SLAVE 'p'
#define PRINTER_MASTER 'P'
#define UNSET 'x'

// GPIO configuration

#define GPIOP_SCK  GPIOA
#define GPION_SCK  GPIO0

#define GPIOP_SIN  GPIOC
#define GPION_SIN  GPIO0

#define GPIOP_SOUT GPIOC
#define GPION_SOUT GPIO1

#define GPIOP_SD   GPIOC
#define GPION_SD   GPIO2

#define GPIOP_LED  GPIOA
#define GPION_LED  GPIO5

#define GPIOP_USART GPIOA
#define GPION_USART_TX GPIO2
#define GPION_USART_RX GPIO3

#endif
