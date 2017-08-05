/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2015 Chuck McManis <cmcmanis@mcmanis.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include "usart.h"

/* STM32F411-Nucleo at 96 MHz */
const struct rcc_clock_scale rcc_hse_8mhz_3v3_96mhz = {
	.pllm = 8,
	.plln = 384,
	.pllp = 4,
	.pllq = 8,
	.pllr = 0,
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_2,
	.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
	.power_save = 1,
	.flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE |
		FLASH_ACR_LATENCY_3WS,
	.ahb_frequency  = 96000000,
	.apb1_frequency = 48000000,
	.apb2_frequency = 96000000,
};

static void
clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3_96mhz);
	//rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

	/* Enable GPIOA clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable GPIOC for game link pins. */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable clocks for USART2. */
	rcc_periph_clock_enable(RCC_USART2);

	/* Enable DMA1 clock */
	rcc_periph_clock_enable(RCC_DMA1);
}

static void
gpio_setup(void)
{
	/* Setup GPIO pin GPIO5 on GPIO port A for LED. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);

	/* Setup USART2 TX/RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
}

volatile int dma_sent = 0;

void
dma1_stream6_isr(void)
{
	if (dma_get_interrupt_flag(DMA1, DMA_STREAM6, DMA_TCIF)) {
	        // Clear Transfer Complete Interrupt Flag
		dma_clear_interrupt_flags(DMA1, DMA_STREAM6, DMA_TCIF);
		dma_sent = 1;
	}

	dma_disable_transfer_complete_interrupt(DMA1, DMA_STREAM6);
	usart_disable_tx_dma(USART2);
	dma_disable_stream(DMA1, DMA_STREAM6);
}

volatile int dma_recvd = 0;

void
dma1_stream5_isr(void)
{
	if (dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_TCIF)) {
	        // Clear Transfer Complete Interrupt Flag
		dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_TCIF);
		dma_recvd = 1;
	}

	dma_disable_transfer_complete_interrupt(DMA1, DMA_STREAM5);
	usart_disable_rx_dma(USART2);
	dma_disable_stream(DMA1, DMA_STREAM5);
}

static inline void
delay_nop(unsigned int t)
{
	unsigned int i;
	for (i = 0; i < t; i++) { /* Wait a bit. */
		__asm__("nop");
	}
}

static void
gblink_sniff_gpio_setup(void)
{
	// PA0 -> SCK
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
	// PC0 -> SIN
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
	// PC1 -> SOUT
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
	// PC2 -> SD
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2);

	nvic_set_priority(NVIC_EXTI0_IRQ, 0);
	nvic_enable_irq(NVIC_EXTI0_IRQ);

	exti_select_source(EXTI0, GPIOA);
	//exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI0);
}

static void
gblink_slave_gpio_setup(void)
{
	// PA0 -> SCK
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
	// PC0 -> SIN
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO0);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0);
	gpio_clear(GPIOC, GPIO0);
	// PC1 -> SOUT
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
	// PC2 -> SD
	//gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO2);
	//gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO2);

	//gpio_set(GPIOC, GPIO2);

	nvic_set_priority(NVIC_EXTI0_IRQ, 0);
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	nvic_set_priority(NVIC_USART2_IRQ, 1);
	nvic_enable_irq(NVIC_USART2_IRQ);

	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI0);

	usart_enable_rx_interrupt(USART2);
}

volatile uint8_t mode;

volatile uint8_t gb_sin, gb_sout;
volatile uint8_t gb_bit;

#define RECV_BUF_LEN 1024
uint8_t recv_buf[RECV_BUF_LEN];
volatile uint32_t recv_buf_head;
volatile uint32_t recv_buf_tail;

void
usart2_isr(void)
{
	uint8_t empty;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
		empty = recv_buf_tail == recv_buf_head ? 1 : 0;

		recv_buf[recv_buf_head] = usart_recv(USART2);
		recv_buf_head = (recv_buf_head + 1) % RECV_BUF_LEN;

		if (empty && gb_bit == 0 ) {
			gb_sin = recv_buf[recv_buf_tail];
			//recv_buf_tail = (recv_buf_tail + 1) % RECV_BUF_LEN;
		}
	}
}

inline static void
exti0_isr_sniff(void)
{
	//delay_nop(1000);
	gb_sin |= gpio_get(GPIOC, GPIO0) ? 1 : 0;
	gb_sout |= gpio_get(GPIOC, GPIO1) ? 1 : 0;
	gb_bit++;

	if (gb_bit == 8) {
		// Send gb_sin and gb_sout over USART2
		usart_send_blocking(USART2, gb_sin);
		usart_send_blocking(USART2, gb_sout);

		// Reset state
		gb_bit = 0;
		gb_sin = 0;
		gb_sout = 0;
	} else {
		gb_sin <<= 1;
		gb_sout <<= 1;
	}
}

inline static void
exti0_isr_slave(void)
{
	if (gpio_get(GPIOA, GPIO0) == 0) { // FALLING
		gb_sout |= gpio_get(GPIOC, GPIO1) ? 1 : 0;
		gb_bit++;

		if (gb_bit == 8) {
			// Send gb_sin and gb_sout over USART2
			usart_send_blocking(USART2, gb_sout);

			// Reset state
			gb_bit = 0;
			gb_sout = 0;

			// Prepare next gb_sin
			if (recv_buf_tail == recv_buf_head) {
				gb_sin = 0x00;
			} else {
				recv_buf_tail = (recv_buf_tail + 1) % RECV_BUF_LEN;
				if (recv_buf_tail != recv_buf_head) {
					gb_sin = recv_buf[recv_buf_tail];
				} else {
					gb_sin = 0x00;
				}
			}
		} else {
			gb_sin <<= 1;
			gb_sout <<= 1;
		}
	} else { // RISING
		(gb_sin & 0x80) ? gpio_set(GPIOC, GPIO0) : gpio_clear(GPIOC, GPIO0);
		//if (gb_sin & 0x80) {
		//	gpio_set(GPIOC, GPIO0);
		//	//usart_send_blocking(USART2, 0x01);
		//} else {
		//	gpio_clear(GPIOC, GPIO0);
		//	//usart_send_blocking(USART2, 0x00);
		//}
	}
}

void
exti0_isr(void)
{
	// NOTE: If this goes at the end of the function, things no longer work!
	exti_reset_request(EXTI0);
	switch (mode) {
	case 's':
		exti0_isr_sniff();
		break;
	case 'b':
		exti0_isr_slave();
		break;
	default:
		break;
	}

	//gpio_toggle(GPIOA, GPIO5); /* LED on/off */
}

int
main(void)
{
	mode = 'x';
	recv_buf_head = 0;
	recv_buf_tail = 0;

	clock_setup();
	gpio_setup();
	//usart_setup(115200);
	//usart_setup(1152000);
	usart_setup(1000000);
	usart_send_dma_setup();
	usart_recv_dma_setup();

	usart_recv(USART2); // Clear initial garbage
	usart_send_srt_blocking("\nHELLO\n");

	//usart_recv_dma(buf, LEN);
	//while (!dma_recvd);
	//usart_send_dma(buf, LEN);
	//while (!dma_sent);

	while (1) {
		mode = usart_recv_blocking(USART2);
		switch (mode) {
		case 's':
			gblink_sniff_gpio_setup();
			while (1);
			break;
		case 'b':
			// DEBUG START
			//recv_buf[0] = 0x00;
			//recv_buf[0] = 0x00;
			//recv_buf[1] = 0x00;
			//recv_buf[2] = 0x00;
			//recv_buf[3] = 0x00;
			//recv_buf[4] = 0x00;
			//recv_buf[5] = 0x00;
			//recv_buf[6] = 0x00;
			//recv_buf[7] = 0x00;
			////recv_buf[7] = 0x00;
			//recv_buf[8] = 0x81;
			////recv_buf[8] = 0x40;
			//recv_buf[9] = 0x00;
			//recv_buf_tail = 0;
			//recv_buf_head = 10;

			//gb_sin = recv_buf[recv_buf_tail];
			//recv_buf_tail = (recv_buf_tail + 1) % RECV_BUF_LEN;
			// DEBUG END
			gblink_slave_gpio_setup();
			while (1);
			break;
		default:
			break;
		}
	}

	while (1) {
		//gpio_toggle(GPIOA, GPIO5); /* LED on/off */
		//send_USART_str_blocking("HOLA QUE TAL");
		//delay_nop(2000);
		//recv_USART_bytes_blocking(buf, 4);
		//send_USART_bytes_blocking(buf, 4);
		//send_USART_bytes_blocking((unsigned char *) "\r\n", 2);
		//buf[0] = usart_recv_blocking(USART2);
		//usart_send_blocking(USART2, buf[0]);
	}

	return 0;
}
