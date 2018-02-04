#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "gb-link.h"
#include "usart.h"
#include "buffer.h"

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
	.flash_config = FLASH_ACR_ICEN | FLASH_ACR_DCEN |
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
	gpio_mode_setup(GPIOP_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPION_LED);

	/* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOP_USART, GPIO_MODE_AF, GPIO_PUPD_NONE, GPION_USART_TX | GPION_USART_RX);

	/* Setup USART2 TX/RX pin as alternate function. */
	gpio_set_af(GPIOP_USART, GPIO_AF7, GPION_USART_TX | GPION_USART_RX);
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
	gpio_mode_setup(GPIOP_SCK, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPION_SCK);
	// PC0 -> SIN
	gpio_mode_setup(GPIOP_SIN, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPION_SIN);
	// PC1 -> SOUT
	gpio_mode_setup(GPIOP_SOUT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPION_SOUT);
	// PC2 -> SD
	gpio_mode_setup(GPIOP_SD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPION_SD);

	nvic_set_priority(NVIC_EXTI0_IRQ, 0);
	nvic_enable_irq(NVIC_EXTI0_IRQ);

	exti_select_source(EXTI0, GPIOP_SCK);
	//exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI0);
}

static void
gblink_slave_gpio_setup(void)
{
	// PA0 -> SCK
	gpio_mode_setup(GPIOP_SCK, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPION_SCK);
	// PC0 -> SIN
	gpio_mode_setup(GPIOP_SIN, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPION_SIN);
	gpio_set_output_options(GPIOP_SIN, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPION_SIN);
	gpio_clear(GPIOP_SIN, GPION_SIN);
	// PC1 -> SOUT
	gpio_mode_setup(GPIOP_SOUT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPION_SOUT);
	// PC2 -> SD
	//gpio_mode_setup(GPIOP_SD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPION_SD);
	//gpio_mode_setup(GPIOP_SD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPION_SD);

	//gpio_set(GPIOP_SD, GPION_SD);

	nvic_set_priority(NVIC_EXTI0_IRQ, 0);
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	// Not using this
	//nvic_set_priority(NVIC_USART2_IRQ, 1);
	//nvic_enable_irq(NVIC_USART2_IRQ);

	exti_select_source(EXTI0, GPIOP_SCK);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI0);

	// Not using this
	//usart_enable_rx_interrupt(USART2);
}

static void
gblink_master_gpio_setup(void)
{
	// PA0 -> SCK
	gpio_mode_setup(GPIOP_SCK, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPION_SCK);
	// PC0 -> SIN
	// NOTE: The gameboy printer signals 0 by connecting to ground and 1 by
	// floating.  This means that a pullup resistor is required to read the
	// 1.  Either connect a pullup resistor to the pin (for example
	// 15KOhm), or set GPIO_PUPD_PULLUP as in the following line.
	gpio_mode_setup(GPIOP_SIN, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPION_SIN);
	// PC1 -> SOUT
	gpio_mode_setup(GPIOP_SOUT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPION_SOUT);
	gpio_set_output_options(GPIOP_SOUT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPION_SOUT);
	gpio_clear(GPIOP_SOUT, GPION_SOUT);
	// PC2 -> SD
	gpio_mode_setup(GPIOP_SD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPION_SD);
	//gpio_mode_setup(GPIOP_SD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPION_SD);

	gpio_set(GPIOP_SD, GPION_SD);

	// Not using this
	//nvic_set_priority(NVIC_USART2_IRQ, 1);
	//nvic_enable_irq(NVIC_USART2_IRQ);

	//usart_enable_rx_interrupt(USART2);
}

static void
tim_setup(uint32_t freq)
{
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */
	/* Set the prescaler to run at 1MHz */
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 1000000) - 1);

	/* Set the initual output compare value for OC1. */
	timer_set_oc_value(TIM2, TIM_OC1, 0);

	/* Disable preload. */
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	/* count full range, as we'll update compare value continuously */
	//timer_set_period(TIM2, 8 - 1);
	timer_set_period(TIM2, (1000000 / freq) - 1);
}

static inline void
tim_start(void)
{
	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

static inline void
tim_stop(void)
{
	timer_disable_counter(TIM2);
	timer_disable_irq(TIM2, TIM_DIER_CC1IE);
}

volatile uint8_t mode;
volatile uint8_t slave_mode;
volatile uint8_t master_mode;

volatile uint8_t gb_sin, gb_sout;
volatile uint8_t gb_bit;

struct circular_buf recv_buf;

void
usart2_isr(void)
{
	uint8_t empty;
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
		empty = buf_empty(&recv_buf);

		buf_push(&recv_buf, usart_recv(USART2));

		if (empty && gb_bit == 0 ) {
			gb_sin = buf_pop(&recv_buf);
		}
	}
}

const char printer_magic[] = {0x88, 0x33};

enum printer_state {MAGIC0, MAGIC1, CMD, ARG0, LEN_LOW, LEN_HIGH, DATA, CHECKSUM0, CHECKSUM1, ACK, STATUS};
enum printer_state printer_state;
enum printer_state printer_state_prev;
uint16_t printer_data_len;

static void
printer_state_update(uint8_t b)
{
	printer_state_prev = printer_state;
	switch (printer_state) {
	case MAGIC0:
		if (b == printer_magic[0]) {
			printer_state = MAGIC1;
		}
		break;
	case MAGIC1:
		if (b == printer_magic[1]) {
			printer_state = CMD;
		} else {
			printer_state = MAGIC0;
		}
		break;
	case CMD:
		printer_state = ARG0;
		break;
	case ARG0:
		printer_state = LEN_LOW;
		break;
	case LEN_LOW:
		printer_data_len = b;
		printer_state = LEN_HIGH;
		break;
	case LEN_HIGH:
		printer_data_len |= b << 8;
		if (printer_data_len != 0) {
			printer_state = DATA;
		} else {
			printer_state = CHECKSUM0;
		}
		break;
	case DATA:
		printer_data_len--;
		printer_state = (printer_data_len == 0) ? CHECKSUM0 : DATA;
		break;
	case CHECKSUM0:
		printer_state = CHECKSUM1;
		break;
	case CHECKSUM1:
		printer_state = ACK;
		break;
	case ACK:
		printer_state = STATUS;
		break;
	case STATUS:
		printer_state = MAGIC0;
		break;
	}
}

static void
printer_state_reset(void)
{
	printer_data_len = 0;
	printer_state = MAGIC0;
}

inline static void
exti0_isr_sniff(void)
{
	// RISING
	//delay_nop(1000);
	gb_sin |= gpio_get(GPIOP_SIN, GPION_SIN) ? 1 : 0;
	gb_sout |= gpio_get(GPIOP_SOUT, GPION_SOUT) ? 1 : 0;
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
	if (gpio_get(GPIOP_SCK, GPION_SCK) == 0) { // FALLING
		gb_sout |= gpio_get(GPIOP_SOUT, GPION_SOUT) ? 1 : 0;
		gb_bit++;

		if (gb_bit == 8) {
			// Send gb_sin and gb_sout over USART2
			usart_send_blocking(USART2, gb_sout);

			switch (slave_mode) {
			case SLAVE_PRINTER:
				printer_state_update(gb_sout);
				switch (printer_state) {
				case ACK:
					buf_push(&recv_buf, 0x81);
					break;
				case STATUS:
					buf_push(&recv_buf, 0x00);
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}

			// Reset state
			gb_bit = 0;
			gb_sout = 0;

			// Prepare next gb_sin
			if (buf_empty(&recv_buf)) {
				gb_sin = 0x00;
			} else {
				gb_sin = buf_pop(&recv_buf);
			}
		} else {
			gb_sin <<= 1;
			gb_sout <<= 1;
		}
	} else { // RISING
		(gb_sin & 0x80) ? gpio_set(GPIOP_SIN, GPION_SIN) : gpio_clear(GPIOP_SIN, GPION_SIN);
	}
}

void
exti0_isr(void)
{
	// NOTE: If this goes at the end of the function, things no longer work!
	exti_reset_request(EXTI0);

	switch (mode) {
	case MODE_SNIFF:
		exti0_isr_sniff();
		break;
	case MODE_SLAVE:
		exti0_isr_slave();
		break;
	default:
		break;
	}

	//gpio_toggle(GPIOP_LED, GPION_LED); /* LED on/off */
}

uint8_t high;
uint8_t stop;

void
tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
		if (high) { // FALLING
			(gb_sout & 0x80) ? gpio_set(GPIOP_SOUT, GPION_SOUT) : gpio_clear(GPIOP_SOUT, GPION_SOUT);
			//usart_send_blocking(USART2, (gb_sout & 0x80) ? 1 : 0);
			//usart_send_blocking(USART2, gb_bit);
			if (gb_bit == 0) {
				switch (master_mode) {
				case MASTER_PRINTER:
					//usart_send_blocking(USART2, gb_sout);
					printer_state_update(gb_sout);
				}
			}

			gpio_clear(GPIOP_SCK, GPION_SCK);
		} else { // RISING

			gb_sin |= gpio_get(GPIOP_SIN, GPION_SIN) ? 1 : 0;
			gb_bit++;

			if (gb_bit == 8) {
				//usart_send_blocking(USART2, gb_sin);
				switch (master_mode) {
				case MASTER_PRINTER:
					switch (printer_state_prev) {
					case ACK:
						usart_send_blocking(USART2, gb_sin);
						break;
					case STATUS:
						usart_send_blocking(USART2, gb_sin);
						break;
					default:
						break;
					}
					break;
				}

				// Reset state
				gb_bit = 0;
				gb_sin = 0;

				// Prepare next gb_sout
				if (buf_empty(&recv_buf)) {
					gb_sout = 0x00;
					stop = 1;
				} else {
					gb_sout = buf_pop(&recv_buf);
				}
			} else {
				gb_sout <<= 1;
				gb_sin <<= 1;
			}

			if (stop) {
				tim_stop();
				gpio_toggle(GPIOP_LED, GPION_LED); /* LED on/off */
			}

			gpio_set(GPIOP_SCK, GPION_SCK);
		}
		high = !high;

		/* Clear compare interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_CC1IF);
	}
}

static void
mode_master_printer(void)
{
	uint8_t len_low, len_high;
	uint16_t len;
	unsigned int i;

	gpio_set(GPIOP_SCK, GPION_SCK);
	high = 1;

	// Block until we get confirmation that the printer has ben turned on
	usart_recv_blocking(USART2);

	while (1) {
		gpio_toggle(GPIOP_LED, GPION_LED); /* LED on/off */
		len_low = usart_recv_blocking(USART2);
		len_high = usart_recv_blocking(USART2);
		len = len_low | ((uint16_t) len_high) << 8;

		buf_clear(&recv_buf);
		for (i = 0; i < len; i++) {
			buf_push(&recv_buf, usart_recv_blocking(USART2));
		}
		gpio_toggle(GPIOP_LED, GPION_LED); /* LED on/off */

		gb_bit = 0;
		gb_sin = 0;
		stop = 0;

		// Prepare fist byte
		gb_sout = buf_pop(&recv_buf);
		// Set first bit of first byte
		//(gb_sout & 0x80) ? gpio_set(GPIOP_SOUT, GPION_SOUT) : gpio_clear(GPIOP_SOUT, GPION_SOUT);

		tim_start();
	}
}

int
main(void)
{
	uint8_t opt;

	mode = UNSET;
	slave_mode = UNSET;
	buf_clear(&recv_buf);

	clock_setup();
	gpio_setup();
	//usart_setup(115200);
	//usart_setup(1152000);
	usart_setup(1000000);
	usart_send_dma_setup();
	usart_recv_dma_setup();

	usart_recv(USART2); // Clear initial garbage
	usart_send_srt_blocking("\nHELLO\n");

	while (1) {
		opt = usart_recv_blocking(USART2);
		switch (opt) {
		case MODE_SNIFF:
			mode = MODE_SNIFF;
			gblink_sniff_gpio_setup();
			while (1);
			break;
		case SLAVE_PRINTER:
			mode = MODE_SLAVE;
			slave_mode = SLAVE_PRINTER;
			printer_state_reset();
			gblink_slave_gpio_setup();
			while (1);
			break;
		case MASTER_PRINTER:
			mode = MODE_MASTER;
			master_mode = MASTER_PRINTER;
			printer_state_reset();
			gblink_master_gpio_setup();
			tim_setup(2 * 8192);
			//tim_setup(2 * 1000);
			while (1) {
				mode_master_printer();
			}
			break;
		case MODE_SLAVE:
			mode = MODE_SLAVE;
			gblink_slave_gpio_setup();
			while (1);
			break;
		default:
			break;
		}
	}

	while (1);

	return 0;
}
