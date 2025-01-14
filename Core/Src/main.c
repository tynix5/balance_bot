/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "main.h"


// configure system clock to run at 84MHz
void clock_config();

// configure I2C master
void i2c1_config();
void i2c1_sw_rst();
uint8_t i2c1_test();
void i2c_start();
void i2c_request_stop();
void i2c_ack();
void i2c_nack();
void i2c1_release();

// debugging
void uart1_config();
void uart_send(uint8_t byte);

void bno055_config();


int main(void)
{

	clock_config();

	RCC->AHB1ENR |= (1 << 0);
	GPIOA->MODER |= (1 << 20);
	GPIOA->MODER &= ~(1 << 21);


	i2c1_config();
	while (i2c1_test() != 0xA0);

	uart1_config();

  while (1)
  {
	  GPIOA->ODR |= (1 << 10);
	  uart_send((uint8_t)'.');
	  for (int i = 0; i < 500000; i++);
  }
}

void clock_config() {

	// 16 MHz HSI oscillator is default on reset, but select anyways
	RCC->CR |= RCC_CR_HSION;
	// wait for HSI to be ready
	while (!((RCC->CR) & RCC_CR_HSIRDY));

	// enable power interface clock for APB1
	RCC->APB1ENR = RCC_APB1ENR_PWREN;

	// configure VCO to scale 2 per CubeMX
	PWR->CR |= PWR_CR_VOS_1;
	PWR->CR &= ~PWR_CR_VOS_0;

	// configure FLASH
	// instruction cache, prefetch enable, and data cache enabled
	uint32_t flash;
	flash = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN;
	flash |= 2;		// 2 wait states for flash
	FLASH->ACR = flash;

	// configure bus prescalers
	uint32_t cfgr = 0;
	cfgr &= ~RCC_CFGR_PPRE2_2;		// APB2 prescaler of 1 (84MHz)
	cfgr |= RCC_CFGR_PPRE1_2;		// APB1 prescaler of 2 (42MHZ)
	cfgr &= ~RCC_CFGR_HPRE;			// AHB prescaler of 1 (84MHz)
	RCC->CFGR = cfgr;

	// configure main PLL
	uint32_t pll_cfg = RCC->PLLCFGR;
	pll_cfg &= ~RCC_PLLCFGR_PLLQ;
	pll_cfg |= RCC_PLLCFGR_PLLQ_2; // configure Q prescaler for USB, SDIO, RNG clocks (/4)

	pll_cfg &= ~RCC_PLLCFGR_PLLP;	// main PLL division factor of 2

	pll_cfg &= ~RCC_PLLCFGR_PLLN;
	pll_cfg |= 168UL << 6;	// pll multiplication factor for VCO (x168)

	pll_cfg &= ~RCC_PLLCFGR_PLLM;
	pll_cfg |= 16UL << 0;	// pll division factor for main PLL and audio PLL (/16)

	RCC->PLLCFGR = pll_cfg;

	// enable PLL and wait for ready
	RCC->CR |= RCC_CR_PLLON;
	while (!((RCC->CR) & RCC_CR_PLLRDY));

	// select clock source
	cfgr = RCC->CFGR;
	cfgr |= RCC_CFGR_SW_1;		// select PLL as system clock
	cfgr &= ~RCC_CFGR_SW_0;
	RCC->CFGR = cfgr;

	// wait for PLL clock source to become active
	while (!((RCC->CFGR) & RCC_CFGR_SWS_1));
}

void i2c1_config() {

	// PB8 is SCL
	// PB9 is SDA
	// enable I2C clock before configuring pins
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;			// enable I2C1 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		// enable GPIO clock

	// before configuring SCL and SDA pins for AF, make sure bus is not busy by clocking out extra
	// data to slave devices
	i2c1_release();


	// set AF mode for SCL and SDA
	GPIOB->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE8_1;
	GPIOB->MODER &= ~(GPIO_MODER_MODE9_0 | GPIO_MODER_MODE8_0);

	// set open-drain for both lines
	GPIOB->OTYPER |= GPIO_OTYPER_OT9 | GPIO_OTYPER_OT8;

	// enable pull ups (already on board)

	// set max output speed for both
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR8;

	// AF04 for PB8 and PB9 to select I2C1_SCL and I2C1_SDA
	GPIOB->AFR[1] |= GPIO_AFRH_AFRH0_2 | GPIO_AFRH_AFRH1_2;


	// I2C steps in sequence (per data sheet)
	// 1. Disable peripheral
	// 2. Program peripheral input clock to generate correct timing
	// 3. Configure clock control registers
	// 4. Configure rise time
	// 5. Enable peripheral
	// 6. Start bit
	I2C1->CR1 = 0;		// disable peripheral
	i2c1_sw_rst();		// reset I2C to clear busy bit
	I2C1->CR2 = 42;		// configure peripheral input clock freq to 42MHz (APB1 clock)

	// 300kHz fast mode
	// Since FREQ = 42MHz, tPCLK = 23.8ns
	// tLOW = 2 * tHIGH
	// 300kHz = 1 / (tLOW + tHIGH) = 1 / (3 * tHIGH)
	// tHIGH = 1.11us
	// tHIGH = CCR * tPCLK
	// CCR = 46.667
	I2C1->CCR = I2C_CCR_FS | 47;	// fast mode and set freq to 300kHz
	// max SCL tRISE is 300ns
	// tPCLK = 23.8ns
	// tRISE / tPCLK + 1 = 12.6 + 1 = 13.6
	I2C1->TRISE = 13;

	I2C1->CR1 |= I2C_CR1_PE;	// enable peripheral
}

void i2c1_sw_rst() {

	// reset I2C1
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;
}

uint8_t i2c1_test() {

	uint8_t slave_addr = 0x50;
	uint8_t who_am_i_addr = 0;

	while (I2C1->SR2 & I2C_SR2_BUSY);

	i2c_start();
	I2C1->DR = slave_addr;
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2;						// dummy read to clear status bit
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// data register needs to be empty
	I2C1->DR = who_am_i_addr;				// set pointer to CHIP ID on BNO055
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// data register needs to be empty

	// For single byte reads, pg 482 of reference manual "Closing the communication"
	i2c_start();
	I2C1->DR = slave_addr | 0x01;			// read mode
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	i2c_nack();
	(void)I2C1->SR2;						// dummy read to clear status bit
	i2c_request_stop();
	while (!(I2C1->SR1 & I2C_SR1_RXNE));	// wait for full data register
	return I2C1->DR;
}


void i2c_start() {

	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));		// wait for start bit generation
}

void i2c_request_stop() {

	I2C1->CR1 |= I2C_CR1_STOP;
}

void i2c_ack() {

	I2C1->CR1 |= I2C_CR1_ACK;
}

void i2c_nack() {

	I2C1->CR1 &= ~I2C_CR1_ACK;
}


void i2c1_release() {

	// set SCL and SDA lines as outputs
	GPIOB->MODER |= GPIO_MODER_MODER9_0 | GPIO_MODER_MODER8_0;
	GPIOB->MODER &= ~(GPIO_MODER_MODER9_1 | GPIO_MODER_MODER8_1);

	// output open drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT9 | GPIO_OTYPER_OT8;

	// set SDA line high, so that after slave releases the SDA and listens for ACK/NACK, it will get NACK
	GPIOB->ODR |= GPIO_ODR_ODR_9;

	// generate 10 clock pulses on SCL line
	for (int i = 0; i < 10; i++) {

		GPIOB->ODR |= GPIO_ODR_ODR_8;
		for (int j = 0; j < 1000; j++)
			__NOP();

		GPIOB->ODR &= ~GPIO_ODR_ODR_8;
		for (int j = 0; j < 1000; j++)
			__NOP();
	}

	// set SCL high, line should be pulled high when I2C is idle
	GPIOB->ODR |= GPIO_ODR_ODR_8;
}


void uart1_config() {

	// UART TX is PA9 or D8

	// enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// select alternate function mode for PA9
	GPIOA->MODER |= GPIO_MODER_MODER9_1;
	GPIOA->MODER &= ~GPIO_MODER_MODER9_0;

	// select UART_TX1 as AF
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH1_2 | GPIO_AFRH_AFRH1_1 | GPIO_AFRH_AFRH1_0;


	// enable USART1 clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1->CR1 |= USART_CR1_UE;		// USART enable
	USART1->CR1 &= ~USART_CR1_M;		// 1 start bit, 8 data bits, n stop bits
	USART1->CR1 &= ~USART_CR1_PCE;		// no parity

	USART1->CR2 = 0;					// 1 stop bit, asynchronous

	// set baud rate to 9600 bps
	// USARTDIV = fCK / (8 * (2 - OVER8) * baud)
	// fCK = 84MHz (APB2 bus)
	// baud = 9600
	// USARTDIV = 546.875
	// Mantissa = 546
	// Fraction = 0.875 * 16 (4 bits for fraction) = 14
	USART1->BRR = 546 << 4 | 14;

	USART1->CR1 |= USART_CR1_TE;		// enable transmitter
}


void uart_send(uint8_t byte) {

	while (!(USART1->SR & USART_SR_TXE));		// wait until data register is empty
	USART1->DR = byte;			// load register
	while (!(USART1->SR & USART_SR_TC));		// wait until transmission complete
}
