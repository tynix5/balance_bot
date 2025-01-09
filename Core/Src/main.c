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

int main(void)
{

	clock_config();
	i2c1_config();

	RCC->AHB1ENR |= (1 << 0);
	GPIOA->MODER |= (1 << 20);
	GPIOA->MODER &= ~(1 << 21);

  while (1)
  {
	  // blink led
	  if (i2c1_test() == 0xA0)
		  GPIOA->ODR = (1 << 10);
	  else
		  GPIOA->ODR = 0;

	  for (int i = 0; i < 20000; i++);
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

	// IC2_2
	// PB8 is SCL
	// PB9 is SDA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		// enable GPIO clock

	// set AF mode for SCL and SDA
	GPIOB->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE8_1;
	GPIOB->MODER &= ~(GPIO_MODER_MODE9_0 | GPIO_MODER_MODE8_0);

	// set open-drain for both lines
	GPIOB->OTYPER |= GPIO_OTYPER_OT9 | GPIO_OTYPER_OT8;

	// enable pull ups (already on board)

	// AF04 for PB8 and PB9
	GPIOB->AFR[1] |= GPIO_AFRH_AFRH0_2 | GPIO_AFRH_AFRH1_2;

	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;			// enable I2C2 clock

	i2c1_sw_rst();

	// I2C steps in sequence
	// 1. Disable peripheral
	// 2. Program peripheral input clock to generate correct timing
	// 3. Configure clock control registers
	// 4. Configure rise time
	// 5. Enable peripheral
	// 6. Start bit
	I2C1->CR1 = 0;		// disable peripheral
	I2C1->CR2 = 42;		// configure peripheral input clock freq to 42MHz (APB1 clock)
	// generate 300kHz I2C
	// Since FREQ = 42MHz, tPCLK = 23.8ns
	// tLOW = 2 * tHIGH
	// 300kHz = 1 / (tLOW + tHIGH) = 1 / (3 * tHIGH)
	// tHIGH = 1.11us
	// tHIGH = CCR * tPCLK
	// CCR = 46.667
//	I2C1->CCR = I2C_CCR_FS | 47;	// fast mode (up to 400kHz SCL) and set freq to 300kHz
	// 10kHz
	I2C1->CCR = 2100;
	// max SCL tRISE is 300ns
	// tPCLK = 23.8ns
	// tRISE / tPCLK + 1 = 12.6 + 1 = 13.6
	I2C1->TRISE = 13;

	I2C1->CR1 |= I2C_CR1_PE;	// enable peripheral
}

void i2c1_sw_rst() {

	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;
}

uint8_t i2c1_test() {

	uint8_t slave_addr = 0x50;
	uint8_t who_am_i_addr = 0;


	I2C1->CR1 |= I2C_CR1_START;
	while (!((I2C1->SR1) & I2C_SR1_SB));		// wait for start bit generation
	I2C1->DR = slave_addr;

	int timeout = 0;
	while (!((I2C1->SR1) & I2C_SR1_ADDR)) {

		timeout++;
		if (timeout == 200000000) {

			i2c1_config();
			return 0;// wait for address to be sent
		}
	}

	uint8_t temp = I2C2->SR2;					// clear address bit
	I2C1->DR = who_am_i_addr;
	while (!((I2C1->SR1) & I2C_SR1_BTF));		// wait for byte transfer complete
	I2C1->CR1 |= I2C_CR1_START;					// repeated start
	while (!((I2C1->SR1) & I2C_SR1_SB));		// wait for start bit generation
	I2C1->DR = slave_addr | 0x01;				// read
	while (!((I2C1->SR1) & I2C_SR1_ADDR));		// wait for address to be sent

	temp = I2C2->SR2;
	while (!((I2C1->SR1) & (1 << 6)));			// wait for data register full
	temp = I2C1->DR;
	I2C1->CR1 |= I2C_CR1_STOP;					// send stop
	return temp;
}
