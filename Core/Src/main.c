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

#include <stdlib.h>
#include <math.h>
#include "main.h"


#define IMU_ADDR				0x50
#define IMU_WHO_AM_I_REG		0x00
#define IMU_ST_RESULT_REG		0x36

#define MOTOR_FWD				0
#define MOTOR_BACKWD			1


// configure system clock to run at 84MHz
void clock_config();

// configure I2C master
void i2c1_config();
void i2c1_sw_rst();
void i2c1_writebyte(uint8_t slave_addr, uint8_t reg_addr, uint8_t byte);
uint8_t i2c1_readbyte(uint8_t slave_addr, uint8_t reg_addr);
void i2c1_readburst(uint8_t slave_addr, uint8_t reg_addr, uint8_t len, uint8_t * bytes);
void i2c1_start();
void i2c1_request_stop();
void i2c1_ack();
void i2c1_nack();
void i2c1_release();

// debugging
void uart1_config(uint32_t baud);
void uart1_writebyte(uint8_t byte);
void uart1_writeint(int num);
void uart1_writestr(char * str);

// PWM generation
void tim2_config();
void motors_config();
void set_pwm_leftmotor(uint8_t direction, uint32_t pwm);
void set_pwm_rightmotor(uint8_t direction, uint32_t pwm);
void enable_leftmotor();
void disable_leftmotor();
void enable_rightmotor();
void disable_rightmotor();

// 100Hz timer
void tim5_config();

uint8_t imu_config();
uint8_t imu_test();
void imu_read_euler(int16_t * roll_raw, int16_t * pitch_raw, int16_t * heading_raw);
void convert_euler(int16_t roll_raw, int16_t pitch_raw, int16_t heading_raw, float * roll, float * pitch, float * heading);


int main(void)
{

	clock_config();
	i2c1_config();
//	uart1_config(9600);
	motors_config();

	tim5_config();


	while (!imu_config());


	const float kp = 1.6;
	const float kd = 0.5;

	const float max_error = 30;
	const float max_controller_out = 10;


	const float pitch_setpoint = 0;

	float last_err = 0;

  while (1)
  {
	  TIM5->SR &= ~TIM_SR_UIF;			// clear update interrupt flag

	  int16_t roll_raw, heading_raw, pitch_raw;
	  float roll, heading, pitch;
	  imu_read_euler(&roll_raw, &pitch_raw, &heading_raw);
	  convert_euler(roll_raw, pitch_raw, heading_raw, &roll, &pitch, &heading);

	  // pitch is one we care about
	  float pitch_err = pitch - pitch_setpoint;

	  // if bot tips over, turn off motors
	  if (fabs(pitch_err) > max_error) {

		  disable_leftmotor();
		  disable_rightmotor();
		  continue;
	  }
	  else {

		  enable_leftmotor();
		  enable_rightmotor();
	  }


	  float controller_out = kp * pitch_err + kd * (pitch_err - last_err);

	  // use absolute value of controller to select pwm duty value
	  // direction of motors is determined by sign of controller_out
	  float controller_abs = fabs(controller_out);

	  // limit the top of the controller
	  if (controller_abs > max_controller_out)
		  controller_abs = max_controller_out;


	  // map function: output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
	  // input: [0, 10]
	  // left motor spins at lower PWM than right motor
	  uint16_t leftmotor_max_pwm = 2047;
	  uint16_t leftmotor_min_pwm = 750;
	  uint16_t rightmotor_max_pwm = 2047;
	  uint16_t rightmotor_min_pwm = 900;
	  uint32_t left_pwm_val = (uint32_t) (leftmotor_min_pwm + ( ((float)(leftmotor_max_pwm - leftmotor_min_pwm)) / (max_controller_out - 0)) * (controller_abs - 0));
	  uint32_t right_pwm_val = (uint32_t) (rightmotor_min_pwm + ( ((float)(rightmotor_max_pwm - rightmotor_min_pwm)) / (max_controller_out - 0)) * (controller_abs - 0));

	  uint8_t motor_dir;

	  if (controller_out > 0)		motor_dir = MOTOR_BACKWD;
	  else							motor_dir = MOTOR_FWD;


	  set_pwm_leftmotor(motor_dir, left_pwm_val);
	  set_pwm_rightmotor(motor_dir, right_pwm_val);

	  last_err = pitch_err;

	  while (!(TIM5->SR & (TIM_SR_UIF)));// wait for next fusion data

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


void i2c1_writebyte(uint8_t slave_addr, uint8_t reg_addr, uint8_t byte) {

	while (I2C1->SR2 & I2C_SR2_BUSY);

	i2c1_start();
	I2C1->DR = slave_addr;
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2;						// dummy read to clear status bit
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// data register needs to be empty
	I2C1->DR = reg_addr;					// set pointer on BNO055
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// data register needs to be empty
	I2C1->DR = byte;
	while (!(I2C1->SR1 & I2C_SR1_TXE));
	i2c1_request_stop();
}


uint8_t i2c1_readbyte(uint8_t slave_addr, uint8_t reg_addr) {

	while (I2C1->SR2 & I2C_SR2_BUSY);

	i2c1_start();
	I2C1->DR = slave_addr;
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2;						// dummy read to clear status bit
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// data register needs to be empty
	I2C1->DR = reg_addr;					// set pointer on BNO055
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// data register needs to be empty

	// For single byte reads, pg 482 of reference manual "Closing the communication"
	i2c1_start();
	I2C1->DR = slave_addr | 0x01;			// read mode
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	i2c1_nack();
	(void)I2C1->SR2;						// dummy read to clear status bit
	i2c1_request_stop();
	while (!(I2C1->SR1 & I2C_SR1_RXNE));	// wait for full data register
	return I2C1->DR;
}


void i2c1_readburst(uint8_t slave_addr, uint8_t reg_addr, uint8_t len, uint8_t * bytes) {

	if (len == 0)
		return;

	while (I2C1->SR2 & I2C_SR2_BUSY);

	i2c1_start();
	I2C1->DR = slave_addr;
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2;						// dummy read to clear status bit
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// data register needs to be empty
	I2C1->DR = reg_addr;					// set first pointer on BNO055
	while (!(I2C1->SR1 & I2C_SR1_TXE));		// data register needs to be empty

	i2c1_start();
	I2C1->DR = slave_addr | 0x01;			// read mode
	while (!(I2C1->SR1 & I2C_SR1_ADDR));

	// acknowledge next byte if burst >1 byte
	if (len != 1)
		(void)I2C1->SR2;

	for (uint8_t i = 0; i < len - 1; i++) {

		i2c1_ack();
		while (!(I2C1->SR1 & I2C_SR1_RXNE));	// wait for full data register
		*bytes = I2C1->DR;					// store byte
		bytes++;							// move pointer
	}

	i2c1_nack();								// clear ACK bit just after reading second last data byte
	if (len == 1)
		(void)I2C1->SR2;
	i2c1_request_stop();
	while (!(I2C1->SR1 & I2C_SR1_RXNE));	// wait for full data register
	*bytes = I2C1->DR;
}


void i2c1_start() {

	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));		// wait for start bit generation
}

void i2c1_request_stop() {

	I2C1->CR1 |= I2C_CR1_STOP;
}

void i2c1_ack() {

	I2C1->CR1 |= I2C_CR1_ACK;
}

void i2c1_nack() {

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


void uart1_config(uint32_t baud) {

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

	// USARTDIV = fCK / (8 * (2 - OVER8) * baud)
	// fCK = 84MHz (APB2 bus)
	// Ex. baud = 9600
	// USARTDIV = 546.875
	// Mantissa = 546
	// Fraction = 0.875 * 16 (4 bits for fraction) = 14
	float usartdiv = (float) 84000000 / (16 * baud);
	uint16_t mantissa = (uint16_t) usartdiv;
	uint8_t fraction = (uint8_t) ((usartdiv - mantissa) * 16);
	USART1->BRR = mantissa << 4 | fraction;

	USART1->CR1 |= USART_CR1_TE;		// enable transmitter
}


void uart1_writebyte(uint8_t byte) {

	while (!(USART1->SR & USART_SR_TXE));		// wait until data register is empty
	USART1->DR = byte;			// load register
	while (!(USART1->SR & USART_SR_TC));		// wait until transmission complete
}


void uart1_writeint(int num) {

	if (num < 0)
		uart1_writebyte('-');

	num = abs(num);
	int power = floor(log10(num));		// calculate power of 10

	for (int i = pow(10, power); i > 0; i /= 10)
		uart1_writebyte((num / i) % 10 + '0');
}


void uart1_writestr(char * str) {

	int i = 0;
	while (str[i] != '\0')
		uart1_writebyte(str[i++]);
}


void tim2_config() {

	// frequency determined by TIMx_ARR
	// duty cycle determined by TIMx_CCRx
	// Use PA0 and PA1
	// PA2 and PA3 are used by USART and must be bridged in order to function
	// PA0 is TIM2_CH1 --> A0
	// PA1 is TIM2_CH2 --> A1
	// TIM2 is a 32 bit counter
	// must set AF for pins

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;			// enable TIM2 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;		// enable GPIOA clock

	// select alternate function mode
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0);

	// alternate function mode 1
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1);
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL0_0 | GPIO_AFRL_AFRL1_0;

	TIM2->CR1 = 0;		// no clock division, edge aligned, up counter, counter disabled
	TIM2->PSC = 0;		// /1 prescaler

	TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;		// configure TIM2_CH1 (PA0) to PWM mode
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;		// configure TIM2_CH2 (PA1) to PWM mode
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;		// output compare 1 and 2 preload enable

	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;		// active high, output channels enabled


	TIM2->ARR = 2048;		// output frequency approx 20.5kHz
	TIM2->CCR1 = 0;			// pwm duty cycle of 0
	TIM2->CCR2 = 0;

	TIM2->CR1 = TIM_CR1_ARPE;	// auto reload register preload enable

	TIM2->EGR = TIM_EGR_UG;		// reinitiliaze counter and update registers

	TIM2->CNT = 0;				// reset counter
	TIM2->CR1 |= TIM_CR1_CEN;	// enable counter
}


void motors_config() {

	tim2_config();

	// motor enable inputs are D2 and D3
	// left motor enable is D2
	// right motor enable is D3
	// D2 is PA10
	// D3 is PB3

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
					RCC_AHB1ENR_GPIOBEN;

	// configure D2 and D3 as outputs
	GPIOA->MODER |= GPIO_MODER_MODER10_0;
	GPIOA->MODER &= ~GPIO_MODER_MODER10_1;

	GPIOB->MODER |= GPIO_MODER_MODER3_0;
	GPIOB->MODER &= ~GPIO_MODER_MODER3_1;

	// motor direction inputs are D4 and D5
	// left motor direction is D4
	// right motor direction is D5
	// D4 is PB5
	// D5 is PB4

	// configure D4 and D5 as outputs
	GPIOB->MODER |= GPIO_MODER_MODER4_0 |
					GPIO_MODER_MODER5_0;
	GPIOB->MODER &= ~(GPIO_MODER_MODER4_1 |
					GPIO_MODER_MODER5_1);

	enable_leftmotor();
	enable_rightmotor();
}


void set_pwm_leftmotor(uint8_t direction, uint32_t pwm) {

	uint32_t pwm_adjusted;

	if (direction == MOTOR_FWD) {

		GPIOB->ODR |= GPIO_ODR_ODR_5;
		pwm_adjusted = 2048 - pwm;

	} else {

		GPIOB->ODR &= ~GPIO_ODR_ODR_5;
		pwm_adjusted = pwm;
	}

	TIM2->CCR1 = pwm_adjusted;
}


void set_pwm_rightmotor(uint8_t direction, uint32_t pwm) {

	uint32_t pwm_adjusted;


	if (direction == MOTOR_FWD) {

		GPIOB->ODR |= GPIO_ODR_ODR_4;
		pwm_adjusted = 2048 - pwm;			// invert duty cycle

	} else {

		GPIOB->ODR &= ~GPIO_ODR_ODR_4;
		pwm_adjusted = pwm;
	}

	TIM2->CCR2 = pwm_adjusted;
}


void enable_leftmotor() {

	// turn on D2
	GPIOA->ODR |= GPIO_ODR_OD10;
}


void disable_leftmotor() {

	// turn off D2
	GPIOA->ODR &= ~GPIO_ODR_OD10;
}


void enable_rightmotor() {

	// turn on D2
	GPIOB->ODR |= GPIO_ODR_OD3;
}


void disable_rightmotor() {

	// turn off D3
	GPIOB->ODR &= ~GPIO_ODR_OD3;
}


void tim5_config() {

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;		// enable TIM5 clock

	TIM5->PSC = 0;				// /1 prescaler
	TIM5->ARR = 420000;			// 42MHz clock on APB1, generates a timer overflow at 100Hz

	TIM5->CNT = 0;				// reset counter
	TIM5->EGR |= TIM_EGR_UG;	// update registers

	TIM5->CR1 |= TIM_CR1_CEN;	// enable counter
}


uint8_t imu_config() {


	while (!imu_test());		// read chip id

	// on restart, configure registers by setting mode to CONFIG
	const uint8_t opr_reg = 0x3d;
	const uint8_t opr_mode_config = 0x00;
	i2c1_writebyte(IMU_ADDR, opr_reg, opr_mode_config);

	// delay >19 ms
	for (volatile int i = 0; i < 2000000; i++);

	// select Euler angles in units of degrees
	const uint8_t unit_sel_reg = 0x3b;
	i2c1_writebyte(IMU_ADDR, unit_sel_reg, 0x80);

	// for current setup, placement P3 is used (from BNO055 datasheet)
	// thus, AXIS_REMAP = 0x21 and AXIS_SIGN = 0x02
	const uint8_t axis_map_config_reg = 0x41;
	const uint8_t axis_map_sign_reg = 0x42;
	i2c1_writebyte(IMU_ADDR, axis_map_config_reg, 0x21);
	i2c1_writebyte(IMU_ADDR, axis_map_sign_reg, 0x02);

	// set up fusion sensor mode in OPR_MODE
	const uint8_t opr_mode_fusion = 0x08;
	i2c1_writebyte(IMU_ADDR, opr_reg, opr_mode_fusion);

	// delay >7 ms
	for (volatile int i = 0; i < 2000000; i++);

	// check POST register ST_RESULT
	// returns 1 if all sensors are working
	return ((i2c1_readbyte(IMU_ADDR, IMU_ST_RESULT_REG) & 0x0f) == 0x0f) ? 1 : 0;
}


uint8_t imu_test() {

	// returns 1 if correct chip id is read
	return (i2c1_readbyte(IMU_ADDR, IMU_WHO_AM_I_REG) == 0xa0) ? 1 : 0;
}


void imu_read_euler(int16_t * roll_raw, int16_t * pitch_raw, int16_t * heading_raw) {

	uint8_t raw_euler[6];			// 6 bytes total for euler angles
	const uint8_t start_addr = 0x1a;		// starting byte is heading LSB
	i2c1_readburst(IMU_ADDR, start_addr, 6, raw_euler);		// read 6 consecutive bytes
	*heading_raw = raw_euler[0] | (raw_euler[1] << 8);
	*roll_raw = raw_euler[2] | (raw_euler[3] << 8);
	*pitch_raw = raw_euler[4] | (raw_euler[5] << 8);
}


void convert_euler(int16_t roll_raw, int16_t pitch_raw, int16_t heading_raw, float * roll, float * pitch, float * heading) {

	*roll = roll_raw / 16.0;		// 1 degree = 16 LSB
	*pitch = pitch_raw / 16.0;
	*heading = heading_raw / 16.0;
}
