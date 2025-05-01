
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "uart1.h"
#include "i2c1.h"


#define IMU_ADDR				0x50
#define IMU_WHO_AM_I_REG		0x00
#define IMU_ST_RESULT_REG		0x36

#define MOTOR_FWD				0
#define MOTOR_BACKWD			1
#define MOTOR_LEFT				0
#define MOTOR_RIGHT				1


// configure system clock to run at 84MHz
void clock_config();

// PWM generation
void pwm_config();
void motors_config();
void motors_set_speed(uint8_t motor, uint8_t dir, uint16_t pwm);
void motors_en();
void motors_dis();

// 100Hz timer to determine when to read sensor
void refresh_tim_config();

uint8_t imu_config();
uint8_t imu_test();
void imu_read_euler(int16_t * roll_raw, int16_t * pitch_raw, int16_t * heading_raw);
void convert_euler(int16_t roll_raw, int16_t pitch_raw, int16_t heading_raw, float * roll, float * pitch, float * heading);


int main(void)
{

	clock_config();
	motors_config();

	refresh_tim_config();

	// ensure IMU connected
	while (!imu_config());


	// PID constants
	const float kp = 0.6;
	const float kd = 0.00;
	const float dt = 1 / 100.0;			// 100Hz fusion refresh rate

	// Critical thresholds
	const float max_err = 30;
	const float min_err = 0.3;
	const float max_pid_out = 10;

	// motor imperfections
	const uint16_t left_max_pwm = 2047;
	const uint16_t left_min_pwm = 750;
	const uint16_t right_max_pwm = 2047;
	const uint16_t right_min_pwm = 750;

	// precompute slopes for map function
	const float left_slope = (float) (left_max_pwm - left_min_pwm) / max_pid_out;
	const float right_slope = (float) (right_max_pwm - right_min_pwm) / max_pid_out;

	const float pitch_setpoint = 0;
	float last_err = 0;

  while (1)
  {
	  TIM5->SR &= ~TIM_SR_UIF;			// clear update interrupt flag
	  while (!(TIM5->SR & (TIM_SR_UIF)));// wait for next fusion data

	  // read orientation
	  int16_t roll_raw, heading_raw, pitch_raw;
	  float roll, heading, pitch;
	  imu_read_euler(&roll_raw, &pitch_raw, &heading_raw);
	  convert_euler(roll_raw, pitch_raw, heading_raw, &roll, &pitch, &heading);

	  // pitch determines angle in our case
	  float pitch_err = pitch - pitch_setpoint;

	  float pid_out = kp * pitch_err + kd * (pitch_err - last_err) / dt;

	  // use absolute value of controller to select pwm duty value
	  // direction of motors is determined by sign of pid_out
	  float pid_abs = fabs(pid_out);

	  // limit the top of the controller
	  if (pid_abs > max_pid_out)
		  pid_abs = max_pid_out;


	  // map function: output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
	  // input: [0, 10]
	  uint32_t left_pwm = left_slope * pid_abs + left_min_pwm;
	  uint32_t right_pwm = right_slope * pid_abs + right_min_pwm;

	  uint8_t motor_dir;

	  if (pid_out > 0)				motor_dir = MOTOR_BACKWD;
	  else							motor_dir = MOTOR_FWD;

	  // if robot passes critical angle, turn off
	  if (fabs(pitch_err) > max_err || fabs(pitch_err) < min_err) {

		  motors_set_speed(MOTOR_LEFT, motor_dir, 0);
		  motors_set_speed(MOTOR_RIGHT, motor_dir, 0);
		  continue;
	  }

	  motors_set_speed(MOTOR_LEFT, motor_dir, left_pwm);
	  motors_set_speed(MOTOR_RIGHT, motor_dir, right_pwm);

	  last_err = pitch_err;

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

void pwm_config() {

	// TIM3 is a 16-bit counter
	// TIM3_CH1 is PA6 (D12)
	// TIM3_CH2 is PA7 (D11)
	// TIM3_CH3 is PB0 (A3)
	// TIM3_CH4 is PB1 (PIN 24)
	// CH1 and CH2 are used to drive left motor
	// CH3 and CH4 are used to drive right motor

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;							// enable TIM3 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;	// enable GPIOA and B clock

	// select alternate function mode for PB0 and PB1
	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1;
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0);
	// select alternate function mode for PA6 and PA7
	GPIOA->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOA->MODER &= ~(GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);

	// select alternate function mode 2 for PB0 and PB1
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1);
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL0_1 | GPIO_AFRL_AFRL1_1;
	// select alternate function mode 2 for PA6 and PA7
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_1 | GPIO_AFRL_AFRL7_1;

	TIM3->CR1 = 0;		// no clock division, edge aligned, up counter, counter disabled
	TIM3->PSC = 0;		// /1 prescaler

	TIM3->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;		// configure TIM3_CH1 (PA6) to PWM mode
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;		// configure TIM3_CH2 (PA7) to PWM mode
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;		// output compare 1 and 2 preload enable

	TIM3->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;		// configure TIM3_CH3 (PB0) to PWM mode
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;		// configure TIM3_CH4 (PB1) to PWM mode
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;		// preload register enable

	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;		// active high, output channels enabled


	TIM3->ARR = 1680;		// output frequency 50kHz
	TIM3->CCR1 = 0;			// pwm duty cycle of 0
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	TIM3->CR1 = TIM_CR1_ARPE;	// auto reload register preload enable

	TIM3->EGR = TIM_EGR_UG;		// reinitiliaze counter and update registers

	TIM3->CNT = 0;				// reset counter
	TIM3->CR1 |= TIM_CR1_CEN;	// enable counter
}


void motors_config() {

	pwm_config();

	// DRV8833 sleep is pin D2 (PA10)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// configure D2 output
	GPIOA->MODER |= GPIO_MODER_MODER10_0;
	GPIOA->MODER &= ~GPIO_MODER_MODER10_1;

	motors_en();
}

void motors_set_speed(uint8_t motor, uint8_t dir, uint16_t pwm) {

	// to set forward PWM on DRV8833, IN1 = PWM, IN2 = 0
	// to set backward PWM, IN1 = 0, IN2 = PWM
	// these are for fast decay mode, enabling responsive motors

	if (motor == MOTOR_LEFT) {

		if (dir == MOTOR_FWD) {

			TIM3->CCR1 = pwm;
			TIM3->CCR2 = 0;
		}
		else if (dir == MOTOR_BACKWD){

			TIM3->CCR1 = 0;
			TIM3->CCR2 = pwm;
		}
	}
	else if (motor == MOTOR_RIGHT) {

		if (dir == MOTOR_FWD) {

			TIM3->CCR3 = pwm;
			TIM3->CCR4 = 0;
		}
		else if (dir == MOTOR_BACKWD) {

			TIM3->CCR3 = 0;
			TIM3->CCR4 = pwm;
		}
	}
}


void motors_en() {

	// turn off PA10 to disable sleep DRV8833
	GPIOA->ODR &= ~GPIO_ODR_OD10;
}


void motors_dis() {

	// turn on PA10 to sleep DRV8833
	GPIOA->ODR |= GPIO_ODR_OD10;
}


void refresh_tim_config() {

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;		// enable TIM5 clock

	TIM5->PSC = 0;				// /1 prescaler
	TIM5->ARR = 840000;			// 84MHz clock on APB1 for TIM5, generates a timer overflow at 100Hz

	TIM5->CNT = 0;				// reset counter
	TIM5->EGR |= TIM_EGR_UG;	// update registers

	TIM5->CR1 |= TIM_CR1_CEN;	// enable counter
}


uint8_t imu_config() {

	i2c1_config();

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
