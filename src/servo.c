// ****************************************************************
// * TEAM50: T. LUM and R. MARTIN
// * CPEG222 servo_main, 10/6/25
// * NucleoF466RE CMSIS STM32F4xx example
// * Move the standard servo using PWM on PC6
// * Send angle and pulse width data over UART2
// * measured data shows +/- 45 deg rotation using 1ms to 2ms pulse width
// ****************************************************************
#include "stm32f4xx.h"
#include <stdio.h>


#define FREQUENCY   16000000UL // 16 MHz  
#define SERVO3_PIN    (6) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT   (GPIOC)

uint32_t pulse_width = 0; //removed volatile not used in an interrupt

void PWM_Output_PC6_Init(void) {
	// Enable GPIOC and TIM3 clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// Set PC6 to alternate function (AF2 for TIM3_CH1)
	GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
	GPIOC->MODER |=  (0x2 << (SERVO3_PIN * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
	GPIOC->AFR[0] |=  (0x2 << (SERVO3_PIN * 4)); // AF2 = TIM3
	// Configure TIM3 for PWM output on CH1 (PC6)
	TIM3->PSC = (FREQUENCY/1000000) - 1; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
	TIM3->ARR = 19999; // Period for 50 Hz
	TIM3->CCR1 = 1500; // Duty cycle (1.475 ms pulse width)
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable
	TIM3->CCER |= TIM_CCER_CC1E; // Enable CH1 output (PC6)
	TIM3->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	TIM3->EGR = TIM_EGR_UG; // Generate update event
	TIM3->CR1 |= TIM_CR1_CEN; // Enable timer
}

void servo_angle_set(int angle) {
	pulse_width = 1500 - (500 * (angle/45.0)); // Map angle to pulse width (1ms to 2ms)
	TIM3->CCR1 = pulse_width;
}

int main(void) {
	UART2_Init();
	PWM_Output_PC6_Init();
	uart2_send_string("CPEG222 Standard Servo Demo Program!\r\n");
	uart2_send_string("Setting angle to 0 degrees.\r\n");
	int angle = 0;
	servo_angle_set(angle);
	for (volatile int i = 0; i < 10000000UL; ++i); // long delay to adjust the horn at 90 degrees
	while (1) {
		for(int angle = -45; angle <= 45; angle += 5) {
			servo_angle_set(angle);
			uart2_send_string("angle(deg): ");
			uart2_send_int32(angle);
			uart2_send_string("\tservo pulsewidth(us): ");
			uart2_send_int32(pulse_width);
			uart2_send_string("\r\n");
			for (volatile int i = 0; i < 1000000; ++i); // Simple delay	
		}
		for (volatile int i = 0; i < 1000000; ++i); // Simple delay
		for(int angle = 45; angle >= -45; angle -= 5) {
			servo_angle_set(angle);
			uart2_send_string("angle(deg): ");
			uart2_send_int32(angle);
			uart2_send_string("\tservo pulsewidth(us): ");
			uart2_send_int32(pulse_width);
			uart2_send_string("\r\n");
			for (volatile int i = 0; i < 1000000; ++i); // Simple delay
		}
		for (volatile int i = 0; i < 1000000; ++i); // Simple delay	
	}
}