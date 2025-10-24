// ****************************************************************
// * TEAM50: N. West and H. Collins
// * CPEG222 Project 3A Demo, 10/6/25
// 
// * Display the distance in hundredths of cm/inch on the SSDs. 
// * If distance > 99.99, display 99.99 on SSDs. 
// * User Button (PC13) with interrupt is used to toggle between inches and centimeters.
// * If the distance is less than 10.00, the first SSD should be blank instead of 0.
// * Use the hardware SysTick timer for 0.5-second interrupts to detect the distance. ✅
// * Write the distance (every 0.5 s)  to the serial monitor (USART2) at 115200 baud ✅
// * Use a general-purpose timer (like TIM2) to generate a 0.5 ms interrupt to update the SSD array every 2 milliseconds. 
// * Use another timer to measure pulse widths.
// ****************************************************************
#include "stm32f4xx.h"
#include "SSD_Array.h"
#include "configure.h"
#include <stdbool.h>
#include <stdio.h>
#include "UART.h"

#define FREQUENCY 16000000UL
#define UART_PORT GPIOA
#define BTN_PIN 13
#define BTN_PORT GPIOC
// trig pin = PA4, echo pin = PB0
#define TRIG_PORT GPIOA
#define TRIG_PIN 4
#define ECHO_PORT GPIOB
#define ECHO_PIN 0
void servo_angle_set(int angle);

// global variables
volatile bool inches = true;
volatile float distance = 0.0f;
volatile int digitSelect = 0;
volatile uint32_t echo_start = 0;
volatile uint32_t echo_end = 0;
volatile bool echo_received = false;
volatile bool trigger_high = false;
volatile uint32_t currentEdge = 0;
volatile int angle = 0;
uint32_t pulse_width = 0; //removed volatile not used in an interrupt
volatile bool angle_increasing = true;

// --------------------- Interrupt Handlers ---------------------
// updates uart w/ distance + sends pulse to trig
void SysTick_Handler(void) {
  if (USART2->CR1 & USART_CR1_UE) {
    if (angle_increasing) {
        angle += 5;
        servo_angle_set(angle);
        if (angle >= 45) {
            angle_increasing = false;
        }
    } else {
        angle -= 5;
        servo_angle_set(angle);
        if (angle <= -45) {
          angle_increasing = true;
        }
    }

    char str[64];
      if (inches) {
        sprintf(str, "%.2f inch\t", distance);
      } else { 
        sprintf(str, "%.2f cm\t", distance*2.54f);
      }
      uart_send_string(str);
      uart_send_string("angle(deg): ");
      uart_send_int32(angle);
      uart_send_string("\t   servo puslewidth(us): ");
      uart_send_int32(pulse_width);
      uart_send_string("\n");
  }


        // uart_send_string("\tservo pulsewidth(us): ");
        // 
  //This interrupt sends a 10us trigger pulse to the HC-SR04 every 0.5 seconds
  TRIG_PORT->ODR |= (1 << TRIG_PIN); // Set the trigger pin high
  currentEdge = TIM5->CNT; // Get the current timer count
  while ((TIM5->CNT - currentEdge) < 10); // Wait for 10us
  TRIG_PORT->ODR &= ~(1 << TRIG_PIN); // Set the trigger pin low
}

// update SSD
void TIM2_IRQHandler(void) {
    if(TIM2->SR & TIM_SR_UIF) {
      if (inches) {
          // Cap distance at 99.99
          if (distance > 99.99f) {
            SSD_update(digitSelect, 9999, 2);
          } else { SSD_update(digitSelect, distance*100, 2);}
      } else if (!inches) { 
        // Cap distance at 99.99
        if (distance*2.54 > 99.99f) {
            SSD_update(digitSelect, 9999, 2);
        } else {SSD_update(digitSelect, distance*2.54*100, 2);}
      }
        digitSelect = (digitSelect + 1) % 4;
        TIM2->SR &= ~TIM_SR_UIF;
    }
}

// button press -> toggle inches/cm
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << BTN_PIN)) {
        inches = !inches;
        EXTI->PR |= (1 << BTN_PIN); // Clear pending bit
    }
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << ECHO_PIN)) {
        if (GPIOB->IDR & (1 << ECHO_PIN)) {
            // Rising edge - start measurement
            echo_start = TIM5->CNT;
        } else {
            // Falling edge - end measurement
            echo_end = TIM5->CNT;
            uint32_t pulse_width;
            
            // Handle timer overflow
            if (echo_end >= echo_start) {
                pulse_width = echo_end - echo_start;
            } else {
                pulse_width = (0xFFFFFFFF - echo_start) + echo_end;
            }
            distance = pulse_width / 148.0f;
            echo_received = true;
       }
        EXTI->PR |= (1 << ECHO_PIN); // Clear pending bit
  }
}

#define SERVO3_PIN    (6) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT   (GPIOC)

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

// --------------------- Main Program ---------------------

int main(void) {
    configure_clocks();
    configure_uart();
    configure_trig_echo_button();
    configure_button();
    SSD_init();
    configure_tim2();
    configure_tim5();  
    SysTick_Config(FREQUENCY/2); // 0.5 second interrupts
    PWM_Output_PC6_Init();

    // ------------------ Startup message ------------------
    for(volatile int i=0; i<1000000; i++); // Brief delay
    uart_send_string("CPEG222 Demo Program!\r\nRunning at 115200 baud...\r\n");
    servo_angle_set(angle);

    // ------------------ Main loop ------------------
    while(1) {
      //do nothing
		}
  return 0;
}



	// for (volatile int i = 0; i < 10000000UL; ++i); // long delay to adjust the horn at 90 degrees
	// while (1) {
