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

#define FREQUENCY 16000000UL
#define UART_PORT GPIOA
#define BTN_PIN 13
#define BTN_PORT GPIOC
// trig pin = PA4, echo pin = PB0
#define TRIG_PORT GPIOA
#define TRIG_PIN 4
#define ECHO_PORT GPIOB
#define ECHO_PIN 0

// global variables
volatile bool inches = true;
volatile float distance = 0.0f;
volatile int digitSelect = 0;
volatile uint32_t echo_start = 0;
volatile uint32_t echo_end = 0;
volatile bool echo_received = false;
volatile bool trigger_high = false;
volatile uint32_t currentEdge = 0;

// --------------------- UART ---------------------
void uart_sendChar(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void uart_sendString(const char* str) {
    while (*str) uart_sendChar(*str++);
}

// --------------------- Interrupt Handlers ---------------------
// updates uart w/ distance + sends pulse to trig
void SysTick_Handler(void) {
  if (USART2->CR1 & USART_CR1_UE) {
    char str[20];
      if (inches) {
        sprintf(str, "%.2f inch\tx degrees\n", distance);
      } else { 
        sprintf(str, "%.2f cm\tx degrees\n", distance*2.54f);
      }
      uart_sendString(str);
  }
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

int main(void) {
    configure_clocks();
    configure_uart();
    configure_trig_echo_button();
    configure_button();
    SSD_init();
    configure_tim2();
    configure_tim5();  
    SysTick_Config(FREQUENCY/2); // 0.5 second interrupts

    // ------------------ Startup message ------------------
    for(volatile int i=0; i<1000000; i++); // Brief delay
    uart_sendString("CPEG222 Demo Program!\r\nRunning at 115200 baud...\r\n");

    // ------------------ Main loop ------------------
    while(1) {
        if (echo_received) {
            echo_received = false;
        }
    }
    return 0;
  }