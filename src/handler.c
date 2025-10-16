#include "stm32f4xx.h"
#include "handler.h"
#include <stdbool.h>
#include "globals.h"
#include "SSD_Array.h"

#define BTN_PIN    (13) // Assuming User Button is connected to GPIOC pin 13
#define BTN_PORT   (GPIOC)
#define FREQUENCY 16000000 // HSI clock frequency
#define TRIG_PORT GPIOA
#define TRIG_PIN  (4) // PA4
#define ECHO_PORT GPIOB
#define ECHO_PIN  (0) // PB0

volatile uint32_t echo_start = 0;
volatile uint32_t echo_end = 0;
volatile bool echo_done = false;
int currentEdge = 0; // Variable to store the timer count at the current edge

// UART helper functions
void uart_sendChar(char c) {
  while (!(USART2->SR & USART_SR_TXE)); // Wait until transmit data register is empty
  USART2->DR = c;
}
void uart_sendString(const char* str) {
  while (*str) {
  uart_sendChar(*str++);
  }
}

//sysTick interrupt handler for sending distance to USART2
void SysTick_Handler(void) {
  // Only attempt to send if USART2 has been enabled
  if (USART2->CR1 & USART_CR1_UE) {
    if (inches) {
      char str[20];
      sprintf(str, "%.2f inch\n", distance);
      uart_sendString(str);
    }
    else {
      char str[20];
      sprintf(str, "%.2f cm\n", distance*2.54f);
      uart_sendString(str);
    }
  }
}

// tim2 interrupt handler for SSD multiplexing
void TIM2_IRQHandler(void){
  if(TIM2->SR & TIM_SR_UIF){ // Check if the update interrupt flag is set
    SSD_update(digitSelect, milliSec, 3); // Update the SSD with the current value of milliSec
    digitSelect = (digitSelect + 1) % 4; // Cycle through digitSelect values 0 to 3
    TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
  }
}

// tim5 interrupt handler for measuring pulse width, fires every 500ms
void TIM5_IRQHandler(void){
  if(TIM5->SR & TIM_SR_UIF){ // Check if the update interrupt flag is set
    TRIG_PORT->ODR |= (1 << TRIG_PIN); // Set the trigger pin high
    currentEdge = TIM3->CNT; // Get the current timer count
    while ((TIM3->CNT - currentEdge) < 10); // Wait for 10us
    TRIG_PORT->ODR &= ~(1 << TRIG_PIN); // Set the trigger pin low
    TIM5->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
  }
}

void EXTI15_10_IRQHandler(void) { // External interrupt handler for button press
  if (EXTI->PR & (1 << BTN_PIN)) { // Check if the interrupt is from BTN_PIN
    if (inches) {
      inches = false;
    } else {
      inches = true;
    }
    EXTI->PR |= (1 << BTN_PIN); // Clear the pending interrupt
  }
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << ECHO_PIN)) {
        if (ECHO_PORT->IDR & (1 << ECHO_PIN)) {
            // Rising edge → start timing
            echo_start = TIM3->CNT;
        } else {
            // Falling edge → stop timing
            echo_end = TIM3->CNT;
            uint32_t pulse = echo_end - echo_start;
            distance = pulse / 58.3f; // cm (HC-SR04 constant)
            echo_done = true;
        }
        EXTI->PR |= (1 << ECHO_PIN); // Clear interrupt flag
    }
}

void configure_tim2(void){
  TIM2->PSC = 15; // Prescaler: (16MHz/16 = 1MHz)
  TIM2->ARR = 499; // Auto-reload: 1MHz/2000 = 500Hz (0.5ms period)
  TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
  TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
  NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
  NVIC_SetPriority(TIM2_IRQn, 1); // Set priority for TIM2
  TIM2->CR1 = (1 << 0); // Enable TIM2
}

void configure_button_interrupt(void){
  BTN_PORT->MODER &= ~(3 << (BTN_PIN * 2)); // Set BTN_PIN as input by clearing MODER bits
  EXTI->IMR |= (1 << BTN_PIN);  // Unmask EXTI line 13
  EXTI->FTSR |= (1 << BTN_PIN); // Trigger on falling edge
  SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4)); // Clear EXTI13 bits
  SYSCFG->EXTICR[3] |= (2 << (1 * 4));    // Map EXTI13 to PC13
  NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable EXTI line[15:10] interrupts in NVIC
  NVIC_SetPriority(EXTI15_10_IRQn, 0);    // Set priority for EXTI
}

// Configure TIM5 to generate an interrupt every 500ms to handle ultrasonic sensor triggering
void configure_tim5(void){
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable TIM5 clock
  TIM5->PSC = 15999; // Prescaler: (16MHz / 16000 = 1kHz, 1ms per tick)
  TIM5->ARR = 499;   // Auto-reload: 500ms period (500 ticks of 1ms)
  TIM5->DIER |= TIM_DIER_UIE; // Enable update interrupt
  TIM5->SR &= ~TIM_SR_UIF;    // Clear any pending flag
  NVIC_EnableIRQ(TIM5_IRQn);  // Enable TIM5 interrupt in NVIC
  NVIC_SetPriority(TIM5_IRQn, 1); // Set interrupt priority
  TIM5->CR1 = TIM_CR1_CEN;    // Enable counter
}

// configure tim3 as a background timer
void configure_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;   // Enable TIM3 clock
    TIM3->PSC = 15;        // Prescaler: (16 MHz / (15+1)) = 1 MHz → 1 tick = 1 µs
    TIM3->ARR = 0xFFFFFFFF; // Let it count full 32-bit range (about 71 minutes before overflow)
    TIM3->EGR = TIM_EGR_UG; // Update registers
    TIM3->CR1 |= TIM_CR1_CEN; // Start TIM3
}
