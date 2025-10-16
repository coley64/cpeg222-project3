#include "stm32f4xx.h"
#include "handler.h"
#include <stdbool.h>
#include "globals.h"
#include "SSD_Array.h"

#define BTN_PIN    (13) // Assuming User Button is connected to GPIOC pin 13
#define BTN_PORT   (GPIOC)
#define FREQUENCY 16000000 // HSI clock frequency

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

void SysTick_Handler(void) {
  // Only attempt to send if USART2 has been enabled
  if (USART2->CR1 & USART_CR1_UE) {
    uart_sendString("SysTick alive!\r\n");
  }
}

void TIM2_IRQHandler(void){
  if(TIM2->SR & TIM_SR_UIF){ // Check if the update interrupt flag is set
    SSD_update(digitSelect, milliSec, 3); // Update the SSD with the current value of milliSec
    digitSelect = (digitSelect + 1) % 4; // Cycle through digitSelect values 0 to 3
    TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
  }
}

void EXTI15_10_IRQHandler(void) { // External interrupt handler for button press
  if (EXTI->PR & (1 << BTN_PIN)) { // Check if the interrupt is from BTN_PIN
    currentPress = TIM5->CNT; // Get the current timer count
    if ((currentPress - lastPress) < 250UL) { // If the time between presses is less than 1 second
      milliSec = 0; // Reset the counter
    }
    lastPress = currentPress; // Update lastPress2 to currentPress
    EXTI->PR |= (1 << BTN_PIN);    // Clear the pending interrupt
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
void configure_tim5(void){
    // 4. Configure TIM5 as a free running timer
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable TIM5 clock
  TIM5->PSC = 15999; // Prescaler: (16MHz/16000 = 1kHz, 1msec period)
  TIM5->ARR = 0xFFFFFFFF; // Auto-reload: Max value for free running
  TIM5->EGR = TIM_EGR_UG;  // Update registers
  TIM5->CR1 = TIM_CR1_CEN; // Enable TIM5
}