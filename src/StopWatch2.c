// ****************************************************************
// * TEAM50: T. LUM and R. MARTIN
// * CPEG222 StopWatch2, 10/6/25
// * NucleoF466RE CMSIS STM32F4xx example
// * Display time in tenths of a second on the four digit 7-segment
// * display using SysTick interrupts for counting 100 ms increments
// * Use TIM2 to generate SSD refresh interrupts at 0.5ms intervals
// * Use User Button and interrupts to pause and reset the counter
// ****************************************************************
#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdbool.h>

#define BTN_PIN    (13) // Assuming User Button is connected to GPIOC pin 13
#define BTN_PORT   (GPIOC)
#define FREQUENCY 16000000 // HSI clock frequency

volatile int milliSec = 0;
volatile int digitSelect = 0;
volatile bool PAUSE = false; // Flag to pause counting
volatile uint32_t currentPress = 0; // variable to track time between button presses
volatile uint32_t lastPress = 0; // variable to track time between button presses

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
    if ((currentPress - lastPress) < 1000UL) { // If the time between presses is less than 1 second
      milliSec = 0; // Reset the counter
      PAUSE = false; // Ensure we resume counting
    }else{
      PAUSE = !PAUSE; // Toggle PAUSE state
    }
    lastPress = currentPress; // Update lastPress2 to currentPress
    EXTI->PR |= (1 << BTN_PIN);    // Clear the pending interrupt
  }
}
void SysTick_Handler(void) {
    if(!PAUSE) { // Only increment if not paused
      milliSec++; // Increment the counter if SysTick interrupt occurs
      if (milliSec > 9999) {
          milliSec = 0; // Reset to 0 after reaching 9999
      }
  }
}

int main(void) {
  // 1. Enable clock for GPIO ports A, B, and C
  SSD_init(); // Initialize SSD GPIO pins

  // 2. Enable TIM2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

//   SysTick->LOAD = FREQUENCY/10 -1; 			// Load value for 0.1 second at 16 MHz
//   SysTick->VAL = 0;         				// Clear current value
//   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
//   NVIC_SetPriority(SysTick_IRQn, 1); 	// Set the SysTick priority (optional)
  SysTick_Config(FREQUENCY/10); // Configure SysTick for 100 millisecond interrupts

  // 3. Configure TIM2 for 0.5ms interrupt (assuming 16MHz HSI clock)
  TIM2->PSC = 15; // Prescaler: (16MHz/16 = 1MHz)
  TIM2->ARR = 499; // Auto-reload: 1MHz/2000 = 500Hz (0.5ms period)
  TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
  TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
  NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
  NVIC_SetPriority(TIM2_IRQn, 1); // Set priority for TIM2
  TIM2->CR1 = (1 << 0); // Enable TIM2

  // 4. Configure TIM5 as a free running timer
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable TIM5 clock
  TIM5->PSC = 15999; // Prescaler: (16MHz/16000 = 1kHz, 1msec period)
  TIM5->ARR = 0xFFFFFFFF; // Auto-reload: Max value for free running
  TIM5->EGR = TIM_EGR_UG;  // Update registers
  TIM5->CR1 = TIM_CR1_CEN; // Enable TIM5

  // 5. Configure external interrupt for the button
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock for EXTI
  BTN_PORT->MODER &= ~(3 << (BTN_PIN * 2)); // Set BTN_PIN as input by clearing MODER bits
  EXTI->IMR |= (1 << BTN_PIN);  // Unmask EXTI line 13
  EXTI->FTSR |= (1 << BTN_PIN); // Trigger on falling edge
  SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4)); // Clear EXTI13 bits
  SYSCFG->EXTICR[3] |= (2 << (1 * 4));    // Map EXTI13 to PC13
  NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable EXTI line[15:10] interrupts in NVIC
  NVIC_SetPriority(EXTI15_10_IRQn, 0);    // Set priority for EXTI

  // 6. Set display to 0 initially
  milliSec = 0; // Start with 0

  // 7. Main loop to update the SSD with numbers from 0 to 999.9
  while (1) {
      // A SysTick interrupt will update milliSec every second
      // A TIM2 interrupt will update a SSD every 0.5 ms
      // An external interrupt on the button will pause/resume or reset the counter
      // free running TIM5 is used to measure time between button presses
  }
}