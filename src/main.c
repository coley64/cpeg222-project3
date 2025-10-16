// ****************************************************************
// * TEAM50: N. West and H. Collins
// * CPEG222 Project 3A Demo, 10/6/25
// 
// * Display the distance in hundredths of cm/inch on the SSDs. 
// * If distance > 99.99, display 99.99 on SSDs. 
// * User Button (PC13) with interrupt is used to toggle between inches and centimeters. che
// * If the distance is less than 10.00, the first SSD should be blank instead of 0.
// * Use the hardware SysTick timer for 0.5-second interrupts to detect the distance. ✅
// * Write the distance (every 0.5 s)  to the serial monitor (USART2) at 115200 baud ✅
// * Use a general-purpose timer (like TIM2) to generate a 0.5 ms interrupt to update the SSD array every 2 milliseconds. 
// * Use another timer to measure pulse widths.
// ****************************************************************

#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdbool.h>
#include "handler.h"
#include "globals.h"
#define UART_TX_PIN 2 // PA2
#define UART_RX_PIN 3 // PA3
#define UART_PORT GPIOA
#define FREQUENCY 16000000UL // 16 MHz
#define BAUDRATE 115200 // Baud rate for USART2
#define BTN_PIN (13) // Assuming User Button is connected to GPIOC pin 13
#define BTN_PORT (GPIOC)
#define TRIG_PORT GPIOA
#define TRIG_PIN  (4) // PA4
#define ECHO_PORT GPIOB
#define ECHO_PIN  (0) // PB0

int main(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock for EXTI
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  GPIOA->MODER &= ~((3 << (UART_TX_PIN*2)) | (3 << (UART_RX_PIN*2))); // Clear mode bits
  GPIOA->MODER |= (2 << (UART_TX_PIN*2)) | (2 << (UART_RX_PIN*2)); // AF mode
  GPIOA->AFR[0] &= ~((0xF << (UART_TX_PIN*4)) | (0xF << (UART_RX_PIN*4))); // clear AF bits
  GPIOA->AFR[0] |= (7 << (UART_TX_PIN*4)) | (7 << (UART_RX_PIN*4)); // AF7 = USART2

  USART2->BRR = FREQUENCY / BAUDRATE;
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, USART
  
    // TRIG (PA4) → output
  TRIG_PORT->MODER &= ~(3 << (TRIG_PIN * 2));
  TRIG_PORT->MODER |=  (1 << (TRIG_PIN * 2));  // Output mode

  // ECHO (PB0) → input
  ECHO_PORT->MODER &= ~(3 << (ECHO_PIN * 2));  // Input mode
  SSD_init(); // Initialize SSD GPIO pins, ABC clocks
  SysTick_Config(FREQUENCY/2); // Configure SysTick for 500 millisecond interrupts
  configure_tim2();
  configure_button_interrupt();
  configure_tim5();



  milliSec = 0; // Set display to 0 initially

  // confirm usart + serial moniter is working
  for (volatile int i=0; i<1000000; i++); // crude delay
  uart_sendString("CPEG222 Demo Program!\r\nRunning at 115200 baud...\r\n");
  // MAKE SURE BAUD RATE OF SERIAL MONITER IS SET TO 115200

  while (1) {
      // A SysTick interrupt will update milliSec every second
      // A TIM2 interrupt will update a SSD every 0.5 ms
      // An external interrupt on the button will pause/resume or reset the counter
      // free running TIM5 is used to measure time between button presses
  }
}