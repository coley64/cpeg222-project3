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
#include <stdbool.h>
#include <stdio.h>

#define UART_TX_PIN 2 // PA2
#define UART_RX_PIN 3 // PA3
#define UART_PORT GPIOA
#define FREQUENCY 16000000UL
#define BAUDRATE 115200
#define BTN_PIN 13
#define BTN_PORT GPIOC

// trig pin = PA4, echo pin = PB0
#define TRIG_PORT GPIOA
#define TRIG_PIN 4
#define ECHO_PORT GPIOB
#define ECHO_PIN 0

// global variables
volatile bool inches = false;
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
        sprintf(str, "%.2f inch\n", distance * 0.3937f);
      } else { 
        sprintf(str, "%.2f cm\n", distance);
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
        SSD_update(digitSelect, distance*100*0.3937f, 2);
      } else { 
        SSD_update(digitSelect, distance*100, 2);
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
            distance = pulse_width / 58.0f;
            
            // Cap distance at 99.99
            if (distance > 99.99f) {
                distance = 99.99f;
            }
            echo_received = true;
       }
        EXTI->PR |= (1 << ECHO_PIN); // Clear pending bit
    }
}

// --------------------- Main ---------------------
int main(void) {
    // ------------------ Clocks ------------------
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // ------------------ UART ------------------
    GPIOA->MODER &= ~((3 << (UART_TX_PIN*2)) | (3 << (UART_RX_PIN*2)));
    GPIOA->MODER |= (2 << (UART_TX_PIN*2)) | (2 << (UART_RX_PIN*2));
    GPIOA->AFR[0] &= ~((0xF << (UART_TX_PIN*4)) | (0xF << (UART_RX_PIN*4)));
    GPIOA->AFR[0] |= (7 << (UART_TX_PIN*4)) | (7 << (UART_RX_PIN*4));
    USART2->BRR = FREQUENCY / BAUDRATE;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    // ------------------ TRIG (Output) ------------------
    GPIOA->MODER &= ~(3 << (TRIG_PIN*2));
    GPIOA->MODER |= (1 << (TRIG_PIN*2)); // Output mode
    GPIOA->ODR &= ~(1 << TRIG_PIN); // Start low

    // ------------------ ECHO (Input) ------------------
    GPIOB->PUPDR &= ~(3 << (ECHO_PIN*2));
    GPIOB->PUPDR |=  (1 << (ECHO_PIN*2)); // pull-down on PB0 (stable low)
    GPIOB->MODER &= ~(3 << (ECHO_PIN*2)); // Input mode
    // Configure EXTI for ECHO pin (PB0)
    SYSCFG->EXTICR[0] &= ~(0xF << (0*4)); // Clear first 4 bits
    SYSCFG->EXTICR[0] |= (1 << (0*4));    // PB0 (0b0001 for GPIOB)
    EXTI->IMR |= (1 << ECHO_PIN);         // Enable interrupt
    EXTI->RTSR |= (1 << ECHO_PIN);        // Rising edge trigger
    EXTI->FTSR |= (1 << ECHO_PIN);        // Falling edge trigger
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 0);

    // ------------------ Button ------------------
    GPIOC->MODER &= ~(3 << (BTN_PIN*2)); // Input mode
    // Configure EXTI for Button (PC13)
    SYSCFG->EXTICR[3] &= ~(0xF << 4);    // Clear bits for EXTI13
    SYSCFG->EXTICR[3] |= (2 << 4);       // PC13 (0b0010 for GPIOC)
    EXTI->IMR |= (1 << BTN_PIN);         // Enable interrupt
    EXTI->FTSR |= (1 << BTN_PIN);        // Falling edge trigger (button press)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 1);

    // ------------------ SSD + Timers ------------------
    SSD_init();
    
    // TIM2 for SSD multiplexing (0.5ms interrupt)
    TIM2->PSC = 16 - 1;      // Divide by 16
    TIM2->ARR = 500 - 1;     // 0.5ms at 1MHz
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->SR &= ~TIM_SR_UIF;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 2);
    TIM2->CR1 |= TIM_CR1_CEN;
    
    // TIM5 as free-running timer for pulse measurement (1MHz)
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // enable TIM5 clock
    TIM5->PSC = 16 - 1;                  // 16 MHz / 16 = 1 MHz → 1 µs tick
    TIM5->ARR = 0xFFFFFFFF;              // free-run to max
    TIM5->EGR = TIM_EGR_UG;              // force update
    TIM5->CR1 |= TIM_CR1_CEN;            // start counter
    
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