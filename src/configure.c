#include "configure.h"
#include "stm32f4xx.h"  // include MCU definitions

#define FREQUENCY 16000000UL
#define BAUDRATE 115200
#define TRIG_PIN 4
#define ECHO_PIN 0
#define BTN_PIN 13

void configure_clocks(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void configure_trig_echo_button(void){
    //trig
    GPIOA->MODER &= ~(3 << (TRIG_PIN*2));
    GPIOA->MODER |= (1 << (TRIG_PIN*2)); // Output mode
    GPIOA->ODR &= ~(1 << TRIG_PIN); // Start low

    //echo
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
}
void configure_button(void){
    GPIOC->MODER &= ~(3 << (BTN_PIN*2)); // Input mode
    // Configure EXTI for Button (PC13)
    SYSCFG->EXTICR[3] &= ~(0xF << 4);    // Clear bits for EXTI13
    SYSCFG->EXTICR[3] |= (2 << 4);       // PC13 (0b0010 for GPIOC)
    EXTI->IMR |= (1 << BTN_PIN);         // Enable interrupt
    EXTI->FTSR |= (1 << BTN_PIN);        // Falling edge trigger (button press)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 1);
}
void configure_tim2(void){
    // TIM2 for SSD multiplexing (0.5ms interrupt)
    TIM2->PSC = 16 - 1;      // Divide by 16
    TIM2->ARR = 500 - 1;     // 0.5ms at 1MHz
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->SR &= ~TIM_SR_UIF;
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 2);
    TIM2->CR1 |= TIM_CR1_CEN;
}
void configure_tim5(void){
    // TIM5 as free-running timer for pulse measurement (1MHz)
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // enable TIM5 clock
    TIM5->PSC = 16 - 1;                  // 16 MHz / 16 = 1 MHz → 1 µs tick
    TIM5->ARR = 0xFFFFFFFF;              // free-run to max
    TIM5->EGR = TIM_EGR_UG;              // force update
    TIM5->CR1 |= TIM_CR1_CEN;            // start counter
}