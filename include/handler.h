#ifndef HANDLER_H
#define HANDLER_H
#include <stdbool.h>

void SysTick_Handler(void);
void TIM2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void configure_tim2(void);
void configure_button_interrupt(void);
void configure_tim5(void);
void uart_sendChar(char c);
void uart_sendString(const char* str);

#endif