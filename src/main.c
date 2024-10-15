/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Feb 7, 2024
  * @brief   ECE 362 Lab 7 student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username!  Even though we're not using an autotest, 
// it should be a habit to fill out your username in this field now.
const char* username = "jwmacdou";

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <stdint.h>

void internal_clock();

// Uncomment only one of the following to test each step
//#define STEP1
//#define STEP2
//#define STEP3
#define STEP4

void init_usart5() {
    // TODO
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    //configure pin PC12 to be routed to USART5_TX ?
    //set moder to alternate function then AFR to correct value
    GPIOC->MODER &= ~(0b11<<24);
    GPIOC->MODER |= 0b10<<24;
    GPIOC->AFR[1] &= ~(0b1111<<16);
    GPIOC->AFR[1] |= 0b1<<17;
    //configure pin PD2 to be routed to USART5_RX ?
    GPIOD->MODER &= ~(0b11<<4);
    GPIOD->MODER |= 0b10<<4;
    GPIOD->AFR[0] &= ~(0b1111<<8);
    GPIOD->AFR[0] |= 0b1<<9;
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
    //Disable USART5 by turning off its UE bit v
    USART5->CR1 &= ~0b1;
    //Set a word size of 8 bits v
    USART5->CR1 &= ~(0b1<<28);
    USART5->CR1 &= ~(0b1<<12);
    //set it for one stop bit (Bit 28 of USART5->CR1, how do you access "n" value for "n stop bits"?) ?
    USART5->CR2 &= ~(0b11<<12);
    //set it for no parity control v
    USART5->CR1 &= ~(0b1<<10);
    //Use 16x oversampling v
    USART5->CR1 &= ~(0b1<<15);
    //Set baud rate to 115.2 KBps (Serial no. 7 on Table 96) v?
    USART5->BRR = 0x1A1;
    //Enable the transmitter and the receiver by setting the TE and RE bits v
    USART5->CR1 |= 0b11<<2;
    //Enable the USART v
    USART5->CR1 |= 0b1;
    //wait for TE and RE bits to be acknowledged by checking TEACK and REACK are both set in the ISR. v ?
    while (!(USART5->ISR & USART_ISR_TEACK) && !(USART5->ISR & USART_ISR_REACK)){

    }
    
    
}

#ifdef STEP1
int main(void){
    internal_clock();
    init_usart5();
    for(;;) {
        while (!(USART5->ISR & USART_ISR_RXNE)) { }
        char c = USART5->RDR;
        while(!(USART5->ISR & USART_ISR_TXE)) { }
        USART5->TDR = c;
    }
}
#endif

#ifdef STEP2
#include <stdio.h>

// TODO Resolve the echo and carriage-return problem

int __io_putchar(int c) {
    // TODO
    if (c == '\n'){
        //if the character passes is a \n first write a \r to USART5->TDR v?
        while (!(USART5->ISR & USART_ISR_TXE)){

        }
        USART5->TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    while (!(USART5->ISR & USART_ISR_RXNE));
    char c = USART5->RDR;
    // TODO
    //if the character read c is a carriage return ('\r'), change it to ('\n') v?
    if (c == '\r'){
        while (!(USART5->ISR & USART_ISR_TXE)){

        }
        USART5->TDR = '\n';
    }
    //echo the character read to the output by calling _io_putchar(c) before it's returned
    __io_putchar(c);
    return c;
}

int main() {
    internal_clock();
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP3
#include <stdio.h>
#include "fifo.h"
#include "tty.h"
int __io_putchar(int c) {
    // TODO
    if (c == '\n'){
        //if the character passes is a \n first write a \r to USART5->TDR v?
        while (!(USART5->ISR & USART_ISR_TXE)){

        }
        USART5->TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    // TODO
    return line_buffer_getchar();
}

int main() {
    internal_clock();
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP4

#include <stdio.h>
#include "fifo.h"
#include "tty.h"

// TODO DMA data structures
#define FIFOSIZE 16
char serfifo [FIFOSIZE];
int seroffset = 0;


void enable_tty_interrupt(void) {
    // TODO
    //raise an interrupt every time the receive data register becomes not empty
    USART5->CR1 |= USART_CR1_RXNEIE;
    USART5->CR3 |= USART_CR3_DMAR;
    NVIC_EnableIRQ(USART3_8_IRQn);
    //trigger a DMA operation every time the receive data register becomes not empty. Do this by enabling DMA mode for reception
    //both of these ^ are flag updates in the USART CR1 and CR3 (and NVIC ISER).

    //Enable RCC clock for DMA controller 2 v
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;
    //-> DMA channel 2 configuration goes here <-
    DMA2_Channel2->CMAR = (uint32_t)serfifo;
    DMA2_Channel2->CPAR = &USART5->RDR;
    DMA2_Channel2->CNDTR = FIFOSIZE;
    DMA2_Channel2->CCR &= ~(0b1<<4);
    DMA2_Channel2->CCR &= ~(0b11<<1);
    DMA2_Channel2->CCR &= ~(0b1111<<8);
    DMA2_Channel2->CCR |= 0b1<<7;
    //PINC SHOULD NOT BE SET, DOES THIS MEAN DISABLED OR JUST NOT SET AT ALL?
    DMA2_Channel2->CCR &= ~(0b1<<6);
    //^
    //Activate circular transfers v [Variable is called CIRC not CIR, is this bad?]
    DMA2_Channel2->CCR |= 0b1<<5;
    DMA2_Channel2->CCR &= ~(0b1<<14);
    DMA2_Channel2->CCR |= 0b11<<12;
    DMA2_Channel2->CCR |=DMA_CCR_EN;
}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    USART_TypeDef *u = USART5;
    // If we missed reading some characters, clear the overrun flag.
    // Wait for a newline to complete the buffer.
    while(fifo_newline(&input_fifo) == 0) {
            asm volatile ("wfi");
    }
    // Return a character from the line buffer.
    char ch = fifo_remove(&input_fifo);
    return ch;
}

int __io_putchar(int c) {
    // TODO copy from STEP2
    if (c == '\n'){
        //if the character passes is a \n first write a \r to USART5->TDR v?
        while (!(USART5->ISR & USART_ISR_TXE)){

        }
        USART5->TDR = '\r';
    }
    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    // TODO Use interrupt_getchar() instead of line_buffer_getchar()
    return interrupt_getchar();
}

// TODO Copy the content for the USART5 ISR here
void USART3_8_IRQHandler(){
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}

// TODO Remember to look up for the proper name of the ISR function

int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();

    setbuf(stdin,0); // These turn off buffering; more efficient, but makes it hard to explain why first 1023 characters not dispalyed
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: "); // Types name but shouldn't echo the characters; USE CTRL-J to finish
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n"); // After, will type TWO instead of ONE
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif