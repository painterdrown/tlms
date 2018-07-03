// PointerTrafficLight.c
// Runs on LM4F120/TM4C123
// Use a pointer implementation of a Moore finite state machine to operate
// a traffic light.
// Daniel Valvano
// September 11, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Volume 2 Program 3.1, Example 3.1

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// east facing red light connected to PB5
// east facing yellow light connected to PB4
// east facing green light connected to PB3
// north facing red light connected to PB2
// north facing yellow light connected to PB1
// north facing green light connected to PB0
// north facing car detector connected to PE1 (1=car present)
// east facing car detector connected to PE0 (1=car present)

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"

#define LIGHT                   (*((volatile uint32_t *)0x400051FC))
#define GPIO_PORTB_OUT          (*((volatile uint32_t *)0x400051FC)) // bits 0~6
#define GPIO_PORTE_IN           (*((volatile uint32_t *)0x4002401C)) // bits 0~2
#define SENSOR                  (*((volatile uint32_t *)0x4002401C))

struct State {
  uint32_t Out;            // 6-bit output
  uint32_t Time;           // 
	uint32_t Color;
  const struct State *Next[8];
};  // depends on 3-bit input
typedef const struct State STyp;
	
#define goN   &FSM[0]  // GREEN
#define waitN &FSM[1]  // YELLOW
#define goE   &FSM[2]  // RED
#define waitE &FSM[3]  // YELLOW
#define Emer  &FSM[4]  // WHITE

#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define GREEN     0x08
#define YELLOW    0x0A
#define WHITE     0x0E
#define WHEELSIZE 4           // must be an integer multiple of 2
const long COLORWHEEL[WHEELSIZE] = {GREEN, YELLOW, RED, WHITE};  // red, yellow, green, white

STyp FSM[5]={
 {0x21, 10, GREEN,  {goN, waitN, goN,   waitN, Emer, Emer, Emer, Emer}},
 {0x22, 03, YELLOW, {goE, goE,   goE,   goE,   Emer, Emer, Emer, Emer}},
 {0x0C, 10, RED,    {goE, goE,   waitE, waitE, Emer, Emer, Emer, Emer}},
 {0x14, 03, YELLOW, {goN, goN,   goN,   goN,   Emer, Emer, Emer, Emer}},
 {0x40, 10, WHITE,  {goN, goN,   goN,   goN,   Emer, Emer, Emer, Emer}}
};

void Ports_Init(void) {
	SYSCTL_RCGCGPIO_R |= 0x32;
	while((SYSCTL_PRGPIO_R&0x0020) == 0){};  // ready?

	// Port F
	GPIO_PORTF_DIR_R |= 0x0E;        // make PF3-1 output (PF3-1 built-in LEDs)
  GPIO_PORTF_AFSEL_R &= ~0x0E;     // disable alt funct on PF3-1
  GPIO_PORTF_DEN_R |= 0x0E;        // enable digital I/O on PF3-1
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;  // configure PF3-1 as GPIO
  GPIO_PORTF_AMSEL_R = 0;          // disable analog functionality on PF

	// Port E
	GPIO_PORTE_DIR_R &= ~0x07;   // make PE2-0 in
  GPIO_PORTE_AFSEL_R &= ~0x07; // disable alt func on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // enable digital I/O on PE2-0
  GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFFFFFFF0)+0x00000000;  // configure PE2-0 as GPIO, 0xFFFFFF00
  GPIO_PORTE_AMSEL_R &= ~0x07; // disable analog functionality on PE2-0

	// Port B
	GPIO_PORTB_DIR_R |= 0x7F;    // make PB6-0 out
  GPIO_PORTB_AFSEL_R &= ~0x7F; // disable alt func on PB6-0
  GPIO_PORTB_DEN_R |= 0x7F;    // enable digital I/O on PB6-0         
  GPIO_PORTB_PCTL_R &= ~0x0FFFFFFF;  // configure PB6-0 as GPIO, 0x00FFFFFF
  GPIO_PORTB_AMSEL_R &= ~0x7F; // disable analog functionality on PB6-0
}



uint32_t Input;
void SysTick_Handler(void){
	Input = (Input+1)%8;
}

void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

int main(void){
	STyp *Pt;                    // state pointer
	uint32_t input;

  PLL_Init();                  // configure for 50 MHz clock
  SysTick_Init(10000);         // initialize SysTick timer
	Ports_Init();                // initialize GPIO ports
	EnableInterrupts();

  Pt = goN;                    // initial state: Green north; Red east
	while(1) {
		LIGHT = Pt->Out;           // set lights to current state's Out value
		LEDS = Pt->Color;          // set LED color
		WaitSeconds(5);
		WaitForInterrupt();
		input = Input;
    Pt = Pt->Next[input];      // transition to next state
  }
}
