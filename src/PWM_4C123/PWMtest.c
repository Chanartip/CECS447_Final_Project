// PWMtest.c
// Runs on TM4C123
// Use PWM0/PB6 and PWM1/PB7 to generate pulse-width modulated outputs.
// Daniel Valvano
// March 28, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 6.7, section 6.3.2

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
#include <stdint.h>
#include "PLL.h"
#include "PWM.h"
#include "tm4c123gh6pm.h"
#include "SysTick.h"

#define DL (*((volatile unsigned long *)0x40007008))
#define DR (*((volatile unsigned long *)0x40007004))

unsigned long MODE=0;

void WaitForInterrupt(void);  // low power mode

void SysTick_Handler(){
    unsigned long static count = 0;
    unsigned long static count2 = 0;
    
    if(count == 500000) {   // 10 msec
        count = 0;
        count2++;
    }
    else count++;
    
    if(count2 == 1000){     // 10 sec
        MODE = (MODE+1)%5;
    }
    
}

void portD_init(){
  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD; // (a) activate clock for port D
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTD_LOCK_R   = GPIO_LOCK_KEY;  // unlock GPIO Port D
  GPIO_PORTD_CR_R     =  0x03;          // allow change to PD1,0
  GPIO_PORTD_DIR_R   |=  0x03;          // output PD1,0
  GPIO_PORTD_AFSEL_R &= ~0x03;          // disable function select PD1,0
  GPIO_PORTD_DEN_R   |=  0x03;          // enable PD1,0
  GPIO_PORTD_PCTL_R  &= ~0x000000FF;    // disable port control PD1,0
  GPIO_PORTD_AMSEL_R &= ~0x03;          // disable analog mode select PD1,0
}

int main(void){
    
    PLL_Init();                      // bus clock at 50 MHz
    SysTick_Init();                    
    PWM0A_Init(40000, 30000);        // initialize PWM0, 1000 Hz, 75% duty
    PWM0B_Init(40000, 30000);        // initialize PWM0, 1000 Hz, 75% duty
    
    while(1){
        switch(MODE){
            case '1': {PWM0A_Init(40000, 30000); PWM0B_Init(40000, 30000); DL=0; DR=0; break;} // Forward
            case '2': {PWM0A_Init(40000, 10000); PWM0B_Init(40000, 10000); DL=1; DR=1; break;} // Backward
            case '3': {PWM0A_Init(40000, 10000); PWM0B_Init(40000, 30000); DL=1; DR=0; break;} // Left Turn
            case '4': {PWM0A_Init(40000, 30000); PWM0B_Init(40000, 10000); DL=0; DR=1; break;} // Right Turn
            default : {PWM0A_Init(40000, 30000); PWM0B_Init(40000, 30000); DL=0; DR=0; break;} // Forward
        }
        
        /*
            dir | pwm | 
             0  |  25 | forward
             0  |  50 | 
             0  |  75 | 
             0  | 100 | 
             1  |  25 | backward
             1  |  50 | 
             1  |  75 | 
             1  | 100 | 
        */
        
    }
}
