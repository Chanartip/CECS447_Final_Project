/*
Name:   Chanartip Soonthornwan
        Samuel Poff

Revision 1.0: Date 4/26/2018

 */
 
 /*
    Pins uses
    PD0 GPIO    - IR signal (in)
 */
 
 
#include <stdint.h>
#include "../lib/PLL.h"
#include "../lib/tm4c123gh6pm.h"
#include "../lib/UART.h"

#define IR_IN   (*((volatile unsigned long *)0x40007004))   //PD0

unsigned char IR_check=0;
unsigned char IR_current=0;
unsigned long SAMPLE=0;
unsigned long ERROR=0;
unsigned long ERROR_PERCENT=0;

/***************************************************************************
Inits
***************************************************************************/

void PORTD_Init(){
  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD; // (a) activate clock for port D
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTD_LOCK_R   = GPIO_LOCK_KEY;  // unlock GPIO Port D
  GPIO_PORTD_CR_R     =  0x01;          // allow change to PD0
  GPIO_PORTD_DIR_R   &= ~0x01;          // output PD0
  GPIO_PORTD_AFSEL_R &= ~0x01;          // disable function select PD0
  GPIO_PORTD_DEN_R   |=  0x01;          // enable PD0
  GPIO_PORTD_PCTL_R  &= ~0x0000000F;    // disable port control PD0
  GPIO_PORTD_AMSEL_R &= ~0x01;          // disable analog mode select PD0
}

// SysTick Init, period will be loaded such that the interrupts happen
// at 1ms intervals.
void SysTick_Init(unsigned long period) {
    NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
    NVIC_ST_RELOAD_R = period-1;// reload value
    NVIC_ST_CURRENT_R = 0;      // any write to current clears it
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6
    // enable SysTick with core clock and interrupts
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC+NVIC_ST_CTRL_INTEN;
}

/***************************************************************************
Interrupts, ISRs

***************************************************************************/
unsigned char cur_IR=0, pre_IR=0;
unsigned char Got_IR=0;
unsigned int  IR_current_time=0;
void SysTick_Handler(void){

    if(Got_IR){
        if(IR_
    }
    else{
        // Positive edge detection
        // if current signal is high
        // and previous signal is low,
        // then it is the rising edge of IR signal
        cur_IR = IR_IN;
        if(cur_IR & (pre_IR^0x01)) {
            Got_IR = 1;
            IR_current_time = 0;
        }
        else  pre_IR = cur_IR;
        
    }
    
    IR_current_time++;
    
    
    IR_check = IR_IN;       //Receiving IR input every 40KHz
    SAMPLE++;
    if(IR_check != IR_current) ERROR++;
    ERROR_PERCENT = ERROR*100/SAMPLE;
    
    
    
}

void update_IR_state(){
    
}


int main(void){
    
    PLL_Init();                // bus clock at 50 MHz
    UART0_Init();
    PORTD_Init();              // IR signal digital input
    SysTick_Init(625);        // Systick interrupt every 12.5us (80KHz for IR_receving purpose)
    
    while(1){
        update_IR_state();
    }
}
