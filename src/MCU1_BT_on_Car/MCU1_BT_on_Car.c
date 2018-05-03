/***************************************************************************
    Name:       Chanartip Soonthornwan
                Samuel Poff

    Email:      Chanartip.Soonthornwan@gmail.com
                spoff42@gmail.com

    Project:    Project2_part2_UART
    Filename:   MCU1_BT_on_Car.c
    Revision 1.0: Date: 4/26/2018
***************************************************************************/
/*
    PB0 UART1(RX)  ->  BT(TX)
    PB1 UART1(TX)  ->  BT(RX)
    
    PF2 M1PWM6     ->  IR_transmitter
*/
/***************************************************************************
Includes, defines, prototypes, global variables.
***************************************************************************/

#include <stdint.h>
#include "../lib/tm4c123gh6pm.h"
#include "../lib/PLL.h"
#include "../lib/UART.h"

#define LED (*((volatile unsigned long *)0x40025038))		// PF3-1

#define _r    0x72
#define _$    0x24
#define _sh   0x23
#define _null 0x00
#define _LF   0x0A
#define _A    0x41
#define _T    0x54

struct State {
    unsigned int  HIGH;
    unsigned int  LOW;
    unsigned char NEXT;
};
typedef const struct State Styp;
// State Instantiation
#define START  0
#define ADDR_1 1
#define ADDR_0 2
#define CMD_3  3
#define CMD_2  4
#define CMD_1  5 
#define CMD_0  6

#define PWM_50P_DUTY 325 // 50% Duty cycle
#define PWM_0P_DUTY 1

Styp IR_CMD[7] = {
//HIGH, LOW,   NEXT
 {2000,1000, ADDR_1}, 
 {1000, 500, ADDR_0},
 { 500, 500,  CMD_3},
 {1000, 500,  CMD_2},
 { 500, 500,  CMD_1},
 {1000, 500,  CMD_0},
 { 500, 500,  START}
};
    
unsigned char UART1_busy=0;
unsigned char UART1_data=0;
char bufPt;
unsigned short max=50;
unsigned char IR_current_state=0;
unsigned long IR_current_time=0;
unsigned char IR_busy=0;
unsigned char Got_IR=0;

/***************************************************************************
Inits
***************************************************************************/

/*
 * Port F Initialization for LED inidicating PWM power percentage
 *      RED LED - for 50%
 *      GREEN LED - for 75%
 *      BLUE LED - for 100%
 */
void PortF_LED_Init(void){ volatile unsigned long delay;
    SYSCTL_RCGCGPIO_R |= 0x20;            // 2) activate port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating delay here
    GPIO_PORTF_AFSEL_R &= ~0x0E;           // enable alt funct on PF3-1
    GPIO_PORTF_PCTL_R  &= ~0x0000FFF0;     // configure PF3-1 as GPIO
    GPIO_PORTF_AMSEL_R &= ~0x0E;          // disable analog functionality on PF3-1
    GPIO_PORTF_DIR_R |= 0x0E;
    GPIO_PORTF_DEN_R |= 0x0E;             // enable digital I/O on PF3-1
}

/*
    PWM (Module1, PWM0) for Generating IR_signal
*/
void M1_PWM0_PD0_Init(unsigned int period, unsigned int duty){
    
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= 0x02;             // 1) activate PWM1
  SYSCTL_RCGCGPIO_R |= 0x08;            // 2) activate port D
  delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating delay here
  GPIO_PORTD_AFSEL_R |= 0x01;           // enable alt funct on PD0
  GPIO_PORTD_PCTL_R &= ~0x0000000F;     // configure PD0 as M1PWM0
  GPIO_PORTD_PCTL_R |=  0x00000005;
  GPIO_PORTD_AMSEL_R &= ~0x01;          // disable analog functionality on PD0
  GPIO_PORTD_DEN_R |= 0x01;             // enable digital I/O on PD0
  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
  PWM1_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  
  PWM1_0_GENA_R = 0xC8;
  PWM1_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0 
  PWM1_0_CMPA_R = duty - 1;             // 6) count value when output rises
  
  PWM1_0_CTL_R |= 0x00000001;           // 7) start PWM1
  PWM1_ENABLE_R |= 0x00000001;          // enable PF2/M1PWM6
    
}
// change duty cycle of PF2
void PWM_PD0_Duty(unsigned int duty){
  PWM1_0_CMPA_R = duty - 1;             // count value when output rises
}

/*
 * Function: BT_Car_Control
 *      Check if there is an input from UART1(Bluetooth)
 *      If there is, BUSY should be flagged and allow this function
 *      to check the character input.
 *      There are nine specific characters; W,A,S,D,Q,I,1,2,and 3.
 *      W - set direction of the car to move forward
 *      A - set direction of the car to turn left
 *      S - set direction of the car to move backward
 *      D - set direction of the car to turn right
 *      Q - Set PWM of the car to stop
 *      I - trig IR PWM signal
 *      1 - set PWM to 100% for forward and backward with Green LED
 *      2 - set PWM to  75% for forward and backward with Blud LED
 *      3 - set PWM to  50% for forward and backward with Red LED
 */
void BT_Car_Control(){
    if(UART1_busy){
            if(UART1_data == 'W'){
               //set direction of the car to move forward
                UART0_OutString("Got W\r\n");
                
            }else if(UART1_data == 'A'){
               //set direction of the car to turn left
                UART0_OutString("Got A\r\n");
                
            }else if(UART1_data == 'S'){
               //set direction of the car to turn right 
                UART0_OutString("Got S\r\n");
                
            }else if(UART1_data == 'D'){
               //set direction of the car to move backward 
                UART0_OutString("Got D\r\n");
                
            }else if(UART1_data == 'Q'){
               //set direction of the car to Stop
                UART0_OutString("Got Q\r\n");
                
            }else if(UART1_data == 'I'){
               //trigger a function which generate IR signal
                UART0_OutString("Got I\r\n");
                Got_IR = 1;
                UART0_OutChar('R');
            }else if(UART1_data == '1'){
               //set pwm value to 100%
               //   note: 100% for both backward and forward
                LED = (LED&0xF1)+0x04;
                UART0_OutString("Got 1\r\n");
                
            }else if(UART1_data == '2'){
               //set pwm value to 75%
               //   not,  e: 100% for both backward and forward  
                LED = (LED&0xF1)+0x08;
                UART0_OutString("Got 2\r\n");
                
            }else if(UART1_data == '3'){
               //set pwm value to 50%
               //   note: 100% for both backward and forward  
                LED = (LED&0xF1)+0x02; 
                UART0_OutString("Got 3\r\n");
            }else{
               // Received an unused character, so reset the buffer.
                UART1_data = _null;
            }
            
            UART1_busy=0; // After finished the task, releases the BUSY flag.
        }
        else{
            // Not busy case, do nothing.
        }
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

function: receiving a character from UART1 (Bluetooth).
          If the input character is null, nothing happen,
          but if the input is a character, set BUSY indicating that
          the main program will have a further process in BT_Car_Control();

Register:
    UART1_data - a character register holding input from Bluetooth.
    BUSY       - a Flag states there will further character checking.
***************************************************************************/
unsigned char delay60Hz=0;
void SysTick_Handler(void){
    
    // delay60Hz is a variable to keep tracking if the system
    // reaches 60Hz, then it will allow the system to update inputs.
    if(delay60Hz==0){
        UART1_data = UART1_NonBlockingInChar();
        if(UART1_data != _null) UART1_busy=1;
        
        if(Got_IR & !IR_busy){
            Got_IR = 0;
            IR_busy = 1;
            IR_current_state=START;  
        }
        
    }
    delay60Hz = (delay60Hz+1)%167;
    IR_current_time++;

}

/*
    Send_IR()
*/
void Send_IR(void){

    while(IR_busy){
        // Transmit IR for the state HIGH
        IR_current_time=0;
        PWM_PD0_Duty(PWM_50P_DUTY);
        while(IR_current_time < IR_CMD[IR_current_state].HIGH);
        
        // Transmit IR for the state LOW
        PWM_PD0_Duty(PWM_0P_DUTY);
        IR_current_time=0;
        while(IR_current_time < IR_CMD[IR_current_state].LOW);
        
        // Update State
        IR_current_state = IR_CMD[IR_current_state].NEXT;
        if(IR_current_state == START) IR_busy = 0;
    }
    
}


/***************************************************************************
 Main function / loop.
***************************************************************************/
int main( void ) {
    
    PLL_Init();                 // 50MHz
    UART0_Init();               // UART0 (microUSB port)
    UART1_Init();               // UART1 (PB0(RX) to TX pin, PB1(TX) to RX pin)
    PortF_LED_Init();
    M1_PWM0_PD0_Init(625,2);    // IR_Transmitter PWM init(40KHz, 0%) PWM_clk_rate = Bus_clk/2 = 50MHz/2 = 25MHz. 
                                // want 40KHz, so 25MHz/40KHz = 625, therefore 625 is 100% with 2(0%)duty cycle.
                                // To use PWM, set up percentage around 625.
//    SysTick_Init( 83350 );      // 16.67ms(or 60Hz) Interrupts
//    SysTick_Init( 50000 );      // 1ms interrupt
    SysTick_Init( 5000 );      // 100us interrupt

    UART0_OutString(">>> Welcome to Serial Terminal <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> Type W,A,S,D,Q,I,1,2,3 <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    
    while(1) {
        BT_Car_Control();
        Send_IR();
    } //end while
} //end main




