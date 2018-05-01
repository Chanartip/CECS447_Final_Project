/***************************************************************************
    Name:       Chanartip Soonthornwan
                Samuel Poff

    Email:      Chanartip.Soonthornwan@gmail.com
                spoff42@gmail.com

    Project:    Project2_part2_UART
    Filename:   MCU1_BT_on_Car.c
    Revision 1.0: Date: 4/26/2018
    Revision 1.1: Date: 4/30/2018
                    - Added Bluetooth function
                    - Added IR signal transmission
                    - Chassie control via bluetooth
                    - Added Chassie State, and IR state
    Revision 1.2: Date: 5/1/2018
                    - Added PortC for Direction control
                    - Removed Chassie State, uses Definition instead
                    - Added cur_speed, pre_speed
***************************************************************************/
/*
    PB0 UART1(RX)  ->  BT(TX)
    PB1 UART1(TX)  ->  BT(RX)
    PB6 M0PWM0     ->  Left wheel PWM
    PB7 M0PWM1     ->  Right wheel PWM
    PC0 GPIO (out) ->  Right wheel direction
    PC1 GPIO (out) ->  Left wheel direction
    PD0 M1PWM0     ->  IR_transmitter
    PF1 GPIO (out) ->  On-board RED LED
    PF2 GPIO (out) ->  On-board BLUE LED
    PF3 GPIO (out) ->  On-board GREEN LED
*/
/***************************************************************************
Includes, defines, prototypes, global variables.
***************************************************************************/

#include <stdint.h>
#include "../lib/tm4c123gh6pm.h"
#include "../lib/PLL.h"
#include "../lib/UART.h"
#include "../lib/PWM.h"

#define LED (*((volatile unsigned long *)0x40025038))        // PF3-1
#define DIR (*((volatile unsigned long *)0x4000600C))       // PC1-0

// Definition for UART
#define _r    0x72
#define _$    0x24
#define _sh   0x23
#define _null 0x00
#define _LF   0x0A
#define _A    0x41
#define _T    0x54

// Definition for PWM
#define P100 25000
#define P75  18750
#define P50  12500
#define P0       2

#define PWM_50P_DUTY 325 // 50% Duty cycle
#define PWM_0P_DUTY 2

// Definition for Car Directions
#define STOP     0x00
#define FORWARD  0x03
#define BACKWARD 0x00
#define LEFT     0x01
#define RIGHT    0x02

//________________IR STATE initialization and definition________________
#define START  0
#define ADDR_1 1
#define ADDR_0 2
#define CMD_3  3
#define CMD_2  4
#define CMD_1  5
#define CMD_0  6

struct State {
    unsigned int  HIGH;         // High time
    unsigned int  LOW;          // Low time
    unsigned char NEXT;         // Next state
};
typedef const struct State Styp;

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
//____________End IR STATE initialization and definition________________

// necessary variables used in the program
unsigned char UART1_busy=0;
unsigned char UART1_data=0;
char bufPt;
unsigned short max=50;
unsigned char IR_current_state=0;
unsigned long IR_current_time=0;
unsigned char IR_busy=0;
unsigned char Got_IR=0;
unsigned int  cur_speed=0;
unsigned int  pre_speed=0;


/***************************************************************************
Inits
***************************************************************************/
/*
 * Port C Initialization for Car direction
 *   PC1 - Left wheel direction
 *   PC0 - Right wheel direction
 */
void PortC_DIR_Init(void){ volatile unsigned long delay;
    SYSCTL_RCGCGPIO_R |= 0x04;            // 2) activate port C
    delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating delay here
    GPIO_PORTF_AFSEL_R &= ~0x03;          // enable alt funct on PC1-0
    GPIO_PORTF_PCTL_R  &= ~0x00000FFF;     // configure PC1-0 as GPIO
    GPIO_PORTF_AMSEL_R &= ~0x03;          // disable analog functionality on PC1-0
    GPIO_PORTF_DIR_R |= 0x03;             // Output PC1-0
    GPIO_PORTF_DEN_R |= 0x03;             // enable digital I/O on PC1-0
}

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
 *
 *      UART1_busy - holding the status that the Bluetooth is busy.
 *      UART1_data - holding the value received from Bluetooth.
 *      dir_L      - Left wheel direction
 *      dir_R      - Right wheel direction
 *      cur_speed  - current speed to assign on PWM
 *      pre_speed  - previous speed to hold speed value before STOP state,
 *                      so after STOP state the car can move without assigning
 *                      new speed.
 *      Got_IR     - holding the status requesting to send IR signal.
 */
void BT_Car_Control(){
    if(UART1_busy){
            if(UART1_data == 'W'){
               //set direction of the car to move forward
                UART0_OutString("Moving Forward\r\n");

                DIR = FORWARD;
                cur_speed = pre_speed;

            }else if(UART1_data == 'A'){
               //set direction of the car to turn left
                UART0_OutString("Turning Left\r\n");

                DIR = LEFT;
                cur_speed = pre_speed;

            }else if(UART1_data == 'S'){
               //set direction of the car to move backward
                UART0_OutString("Moving Backward\r\n");

                DIR = BACKWARD;
                cur_speed = pre_speed;

            }else if(UART1_data == 'D'){
               //set direction of the car to turn right
                UART0_OutString("Turning Right\r\n");

                DIR = RIGHT;
                cur_speed = pre_speed;

            }else if(UART1_data == 'Q'){
               //set direction of the car to Stop
                UART0_OutString("Stopping\r\n");

                DIR = STOP;
                pre_speed = cur_speed;
                cur_speed = P0;

            }else if(UART1_data == 'I'){
               //trigger a function which generate IR signal
                UART0_OutString("Sending IR signal\r\n");
                Got_IR = 1;

            }else if(UART1_data == '1'){
               //set pwm value to 100%
               //   note: 100% for both backward and forward
                LED = (LED&0xF1)+0x04;
                UART0_OutString("PWM 100%\r\n");
                cur_speed = P100;
                pre_speed = cur_speed;

            }else if(UART1_data == '2'){
               //set pwm value to 75%
               //   note: 100% for both backward and forward
                LED = (LED&0xF1)+0x08;
                UART0_OutString("PWM 75%\r\n");
                cur_speed = P75;
                pre_speed = cur_speed;

            }else if(UART1_data == '3'){
               //set pwm value to 50%
               //   note: 100% for both backward and forward
                LED = (LED&0xF1)+0x02;
                UART0_OutString("PWM 50%\r\n");
                cur_speed = P50;
                pre_speed = cur_speed;

            }else{
               // Received an unused character, so reset the buffer.
                UART1_data = _null;
            }

            // Assign Left and Right wheel PWM duty
            PWM0L_Duty(cur_speed);
            PWM0R_Duty(cur_speed);

            UART1_busy=0; // After finished the task, releases the BUSY flag.
        }
        else{
            // Not busy case, do nothing.
        }
}

/*
 * SysTick Init,
 *      period will be loaded such that the interrupts happen at 1ms intervals.
 */
void SysTick_Init(unsigned long period) {
    NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
    NVIC_ST_RELOAD_R = period-1;// reload value
    NVIC_ST_CURRENT_R = 0;      // any write to current clears it
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6
    // enable SysTick with core clock and interrupts
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC+NVIC_ST_CTRL_INTEN;
}

/***************************************************************************
 * Interrupts, ISRs
 *
 * function: receiving a character from UART1 (Bluetooth).
 *           If the input character is null, nothing happen,
 *           but if the input is a character, set BUSY indicating that
 *           the main program will have a further process in BT_Car_Control();
 *
 * Register:
 *     UART1_data - a character register holding input from Bluetooth.
 *     IR_busy    - a flag states there will further character checking.
 *     delay60Hz  - a time counter to count upto 60Hz frequency time for
 *                     updating the Bluetooth input.
 *     Got_IR     - a request to send IR signal.
 *     IR_current_state - current state of IR will be reset to START if
 *                        got the IR transmitting request and the IR is not busy.
 *     IR_busy    - stating the IR is currently transmitting.
 *
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
    Send_IR
function:   a busy wait to send a pre-set IR signal of START+10+1010

precedure:
 -> reset IR time -> start PWM for IR signal for the state high time
 -> reach the time -> stop the PWM -> busy wait for the state low time
 -> update current state -> if finish, exit the function and set IR_busy to zero.

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
    PortC_DIR_Init();           // Direction Initialization
    PortF_LED_Init();           // On-board LEDs Initialization
    PWM0L_Init(25000,12500);    // 50MHz/2= 25MHz; if 1000Hz, 100% period is 25000.
    PWM0R_Init(25000,12500);
    M1_PWM0_PD0_Init(625,2);    // IR_Transmitter PWM init(40KHz, 0%) PWM_clk_rate = Bus_clk/2 = 50MHz/2 = 25MHz.
                                // want 40KHz, so 25MHz/40KHz = 625, therefore 625 is 100% with 2(0%)duty cycle.
                                // To use PWM, set up percentage around 625.
    SysTick_Init( 5000 );       // 100us interrupt

    UART0_OutString(">>> Welcome to Serial Terminal <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> Type W,A,S,D,Q,I,1,2,3 <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> W,A,S,D to control direction <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> Q to stop <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> I to send IR signal<<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> 1,2,3 to set 100% 75% 50% speed respectively. <<<"); UART0_OutChar(CR); UART0_OutChar(LF);

    while(1) {
        BT_Car_Control();
        Send_IR();
    } //end while
} //end main

