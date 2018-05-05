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
    Revision 1.3: Date 5/3/2018
                    - Changed Car PWM and controls ports
                    - Rearrange codes and comments
    Revision 1.4: Date 5/4/2018
                    - Added current LED and current car speed logic
                    - Completed IR signal transmission
                    - Completed Car speed control and direction control
***************************************************************************/
/*
    PB0 UART1(RX)  ->  BT(TX)
    PB1 UART1(TX)  ->  BT(RX)
    PA2 GPIO       ->  Right wheel control
    PA3 GPIO       ->  Right wheel control
    PA4 GPIO       ->  Left wheel control
    PA5 GPIO       ->  Left wheel control
    PA6 M1PWM2     ->  Left wheel PWM
    PA7 M1PWM3     ->  Right wheel PWM
    PC4 GPRIO      ->  IR_transmitter
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

#define LED (*((volatile unsigned long *)0x40025038))       // PF3-1
#define DIR (*((volatile unsigned long *)0x400040F0))       // PA5-2
#define IR  (*((volatile unsigned long *)0x40006040))       // PC4

// Definition for UART
#define _r    0x72
#define _$    0x24
#define _sh   0x23
#define _null 0x00
#define _LF   0x0A
#define _A    0x41
#define _T    0x54

// Definition for PWM
#define P100 24850
#define P75  18750
#define P50  12500
#define P0       2

#define PWM_50P_DUTY 325 // 50% Duty cycle
#define PWM_0P_DUTY    2 //  0% Duty cycle

// Definition for Car Directions
/* PA  5    4    3    2
 *   In1, In2, In3, In4
 *   In1 and In2 control Motor-A direction
 *   In3 and In4 control Motor-B direction
 * credit: https://tronixlabs.com.au/news/tutorial-l298n-
 *                 dual-motor-controller-module-2a-and-arduino
 *  and    https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf
 */
/* PA| 5  4  3  2 
 *  Q| 0  0  0  0 // STOP     0000_0000
 *  W| 1  0  1  0 // FORWARD  0010_1000
 *  S| 0  1  0  1 // BACKWARD 0001_0100
 *  A| 0  1  1  0 // LEFT     0001_1000
 *  D| 1  0  0  1 // RIGHT    0010_0100
 */
#define STOP     0x00
#define FORWARD  0x28
#define BACKWARD 0x14
#define LEFT     0x18
#define RIGHT    0x24

// necessary variables used in the program
unsigned char UART1_busy=0;         // BT uart1 busy flag
unsigned char UART1_data=0;         // character input from uart1
char bufPt;                         // first index for bufPt string
unsigned short max=50;              // maximum of incoming character string
unsigned long IR_current_time=0;    // counter tracking IR time
unsigned char IR_busy=0;            // IR busy flag
unsigned char Got_IR=0;             // IR transmission request
unsigned int  cur_speed=P50;        // current speed start at 50%
unsigned int  pre_speed=P50;        // previous speed
unsigned int  cur_pwm=P50;          // current PWM duty
unsigned char pre_LED=0x02;         // previous LED, originally start at 50%(RED)

/***************************************************************************
    Initializations
***************************************************************************/
/*
 * Port A Initialization for Car direction
 *   PA5,4 - Left Wheel direction (GPIO out)
 *   PA3,2 - Right Wheel direction (GPIO out)
 */
void PortA_DIR_Init(void){ volatile unsigned long delay;
    SYSCTL_RCGCGPIO_R |= 0x01;            // 2) activate port A
    delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating delay here
    GPIO_PORTA_AFSEL_R &= ~0x3C;          // enable alt funct on PA5-2
    GPIO_PORTA_PCTL_R  &= ~0x00FFFF00;    // configure PA5-2 as GPIO
    GPIO_PORTA_AMSEL_R &= ~0x3C;          // disable analog functionality on PA5-2
    GPIO_PORTA_DIR_R |= 0x3C;             // Output PA5-2
    GPIO_PORTA_DEN_R |= 0x3C;             // enable digital I/O on PA5-2
}

/*
 * Port C Initialization for IR transmitter
 *   PC4 - IR LED (GPIO out)
 */
void PortC_IR_Init(void){ volatile unsigned long delay;
    SYSCTL_RCGCGPIO_R |= 0x04;            // 2) activate port C
    GPIO_PORTC_LOCK_R = GPIO_LOCK_KEY;
    delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating delay here
    GPIO_PORTC_AFSEL_R &= ~0x10;          // enable alt funct on PC4
    GPIO_PORTC_PCTL_R  &= ~0x000F0000;    // configure PC4 as GPIO
    GPIO_PORTC_AMSEL_R &= ~0x10;          // disable analog functionality on PC4
    GPIO_PORTC_DIR_R |= 0x10;             // Output PC4
    GPIO_PORTC_DEN_R |= 0x10;             // enable digital I/O on PC4
    IR = 0;                               // Turn off IR LED 
}

/*
 * Port F Initialization for LED inidicating PWM duty percentage
 *      RED   LED - for  50%  (GPIO out)
 *      GREEN LED - for  75%  (GPIO out)
 *      BLUE  LED - for 100%  (GPIO out)
 */
void PortF_LED_Init(void){ volatile unsigned long delay;
    SYSCTL_RCGCGPIO_R |= 0x20;            // 2) activate port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating delay here
    GPIO_PORTF_AFSEL_R &= ~0x0E;          // enable alt funct on PF3-1
    GPIO_PORTF_PCTL_R  &= ~0x0000FFF0;    // configure PF3-1 as GPIO
    GPIO_PORTF_AMSEL_R &= ~0x0E;          // disable analog functionality on PF3-1
    GPIO_PORTF_DIR_R |= 0x0E;             // Output PF3-1
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
 *      DIR      - Left and Right wheel direction
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
                UART1_OutString("Moving Forward\r\n");
                LED = pre_LED;
                DIR = FORWARD;
                cur_speed = cur_pwm;
                M1PWM2_L_Duty(cur_speed);
                M1PWM3_R_Duty(cur_speed);

            }else if(UART1_data == 'A'){
               //set direction of the car to turn left
                UART1_OutString("Turning Left\r\n");
                LED = pre_LED;
                DIR = LEFT;
                cur_speed = cur_pwm;
                M1PWM2_L_Duty(cur_speed*65/100);
                M1PWM3_R_Duty(cur_speed);

            }else if(UART1_data == 'S'){
               //set direction of the car to move backward
                UART1_OutString("Moving Backward\r\n");
                LED = pre_LED;
                DIR = BACKWARD;
                cur_speed = cur_pwm;
                M1PWM2_L_Duty(cur_speed);
                M1PWM3_R_Duty(cur_speed);

            }else if(UART1_data == 'D'){
               //set direction of the car to turn right
                UART1_OutString("Turning Right\r\n");
                LED = pre_LED;
                DIR = RIGHT;
                cur_speed = cur_pwm;
                M1PWM2_L_Duty(cur_speed);
                M1PWM3_R_Duty(cur_speed*65/100);

            }else if(UART1_data == 'Q'){
               //set direction of the car to Stop
               // if previous state is STOP, keep the state
               // before previous state.
                UART1_OutString("Stopping\r\n");
                if(LED == 0x00) pre_LED = pre_LED; 
                else pre_LED = LED;
                LED = 0;
                DIR = STOP;
                cur_speed = P0;

            }else if(UART1_data == 'I'){
               //trig a flag to send IR signal
                UART1_OutString("Sending IR signal\r\n");
                Got_IR = 1;

            }else if(UART1_data == '1'){
               //set pwm value to 100%
               //   note: 100% for both backward and forward
                LED = (LED&0xF1)+0x04;
                pre_LED = LED;
                UART1_OutString("PWM 100%\r\n");
                cur_pwm = P100;
                cur_speed = cur_pwm;

            }else if(UART1_data == '2'){
               //set pwm value to 75%
               //   note: 100% for both backward and forward
                LED = (LED&0xF1)+0x08;
                pre_LED = LED;
                UART0_OutString("PWM 75%\r\n");
                cur_pwm = P75;
                cur_speed = cur_pwm;

            }else if(UART1_data == '3'){
               //set pwm value to 50%
               //   note: 100% for both backward and forward
                LED = (LED&0xF1)+0x02;
                pre_LED = LED;
                UART1_OutString("PWM 50%\r\n");
                cur_pwm = P50;
                cur_speed = cur_pwm;

            }else{
               // Received an unused character, so reset the buffer.
                UART1_data = _null;
            }

            UART1_busy = 0; // After finished the task, releases the BUSY flag.
        }
        else{
            // Not busy case, do nothing.
        }
}

/*
 * SysTick Init
 *      initializes system timer with Interrupt Enable for
 *      utilizing Systick_Handler as an Interrupt Service Routine(ISR)
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
 * Interrupts, ISR
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
 *     IR_busy    - stating the IR is currently transmitting.
 *     IR_current_time - time to keep tracking IR signal interval
 *     repeat     - to repeat the IR signal before releasing IR_busy
 *
 ***************************************************************************/
unsigned char delay60Hz=0;
unsigned char repeat=0;
void SysTick_Handler(void){

    if(IR_busy){
        
        if( (IR_current_time <  160) ||                          // START_H
            (IR_current_time >= 240 && IR_current_time < 320) || // ADDR1_H
            (IR_current_time >= 360 && IR_current_time < 400) || // ADDR0_H
            (IR_current_time >= 440 && IR_current_time < 520) || // CMD3_H
            (IR_current_time >= 560 && IR_current_time < 600) || // CMD2_H
            (IR_current_time >= 640 && IR_current_time < 720) || // CMD1_H
            (IR_current_time >= 760 && IR_current_time < 800) )  // CMD0_H
        {
            IR ^= 0x10; // generate 40KHz IR signal                      
        }
        // another frame period after finished first IR signal
        else if(IR_current_time >= 1680) {   
            if(repeat==0) {
                IR_current_time = 0;
                repeat++;
            }
            else{    
                IR_busy = 0;
                repeat = 0;
                IR_current_time = 0;
            }
        }
        else{
            IR = 0x00;          // Stay low for Low time
        }
        
    }
    else{
        // delay60Hz is a variable to keep tracking if the system
        // reaches 60Hz, then it will allow the system to update inputs.
        if(delay60Hz==0){
            // Get a character and check if there is an input
            UART1_data = UART1_NonBlockingInChar();
            if(UART1_data != _null) UART1_busy=1;

            if(Got_IR & !IR_busy){
                Got_IR = 0;
                IR_busy = 1;
                IR_current_time=0;
            }
        }
        delay60Hz = (delay60Hz+1)%1333;
        
    }
    
    IR_current_time++;
}


/***************************************************************************
 Main function / loop.
***************************************************************************/
int main( void ) {

    PLL_Init();                 // 50MHz
    UART0_Init();               // UART0 (microUSB port)
    UART1_Init();               // UART1 (PB0(RX) to TX pin, PB1(TX) to RX pin)
    PortA_DIR_Init();           // Car direction Initialization
    PortC_IR_Init();            // IR LED Initialization
    PortF_LED_Init();           // On-board LEDs Initialization
    M1PWM2_L_Init(25000,12500); // 50MHz/2= 25MHz; if 1000Hz, 100% period is 25000.
    SysTick_Init( 625 );        // 80KHz interrupt
    
    UART0_OutString(">>> Welcome to Serial Terminal <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> Type W,A,S,D,Q,I,1,2,3 <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> W,A,S,D to control direction <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> Q to stop <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> I to send IR signal<<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> 1,2,3 to set 100% 75% 50% speed respectively. <<<"); UART0_OutChar(CR); UART0_OutChar(LF);

    while(1) {
        BT_Car_Control();
    } //end while
} //end main

