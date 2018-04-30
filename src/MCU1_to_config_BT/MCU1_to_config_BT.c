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

#define _r    0x72
#define _$    0x24
#define _sh   0x23
#define _null 0x00
#define _LF   0x0A
#define _A    0x41
#define _T    0x54

    
unsigned char UART1_busy=0;
unsigned char UART1_data=0;
char bufPt;
unsigned short max=50;

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
        
    }
    delay60Hz = (delay60Hz+1)%167;

}

/***************************************************************************
 Main function / loop.
***************************************************************************/
int main( void ) {
    unsigned char set=0;
    PLL_Init();                 // 50MHz
    UART0_Init();               // UART0 (microUSB port)
    UART1_Init();               // UART1 (PB0(RX) to TX pin, PB1(TX) to RX pin)
    SysTick_Init( 5000 );      // 100us interrupt

    UART0_OutString(">>> Welcome to Serial Terminal <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> Type 'AT' and follow with a command <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    
    while(1) {
        if(set <= 10){
            UART1_OutString("AT+NAME=CSSP\r\n");
            UART1_OutString("AT+UART=57600,1,0\r\n");
            UART1_OutString("AT+PSWD=1112\r\n");
            UART1_OutString("AT+ROLE=0\r\n");
            set++;
        }
        if(set == 10){
            UART0_OutString("Config BT Name: CSSP\r\n");
            UART0_OutString("Config BT UART: 57600, 8-bit, 1 stop, odd Parity\r\n");
            UART0_OutString("New Passcode: 1112\r\n");
            UART0_OutString("Config as Slave Mode\r\n");
            UART0_OutString("Finish Configuration\r\n");
            set++;
        }
    } //end while
} //end main




