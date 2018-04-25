/***************************************************************************
    Name:       Chanartip Soonthornwan
                Samuel Poff

    Email:      Chanartip.Soonthornwan@gmail.com
                spoff42@gmail.com

    Project:    Project2_part2_UART
    Filename:   Proj2_MCU2.c
    Revision 1.0: Date: 3/21/2018
    Revision 1.1: Date: 3/21/2018
            - Added PortB_UART1_Init()
            - Added PortE_AIN0_Init()
    Revision 1.2: Date: 3/30/2018
            - Debounced SW0 toggles 'button_says_okay' flag.
            - SysStick counts up in 1ms increments. Every second
              it toggles a flag called 'timer_says_okay'.
    Revision 1.3: Date: 3/31/2018
            - Remove crude delay() by utilizing Systick to count
                a counter to 17ms (~60Hz)

***************************************************************************/



/***************************************************************************
Includes, defines, prototypes, global variables.
***************************************************************************/

#include <stdint.h>
#include "../lib/tm4c123gh6pm.h"
#include "../lib/PLL.h"
#include "../lib/UART.h"

#define _r 0x72
#define _$ 0x24
#define _sh 0x23
#define _null 0x00
#define _LF 0x0A

#define _A 0x41
#define _T 0x54

unsigned char BUSY=0;
unsigned char UART0_in_char;
char bufPt;
unsigned short max=50;

/***************************************************************************
Inits
***************************************************************************/

// SysTick Init, period will be loaded such that the interrupts happen
// at 1ms intervals.
void SysTick_Init(unsigned long period) {
    NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
    NVIC_ST_RELOAD_R = period-1;// reload value
    NVIC_ST_CURRENT_R = 0;      // any write to current clears it
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x80000000; // priority 4
    // enable SysTick with core clock and interrupts
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC+NVIC_ST_CTRL_INTEN;
}

/***************************************************************************
Interrupts, ISRs
***************************************************************************/
unsigned char delay60Hz=0;
void SysTick_Handler(void){

    delay60Hz = (delay60Hz+1)%17; // increment up to 17ms for 60Hz delay
       if(delay60Hz == 0){

            UART0_in_char = UART0_NonBlockingInChar();
            /*
                Check if the first Character is 'A' followed by 'T'.
                if so, set BUSY flag to receive the rest of string.
                else, not set the flag and exit the condition.
             */
            if(UART0_in_char == _A){
                UART0_in_char = UART0_NonBlockingInChar();
                if(UART0_in_char == _T) 
                    BUSY = 1;
                else 
                    BUSY = 0;
            }
                
            /*
                if the "AT" is received, then the Terminal will surely 
                send a string right after the "AT". Therefore, MCU1 should
                split out "AT" with the following string endded by CRLF
                through UART1, so the BlueTooth receiver could response
                back via UART1.
                Then, MCU1 will busy wait the BlueTooth Receiver to respond
                and display the message on Terminal.
             */
            if(BUSY){
                UART0_OutChar(_A);
                UART0_InString(&bufPt, max); // how to deal with long string?
                
                UART0_OutString("AT");
                UART0_OutString(&bufPt);
                UART0_OutChar(CR);
                UART0_OutChar(LF);
                BUSY = 0;
            }
             UART0_OutChar(_sh);   
        }
}


/***************************************************************************
 Main function / loop.
***************************************************************************/
int main( void ) {
    
    PLL_Init();             // 50MHz
    UART0_Init();           // UART0
    UART1_Init();           // UART1
    SysTick_Init( 50000 );  // 1ms Interrupts

    UART0_OutString(">>> Welcome to Serial Terminal <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    UART0_OutString(">>> Type 'AT' and follow with a command <<<"); UART0_OutChar(CR); UART0_OutChar(LF);
    
    
    while(1) {
 

        
  } //end while
} //end main

