#include  "../header/api.h"         // private library - API layer
#include  "../header/app.h"         // private library - APP layer
#include "msp430.h"
enum FSMstate state;
enum SYSmode lpm_mode;

void main(void){

    state = state1;  // start in idle state on RESET
    lpm_mode = mode1;     // start in idle state on RESET
    sysConfig();
    lcd_init();
    lcd_clear();

    P2DIR |= 0xF;

    while(1){
        switch(state){
            case state0: //idle
                enterLPM(mode1);

                break;

            case state1: //PB0
                P2OUT ^= 0x1 ;
                freqMeas();  // Frequency Measurements
                break;

            case state2: //PB1
                P2OUT ^= 0x2 ;

                enable_interrupts();  // Enable to Cut the state
                lcd_clear();          // Clear LCD screen
                CountDown();          // Perform stop watch
                if (state == state2)  // Go to sleep only if timer reached 00:00
                    state = state0;
            break;

            case state3: ; //PB2
                P1OUT ^= 0x4 ;

                enable_interrupts();  // Enable to Cut the state
                tone_generator();
                break;

            case state4: //PB3 For Final_Lab
                P2OUT ^= 0x8 ;

                Signal_shape();
                break;

        }
  }
}







