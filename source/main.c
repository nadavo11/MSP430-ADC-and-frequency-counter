#include  "../header/api.h"    		// private library - API layer
#include  "../header/app.h"    		// private library - APP layer

enum FSMstate state;
enum SYSmode lpm_mode;

void main(void){
  
  
  state = state0;  //START STATE0=IDLE
  lpm_mode = mode0;     // start in idle state on RESET
  sysConfig();
  lcd_init();
  lcd_clear();


  while(1){
	switch(state){
	  case state0: //idle
	      enterLPM(mode0);
	      break;
		 
	  case state1: //PB0 
	    freqMeas();  // Frequency Measurements
	    break;

	  case state2: //PB1
	    enable_interrupts();  // Enable to Cut the state
	    lcd_clear();          // Clear LCD screen
            CountUp();
	    CountDown();          // Perform stop watch
            ////
            ////CountUp();
            lcd_clear();
            if (state == state2)  // Go to sleep only if timer reached 00:00
               state = state0;
		break;
                    
          case state3: ; //PB2
            enable_interrupts();  // Enable to Cut the state
            tone_generator();
            
                    break;
                

		
	}
  }
}

  ////updated 9:55
  
  
  
  
  
