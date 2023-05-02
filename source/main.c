#include  "../header/api.h"    		// private library - API layer
#include  "../header/app.h"    		// private library - APP layer

enum FSMstate state;
enum SYSmode lpm_mode;

void main(void){
  
  
  state = state0;
  lpm_mode = mode0;
  sysConfig();
  lcd_init();
  lcd_clear();


  while(1){
	switch(state){
	  case state0: //idle
	      enterLPM(mode0);
	      break;
		 
	  case state1: //PB0 
	    freqMeas();
	    break;

	  case state2: //PB1
	    enable_interrupts();
	    lcd_clear();
        CountUp();
	    CountDown();

            lcd_clear();
            if (state == state2)
               state = state0;
		break;
                    
          case state3: ; //PB2
            lcd_clear();
            enable_interrupts();
            tone_generator();
            
                    break;
                

		
	}
  }
}

  ////updated 9:55
  
  
  
  
  
