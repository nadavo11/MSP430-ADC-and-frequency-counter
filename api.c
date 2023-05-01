#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"


// Global Variables

//unsigned int REdge1, REdge2;
#define ADC_NUMBER_CAPTURES 100
unsigned int adcCaptureValues[ADC_NUMBER_CAPTURES];
unsigned int adcCapturePointer;
//-------------------------------------------------------------
//                         CountDown
//-------------------------------------------------------------
void CountDown(){
      int i;
      char dozen = 0x35;  // '5'
      char unity = 0x39; // '9'
      for (i =  0 ; i <= 3 ; i++){
        if (state == state2){
            lcd_cmd(0x2);

            if( i == 0){
              char const * startWatch ="01:00";
              lcd_puts(startWatch);
              wait_1_sec();
            }
            else {
              char const * minute_str ="00:";
              lcd_puts(minute_str);
              lcd_data(dozen);
              lcd_data(unity);

              unity --;
              // zero on units
              if( unity == 0x2F){
                unity = 0x39;
                //decrease decimal value
                dozen = dozen-1;
              }
              wait_1_sec();
            }
        }
        else
        {
            break;
        }
      }
}
/////////////////////////////////////////////////////////////////////////
void CountUp(){
      int i;
      char dozen = 0x30;  // '0'
      char unity = 0x31; // '1'
      for (i = 0 ; i < 3 ; i++){
        if (state == state2){
            lcd_clear();
            lcd_home();
            
            if( i == 0){
              char const * startWatch ="00:00";
              lcd_puts(startWatch);
              wait_1_sec();
            }
            else {
              char const * minstr ="00:";
              lcd_puts(minstr);
              lcd_data(dozen);
              lcd_data(unity);

              unity = unity +1;
              if( unity == 0x3A){
                unity = 0x30;
                dozen = dozen+1;
              }
              wait_1_sec();
            }
        }
        else
        {
            break;
        }
      }

}
//-------------------------------------------------------------
//              StartTimer For Count Down
//-------------------------------------------------------------
void startTimerA0(){
    TACCR0 = 0xFFFF;  // Timer Cycles - max
    TA0CTL = TASSEL_2 + MC_1 + ID_3;  //  select: 2 - SMCLK ;
                                      //control: 3 - Up/Down  ; 
                                      //divider: 3 - /8
    // no ACLCK, we use SMCLK.
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
}

//-------------------------------------------------------------
//              Tone Generator
//-------------------------------------------------------------
void tone_generator(){
    TA1CTL = TASSEL_2 + MC_1;                  // SMCLK, upmode
    
    while(state == state3){
        ADC10CTL0 |= ENC + ADC10SC;             // Start sampling
        __bis_SR_register(LPM1_bits + GIE);       // Enter LPM0 w/ interrupt
        ADC10CTL0 &= ~ADC10ON; // Don't get into interrupt

        unsigned int adc_conv = ADC10MEM;
        float coeff = 1.956;  // coeff = 2000 / 1023;
        float f_out = coeff * adc_conv + 1000;  // Choose Linear Mapping

        float SMCLK_FREQ = 1048576; // SMCLK freq 2^20
        unsigned int period_to_pwm = SMCLK_FREQ/f_out;

        TA1CCR0 = period_to_pwm;
        TA1CCR1 = (int) period_to_pwm/2;

    }
    TA1CTL = MC_0 ; // Stop Timer
}

//-------------------------------------------------------------
//              Signal Shape
//-------------------------------------------------------------

void Signal_shape(){
        while(state == state4){
            ADC10CTL0 |= ENC + ADC10SC;             // Start sampling
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            ADC10CTL0 &= ~ADC10ON; // Don't get into interrupt
            char * strLCD;
            if(adcCapturePointer < ADC_NUMBER_CAPTURES) adcCaptureValues[adcCapturePointer++] = ADC10MEM;
            else {
                adcCapturePointer = 0;
                unsigned int i;
                unsigned int low_val_counter;
                unsigned int high_val_counter;
                unsigned int tri_counter;
                low_val_counter = 0;
                high_val_counter = 0;
                tri_counter = 0;
                for (i=1;i<ADC_NUMBER_CAPTURES-1;i++){
                    //pwm
                    if(adcCaptureValues[i] <= 5) low_val_counter ++;
                    else if (adcCaptureValues[i] >= 800) high_val_counter ++;
                    // tri
                    if(abs(2*adcCaptureValues[i]-adcCaptureValues[i-1]-adcCaptureValues[i+1])<3) tri_counter++;


                }
                if (low_val_counter > 5 && high_val_counter > 5) strLCD = "pwm";
                else if (tri_counter > 16) strLCD = "tri";
                else strLCD = "sin";
                DelayMs(500);
                write_signal_shape_tmp_LCD();
                lcd_puts(strLCD);
            }

        }
}




