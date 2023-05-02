#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"


// Global Variables

//unsigned int REdge1, REdge2;
#define ADC_NUMBER_CAPTURES 100
unsigned int adcCaptureValues[ADC_NUMBER_CAPTURES];
unsigned int adcCapturePointer;

//                         CountDown


//                             CountUp
void CountDown(){
      int i;

      char const * startWatch ="01:00";
      lcd_puts(startWatch);
      wait_1_sec();

      for (i = 59 ; i >= 0 ; i--){
        if (state != state2){
            break;
        }

        lcd_clear();
        lcd_home();

        char const * minstr ="00:";
        lcd_puts(minstr);
        lcd_print_num(i);

        wait_1_sec();
     }
}


void CountUp(){
      int i;
      for (i = 0 ; i < 60 ; i++){
        if (state != state2){
            break;
        }

        lcd_clear();
        lcd_home();

        char const * minstr ="00:";
        lcd_puts(minstr);
        lcd_print_num(i);

        wait_1_sec();
     }
}

//              StartTimer For Count Down

void startTimerA0(){
    TACCR0 = 0xFFFF;  // Timer Cycles - max
    TA0CTL = TASSEL_2 + MC_1 + ID_3;  //  select: 2 - SMCLK ;
                                      //control: 3 - Up/Down  ; 
                                      //divider: 3 - /8
    // no ACLCK, we use SMCLK.
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
}


//              Tone Generator

void tone_generator(){

    // constants
    float SMCLK_FREQ = 1048576; // SMCLK freq 2^20
    float coeff = 1.956;  // coeff = 2000 / 1023;
    unsigned int adc_conv;

    //sets up Timer A1 to use the SMCLK (2^20 Hz) and to count up to the value
    //in TA1CCR0 before resetting. This will produce a PWM signal on the corresponding pin.
    TA1CTL = TASSEL_2 + MC_1;                  // SMCLK, upmode
    
    while(state == state3){


        adc_conv = get_ADC();

        //perform linear mapping to convert the analog input to a frequency
        //value using the formula f_out = coeff * adc_conv + 1000

        float f_out = coeff * adc_conv + 1000;
        unsigned int period_to_pwm = SMCLK_FREQ/f_out;

        set_pwm((int)period_to_pwm);
    }
    stop_pwm();
}


//              Signal Shape


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




