#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"

// Global Variables
unsigned int Count = 0x0;
unsigned int REdge1, REdge2;
//                        Wait 1 sec
void wait_1_sec(){
  startTimerA0();
  startTimerA0();
}


/******************************************************************************
 *
 *
 *                              freqMeas
 * ____________________________________________________________________________
 * Measures the frequency of a signal connected to P2.2/TA1.CCI2B pin
 *
 * Parameters: none
 *
 * Returns: void
 *
 *____________________________________________________________________________

******************************************************************************/


void freqMeas(){
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
    float N_SMCLK; // Number of SMCLK cycles
    float freq; // Measured frequency
    float tmp = 0; // Temporary variable
    float SMCLK_FREQ = 1048576; // SMCLK frequency (2^20)


    unsigned int real_freq; // Measured frequency as an integer
    char strFreq[6] = {'\0'}; // String to hold the measured frequency
    write_freq_tmp_LCD(); // Write template of Frequency to LCD
    TA1CTL |= TASSEL_2 + MC_2 + TACLR; // Start Timer_A with SMCLK as source, in continuous mode, and clear the timer
    while(state == state1){
        disable_interrupts(); // Disable interrupts while measuring frequency
        strFreq[6] = '\0'; // Reset strFreq
        REdge2 = REdge1 =  0; // Reset edge detection variables
        TA1CCTL2 |= CCIE; // Enable interrupt for CCI2B input
        __bis_SR_register(LPM0_bits + GIE); // Enter LPM0 with interrupts enabled
        if(REdge1 == 0 && REdge2 == 0) { // If it's the first time measuring, skip to next iteration
            continue;
        }
        tmp = 1.1915; // Correction factor for measurement error
        N_SMCLK = 0.9*(REdge2 - REdge1)*tmp; // Calculate number of SMCLK cycles
        freq = SMCLK_FREQ / N_SMCLK; // Calculate frequency
        real_freq = (unsigned int) freq ; // Convert frequency to an integer
        if (real_freq == 65535) { // If the frequency is out of range, skip to next iteration (delete later)
            continue;
        }
        sprintf(strFreq, "%d", real_freq); // Convert frequency to a string
        write_freq_tmp_LCD(); // Write template of Frequency to LCD
        lcd_home(); // Move cursor to first position
        lcd_cursor_right(); // Move cursor to position of measured frequency
        lcd_cursor_right();
        lcd_cursor_right();
        lcd_cursor_right();
        lcd_puts(strFreq); // Print measured frequency to LCD
        cursor_off; // Turn off cursor
        DelayMs(500); // Wait 500 ms
        enable_interrupts(); // Re-enable interrupts
    }
    TA1CTL = MC_0 ; // Stop Timer_A


}

//             System Configuration  

void sysConfig(void){ 
	GPIOconfig();
	TIMER0_A0_config();
	TIMER1_A1_config();
	TIMER1_A2_config();
	ADCconfig();
}


// 				Set Byte to Port

void SetByteToPort(char ch){
	PBsArrPortOut |= ch;  
} 

// 				Clear Port Byte

void clrPortByte(char ch){
	PBsArrPortOut &= ~ch;
} 

//            Polling based Delay function

void delay(unsigned int t){  //
	volatile unsigned int i;
	
	for(i=t; i>0; i--);
}


//            Enter from LPM0 mode

void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
	  _BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
        else if(LPM_level == 0x01) 
	  _BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
        else if(LPM_level == 0x02) 
	  _BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
	else if(LPM_level == 0x03) 
	  _BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
        else if(LPM_level == 0x04) 
	  _BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}

//            Enable interrupts

void enable_interrupts(){
  _BIS_SR(GIE);
}

//            Disable interrupts

void disable_interrupts(){
  _BIC_SR(GIE);
}

//            LCD


//             send a command to the LCD

void lcd_cmd(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}

//                       send data to the LCD
void lcd_data(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);
    if (LCD_MODE == FOURBIT_MODE)
    {
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
            LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
    }
    else
    {
            LCD_DATA_WRITE = c;
            lcd_strobe();
    }

    LCD_RS(0);
}

//                write a string of chars to the LCD

void lcd_puts(const char * s){

    while(*s)
        lcd_data(*s++);
}

//    write frequency template to LCD

void write_freq_tmp_LCD(){
   lcd_clear();
   lcd_home();
    const char SquareWaveFreq[] = "fin=";
    const char Hz[] = "Hz";
     lcd_puts(SquareWaveFreq);
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_cursor_right();
     lcd_puts(Hz);
}
//    write signal shape template to LCD

void write_signal_shape_tmp_LCD(){
   lcd_clear();
   lcd_home();
    const char signal_shape[] = "signal shape: ";
     lcd_puts(signal_shape);
     lcd_new_line;
}

// initialize the LCD

/******************************************************************
 * Displays a two-digit integer on the LCD screen.
 * If the integer is less than 10, a leading zero is displayed.
 *
 * @param num The integer to display (should be between 0 and 99).
 */
void lcd_print_num(int num) {
  // Extract the tens and ones digits
  int tens = num / 10 + 0x30;
  int ones = num % 10 + 0x30;

  // Display the tens digit

  lcd_data((char)tens);


  // Display the ones digit
  lcd_data((char)ones);
}




void lcd_init(){

    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();

    if (LCD_MODE == FOURBIT_MODE){
        LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1); //Display Clear
    lcd_cmd(0x6); //Entry Mode
    lcd_cmd(0x80); //Initialize DDRAM address to zero
}

// lcd strobe functions

void lcd_strobe(){
  LCD_EN(1);
  asm("NOP");
 // asm("NOP");
  LCD_EN(0);
}




// The " DelayUs" function  Delay usec functions
//parameter and outputs the corresponding number on an LCD display.
void DelayUs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm("nop"); // tha command asm("nop") takes raphly 1usec

}

// Delay msec functions

void DelayMs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec

}
/*********************************************************************************************
 *
 *                              set pwm
 *
 *____________________________________________________________________________________________
*        Configures Timer_A1 to generate a PWM signal with the specified period and
*        duty cycle.
*
*
*       This function configures Timer_A1 to generate a PWM signal with a duty cycle of 50%.
*       The Timer_A1 clock source should be configured before calling this function.
*       The period parameter should be selected based on the desired frequency of the PWM signal
*       and the Timer_A1 clock frequency.
*************************************************************************************************/
void set_pwm(int period){
    TA1CCR0 = period;
    TA1CCR1 = period/2;

}

void stop_pwm(){
    TA1CTL = MC_0 ; // Stop Timer

}


/******************************************************************************
 *
 *                          get ADC
 *
 *____________________________________________________________________________
* @brief Reads the value from the ADC10 module and returns it.
*
* @return The converted value from the ADC10 module.
*
* @note This function enables the ADC10 module, starts a conversion, waits for the
*       conversion to complete, and then disables the ADC10 module. The converted
*       value is read from the ADC10MEM register and returned.
******************************************************************************/

unsigned int get_ADC(){

    // Turn on ADC10 module
    ADC10CTL0 |= ADC10ON;

    // starts an ADC conversion by setting the ENC (enable conversion)
    // and ADC10SC (start conversion) bits in ADC10CTL0.
    ADC10CTL0 |= ENC + ADC10SC;             // Start sampling


    //wait until the ADC conversion is complete before resuming execution.
    __bis_SR_register(LPM1_bits + GIE);       // Enter LPM0 w/ interrupt

    ADC10CTL0 &= ~ADC10ON;                   //Disable ADC10 interrupt

    //reads the converted value from ADC10MEM.
    return ADC10MEM;

}

/******************************************************************************
 *
 *                          TimerA1 ISR
 *
 *____________________________________________________________________________

******************************************************************************/

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#elif defined(__GNUC__)

void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(TA1IV, 0x0A))
  {
      case  TA1IV_NONE: break;              // Vector  0:  No interrupt
      case  TA1IV_TACCR1:                   // Vector  2:  TACCR1 CCIFG
          TA1CTL &= ~(TAIFG);
        break;
      case TA1IV_TACCR2:                    // Vector  4:  TACCR2 CCIFG
          if (TA1CCTL2 & CCI)                 // Capture Input Pin Status
                  {
                      // Rising Edge was captured
                      if (!Count)
                      {
                          REdge1 = TA1CCR2;
                          Count++;
                      }
                      else
                      {
                          REdge2 = TA1CCR2;
                          TA1CCTL2 &= ~CCIE;
                          Count=0x0;
                          __bic_SR_register_on_exit(LPM0_bits + GIE);  // Exit LPM0 on return to main
                      }


                  }
          break;
   case TA1IV_6: break;                  // Vector  6:  Reserved CCIFG
   case TA1IV_8: break;                  // Vector  8:  Reserved CCIFG
   case TA1IV_TAIFG: break;              // Vector 10:  TAIFG
   default:  break;
  }
}

/******************************************************************************
 *
 *                          TimerA0 ISR
 *
 *____________________________________________________________________________

******************************************************************************/

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{

    LPM0_EXIT;
    TACTL = MC_0+TACLR;
}

/******************************************************************************
 *
 *                          ADC ISR
 *
 *____________________________________________________________________________

******************************************************************************/
#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
    __bic_SR_register_on_exit(CPUOFF);
}


/******************************************************************************
 *
 *                          Buttons ISR
 *
 *____________________________________________________________________________

******************************************************************************/
#pragma vector=PORT1_VECTOR
  __interrupt void PBs_handler(void){
   
	delay(debounceVal);

//            selector of transition between states

	if(PBsArrIntPend & PB0){
	  state = state1;
	  PBsArrIntPend &= ~PB0;
        }
        else if(PBsArrIntPend & PB1){
	  state = state2;
	  PBsArrIntPend &= ~PB1; 
        }
	else if(PBsArrIntPend & PB2){ 
	  state = state3;
	  PBsArrIntPend &= ~PB2;
        }

//            Exit from a given LPM 

        switch(lpm_mode){
		case mode0:
		 LPM0_EXIT; // must be called from ISR only
		 break;
		 
		case mode1:
		 LPM1_EXIT; // must be called from ISR only
		 break;
		 
		case mode2:
		 LPM2_EXIT; // must be called from ISR only
		 break;
                 
                case mode3:
		 LPM3_EXIT; // must be called from ISR only
		 break;
                 
                case mode4:
		 LPM4_EXIT; // must be called from ISR only
		 break;
	}
        
}



//            Port2 Interrupt Service Routine

#pragma vector=PORT2_VECTOR
  __interrupt void PBs_handler_P2(void){
      delay(debounceVal);

//            selector of transition between states

      if(PB3sArrIntPend & PB3){    // For Main Lab
          state = state4;
          PB3sArrIntPend &= ~PB3;
      }

//            Exit from a given LPM

      switch(lpm_mode){
      case mode0:
          LPM0_EXIT; // must be called from ISR only
          break;

      case mode1:
          LPM1_EXIT; // must be called from ISR only
          break;

      case mode2:
          LPM2_EXIT; // must be called from ISR only
          break;

      case mode3:
          LPM3_EXIT; // must be called from ISR only
          break;

      case mode4:
          LPM4_EXIT; // must be called from ISR only
          break;
      }
  }





