#include  "../header/bsp.h"    // private library - BSP layer

//-----------------------------------------------------------------------------  
//           GPIO configuration
//-----------------------------------------------------------------------------

void GPIOconfig(void){
 // volatile unsigned int i; // in case of while loop usage
  
  WDTCTL = WDTHOLD | WDTPW;		// Stop WDT
   
  // LCD configuration//////////////////////////////////
  LCD_DATA_WRITE &= ~0xFF;
  LCD_DATA_DIR |= 0xF0;    // P1.4-P1.7 To Output('1')
  LCD_DATA_SEL &= ~0xF0;   // Bit clear P2.4-P2.7
  LCD_CTL_SEL  &= ~0xE0;   // Bit clear P2.5-P2.7
  
  // Generator Setup
  //From the table at CCIx p2.4
  GenPortDir &=  ~BIT4;               // P2.4 Input Capture = '1'
  GenPortSel |=  BIT4;              // P2.4 Select = '1'

  // Buzzer Setup
  BuzzPortDir |= BIT2;             // P2.2 Output compare - '1'
  BuzzPortSel |= BIT2;             // P2.2 Select = '1'
  BuzzPortOut &= ~BIT2;             // P2.2 out = '0'

  // PushButtons Setup
  PBsArrPortSel &= ~0x07;           //
  PBsArrPortOut &= ~0x07;            // Set P1Out to '0'
  PBsArrPortDir &= ~0x07;            // P1.0-2 - Input ('0')
  PBsArrPortDir |= 0x08;             // P1.3 - Output ('1')
  PBsArrIntEdgeSel |= 0x03;  	     // pull-up mode   P1.0-P1.1 - '1'
  PBsArrIntEdgeSel &= ~0x0C;         // pull-down mode  P1.2 - '0'
  PBsArrIntEn |= 0x07;               // P1.0-2 - '1'
  PBsArrIntPend &= ~0xFF;            // clear pending interrupts P1.0-P1.3 all P1
  
  // PushButton 3 Setup For Main Lab
   PB3sArrPortSel &= ~BIT0;           //
   PB3sArrPortOut &= ~BIT0;            // Set P2Out to '0'
   PB3sArrPortDir &= ~BIT0;            // P2.0 - Input ('0')
   PB3sArrIntEdgeSel &= ~BIT0;         // pull-down mode  P2.0 - '0'
   PB3sArrIntEn |= BIT0;               // P1.0-2 - '1'
   PB3sArrIntPend &= ~BIT0;            // clear pending interrupts P2.0


   //realtime

   //PIN 2.1 is the measurement pin
    meas_DIR            |= BIT0;
    meas_SEL            &= ~BIT0;

    meas_OUT            &= ~BIT0;

  _BIS_SR(GIE);                     // enable interrupts globally


}



//-------------------------------------------------------------------------------------
//            Timer1 A2 configuration - For state1
//-------------------------------------------------------------------------------------
void TIMER1_A2_config(void){
    TA1CCTL2 = CAP + CM_1 + CCIE + SCS + CCIS_0; // Timer1 configuration;
    // CM_1 - * Capture mode: 1 - pos. edge */
    // SCS - /* Capture synchronize */
}

//-----------------------------------------------------s--------------------------------
//            Timer1 A1 configuration - For state3
//-------------------------------------------------------------------------------------
void TIMER1_A1_config(void){
    TA1CCTL1 =  OUTMOD_7; // TA1CCR1 reset/set;
}

//------------------------------------------------------------------------------------- 
//            Timer 1sec configuration
//-------------------------------------------------------------------------------------
void TIMER0_A0_config(void){
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    TA0CCTL0 = CCIE;
    TACCR0 = 0xFFFF;
    TA0CTL = TASSEL_2 + MC_0 + ID_3;  //  select: 2 - SMCLK ; control: 3 - Up/Down  ; divider: 3 - /8
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
} 

//------------------------------------------------------------------------------------- 
//            ADC configuration
//-------------------------------------------------------------------------------------
void ADCconfig(void){
      ADC10CTL0 = ADC10SHT_2 + ADC10ON+ SREF_0 + ADC10IE;  // 16*ADCLK+ Turn on, set ref to Vcc and Gnd, and Enable Interrupt
      ADC10CTL1 = INCH_3 + ADC10SSEL_3;     // Input A3 and SMCLK, was |
      ADC10AE0 |= BIT3;                         // P1.3 ADC option select
}
