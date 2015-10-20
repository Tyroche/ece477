/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
*/

#include <msp430.h>
#include <stdio.h>
#include <stdint.h>


#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3
#define HT16K33_CMD_BRIGHTNESS 0xE0
#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3
#define LED_ON 1
#define LED_OFF 0

void begin_I2C();
void wait_for_stop();
void i2c_bar_1_set(int, int, int);
void switch_bars(int);

int arr[3] = { 0 };
int i;
uint8_t t = 0;
uint8_t b;
int j = 0;
int d = 0;

int i2c_bar_1_addr  = 0x00;
int i2c_bar_1_red   = 0x00;
int i2c_bar_1_green = 0x00;

int redtest = 1;
int greentest = 0;
int tracker = 1;

int both_bars = 0;

int i2c_b1_enable = 1;
int spi_a1_enable = 1;
int dac12_enable  = 1;
int spi_a2_enable = 0;
int timer_A0_enable = 1;
int adc12_0_enable = 1;
int timer_A1_enable = 0;

int stop_i2c_b1 = 0;

int push = 0;
int bcnt = 0;
int timcnt = 0;
int buf2full = 0;

uint8_t soundbuf[4][4096];

int main(void)
{

	WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer

  /* I2C B Start */
  if(i2c_b1_enable == 1){
	  UCB1CTL1 |= UCSWRST;                      // Enable SW reset
	  P8SEL |= BIT5+BIT6;                        // P3.0,1 option select (0 = SDA | 1 = SCL)
	  UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
	  UCB1CTL1 = UCSSEL_2 + UCTR;            // Use SMCLK
	  UCB1BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
	  UCB1BR1 = 0;
	  UCB1I2CSA = 0x70;                         // Slave Address is 70h
	  UCB1CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
	  UCB1IE |= UCTXIE;
  }


  /* SPI A1 Start */
  if(spi_a1_enable == 1){
	  P8SEL |= BIT1+BIT2+BIT3+BIT4;                  // P3.3,4 option select (2 = SS | 3 = SIMO | 4 = SOMI)
	  UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
	  UCA1CTL0 |= UCCKPH+UCSYNC+UCMSB+UCMODE_2;
	  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	  UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  }

  /* DAC12_A Start */
  if(dac12_enable == 1){
	  P6SEL |= BIT6+BIT7;
	  DAC12_0CTL0 = DAC12SREF_0+DAC12AMP_7+DAC12OG+DAC12ENC+DAC12CALON;
  }

  /* SPI A2 Start */
  if(spi_a2_enable == 1){
	  P9SEL |= BIT1+BIT2+BIT3;
	  UCA2CTL1 |= UCSWRST;                      // **Put state machine in reset**
	  UCA2CTL0 |= UCCKPH+UCSYNC+UCMSB+UCMST;
	  //UCA2CTL1 |= UCSSEL3;
	  UCA2CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	  UCA2IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  }


  /* Timer A0 Start */
  if(timer_A0_enable == 1){
	  P1DIR |= 0x02;                            // P1.0 output
	  TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
	  TA0CCR0 = 1000;
	  TA0CTL = TASSEL_2 + MC_1 + TACLR + ID_3;         // SMCLK, upmode, clear TAR
	  TA0EX0 = TAIDEX_7;
  }


  /* Timer A0 Start */
  if(timer_A1_enable == 1){
	  P3DIR |= BIT0+BIT1;                            // P1.0 output
	  TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
	  TA0CCR0 = 62;
	  TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR
  }

  /* ADC 0 Start */
  if(adc12_0_enable == 1)
  {
	  ADC12CTL0 = ADC12SHT02 + ADC12ON + ADC12REFON;         // Sampling time, ADC12 on
	  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
	  ADC12IE = 0x01;                           // Enable interrupt
	  ADC12CTL0 |= ADC12ENC;
	  P6SEL |= 0x01;                            // P6.0 ADC option select
	  P1DIR |= 0x01;                            // P1.0 output
  }

  /* Lights */
  P1DIR |= 0x02;
  P3OUT &= ~BIT1;
  P1OUT |= 0x02;

  //P4OUT &= ~BIT7;


  __bis_SR_register(GIE);

  if(i2c_b1_enable == 1){
	  begin_I2C();
  }

  /*
  switch_bars(0x71);

  begin_I2C();

  switch_bars(0x70);

  */

  int delay1;
  int delay2;
  while (1)
  {
	  if(adc12_0_enable == 1){
		  /*
		  for(delay1 = 0; delay1 < 1000; delay1++){
			  for(delay2 = 0; delay2 < 1000; delay2++){

			  }
		  } */
		  //ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
	  }
    //__bis_SR_register(GIE);     // LPM0, ADC12_ISR will force exit
    __bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_ISR will force exit
    __no_operation();                       // For debugger
  }
}

void begin_I2C(){
	UCB1CTL1 |= UCTR + UCTXSTT;
	wait_for_stop();
	UCB1CTL1 |= UCTR + UCTXSTT;
	wait_for_stop();
	UCB1CTL1 |= UCTR + UCTXSTT;
	wait_for_stop();
	return;
}

void wait_for_stop(){
	__bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
	__no_operation();                       // Remain in LPM0 until all data
	while (UCB1CTL1 & UCTXSTP);
}

void i2c_bar_1_set(int address, int red, int green){
	i2c_bar_1_addr  = address;
	i2c_bar_1_red   = red;
	i2c_bar_1_green = green;

	//switch_bars(0x70);

	UCB1CTL1 |= UCTR + UCTXSTT;
	wait_for_stop();

	/*
	switch_bars(0x71);

	UCB1CTL1 |= UCTR + UCTXSTT;
	wait_for_stop();

	switch_bars(0x70);

	*/

	return;
}

void switch_bars(int address){
	UCB1CTL1 |= UCSWRST;
	UCB1I2CSA = address;
	UCB1CTL1 &= ~UCSWRST;
}




#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not
supported!
#endif
{
  while (!(UCRXIFG));             // USCI_A0 RX buffer ready?
  //__bis_SR_register(GIE);
  //int r = UCA0RXBUF;
  //printf("Recieved information: %d\n\n", UCA0RXBUF);
  /*
  switch(UCA1RXBUF){
  	  case 1:
  		 P1OUT |= BIT1;
  		 break;
  	  case 2:
  		 P1OUT |= BIT2;
  	  	 break;
  	  case 3:
  		 P1OUT &= ~BIT1;
  		 break;
  	  default: break;
  }*/

  //int task = UCA1RXBUF / 16;
  /*
  if(bcnt == 0){
	  buf[0] = UCA1RXBUF >> 4;
	  bcnt = 1;
  }
  else if(bcnt == 1){
	  buf[1] = UCA1RXBUF << 4;
	  bcnt = 0;

	  DAC12_0DAT = buf[1] ^ buf[0];
  }*/

  P3OUT |= 0x03;
  DAC12_0DAT = UCA1RXBUF;

  UCA1IFG = 0;
}


//------------------------------------------------------------------------------
// The USCIAB1_ISR is structured such that it can be used to transmit any
// number of bytes by pre-loading TXByteCtr with the byte count.
//------------------------------------------------------------------------------


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B1_VECTOR))) USCI_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
	//__bis_SR_register(GIE);
 	if  (j == 0){
		UCB1TXBUF = 0x21;
		t = 0;
		j++;
	}
	else if(j == 1){
		UCB1CTL1 |= UCTXSTP;                  // I2C stop condition
		UCB1IFG &= ~UCTXIFG;                  // Clear USCI_B1 TX int flag
		__bic_SR_register_on_exit(LPM0_bits);
		j++;
	}
	else if(j == 2){
		UCB1TXBUF = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | HT16K33_BLINK_OFF;
		j++;
	}
	else if(j == 3){
		UCB1CTL1 |= UCTXSTP;                  // I2C stop condition
		UCB1IFG &= ~UCTXIFG;                  // Clear USCI_B1 TX int flag
		__bic_SR_register_on_exit(LPM0_bits);
		j++;
	}
	else if(j == 4){
			UCB1TXBUF = HT16K33_CMD_BRIGHTNESS | 15;
			j++;
	}
	else if(j == 5){
		UCB1CTL1 |= UCTXSTP;                  // I2C stop condition
		UCB1IFG &= ~UCTXIFG;                  // Clear USCI_B1 TX int flag
		__bic_SR_register_on_exit(LPM0_bits);
		/*if(both_bars == 0){
			both_bars++;
			j = 0;
		}
		else{
			j = 6;
		}*/
		j++;
	}
	else if(j == 6){
		UCB1TXBUF = i2c_bar_1_addr;
			j++;

			/* Atomicity Interupt Disables */
			UCA1IE &= ~UCRXIE;
			ADC12IE = 0x00;
			TA0CCTL0 &= ~CCIE;
	}
	else if(j == 7){
		UCB1TXBUF = i2c_bar_1_red;
		j++;
	}

	else if(j == 8){
	  	UCB1TXBUF = i2c_bar_1_green;
	  	j++;
	}

	else{
		j = 6;

		UCB1CTL1 |= UCTXSTP;                  // I2C stop condition

		/* Renable interrupts! */
		UCA1IE |= UCRXIE;
		ADC12IE = 0x01;
		TA0CCTL0 = CCIE;

		UCB1IFG &= ~UCTXIFG;                  // Clear USCI_B1 TX int flag
		__bic_SR_register_on_exit(LPM0_bits);
	}
}



// Timer0 A0 interrupt service routine

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	/*
	if(bcnt > 4096){
		DAC12_0DAT = buf1[timcnt++];
	}
	else{
		if(buf2full == 1){
			DAC12_0DAT = buf2[timcnt++];
		}
	}

	if(timcnt == 4095){
		timcnt = 0;
	}

	P3OUT ^= 0x03;
	*/
	ADC12CTL0 |= ADC12SC;
	TA1CCTL0 &= ~CCIFG;
	__bic_SR_register_on_exit(LPM0_bits);
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
	    //__bis_SR_register(GIE);
	  	//UCB1IE |= UCTXIE;
	  	 if(ADC12MEM0 <= 0x7FF){
	  		 P1OUT &= 0x00;
	  	 }
	  	 else{
	  		 P1OUT |= 0x02;
	  	 }


	  	/* Last 7 Bars, Red */
		if (ADC12MEM0 >= 0xFF0){
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0xF0, 0x0F);
			i2c_bar_1_set(4, 0xFF, 0x0F);
		}
		else if (ADC12MEM0 >= 0xF46) {
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0xF0, 0x0F);
			i2c_bar_1_set(4, 0x7F, 0x0F);
	  	}
		else if (ADC12MEM0 >= 0xE9C) {
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0xF0, 0x0F);
			i2c_bar_1_set(4, 0x3F, 0x0F);
	  	}
		else if (ADC12MEM0 >= 0xDF2) {
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0xF0, 0x0F);
			i2c_bar_1_set(4, 0x1F, 0x0F);
		}
		else if (ADC12MEM0 >= 0xD48) {
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0xF0, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		else if (ADC12MEM0 >= 0xC9E) {
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0x70, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		else if (ADC12MEM0 >= 0xBF4) {
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0x30, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		else if (ADC12MEM0 >= 0xB4A) {
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0x10, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		/* Second 8 Bars, Yellow */
		else if (ADC12MEM0 >= 0xAA0) {
			i2c_bar_1_set(0, 0xF0, 0xFF);
			i2c_bar_1_set(2, 0x00, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		else if (ADC12MEM0 >= 0x9F6) {
			i2c_bar_1_set(0, 0x70, 0x7F);
			i2c_bar_1_set(2, 0x00, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		else if (ADC12MEM0 >= 0x94C) {
			i2c_bar_1_set(0, 0x30, 0x3F);
			i2c_bar_1_set(2, 0x00, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		else if (ADC12MEM0 >= 0x8A2) {
			i2c_bar_1_set(0, 0x10, 0x1F);
			i2c_bar_1_set(2, 0x00, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		else if (ADC12MEM0 >= 0x7F8) {
			i2c_bar_1_set(0, 0x00, 0x0F);
			i2c_bar_1_set(2, 0x00, 0x0F);
			i2c_bar_1_set(4, 0x0F, 0x0F);
		}
		else if (ADC12MEM0 >= 0x74E) {
			i2c_bar_1_set(0, 0x00, 0x0F);
			i2c_bar_1_set(2, 0x00, 0x0F);
			i2c_bar_1_set(4, 0x07, 0x07);
		}
		else if (ADC12MEM0 >= 0x6A4) {
			i2c_bar_1_set(0, 0x00, 0x0F);
			i2c_bar_1_set(2, 0x00, 0x0F);
			i2c_bar_1_set(4, 0x03, 0x03);
		}
		else if (ADC12MEM0 >= 0x5FA) {
			i2c_bar_1_set(0, 0x00, 0x0F);
			i2c_bar_1_set(2, 0x00, 0x0F);
			i2c_bar_1_set(4, 0x01, 0x01);
		}
		/* First 8 Bars, Green */
		else if (ADC12MEM0 >= 0x550) {
			i2c_bar_1_set(0, 0x0, 0x0F);
			i2c_bar_1_set(2, 0x0, 0x0F);
			i2c_bar_1_set(4, 0x00, 0);
		}
		else if (ADC12MEM0 >= 0x4A6) {
			i2c_bar_1_set(0, 0x0, 0x0F);
			i2c_bar_1_set(2, 0x0, 0x07);
			i2c_bar_1_set(4, 0x00, 0);
		}
		else if (ADC12MEM0 >= 0x3FC) {
			i2c_bar_1_set(0, 0x0, 0x0F);
			i2c_bar_1_set(2, 0x0, 0x03);
			i2c_bar_1_set(4, 0x00, 0);
		}
		else if (ADC12MEM0 >= 0x352) {
			i2c_bar_1_set(0, 0x0, 0x0F);
			i2c_bar_1_set(2, 0x0, 0x01);
			i2c_bar_1_set(4, 0x00, 0);
		}
		else if (ADC12MEM0 >= 0x2A8) {
			i2c_bar_1_set(0, 0x0, 0x0F);
			i2c_bar_1_set(2, 0x00, 0);
			i2c_bar_1_set(4, 0x00, 0);
		}
		else if (ADC12MEM0 >= 0x1FE) {
			i2c_bar_1_set(0, 0x0, 0x07);
			i2c_bar_1_set(2, 0x00, 0);
			i2c_bar_1_set(4, 0x00, 0);
		}
		else if (ADC12MEM0 >= 0x154) {
			i2c_bar_1_set(0, 0x0, 0x03);
			i2c_bar_1_set(2, 0x00, 0);
			i2c_bar_1_set(4, 0x00, 0);
		}
		else if (ADC12MEM0 >= 0xAA) {
			i2c_bar_1_set(0, 0x0, 0x01);
			i2c_bar_1_set(2, 0x00, 0);
			i2c_bar_1_set(4, 0x00, 0);
		}
		else{
			i2c_bar_1_set(0, 0x00, 0);
			i2c_bar_1_set(2, 0x00, 0);
			i2c_bar_1_set(4, 0x00, 0);
		}


    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}


