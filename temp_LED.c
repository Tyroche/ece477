/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
*/

#include <msp430.h>
#include <stdint.h>
#include <string.h>


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

/* I2C Bargraph Commands */
void begin_I2C();
void wait_for_stop();
void i2c_bar_1_set(int, int, int);
void switch_bars(int);

/* LCD Commands */
void shiftout(char a);
void delay(void);
void send_byte(char a);
void send_i(char a);
void chgline(char a);
void print_c(char a);
void pmsglcd(char * a);
void pmsglcd_scroll(char *a, int speed); //Scrolling commands
void print_c_scroll(char a, int speed);
void delay_scroll(int speed);
void send_byte_scroll(char a, int speed);
void clear_lcd(int line);

char * message;
int refval = 0;


//int musictime = 0;
//int infotime = 1;
int infocount = 0;

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

/* LED */
uint8_t i2c_b1_enable   = 0;
uint8_t timer_A0_enable = 0;
uint8_t adc12_0_enable  = 0;

/* Music */
uint8_t spi_a1_enable   = 1;
uint8_t dac12_enable    = 1;
uint8_t timer_A1_enable = 1;
uint8_t bit12_enable	= 0;



/* DMA */
uint8_t dma_in_enable	= 1;
uint8_t dma_out_enable	= 1;

/* LCD */
uint8_t lcd_enable      = 1;
uint8_t timer_A2_enable = 1;

int stop_i2c_b1 = 0;

int push = 0;
int bcnt = 0;
int timcnt = 0;
int buf2full = 0;

uint8_t second_half = 0;
uint16_t input_sound = 0;
uint16_t soundbuf[12288];
uint16_t byte_write_cnt = 0;
uint16_t byte_read_cnt = 0;
uint16_t cur_write_buf = 0;
uint16_t cur_read_buf = 0;
uint8_t firstfour = 0;
uint8_t sampler[16] = {0};
uint16_t average = 0;
uint8_t avgcnt = 0;

/* LCD Stuff */
// ASCII character definitions
int CR = 0x0D;

//;LCD INSTRUCTION CHARACTERS
char LCDON = 0x0F;     //;LCD initialization command
char LCDCLR = 0x01;     //;LCD clear display command
char TWOLINE = 0x38;     //;LCD 2-line enable command
char CURMOV = 0xFE;     //;LCD cursor move instruction
char LINE1 = 0x80;     //;LCD line 1 cursor position
char LINE2 = 0xC0;     //;LCD line 2 cursor position
char LINE3 = 0x94;
char LINE4 = 0xD4;
char MIDLINE = 0x9A;
char CURLINE = 0x00;

uint8_t goforward = 0;

/* For sending song information */
//char inputbuffer[160];
char infobuf[20];
//char infobuf2[20];
//char infobuf3[20];
//char infobuf4[20];
int inputbufcnt = 0;
int change  = 0;



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
	  if (dma_in_enable == 0){
		  DAC12_0CTL0 = DAC12SREF_0 + DAC12AMP_7 + DAC12OG + DAC12ENC + DAC12CALON;
	  }
	  else{
		  //DAC12_0CTL0 = DAC12SREF_0 + DAC12AMP_7 + DAC12OG + DAC12ENC + DAC12CALON + ;
		  DAC12_0CTL0 = DAC12LSEL_0 + DAC12IR + DAC12AMP_7 + DAC12IFG + DAC12ENC + DAC12SREF_3;//dma

	  }


  }

  /* REF START */
  //REFCTL0 |= REFVSEL_0;

  /* SPI A2 Start
  if(spi_a2_enable == 1){
	  P9SEL |= BIT1+BIT2+BIT3;
	  UCA2CTL1 |= UCSWRST;                      // **Put state machine in reset**
	  UCA2CTL0 |= UCCKPH+UCSYNC+UCMSB+UCMST;
	  //UCA2CTL1 |= UCSSEL3;
	  UCA2CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	  UCA2IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  } */


  /* Timer A0 Start */
  if(timer_A0_enable == 1){
	  P1DIR |= 0x02;                            // P1.0 output
	  TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
	  TA0CCR0 = 20000;
	  TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR
	  TA0EX0 = TAIDEX_7;
  }


  /* Timer A1 Start */
  if(timer_A1_enable == 1){
	  	P3DIR |= BIT0+BIT1;                            // P1.0 output
		TA1CCTL0 = CCIE;                          // CCR0 interrupt enabled
		TA1CCR0 = 140; 							  //140 for 8bit 8k  with 90k
		TA1CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR
  }

  /* Timer A2 Start */
  if(timer_A2_enable == 1){
	  TA2CCTL0 = CCIE;                          // CCR0 interrupt enabled
	  TA2CCR0 = 65000;							// LCD Control
	  TA2CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR
  }


  /* ADC 0 Start */
  if(adc12_0_enable == 1)
  {
	  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
	  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
	  ADC12IE |= BIT0;                           // Enable interrupt
	  ADC12CTL0 |= ADC12ENC;
	  P6SEL |= BIT0;                            // P6.0 ADC option select
	  P1DIR |= 0x01;                            // P1.0 output
  }

  /* DMA0_DAC_12 */
    if (dma_out_enable == 1){
  	  //Setup DMA triggers for DMA channel
  	  DMACTL0 = DMA0TSEL_0;    														// software trigger

  	  // Setup DMA0
  	  __data16_write_addr((unsigned short) &DMA0SA, (unsigned long) soundbuf); 		// Source block address
  	  __data16_write_addr((unsigned short) &DMA0DA, (unsigned long) &DAC12_0DAT);	// Destination address
  	  DMA0SZ = 12288;                        										// Block size pf Soundbuf
  	  DMA0CTL = DMADT_4|DMADSTINCR_0|DMASRCINCR_3;//|DMADSTBYTE|DMASRCBYTE; 			// Rpt single ch, inc src, byte-byte

  	  DMA0CTL |=  DMAEN;
    }

    if (dma_in_enable == 1){
  	  //Setup DMA triggers for DMA channel
  	  DMACTL1 = DMA0TSEL_1;    														// software trigger

  	  // Setup DMA1
  	  __data16_write_addr((unsigned short) &DMA1SA, (unsigned long) &input_sound); 		// Source block address
  	  __data16_write_addr((unsigned short) &DMA1DA, (unsigned long) soundbuf);		// Destination address
  	  DMA1SZ = 12288;                        										// Block size pf Soundbuf
	  DMA1CTL = DMADT_4|DMADSTINCR_3|DMASRCINCR_0;//|DMADSTBYTE; 			// Rpt single ch, inc src, byte-byte
  	  DMA1CTL |=  DMAEN;
    }

  /* VOLUME CONTROL */
  P5SEL |= BIT0;

  /* LCD Init Start */
  if(lcd_enable == 1){

	  __bis_SR_register(GIE);
	  /* LCD Output pin enable */
	  P3DIR |= BIT6 + BIT7;
	  P9DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

	  P3OUT &= ~BIT6;
	  P3OUT |= BIT7;
	  P4DIR |= BIT5;

	  send_i(LCDON);
	  send_i(TWOLINE);
	  send_i(LCDCLR);
	  delay();

	  /*
	  chgline(LINE1);
	  pmsglcd("Cody");
	  chgline(LINE2);
	  pmsglcd("Chris");
	  chgline(LINE3);
	  pmsglcd("Aditya");
	  chgline(LINE4);
	  pmsglcd("Arthur");
*/
	  chgline(LINE1);
	  pmsglcd("*. Democratic DJ .*");
	  chgline(LINE2);
	  pmsglcd("Song Title Below:");
	  /*
	  chgline(LINE3);
	  pmsglcd("Song: ");
	  message = "Shake it Off";
	  TA2CTL &= ~MC_0;
	  pmsglcd(message);*/
	  chgline(LINE4);
	  char symb[21] = {0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,'\0'};
	  pmsglcd(symb);

  }

  /* Lights */
  P1DIR |= 0x02;
  P3OUT &= ~BIT1;
  P1OUT |= 0x02;
  P4DIR |= BIT5;


  //P4OUT &= ~BIT7;

  __bis_SR_register(GIE);

  if(i2c_b1_enable == 1){
	  begin_I2C();
  }


  P2SEL &= ~BIT0;
  P2DIR &= ~BIT0;
  P2IES &= ~BIT0;
  P2IFG &= ~BIT0;
  P2IE  |= BIT0;

  /*
  switch_bars(0x71);

  begin_I2C();

  switch_bars(0x70);

  */
  uint8_t delivered = 0;
  while (1)
  {

    //__bis_SR_register(GIE);     // LPM0, ADC12_ISR will force exit
    //__bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_ISR will force exit
    //__no_operation();                       // For debugger

	  /*
	  if( (P4IN & BIT6) && delivered == 0){
	  		  TA2CCTL0 |= CCIE;

	  		  memcpy(infobuf1,&inputbuffer[0],20);
	  		  memcpy(infobuf2,&inputbuffer[40],20);
	  		  memcpy(infobuf3,&inputbuffer[80],20);
	  		  memcpy(infobuf4,&inputbuffer[120],20);

	  		  infobuf1[19] = '\0';
	  		  infobuf2[19] = '\0';
	  		  infobuf3[19] = '\0';
	  		  infobuf4[19] = '\0';

	  		  chgline(LINE1);
	  		  pmsglcd(infobuf1);

	  		  chgline(LINE2);
	  		  pmsglcd(infobuf2);

	  		  chgline(LINE3);
	  		  pmsglcd(infobuf3);

	  		  chgline(LINE4);
	  		  pmsglcd(infobuf4);

	  		  delivered = 1;
	  }
	  */

	  /*
	  if (P4IN & BIT6){
		  infocount = 0;
		  //firstfour = 0;
	  }
	  if (infocount == 20 && delivered == 0){
		  clear_lcd(LINE3);
		  chgline(LINE3);
		  pmsglcd(infobuf);

		  delivered = 1;
	  }*/
	  //ADC12CTL0 |= ADC12SC;
	  //ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
	  //__bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_ISR will force exit
	  //__no_operation();                       // For debugger
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
	//__no_operation();                       // Remain in LPM0 until all data
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

void delay(){

	TA2CTL |= MC_1;
 	while(goforward != 1){}
	TA2CTL &= ~MC_0;
	goforward = 0;
}

void send_byte(char a)
{
     //Shift out character
     P9OUT = 0;
     P9OUT = a;
     //Pulse LCD clock line low->high
     P3OUT &= ~BIT7;
     delay();
     P3OUT |= BIT7;
     //Wait 2 ms for LCD to process data
     delay();
}

void send_i(char a)
{
        //Set the register select line low (instruction data)
        P3OUT &= ~BIT6;
        //Send byte
        send_byte(a);
}

void chgline(char a)
{
  send_i(CURMOV);
  send_i(a);

}

void print_c(char a)
{
  P3OUT |= BIT6;
  send_byte(a);
}

void pmsglcd(char *a)
{
  // for(i = 0; i < 16; i++)
  // {
  //    print_c(" ");
  // }

   while(*a != '\0')
   {
      print_c(*a);
      a++;
   }

}

void pmsglcd_scroll(char *a, int speed)
{

   chgline(MIDLINE);

   if(strlen(message) < 15){
	   pmsglcd(message);
	   return;
   }

   int length = strlen(a);
   char cpy[15];
   if(length > 14){
	   memcpy(cpy,a,14);
	   cpy[14] = '\0';
	   pmsglcd(cpy);

	   char * b = a + 1;
	   pmsglcd_scroll(b,speed);
   }
   else if(length > 10){
	   pmsglcd(a);

	   char * b = a + 1;
	   pmsglcd_scroll(b,speed);
   }
   else{
	   pmsglcd_scroll(message, 1);
   }

}

void print_c_scroll(char a, int speed)
{
  P3OUT |= BIT6;
  send_byte_scroll(a, speed);
}

void delay_scroll(int speed){

	TA2CTL |= MC_1;
	while(goforward != speed){}
	TA2CTL &= ~MC_0;
	goforward = 0;
}

void send_byte_scroll(char a,int speed)
{
     //Shift out character
     P9OUT = 0;
     P9OUT = a;
     //Pulse LCD clock line low->high
     P3OUT &= ~BIT7;
     P3OUT |= BIT7;
     //Wait
     delay_scroll(speed);
}

void clear_lcd(int line){
	chgline(line);
	pmsglcd("                    ");
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
#else
#error Compiler not
supported!
#endif
{
	//__bis_SR_register(GIE);
  while (!(UCRXIFG));             // USCI_A0 RX buffer ready?



  	if (infocount < 20){
  		infobuf[infocount] = UCA1RXBUF;
  		infocount += 1;
  	}
  	else {
		byte_write_cnt++;
		input_sound = UCA1RXBUF;
		DMA1CTL |= DMAREQ;
  	}
  	if(byte_write_cnt == 4096){
	  byte_write_cnt = 0;
	  cur_write_buf = cur_write_buf < 8000 ? cur_write_buf + 4096 : 0;
	  firstfour++;
	  UCA1IFG = 0;
	  P4OUT = 0x00;
  	}


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
			TA1CCTL0 &= ~CCIE;
			TA2CCTL0 &= ~CCIE;

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

		UCB1IFG &= ~UCTXIFG;                  // Clear USCI_B1 TX int flag

		/* Renable interrupts! */
		UCA1IE |= UCRXIE;
		ADC12IE |= BIT0;
		TA0CCTL0 = CCIE;
		TA1CCTL0 = CCIE;
		TA2CCTL0 = CCIE;


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

	//__bis_SR_register(GIE);

	ADC12CTL0 |= ADC12SC;
	//TA1CCTL0 &= ~CCIFG;

	//__bic_SR_register_on_exit(LPM0_bits);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{

	//ADC12CTL0 |= ADC12SC;
	//TA1CCTL0 &= ~CCIFG;
	//P1OUT ^= 0x02;
	//TA2CTL &= ~MC_0;
	__bis_SR_register(GIE);
	if(firstfour >= 3){
		if (dma_in_enable == 0){
		    //DAC12_0DAT = soundbuf[cur_read_buf + byte_read_cnt++];
		}
		else {
			DMA0CTL |= DMAREQ;
			byte_read_cnt++;
		}

		  if(byte_read_cnt == 4096){
			  byte_read_cnt = 0;
			  cur_read_buf = cur_read_buf < 8000 ? cur_read_buf + 4096 : 0;
			  P4OUT |= BIT5;
		  }

		  firstfour = 3;

	}
    //TA2CTL |= MC_1;

	//__bic_SR_register_on_exit(LPM0_bits);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_A0_VECTOR))) TIMER2_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	goforward += 1;
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
		}                     // P1.0 = 0

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

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
		/* Temporarily Turning OFF Play/Pause Function */
		//P4OUT ^= BIT0;
		TA1CCTL0 ^= CCIE;
		/*if(refval == 0){
			REFCTL0 |= REFVSEL_1;
			refval = 1;
		}
		else if(refval == 1){
			REFCTL0 |= REFVSEL_2;
			refval = 2;
		}
		else if(refval == 2){
			REFCTL0 |= REFVSEL_0;
			refval = 0;
		}*/
		P2IFG &= ~BIT0;

	/*
	if(CURLINE == LINE1){
		__bis_SR_register(GIE);
		chgline(ENDLINE1);
		pmsglcd(blank);
		chgline(ENDLINE2);
		pmsglcd(marker);
		CURLINE = LINE2;
	}

	P4OUT ^= BIT0;
	P2IFG &= ~BIT0;
	*/

}