/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
*/

#include <msp430.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>


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

/* LED Commands */
void print_Wifi();
char * message;
char marker[2] = {0xF3,'\0'};
char blank[1] = {" "};

/* For sending song information */
char inputbuffer[160];
char infobuf1[20];
char infobuf2[20];
char infobuf3[20];
char infobuf4[20];
int inputbufcnt = 0;
int change  = 0;

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

int capture = 0;

/* LED */
uint8_t i2c_b1_enable   = 0;
uint8_t timer_A0_enable = 0;
uint8_t adc12_0_enable  = 0;

/* Music */
uint8_t spi_a1_enable   = 0;
uint8_t dac12_enable    = 0;
uint8_t timer_B0_enable = 0;
uint8_t musictime 		= 0;

uint8_t wifitime 		= 0;
uint8_t infotime 		= 0;

/* LCD */
uint8_t lcd_enable      = 1;
uint8_t timer_A2_enable = 1;
uint8_t wifi_enable     = 0;
char wifibuf[10][18];
uint8_t wificnt = 0;
uint8_t wifientries = 0;


int stop_i2c_b1 = 0;

int push = 0;
int bcnt = 0;
int timcnt = 0;
int buf2full = 0;

uint8_t soundbuf[12288];
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
char ENDLINE1 = 0x93;
char ENDLINE2 = 0xD3;
char ENDLINE3 = 0xA7;
char ENDLINE4 = 0xE7;

uint8_t goforward = 0;

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
	  //UCA1IE |= UCRXIE + UCTXIE;                         // Enable USCI_A0 RX interrupt
	  UCA1IE |= UCRXIE;
  }

  /* DAC12_A Start */
  if(dac12_enable == 1){
	  P6SEL |= BIT6+BIT7;
	  DAC12_0CTL0 = DAC12SREF_0+DAC12AMP_7+DAC12OG+DAC12ENC+DAC12CALON;
  }

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


  /* Timer B0 Start */
  if(timer_B0_enable == 1){
	  P3DIR |= BIT0+BIT1;                            // P1.0 output
	  TB0CCTL0 = CCIE;                          // CCR0 interrupt enabled
	  TB0CCR0 = 100;
	  TB0CTL = TBSSEL_2 + MC_1 + TBCLR;         // SMCLK, upmode, clear TAR
  }

  /* Timer A2 Start */
  if(timer_A2_enable == 1){
	  TA2CCTL0 = CCIE;                          // CCR0 interrupt enabled
	  TA2CCR0 = 45000;
	  TA2CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR
  }

  /* Timer A1 Start
  if(timer_A1_enable == 1){
	  TA1CCTL0 = CCIE;                          // CCR0 interrupt enabled
	  TA1CCR0 = 10000;
	  TA1CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR
  } */

  /* ADC 0 Start */
  if(adc12_0_enable == 1)
  {
	  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
	  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
	  ADC12IE = 0x01;                           // Enable interrupt
	  ADC12CTL0 |= ADC12ENC;
	  P6SEL |= 0x01;                            // P6.0 ADC option select
	  P1DIR |= 0x01;                            // P1.0 output
  }

  /* LCD Init Start */
  if(lcd_enable == 1){

	  __bis_SR_register(GIE);

	  /* LCD Output pin enable */
	  P3DIR |= BIT6 + BIT7;
	  P9DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

	  P3OUT &= ~BIT6;
	  P3OUT |= BIT7;

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

	  /* */
	  chgline(LINE1);
	  pmsglcd("*. Democratic DJ .*");
	  chgline(LINE2);
	  pmsglcd("Artist: TSwift");
	  chgline(LINE4);
	  char symb[21] = {0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,0xF3,'\0'};
	  pmsglcd(symb);
	  chgline(LINE3);
	  pmsglcd("Song: ");
	  message = "Shake it Off - Team 5 ";


	  /*
	  chgline(LINE1);
	  CURLINE = LINE1;
	  TA2CCTL0 &= ~CCIE;
	  */

	  /* Lights */
	  P1DIR |= 0x02;
	  P3OUT &= ~BIT1;
	  P1OUT |= 0x02;

	  /* RED LED */
	  //P4DIR |= BIT4;

	  P2SEL &= ~BIT0;
	  P2DIR &= ~BIT0;
	  P2IES &= ~BIT0;
	  P2IFG &= ~BIT0;
	  P2IE  &= ~BIT0;

	  if(wifi_enable == 0){
		  pmsglcd_scroll(message, 1);
	  }
  }

  /* Lights */
  P1DIR |= 0x02;
  P3OUT &= ~BIT1;
  P1OUT |= 0x02;

  /* RED LED */
  //P4DIR |= BIT0;

  /* P2 Interrupt */
  P2SEL &= ~BIT0;
  P2DIR &= ~BIT0;
  P2IES &= ~BIT0;
  P2IFG &= ~BIT0;
  P2IE  |= BIT0;



  __bis_SR_register(GIE);

  if(i2c_b1_enable == 1){
	  begin_I2C();
  }



  /*
  switch_bars(0x71);

  begin_I2C();

  switch_bars(0x70);

  */

  //UCA1TXBUF = 3;
  CURLINE = LINE1;
  uint8_t delivered = 0;
  /* DEDICATED LOADING AND PRINTING REQUIRED */
  while (1)
  {
	  TA2CCTL0 &= ~CCIE;
	  if(wifi_enable == 1){
		  if((P4IN & BIT5) && capture < 4){
			  print_Wifi();
		  }
	  }
	  if( (P4IN & BIT4) && delivered == 0){
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

    //__bis_SR_register(GIE);     // LPM0, ADC12_ISR will force exit
    //__bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_ISR will force exit
    //__no_operation();                       // For debugger
  }
}


void print_Wifi(){
			  TA2CCTL0 = CCIE;
		  	  pmsglcd(wifibuf[capture++]);
				if(CURLINE == LINE1){
					chgline(ENDLINE1);
					pmsglcd(marker);
					chgline(LINE2);
					CURLINE = LINE2;
				}
				else if(CURLINE == LINE2){
					chgline(LINE3);
					CURLINE = LINE3;
				}
				else if(CURLINE == LINE3){
					chgline(LINE4);
					CURLINE = LINE4;
				}
				else if(CURLINE == LINE4){
					chgline(LINE1);
					CURLINE = LINE1;
				}
	return;
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

 	while(goforward != 1){}
	goforward = 0;
}

void send_byte(char a)
{
     //Shift out character
     P9OUT = 0;
     P9OUT = a;
     //Pulse LCD clock line low->high
     P3OUT &= ~BIT7;
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
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not
supported!
#endif
{
	if(musictime == 1){
					while (!(UCRXIFG));             // USCI_A0 RX buffer ready?

					  soundbuf[cur_write_buf + byte_write_cnt++] = UCA1RXBUF;

					  if(byte_write_cnt == 4096){
						  byte_write_cnt = 0;
						  cur_write_buf = cur_write_buf < 8000 ? cur_write_buf + 4096 : 0;
						  firstfour++;
						  UCA1IFG = 0;
						  P1OUT = 0x00;
					  }


					  //UCA1IFG = 0;
	}

	/* else if(wifitime == 1){while (!(UCRXIFG));UCA1IFG = 0;}*/


	else if(wifitime == 1){

		    		while (!(UCRXIFG));
		    		int i = 0;
		    		int b = i;
		    		b = 2;

		    		if(wificnt == 0){
		    			wifibuf[wifientries][0] = (wifientries + 1) + '0';
		    			wifibuf[wifientries][1] = '.';
		    			wificnt = 2;
		    		}
		    		wifibuf[wifientries][wificnt] = UCA1RXBUF;
		    		wificnt++;

		    		if(wificnt == 17){
		    			wificnt = 0;
		    			wifientries++;
		    		}

		    		UCA1IFG = 0;
	}
	else if(infotime == 1){
		while (!(UCRXIFG));
		if(UCA1RXBUF != '\n' && inputbufcnt != 40){
			inputbuffer[change + inputbufcnt++] = UCA1RXBUF;
		}
		else{
			inputbuffer[change + inputbufcnt] = '\0';
			inputbufcnt = 0;
			change += 40;

			if(change >= 160) change = 0;


			TA2CCTL0 |= CCIE;
			__bis_SR_register(GIE);
			if(CURLINE = LINE1){
				chgline(LINE2);
				CURLINE = LINE2;
			}
			if(CURLINE = LINE2){
				chgline(LINE3);
				CURLINE = LINE3;
			}
			if(CURLINE = LINE3){
				chgline(LINE4);
				CURLINE = LINE4;
			}
			if(CURLINE = LINE4){
				chgline(LINE1);
				CURLINE = LINE1;
			}

		}
		UCA1IFG = 0;
	}


	/*
	else{
	  switch(__even_in_range(UCA1IV,4))
	  {
	    case 0:break;                             // Vector 0 - no interrupt
	    case 2:                                   // Vector 2 - RXIFG

	    	if(wifitime == 1){
	    		while (!(UCRXIFG));
	    		if(wificnt == 0){
	    			wifibuf[wifientries][0] = (wifientries + 1) + '0';
	    			wifibuf[wifientries][1] = '.';
	    			wificnt = 2;
	    		}
	    		wifibuf[wifientries][wificnt] = UCA1RXBUF;
	    		wificnt++;

	    		if(wificnt == 17){
	    			wificnt = 0;
	    			wifientries++;
	    		}
	    		UCA1IFG = 0;
	    	}

	      break;
	    case 4:
	    	UCA1TXBUF = 0xF;
	    	P4OUT &= ~BIT0;
	    	break;                             // Vector 4 - TXIFG
	    default: break;

	  }
	}
	*/
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
		ADC12IE = 0x01;
		TA0CCTL0 = CCIE;
		TA1CCTL0 = CCIE;

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

	ADC12CTL0 |= ADC12SC;
	//TA1CCTL0 &= ~CCIFG;

  	/* Last 7 Bars, Red */
	/*
	if (average >= 255){
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0xF0, 0x0F);
		i2c_bar_1_set(4, 0xFF, 0x0F);
	}
	else if (average >= 253) {
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0xF0, 0x0F);
		i2c_bar_1_set(4, 0x7F, 0x0F);
  	}
	else if (average >= 242) {
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0xF0, 0x0F);
		i2c_bar_1_set(4, 0x3F, 0x0F);
  	}
	else if (average >= 231) {
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0xF0, 0x0F);
		i2c_bar_1_set(4, 0x1F, 0x0F);
	}
	else if (average >= 220) {
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0xF0, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 209) {
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0x70, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 198) {
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0x30, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 187) {
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0x10, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 176) {
		i2c_bar_1_set(0, 0xF0, 0xFF);
		i2c_bar_1_set(2, 0x00, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 165) {
		i2c_bar_1_set(0, 0x70, 0x7F);
		i2c_bar_1_set(2, 0x00, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 154) {
		i2c_bar_1_set(0, 0x30, 0x3F);
		i2c_bar_1_set(2, 0x00, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 143) {
		i2c_bar_1_set(0, 0x10, 0x1F);
		i2c_bar_1_set(2, 0x00, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 132) {
		i2c_bar_1_set(0, 0x00, 0x0F);
		i2c_bar_1_set(2, 0x00, 0x0F);
		i2c_bar_1_set(4, 0x0F, 0x0F);
	}
	else if (average >= 121) {
		i2c_bar_1_set(0, 0x00, 0x0F);
		i2c_bar_1_set(2, 0x00, 0x0F);
		i2c_bar_1_set(4, 0x07, 0x07);
	}
	else if (average >= 110) {
		i2c_bar_1_set(0, 0x00, 0x0F);
		i2c_bar_1_set(2, 0x00, 0x0F);
		i2c_bar_1_set(4, 0x03, 0x03);
	}
	else if (average >= 99) {
		i2c_bar_1_set(0, 0x00, 0x0F);
		i2c_bar_1_set(2, 0x00, 0x0F);
		i2c_bar_1_set(4, 0x01, 0x01);
	}
	else if (average >= 88) {
		i2c_bar_1_set(0, 0x0, 0x0F);
		i2c_bar_1_set(2, 0x0, 0x0F);
		i2c_bar_1_set(4, 0x00, 0);
	}
	else if (average >= 77) {
		i2c_bar_1_set(0, 0x0, 0x0F);
		i2c_bar_1_set(2, 0x0, 0x07);
		i2c_bar_1_set(4, 0x00, 0);
	}
	else if (average >= 66) {
		i2c_bar_1_set(0, 0x0, 0x0F);
		i2c_bar_1_set(2, 0x0, 0x03);
		i2c_bar_1_set(4, 0x00, 0);
	}
	else if (average >= 55) {
		i2c_bar_1_set(0, 0x0, 0x0F);
		i2c_bar_1_set(2, 0x0, 0x01);
		i2c_bar_1_set(4, 0x00, 0);
	}
	else if (average >= 44) {
		i2c_bar_1_set(0, 0x0, 0x0F);
		i2c_bar_1_set(2, 0x00, 0);
		i2c_bar_1_set(4, 0x00, 0);
	}
	else if (average >= 33) {
		i2c_bar_1_set(0, 0x0, 0x07);
		i2c_bar_1_set(2, 0x00, 0);
		i2c_bar_1_set(4, 0x00, 0);
	}
	else if (average >= 22) {
		i2c_bar_1_set(0, 0x0, 0x03);
		i2c_bar_1_set(2, 0x00, 0);
		i2c_bar_1_set(4, 0x00, 0);
	}
	else if (average >= 11) {
		i2c_bar_1_set(0, 0x0, 0x01);
		i2c_bar_1_set(2, 0x00, 0);
		i2c_bar_1_set(4, 0x00, 0);
	}
	else{
		i2c_bar_1_set(0, 0x00, 0);
		i2c_bar_1_set(2, 0x00, 0);
		i2c_bar_1_set(4, 0x00, 0);
	}
	*/

	__bic_SR_register_on_exit(LPM0_bits);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B0_VECTOR))) TIMER0_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{

	//ADC12CTL0 |= ADC12SC;
	//TA1CCTL0 &= ~CCIFG;
	//P1OUT ^= 0x02;
	__bis_SR_register(GIE);

	if(firstfour >= 3){
		  //sampler[avgcnt] = soundbuf[cur_read_buf + byte_read_cnt++];
		  DAC12_0DAT = soundbuf[cur_read_buf + byte_read_cnt++];
		  //sampler[avgcnt] = soundbuf[cur_read_buf][byte_read_cnt];

		  if(byte_read_cnt == 4096){
			  byte_read_cnt = 0;
			  cur_read_buf = cur_read_buf < 8000 ? cur_read_buf + 4096 : 0;
			  P1OUT = 0x02;
		  }

		  firstfour = 3;

		  /*
		  avgcnt++;
		  if(avgcnt == 128){
			  int cnt = 0;
			  for(cnt = 0; cnt < 16; cnt++){
				  average += sampler[cnt];
			  }
			  average = average >> 5;
			  avgcnt = 0;
		  }*/

	}

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

/*
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B0_VECTOR))) TIMER0_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	/* Debouncing
	if(preplayval == 0 && playval == 1){
		P4OUT ^= BIT0;
		TB0CCTL0 ^= CCIE;
	}

	/*
	if(preplayval == 1 && playval == 0){
		P4OUT |= BIT0;
		TB0CCTL0 = CCIE;
	}


	preplayval = playval;
}
*/


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

	  /*
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
	  		else if (ADC12MEM0 >= 0xBF0) {
	  			i2c_bar_1_set(0, 0xF0, 0xFF);
	  			i2c_bar_1_set(2, 0x00, 0x0F);
	  			i2c_bar_1_set(4, 0x0F, 0x0F);
	  		}
	  		else if (ADC12MEM0 >= 0xBDB) {
	  			i2c_bar_1_set(0, 0x70, 0x7F);
	  			i2c_bar_1_set(2, 0x00, 0x0F);
	  			i2c_bar_1_set(4, 0x0F, 0x0F);
	  		}
	  		else if (ADC12MEM0 >= 0xBC6) {
	  			i2c_bar_1_set(0, 0x30, 0x3F);
	  			i2c_bar_1_set(2, 0x00, 0x0F);
	  			i2c_bar_1_set(4, 0x0F, 0x0F);
	  		}
	  		else if (ADC12MEM0 >= 0xBB1) {
	  			i2c_bar_1_set(0, 0x10, 0x1F);
	  			i2c_bar_1_set(2, 0x00, 0x0F);
	  			i2c_bar_1_set(4, 0x0F, 0x0F);
	  		}
	  		else if (ADC12MEM0 >= 0xB9C) {
	  			i2c_bar_1_set(0, 0x00, 0x0F);
	  			i2c_bar_1_set(2, 0x00, 0x0F);
	  			i2c_bar_1_set(4, 0x0F, 0x0F);
	  		}
	  		else if (ADC12MEM0 >= 0xB72) {
	  			i2c_bar_1_set(0, 0x00, 0x0F);
	  			i2c_bar_1_set(2, 0x00, 0x0F);
	  			i2c_bar_1_set(4, 0x07, 0x07);
	  		}
	  		else if (ADC12MEM0 >= 0xB5D) {
	  			i2c_bar_1_set(0, 0x00, 0x0F);
	  			i2c_bar_1_set(2, 0x00, 0x0F);
	  			i2c_bar_1_set(4, 0x03, 0x03);
	  		}
	  		else if (ADC12MEM0 >= 0xB48) {
	  			i2c_bar_1_set(0, 0x00, 0x0F);
	  			i2c_bar_1_set(2, 0x00, 0x0F);
	  			i2c_bar_1_set(4, 0x01, 0x01);
	  		}
	  		else if (ADC12MEM0 >= 0xB33) {
	  			i2c_bar_1_set(0, 0x0, 0x0F);
	  			i2c_bar_1_set(2, 0x0, 0x0F);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}
	  		else if (ADC12MEM0 >= 0xB1E) {
	  			i2c_bar_1_set(0, 0x0, 0x0F);
	  			i2c_bar_1_set(2, 0x0, 0x07);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}
	  		else if (ADC12MEM0 >= 0xB09) {
	  			i2c_bar_1_set(0, 0x0, 0x0F);
	  			i2c_bar_1_set(2, 0x0, 0x03);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}
	  		else if (ADC12MEM0 >= 0xAF4) {
	  			i2c_bar_1_set(0, 0x0, 0x0F);
	  			i2c_bar_1_set(2, 0x0, 0x01);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}
	  		else if (ADC12MEM0 >= 0xADF) {
	  			i2c_bar_1_set(0, 0x0, 0x0F);
	  			i2c_bar_1_set(2, 0x00, 0);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}
	  		else if (ADC12MEM0 >= 0xACA) {
	  			i2c_bar_1_set(0, 0x0, 0x07);
	  			i2c_bar_1_set(2, 0x00, 0);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}
	  		else if (ADC12MEM0 >= 0xAB5) {
	  			i2c_bar_1_set(0, 0x0, 0x03);
	  			i2c_bar_1_set(2, 0x00, 0);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}
	  		else if (ADC12MEM0 >= 0xAA0) {
	  			i2c_bar_1_set(0, 0x0, 0x01);
	  			i2c_bar_1_set(2, 0x00, 0);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}
	  		else{
	  			i2c_bar_1_set(0, 0x00, 0);
	  			i2c_bar_1_set(2, 0x00, 0);
	  			i2c_bar_1_set(4, 0x00, 0);
	  		}*/

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
	  	/* Last 7 Bars, Red
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
		 Second 8 Bars, Yellow
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
		 First 8 Bars, Green
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
		} */


    //__bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
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
		TB0CCTL0 ^= CCIE;
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

