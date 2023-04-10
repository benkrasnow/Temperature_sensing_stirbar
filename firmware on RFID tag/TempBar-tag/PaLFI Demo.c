//******************************************************************************
//   MSP430F22x4 + TMS37157 Demo V1.0
//
//   Description: This programm demonstrates the Communication between the MSP430F2274
//   and the TMS37157 PaLFI (a Texas Instruments RFID Product). It covers all Demo Function 
//	 that are included in the eZ430-PaLFI Demo Tool. The most of the time, the programm will wait
//	 in LPM3. 
//
//	 On a Push Button Event, it tests if the PaLFI is trimmed, if not it it trims it, 
//   after this, it will read Page 9 of the EEPROM of the PaLFI and blink the green LED as often
//   as saved in Page 9. The Memory Content of Page 9 will be copied to Page 11.
//
//	 Upon recieve of an MSP Access Command (Busy Interrupt) the MSP will fetch the MSP Data from
//	 from the PaLFI and blink the red or green LED as many times as it was in the MSP Access Command.
//
//	 Version Updates
//	 	
//	 1.0 : initial Version
//	 
//   Stefan Recknagel
//   Texas Instruments Inc.
//   February 2009
//   Built with CCE v3.1   Build: 3.2.3.6.4 
//******************************************************************************
#include  "msp430g2452.h"
#include "stdlib.h"
#include "string.h"
#include "intrinsics.h"
#include "PaLFI_Transponder.h"
//#include "DCO_Library.h"

char buf[5] 		= {0};
unsigned int i,t,j,first_run;
unsigned short test;
unsigned char  ucWDT_Count,status;
int result;


void main(void)
{
	
	unsigned char ucPageData[5] = {0};		// one Page is always 5 Byte
	unsigned char ucMSPAccessData[6] = {0};	// MSP Access Data is always 6 Byte
	
	WDTCTL = WDTPW + WDTHOLD;               // Stop WDT
	
	BCSCTL3 |= (LFXT1S1);				// Set ACLK to internal 12KHz VLO

	Delay_500ms();

		

	P1OUT = 0;                              // All P1.x reset
	P2OUT = 0;                              // All P2.x reset
	P1DIR = 0;
	P2DIR = 0;                         		// Configure P2.x

	//P3SEL |= 0x30;                          // P3.4,5 = USCI_A0 TXD/RXD
	//P3DIR = 0xFB;                           // Configure P3.x
	//P3OUT = 0;                              // All P3.x reset
	//P4DIR = 0xFF;                           // All P4.x outputs
	//P4OUT = 0;                              // All P4.x reset
	
	//P1DIR &= ~PUSH_BUTTON;					// Push Button to Input
	//P1REN |= PUSH_BUTTON;					// Activate Pull up/down at P1.2 Push Button
  	//P1OUT |= PUSH_BUTTON;					// Set Pull up at P1.2
  	P1DIR |= 0x03;                          // Set P1.0 and P1.1 to output direction, LED_RED,LED_GREEN

  	//P1IES &= ~(CU_BUSY);					//Interrupt on low-to-high transitions  should be cleared by default
  	
  	//P1IE &= ~PUSH_BUTTON;					// disable Push Button interrupt
  	//P1IFG &= ~PUSH_BUTTON;					// reset Push Button interupt flag
	//P1IES |= PUSH_BUTTON;					// set interrupt to high to low transition



	MSP430_SPI_Init();						// initialize SPI for Communication with PaLFI

	first_run = 0;	




	while (1)
	{ 
	  
	/*  if ((P1IFG & PUSH_BUTTON) == PUSH_BUTTON)	// Push Button Interrupt
	  {
	  //	*****************  Perform PaLFI Read and if necessary trim PaLFI    ********************
	  		  	
		Wake_PaLFI();						// Wake up PaLFI
		
		UCB0CTL1 &= ~UCSWRST;				// activate SPI State machine  
		
		SPI_Read_PCU_State();					// read PCU State
		
		SPI_Read_SerialNum();                   // First PaLFI access defines selective address! (not implemented in PaLFI)
		
		SPI_Read_UserPage(Page2,ucPageData);	// if PaLFI trimmed Page2 = 0x01

		if (ucPageData[0] == 0x00) 
		{
			SPI_Power_Down();
			while ((P1IN & PUSH_BUTTON) == 0x00);	// wait until Push Button is released
			autotrim();							// trim PaLFI, performs SPI Power Down!
			Delay_500ms();						// delay 500 ms
			Wake_PaLFI();						// Wake up PaLFI
		}
					  	
	  //	*****************  PaLFI is trimmed    ********************
	  	
		SPI_Read_UserPage(Page9,ucPageData);	// read Page9, how often should green LED blink
		SPI_Program_UserPage(Page11,ucPageData);// programm Data in Page 11
		SPI_Power_Down();						// Power down PaLFI
		
		UCB0CTL1 |= UCSWRST;					// deactivate SPI State machine
		
		while ((P1IN & PUSH_BUTTON) == 0x00);	// wait until Push Button is released

		for (i=0;i<ucPageData[0];i++)
		{	
			Toggle_LED_500ms (LED_GREEN);
		}
	  }

	  */
	  
	 // else if (((P2IFG & CU_BUSY) == CU_BUSY)&&(first_run == 1))	// Busy Interrupt from MSP Access Mode
	if (((P1IFG & CU_BUSY) == CU_BUSY)&&(first_run == 1))	// Busy Interrupt from MSP Access Mode
	  {

		USICTL0 &= ~USISWRST;							// activate SPI State machine
	  	while ((P1IN & CU_BUSY) == CU_BUSY);			// wait until busy goes low (PaLFI is ready)

	  	SPI_Read_CU_Data(ucMSPAccessData);				// request Data from PaLFI for MSP Action


	  	if (ucMSPAccessData[1] == 0x0)				// if Byte 1 of MSP Access = 0x1 blink red LED
	  		  	{
	  			ADC10CTL0=SREF_1 + REFON + ADC10ON + ADC10SHT_3 ; //1.5V ref,Ref on,64 clocks for sample
	  		    ADC10CTL1=INCH_10 + ADC10DIV_3; //temp sensor is at 10 and clock/4
	  		    __delay_cycles(1000);              //wait 4 ref to settle
	  		    ADC10CTL0 |= ENC + ADC10SC;      //enable conversion and start conversion
	  		    while(ADC10CTL1 & BUSY);         //wait..i am converting..pum..pum..
	  		    t=ADC10MEM;                       //store val in t
	  		    ADC10CTL0&=~ENC;                     //disable adc conv
	  		    ucMSPAccessData[3] =   ((t * 27069L - 18169625L) >> 16); //convert and pass

	  		  	}

	  	status = SPI_Write_CU_Data(ucMSPAccessData);	// send MSP Access Data back, no SPI Power Down, no change to MSP Access Data (in this case)!

	  	USICTL0 |= USISWRST;                     		// **deactivate SPI state machine**
 		  		
	  	if (ucMSPAccessData[1] == 0x1)				// if Byte 1 of MSP Access = 0x1 blink red LED
	  	{
			//Delay_500ms (); 						// wait additional 500 ms
		  	  	
	  	  	for (i=0;i<ucMSPAccessData[0];i++)
			{		
				//Toggle_LED_500ms (LED_GREEN);			// Toggle Red LED in a 500ms Interval
			}
	  	}
		  	
	  	else if (ucMSPAccessData[1] == 0x0)			// if Byte 1 of MSP Access = 0x0 blink green LED
	  	{
			//Delay_500ms ();							// wait additional 500 ms
	  		
	  		for (i=0;i<ucMSPAccessData[0];i++)
			{
				//Toggle_LED_500ms (LED_GREEN);		// Toggle Green LED in a 500ms Interval
			}
	  	}	  	 	 	
	  }
  	  
	 // P1IFG &= ~BIT2;								// disable interupt flag
	  P1IFG &= ~CU_BUSY;							// disable busy interupt flag
	  //P1IE |= BIT2;								// enable Push Button Interrupt
	  P1IE |= CU_BUSY;							// enable Busy Interupt
 
	  first_run = 1;

	  __bis_SR_register(LPM3_bits + GIE);   	// Enter LPM3, enable interrupts
	}    
}

/********************************additional functions**********************************************/



 /*****************************************************************************************
  * Trims the TMS37157 to 134,2 kHz +- 200Hz. CLKAM is connected to P2.1 as INCLK for TIMER A. 
  * Timer A counts up to 800 clock cycles of CLK_AM. The DCO is calibrated to 8MHz.
  * And is used as clock source for TIMER B (SMCLK). TIMER B counts until TIMER A reaches 800.
  * The Value saved in TBR is the time needed for 800 clocks of CLK_AM. As long as TBR<47500 
  * the MSP430 increases the trim capcitors in the TMS37157, until the measured time >= 47500.
  * The corresponding trim capacitor value is then programmed into the Trim EEPROM of the TMS37157. 
  * 
  * *****************************************************************************************/

/*
int autotrim(void)								// calibrates DCO and trims PaLFI Chip to 134,2 +- 200 Hz
{

   
	unsigned int old_time = 0;
	unsigned int time = 0;
	unsigned int timex[128] = {0};				// used for debugging, can be deleted to save memory
	int prog_data = 0;							// 
	unsigned char ucPageData[5] = {1,0,0,0,0};	// if autotrim succeeded this value gets programmed to Page 2
	
	LED_ON(LED_GREEN);					// Switch on green LED
	LED_ON(LED_RED);					// Switch on red LED

	result = TI_SetDCO(TI_DCO_8MHZ);			// Start Calibration to 8MHz
	
	if( result == TI_DCO_SET_TO_SLOWEST )       // returned result if DCO registers hit min
  	{
    	while(1);                               // set breakpoint here to see if hit
  	}
  	else if( result == TI_DCO_SET_TO_FASTEST )  // returned result if DCO registers hit max
  	{
    	while(1);                               // set breakpoint here to see if hit
  	}
  	else if( result == TI_DCO_TIMEOUT_ERROR )   // returned result if DCO takes >10000 loops
  	{
    	while(1);                               // set breakpoint here to see if hit
  	}
  	
  	Wake_PaLFI();
	SPI_TRIM_W_PROG(prog_data);					// set trim EEPROM to 0
	

	while ((time<47500)||(time>47700)) 			// time for 800 Clocks of CLKAM at 8 MHz
	{
		if (prog_data == 0x80) 					// maximum capacitor value reached
		{
			prog_data = 0xFF;					// autotrim failed
			break;   					
		}
				
		TACTL |= TACLR;						// clear TIMERA Control Register
	  	TBCTL |= TBCLR;						// clear TIMERB Control Register
		TACCTL0 &= ~CCIFG;					// reset TIMERA CCIFG flag
		TBCCTL0 &= ~CCIFG;					// reset TIMERB CCIFG flag
		TACTL &= ~TAIFG;					// reset TIMERA IFG flag
		TBCTL &= ~TBIFG;					// reset TIMERB IFG flag
		
		TBCTL = TBSSEL_2 + MC_0;			// halt Timer B
		
		TACCR0 = 0;							// halt Timer A				
		
		if (prog_data != 0)SPI_TRIM_WO_PROG(prog_data);	// trim PaLFI without programming the EEPROM
		
	  	SPI_CLKA_ON();						// switch on LC Tank Circuit and switch output to CLKAM
	  	  	  	
	  	TACTL = TASSEL_3 + MC_1;		    // Timer A select, clock source INCLK (CLK_AM), count up to TACCR0
	  	P4OUT |= 0x10;						// Set P4.4 to show beginning of meassurement
	  	
	  	while ((P2IN & CLK_AM) == 0x00); 	// wait for low to high transition of CLK_AM
	  	while ((P2IN & CLK_AM) == CLK_AM); 	// wait for high to low transition of CLK_AM
	  	TACCR0 = 800;						// clock 800 cycles of CLK-AM, load Capture Compare of Timer A
	  	while ((P2IN &CLK_AM) == 0x00);		// start Timer B with first low to high transition of CLK_AM
	  	  	
	  	TBCTL = TBSSEL_2 + MC_2;			// Timer B select, clock source SMCLK, continous mode, start Timer B
	  	while ((TACCTL0 & CCIFG) == 0x00);	// wait until Capture Compare Interrupt Flag is set (Timer A)
	  	TBCTL = TBSSEL_2 + MC_0;			// halt Timer B
	  	P4OUT &= ~0x10;						// reset P4.4 to show end of meassurement
	  	
	  	SPI_CLKA_OFF();						// switch of CLK_AM
	  	TACCR0 = 0;							// halt TIMER A
	  	TACCTL0 &= ~CCIFG;					// reset CCIFG flag
	  	
	 	if (prog_data != 0) old_time = time;
	 	
	 	time = TBR;							// read Timer B out
	 	timex[prog_data] = time;			// save Timer Value - for debugging
	  	
	  	prog_data++	;						// increment trim data for next trimming command
	}
  	
  	
  	if (prog_data == 0xFF)
  	{
		SPI_Power_Down();
		while(1)								// trap CPU if trimming failed
		{
  			LED_ON(LED_RED);					// Switch on red LED
			
			TACCTL0 &= ~CCIFG;					// reset CCIFG flag
			TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
			TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
			TACCR0 = 8192;						// clock 0.25 second, load Capture Compare of Timer A
			__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
	  		
	  		LED_OFF(LED_RED);					// Switch off red LED
			
			TACCTL0 &= ~CCIFG;					// reset CCIFG flag
			TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
			TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
			TACCR0 = 8192;						// clock 0.25 second, load Capture Compare of Timer A
			__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
		}
  	}
  	
  	else
  	{  	
  		if (47500-old_time <= time-47500)		// calculate difference to perfect value and choose the closer one 
  		{
	 		SPI_TRIM_W_PROG(prog_data - 2);		// prog last values
  		}
	 	else
	 	{
	 		SPI_TRIM_W_PROG(prog_data - 1);		// prog last values
	 	}
	 			
  		SPI_Program_UserPage (Page2,ucPageData);	// Set Bit for trimmed Device
  		SPI_Power_Down();							// switch of PaLFI
		
		LED_OFF(LED_RED);							// Switch off red LED		
		
		Delay_500ms();							
		Delay_500ms();							// delay 1 second
		
		LED_OFF(LED_GREEN);						// Switch off green LED
	}
	result = TI_SetDCO(TI_DCO_2MHZ);			// Start Calibration to 2MHz
	
	if( result == TI_DCO_SET_TO_SLOWEST )       // returned result if DCO registers hit min
  	{
    	while(1);                               // set breakpoint here to see if hit
  	}
  	else if( result == TI_DCO_SET_TO_FASTEST )  // returned result if DCO registers hit max
  	{
    	while(1);                               // set breakpoint here to see if hit
  	}
  	else if( result == TI_DCO_TIMEOUT_ERROR )   // returned result if DCO takes >10000 loops
  	{
    	while(1);                               // set breakpoint here to see if hit
  	}
  	return time;
}

*/



void LED_ON (int LED)
{
	P1OUT |= LED;			// switch on LED
}

void LED_OFF (int LED)
{
	P1OUT &= ~LED;		// Switch off LED
}

void Toggle_LED_500ms (int LED)
{
	Delay_500ms ();
	LED_ON(LED);						// Toggle green LED
	Delay_500ms ();
	LED_OFF(LED);						// Toggle green LED
}

void Delay_500ms(void)
{
	TACCTL0 &= ~CCIFG;					// reset CCIFG Interrupt Flag
	TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
	TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
	TACCR0 = 5500;						// clock 0.5 second, load Capture Compare of Timer A
	__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe	
}


/***************************************Interrupt Vectors***************************/

// Port 1 Interrupt Service Routine (Push Button or Busy Interrupt)

#pragma vector=PORT1_VECTOR							// Push Button Interrupt
__interrupt void PORT1_ISR(void)
{
	P1IE &= ~CU_BUSY;								// disable Busy Interupt
	//P1IE&= ~PUSH_BUTTON;							// disable Push Button Interrupt
	P1OUT &= ~(0x01);
	P1OUT |= 0x01;
	P1OUT &= ~(0x01);
	__bic_SR_register_on_exit(LPM3_bits + GIE); 	// Clear LPM3 bits and GIE bits from 0(SR)
}

// Port 2 Interrupt Service Routine (Not used)

//#pragma vector=PORT2_VECTOR							// Not used
//__interrupt void PORT2_ISR(void)
//{
	//P1IE&= ~PUSH_BUTTON;							// disable Push Button Interrupt
	//P1IE &= ~CU_BUSY;								// disable Busy Interupt
	//__bic_SR_register_on_exit(LPM3_bits + GIE);     // Clear LPM3 bits and GIE bits from 0(SR)
//}


// Timer A0 interrupt service routine

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
	__bic_SR_register_on_exit(LPM3_bits + GIE);     // Clear LPM3 bits and GIE bits from 0(SR)
}



// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

