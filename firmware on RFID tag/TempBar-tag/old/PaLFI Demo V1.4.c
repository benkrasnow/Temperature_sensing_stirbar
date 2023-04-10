//******************************************************************************
//   MSP430F22x4 + TMS37157 Demo V1.4
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
//	 Upon recieve of a "01" Command from the Serial Interface (9600 - 8N1) it will read Page 10
//	 of the EEPROM of the PaLFI and blink the red LED as often as saved in Page 10. The Memory 
// 	 Content of Page 10 will be copied to Page 12.
//
//	 Page 13 Byte 0 defines the selective address of the TMS37157, which will be checked for the MSP Access Command 	
//
//	 Upon recieve of an MSP Access Command (Busy Interrupt) the MSP will fetch the MSP Data from
//	 from the PaLFI and blink the red or green LED as many times as it was in the MSP Access Command.
//
//	 //   Serial Baud rate divider with 32768Hz XTAL @9600 = 32768Hz/9600 = 3.41
//   //* An external watch crystal is required on XIN XOUT for ACLK *//
//
//	 Version Updates
//	 
//	 1.1 : Added support for Batteryless MSP Access and batteryless temperature sensor
//	 1.2 : Added selective addressing (hard coded) for MSP Access
//	 1.3 : Added battery Meassurement by the MSP430 through MSP Access 			
//	 1.4 : Changed selective Adress to page 13 Byte 0 Adress can be programmed through LF now	
//
//   Stefan Recknagel
//   Texas Instruments Inc.
//   October 2008
//   Built with CCE Version: 3.2.2
//******************************************************************************
#include  "msp430x22x4.h"
#include "stdlib.h"
#include "string.h"
#include "intrinsics.h"
#include "PaLFI_Transponder.h"
#include "DCO_Library.h"

const char LED_blink_RED[] 		= { "Blink Red LED\r\n" };
const char LED_blink_GREEN[] 	= { "Blink Green LED\r\n" };
const char MSP_Access[] 		= { "MSP Access\r\n" };
const char PUSH_Button_Text[] 	= { "PUSH Button pressed\r\n" };
const char SERIAL_Recieved[] 	= { "Serial Recieve\r\n" }; 
const char wrong_serial_code[] 	= { "Type 01 to start\r\n" }; 
char recieve[5]	  	= {"00000"};
char start_string[5]= {"start"};
char buf[5] 		= {0};
unsigned int i,t,j,first_run;
unsigned short second[2],minute[2],hour[2];
unsigned short test;
unsigned char  ucWDT_Count,status;
int result;
void delay_us(int us);
long temp;
volatile long IntDegF = 0;
volatile long IntDegC = 0;

void main(void)
{
	
	unsigned char ucPageData[5] = {0};		// one Page is always 5 Byte
	unsigned char ucMSPAccessData[6] = {0};	// MSP Access Data is always 6 Byte
	
	WDTCTL = WDTPW + WDTHOLD;               // Stop WDT
	
	for (i=1000;i==0;i--)					// wait for PaLFI to Start
	{
		i++;
		i--;
	}
	
	BCSCTL3 |= (XCAP0 + XCAP1);				// Set ACLK Capacity to 12.5 pF
	P1DIR = 0xFF;                           // All P1.x outputs
	P1OUT = 0;                              // All P1.x reset
	P2OUT = 0;                              // All P2.x reset
	P2DIR = 0xFC;                         	// Configure P2.x
	P2OUT = 0;                              // All P2.x reset
	P3SEL |= 0x30;                          // P3.4,5 = USCI_A0 TXD/RXD
	P3DIR = 0xFB;                           // Configure P3.x
	P3OUT = 0;                              // All P3.x reset
	P4DIR = 0xFF;                           // All P4.x outputs
	P4OUT = 0;                              // All P4.x reset
	
	P1DIR &= ~PUSH_BUTTON;					// Push Button to Input
	P1REN |= PUSH_BUTTON;					// Activate Pull up/down at P1.2 Push Button
  	P1OUT |= PUSH_BUTTON;					// Set Pull up at P1.2
  	P1DIR |= 0x03;                          // Set P1.0 and P1.1 to output direction, LED_RED,LED_GREEN
  	
  	P1IE &= ~PUSH_BUTTON;					// disable Push Button interrupt
  	P1IFG &= ~PUSH_BUTTON;					// rest Push Button interupt flag
	P1IES |= PUSH_BUTTON;					// set interrupt to high to low transition
	
	UCA0CTL1 |= UCSSEL_1;                   // CLK = ACLK
	UCA0BR0 = 0x03;                         // Baud Rate Selection for serial Connection to Host PC
	UCA0BR1 = 0x00;                         // 32kHz/9600 = 3.41
	UCA0MCTL = UCBRS1 + UCBRS0;             // Modulation UCBRSx = 3
	UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**

	MSP430_SPI_Init();						// initialize SPI for Communication with PaLFI

	/***************** temperature meassurement for extended mode, enhanced RFID reader needed
	 
	for( i = 0; i < 0xFFFF; i++){}          // delay for ACLK startup
		
	ADC10CTL1 = INCH_10 + ADC10DIV_3;       // Temp Sensor ADC10CLK/4
  	ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE; 
  	
  	TACCTL0 &= ~CCIFG;						// reset CCIFG Interrupt Flag
	TACCTL0 = CCIE;                     	// TACCR0 interrupt enabled
	TACTL = TASSEL_1 + MC_1;				// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
	TACCR0 = 10;							// clock 300us second to settle reference, load Capture Compare of Timer A
  	__bis_SR_register(LPM3_bits + GIE); 	// Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
  	
  	ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    __bis_SR_register(CPUOFF + GIE);        // enter LPM0 with interrupts enabled
  	
  	// oF = ((A10/1024)*1500mV)-923mV)*1/1.97mV = A10*761/1024 - 468
    temp = ADC10MEM;
    IntDegF = ((temp - 630) * 761) / 1024;	// calculate temp in Fahrenheit

    // oC = ((A10/1024)*1500mV)-986mV)*1/3.55mV = A10*423/1024 - 278
    temp = ADC10MEM;
    IntDegC = ((temp - 673) * 423) / 1024;	// calculate temp in Degree
	
	ADC10CTL0 &= ~ENC;						// clear ENC bit
	ADC10CTL0 = 0;							// siwtch off ADC - only works if ENC cleared before
	
	********************************************************************************/
	
	
	first_run = 0;	

	while (1)
	{ 
	  
	  if ((P1IFG & PUSH_BUTTON) == PUSH_BUTTON)	// Push Button Interrupt	
	  {
	  		  	/*****************  Perform PaLFI Read and if necessary trim PaLFI    ********************/
	  		  	
		Wake_PaLFI();						// Wake up PaLFI
		
		SPI_Read_PCU_State();					// read PCU State
		
		SPI_Read_SerialNum();                   // First PaLFI access defines selective address! (not implemented in PaLFI)
		
		SPI_Read_UserPage(Page2,ucPageData);	// if PaLFI trimmed Page2 = 0x01

		if (ucPageData[0] == 0x00) 
		{
			SPI_Power_Down();
			while ((P1IN & PUSH_BUTTON) == 0x00);	// wait until Push Button is released
			autotrim();							// trim PaLFI if not happened yet, performs SPI Power Down!
			for (i=10000;i>1;i--)				// delay for next SPI Command 
			{									// wait until PaLFI is powered down
			} 
			Wake_PaLFI();						// Wake up PaLFI
		}
					  	
	  	/*****************  PaLFI is trimmed    ********************/	
	  	
		SPI_Read_UserPage(Page9,ucPageData);	// read Page9, how often should green LED blink
		SPI_Program_UserPage(Page11,ucPageData);// programm Data in Page 11
		SPI_Power_Down();						// Power down PaLFI
		
		while ((P1IN & PUSH_BUTTON) == 0x00);	// wait until Push Button is released
		j=0;
  		while (j< sizeof PUSH_Button_Text)		// Send out Text over Serial to Host PC
  		{
  			while (!(IFG2&UCA0TXIFG));      	// USCI_A0 TX buffer ready?
  			UCA0TXBUF = PUSH_Button_Text[j];
  			j++;
  		}

		for (i=0;i<ucPageData[0];i++)
		{
			j=0;
			TACCTL0 &= ~CCIFG;					// reset CCIFG Interrupt Flag
			TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
			TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
			TACCR0 = 16384;						// clock 0.5 second, load Capture Compare of Timer A
			__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
			LED_ON(LED_GREEN);					// Toggle green LED
			
			while (j< sizeof LED_blink_GREEN)	// Send out Text over Serial to Host PC
	  		{
	  			while (!(IFG2&UCA0TXIFG));      // USCI_A0 TX buffer ready?
	  			UCA0TXBUF = LED_blink_GREEN[j];
	  			j++;
	  		}
	  		
			TACCTL0 &= ~CCIFG;					// reset CCIFG Interrupt Flag
			TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
			TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source INCLK, count up to TACCR0, activate Interrupt Request
			TACCR0 = 16384;						// clock 0.5 second, load Capture Compare of Timer A
			__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
			LED_OFF(LED_GREEN);					// Toggle green LED
		}
	  }
	  else if ((IFG2&UCA0RXIFG)== UCA0RXIFG)	// Serial Interrupt
	  {
	  	j=0;
	  	
	  	while (j<2)								// length for blink Command = 2 (01)
	  	{
	  		recieve[j] = UCA0RXBUF;				// recieve character
	  		if (j == 1) break;
	  		while((IFG2&UCA0RXIFG)== 0x00);		// wait for next character to recieve
	  		j++;
	  	}
	  	
	  	if ((recieve[0] == 0x30) && (recieve[1] == 0x31))	// start signal = 01;
	  	{
			Wake_PaLFI();						// Wake up PaLFI
		  	
			SPI_Read_UserPage(Page10,ucPageData);	// read Page10, how often should green LED blink
			SPI_Program_UserPage(Page12,ucPageData);// programm Data in Page 12
			SPI_Power_Down();						// Power down PaLFI
			
			j=0;
	  		while (j< sizeof SERIAL_Recieved)		// Send out Text over Serial to Host PC
	  		{
	  			while (!(IFG2&UCA0TXIFG));      	// USCI_A0 TX buffer ready?
	  			UCA0TXBUF = SERIAL_Recieved[j];
	  			j++;
	  		}
		
			
	  		for (i=0;i<ucPageData[0];i++)
			{
				j=0;
				TACCTL0 &= ~CCIFG;
				TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
				TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
				TACCR0 = 16384;						// clock 0.5 second, load Capture Compare of Timer A
				__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
				LED_ON(LED_RED);					// Toggle red LED
				
		  		while (j< sizeof LED_blink_RED)		// Send out Text over Serial to Host PC
		  		{
		  			while (!(IFG2&UCA0TXIFG));      // USCI_A0 TX buffer ready?
		  			UCA0TXBUF = LED_blink_RED[j];
		  			j++;
		  		}
		  		
				TACCTL0 &= ~CCIFG;
				TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
				TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
				TACCR0 = 16384;						// clock 0.5 second, load Capture Compare of Timer A
				__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
				LED_OFF(LED_RED);					// Toggle red LED
			}
	  	}
	  	else										// recieved wrong serial Command
	  	{
	  		j=0;
	  		while (j< sizeof wrong_serial_code)		// Send out Text over Serial to Host PC
	  		{
	  			while (!(IFG2&UCA0TXIFG));      	// USCI_A0 TX buffer ready?
	  			UCA0TXBUF = wrong_serial_code[j];
	  			j++;
	  		}
	  	}
	  }
	  
	  
	  else if (((P2IFG & CU_BUSY) == CU_BUSY)&&(first_run == 1))	// Busy Interrupt from MSP Access Mode
	  {
	  	while ((P2IN & CU_BUSY) == CU_BUSY);			// wait until busy goes low (PaLFI is ready)
	  	
	  	UCA0CTL1 |= UCSWRST;     			            // **deactivate USCI state machine**
	  	j=0;
	  	SPI_Read_UserPage(Page13,ucPageData);			// read Page13, Byte 0 defines selective Adress
	  	SPI_Read_CU_Data(ucMSPAccessData);				// request Data from PaLFI for MSP Action
	  	
	  	
	  	//ucMSPAccessData[2] = IntDegC;					// add temperature to MSP Access Data
	  	//ucMSPAccessData[3] = IntDegF;					// add temperature to MSP Access Data
	  	
	  	if ((ucMSPAccessData[4] == ucPageData[0]) || (ucMSPAccessData[4] == 0x00)) // Selective Adress for this device
	  	
	  	{
	
		  	if (ucMSPAccessData[1] == 0x1)				// if Byte 1 of MSP Access = 0x1 blink red LED
		  	{
			  	status = SPI_Write_CU_Data(ucMSPAccessData);	// send MSP Access Data back, no SPI Power Down, no change to MSP Access Data (in this case)!
			  	UCB0CTL1 |= UCSWRST;                     		// **deactivate SPI state machine**
			  	
			  	TACCTL0 &= ~CCIFG;								// Disable TIMERA Capture Compare Flag
				TACCTL0 = CCIE;                     			// TACCR0 interrupt enabled
				TACTL = TASSEL_1 + MC_1;						// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
				TACCR0 = 16384;									// clock 0.5 second, load Capture Compare of Timer A
				__bis_SR_register(LPM3_bits + GIE); 			// Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
			  	  	
		  	  	for (i=0;i<ucMSPAccessData[0];i++)
				{
					j=0;
					TACCTL0 &= ~CCIFG;
					TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
					TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
					TACCR0 = 16384;						// clock 0.5 second, load Capture Compare of Timer A
					__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
					LED_ON(LED_RED);					// Toggle red LED
	
					TACCTL0 &= ~CCIFG;
					TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
					TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
					TACCR0 = 16384;						// clock 0.5 second, load Capture Compare of Timer A
					__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
					LED_OFF(LED_RED);					// Toggle red LED
				}
		  	}
		  	
		  	else if (ucMSPAccessData[1] == 0x0)			// if Byte 1 of MSP Access = 0x0 blink green LED
		  	{
		  	
			  	status = SPI_Write_CU_Data(ucMSPAccessData);	// send MSP Access Data back, no SPI Power Down, no change to MSP Access Data (in this case)!
			  	UCB0CTL1 |= UCSWRST;                     		// **deactivate SPI state machine**
			  	
			  	TACCTL0 &= ~CCIFG;								// Disable TIMERA Capture Compare Flag
				TACCTL0 = CCIE;                     			// TACCR0 interrupt enabled
				TACTL = TASSEL_1 + MC_1;						// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
				TACCR0 = 16384;									// clock 0.5 second, load Capture Compare of Timer A
				__bis_SR_register(LPM3_bits + GIE); 			// Enter LPM3, enable interrupts, only TIMERA Interrupt availabe	
		  		
		  		for (i=0;i<ucMSPAccessData[0];i++)
				{
					j=0;
					TACCTL0 &= ~CCIFG;
					TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
					TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
					TACCR0 = 16384;						// clock 0.5 second, load Capture Compare of Timer A
					__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
					LED_ON(LED_GREEN);					// Toggle green LED
	
					TACCTL0 &= ~CCIFG;
					TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
					TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
					TACCR0 = 16384;						// clock 0.5 second, load Capture Compare of Timer A
					__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
					LED_OFF(LED_GREEN);					// Toggle green LED
				}
		  	}
	  	  	else if (ucMSPAccessData[1] == 0xFF)			// Battery Meassurement by the MSP430
		  	{
		  		ADC10CTL1 = INCH_11;                      // AVcc/2
				ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE ; // Switch on ADC, 2.5 V RefV
				
				TACCTL0 &= ~CCIFG;
				TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
				TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
				TACCR0 = 50;						// clock 0.5 second, load Capture Compare of Timer A
				__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
	
			  	ADC10CTL0 |= ENC + ADC10SC;         // Sampling and conversion start
			  	
			  	__bis_SR_register(CPUOFF + GIE);        // enter LPM0 with interrupts enabled
			  	
			  	//while (ADC10CTL1 & ADC10BUSY);      // ADC10BUSY? wait for ADC10 to finish meassurement
			 	
			  	if (ADC10MEM == 0x3FF)				// if voltage is bigger than 1.5 V use 2.5 V Reference Voltage
			  	{
			  		ADC10CTL0 &= ~ENC;						// clear ENC bit
			  		ADC10CTL1 = INCH_11;                    // ADC Channel measures half of the Supply Voltage AVcc/2
					ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE + REF2_5V; // Switch on ADC, 2.5 V RefV
				
					TACCTL0 &= ~CCIFG;					// clear Capture Compare Interrupt Flag
					TACCTL0 = CCIE;                     // TACCR0 interrupt enabled
					TACTL = TASSEL_1 + MC_1;			// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
					TACCR0 = 50;						// clock 0.5 second, load Capture Compare of Timer A
					__bis_SR_register(LPM3_bits + GIE); // Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
	
			  		ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
			  	
			  		__bis_SR_register(CPUOFF + GIE);        // enter LPM0 with interrupts enabled
			  		
			  		//while (ADC10CTL1 & ADC10BUSY);      // ADC10BUSY? wait for ADC10 to finish meassurement
			  		ucMSPAccessData[5] = 0x1;			// mark that Reference Voltage 2.5V was used	
			  	}
			  	else
			  	{
			  		ucMSPAccessData[5] = 0x0;			// mark that Reference Voltage 1.5V was used
			  	}
			  		  	
			  	ucMSPAccessData[3] = (ADC10MEM & 0xFF); // prepare low byte of result
			  	ucMSPAccessData[2] = (ADC10MEM >> 8);	// prepare high byte of result
			
			 	status = SPI_Write_CU_Data(ucMSPAccessData);	// send MSP Access Data back, no SPI Power Down, send back battery voltage!
				
				ADC10CTL0 &= ~ENC;						// clear ENC bit
				ADC10CTL0 = 0;							// siwtch off ADC - only works if ENC cleared before
		
			}
	  	}
	  		
	    else 
	    {
	  		SPI_Power_Down();						// Power down PaLFI
	  	}
	  	
	  UCB0CTL1 &= ~UCSWRST;                     	// **reactivte SPI state machine**
	  UCA0CTL1 &= ~UCSWRST;     			        // **reactivate USCI state machine**
	  		
	  	
	  }
  
	    	  
	  P1IFG&=~BIT2;								// disable interupt flag
	  P2IFG&=~CU_BUSY;							// disable busy interupt flag
	  IFG2 &=~UCA0RXIFG;						// disable Serial Interrupt Flag	
	  P1IE|= BIT2;								// enable Push Button Interrupt
	  P2IE|=CU_BUSY;							// enable Busy Interupt
	  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
 
	  first_run = 1;
	  	
	  __bis_SR_register(LPM3_bits + GIE);   	// Enter LPM3, enable interrupts
	}    
}

/********************************additional functions**********************************************/


int autotrim(void)								// calibrates DCO and trims PaLFI Chip to 134,2 +- 200 Hz
{
 /*****************************************************************************************
  * Trims the TMS37157 to 134,2 kHz +- 200Hz. CLKAM is connected to P2.1 as INCLK for TIMER A. 
  * Timer A counts up to 800 clock cycles of CLK_AM. The DCO is calibrated to 8MHz.
  * And is used as clock source for TIMER B (SMCLK). TIMER B counts until TIMER A reaches 800.
  * The Value saved in TBR is the time needed for 800 clocks of CLK_AM. As long as TBR<47630 
  * the MSP430 increases the trim capcitors in the TMS37157, until the measured time >= 47630.
  * The corresponding trim capacitor value is then programmed into the Trim EEPROM of the TMS37157. 
  * 
  * *****************************************************************************************/
   
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
	  	TACCR0 = 800;						// clock 1600 cycles of CLK-AM, load Capture Compare of Timer A
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
	 		SPI_TRIM_W_PROG(prog_data - 1);				// prog last values
	 	}
	 			
  		SPI_Program_UserPage (Page2,ucPageData);	// Set Bit for trimmed Device
  		SPI_Power_Down();							// switch of PaLFI
		
		LED_OFF(LED_RED);							// Switch off red LED		
				
		TACCTL0 &= ~CCIFG;							// reset CCIFG flag
		TACCTL0 = CCIE;                     	// TACCR0 interrupt enabled
		TACTL = TASSEL_1 + MC_1;				// Timer A select, clock source ACLK, count up to TACCR0, activate Interrupt Request
		TACCR0 = 32768;							// clock 1 second, load Capture Compare of Timer A
		__bis_SR_register(LPM3_bits + GIE); 	// Enter LPM3, enable interrupts, only TIMERA Interrupt availabe
		
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

void LED_ON (int LED)
{
	P1OUT |= LED;
}

void LED_OFF (int LED)
{
	P1OUT &= ~LED;
}

/***************************************Interrupt Vectors***************************/

//Serial A Interrupt Service Routine

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	P1IE&= ~PUSH_BUTTON;							// disable Push Button Interrupt
	P2IE&= ~CU_BUSY;								// disable Busy Interupt
	IE2 &= ~UCA0RXIE;                          		// Disable USCI_A0 RX interrupt
	__bic_SR_register_on_exit(LPM3_bits + GIE);    	// Clear LPM3 bits and GIE bits from 0(SR)
}

// Port 1 Interrupt Service Routine (Push Button Interrupt)

#pragma vector=PORT1_VECTOR							// Push Button Interrupt
__interrupt void PORT1_ISR(void)
{
	P1IE&= ~PUSH_BUTTON;							// disable Push Button Interrupt
	P2IE&= ~CU_BUSY;								// disable Busy Interupt
	IE2 &= ~UCA0RXIE;                          		// Disable USCI_A0 RX interrupt
	__bic_SR_register_on_exit(LPM3_bits + GIE); 	// Clear LPM3 bits and GIE bits from 0(SR)
}

// Port 2 Interrupt Service Routine (Busy Interrupt) 

#pragma vector=PORT2_VECTOR							// Busy Interrupt -- MSP Access
__interrupt void PORT2_ISR(void)
{
	P1IE&= ~PUSH_BUTTON;							// disable Push Button Interrupt
	P2IE&= ~CU_BUSY;								// disable Busy Interupt
	IE2 &= ~UCA0RXIE;                          		// Disable USCI_A0 RX interrupt
	__bic_SR_register_on_exit(LPM3_bits + GIE);     // Clear LPM3 bits and GIE bits from 0(SR)
}

// Timer A0 interrupt service routine

#pragma vector=TIMERA0_VECTOR
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

