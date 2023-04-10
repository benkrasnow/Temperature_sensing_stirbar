/******************************************************************************\
*           Copyright (C) 2005 Texas Instruments
*                           All Rights Reserved
*------------------------------------------------------------------------------
* FILENAME...... SPI_LowLevel.c
* DATE CREATED.. 10/21/2008
* WRITTEN BY.... J. Austen, S.Recknagel
*
*   Implementation of the Basic SPI communication from a microcontroller to the TMS37157 Frontend
*   Basic functionaly:
*       - Initialization of the SPI interface of the MSP430F2274
*       - Transmit bytes to the Frontend
*       - Receive bytes from the Frontend
*
*------------------------------------------------------------------------------
* HISTORY:
*


\******************************************************************************/
#include  "PaLFI_Transponder.h"
#include  "msp430g2452.h"

/******************************************************************************/

unsigned char ucTX_ONLY;
extern unsigned char ucWDT_Count;

/******************************************************************************/
void MSP430_SPI_Init(void)
/****************************************************************************************
 *
 *  Initialization of the SPI Interface
 *
 *   SPI_SIMO	  //  Output
 *   SPI_SOMI	  //  Input
 *   SPI_CLK 	  //  Output
 *   TMS_BUSY	  //  Input
 *   PUSH		  //  Output
 *   CLK_AM	  //  Input
 *
 ****************************************************************************************/
{
    P1DIR |= CU_PUSH;							// Push Output
    P1DIR &= ~CU_BUSY;						// Busy Input
    P1DIR &= ~CLK_AM;							// CLK_AM Input

    P1DIR |= SPI_CLK + SPI_SIMO;							// SPI



    //P1SEL |= CLK_AM;							// TACLK Function for CLK_AM  only used for autotune

    P1SEL |= SPI_SIMO + SPI_SOMI + SPI_CLK;	// Configure Port 1 for SPI functions


    //UCB0CTL0 |= UCMSB + UCMST + UCSYNC;       // 3-pin, 8-bit SPI mstr, MSB 1st
    USICTL0 |= USIPE7 + USIPE6 + USIPE5 + USIMST;
    //USICTL1 |=  USIIE;  //  interrupt enable

    //UCB0CTL1 |= UCSSEL_2;                     // SMCLK
    //UCB0BR0 = 0x08;							// SPICLK = 2MHz/8 (About 250kHz)
    // UCB0BR1 = 0;
    USICKCTL |= USIDIV_2 + USISSEL_2; //Use SMCLK divided by 4



    //UCB0CTL1 |= UCSWRST;                     // **don't activate SPI yet
    USICTL0 |= USISWRST;

}


//******************************************************************************

void ErrorMode(void) {
    WDTCTL = WDT_MRST_0_064;
    //WDTCTL = WDT_MRST_32;
    //for(;;);					// MCU restart....
}
/******************************************************************************/
void Wake_PaLFI(void) {
    P1OUT |= CU_PUSH;						// pess push at PaLFI, to wake up PaLFI

    while ((P1IN & CU_BUSY) == 0x00);
    while ((P1IN & CU_BUSY) == CU_BUSY);	// wait until PaLFI is ready

    P1OUT &= ~CU_PUSH;						// release PUSH
}

void Stop_SPI_WDT(void) {
    WDT_HOLD();
    ucWDT_Count = 0;
    IE1 &= ~WDTIE;
}

void Wait_for_Busy_low (void) {
    while ((P1IN&CU_BUSY) == CU_BUSY);	// wait until busy goes low, PaLFI is ready then
}

void Start_SPI_WDT(void) {
    WDT_HOLD();
    /*
        ucWDT_Count = 0;
        WDT_INTERVAL_MODE();                      // Start timer in interval mode
        IE1 |= WDTIE;
        _EINT();                                  // Enable interrupts
        */ //Error
}


/******************************************************************************/
unsigned char MSP430_SPI_Rx(unsigned char *RxBuffer, unsigned char size)
/****************************************************************************************
 *
 *  Receive a telegram with the SPI Emulation
 *
 ****************************************************************************************/
{
    if(!size) return Fail;					        		// invalid data size

    size--;

    do {

        USICNT = 0x08;
        while (!(USIIFG & USICTL1));
        while((P1IN & CU_BUSY) == CU_BUSY);
        while(!(P1IN & CU_BUSY));
        *RxBuffer++ = USISRL;

    } while(--size);


    USICNT = 0x08;
    while (!(USIIFG & USICTL1));
    while((P1IN & CU_BUSY) == CU_BUSY);
    *RxBuffer = USISRL;

    return Okay;
}



/******************************************************************************/
unsigned char MSP430_SPI_Tx(unsigned char *TxBuffer, unsigned char size) {
    if(!size) return Fail;					        	// invalid data size

    if((P1IN & CU_BUSY) == CU_BUSY) return Fail;	  	// Control Unit is not ready(Busy is high)
    if (!(USIIFG & USICTL1)) return Fail;					// MSP430 SPI Tx buffer is not ready.

    do {
        USICTL0 |= USIOE;  // Set output enable
        USISRL = *TxBuffer++;


        USICNT = 0x08;

        while (!(USIIFG & USICTL1));
        while((P1IN & CU_BUSY) == CU_BUSY);
        if (ucTX_ONLY && size == 0x01)
            ;                           			// Only transmit data to frontend then busy line stays low after last SPI byte !
        else
            while(!(P1IN & CU_BUSY));               // wait until busy goes from low to high again
    } while(--size);

    USICTL0 &= ~USIOE;  // Clear output enable
    //  Stop_SPI_WDT();
    return Okay;
}






