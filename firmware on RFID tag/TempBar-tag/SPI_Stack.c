/******************************************************************************\
*           Copyright (C) 2005 Texas Instruments
*                           All Rights Reserved
*------------------------------------------------------------------------------
* FILENAME...... SPI.c
* DATE CREATED.. 10/21/2008
* WRITTEN BY.... J. Austen, S. Recknagel
*
*   Implementation of the SPI communication from a microcontroller to the TMS37157 Frontend
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
#include  <math.h>
/******************************************************************************/
// Basic function from SPI_LowLevel
unsigned char MSP430_SPI_Tx(unsigned char *TxBuffer, unsigned char size);


/******************************************************************************/
extern unsigned char SW_Data,ucTX_ONLY;
#define MAX_BUF_SIZE      16

/******************************************************************************/
struct stSPI_Stack
{
  unsigned char ucOutput[MAX_BUF_SIZE];
  unsigned char ucInput[MAX_BUF_SIZE];
  unsigned char ucOutputPointer;
};

/******************************************************************************/
struct _st_TRP_CONTENT  TRP_Data;
struct stSPI_Stack      SPI_Stack;

void SPI_Buf_Reset(void)
{
 /****************************************************************************************
  *
  *  Set Buffer Pointer to second position
  *  First position is reserved for the length byte
  *  and could be accessed by the function SPI_Buf_Set_Telegram_Length()
  *
  ****************************************************************************************/

 SPI_Stack.ucOutputPointer = 1;
}

void SPI_Buf_Set_Telegram_Length(void)
{
 /****************************************************************************************
  *
  *  Directly access the output buffer on the position 0 --> Length of the telegram
  *
  ****************************************************************************************/
  if (SPI_Stack.ucOutputPointer)
    SPI_Stack.ucOutput[0] = SPI_Stack.ucOutputPointer-1;
}

void SPI_Buf_Set_Output_Byte(unsigned char ucByte)
{
 /****************************************************************************************
  *
  *  Directly access the output buffer on the position 0 --> Length of the telegram
  *
  ****************************************************************************************/
  if (SPI_Stack.ucOutputPointer >= MAX_BUF_SIZE)
    {
      ErrorMode();
    }
  else
    {
    SPI_Stack.ucOutput[SPI_Stack.ucOutputPointer] = ucByte;
    SPI_Stack.ucOutputPointer++;
    }
}

void SPI_Buf_Send(void)
{
 /****************************************************************************************
  *
  *  Send the Buffer to the Frontend
  *
  ****************************************************************************************/
   if (MSP430_SPI_Tx(SPI_Stack.ucOutput,SPI_Stack.ucOutput[0]+1))
      ErrorMode();
}

void SPI_Set_Up_Telegram(void)
{ /****************************************************************************************
  *
  *  This set up is equal for each telegram
  *
  ****************************************************************************************/
    SPI_Buf_Reset();                                      // Reset the Send Buffer Pointer
    SPI_Buf_Set_Output_Byte(TAC_COMMAND_BYTE);            // Do a TAC command
}

/**********************************  Transponder Access Commands (TAC) **********************************/

void SPI_Read_SerialNum(void)
 /****************************************************************************************
  *
  *  Read out serial number, User data 1 and Password
  *
  ****************************************************************************************/
{
  SPI_Set_Up_Telegram();                                 
  SPI_Buf_Set_Output_Byte(Page3);                        
  SPI_Buf_Set_Telegram_Length();

  SPI_Buf_Send();

  if (MSP430_SPI_Rx(SPI_Stack.ucInput,7))
    ErrorMode();

  TRP_Data.SelectiveAddress = SPI_Stack.ucInput[0];
  TRP_Data.KeyNumber        = SPI_Stack.ucInput[1];     // equal to User data 1
  TRP_Data.SerialNumber[0]  = SPI_Stack.ucInput[2];		// Manu Code / Page 3
  TRP_Data.SerialNumber[1]  = SPI_Stack.ucInput[3];		// Ser. Nr. / Page 3
  TRP_Data.SerialNumber[2]  = SPI_Stack.ucInput[4];		// Ser. Nr. / Page 3
  TRP_Data.SerialNumber[3]  = SPI_Stack.ucInput[5];		// Ser. Nr. / Page 3
}


void SPI_Read_UserPage(unsigned char ucPage, unsigned char *ucData)
/******************************************************************************
 * 
 *	Read User Page out of EEPROM from PaLFI
 * 
 ******************************************************************************/
{
  unsigned char i;

  SPI_Set_Up_Telegram();                                  
  SPI_Buf_Set_Output_Byte(ucPage);              	// Read page
  SPI_Buf_Set_Telegram_Length();

  SPI_Buf_Send();

  if(MSP430_SPI_Rx(SPI_Stack.ucInput,7))			// recieve 7 Data Bytes (Page2(1),Page to read (5), Read Adress(1))
    ErrorMode();

  for (i=0; i<5; i++)
  {
    ucData[i] = SPI_Stack.ucInput[i+1];				// copy recieved data to ucData (only Read Page data (5 Bytes))
  }
}

void SPI_Program_UserPage(unsigned char ucPage, unsigned char *ucData)
/******************************************************************************
 * 
 *	Program User Page to EEPROM from PaLFI
 * 
 ******************************************************************************/
{
  unsigned char i;
  unsigned char count = 5;
  
  if ((ucPage == 0x04) || (ucPage == 0x08)) count=1;	// Page 1 and 2 only 1 Byte

  SPI_Set_Up_Telegram();                                // Enter CSP and if needed password
  SPI_Buf_Set_Output_Byte(ucPage+1);                    // Select command 'Program Page X'
  if (TRP_Data.SelectiveAddress != 0xFF)				// if Page 1 different 0xFF than selective adressing is necessary
    {// Selective address required for programming
      SPI_Buf_Set_Output_Byte(TRP_Data.SelectiveAddress);	// add selective Adress to SPI Command
    }
  for (i=0; i<count; i++)
    {
      SPI_Buf_Set_Output_Byte(ucData[i]);				// Set data to program in Page
    }
  SPI_Buf_Set_Telegram_Length();
  SPI_Buf_Send();

  if(MSP430_SPI_Rx(SPI_Stack.ucInput,7))                // Get response,program Command performs program and read!
    ErrorMode();
  
  for (i=0; i<count; i++)
    {
      ucData[i] = SPI_Stack.ucInput[i+1];				// copy recieved data to ucData (only Read Page data (5 Bytes))	
    }
}

void SPI_Read_CU_Data(unsigned char *ucData)
/******************************************************************************
 * 
 *	MSP-Access: Read recieved MSP Acccess Data from PaLFI (6 Byte)
 * 
 ******************************************************************************/
{
  unsigned char i;

  SPI_Buf_Reset();                                  
  SPI_Buf_Set_Output_Byte(MSP430_CU_GetData);                                              // Read page
  SPI_Buf_Set_Telegram_Length();

  SPI_Buf_Send();

  if(MSP430_SPI_Rx(SPI_Stack.ucInput,6))			// recieve 6 Data Bytes 
    ErrorMode();

  for (i=0; i<6; i++)
  {
    ucData[i] = SPI_Stack.ucInput[i];				// MSP Access data is always 6 Byte
  }
}


unsigned char SPI_Write_CU_Data(unsigned char *ucData)
/******************************************************************************
 * 
 *	MSP-Access: Send MSP Acccess Data back to PaLFI (6 Byte)
 * 
 ******************************************************************************/
{
  unsigned char i;
  unsigned char count = 6;
  unsigned char recieve;	

  SPI_Buf_Reset();                                  
  SPI_Buf_Set_Output_Byte(MSP430_CU_SendData);                                              // Read page
  for (i=0; i<count; i++)
  {
    SPI_Buf_Set_Output_Byte(ucData[i]);
  }
    
  SPI_Buf_Set_Telegram_Length();
  
  SPI_Buf_Send();



  USICNT = 0x08;
  while (!(USIIFG & USICTL1));
  while((P1IN & CU_BUSY) == CU_BUSY);
  recieve = USISRL;
  
  return recieve;
}

/***************************************  Enhanced Commands (EC) ****************************************/

void SPI_Read_PCU_State(void)
{
/****************************************************************************************
 *
 *  Read PCU States, defines if MSP Started because of an MSP Access or a Push Button Command
 *  only necessary if MSP is supplied with VBATI of PaLFI and not directly connected to Battery, 
 * 	if MSP is directly connected to the Battery, a Busy Interrupt could wake the MSP from LPM,
 * 	signalizing that the TMS37157 recieved data which has to be forwarded to the MSP.
 *
 ****************************************************************************************/

  SPI_Buf_Reset();                                       // Reset the Send Buffer Pointer
  SPI_Buf_Set_Output_Byte(MSP430_Read_CU_Status);         // Command: Read Status from PCU
  SPI_Buf_Set_Telegram_Length();

  SPI_Buf_Send();

  // Read out response
  if(MSP430_SPI_Rx(SPI_Stack.ucInput,2))//Error should be 2
    ErrorMode();
  switch (SPI_Stack.ucInput[0])
  {
    case 1:
      TRP_Data.ucPCU_Mode = PCU_PUSH_BUTTON;
      break;
    case 2:
      TRP_Data.ucPCU_Mode = PCU_MSP_ACCESS;
      break;
    default:
      ErrorMode();
      break;
  }
}

void SPI_CLKA_ON(void)
/******************************************************************************
 * 
 *	Switch on the LC Tank Circuit and switch it out to CLKAM
 * 
 ******************************************************************************/
{
	unsigned int i;
	
	ucTX_ONLY = true;
	SPI_Buf_Reset();                                       // Reset the Send Buffer Pointer
	SPI_Buf_Set_Output_Byte(CU_Oscillator_134KHz);         // Command: switch 134kHz Osc on
	SPI_Buf_Set_Telegram_Length();
	
	SPI_Buf_Send();
	
	SPI_Buf_Reset();                                       // Reset the Send Buffer Pointer
	SPI_Buf_Set_Output_Byte(CU_CLKA_On);         			// Command: switch OSC to CLKAM
	SPI_Buf_Set_Telegram_Length();
	
	SPI_Buf_Send();
	ucTX_ONLY = false;
	
	Wait_for_Busy_low();
	
	for( i = 0; i < 0xFFF; i++){}					// delay for start up and stabilizing LC Tank Circuit	
}

void SPI_CLKA_OFF(void)
/******************************************************************************
 * 
 *	Switch off the LC Tank Circuit 
 * 
 ******************************************************************************/
{
	ucTX_ONLY = true;								// no answer from 37157 expected	
	SPI_Buf_Reset();                                // Reset the Send Buffer Pointer
	SPI_Buf_Set_Output_Byte(CU_Oscillator_Off);     // Command: switch 134kHz Osc off
	SPI_Buf_Set_Telegram_Length();
	
	SPI_Buf_Send();									// Send data through SPI Interface
	
	SPI_Buf_Reset();                              	// Reset the Send Buffer Pointer
	SPI_Buf_Set_Output_Byte(CU_CLKA_Off);         	// Command: switch CLKAM off
	SPI_Buf_Set_Telegram_Length();					// include Length Byte in SPI Command
	
	SPI_Buf_Send();									// Send data through SPI Interface
	ucTX_ONLY = false;								// set global variable back to: answer expected
}

void SPI_TRIM_W_PROG(char data)						// program trim EEPROM
/******************************************************************************
 * 
 *	Programm Trim EEPROM
 * 
 ******************************************************************************/
{
	ucTX_ONLY = true;									// no answer from Front End
	SPI_Buf_Reset();                                    // Reset the Send Buffer Pointer
	SPI_Buf_Set_Output_Byte(MSP430_Trim_w_prog);        // Command: Programm Trim EEPROM
	SPI_Buf_Set_Output_Byte(data);						// send trim Data
	SPI_Buf_Set_Telegram_Length();						// Set Telegram Length
	
	SPI_Buf_Send();										// Send SPI String
	ucTX_ONLY = false;									// set global variable back
}

void SPI_TRIM_WO_PROG(char data)						// set trim capacitor without programming trim EEPROM
/******************************************************************************
 * 
 *	Set Trim Switches but don't programm EEPROM
 * 
 ******************************************************************************/
{
	ucTX_ONLY = true;									// no answer from Front End
	
	SPI_Buf_Reset();                                    // Reset the Send Buffer Pointer
	SPI_Buf_Set_Output_Byte(MSP430_Trim_wo_prog);       // Command: set trim capacitor without programming
	SPI_Buf_Set_Output_Byte(data);						// send trim Data
	SPI_Buf_Set_Telegram_Length();						// Set Telegram Length
	
	SPI_Buf_Send();										// Send SPI String
	ucTX_ONLY = false;									// set global variable back
}


void SPI_Power_Down(void)
{
/****************************************************************************************
 *
 *  Send Power down command: After this command
 *
 ****************************************************************************************/
  ucTX_ONLY = true;
  SPI_Buf_Reset();                                       // Reset the Send Buffer Pointer
  SPI_Buf_Set_Output_Byte(MSP430_PowerDownMode);         // Command: Read Status from PCU
  SPI_Buf_Set_Telegram_Length();

  SPI_Buf_Send();
  ucTX_ONLY = false;
}

void SPI_CRC_Calc(unsigned char ucStart,unsigned char ucLength, unsigned char *ucData, unsigned char *ucCRC)
/****************************************************************************************
 *
 *  perform CRC Calculation 
 *
 ****************************************************************************************/
{ 
  unsigned char i;

  SPI_Buf_Reset();                                       // Reset the Send Buffer Pointer
  SPI_Buf_Set_Output_Byte(CU_CRC_Calculation);           // Command: CRC calculation with Start value 0x3791
  SPI_Buf_Set_Output_Byte(ucLength-ucStart);             // Send Length of data stream
  for (i=ucStart; i<ucLength; i++)
    {
      SPI_Buf_Set_Output_Byte(ucData[i]);
    }
  SPI_Buf_Set_Telegram_Length();

  SPI_Buf_Send();

  // Read out response
  if(MSP430_SPI_Rx(SPI_Stack.ucInput,2))
    ErrorMode();
  ucCRC[0] = SPI_Stack.ucInput[0];
  ucCRC[1] = SPI_Stack.ucInput[1];

}






