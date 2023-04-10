#ifndef CED80_TRANSPONDER_H_
#define CED80_TRANSPONDER_H_

#define BIT0                (0x0001)
#define BIT1                (0x0002)
#define BIT2                (0x0004)
#define BIT3                (0x0008)
#define BIT4                (0x0010)
#define BIT5                (0x0020)
#define BIT6                (0x0040)
#define BIT7                (0x0080)
#define BIT8                (0x0100)
#define BIT9                (0x0200)
#define BITA                (0x0400)
#define BITB                (0x0800)
#define BITC                (0x1000)
#define BITD                (0x2000)
#define BITE                (0x4000)
#define BITF                (0x8000)

/**************Configuration for eZ430-PaLFI******************************/

#define LED_RED				(0x0001)		// P1.0 Output: Red LED
#define LED_GREEN			(0x0002)		// P1.1 Output: Green LED
//#define PUSH_BUTTON			(0x0004)		// P1.2 Input: Push Button  // Not used in TempBar

#define	SPI_SIMO		    BIT6	    // P1.7 Output:	Output to Frontend = TMS37157
#define	SPI_SOMI		    BIT7	    // P1.8 Input:	Input from Frontend = TMS37157
#define	SPI_CLK 		    BIT5		// P1.6 Output:	SPI clock

#define	CU_BUSY	            BIT3        // P1.3 Input:	Busy signal from Frontend   //Not used in TempBar
#define CLK_AM				BIT4		// P1.4 Input: CLK_AM signal from Frontend
#define	CU_PUSH	            BIT2        // P1.2 Output:	Push signal to Frontend

/**************SPI Libary definitions for TMS37157***************************/

#define false               (0x00)
#define true                (0x01)

#define STOP_OPERATION      true
#define CONTINUE_OPERATION  false

#define WDT_HOLD()          (WDTCTL = WDTPW+WDTCNTCL+WDTHOLD)

struct
_st_TRP_CONTENT
{
  unsigned char ucPCU_Mode;			// Defines in MSP_Access or Push Button Command
  unsigned char Command;
  unsigned char SerialNumber[4];	// Serial Number + Manu Code Page3
  unsigned char KeyNumber;	      	// equal to User data 1/Page 2
  unsigned char SelectiveAddress;	// Page 1, in eZ430-PaLFI locked to FF
  unsigned char PushButtonMask;		// Page 18
  unsigned char RollingCode[5];		// Page 55, not important
};

union unChar2Int
{
unsigned char ucChar[2];
unsigned int  uiInt;
};

struct _stByteRC
{
  unsigned char  ucData[5];
};

struct _stLongRC
{
  unsigned long  ulLong;
  unsigned char  ucChar;
};

union _unRC
{
struct _stByteRC stCharRC;
struct _stLongRC stLongRC;
};

/******************************************************************************/
#define     PCU_PUSH_BUTTON     1
#define     PCU_MSP_ACCESS      2

/******************************************************************************/
#define		Page1		0x04		// SELECTIVE ADDRESS
#define		Page2	  	0x08		// USER DATA 1
#define		Page3 		0x0C		// UNIQUE IDENTIFICATION
#define		Page8		0x20		// USER DATA 2
#define		Page9  		0x24		// USER DATA 3
#define		Page10		0x28		// USER DATA 4
#define		Page11		0x2C		// USER DATA 5   
#define		Page12		0x30		// USER DATA 6   
#define		Page13		0x34		// USER DATA 7
#define		Page14		0x38		// USER DATA 8
#define		Page15		0x3C		// USER DATA 9
#define		Page30		0x78		// CONFIGURATION

#define		Page40		0xA0		// User data 10
#define		Page41		0xA4		// User data 11
#define		Page42		0xA8		// User data 12
#define		Page43		0xAC		// User data 13
#define		Page44		0xB0		// User data 14
#define		Page45		0xB4		// User data 15
#define		Page46		0xB8		// User data 16
#define		Page47		0xBC		// User data 17
#define		Page48		0xC0		// User data 18
#define		Page49		0xC4		// User data 19
#define		Page50		0xC8		// User data 20
#define		Page51		0xCC		// User data 21
#define		Page52		0xD0		// User data 22
#define		Page53		0xD4		// User data 23
#define		Page54		0xD8		// User data 24
#define		Page55		0xDC		// User data 25

/******************************************************************************/

/******************************************************************************/
// Transponder Access Command
#define		TAC_COMMAND_BYTE		  0x00

// Enhanced Commands
#define		MSP430_Read_CU_Status		  0xB4	// Get Status from TMS37157
#define		MSP430_PowerDownMode		  0xB8	// Power Down 37157
#define     MSP430_CU_GetData             0xB0	// Request Data from MSP Access
#define     MSP430_CU_SendData            0xB1	// Send Data for MSP Access
#define 	MSP430_Trim_w_prog			  0x89	// Trim TMS37157 with Trim EEPROM
#define		MSP430_Trim_wo_prog			  0xA9	// Trim TMS37157 without Trim EEPROM

#define		CU_CRC_Calculation	          0x80	// Perform CRC16 Calculation
#define		CU_Oscillator_Off		  	  0x94	// Switch off the LC Tank Circuit
#define		CU_Oscillator_134KHz		  0x95	// Switch on LC Tank Circuit
#define		CU_Oscillator_134KHz_DIV4	  0x96	// Switch on LC Tank Circuit with 1/4 of the Frequency
#define		CU_CLKA_Off			  		  0x9C	// Switch of LC Tank Circuit at CLK_AM Pin
#define		CU_CLKA_On			          0x9D	// Switch LC Tank Circuit to CLK_AM Pin

/******************************************************************************/

#define		Okay				  0x00
#define		Fail				  0x01

/*****************SPI functions to communicate with TMS37157**********************/

void ErrorMode(void);
void MSP430_SPI_Init(void);
void Wake_PaLFI(void);

void SPI_Prog_RC(void);
void SPI_Power_Down(void);
void TRP_Read_PCU_Data(void);
void SPI_Read_SerialNum(void);
void SPI_Read_PCU_State(void);
void SPI_Read_UserPage(unsigned char ucPage, unsigned char *ucData);
void SPI_Program_UserPage(unsigned char ucPage, unsigned char *ucData);
void SPI_CRC_Calc(unsigned char ucStart,unsigned char ucLength, unsigned char *ucData, unsigned char *ucCRC);

void SPI_CLKA_Config(unsigned char ucMode);
void SPI_CLKA_ON(void);
void SPI_CLKA_OFF(void);
void SPI_TRIM_W_PROG(char data);
void SPI_TRIM_WO_PROG(char data);

void SPI_Read_CU_Data(unsigned char *ucData);
unsigned char SPI_Write_CU_Data(unsigned char *ucData);

unsigned char MSP430_SPI_Rx(unsigned char *RxBuffer, unsigned char size);

void Wait_for_Busy_low (void);

void LED_ON (int LED);
void LED_OFF (int LED);
void Toggle_LED_500ms (int LED);
void Delay_500ms (void);

#endif /*CED80_TRANSPONDER_H_*/
