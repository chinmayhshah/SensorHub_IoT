/**************************************************************************
*
*
*
*	Program to integrate following sensors with EFM32 (Leopard Gecko board)
*	and send commands to set points and read status from sensors
*	Interfacing following External Sensors
*	1)Magnetometer - Using I2C Protocol for interfacing Mag Sensor
*	2)Proximity Sensor - Using Interrupt,  Waits for  a count set as #define
*	3)LESENSE - Inductor (Internal),
*	5)Emulation Code -Referral from App Notes for Emulation of EEPROM and save
*		alert Set points
*   6)LESENSE - Ambient Light Sensor - Darkness  with PCNT and PRS()- have
*   	been removed from implementation as conflicting with LC Sensor
*	7)As of now the code is working only in EM1
* Version 5.1
*	References
*		1) For HMC5833L
*			a)http://hsel.co.uk/2014/08/13/stm32f0-mini-tutorial-using-the-i2c-peripheral-to-communicate-with-a-hmc5883l-digital-compass-ic/
*
*
*
****************************************************************************/


/******************************************************************//**
 * Headers Section
 *********************************************************************/
#include "mbed.h"
#include "em_chip.h"  //For Initializing chip according to design
#include "sleepmodes.h" //For using enable and disable  Energy modes
#include "em_letimer.h" //For using  LETimer
#include "em_gpio.h" // For using  LETimer
#include "em_emu.h" //For using Sleep modes
#include "em_adc.h"//For ADC
#include "em_leuart.h"//Header for LEUART
#include "string.h"//For String Functions
#include "em_lesense.h"// For LESENSE
#include "em_pcnt.h"// For PCNT
#include "em_prs.h"// For PRS
#include "em_acmp.h"// For ACMP
#include "em_i2c.h"// For I2C (HMC5833L)
#include <math.h>//for Floating
//For LESENSE
#include "em_msc.h"
#include "em_assert.h"
#include "em_dac.h"
#include "em_int.h"





/******************************************************************//**
 * Macro Section for Proximity Sensor
 *********************************************************************/
#define MAX_PROXIMITY_SENSOR 2
/******************************************************************//**
 * Macro Definition for LEUART
 *********************************************************************/

#define BAUD_RATE			  9600 					// Baudrate for LEUART(in bps)
#define NoOfBitsPerCharacter  leuartDatabits8    	//Number of Bits per character in LEUART
#define NoOfStopBits          leuartStopbits1   	//Number of Stop Bits in LEUART
#define Parity				  leuartNoParity		//Parity Bit in LEUART
#define RXI_PIN				  5						//Port D ,Pin5(TXO of BLE )
#define TX_PIN				  4						//Port D ,Pin4(RXI of BLE )
#define CTS_PIN				  8 					//Port D ,Pin15(CTS of BLE --Logic Zero )
#define debug 				  false					//Enable and Disable of Debugging
#define debugLoop 			  false					//Enable and Disable of LoopBack to debug UARt
#define MAX_COMMAND			  10					//maximum length of command
#define MAX_MESSAGE			  4*MAX_COMMAND			//maximum length of message


#define DMA_CHANNELS		 3
/******************************************************************//**
 * Macro Definition for DMA Channel for TX
 *********************************************************************/

#define DMA_CHANNEL_TX     1
#define DMA_PRIORITY_TX    false
#define DMA_ARBITRATE_TX   dmaArbitrate1
#define DMA_BUFFSIZE_TX    MAX_COMMAND
//#define DMA_TRFSIZE_TX     0

/******************************************************************//**
 * Macro Definition for DMA Channel for RX
 *********************************************************************/

#define DMA_CHANNEL_RX     0
#define DMA_PRIORITY_RX    true
#define DMA_ARBITRATE_RX   dmaArbitrate1
#define DMA_BUFFSIZE_RX    MAX_MESSAGE
//#define DMA_TRFSIZE_RX     0


/******************************************************************//**
 * Macro Definition for DMA Channel for Storing ADC Conversions
 *********************************************************************/

#define DMA_CHANNEL_ADC     2 //Assigning ADC Channel for DMA
#define DMA_PRIORITY_ADC    true
#define DMA_ARBITRATE_ADC   dmaArbitrate1
//#define DMA_TRFSIZE_ADC     0


/******************************************************************//**
 * Macro Definition for ADC Conversions and Temperature Limits
 *********************************************************************/

//#define ADCTriggerPeriod 	4000// Defining  count for Timer period to be 4 seconds(Count in decimal 4000 )
#define ADCTriggerPeriod 	40000// Defining  count for Timer period to be 4 seconds(Count in decimal 4000 )
								//After every four seconds Timer Interrupts
#define NumberOfSamples		200	//Number of Samples required for average by DMA
#define ScaledFrequency		875000  //for Acquisition time >2us,6.25KSPS of Smapling
									//Calculation requires   .923076 MHz with a scaling factor of 15.1667
									//875000(for Scalling of 16)
								    //As Prescalling allows only powers of 2 ,Prescalar 16 choosen
									//For 2 us Freq- 857142.85 (for Scalling of 16.33),Approximated to 16 which inturn results in 6.25KSPS of Smapling
#define TEMP_SIZE			9

short int LowerTempLimit;// Lower Limit Temperature
short int UpperTempLimit;// Upper Limit Temperature

/******************************************************************//**
 * Macro Definition for LESENSE Module to excite Ambient Light Sensor
 *********************************************************************/
#define LIGHT_SENSE 	6 //Port C ,Pin 6 ,Input pin for Controller
#define LIGHT_EXCITE	6 //Port D ,Pin 6 ,Output pin for Controller
#define LED_PIN			2 //Port E ,Pin 2 ,Output pin

#define DARK_VOLTAGE	0.9 //(In Voltage) Negative Select of Comparator when LEDON ----> LEDOFF
#define BRIGHT_VOLTAGE	2.0 //(In Voltage) Negative Select of Comparator when LEDON ----> LEDOFF
#define VDD 			3.3



//#define LCSENSE_SCAN_FREQ         20
#define LCSENSE_SCAN_FREQ         5
#define LIGHTSENSE_INTERRUPT      LESENSE_IF_CH6

/* LESENSE for IR Sensor*/
#define LIGHTSENSE_CH             6


/* ACMP */
#define ACMP_NEG_REF           acmpChannelVDD
#define ACMP_THRESHOLD         0x38                         /* Reference value for the lightsensor.
                                                             * Value works well in office light
                                                             * conditions. Might need adjustment
                                                             * for other conditions. */
  	  	  	  	  	  	  	  	  /* refer from LIGHT SENSE example*/
/* PRS */
#define PRS_CHANNEL            0

#define LIGHTSENSE_NUMOF_EVENTS 1 /* PCNT  PCNT_TOP*/


/******************************************************************//**
 * Macro Definition for LESENSE Module For LC Sensor
 *********************************************************************/
#define LC_SENSE 		7 //Port C ,Pin 6 ,Input pin for Controller
//#define LIGHT_EXCITE	6 //Port D ,Pin 6 ,Output pin for Controller
#define LED_PIN			1 //Port E ,Pin 2 ,Output pin

//#define LCSENSE_SCAN_FREQ         20
#define LC_INTERRUPT	    LESENSE_IF_CH7

/* LESENSE for IR Sensor*/
#define LCSENSE_CH             7


/* ACMP */
//#define ACMP_NEG_REF           acmpChannelVDD
#define ACMP_THRESHOLD_LC      0x38                         /* Reference value for the lightsensor.
                                                             * Value works well in office light
                                                             * conditions. Might need adjustment
                                                             * for other conditions. */
  	  	  	  	  	  	  	  	  /* refer from LIGHT SENSE example*/
/* PRS */
//#define PRS_CHANNEL            0

#define LC_NUMOF_EVENTS			 1 /* PCNT  PCNT_TOP*/


/* DAC */
#define DAC_FREQ               500000
#define DAC_CHANNEL            1
#define DAC_DATA               800





/******************************************************************//**
 * Macro Definition for LED
 *********************************************************************/

#define LED_GPIO_PORT gpioPortE
#define LED_GPIO_PIN  2


/******************************************************************//**
 * Macro Definition for BLE
 *********************************************************************/


/* BLE Command macro definition */
#define BLE_Command_1	"+++\n" // TO shift to
#define BLE_Command_2	"AT+HWMODELED=DISABLE\n" // To Disable LED
#define BLE_Command_3	"AT+BLEPOWERLEVEL=-8\n"// TO Set Power of Device
#define BLE_Command_4	"AT+GAPINTERVALS=50,3980,10240,50\n"//To set the intervals and continue working
#define BLE_Command_6	"ATZ\n"//TO query intervals
#define BLECommand false



/******************************************************************//**
 * Macro Definition for I2C
 *********************************************************************/

#define HMC_ADDRESS       	  0x1E << 1   // This shift is important!
#define DEVICE_ID             0x10
#define CMD_ARRAY_SIZE        1
#define DATA_ARRAY_SIZE       10

#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02
#define DataRegisterBegin 0x03

#define Measurement_Continuous 0x00
#define Measurement_SingleShot 0x01
#define Measurement_Idle 0x03

#define MaxMagLenght 3

//Limits for each Axis
short int LowerXLimit;// Lower Limit of X-axis (HMC)
short int UpperXLimit;// Lower Limit of X-axis (HMC)
short int LowerYLimit;// Lower Limit of Y-axis (HMC)
short int UpperYLimit;// Lower Limit of Y-axis (HMC)
short int LowerZLimit;// Lower Limit of Z-axis (HMC)
short int UpperZLimit;// Lower Limit of Z-axis (HMC)

// Globals for persistent storage
uint8_t cmd_array[CMD_ARRAY_SIZE];
uint8_t data_array[DATA_ARRAY_SIZE];






/******************************************************************//**
 * Macro Definition for LC using LESENSE
 *********************************************************************/

/* LESENSE */
#define LCSENSE_CH             7
#define LCSENSE_CH_PORT        gpioPortC
#define LCSENSE_CH_PIN         7
#define LCSENSE_SCAN_FREQ      20

/* ACMP */
#define ACMP_NEG_REF           acmpChannelVDD
#define ACMP_VDD_SCALE         0x0D                         /* reference for the LC sensor to be
                                                             * close to the DAC voltage
                                                             * This was calibrated using a scope
                                                             * and probing both the LC sensor and
                                                             * the ACMP output */



#define LESENSE_MAX_CHANNELS 	16
#define  BUFFER_INDEX_LAST  	15

/* DAC */
#define DAC_FREQ               500000
#define DAC_CHANNEL            1
#define DAC_DATA               800



/******************************************************************//**
 * Macro (typedef )Definition for LESENSE
 *********************************************************************/


/** Configuration for LC Sensor Sense channels.Refering App Note for LCSENSE */
#define LESENSE_LCSENSE_SENSOR_CH_CONF                     \
{                                                  \
  true,                      /* Enable scan channel. */    \
  true,                      /* Enable the assigned pin on scan channel. */ \
  true,                      /* Enable interrupts on channel. */ \
  lesenseChPinExLow,         /* GPIO pin is low during the excitation period. */    \
  lesenseChPinIdleDis,       /* GPIO pin is low during the idle period. */ \
  false,                     /* Use alternate excitation pins for excitation. */    \
  false,                     /* Disabled to shift results from this channel to the decoder register. */ \
  false,                     /* Disabled to invert the scan result bit. */  \
  true,                      /* Enabled to store counter value in the result buffer. */   \
  lesenseClkHF,              /* Use the LF clock for excitation timing. */    \
  lesenseClkLF,              /* Use the LF clock for sample timing. */ \
  0x0F,                      /* Excitation time is set to 1(+1) excitation clock cycles. */    \
  0x02,                      /* Sample delay is set to 1(+1) sample clock cycles. */ \
  0x00,                      /* Measure delay is set to 0 excitation clock cycles.*/    \
  0x0D,         			 /* ACMP threshold has been set to 0x38. */ \
  lesenseSampleModeCounter,     /* ACMP will be used in comparison.,Sampling acmp, not counting */    \
  /*lesenseSetIntLevel ,     Interrupt is generated if the sensor triggers,Interrupt when voltage falls below threshold. */ \
  lesenseSetIntPosEdge,/* */\
  0x0000,                    /* Counter threshold has been set to 0x00. */    \
  lesenseCompModeLess         /*Compare mode has been set to trigger interrupt on "less". */\
/*lesenseCompModeGreaterOrEq	/*Compare mode has been set to trigger interrupt on "less". */\
}


/** Configuration for ambient light sense channels. */
#define LESENSE_LIGHTSENSE_SENSOR_CH_CONF                     \
{                                                  \
  true,                      /* Enable scan channel. */    \
  false,                     /* Enable the assigned pin on scan channel. */ \
  true,                      /* Enable interrupts on channel. */ \
  lesenseChPinExHigh,        /* GPIO pin is high during the excitation period. */    \
  lesenseChPinIdleDis,       /* GPIO pin is low during the idle period. */ \
  true,                      /* Use alternate excitation pins for excitation. */    \
  false,                     /* Disabled to shift results from this channel to the decoder register. */ \
  false,                     /* Disabled to invert the scan result bit. */  \
  true,                      /* Enabled to store counter value in the result buffer. */   \
  lesenseClkLF,              /* Use the LF clock for excitation timing. */    \
  lesenseClkLF,              /* Use the LF clock for sample timing. */ \
  0x01,                      /* Excitation time is set to 1(+1) excitation clock cycles. */    \
  0x01,                      /* Sample delay is set to 1(+1) sample clock cycles. */ \
  0x00,                      /* Measure delay is set to 0 excitation clock cycles.*/    \
  ACMP_THRESHOLD,            /* ACMP threshold has been set to 0x38. */ \
  lesenseSampleModeACMP,     /* ACMP will be used in comparison.,Sampling acmp, not counting */    \
  lesenseSetIntNegEdge,      /* Interrupt is generated if the sensor triggers,Interrupt when voltage falls below threshold. */ \
  0x0000,                    /* Counter threshold has been set to 0x00. */    \
  lesenseCompModeLess        /* Compare mode has been set to trigger interrupt on "less". */ \
}



/** Configuration for disabled channels. */
#define LESENSE_DISABLED_CH_CONF                     \
{                                                  \
  false,                     /* Disable scan channel. */    \
  false,                     /* Disable the assigned pin on scan channel. */ \
  false,                     /* Disable interrupts on channel. */ \
  lesenseChPinExDis,         /* GPIO pin is disabled during the excitation period. */    \
  lesenseChPinIdleDis,       /* GPIO pin is disabled during the idle period. */ \
  false,                     /* Don't use alternate excitation pins for excitation. */    \
  false,                     /* Disabled to shift results from this channel to the decoder register. */ \
  false,                     /* Disabled to invert the scan result bit. */  \
  false,                     /* Disabled to store counter value in the result buffer. */   \
  lesenseClkLF,              /* Use the LF clock for excitation timing. */    \
  lesenseClkLF,              /* Use the LF clock for sample timing. */ \
  0x00U,                     /* Excitation time is set to 5(+1) excitation clock cycles. */    \
  0x00U,                     /* Sample delay is set to 7(+1) sample clock cycles. */ \
  0x00U,                     /* Measure delay is set to 0 excitation clock cycles.*/    \
  0x00U,                     /* ACMP threshold has been set to 0. */ \
  lesenseSampleModeCounter,  /* ACMP output will be used in comparison. */    \
  lesenseSetIntNone,         /* No interrupt is generated by the channel. */ \
  0x00U,                     /* Counter threshold has been set to 0x01. */    \
  lesenseCompModeLess        /* Compare mode has been set to trigger interrupt on "less". */ \
}


#define LESENSE_LIGHTSENSE_SCAN_CONF                                         \
{                                                                            \
  {                                                                          \
    LESENSE_DISABLED_CH_CONF,          /* Channel 0. */                      \
    LESENSE_DISABLED_CH_CONF,          /* Channel 1. */                      \
    LESENSE_DISABLED_CH_CONF,          /* Channel 2. */                      \
    LESENSE_DISABLED_CH_CONF,          /* Channel 3. */                      \
    LESENSE_DISABLED_CH_CONF,          /* Channel 4. */                      \
    LESENSE_DISABLED_CH_CONF,          /* Channel 5. */                      \
    /*LESENSE_LIGHTSENSE_SENSOR_CH_CONF,  Channel 6. */                      \
	LESENSE_DISABLED_CH_CONF,          /* Channel 6. */                      \
	LESENSE_LCSENSE_SENSOR_CH_CONF,    /* Channel 7. */                      \
    LESENSE_DISABLED_CH_CONF,          /* Channel 8. */                      \
    LESENSE_DISABLED_CH_CONF,          /* Channel 9. */                      \
    LESENSE_DISABLED_CH_CONF,          /* Channel 10. */                     \
    LESENSE_DISABLED_CH_CONF,          /* Channel 11. */                     \
    LESENSE_DISABLED_CH_CONF,          /* Channel 12. */                     \
    LESENSE_DISABLED_CH_CONF,          /* Channel 13. */                     \
    LESENSE_DISABLED_CH_CONF,          /* Channel 14. */                     \
    LESENSE_DISABLED_CH_CONF           /* Channel 15. */                     \
  }                                                                          \
}

/** Default configuration for alternate excitation channel. */
#define LESENSE_LIGHTSENSE_ALTEX_CH_CONF                                       \
{                                                                              \
  true,                   /* Alternate excitation enabled.*/                  \
  lesenseAltExPinIdleDis, /* Alternate excitation pin is low in idle. */ \
  true                    /* Excite only for corresponding channel. */        \
}

/** Default configuration for alternate excitation channel. */
#define LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF                                   \
{                                                                              \
  false,                   /* Alternate excitation enabled.*/                  \
  lesenseAltExPinIdleDis,  /* Alternate excitation pin is disabled in idle. */ \
  false                    /* Excite only for corresponding channel. */        \
}

/** Default configuration for all alternate excitation channels. */
#define LESENSE_LIGHTSENSE_ALTEX_CONF                                          \
{                                                                              \
  lesenseAltExMapALTEX,                                                         \
  {                                                                            \
    LESENSE_LIGHTSENSE_ALTEX_CH_CONF,     /* Alternate excitation channel 0. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 1. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 2. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 3. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 4. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 5. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF, /* Alternate excitation channel 6. */\
    LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF  /* Alternate excitation channel 7. */\
  }                                                                            \
}


/******************************************************************//**
 * Macro (typedef )Definition for EEPROM
 *********************************************************************/


#define PAGE_SIZE                        0x200


#define DEFAULT_NUMBER_OF_PAGES          2
#define MAX_NUMBER_OF_PAGES              (FLASH_SIZE/PAGE_SIZE)

#define SIZE_OF_DATA                     2                                        /* 2 bytes */
#define SIZE_OF_VIRTUAL_ADDRESS          2                                        /* 2 bytes */
#define SIZE_OF_VARIABLE                 (SIZE_OF_DATA + SIZE_OF_VIRTUAL_ADDRESS) /* 4 bytes */

#define MAX_ACTIVE_VARIABLES             (PAGE_SIZE / SIZE_OF_VARIABLE)

#define PAGE_STATUS_ERASED               0xFF
#define PAGE_STATUS_RECEIVING            0xAA
#define PAGE_STATUS_ACTIVE               0x00



#define VECTOR_SIZE (16+30)
/* Align the RAM vector table */
#if defined (__ICCARM__)
#pragma data_alignment=256
uint32_t vectorTable[VECTOR_SIZE];
#elif defined (__CC_ARM)
uint32_t vectorTable[VECTOR_SIZE] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
uint32_t vectorTable[VECTOR_SIZE] __attribute__ ((aligned(256)));
#else
#error "Undefined toolkit, need to define alignment"
#endif



/* SysTick interrupt handler. Counts ms ticks */
volatile uint32_t msTicks = 0;


/* Place interrupt handler in RAM */
#if defined (__ICCARM__)               			/* IAR compiler */
__ramfunc
#elif defined(__CC_ARM)
 __attribute__ ((section ("ram_code")))
#elif defined(__GNUC__)                  		/* GCC based compilers */
#if defined (__CROSSWORKS_ARM)         			/* Rowley Crossworks */
__attribute__ ((section(".fast")))
#else                           						/* Sourcery G++ */
__attribute__ ((section(".ram")))
#endif
#endif



/******************************************************************//**
 * Global Variables
 *********************************************************************/
uint8_t tempNumber = 0x41;
uint8_t testChar = 0x08;
char rxData[MAX_MESSAGE];
volatile unsigned char j=0,k=0,r=0;
//volatile bool result=0;

//For Magnetometer
int16_t XVal = 0, YVal = 0, ZVal = 0;

char Command0[MAX_COMMAND] ="RetTemp";
char Command1[MAX_COMMAND] ="RetMag";
char Error1[MAX_MESSAGE] ="\nError Command!";
char Message[4*MAX_MESSAGE]; //For Blank Message
char Above_limit[]="\nTemperature ABOVE set minimum=";
char Below_limit[]="\nTemperature BELOW set minimum=";
char Temperature[]="\nXX.X C";
char dark_Room[MAX_MESSAGE]="\nRoom is dark";
char FloattoString[TEMP_SIZE];
char AxistoString[TEMP_SIZE];
//Messages for Magnetometer
char XMagMessage[]="\nX:";
char YMagMessage[]="\nY:";
char ZMagMessage[]="\nZ:";
char MagMessage[4*MAX_MESSAGE];
char Magbelow[]="axis Inclination below minimum set=";
char MagAbove[]="axis Inclination above maximum set=";
char Success[]="Done\n";

char MinLimit[4]={'\0','\0','\0','\0'},MaxLimit[4]={'\0','\0','\0','\0'};

float tempComp =(UpperTempLimit+LowerTempLimit)/2;// Initial Value as Mid value

//For Comparator


//For DMA
//Structure define for call back function
DMA_CB_TypeDef cb[DMA_CHANNELS];

///uint16_t trfComplete =false;

//Buffer for Data Memory
volatile uint16_t BufferAdcData[NumberOfSamples];


//EEPROM EMulation


/* Since the data to be written to flash must be read from ram, the data used to
 * set the pages' status, is explicitly written to the ram beforehand. */
static uint32_t pageStatusActiveValue    = ((uint32_t) PAGE_STATUS_ACTIVE << 24) | 0x00FFFFFF;
static uint32_t pageStatusReceivingValue = ((uint32_t) PAGE_STATUS_RECEIVING << 24) | 0x00FFFFFF;


/* Variables to keep track of what pages are active and receiving. */
static int  activePageNumber    = -1;
static int  receivingPageNumber = -1;
static bool initialized = false;

InterruptIn button(PC0);


/*******************************************************************************
 ******************************   STRUCTS   ************************************
 ******************************************************************************/

typedef struct
{
  /* Each variable is assigned a unique virtual address automatically when first
   * written to, or when using the declare function. */
  uint16_t virtualAddress;
} EE_Variable_TypeDef;

typedef struct
{
  uint32_t *startAddress;
  uint32_t *endAddress;
} EE_Page_TypeDef;



typedef enum
{
  eePageStatusErased    = PAGE_STATUS_ERASED,
  eePageStatusReceiving = PAGE_STATUS_RECEIVING,
  eePageStatusActive    = PAGE_STATUS_ACTIVE,
} EE_PageStatus_TypeDef;


/* Define the non-volatile variables. */
EE_Variable_TypeDef var1, var2, var3, boolVar;
EE_Variable_TypeDef  Tempmax,Tempmin,MagXmin,MagXmax,MagYmin,MagYmax,MagZmin,MagZmax,LCMax;//,MaxLimit,MinLimit;
uint16_t             readValue;

/* Array of all pages allocated to the eeprom */
static EE_Page_TypeDef pages[MAX_NUMBER_OF_PAGES];
static int16_t         numberOfVariablesDeclared = 0;
static int16_t         numberOfActiveVariables = 0;
static int16_t         numberOfPagesAllocated;

/**************************************************************************//**
 * Interrupt handlers prototypes
 *****************************************************************************/
void LESENSE_IRQHandler(void);

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/
//EEPROM
bool EE_Init(uint32_t);
bool EE_Format(uint32_t);
bool EE_Read(EE_Variable_TypeDef *var, uint16_t *readData);
void EE_Write(EE_Variable_TypeDef *var, uint16_t writeData);
bool EE_DeclareVariable(EE_Variable_TypeDef *var);
void EE_DeleteVariable(EE_Variable_TypeDef *var);
uint32_t EE_GetEraseCount(void);


void BSP_TraceSwoSetup(void);
//LEUART
void LEUART_Initialization(void);//Initializing LEUART
void GPIO_Initialization(void);//Initializing GPIO

//BLE COMOMANDS
void CheckCommand(void);//Checking Command received


//MISC
bool CompareString (char Str1[],char Str2[]);//Compare String and return result
//void StringCopy(volatile char Destination[],char Source[]); // Copy String from one string to another
void ConvertFloattoString(float Number);
void ConvertToTemp(char Str[]);
void ConvertInttoString(int Number);
unsigned char typeCommand( char compareCommand[]);

//Initialization
void ADC_Initialization(void);
void DMA_Initialization(void);
void DMARX_Initialization(void);
void DMATX_Initialization(void);
void DMAADC_Initialization(void);
void LESENSE_Initialization(void);
void LETIMER_Initialization(void);


//Call Back
void TransferADCComplete(unsigned int channel, bool primary, void *user);//call back function
void TempCalculationAndDisplay();//To calculate the average and display on Led 0 or Led 1


//HMC
void HMCAxisLimitCheck();





/**************************************************************************//**
 * Interrupt handlers prototypes
 *****************************************************************************/
void LESENSE_IRQHandler(void);
//void RTC_IRQHandler(void);

/**************************************************************************//**
 * Functions prototypes
 *****************************************************************************/
void writeDataDAC(DAC_TypeDef *dac, unsigned int value, unsigned int ch);
void setupDAC(void);
void setupCMU(void);
void setupACMP(void);
void setupLESENSE(void);
void setupGPIO(void);
void setupRTC(void);


/******************************************************************//**
 * Calibrating LESENSE for LC
 *********************************************************************/
void lesenseCalibrateLC(uint8_t chIdx)
{
  uint8_t i;

  /* Enable scan and pin on selected channel */
  LESENSE_ChannelEnable(chIdx, true, true);

  /* Disable scan and pin on all other channels */
  for(i=0; i<LESENSE_MAX_CHANNELS; i++)
  {
    if(i!=chIdx)
        LESENSE_ChannelEnable(i, false, false);
  }

  /* Start scan. */
  LESENSE_ScanStart();

  /* Waiting for buffer to be full */
  while(!(LESENSE->STATUS & LESENSE_STATUS_BUFFULL));

  uint32_t calibValue = LESENSE_ScanResultDataBufferGet(BUFFER_INDEX_LAST);
  /*  Use last result as counter threshold */
  LESENSE_ChannelThresSet(chIdx, 0, calibValue);

  /* Stop scan. */
  LESENSE_ScanStop();

  /* Clear result buffer */
  LESENSE_ResultBufferClear();
}






/**************************************************************************//**
 * Functions
 *****************************************************************************/
/**************************************************************************//**
 * @brief  Enable clocks for all the peripherals to be used
 *****************************************************************************/
void setupCMU(void)
{
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* DAC */
  CMU_ClockEnable(cmuClock_DAC0, true);

  /* ACMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);

  /* GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

/* Low energy peripherals
 *   LESENSE
 *   LFRCO clock must be enables prior to enabling
 *   clock for the low energy peripherals */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LESENSE, true);

  /* RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Disable clock source for LFB clock*/
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);
}


/**************************************************************************//**
 * @brief  Sets up the DAC
 *****************************************************************************/
void setupDAC(void)
{
  /* Configuration structure for the DAC */
  static const DAC_Init_TypeDef dacInit =
  {
    .refresh      = dacRefresh8,
    .reference    = dacRefVDD,
    .outMode      = dacOutputPin,
    .convMode     = dacConvModeContinuous,
    .prescale     = 0,
    .lpEnable     = false,
    .ch0ResetPre  = false,
    .outEnablePRS = false,
    .sineEnable   = false,
    .diff         = false
  };

  /* Initialize DAC */
  DAC_Init(DAC0, &dacInit);

  /* Set data for DAC channel 0 */
  writeDataDAC(DAC0, (unsigned int) DAC_DATA, DAC_CHANNEL);
}

/**************************************************************************//**
 * @brief  Write DAC conversion value
 *****************************************************************************/
void writeDataDAC(DAC_TypeDef *dac, unsigned int value, unsigned int ch)
{
  /* Write data output value to the correct register. */
  if (!ch)
  {
    /* Write data to DAC ch 0 */
    dac->CH0DATA = value;
  }
  else
  {
    /* Write data to DAC ch 1 */
    dac->CH1DATA = value;
  }
}

/**************************************************************************//**
 * @brief  Sets up the ACMP
 *****************************************************************************/
void setupACMP(void)
{
  /* Configuration structure for ACMP */
  static const ACMP_Init_TypeDef acmpInit =
  {
    .fullBias                 = false,
    .halfBias                 = true,
    .biasProg                 = 0xE,
    .interruptOnFallingEdge   = false,
    .interruptOnRisingEdge    = false,
    .warmTime                 = acmpWarmTime4,
    .hysteresisLevel          = acmpHysteresisLevel0,
    .inactiveValue            = false,
    .lowPowerReferenceEnabled = false,
    .vddLevel                 = ACMP_VDD_SCALE,
    .enable                   = false
  };

  /* Initialize ACMP */
  ACMP_Init(ACMP0, &acmpInit);

  /* Select Vdd as negative reference
   * Positive reference is controlled by LESENSE */
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel7);
}

/**************************************************************************//**
 * @brief  Sets up the LESENSE
 *****************************************************************************/
void setupLESENSE(void)
{
  /* LESENSE configuration structure */
  static const LESENSE_Init_TypeDef initLesense =
  {
    .coreCtrl         =
    {
      .scanStart    = lesenseScanStartPeriodic,
      .prsSel       = lesensePRSCh0,
      .scanConfSel  = lesenseScanConfDirMap,
      .invACMP0     = false,
      .invACMP1     = false,
      .dualSample   = false,
      .storeScanRes = false,
      .bufOverWr    = true,
      .bufTrigLevel = lesenseBufTrigHalf,
      .wakeupOnDMA  = lesenseDMAWakeUpDisable,
      .biasMode     = lesenseBiasModeDutyCycle,
      .debugRun     = false
    },

    .timeCtrl         =
    {
      .startDelay     = 0
    },

    .perCtrl          =
    {
      .dacCh0Data     = lesenseDACIfData,
      .dacCh0ConvMode = lesenseDACConvModeSampleOff,
      .dacCh0OutMode  = lesenseDACOutModeDisable,
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeSampleOff,
      .dacCh1OutMode  = lesenseDACOutModePin,
      .dacPresc       = 31,
      .dacRef         = lesenseDACRefVdd,
      .acmp0Mode      = lesenseACMPModeMux,
      .acmp1Mode      = lesenseACMPModeDisable,
      .warmupMode     = lesenseWarmupModeNormal
    },

    .decCtrl          =
    {
      .decInput  = lesenseDecInputSensorSt,
      .initState = 0,
      .chkState  = false,
      .intMap    = false,
      .hystPRS0  = false,
      .hystPRS1  = false,
      .hystPRS2  = false,
      .hystIRQ   = false,
      .prsCount  = true,
      .prsChSel0 = lesensePRSCh0,
      .prsChSel1 = lesensePRSCh1,
      .prsChSel2 = lesensePRSCh2,
      .prsChSel3 = lesensePRSCh3
    }
  };

  /* Channel configuration */
  static const LESENSE_ChDesc_TypeDef initLesenseCh7 =
  {
    .enaScanCh     = true,
    .enaPin        = true,
    .enaInt        = true,
    .chPinExMode   = lesenseChPinExLow,
    .chPinIdleMode = lesenseChPinIdleDis,
    .useAltEx      = false,
    .shiftRes      = false,
    .invRes        = false,
    .storeCntRes   = true,
    .exClk         = lesenseClkHF,
    .sampleClk     = lesenseClkLF,
    .exTime        = 0x07,
    .sampleDelay   = 0x01,
    .measDelay     = 0x00,
    .acmpThres     = 0x00,
    .sampleMode    = lesenseSampleModeCounter,
    .intMode       = lesenseSetIntPosEdge,
    .cntThres      = 0x0000,
    .compMode      = lesenseCompModeLess
  };

  /* Initialize LESENSE interface _with_ RESET. */
  LESENSE_Init(&initLesense, true);

  /* Configure channel 7 */
  LESENSE_ChannelConfig(&initLesenseCh7, LCSENSE_CH);

  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, LCSENSE_SCAN_FREQ);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_2);
  /* Set clock divisor for HF clock. */
  LESENSE_ClkDivSet(lesenseClkHF, lesenseClkDiv_1);

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);

  /* Calibrate LC sensor */
  lesenseCalibrateLC(LCSENSE_CH);

  /* Start scan. */
  LESENSE_ScanStart();
}

/**************************************************************************//**
 * @brief  Sets up the GPIO
 *****************************************************************************/
void setupGPIO(void)
{
  /* Configure excitation/measure pin as push pull */
  GPIO_PinModeSet(LCSENSE_CH_PORT, LCSENSE_CH_PIN, gpioModePushPull, 0);

  /* Configure user led as output */
  //GPIO_PinModeSet(LED_GPIO_PORT, LED_GPIO_PIN, gpioModePushPull, 0);
}
//fine





//EEPROM functions
//Leveraged from EEPROM Application note and changes made for Energy by chinmay.shah@colorado,edu

/***************************************************************************//**
 * @brief
 *   Returns the page status of the given page.
 *
 * @param[in] page
 *   Pointer to the page whose status to be returned.
 *
 * @return
 *   Returns the status of the given page.
 ******************************************************************************/
EE_PageStatus_TypeDef EE_getPageStatus(EE_Page_TypeDef *page)
{
  return (EE_PageStatus_TypeDef)((*(page->startAddress) >> 24) & 0xFF);
}

/***************************************************************************//**
 * @brief
 *   Sets status of the given page to active.
 *
 * @param[in] page
 *   Pointer to the page whose status to be altered.
 *
 * @return
 *   Returns the status of the flash operation.
 ******************************************************************************/
 msc_Return_TypeDef EE_setPageStatusActive(EE_Page_TypeDef *page)
{
  return MSC_WriteWord(page->startAddress, &pageStatusActiveValue, SIZE_OF_VARIABLE);
}

/***************************************************************************//**
 * @brief
 *   Sets status of the given page to receiving.
 *
 * @param[in] page
 *   Pointer to the page whose status to be altered.
 *
 * @return
 *   Returns the status of the flash operation.
 ******************************************************************************/
msc_Return_TypeDef EE_setPageStatusReceiving(EE_Page_TypeDef *page)
{
  return MSC_WriteWord(page->startAddress, &pageStatusReceivingValue, SIZE_OF_VARIABLE);
}


/***************************************************************************//**
* @brief
*   Checks if all the bits in the page are 1's.
*
* @param[in]
*   Pointer to the page that is to be validated.
*
* @return
*   Returns the result of the check.
*
* @verbatim
*   true - All bits in the page are 1's.
*   false - One or more bits in the page are 0's.
* @endverbatim
*******************************************************************************/
static bool EE_validateIfErased(EE_Page_TypeDef *page)
{
  uint32_t *address = page->startAddress;

  /* Iterate through all the words of the page, and validate that all bits are set. */
  while (address <= page->endAddress)
  {
    if (*address != 0xFFFFFFFF)
    {
      /* 0 bit detected */
      return false;
    }
    address++;
  }
  /* All bits are 1's. */
  return true;
}


/***************************************************************************//**
 * @brief
 *   Writes the desired data to the specified page.
 *
 * @param[in] page
 *   Pointer to the page to write variable to.
 *
 * @param[in] virtualAddress
 *   The virtual address of the variable to be written.
 *
 * @param[in] writeData
 *   The data to be associated with the given virtual address.
 *
 * @return
 *   Returns whether the write has been a success or not. In normal operation
 *   a failed write indicates that the currently active page is full.
 ******************************************************************************/
static bool EE_WriteToPage(EE_Page_TypeDef *page, uint16_t virtualAddress, uint16_t writeData)
{
  /* Start at the second word. The fist one is reserved for status and erase count. */
  uint32_t *address = page->startAddress + 1;
  uint32_t virtualAddressAndData;

  /* Iterate through the page from the beginning, and stop at the fist empty word. */
  while (address <= page->endAddress)
  {
    /* Empty word found. */
    if (*address == 0xFFFFFFFF)
    {
      /* The virtual address and data is combined to a full word. */
      virtualAddressAndData = ((uint32_t)(virtualAddress << 16) & 0xFFFF0000) | (uint32_t)(writeData);

      /* Make sure that the write to flash is a success. */
      if (MSC_WriteWord(address, &virtualAddressAndData, SIZE_OF_VARIABLE) != mscReturnOk)
      {
        /* Write failed. Halt for debug trace, if enabled. */
        EFM_ASSERT(0);
        return false;
      }
      /* Data written successfully. */
      return true;
    }
    else
    {
      address++;
    }
  }
  /* Reached the end of the page without finding any empty words. */
  return false;
}


/***************************************************************************//**
 * @brief
 *   Erase all pages allocated to the eeprom emulator, and force page 0 to be
 *   the active page.
 *
 * @return
 *   Returns true if the format was successful.
 ******************************************************************************/
bool EE_Format(uint32_t numberOfPages)
{
  uint32_t eraseCount = 0xFF000001;
  int i;
  msc_Return_TypeDef retStatus;

  /* Make the number of pages allocated accessible throughout the file. */
  numberOfPagesAllocated = numberOfPages;

  /* Initialize the address of each page */
  for (i = 0; i < numberOfPagesAllocated; i++)
  {
    pages[i].startAddress = (uint32_t *)(FLASH_SIZE - i * PAGE_SIZE - PAGE_SIZE);
    pages[i].endAddress   = (uint32_t *)(FLASH_SIZE - i * PAGE_SIZE - 4);
  }

  /* Erase all pages allocated to the eeprom emulator*/
  for (i = numberOfPagesAllocated - 1; i >= 0; i--)
  {
    /* Validate if the page is already erased, and erase it if not. */
    if (!EE_validateIfErased(&pages[i]))
    {
      /* Erase the page, and return the status if the erase operation is unsuccessful. */
      retStatus = MSC_ErasePage(pages[i].startAddress);
      if (retStatus != mscReturnOk) {
        return false;
      }
    }
  }

  /* Page 0 is the active page. */
  activePageNumber = 0;

  /* There should be no receiving page. */
  receivingPageNumber = -1;

  /* Write erase count of 1 to the page 0 head. */
  retStatus = MSC_WriteWord(pages[activePageNumber].startAddress, &eraseCount, 4);
  if (retStatus != mscReturnOk) {
    return false;
  }

  /* Set page status active to page 0. */
  retStatus = EE_setPageStatusActive(&pages[activePageNumber]);
  if ( retStatus != mscReturnOk ) {
    return false;
  }

  /** Successfully formatted pages */
  return true;
}


/***************************************************************************//**
 * @brief
 *   Transfers the most recently written value of each variable, from the active
 *   to a new receiving page.
 *
 * @param[in] var
 *   Pointer to a variable whose data will be written to the first free word of
 *   the receiving page. If var is a null pointer, this operation will be
 *   skipped.
 *
 * @param[in] writeData
 *   Data to be associated with var's virtual address, if var is not a null
 *   pointer.
 *
 * @return
 *   Returns the status of the last flash operation.
 ******************************************************************************/
static msc_Return_TypeDef EE_TransferPage(EE_Variable_TypeDef *var, uint16_t writeData)
{
  msc_Return_TypeDef retStatus;
  uint32_t           *activeAddress;
  uint32_t           *receivingAddress;
  bool               newVariable;
  uint32_t           eraseCount;

  /* If there is no receiving page predefined, set it to cycle through all allocated pages. */
  if (receivingPageNumber == -1)
  {
    receivingPageNumber = activePageNumber + 1;

    if (receivingPageNumber >= numberOfPagesAllocated) {
      receivingPageNumber = 0;
    }

    /* Check if the new receiving page really is erased. */
    if (!EE_validateIfErased(&pages[receivingPageNumber]))
    {
      /* If this page is not truly erased, it means that it has been written to
       * from outside this API, this could be an address conflict. */
      EFM_ASSERT(0);
      MSC_ErasePage(pages[receivingPageNumber].startAddress);
    }
  }

  /* Set the status of the receiving page */
  EE_setPageStatusReceiving(&pages[receivingPageNumber]);

  /* If a variable was specified, write it to the receiving page */
  if (var != NULL)
  {
    EE_WriteToPage(&pages[receivingPageNumber], var->virtualAddress, writeData);
  }

  /* Start at the last word. */
  activeAddress = pages[activePageNumber].endAddress;

  /* Iterate through all words in the active page. Each time a new virtual
   * address is found, write it and it's data to the receiving page */
  while (activeAddress > pages[activePageNumber].startAddress)
  {
    /* 0x0000 and 0xFFFF are not valid virtual addresses. */
    if ((uint16_t)(*activeAddress >> 16) == 0x0000 || (uint16_t)(*activeAddress >> 16) == 0xFFFF)
    {
      newVariable = false;
    }
    /* Omit when transfer is initiated from inside the EE_Init() function. */
    else if (var != NULL && (uint16_t)(*activeAddress >> 16) > numberOfVariablesDeclared)
    {
      /* A virtual address outside the virtual address space, defined by the
       * number of variables declared, are considered garbage. */
      newVariable = false;
    }

    else
    {
      receivingAddress = pages[receivingPageNumber].startAddress + 1;

      /* Start at the beginning of the receiving page. Check if the variable is
       * already transfered. */
      while (receivingAddress <= pages[receivingPageNumber].endAddress)
      {
        /* Variable found, and is therefore already transferred. */
        if ((uint16_t)(*activeAddress >> 16) == (uint16_t)(*receivingAddress >> 16))
        {
          newVariable = false;
          break;
        }
        /* Empty word found. All transferred variables are checked.  */
        else if (*receivingAddress == 0xFFFFFFFF)
        {
          newVariable = true;
          break;
        }
        receivingAddress++;
      }
    }

    if (newVariable)
    {
      /* Write the new variable to the receiving page. */
      EE_WriteToPage(&pages[receivingPageNumber], (uint16_t)(*activeAddress >> 16), (uint16_t)(*activeAddress));
    }
    activeAddress--;
  }

  /* Update erase count */
  eraseCount = EE_GetEraseCount();

  /* If a new page cycle is started, increment the erase count. */
  if (receivingPageNumber == 0)
    eraseCount++;

  /* Set the first byte, in this way the page status is not altered when the erase count is written. */
  eraseCount = eraseCount | 0xFF000000;

  /* Write the erase count obtained to the active page head. */
  retStatus = MSC_WriteWord(pages[receivingPageNumber].startAddress, &eraseCount, 4);
  if (retStatus != mscReturnOk) {
    return retStatus;
  }

  /* Erase the old active page. */
  retStatus = MSC_ErasePage(pages[activePageNumber].startAddress);
  if (retStatus != mscReturnOk) {
    return retStatus;
  }

  /* Set the receiving page to be the new active page. */
  retStatus = EE_setPageStatusActive(&pages[receivingPageNumber]);
  if (retStatus != mscReturnOk) {
    return retStatus;
  }

  activePageNumber    = receivingPageNumber;
  receivingPageNumber = -1;

  return mscReturnOk;
}

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *    Should be run once before any other eeprom emulator functions are called.
 *    It restores the pages to a known good state in case of page status
 *    corruption after a power loss or another unwanted system reset.
 *
 * @param[in] numberOfPages
 *   Number of pages to be allocated to the eeprom emulator for the entire
 *   lifetime of the application. Must be 2 or greater.
 *
 * @note
 *   When choosing the number of pages to assign to the eeprom emulator, it is
 *   highly recommended to have a good insight into how much data that will be
 *   written to the eeprom emulator during the application lifetime. Using an
 *   estimate of this data, and the maximum number of rewrite cycles the flash
 *   is guaranteed to endure, it should be easy to get a good idea of how many
 *   pages that are needed to keep flash wear within the recommended level
 *   throughout the application lifetime.
 *   It should also be noted that it is critical to have control over what
 *   areas of the flash that is in use by other parts of the application, to
 *   avoid possible conflicts and hard-to-find bugs.
 *
 * @return
 *   Returns true if the initialization was successful.
 ******************************************************************************/
bool EE_Init(uint32_t numberOfPages)
{
  /* Make sure that the eeprom emulator is only initialized once. More that one
   * initialization may result in undefined behavior. */
  EFM_ASSERT(!initialized);

  initialized = true;

  /* Number of pages must be at least 2. */
  if (numberOfPages < 2) {
    numberOfPages = DEFAULT_NUMBER_OF_PAGES;
  }

  /* Make the number of pages allocated accessible throughout the file. */
  numberOfPagesAllocated = numberOfPages;

  /* Initialize the address of each page */
  uint32_t i;
  for (i = 0; i < numberOfPages; i++)
  {
    pages[i].startAddress = (uint32_t *)(FLASH_SIZE - i * PAGE_SIZE - PAGE_SIZE);
    pages[i].endAddress   = (uint32_t *)(FLASH_SIZE - i * PAGE_SIZE - 4);
  }

  /* Check status of each page */
  for (i = 0; i < numberOfPages; i++)
  {
    switch (EE_getPageStatus(&pages[i]))
    {
    case eePageStatusActive:
      if (activePageNumber == -1)
      {
        activePageNumber = i;
      }
      else
      {
        /* More than one active page found. This is an invalid system state. */
        return false;
      }
      break;
    case eePageStatusReceiving:
      if (receivingPageNumber == -1)
      {
        receivingPageNumber = i;
      }
      else
      {
        /* More than one receiving page foudn. This is an invalid system state. */
        return false;
      }
      break;
    case eePageStatusErased:
      /* Validate if the page is really erased, and erase it if not. */
      if (!EE_validateIfErased(&pages[i]))
      {
        MSC_ErasePage(pages[i].startAddress);
      }
      break;
    default:
      /* Undefined page status, erase page. */
      MSC_ErasePage(pages[i].startAddress);
      break;
    }
  }

  /* No receiving or active page found. This is an invalid system state. */
  if (receivingPageNumber == -1 && activePageNumber == -1)
  {
    return false;
  }

  /* One active page only. All good. */
  if (receivingPageNumber == -1)
  {
    return true;
  }

  /* One receiving page only. */
  else if (activePageNumber == -1)
  {
    /* Set current receiving page as active. */
    activePageNumber    = receivingPageNumber;
    receivingPageNumber = -1;
    EE_setPageStatusActive(&pages[receivingPageNumber]);
  }
  /* Found exactly one active and one receiving page. */
  else
  {
    /* Transfer variables from active to receiving page. */
    EE_TransferPage(NULL, NULL);
  }

  /* Initialization completed successfully */
  return true;
}


/***************************************************************************//**
 * @brief
 *   Read the latest data associated with the given variable.
 *
 * @note
 *   If attempting to read from an undeclared variable, or a variable that has
 *   no valid value written to it, a null value will be returned.
 *
 * @param[in] var
 *   Pointer to the variable with the virtual address to look for.
 *
 * @param[out] readData
 *   Pointer to the memory area to write the read data to.
 *
 * @return
 *   Returns whether the variable given was found or not.
 ******************************************************************************/
bool EE_Read(EE_Variable_TypeDef *var, uint16_t *readData)
{
  /* Make sure that the eeprom emulator is initialized. */
  EFM_ASSERT(initialized);

  uint32_t *address;

  address = (pages[activePageNumber].endAddress);

  /* 0x0000 and 0xFFFF are illegal addresses. */
  if (var->virtualAddress != 0x0000 && var->virtualAddress != 0xFFFF)
  {
    /* Iterate through the active page, starting from the end. */
    while (address > pages[activePageNumber].startAddress)
    {
      /* Check if the stored virtual address matches the one wanted. */
      if ((uint16_t)(*address >> 16) == var->virtualAddress)
      {
        /* Correct virtual address found, return the corresponding data. */
        *readData = (uint16_t)(*address);
        return true;
      }
      address--;
    }
  }
  /* Variable not found, return null value. */
  *readData = 0x0000;
  return false;
}


/***************************************************************************//**
 * @brief
 *   Writes the desired data, together with the given variable's virtual address
 *   to the emulated eeprom memory.
 *
 * @param[in] var
 *   Pointer to the variable to associate data with.
 *
 * @param[in] writeData
 *   The 16 bit data to write to the flash memory. Any 16 bit data can be sent,
 *   as long as it is casted to a uint16_t type.
 ******************************************************************************/
void EE_Write(EE_Variable_TypeDef *var, uint16_t writeData)
{
  /* Make sure that the eeprom emulator is initialized. */
  EFM_ASSERT(initialized);

  /* Make sure that the virtual address is declared and valid. */
  EFM_ASSERT(var->virtualAddress <= numberOfVariablesDeclared);

  uint16_t readData;

  /* Check whether the variable already has a value associated to it. */
  if (EE_Read(var, &readData))
  {
    /* Do not write if data is duplicate. */
    if (readData == writeData)
    {
      return;
    }
  }
  /* Write to flash. */
  if (!EE_WriteToPage(&pages[activePageNumber], var->virtualAddress, writeData))
  {
    /* The write was not successful, which indicates that the active page is full. */
    EE_TransferPage(var, writeData);
  }
}


/***************************************************************************//**
* @brief
*   If the variable no longer is needed, it can be deleted to free flash space.
*
* @note
*   The function writes 0x0000 to the virtual address field of all previous
*   versions of the input variable. All stored data with virtual address 0x0000
*   is marked as garbage, which will be collected on the next page transfer.
*
* @param[in] var
*   Pointer to the variable to be deleted from the emulated eeprom.
*******************************************************************************/
void EE_DeleteVariable(EE_Variable_TypeDef *var)
{
  /* If the eeprom emulator is not initialized, this function has no meaning. */
  if (!initialized)
  {
    /* Halt for debug trace. */
    EFM_ASSERT(0);
    return;
  }

  uint32_t deleteData = 0x0000FFFF;

  uint32_t *address = (pages[activePageNumber].endAddress);

  /* Keep track if we actually removed a variable */
  bool varDeleted = false;

  /* Iterate through the active page from the end. */
  while (address > pages[activePageNumber].startAddress)
  {
    /* Write the virtual address 0x0000 to all instances of the chosen variable.
     * Since 0x0000 is not a valid virtual address, the variable will not
     * transferred to a new page on the next page transfer. */
    if ((uint16_t)(*address >> 16) == var->virtualAddress)
    {
      MSC_WriteWord(address, &deleteData, sizeof deleteData);
      varDeleted = true;
    }
    address--;
  }

  if (varDeleted) {
    numberOfActiveVariables--;
  }
}


/***************************************************************************//**
* @brief
*   Assign a virtual address to a new variable.
*
* @note
*   All variables that is to be used in an application, should be declared
*   prior to any write operations to ensure that all variables are transfered
*   correctly whenever a page is full. The virtual addresses are assigned to
*   the variables according to the order of declaration. This means that in
*   case of a system reset, all variables must be declared in the same order on
*   each startup to retain its previous virtual address.
*
* @param[in] var
*   Pointer to variable to be assigned a virtual address.
* @return
*   Returns whether the declaration was successful.
*******************************************************************************/
bool EE_DeclareVariable(EE_Variable_TypeDef *var)
{
  if ( numberOfActiveVariables < MAX_ACTIVE_VARIABLES ) {

    /* The virtual addresses are assigned according to the order of declaration. */
    var->virtualAddress = ++numberOfVariablesDeclared;

    numberOfActiveVariables++;

    return true;
  } else {
    return false;
  }
}


/***************************************************************************//**
 * @brief
 *   Returns the number of times all pages has been erased.
 *
 * @note
 *   The erase count is the number of cycles where all the pages has been
 *   erased. The value is always written to the 24 LSB of the first word on the
 *   active page. The value is incremented each time data is transferred to
 *   page 0.
 *
 * @return
 *   Returns the number of complete erase cycles endured.
 ******************************************************************************/
uint32_t EE_GetEraseCount(void)
{
  /* Make sure that there is an active page */
  EFM_ASSERT(activePageNumber != -1);

  uint32_t eraseCount;

  /* The number of erase cycles is the 24 LSB of the first word of the active page. */
  eraseCount = (*(pages[activePageNumber].startAddress) & 0x00FFFFFF);

  /* if the page has never been erased, return 0. */
  if (eraseCount == 0xFFFFFF) {
    return 0;
  }

  return eraseCount;
}



void SysTick_Handler(void)
{
  msTicks++;
}


void moveInterruptVectorToRam(void)
{
  memcpy(vectorTable, (uint32_t*)SCB->VTOR, sizeof(uint32_t) * VECTOR_SIZE);
  SCB->VTOR = (uint32_t)vectorTable;
}

//END EEPROM

/******************************************************************//**
 * Declaration of Initializing  Functions
 *********************************************************************/
/******************************************************************//**
 * Initialize LEUART
 *********************************************************************/


void LEUART_Initialization()
{
	  /* Reseting and initializing LEUART0 */
	  LEUART_Reset(LEUART0);

	  CMU_OscillatorEnable(cmuOsc_LFXO,true,true);// Enable Oscillator
	  /* Start LFXO, and use LFXO for low-energy modules */
	  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);//LFXO and LFB Clock Tree
	  //CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);//LFXO and LFA Clock Tree
	  /* Enabling Clock */
	  CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART0 clock */
	  /* Defining the LEUART1 initialization data */
	  LEUART_Init_TypeDef leuart_Init ;

	  leuart_Init.enable=leuartEnable;//leuartEnable ;/*Enable Both Tx and Rx on LEUART accessing CMD reg*/
	  leuart_Init.refFreq=CMU_ClockFreqGet(cmuClock_LEUART0) ;                      /* Inherit the clock frequency from the LEUART clock source */
	  leuart_Init.baudrate=BAUD_RATE;			  /* Inherit the clock frequency from the LEUART clock source */
	  leuart_Init.databits=NoOfBitsPerCharacter;  /* Each LEUART frame contains  NoOfBitsPerCharacter databits */
	  leuart_Init.parity=Parity;				  /* Parity bits in use */
	  leuart_Init.stopbits=NoOfStopBits;		  /* Setting the number of stop bits in a frame to 2 bitperiods */

	  /*Initialize LEUART */
	  LEUART_Init(LEUART0, &leuart_Init); //Initialize LEUART0
	  //LEUART0->CMD |= LEUART_CMD_RXBLOCKDIS	;


	  /* Configuring Pin location */
	  // Location of TXO and RXI pins and enable them
	  LEUART0->ROUTE |= LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN| LEUART_ROUTE_LOCATION_LOC0;

	  /* feeding Tx to Rx for debugging purpose and checking LEUART functionality */
	  if (debugLoop) LEUART0->CTRL |= LEUART_CTRL_LOOPBK;
	  LEUART0->CTRL |=LEUART_CTRL_TXDELAY_DOUBLE;



}


/******************************************************************//**
 * Initialize GPIO for Led

 *********************************************************************/
void GPIO_LEDInit(){

  /* Configure user led as output */
  GPIO_PinModeSet(LED_GPIO_PORT, LED_GPIO_PIN, gpioModePushPull, 0);

}
/******************************************************************//**
 * Initialize GPIO for BLE Intergration using LEUART0
 *RXI - Input connected to PD4 (Input Pulled up)
 *TXO - Output connected to PD5 (Push Pulled )
 *********************************************************************/
void GPIO_BLEInit()
{

	  /* Configure PD4 as a RXI  input pin*/
	  GPIO_PinModeSet(gpioPortD, RXI_PIN, gpioModeInputPull, 1);
	  /* Configure PD5 as a TXO output pin*/
	  GPIO_PinModeSet(gpioPortD, TX_PIN, gpioModePushPull, 1);
	  /* Configure PD5 as a CTS output pin*/
	  GPIO_PinModeSet(gpioPortD, CTS_PIN, gpioModePushPull, 0);
	  //Clear GPIO pin -- Constant Gnd
	  GPIO_PinOutClear(gpioPortD,CTS_PIN);
}





/******************************************************************//**
 * Initialize GPIO for BLE Intergration using LEUART0
 * Drive Input LIGHT SENSE
 * Drive Analog Comparator by output LIGHT EXCITE of light sensor
 *********************************************************************/
void GPIO_LESENSEInit()
{

	/* Configure the drive strength of the ports for the light sensor. */
	  GPIO_DriveModeSet(gpioPortD, gpioDriveModeStandard);
	  GPIO_DriveModeSet(gpioPortC, gpioDriveModeStandard);

	  /* Configure PC6 as a LIGHT_SENSE  input pin and disable it */
	  GPIO_PinModeSet(gpioPortC, LIGHT_SENSE, gpioModeDisabled, 0);
	  /* Configure PD6 as a LIGHT_EXCITE output pin*/
	  GPIO_PinModeSet(gpioPortD, LIGHT_EXCITE, gpioModePushPull, 0);
	  /* Clearing the pin initially */
	  GPIO_PinOutClear(gpioPortD,LIGHT_EXCITE);
	  /* Configure PE6 as a LED_PIN output pin*/
	  GPIO_PinModeSet(gpioPortE, LED_PIN, gpioModePushPull, 0);


}


/******************************************************************//**
 * Initialize GPIO for I2C1 for HMC5833L
 * ReRoute to I2C1_SCL - PC4 (o/p)
 * 			  I2C1_SDA - PC5 (i/p and o/p)
 * 			  Enable Internal Pull ups
 *Enable Routing of Location 0 for this Pins
 *********************************************************************/

void GPIO_I2C1Init()
{

	  /* Enable pins at location 1 */
	  I2C1->ROUTE = I2C_ROUTE_SDAPEN |
	                I2C_ROUTE_SCLPEN |
	                //(1 << _I2C_ROUTE_LOCATION_SHIFT);
	                (I2C_ROUTE_LOCATION_LOC0);


	/* Using PC4 (SDA) and PC5 (SCL) */
	  GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);
	  GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);



	  // Setting up PE1 to indicate transfer direction
	  GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);



}

/******************************************************************//**
 * Initialize GPIO for Proximity
 *
 *********************************************************************/
void GPIO_ProxyInit()
{
	/* Using PC0 */
	 GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUp, 1);

}
/******************************************************************//**
 * Initialization of all GPIO
 *********************************************************************/

void GPIO_Initialization()

{
   /* Enable clock for GPIO module */
   CMU_ClockEnable(cmuClock_GPIO, true);
   /* Enable GPIO for BLE module */
   GPIO_BLEInit();
   /* Enable GPIO for LESENSE module */
   //GPIO_LESENSEInit();//Discarded as ambient Sensor removed
   /* Enable GPIO for LED */
   GPIO_LEDInit();
   /* Enable GPIO for I2C1 for HMC5833L */
   GPIO_I2C1Init();

}

/******************************************************************//**
 * Function to Initialize LETIMER (Low Energy Timer)
 *********************************************************************/

void LESENSE_Initialization(void)
{
	/* enable Oscillator */
	//CMU_OscillatorEnable(cmuOsc_LFRCO,true,true);// Enable Oscillator
	/* Start LFXO, and use LFXO for low-energy modules */


	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);//LFXO and LFA Clock Tree

	    /* Enable CORELE clock. */
	CMU_ClockEnable(cmuClock_CORELE, true);


	CMU_ClockEnable(cmuClock_LESENSE, true);
	/* Enable clock divider for LESENSE. */
	CMU_ClockDivSet(cmuClock_LESENSE, cmuClkDiv_1);

	/* Disable clock source for LFB clock*/
	//CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);

	CMU_ClockEnable(cmuClock_DAC0, true);
	CMU_ClockEnable(cmuClock_ACMP0, true);
	/* GPIO */
	CMU_ClockEnable(cmuClock_GPIO, true);


	  //LC Sensor
	  /* Configure excitation/measure pin as push pull */
	   GPIO_PinModeSet(gpioPortC, LCSENSE_CH, gpioModePushPull, 0);


	 /* LESENSE channel configuration constant table. */
	 static LESENSE_ChAll_TypeDef initChs = LESENSE_LIGHTSENSE_SCAN_CONF;

	 static  LESENSE_ChDesc_TypeDef initLesenseCh7=LESENSE_LCSENSE_SENSOR_CH_CONF;

	 /* Configure alternate excitation channels. */
	 //static LESENSE_ConfAltEx_TypeDef initAltEx = LESENSE_LIGHTSENSE_ALTEX_CONF;



	/*********** Configure  Structure *************/
	static  LESENSE_Init_TypeDef initLESENSE;
	initLESENSE.coreCtrl.scanStart=lesenseScanStartPeriodic;// New scan Started everytime counter overflows
	initLESENSE.coreCtrl.prsSel=lesensePRSCh0;// Any Chnnel , As PRS is not used for trigger scan
	initLESENSE.coreCtrl.scanConfSel=lesenseScanConfDirMap ;//Directly mapped to channel number
	initLESENSE.coreCtrl.invACMP0=false ;
	initLESENSE.coreCtrl.invACMP1=false ;
	initLESENSE.coreCtrl.dualSample=false ;
	initLESENSE.coreCtrl.storeScanRes=false ;
	initLESENSE.coreCtrl.bufOverWr=true ; // Allows writting to buffer even if full
	initLESENSE.coreCtrl.bufTrigLevel = lesenseBufTrigHalf;//Trigger Conditions for Intr and DMA
															// Triggers when buffer half full
	initLESENSE.coreCtrl.wakeupOnDMA=lesenseDMAWakeUpDisable ;//can be use to wake DMA up
															 //USE LATER ON lesenseDMAWakeUpBufValid

	initLESENSE.coreCtrl.biasMode=lesenseBiasModeDutyCycle  ;// Between low poer and high accuracy mode
	initLESENSE.coreCtrl.debugRun=false ; // LESENSE not in debug mode



	/*********** Configure Timing  		 *************/
	initLESENSE.timeCtrl.startDelay=0; // NO Delay

	/*********** Configure Peripheral    *************/
	initLESENSE.perCtrl.dacCh0Data=lesenseDACIfData; //
	initLESENSE.perCtrl.dacCh0ConvMode=lesenseDACConvModeSampleOff;
	//initLESENSE.perCtrl.dacCh0ConvMode=lesenseDACConvModeDisable;
	initLESENSE.perCtrl.dacCh0OutMode=lesenseDACOutModeDisable;
	initLESENSE.perCtrl.dacCh1Data=lesenseDACIfData;
	//initLESENSE.perCtrl.dacCh1ConvMode=lesenseDACConvModeDisable;
	initLESENSE.perCtrl.dacCh1ConvMode=lesenseDACConvModeSampleOff;
	//initLESENSE.perCtrl.dacCh1OutMode=lesenseDACOutModeDisable;
	initLESENSE.perCtrl.dacCh1OutMode=lesenseDACOutModePin;
	//initLESENSE.perCtrl.dacPresc= 0;
	initLESENSE.perCtrl.dacPresc=31;
	//initLESENSE.perCtrl.dacRef=lesenseDACRefBandGap;
	initLESENSE.perCtrl.dacRef=lesenseDACRefVdd;
	//initLESENSE.perCtrl.acmp0Mode=lesenseACMPModeMuxThres; /* Allow LESENSE to control ACMP mux and reference threshold. */
	initLESENSE.perCtrl.acmp0Mode=lesenseACMPModeMux;
	//initLESENSE.perCtrl.acmp1Mode=lesenseACMPModeMuxThres;
	initLESENSE.perCtrl.acmp1Mode=lesenseACMPModeDisable,
	initLESENSE.perCtrl.warmupMode=lesenseWarmupModeNormal; /* Normal mode means LESENSE is allowed to dutycycle comparator and reference. */


	/*********** Configure Decoder   *************/
	initLESENSE.decCtrl.decInput= lesenseDecInputSensorSt;
	initLESENSE.decCtrl.initState=0;
	initLESENSE.decCtrl.chkState=false;
	initLESENSE.decCtrl.intMap=false;
	initLESENSE.decCtrl.hystPRS0=false;
	initLESENSE.decCtrl.hystPRS1=false;
	initLESENSE.decCtrl.hystPRS2=false;
	initLESENSE.decCtrl.hystIRQ=false;
	initLESENSE.decCtrl.prsCount=true;
	initLESENSE.decCtrl.prsChSel0 = lesensePRSCh0;
	initLESENSE.decCtrl.prsChSel1 = lesensePRSCh1;
	initLESENSE.decCtrl.prsChSel2 = lesensePRSCh2;
	initLESENSE.decCtrl.prsChSel3 = lesensePRSCh3;


	 /* Initialize LESENSE interface  */
	//Initialize w/o reset
	 LESENSE_Init(&initLESENSE, true);

	 //Initializing all with Ambient Light and LC  Channel, for Sensing
	 ////LESENSE_ChannelAllConfig(&initChs);
	 LESENSE_ChannelConfig(&initLesenseCh7, LCSENSE_CH);
	 //Initializing all with Ambient Light  Channel, for Excitation
	 ////LESENSE_AltExConfig(&initAltEx);


	 /* Set scan frequency (in Hz).   Fscan = LFACLKles / ((1+PCTOP)*2^PCPRESC)
	  * 0 - Current clock as reference clock
	  * scanFreq - Desired frequency
	  * */
	 LESENSE_ScanFreqSet(0, LCSENSE_SCAN_FREQ);

	 /* Set clock divisor for LF clock. */
	 LESENSE_ClkDivSet(lesenseClkHF, lesenseClkDiv_1);
	 LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_2);
	 //LESENSE->IEN|=LESENSE_IEN_SCANCOMPLETE;

	 /* Enable interrupt in NVIC. */
	 NVIC_EnableIRQ(LESENSE_IRQn);
	  /* Calibrate LC sensor */
	 lesenseCalibrateLC(LCSENSE_CH);
	 //LESENSE_ChannelThresSet(7,0,0);
	 //Start Scanning LESENSE
	 LESENSE_ScanStart();

}


/**************************************************************************//**
 * Initialization of  ACMP
 *****************************************************************************/
void ACMP_Initialization(void)
{

  /* Enable clocks for ACOMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);
  /* ACMP configuration constant table. */
  static  ACMP_Init_TypeDef initACMP;

  initACMP.fullBias = false;                 /* fullBias */
  initACMP.halfBias = true;                 /* halfBias */
  //initACMP.biasProg =  0x0;                 /* biasProg */
  initACMP.biasProg =  0xE;                 /* biasProg */
  initACMP.interruptOnFallingEdge =  false;  /* interrupt on rising edge */
  initACMP.interruptOnRisingEdge =  false;   /* interrupt on falling edge */
  //initACMP.warmTime = acmpWarmTime512;       /* 512 cycle warmup to be safe */
  initACMP.warmTime = acmpWarmTime4;       /* 512 cycle warmup to be safe */ //for LC
  initACMP.hysteresisLevel =acmpHysteresisLevel0; /* hysteresis level 5  for Ambient*/
  //initACMP.hysteresisLevel = acmpHysteresisLevel0; /* hysteresis level 5  for LCt*/
  initACMP.inactiveValue = false;            /* inactive value */
  initACMP.lowPowerReferenceEnabled = false; /* low power reference */
  initACMP.vddLevel = 0x0D;                  /* VDD level */
  initACMP.enable = false;                    /* Don't request enabling. */



  /* Configure ACMP. */
  ACMP_Init(ACMP0, &initACMP);
  /* Disable ACMP0 out to a pin. */
  //ACMP_GPIOSetup(ACMP0, 0, false, false);
  /* Set up ACMP negSel to VDD, posSel is controlled by LESENSE. */
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel7);
  /* LESENSE controls ACMP thus ACMP_Enable(ACMP0) should NOT be called in order
   * to ensure lower current consumption. */
}





/**************************************************************************//**
 * Initialization of  PRS
 *****************************************************************************/
void PRS_Initilization(void)
{
   /* PRS */
   CMU_ClockEnable(cmuClock_PRS, true);

  /* PRS channel 0 configuration */
  /* LESENSE SCANRES bit PRS_SIGSEL_CHANNEL will be used as PRS signal */
  PRS_SourceAsyncSignalSet(PRS_CHANNEL,
                           PRS_CH_CTRL_SOURCESEL_LESENSEL,
                           PRS_CH_CTRL_SIGSEL_LESENSESCANRES6);
}


/**************************************************************************//**
 * Initialization of PCNT
 *****************************************************************************/
void PCNT_Initilization(void)
{

  /* PCNT */
  CMU_ClockEnable(cmuClock_PCNT0, true);

  /* PCNT configuration constant table. */
  static PCNT_Init_TypeDef initPCNT;
  initPCNT.mode = pcntModeOvsSingle; /* Oversampling, single mode. */
  initPCNT.counter = 0; /* Counter value has been initialized to 0. */
  initPCNT.top = LIGHTSENSE_NUMOF_EVENTS; /* Counter top value. */
  initPCNT.negEdge = false; /* Use positive edge. */
  initPCNT.countDown = false; /* Up-counting. */
  initPCNT.filter = false; /* Filter disabled. */
  initPCNT.hyst = false; /* Hysteresis disabled. */
  initPCNT.s1CntDir = false; /* Counter direction is given by CNTDIR. */
  initPCNT.cntEvent = pcntCntEventUp;/* Regular counter counts up on upcount events. */
  initPCNT.auxCntEvent = pcntCntEventNone; /* Auxiliary counter doesn't respond to events. */
  initPCNT.s0PRS = pcntPRSCh0; /* PRS channel 0 selected as S0IN. */
  initPCNT.s1PRS = pcntPRSCh0;  /* PRS channel 0 selected as S1IN. */



  /* Initialize PCNT. */
  PCNT_Init(PCNT0, &initPCNT);
  /* Enable PRS input S0 in PCNT. */
  PCNT_PRSInputEnable(PCNT0, pcntPRSInputS0, true);

  /* Enable the PCNT peripheral. */
  PCNT_Enable(PCNT0, pcntModeOvsSingle);
  /* Enable the PCNT overflow interrupt. */
  PCNT_IntEnable(PCNT0, PCNT_IEN_OF);

  /* Enable the PCNT vector in NVIC */
  NVIC_EnableIRQ(PCNT0_IRQn);
}


/******************************************************************//**
 * Function to Initialize LETIMER (Low Energy Timer)
 *********************************************************************/

void LETIMER_Initialization(void)
{
    LETIMER_Init_TypeDef LETIMER_IntValues;
    /* Turn on clocks
    using ULFRCO(1Khz)
    */
    CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);//Oscillator Enabled (in EM3 ULFRCO available)
    CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);// LFA is connected to LETIMER
    CMU_ClockEnable(cmuClock_LETIMER0,true); // Enable Low Timer Energy Clock

    /*Initialize values of  LETIMER*/
    LETIMER_IntValues.enable = false ;// not start counting when init complete ,explicitly start
    LETIMER_IntValues.debugRun = false; // halt counting when debug
    LETIMER_IntValues.comp0Top = true; // Load Value of Comp0Top when CNT underflows
    LETIMER_IntValues.rtcComp0Enable = false;/* Don't start counting on RTC COMP0 match. */
    LETIMER_IntValues.rtcComp1Enable = false;/* Don't start counting on RTC COMP1 match. */
    LETIMER_IntValues.out0Pol=0;/* Idle value for output 0. */
    LETIMER_IntValues.out1Pol=0;/* Idle value for output 0. */
    LETIMER_IntValues.ufoa0=letimerUFOANone ;//Configuring the output 0 one in PWM mode
    LETIMER_IntValues.ufoa1=letimerUFOANone;//Configuring the output 0 one in No output action mode
    LETIMER_IntValues.repMode = letimerRepeatFree;/* Count until Stopped */

    /* initialize timer */
    LETIMER_Init(LETIMER0,&LETIMER_IntValues);//Initialize all values set


    /* Compare values of the COMP0 and COMP1 set and same value re loaded when under
     * flow occurs
     */
    LETIMER_CompareSet(LETIMER0,0,ADCTriggerPeriod);//For Frequency of PWM

    /* Repetition values must be nonzero so that the outputs
         return switch between idle and active state */
    LETIMER_RepeatSet(LETIMER0, 0, 0x01);



    /* clear the interrupts*/

    LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0);//Clear Comp0 and UF interrupts

   /*Enable the interrupts */

    LETIMER_IntEnable(LETIMER0, LETIMER_IFC_COMP0);//Enable Comp0 and UF interrupts

    NVIC_EnableIRQ(LETIMER0_IRQn);
    }



/******************************************************************//**
 * Function to Initialize ADC (Analog To Digital Convertor )
 * Sample Rate - 62.5Khz ,
 * Resolution - 12 bit
 * Voltage Reference - 1.25 V
 * Channel - Temperature
 * Mode - Single Mode
 *********************************************************************/
void ADC_Initialization(void)
	{
    /* Enable clocks for ADC */
	CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_ADC0 , true);


    /**************************************************************
    *
    *Initializing values for ADC and ADC Single mode
    ***************************************************************/
    //Assigning  ADC Initial values
    ADC_Init_TypeDef     ADCInit ;//    = ADC_INIT_DEFAULT;
    ADCInit.ovsRateSel =adcOvsRateSel2 ; // 2x oversampling (if enabled).
    ADCInit.lpfMode =adcLPFilterBypass ; //No input filter selected.
    ADCInit.warmUpMode =adcWarmupNormal ;//ADC shutdown after each conversion
											/*
											 * adcWarmupFastBG 	Do not warm-up bandgap references.
											 * adcWarmupKeepScanRefWarm Reference selected for scan mode kept warm.
											*/
    ADCInit.timebase=ADC_TimebaseCalc(0);	//Set to 0 to use currently defined HFPER clock setting
    ADCInit.prescale = ADC_PrescaleCalc(ScaledFrequency, 0);//calculated value for 62.5Khz Sampling

    //Initialize ADC
    ADC_Init(ADC0, &ADCInit);


    //Assigning  ADC Single Input Initial values
    ADC_InitSingle_TypeDef   ADCSingleInit ;// ADC_INITSINGLE_DEFAULT;

    ADCSingleInit.prsSel = adcPRSSELCh0 ; /* PRS ch0 (if enabled). */
    ADCSingleInit.acqTime = adcAcqTime2  ;  /* 2 ADC_CLK cycle acquisition time,2.28 if prescalar is 16 */
    ADCSingleInit.reference = adcRef1V25   ;   /* 1.25V internal reference for Temperature Sensor. */
    ADCSingleInit.resolution =   adcRes12Bit   ;   /* 1.25V internal reference for Temperature Sensor. */
    ADCSingleInit.input= adcSingleInpTemp;//adcSingleInputTemp variable not working*/
    										//Using Temperature Sensor as input
    ADCSingleInit.diff=false;//Selecting single ended
    ADCSingleInit.prsEnable=false; //PRS disabled
    ADCSingleInit.leftAdjust=false;//Right Adjusted Selected
    ADCSingleInit.rep=true; /* De-activating conversion after one scan*/


    //Initializing values for Single Input ADC
    ADC_InitSingle(ADC0,&ADCSingleInit);

	}



/******************************************************************//**
 * Function to Initialize DMA (Direct Memory Access)
 * Configure in Basic Mode
 * Transfer between ADC to DMA
 *********************************************************************/

void DMA_Initialization()
{
	/* Enable Clock to DMA */
	CMU_ClockEnable(cmuClock_DMA , true);

	/* Initialize the DMA */
	DMA_Init_TypeDef DMAInit;
	DMAInit.hprot =0;// Set 0 , as protection not used
	DMAInit.controlBlock = dmaControlBlock;
	/* Initial DMA */
	DMA_Init(&DMAInit);
}

/******************************************************************//**
 * Function to Configure  DMA for ADC (Direct Memory Access)
 * Configure in Basic Mode
 * Transfer between ADC to DMA
 *********************************************************************/


void DMAADC_Initialization()
{
	/* Setting up DMA Descriptor for Basic Transfer*/
	DMA_CfgDescr_TypeDef  DMA_Desc;
	DMA_Desc.dstInc=dmaDataInc2;//Increment the destination address by two after each transfer
	DMA_Desc.srcInc=dmaDataIncNone;//No Increment in source address as ADC buffer remains constant
	DMA_Desc.size=dmaDataSize2;//Half Word ,two bytes of data
	DMA_Desc.arbRate=DMA_ARBITRATE_ADC;//arbitrate after 1 DMA transfer(can be changed to 256 as working only on ADC)
	DMA_Desc.hprot=0;//Set for Disabling Protection

	/*Configure DMA Descriptor and configuring ADC as primary descriptor*/
	DMA_CfgDescr(DMA_CHANNEL_ADC, true, &DMA_Desc);

	/* Setting up call-back function */
	cb[DMA_CHANNEL_ADC].cbFunc  = TransferADCComplete;//
	cb[DMA_CHANNEL_ADC].userPtr = NULL;
	cb[DMA_CHANNEL_ADC].primary = true;

	/* Configure DMA Channel */
	DMA_CfgChannel_TypeDef DMAChannel;

	DMAChannel.highPri = DMA_PRIORITY_ADC;
	DMAChannel.enableInt = true;//Interrupt enabled for DMA Call Backup after completion
	DMAChannel.select	= DMAREQ_ADC0_SINGLE;//Source Channel selected is ADC Single mode channel
	DMAChannel.cb		= &cb[DMA_CHANNEL_ADC];//Using Call Back feature ,Pointing to function
	DMA_CfgChannel(DMA_CHANNEL_ADC, &DMAChannel);//Configuring Channel


	/*Setting up DMA descriptors for channel and activating it for transfer
	 * Basic Transfer as each transfer activated by ADC
	 * * */
	DMA_ActivateBasic(DMA_CHANNEL_ADC,true,false,(void *)BufferAdcData,(void *)&(ADC0->SINGLEDATA),NumberOfSamples - 1);
}




/******************************************************************//**
 * Function to Configure  DMA for TX (Direct Memory Access)
 * Configure in Basic Mode
 * Transfer between  DMA to LEUART Tx
 *********************************************************************/

void DMATX_Initialization()
{
	/* Setting up DMA Descriptor for Basic Transfer*/
	DMA_CfgDescr_TypeDef  DMA_TX_Desc;
	DMA_TX_Desc.dstInc=dmaDataIncNone;//No Increment in source address as TX LEUART buffer remains constant
	DMA_TX_Desc.srcInc=dmaDataInc1;//Increment the destination address by one bye after each transfer
	DMA_TX_Desc.size=dmaDataSize1;//one byte of data
	DMA_TX_Desc.arbRate=DMA_ARBITRATE_TX;//arbitrate after 1 DMA transfer(can be
	DMA_TX_Desc.hprot=0;//Set for Disabling Protection

	/*Configure DMA Descriptor and configuring ADC as primary descriptor*/
	DMA_CfgDescr(DMA_CHANNEL_TX, true, &DMA_TX_Desc);

	/* Setting up call-back function */
	cb[DMA_CHANNEL_TX].cbFunc  = NULL;//
	cb[DMA_CHANNEL_TX].userPtr = NULL;
	cb[DMA_CHANNEL_TX].primary = false;

	/* Configure DMA Channel */
	DMA_CfgChannel_TypeDef DMATXChannel;


	DMATXChannel.highPri = DMA_PRIORITY_TX;
	DMATXChannel.enableInt = false;//Interrupt enabled for DMA Call Backup after completion
	DMATXChannel.select	= DMAREQ_LEUART0_TXBL;//Source Channel selected is ADC Single mode channel
	DMATXChannel.cb = NULL;//Using Call Back feature ,Pointing to function
	DMA_CfgChannel(DMA_CHANNEL_TX, &DMATXChannel);//Configuring Channel

	/*Setting up DMA descriptors for channel and activating it for transfer
	 * Basic Transfer as each transfer activated by ADC
	 * * */
	DMA_ActivateBasic(DMA_CHANNEL_TX,true,false,(void *)&(LEUART0->TXDATA),(void *)&Message,MAX_MESSAGE-1);

	LEUART_TxDmaInEM2Enable(LEUART0,true);
}


/******************************************************************//**
 * Function to Configure  DMA for RX LEUART (Direct Memory Access)
 * Configure in Basic Mode
 * Transfer between  LEUART Tx to DMA
 *********************************************************************/



void DMARX_Initialization()
{
	/* Setting up DMA Descriptor for Basic Transfer*/
	DMA_CfgDescr_TypeDef  DMA_Desc;
	DMA_Desc.dstInc=dmaDataInc1;//Increment the destination address by one after each transfer
	DMA_Desc.srcInc=dmaDataIncNone;//No Increment in source address as rx buffer buffer remains constant
	DMA_Desc.size=dmaDataSize1;//one byte of data
	DMA_Desc.arbRate=DMA_ARBITRATE_RX;//arbitrate after 1 DMA transfer(can be changed to 256 as working only on ADC)
	DMA_Desc.hprot=0;//Set for Disabling Protection

	/*Configure DMA Descriptor and configuring ADC as primary descriptor*/
	DMA_CfgDescr(DMA_CHANNEL_RX, true, &DMA_Desc);

	/* Setting up call-back function */
	cb[DMA_CHANNEL_RX].cbFunc  = NULL;//
	cb[DMA_CHANNEL_RX].userPtr = NULL;
	cb[DMA_CHANNEL_RX].primary = true;

	/* Configure DMA Channel */
	DMA_CfgChannel_TypeDef DMAChannel;

	DMAChannel.highPri = DMA_PRIORITY_RX;//Priority for Rx
	DMAChannel.enableInt = false;//Interrupt enabled for DMA Call Backup after completion
	DMAChannel.select	= DMAREQ_LEUART0_RXDATAV;//Source Channel selected is LEUART0
	DMAChannel.cb		= NULL;//Using Call Back feature ,Pointing to function
	DMA_CfgChannel(DMA_CHANNEL_RX, &DMAChannel);//Configuring Channel


	/* Starting the transfer. Using Basic Mode */
	  DMA_ActivateBasic(DMA_CHANNEL_RX,                /* Activate channel selected */
	                    true,                       /* Use primary descriptor */
	                    false,                      /* No DMA burst */
	                    (void *) &rxData,            /* Destination address */
	                    (void *) &LEUART0->RXDATA,  /* Source address*/
	                    MAX_MESSAGE -1);               /* Size of buffer minus1 */

	  /* Set LEUART signal frame */
	  LEUART0->SIGFRAME = 'K';

	  /* Enable Interrupt on following
	   * Tx Complete (TXC Complete)
	   * Valid Received Data (RXDATAV) */
	  LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF);//|LEUART_IEN_RXDATAV);
	  //LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF|LEUART_IEN_RXDATAV);
	  LEUART_IntDisable(LEUART0,LEUART_IEN_TXC);
	  //LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF|LEUART_IEN_TXC|LEUART_IEN_RXDATAV);

	  //LEUART0->CMD |= LEUART_CMD_RXBLOCKEN;// RXBLOCK
	  /* Enable LEUART0 interrupt vector */
	  NVIC_EnableIRQ(LEUART0_IRQn);

	  /* Make sure the LEUART wakes up the DMA on RX data */
	  //For DMA interrupt
	  //LEUART0->CTRL = LEUART_CTRL_RXDMAWU|LEUART_CTRL_TXDMAWU;
	  //Or use following
	  //LEUART_TxDmaInEM2Enable(LEUART0,true);
	  LEUART_RxDmaInEM2Enable(LEUART0,true);

}


/**************************************************************************//**
 * I2C Read and Write Functions
 *****************************************************************************/
// Used by the read_register and write_register functions
// data_array is read data for WRITE_READ and tx2 data for WRITE_WRITE
void i2c_transfer(uint16_t device_addr, uint8_t cmd_array[], uint8_t data_array[], uint16_t cmd_len, uint16_t data_len, uint8_t flag)
{
      // Transfer structure
      I2C_TransferSeq_TypeDef i2cTransfer;

      // Initialize I2C transfer
      I2C_TransferReturn_TypeDef result;
      i2cTransfer.addr          = device_addr;
      i2cTransfer.flags         = flag;
      i2cTransfer.buf[0].data   = cmd_array;
      i2cTransfer.buf[0].len    = cmd_len;

      // Note that WRITE_WRITE this is tx2 data
      i2cTransfer.buf[1].data   = data_array;
      i2cTransfer.buf[1].len    = data_len;

      // Set up the transfer
      result = I2C_TransferInit(I2C1, &i2cTransfer);

      // Do it until the transfer is done
      while (result != i2cTransferDone)
      {
            if (result != i2cTransferInProgress)
            {
                  break;
            }
            result = I2C_Transfer(I2C1);
      }
}

// Read a config register on an I2C device
// Tailored for the HMC5833L device only i.e. 1 byte of TX
uint8_t i2c_read_register(uint8_t reg_offset)
{
      cmd_array[0] = reg_offset;
      i2c_transfer(HMC_ADDRESS, cmd_array, data_array, 1, 1, I2C_FLAG_WRITE_READ);
      return data_array[0];
}



// Write a config register on an I2C device
// Tailored for the HMC5833L device only i.e. 1 byte of TX
void i2c_write_register(uint8_t reg_offset, uint8_t write_data)
{
      cmd_array[0] = reg_offset;
      data_array[0] = write_data;
      i2c_transfer(HMC_ADDRESS, cmd_array, data_array, 1, 1, I2C_FLAG_WRITE_WRITE);
}

/******************************************************************//**
 * To Configure  I2C protocol for communicating with HMC5833L
 * (Magnetometer ) with following interface
 * HMC5833L    EFM32 (Leopard Gecko)
 * SCL			PC5
 * SDA			PC4
 * GND			GND
 * VCC			5V
 * DRDY
 *
 * Configure in Basic Mode
 * Transfer between  DMA to LEUART Tx
 *********************************************************************/

void I2C_HMC_Initialization()
{

	/* Enabling clock to the I2C1 */
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C1, true);

	 // Using default settings
     I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;


     i2cInit.enable                    = 1;
     i2cInit.master                    = 1;
     i2cInit.freq                      = I2C_FREQ_STANDARD_MAX;
     i2cInit.clhr                      = i2cClockHLRStandard;

     /* Initializing the I2C */
     I2C_Init(I2C1, &i2cInit);

}

/*********************************************************************************
 * HMC5833l Initialization function
 * Initializing Configuration A and B
 ********************************************************************************/
void HMC_Initilization ()
{
		I2C_HMC_Initialization(); //Initialize the HMC
	    GPIO_I2C1Init(); //Initialize and Route
		i2c_write_register(ConfigurationRegisterA,0x70);
		i2c_write_register(ConfigurationRegisterB,0xA0);//..440lsb/Gauss
}

/*********************************************************************************
 * HMC5833l Initialization function
 ********************************************************************************/
void HMC_CoordinateRead()
{
	XVal = 0, YVal = 0, ZVal = 0;
	//Refer Pg 11 from HMC5833L for
	XVal = (data_array[0]<<8) | data_array[1];//As Data Out XA(0x03) - MSB
											  //As Data Out XB(0x04) - LSB
	ZVal = (data_array[2]<<8) | data_array[3];//As Data Out ZA(0x05) - MSB
	  	  	  	  	  	  	  	  	  	  	  //As Data Out ZB(0x06) - LSB
	YVal = (data_array[4]<<8) | data_array[5];//As Data Out YA(0x07) - MSB
	  	  	  	  	  	  	  	  	  	  	  //As Data Out YB(0x08) - LSB

	//TO limit the values from -180 to 180
	XVal = XVal % 180;
	YVal = YVal % 180;
	ZVal = ZVal % 180;
}

/*********************************************************************************
 * HMC5833l Initialization function for Continuous Mode
 ********************************************************************************/
void HMC_ContinousMode(unsigned char MAX_Reading)
{
			int i =MAX_Reading;
			i2c_write_register(ModeRegister,Measurement_Continuous);
	        while (i--)
	        {
				// Measurement data starts at DATAX0, and ends at DATAZ1, 6 bytes long
				cmd_array[0] = DataRegisterBegin;
				// Read 6 bytes at once
				i2c_transfer(HMC_ADDRESS, cmd_array, data_array, 1, 6, I2C_FLAG_WRITE_READ);
				HMC_CoordinateRead();
	        }

}

/*********************************************************************************
 * HMC5833l Initialization function for Single Mode
 ********************************************************************************/

void HMC_SingleMode()
{
	i2c_write_register(ModeRegister,Measurement_Continuous);
   	cmd_array[0] = DataRegisterBegin;
    // Read 6 bytes at once
    i2c_transfer(HMC_ADDRESS, cmd_array, data_array, 1, 6, I2C_FLAG_WRITE_READ);
    HMC_CoordinateRead();
}

/*********************************************************************************
 * HMC5833l Check Limit by
 * Running in Single Mode
 * Constructing Message to be displayed
 ********************************************************************************/
void HMCLimitCheck()
{
	HMC_SingleMode();//run Magnetometer
	memset(MagMessage,0,sizeof(MagMessage));//Clearing the Message before writing new message
	strcpy(MagMessage,"\nX: ");
	ConvertInttoString(XVal);// Convert value to String
	strcat(MagMessage,AxistoString);
	strcat(MagMessage,"\tY: ");
	ConvertInttoString(YVal);
	strcat(MagMessage,AxistoString);
	strcat(MagMessage,"\tZ: ");
	ConvertInttoString(ZVal);
	strcat(MagMessage,AxistoString);
	strcat(MagMessage,"\n");
	unsigned char Length = strlen(MagMessage);
	DMA_ActivateBasic(DMA_CHANNEL_TX,true,false,(void *)&(LEUART0->TXDATA),(void *)&MagMessage,Length-1);
}



/******************************************************************//**
 * Declaration of Call Back function after DMA complete
 *********************************************************************/
void TransferADCComplete(unsigned int channel, bool primary, void *user)
	{
	  //(void) channel;
	  (void) primary;
	  (void) user;

	  /* Reset ADC */
	  ADC_Reset(ADC0);

	  TempCalculationAndDisplay();//Calling function to calculate and LED On and OFF

	}

/******************************************************************//**
 * ISR Definition of Interrupts(Not using in code of without Interrupts )
 *********************************************************************/


/******************************************************************//**
 * LEUART0 ISR
 * Implementing
 * 1) RX Interrupt
 * 2) SIGFRAME Interrupt -
 * 		(i)Replying on correct
 * 		(ii)Replying on incorrect command
 * 	3) Call to functions for Check of Command
 *********************************************************************/

void LEUART0_IRQHandler()
{

	 //To transmit data if tx completed
	 if (LEUART0->IF & LEUART_IF_SIGF) //Detecting Signal Frame
	 	 	{
				CheckCommand();//Check for Command received
				k=0;//ReIntilaize variable for

				//strcat (retService,FloattoString);
				//strcpy (retService,FloattoString);
				/* Reactivate DMA for RX */
				DMA_ActivateBasic(DMA_CHANNEL_RX,  /* Activate DMA channel for Rx LEUART */
				                  true,            /* Activate using primary descriptor */
				                  false,           /* No DMA burst */
				                  NULL,            /* Keep source */
				                  NULL,            /* Keep destination */
				                  MAX_MESSAGE - 1);/* Number of DMA transfer elements (minus 1) */

	 	 	}
	 	 /* Reactivate DMA for Tx */
	 	 //DMA_ActivateBasic(DMA_CHANNEL_TX,true,false,(void *)&(LEUART0->TXDATA),(void *)&retService,MAX_MESSAGE-1);
	 	 /* Clear Interrupts */
	 	 LEUART_IntClear(LEUART0,LEUART_IFC_SIGF|LEUART_IFC_FERR|LEUART_IFC_PERR);

}



/******************************************************************//**
 * LETIMER0 Interrupt Handler Definition
 *********************************************************************/

void LETIMER0_IRQHandler()
{

	/* clearing the interrupts */
    LETIMER_IntClear(LETIMER0,LETIMER_IFC_COMP0);
    ADC_Initialization();//Initializing the ADC , No clock tree and Clock Osc required
    ADC_Start(ADC0, adcStartSingle);//Start ADC
    /*Setting up DMA descriptors for channel and activating it for transfer
     * Basic Transfer as each transfer activated by ADC
     * * */
    DMA_ActivateBasic(DMA_CHANNEL_ADC,true,false,(void *)BufferAdcData,(void *)&(ADC0->SINGLEDATA),NumberOfSamples - 1);
    HMCAxisLimitCheck();//Read and Check values of HMC Sensor
    ////unblockSleepMode(EM1);
    //blockSleepMode(EM1);
}

/******************************************************************//**
 * LESENSE Interrupt Handler Definition
 *********************************************************************/


/**************************************************************************//**
 * Interrupt handlers
 *****************************************************************************/
/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler(void)
{
  /* Clear interrupt flag */
  LESENSE_IntClear(LESENSE_IF_CH7);

	static unsigned char j=0;//Counter for Detection on Proximity Sensor
	unsigned char Length=0;
	j++;

	//if (j>MAX_PROXIMITY_SENSOR)
	//{
		Length=strlen("\nMetal Detected!!");
		strcpy(Message,"\nMetal Detected!!");
		//GPIO_PinOutSet(LED_GPIO_PORT, LED_GPIO_PIN);
		if (debug) r++;//Debug Integer to view
		DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
												  true,            /* Activate using primary descriptor */
												  false,           /* No DMA burst */
												  NULL,            /* Keep destination */
												  (void *)&Message,/* Change source */
												  Length - 1);/* Number of DMA transfer elements (minus 1) */
	//	j=0;
	//}

}




/******************************************************************//**
 * PCNT0 Interrupt Handler Definition
 *********************************************************************/

void PCNT0_IRQHandler(void)
{
  /* Clear interrupt flag */
  PCNT_IntClear(PCNT0, PCNT_IEN_OF);
  unsigned char Length;

  /* Disable RTC first to reset counter */
  //RTC_Enable(false);
  /* Set compare value */
  //RTC_CompareSet(0, RTC_COMP_VALUE);
  /* Enable RTC */
  //RTC_Enable(true);

  /* Turn on user led */
  GPIO_PinOutSet(LED_GPIO_PORT, LED_GPIO_PIN);

  strcpy(Message,"Dark Room\n");
  Length = strlen(Message);
  DMA_ActivateBasic(DMA_CHANNEL_TX,true,false,(void *)&(LEUART0->TXDATA),(void *)&Message,Length-1);
}

void clearUARTDMA(char Message[],char Length)
{
	LEUART0->CMD = LEUART_CMD_CLEARRX;
	while((LEUART0->SYNCBUSY & LEUART_SYNCBUSY_CMD) != 0);
	strncpy(Message,"",Length);

}


/******************************************************************//**
 * Definition and Declaring  of Functions
 *********************************************************************/


//Compare string
bool CompareString (char Str1[],char Str2[])
{
	unsigned char a =0, b=0,c=0;
	 if ((Str1[a] == '\0') || (Str2[b]== '\0')||(Str1[a] == '\n') || (Str2[b]== '\n'))
	 	 {
		 	 return false;
	 	 }
	  while ((Str1[a] !='\n') && (Str2[b]!= '\n')  &&  (Str1[a] !='\0') && (Str2[b]!= '\0'))
		  {
		  	  if ( Str1[a] != Str2[b] )
		  	  {
		  		  c++;
		  		  break;
		  	  }
		  	a++;
		  	b++;

	  	  }
	  if ( c !=0 )
	  {
		  return false;
	  }
	  else
	  {
		  return true;
	  }

}




/******************************************************************************
 * Check type of command
 * Pre input - Command to be checked
 * Post - Command type
 *		0 - Temp
 *		1 - Mag Status
 *		2 - X axis to be set
 *		3 - Y axis to be set
 *		4 - Z axis to be set
 *		5 - Temp Sensor
 *		6 - LC //Not implemented
 *		7
 *
 */

unsigned char typeCommand( char compareCommand[])
{

	//bool resultCommand;
	//unsigned char cmpstr=strncpy(rxData,7);
	unsigned char i=0;
	char cmpstr[8];
	while(rxData[i]!= '=' && rxData[i]!='\0'  )
	{
		cmpstr[i]=rxData[i];
		i++;
	}
	cmpstr[i]='\0';
	if (CompareString(cmpstr,Command0))// Result of Comparision Stored
			return 1;
	else if (CompareString(cmpstr,Command1))// Result of Comparision Stored
			return 2;
	else if (CompareString(cmpstr,"SetMagX"))
			return 3;
	else if (CompareString(cmpstr,"SetMagY"))
			return 4;
	else if (CompareString(cmpstr,"SetMagZ"))
			return 5;
	else if (CompareString(cmpstr,"SetTemp"))
			return 6;
	else if (CompareString(cmpstr,"SetLCMX"))
			return 7;
	else
			return 9;
}

//Split command
//Zero - Ret
//One - Set

unsigned char splitCommandType()
{
	char typeStr[3];
	strncpy(typeStr,rxData,3);
	if (CompareString(typeStr,"Set"))
		return 1;
	else if (CompareString(typeStr,"Ret"))
		return 0 ;
	else
		return 2;
}


/************************************************************
 * Command - Set<Sensor>=Min,Max
 * Pre - Set<Sensor>=Min,Max input to function
 * Post - Store Min and Max values into MinLimit and MaxLimit
 * respectively with following format
 *		 SetMag=Min,Max
 *	Mag - Sensor Name
 *	Min - Min value , check for negative value
 *	Max - Max value , check for positive value
 ***********************************************************/

unsigned char splitLimit()
{
	unsigned char i=0,j=0
			;//,MaxLimit[3],MinLimit[3];
	memset(Message,0,sizeof(MaxLimit));//Clear max Limit
	memset(Message,0,sizeof(MinLimit));//Clear min Limit
	while (rxData[i]!='=' && rxData[i]!='\0')
	{
		i++;
	}
	if (rxData[i]=='\0')//Check if First element is a null character
	{
		return 0;
	}
	while (rxData[++i]!=',')// Wait for "," for parsing Min value

	{
		if ((rxData[i]>='0' && rxData[i]<='9') || rxData[i]==',' || rxData[i]=='-')
		{
			MinLimit[j++]=rxData[i];//Store Min value
		}
		else
		{
			return 0;// If non Number
		}
		//i++;
	}
	j=0;
	while (rxData[++i]!='\0' && rxData[i]!='\n')//Wait "\0" for parsing Max value
	{
		if (rxData[i]>='0' && rxData[i]<='9' || rxData[i]=='-')// Check if Numerical value
				{
					MaxLimit[j++]=rxData[i];//Store Max value
				}
				else
				{
					return 0;//If non - Number
				}
		//i++;
	}
	return 1;//Return a value

}


///Converting value String/ASCII value to Hex nuumber
short int ASCIItoNumber(char number[])
{
     unsigned char i=0,j=0 ;
     unsigned char tempnumber=0;
     short int decimalValue=0;
     signed int  signvalue=1;
     while (number[j]!='\0')
     {
         length ++;
         j++;

     }
     //Error Conditions
     if(number[0]=='-')
     {
    	 signvalue=-1;
    	 i++;
     }
    //Conversion of ASCII to hex value
	decimalValue =0;//Reinitialize value to zero
	decimalValue=atoi(number);
	return (decimalValue*signvalue);
}


/******************************************************************//**
 * Function to Check Command received and
 *
 *type(Ret)- 0 -Return Status of Sensor
 *type(Set)- 1 -Set Alert points into Memory (EEPROM)
 *
 *
 *********************************************************************/
void CheckCommand()
{
		int Length=0;
		unsigned char retResult=0 ,type=0;
		type=splitCommandType();
		//Section of RET status command
		if (!type)//Ret
		{
							retResult=typeCommand(rxData);
							switch (retResult)
							{
								case 1 :
										{
												ConvertFloattoString(tempComp);
												Length=strlen(FloattoString);
												strcpy(Message,FloattoString);
												if (debug) r++;//Debug Integer to view
												DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
																						  true,            /* Activate using primary descriptor */
																						  false,           /* No DMA burst */
																						  NULL,            /* Keep destination */
																						  (void *)&Message,/* Change source */
																						  Length - 1);/* Number of DMA transfer elements (minus 1) */


										}
										break;
								case 2:
										{
											HMCLimitCheck();// Check for HMC
										}
										break;
								default:
										{
											//Display an Error Command
											Length=strlen(Error1);
											clearUARTDMA(Message,Length);
											strcpy(Message,Error1);
											if (debug) r++;//Debug Integer to view
											DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
																	  true,            /* Activate using primary descriptor */
																	  false,           /* No DMA burst */
																	  NULL,            /* Keep destination */
																	  (void *)&Message, /* Change source */
																	  Length - 1);/* Number of DMA transfer elements (minus 1) */
										}
										break;
							}
		}
		else if (type==1)//Set
		{
			signed int LimitMin=0;
			signed int LimitMax=0;
			unsigned char storeSuccess=0;
			unsigned splitResult=0;
			retResult=typeCommand(rxData);
					switch (retResult)
					{
						//For X-axis
						case 3:
								{
									//Split the values int Min , Max values
									splitResult=splitLimit();
									//Covert values to hex

									if(splitResult)
									{
										LimitMax=ASCIItoNumber(MaxLimit);

										LimitMin=ASCIItoNumber(MinLimit);
											if (LimitMax > LimitMin)
											{
													//Write values into the EEPROM
													EE_Write(&MagXmax,LimitMax);//Write Minimum Value
													EE_Write(&MagXmin,LimitMin);//Write Maximum Value
													//Read and Check right values
													EE_Read(&MagXmax,&readValue);
													EE_Read(&MagXmin,&readValue);
											}
											else
											{
												storeSuccess++;// Not Success
											}
									}
									else
									{
										storeSuccess++;//Not Success
									}
									if (!storeSuccess) //if Successful
									{
										Length=strlen(Success);
										clearUARTDMA(Message,Length);
										strcpy(Message,Success);
									}
									else //If Any error
									{
										Length=strlen(Error1);
										clearUARTDMA(Message,Length);
										strcpy(Message,Error1);
									}
									//if (debug) r++;//Debug Integer to vie
									DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
															  true,            /* Activate using primary descriptor */
															  false,           /* No DMA burst */
															  NULL,            /* Keep destination */
															  (void *)&Message, /* Change source */
															  Length - 1);/* Number of DMA transfer elements (minus 1) */

								}
								break;
						//For Y- axis
						case 4:
						{
														//Split the values int Min , Max values
														splitResult=splitLimit();
														//Covert values to hex

														if(splitResult)
														{
															LimitMax=ASCIItoNumber(MaxLimit);

															LimitMin=ASCIItoNumber(MinLimit);
																if (LimitMax > LimitMin)
																{
																		//Write values into the EEPROM
																		EE_Write(&MagYmax,LimitMax);//Write Minimum Value
																		EE_Write(&MagYmin,LimitMin);//Write Maximum Value
																		//Read and Check right values
																		EE_Read(&MagYmax,&readValue);
																		EE_Read(&MagYmin,&readValue);
																}
																else
																{
																	storeSuccess++;//If Error
																}
														}
														else
														{
															storeSuccess++;
														}
														if (!storeSuccess) //if Successful
														{
															Length=strlen(Success);
															clearUARTDMA(Message,Length);
															strcpy(Message,Success);

														}
														else //If Any error
														{
															Length=strlen(Error1);
															clearUARTDMA(Message,Length);
															strcpy(Message,Error1);
														}
														//if (debug) r++;//Debug Integer to vie
														DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
																				  true,            /* Activate using primary descriptor */
																				  false,           /* No DMA burst */
																				  NULL,            /* Keep destination */
																				  (void *)&Message, /* Change source */
																				  Length - 1);/* Number of DMA transfer elements (minus 1) */

													}
													break;
						// For Z -axis
						case 5:
						{
														//Split the values int Min , Max values
														splitResult=splitLimit();
														//Covert values to hex

														if(splitResult)
														{
															LimitMax=ASCIItoNumber(MaxLimit);

															LimitMin=ASCIItoNumber(MinLimit);
																if (LimitMax > LimitMin)
																{
																		//Write values into the EEPROM
																		EE_Write(&MagZmax,LimitMax);//Write Minimum Value
																		EE_Write(&MagZmin,LimitMin);//Write Maximum Value
																		//Read and Check right values
																		EE_Read(&MagZmax,&readValue);
																		EE_Read(&MagZmin,&readValue);
																}
																else
																{
																	storeSuccess++;
																}
														}
														else
														{
															storeSuccess++;
														}
														if (!storeSuccess) //if Successful
														{
															Length=strlen(Success);
															clearUARTDMA(Message,Length);
															strcpy(Message,Success);

														}
														else //If Any error
														{
															Length=strlen(Error1);
															clearUARTDMA(Message,Length);
															strcpy(Message,Error1);
														}
														//if (debug) r++;//Debug Integer to vie
														DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
																				  true,            /* Activate using primary descriptor */
																				  false,           /* No DMA burst */
																				  NULL,            /* Keep destination */
																				  (void *)&Message, /* Change source */
																				  Length - 1);/* Number of DMA transfer elements (minus 1) */

													}
													break;
                       // For Temperature
						case 6:
						{
														//Split the values int Min , Max values
														splitResult=splitLimit();
														//Covert values to hex

														if(splitResult)
														{
															LimitMax=ASCIItoNumber(MaxLimit);

															LimitMin=ASCIItoNumber(MinLimit);
																if (LimitMax > LimitMin)
																{
																		//Write values into the EEPROM
																		EE_Write(&Tempmax,LimitMax);//Write Minimum Value
																		EE_Write(&Tempmin,LimitMin);//Write Maximum Value
																		//Read and Check right values
																		EE_Read(&Tempmax,&readValue);
																		EE_Read(&Tempmin,&readValue);
																}
																else
																{
																	storeSuccess++;
																}
														}
														else
														{
															storeSuccess++;
														}
														if (!storeSuccess) //if Successful
														{
															Length=strlen(Success);
															clearUARTDMA(Message,Length);
															strcpy(Message,Success);

														}
														else //If Any error
														{
															Length=strlen(Error1);
															clearUARTDMA(Message,Length);
															strcpy(Message,Error1);
														}
														//if (debug) r++;//Debug Integer to vie
														DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
																				  true,            /* Activate using primary descriptor */
																				  false,           /* No DMA burst */
																				  NULL,            /* Keep destination */
																				  (void *)&Message, /* Change source */
																				  Length - 1);/* Number of DMA transfer elements (minus 1) */

													}
													break;
						default://For Error
								{
									Length=strlen(Error1);
									clearUARTDMA(Message,Length);
									strcpy(Message,Error1);
									if (debug) r++;//Debug Integer to view
									DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
															  true,            /* Activate using primary descriptor */
															  false,           /* No DMA burst */
															  NULL,            /* Keep destination */
															  (void *)&Message, /* Change source */
															  Length - 1);/* Number of DMA transfer elements (minus 1) */
								}


					}

		}
		//For ERROR
		else if (type==2)
		{
			Length=strlen(Error1);
			clearUARTDMA(Message,Length);
			strcpy(Message,Error1);
			if (debug) r++;//Debug Integer to view
			DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
									  true,            /* Activate using primary descriptor */
									  false,           /* No DMA burst */
									  NULL,            /* Keep destination */
									  (void *)&Message, /* Change source */
									  Length - 1);/* Number of DMA transfer*/



		}


}
/******************************************************************//**
 * Conversion to Celsius function
 * input ADC Sample
 * output Temp in Celsius
 *********************************************************************/


float convertToCelsius(int32_t adcSample)
{
	float temp;
	/* Factory calibration temperature from device information page. */
	float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);
	float cal_value_0 = (float)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
	/* Temperature gradient (from datasheet) */
	float t_grad = -6.27;
	temp = (cal_temp_0 - ((cal_value_0 - adcSample) / t_grad));
	return temp;
}


/******************************************************************//**
 * Declaration of  function to
 * 1) find average of readings
 * 2)Convert to Degree Celcius
 *********************************************************************/
void TempCalculationAndDisplay()
{

	int tempSum =0;
	int tempAverage =0;
	int i=0;
	int Length=0;
	while (i<NumberOfSamples)
		{
			tempSum+=BufferAdcData[i];
			i++;
		}
	tempAverage = tempSum/NumberOfSamples;

	tempComp = convertToCelsius(tempAverage);
	EE_Read(&Tempmax,&readValue);
	UpperTempLimit=readValue;//Read Upper Limit
	EE_Read(&Tempmin,&readValue);
	LowerTempLimit=readValue;//Read Lower Limit

	//strcpy(Temperature,FloattoString);
	if (tempComp >= UpperTempLimit)
		{
			ConvertFloattoString(tempComp);
			strcpy(Message,Above_limit);
			strcat(Message,FloattoString);
			Length=strlen(Message);//Calculating length of string
			DMA_ActivateBasic(DMA_CHANNEL_TX,true,false,(void *)&(LEUART0->TXDATA),(void *)&Message,Length-1);
		}
	if (tempComp <= LowerTempLimit)
		{
			ConvertFloattoString(tempComp);
			strcpy(Message,Below_limit);
			strcat(Message,FloattoString);
			Length=strlen(Message);//Calculating length of string
			DMA_ActivateBasic(DMA_CHANNEL_TX,true,false,(void *)&(LEUART0->TXDATA),(void *)&Message,Length-1);
		}


}



/******************************************************************//**
 * Function to check if any Limit for each of axis and generate
 * relevant  message for same
 *********************************************************************/
void HMCAxisLimitCheck()
{

	int Length=0;
	unsigned int checkAxis=0;//Variable to check if any previous message/violation of limits

	HMC_SingleMode();

	//Read Value of  X axis from emulated  EEPROM
	EE_Read(&MagXmax,&readValue);
	UpperXLimit=readValue;
	EE_Read(&MagXmin,&readValue);
	LowerXLimit=readValue;
	//Read Value of  Y axis from emulated  EEPROM
	EE_Read(&MagYmax,&readValue);
	UpperYLimit=readValue;
	EE_Read(&MagYmin,&readValue);
	LowerYLimit=readValue;
	//Read Value of  Z axis from emulated  EEPROM
	EE_Read(&MagZmax,&readValue);
	UpperZLimit=readValue;
	EE_Read(&MagZmin,&readValue);
	LowerZLimit=readValue;


	//Check XVal if exceeds UpperLimit
	if (XVal >= UpperXLimit)
		{
			checkAxis++;
			memset(Message,0,sizeof(Message));
			ConvertInttoString(XVal);
			strcpy(Message,"\n X-axis inclination above Max set: ");
			strcat(Message,AxistoString);

		}
	//Check XVal if exceeds Lower X Limit
	else if (XVal <= LowerXLimit)
		{
			checkAxis++;
			memset(Message,0,sizeof(Message));
			ConvertInttoString(XVal);
			strcpy(Message,"\n X-axis inclination below Min set: ");
			strcat(Message,AxistoString);

		}
		//Check for Y Axis
		if (YVal >= UpperYLimit)
			{


				ConvertInttoString(YVal);
				if (!checkAxis)
				{
					memset(Message,0,sizeof(Message));
					strcpy(Message,"\n Y-axis inclination above Max set: ");
					//strcat(Message,"\n");
				}
				else
				{
					checkAxis++;
					strcat(Message,"\n Y-axis inclination above Max set: ");
				}

				strcat(Message,AxistoString);

			}

		else if (YVal <= LowerYLimit)
			{

				ConvertInttoString(YVal);
				if (!checkAxis)
				{
					memset(Message,0,sizeof(Message));
					strcpy(Message,"\n Y-axis inclination below Min set: ");
				}
				else
				{

					strcat(Message,"\n Y-axis inclination below Min set: ");
				}
				checkAxis++;
				strcat(Message,AxistoString);

			}
		//Check for Z Axis
		if (ZVal >= UpperZLimit)
			{


				ConvertInttoString(ZVal);
				if (!checkAxis)
				{
					memset(Message,0,sizeof(Message));
					strcpy(Message,"\n Z-axis inclination above Max set: ");
				}
				else
				{

					strcat(Message,"\n Z-axis inclination above Max set: ");
				}
				checkAxis++;
				strcat(Message,AxistoString);

			}
		//Check for Z Axis
		else if (ZVal <= LowerYLimit)
			{

				ConvertInttoString(ZVal);
				if (!checkAxis)
				{
					memset(MagMessage,0,sizeof(Message));
					strcpy(Message,"\nZ-axis inclination below Min set: ");
				}
				else
				{

					strcat(Message,"\nZ-axis inclination below Min set: ");
				}
				checkAxis++;
				strcat(Message,AxistoString);

			}
		// Data to Send through DMA
	if (checkAxis)//Set DMA axis if any limit violation is found
	{
		strcpy(MagMessage,Message);
		Length=strlen(Message);//Calculating length of string
		clearUARTDMA(Message,Length);
		strcpy(Message,MagMessage);
		DMA_ActivateBasic(DMA_CHANNEL_TX,true,false,(void *)&(LEUART0->TXDATA),(void *)&Message,Length-1);
	}
	checkAxis=0;
}


/**********************************************************
 * Convert Float to String
 * Pre Function : Input Float Number
 * 				Assumtion XX.XX (float)
 * Post Function:store into HH.L C(XX.X C) format in String
 * It takes care of Positive and Negative numbers
 * Has an Issue with Single digit Negative Number
 *
 * ******************************************************/
void ConvertFloattoString(float Number)
{
	int TempNum= Number * 100;
	//char Temp[5];
	unsigned char z=0;
	char tempStr[TEMP_SIZE];
			tempStr[0]='\n';
			if (TempNum<0)
			{
				tempStr[0]='-';
			}
			else{
				tempStr[0]='+';
			}
			//if(TempNum>-10 && TempNum<10);
			//{FloattoString[1]=0;z++;}
			while (TempNum/10 > 0)
		{
			FloattoString[z]=((TempNum % 10)+0x30);
			TempNum=TempNum/10;
			z++;
		}
		FloattoString[z]=((TempNum % 10)+0x30);
		tempStr[1]=FloattoString[3];
		tempStr[2]=FloattoString[2];
		tempStr[3]='.';
		tempStr[4]=FloattoString[1];
		tempStr[5]=FloattoString[0];
		tempStr[6]=' ';
		tempStr[7]='C';

		tempStr[8]='\0';


		strcpy(FloattoString,tempStr);
}

/**********************************************************
 * Convert Float to String
 * Pre Function : Input Float Number
 * 				Assumtion XXX (int)
 * Post Function:store into HHH format in String
 * ******************************************************/
void ConvertInttoString(int Number)
{
	int TempNum=0;
	//char Temp[5];
	unsigned char z=0,totallength=MaxMagLenght+1;

	memset(AxistoString,0,sizeof(AxistoString));//ReInitializing value to zeros (as Global value)
	//Check for Negative NUmbers and append a

	if (Number < 0)
	{
		AxistoString[z]='-';
		TempNum = 0 - (Number);
	}
	else
	{
		AxistoString[z]='+';
		TempNum = Number;
	}
	z++;
	while (TempNum/10 > 0 || z < totallength)
	{
			AxistoString[totallength-z]=((TempNum % 10)+0x30);
			TempNum=TempNum/10;
			z++;
	}

}

/**********************************************************
 * Function For Correlation
 * ******************************************************/

void BSP_TraceSwoSetup(void)
{
  /* Enable GPIO clock */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

  /* Set correct location */
  /* This location is valid for GG, LG and WG! */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on correct pin. */
  /* This pin is valid for GG, LG and WG! */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;

  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  /* Wait until clock is ready */
  while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY)) ;

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= CoreDebug_DHCSR_C_DEBUGEN_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  DWT->CTRL = 0x400113FF;

  /* Set TPIU prescaler to 16. */
  TPI->ACPR = 15;

  /* Set protocol to NRZ */
  TPI->SPPR = 2;

  /* Disable continuous formatting */
  TPI->FFCR = 0x100;

  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;

  /* ITM Channel 0 is used for UART output */
  ITM->TER |= (1UL << 0);
}


/**********************************************************
*Function to Program BLE using UART
*Following parameters are programmed
*******************************************************/


void TransmitCommand( char Str1[])
{
	unsigned char  x=0;
	LEUART0->STATUS |= LEUART_STATUS_TXBL;
		do
		{
			while (!(LEUART0->STATUS & LEUART_STATUS_TXBL));
			LEUART0->TXDATA = Str1[x];
			x++;
		}
		while (Str1[x]!='\0');


}

/**********************************************************
 *Function to Program BLE using UART
 *Following parameters are programmed
 *******************************************************/


void delay(unsigned WAIT_PERIOD)
{
	unsigned int i ;
	for(i=0;i<WAIT_PERIOD;i++);


}


/**********************************************************
 *Function to Program BLE using UART
 *Following parameters rxare programmed
 * ******************************************************/
void Program_BLE()
{

	// To change to Command Mode
	char COMMAND[MAX_MESSAGE]=BLE_Command_1;
	unsigned char count=10;
	TransmitCommand(COMMAND);
	//TO check for Command successfully rx or not
    while (	!(strcmp(rxData,"1OK")) && count>0)
    {
    	delay(100000);
    	TransmitCommand(COMMAND);
    	count--;
    }
    memset(COMMAND,0,sizeof(COMMAND));
    memset(rxData,0,sizeof(rxData));



	  DMA_ActivateBasic(DMA_CHANNEL_RX,                /* Activate channel selected */
	                    true,                       /* Use primary descriptor */
	                    false,                      /* No DMA burst */
	                    (void *) &rxData,            /* Destination address */
	                    (void *) &LEUART0->RXDATA,  /* Source address*/
	                    MAX_MESSAGE -1);               /* Size of buffer minus1 */

	  // To change LED MODE
    strcpy(COMMAND,BLE_Command_2);
	TransmitCommand(COMMAND);
	delay(100000);

	  DMA_ActivateBasic(DMA_CHANNEL_RX,                /* Activate channel selected */
	                    true,                       /* Use primary descriptor */
	                    false,                      /* No DMA burst */
	                    (void *) &rxData,            /* Destination address */
	                    (void *) &LEUART0->RXDATA,  /* Source address*/
	                    MAX_MESSAGE -1);               /* Size of buffer minus1 */

		memset(COMMAND,0,sizeof(COMMAND));
		memset(rxData,0,sizeof(rxData));

		 // To change Power LEVEL
	    strcpy(COMMAND,BLE_Command_3);
		TransmitCommand(COMMAND);
		delay(100000);


		 DMA_ActivateBasic(DMA_CHANNEL_RX,                /* Activate channel selected */
		                    true,                       /* Use primary descriptor */
		                    false,                      /* No DMA burst */
		                    (void *) &rxData,            /* Destination address */
		                    (void *) &LEUART0->RXDATA,  /* Source address*/
		                    MAX_MESSAGE -1);               /* Size of buffer minus1 */

		memset(COMMAND,0,sizeof(COMMAND));
		memset(rxData,0,sizeof(rxData));


		// To set Connection and Advertising Intervals
	    strcpy(COMMAND,BLE_Command_4);
		TransmitCommand(COMMAND);
		delay(100000);

		 DMA_ActivateBasic(DMA_CHANNEL_RX,                /* Activate channel selected */
		                    true,                       /* Use primary descriptor */
		                    false,                      /* No DMA burst */
		                    (void *) &rxData,            /* Destination address */
		                    (void *) &LEUART0->RXDATA,  /* Source address*/
		                    MAX_MESSAGE -1);               /* Size of buffer minus1 */

		memset(COMMAND,0,sizeof(COMMAND));
		memset(rxData,0,sizeof(rxData));


		// To set  values and reset to Data mode
	    strcpy(COMMAND,BLE_Command_6);
		TransmitCommand(COMMAND);
		delay(100000);

		  DMA_ActivateBasic(DMA_CHANNEL_RX,                /* Activate channel selected */
		                    true,                       /* Use primary descriptor */
		                    false,                      /* No DMA burst */
		                    (void *) &rxData,            /* Destination address */
		                    (void *) &LEUART0->RXDATA,  /* Source address*/
		                    MAX_MESSAGE -1);               /* Size of buffer minus1 */

			memset(COMMAND,0,sizeof(COMMAND));
			memset(rxData,0,sizeof(rxData));


			//Change Sig frame for normal mode
			LEUART0->SIGFRAME = '\n';


}

void EEPROM_Init()
{
	/* Configure HFRCO Band */
	  CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);

	  /* Enable SysTick Interrupt */
	  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

	  //SegmentLCD_Init(false);

	  /* Move the interrupt vector table to RAM to safely handle interrupts
	   * while performing write/erase operations on flash */
	  moveInterruptVectorToRam();

	  /* Enables the flash controller for writing. */
	  MSC_Init();

	  /* Initialize the eeprom emulator using 3 pages. */
	  if ( !EE_Init(3) ) {

	    /* If the initialization fails we have to take some measure
	     * to obtain a valid set of pages. In this example we simply
	     * format the pages */
	    EE_Format(3);
	  }
	  /* All variables should be declared prior to any writes. */
	  EE_DeclareVariable(&var1);
	  EE_DeclareVariable(&var2);
	  EE_DeclareVariable(&var3);
	  EE_DeclareVariable(&MagXmax);
	  EE_DeclareVariable(&MagYmax);
	  EE_DeclareVariable(&MagZmax);
	  EE_DeclareVariable(&MagXmin);
	  EE_DeclareVariable(&MagYmin);
	  EE_DeclareVariable(&MagZmin);
	  EE_DeclareVariable(&Tempmax);
	  EE_DeclareVariable(&Tempmin);




	  //Write values
	  EE_Write(&var1, 0x7777);

	  EE_Write(&var2, 0x2222);

	  /* Write to var3. */
	  EE_Write(&var3, 0x3333);
	  //EE_Write(&MagXMax,0x100);
	  /* Write to var1. */


	  /* Read the value of var2. */
	  EE_Read(&var2, &readValue);
	  EE_Read(&var1, &readValue);
	  EE_Read(&var3, &readValue);
	  //EE_Read(&MagXMax, &readValue);

}


void ProxyISR()
{
//
	static unsigned char j=0;//Counter for Detection on Proximity Sensor
	unsigned char Length=0;
	j++;

	if (j>MAX_PROXIMITY_SENSOR)
	{
		Length=strlen("\nIntruder Alert!!");
		strcpy(Message,"\nIntruder Alert!!");
		GPIO_PinOutSet(LED_GPIO_PORT, LED_GPIO_PIN);
		if (debug) r++;//Debug Integer to view
		DMA_ActivateBasic(DMA_CHANNEL_TX,  /* Activate DMA channel for Rx LEUART */
												  true,            /* Activate using primary descriptor */
												  false,           /* No DMA burst */
												  NULL,            /* Keep destination */
												  (void *)&Message,/* Change source */
												  Length - 1);/* Number of DMA transfer elements (minus 1) */
		j=0;
	}


	GPIO_PinOutClear(LED_GPIO_PORT, LED_GPIO_PIN);
}

void ProxyInitialization()
{
	unsigned char Pin;

	button.fall(&ProxyISR);
}




/******************************************************************//**
 * Main of Code
 * Calling ChipInit() - Initialization
 * LEUART Initialization
 * GPIO Initialized
 *********************************************************************/


int main()

{
    CHIP_Init(); //Initializing the Chip according to device before
    //if (debug) BSP_TraceSwoSetup();// For Co-relation
    blockSleepMode(EM0); // to disable mode below EM2.Enter the EM2 when Sleep called
    					//As LEUART works in EM2 mode
    //blockSleepMode(EM1); // to disable mode below EM2.Enter the EM2 when Sleep called
    CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
    /* Turn off unused oscillators */
    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);


    /* Disable global interrupts */
      INT_Disable();

      /* Enable clocks for used peripherals */
      setupCMU();

      /* Setup the ACMP */
      setupACMP();

      /* Setup the DAC */
      setupDAC();

      /* Setup the GPIO */
      setupGPIO();

      /* Setup the RTC */
      //setupRTC();

      /* setup lesense */
      setupLESENSE();

      /* Enable global interrupts */
      INT_Enable();






    LEUART_Initialization();// Initialize LEUART0
    						//CMD is not changing
    GPIO_Initialization();//Initializing GPIO
    LETIMER_Initialization();//Initialize LETIMER
    DMA_Initialization();//Initialize DMA
    DMARX_Initialization();//Initialize DMA for Rx of LEUART
    DMAADC_Initialization();//Initialize DMA for ADC


    ADC_Initialization();//Initializing the ADC , No clock tree and Clock Osc required
    //As ADC cannot be initialized before DMA
    //ConvertLimittoString();



    //if (BLECommand) Program_BLE();
    LEUART0->SIGFRAME = '\n';

    DMATX_Initialization();//Initialize DMA for Tx

    //for Integrating LESENSE for ambient light and LC sensor

    //INT_Disable();
    setupDAC();
    ACMP_Initialization();
    //PCNT_Initilization();
    //PRS_Initilization();
    LESENSE_Initialization();
    //INT_Enable();
    //Initialize HMC
    HMC_Initilization();

    EEPROM_Init();

    ProxyInitialization();

    //Enabling Timer as last controlling part of  the system
    LETIMER_Enable(LETIMER0,true); //Enable LETimer
    //unsigned valuePin;

    while(1)
    {
    	/* Transmitting Read data */
    	//valuePin=button.read();
     	sleep();// to go in sleep mode
    }
}
