/*
Program to Send commands to BLE and
Experiment with following

1. Maximum Advertising Interval is - 10240 ms at advertising Timeout - 50 ms before error
2. Maximum Connection Interval is - 4000 ms at minimum connection interval - 50 ms before error
3. Tx POWER
   At about Set Tx =-8 ,RSSI is observed to -77 to -74 with Power loss of 69 db to 66 db at 5 feet
4. LED is OFF after command for saving energy by using #define BLE_Command_2	"AT+HWMODELED=DISABLE\n" // To Disable LED
5.With Using Advertising 250us
	Average current 1.45 ms -  at values of 50,3980,10240,50
	Average current 1.78 ma - 250 ms Advertising
6.With Using Connection Interval 150us
	Average current 1.76ma ma - 150 ms Connection Interval
	Average current 1.85ma	 at values of 50,3980,10240,50


	Code changes made to accommodate COmmand to BLE
	1)SIGFRAME ='K' - For Command to BLE from Code
	2)SIGFRAME ='\n' - For Command from MObile Apps To BLE


*/


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

#include "em_i2c.h"




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


// Globals for persistent storage
uint8_t cmd_array[CMD_ARRAY_SIZE];
uint8_t data_array[DATA_ARRAY_SIZE];



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

//================================================================================
// I2C1_enter_DefaultMode_from_RESET
//================================================================================
extern void I2C1_enter_DefaultMode_from_RESET(void) {
      // $[I2C1 initialization]
      I2C_Init_TypeDef init = I2C_INIT_DEFAULT;

      init.enable                    = 1;
      init.master                    = 1;
      init.freq                      = I2C_FREQ_STANDARD_MAX;
      init.clhr                      = i2cClockHLRStandard;
      I2C_Init(I2C1, &init);
      // [I2C1 initialization]$
}



void GPIO_I2C1Init()
{

	  /* Enable pins at location 1 */
	  I2C1->ROUTE = I2C_ROUTE_SDAPEN |
	                I2C_ROUTE_SCLPEN |
	                //(1 << _I2C_ROUTE_LOCATION_SHIFT);
	                (I2C_ROUTE_LOCATION_LOC0);


	/* Using PC0 (SDA) and PC1 (SCL) */
	  GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);
	  GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);



	  // Setting up PE1 to indicate transfer direction
	  GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);



}


void I2C_HMC_Initialization()
{

	//CMU_OscillatorEnable(cmuOsc_HFRCO,true,true);// Enable Oscillator
	/* Enabling clock to the I2C1 */
	//CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_HFRCO);//LFXO and LFA Clock Tree
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
/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
      CHIP_Init();

      //delay(100);

      	  	  I2C_HMC_Initialization();

            // Offset zero is Device ID

            GPIO_I2C1Init();
            uint16_t value = i2c_read_register(0);
            // Set an LED on the Starter Kit if success
            if (value == DEVICE_ID) {
                ///  set_led(1,1);
            }

            i2c_write_register(ConfigurationRegisterA,0x70);
            //i2c_write_register(ConfigurationRegisterB,0xA0);//..for Gain = 5
            i2c_write_register(ConfigurationRegisterB,0x80);//..440lsb/Gauss
            i2c_write_register(ModeRegister,Measurement_SingleShot);
            uint16_t valuex = i2c_read_register(0x06);

            for (int i=0;i<6;i++);

            int8_t RecData[6] = {0,0,0,0,0,0};
           	int32_t XVal = 0, YVal = 0, ZVal = 0;



            // Infinite loop
            while (1) {}
}
