/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
// Include required header files
#include "I2C_Interface.h"
#include "InterruptRoutines.h"
#include "project.h"
#include "stdio.h"
#include "ErrorCode.h"

uint8 frequency; 

/**
*   \brief 7-bit I2C address of the slave device.
*/
#define LIS3DH_DEVICE_ADDRESS 0x18

/**
*   \brief Address of the WHO AM I register
*/
#define LIS3DH_WHO_AM_I_REG_ADDR 0x0F

/**
*   \brief Address of the Status register
*/
#define LIS3DH_STATUS_REG 0x27

/**
*   \brief Address of the Control register 1
*/
#define LIS3DH_CTRL_REG1 0x20


/**
*   \brief Hex value to set high resolution mode to the accelerator
*/
#define FREQ_1Hz    0x17  //FREQ=1Hz
#define FREQ_10Hz   0x27 
#define FREQ_25Hz   0x37 
#define FREQ_50Hz   0x47 
#define FREQ_100Hz  0x57 
#define FREQ_200Hz  0x67 

/**
*   \brief  Address of the Temperature Sensor Configuration register
*/
#define LIS3DH_TEMP_CFG_REG 0x1F 

#define LIS3DH_TEMP_CFG_REG_ACTIVE 0xC0

/**
*   \brief Address of the Control register 4
*/
#define LIS3DH_CTRL_REG4 0x23

#define LIS3DH_CTRL_REG4_BDU_ACTIVE 0x88 //In high resolution mode HR=1 

/**
*   \brief Address of the ADC output LSB register
*/
#define LIS3DH_OUT_ADC_3L 0x0C

/**
*   \brief Address of the ADC output MSB register
*/
#define LIS3DH_OUT_ADC_3H 0x0D

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    EEPROM_UpdateTemperature();
    EEPROM_Start();  
    
    //Select a sampling frequency for the LIS3DH Accelerometer from the EEPROM
    frequency = EEPROM_ReadByte(0x00); 
   

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    I2C_Peripheral_Start();
    UART_Start();
    
    CyDelay(5); //"The boot procedure is complete about 5 milliseconds after device power-up."
    
    // String to print out messages on the UART
    char message[50] = {'\0'};
    
    UART_Start();
    UART_PutString("**************\r\n");
    UART_PutString("** I2C Scan **\r\n");
    UART_PutString("**************\r\n");
    
    CyDelay(100);
    
    uint32_t rval;
 
	// Setup the screen and print the header
	UART_PutString("\n\n   ");
	for(uint8_t i = 0; i<0x10; i++)
	{
        sprintf(message, "%02X ", i);
		UART_PutString(message);
	}
 
	// Iterate through the address starting at 0x00
	for(uint8_t i2caddress = 0; i2caddress < 0x80; i2caddress++)
	{
		if(i2caddress % 0x10 == 0 )
        {
            sprintf(message, "\n%02X ", i2caddress);
		    UART_PutString(message);
        }
 
		rval = I2C_Master_MasterSendStart(i2caddress, I2C_Master_WRITE_XFER_MODE);
        
        if(rval == I2C_Master_MSTR_NO_ERROR) // If you get ACK then print the address
		{
            sprintf(message, "%02X ", i2caddress);
		    UART_PutString(message);
		}
		else //  Otherwise print a --
		{
		    UART_PutString("-- ");
		}
        I2C_Master_MasterSendStop();
	}
	UART_PutString("\n\n");
    
    /******************************************/
    /*            I2C Reading                 */
    /******************************************/
    
    /* Read WHO AM I REGISTER register */
    uint8_t who_am_i_reg;
    ErrorCode error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                                  LIS3DH_WHO_AM_I_REG_ADDR, 
                                                  &who_am_i_reg);
    if (error == NO_ERROR)
    {
        sprintf(message, "WHO AM I REG: 0x%02X [Expected: 0x33]\r\n", who_am_i_reg);
        UART_PutString(message); 
    }
    else
    {
        UART_PutString("Error occurred during I2C comm\r\n");   
    }
    
    /*      I2C Reading Status Register       */
    
    uint8_t status_register; 
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_STATUS_REG,
                                        &status_register);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "STATUS REGISTER: 0x%02X\r\n", status_register);
        UART_PutString(message); 
    }
    else
    {
        UART_PutString("Error occurred during I2C comm to read status register\r\n");   
    }
    
    /******************************************/
    /*        Read Control Register 1         */
    /******************************************/
    uint8_t ctrl_reg1; 
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG1,
                                        &ctrl_reg1);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 1: 0x%02X\r\n", ctrl_reg1);
        UART_PutString(message); 
    }
    else
    {
        UART_PutString("Error occurred during I2C comm to read control register 1\r\n");   
    }
    
    /******************************************/
    /*            I2C Writing                 */
    /******************************************/
    
    
    if (ctrl_reg1 != frequency)
    {
        ctrl_reg1 = frequency;
    
        error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                             LIS3DH_CTRL_REG1,
                                             ctrl_reg1);
    
        if (error == NO_ERROR)
        {
            sprintf(message, "CONTROL REGISTER 1 successfully written as: 0x%02X\r\n", ctrl_reg1);
            UART_PutString(message); 
        }
        else
        {
            UART_PutString("Error occurred during I2C comm to set control register 1\r\n");   
        }
    }
    
    
    
     /******************************************/
     /* I2C Reading Temperature sensor CFG reg */
     /******************************************/

    uint8_t tmp_cfg_reg;

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_TEMP_CFG_REG,
                                        &tmp_cfg_reg);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "TEMPERATURE CONFIG REGISTER: 0x%02X\r\n", tmp_cfg_reg);
        UART_PutString(message); 
    }
    else
    {
        UART_PutString("Error occurred during I2C comm to read temperature config register\r\n");   
    }
    
    
    tmp_cfg_reg |= LIS3DH_TEMP_CFG_REG_ACTIVE; // must be changed to the appropriate value
    
    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_TEMP_CFG_REG,
                                         tmp_cfg_reg);
    
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_TEMP_CFG_REG,
                                        &tmp_cfg_reg);
    
    
    if (error == NO_ERROR)
    {
        sprintf(message, "TEMPERATURE CONFIG REGISTER after being updated: 0x%02X\r\n", tmp_cfg_reg);
        UART_PutString(message); 
    }
    else
    {
        UART_PutString("Error occurred during I2C comm to read temperature config register\r\n");   
    }
    
    uint8_t ctrl_reg4;

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG4,
                                        &ctrl_reg4);
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 4: 0x%02X\r\n", ctrl_reg4);
        UART_PutString(message); 
    }
    else
    {
        UART_PutString("Error occurred during I2C comm to read control register4\r\n");   
    }
    
    
    ctrl_reg4 = LIS3DH_CTRL_REG4_BDU_ACTIVE; // must be changed to the appropriate value
    
    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG4,
                                         ctrl_reg4);
    
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG4,
                                        &ctrl_reg4);
    
    
    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 4 after being updated: 0x%02X\r\n", ctrl_reg4);
        UART_PutString(message); 
    }
    else
    {
        UART_PutString("Error occurred during I2C comm to read control register4\r\n");   
    }
    
    uint8_t header = 0xA0;
    uint8_t footer = 0xC0;
    uint8_t OutArray [4];
    OutArray[0] = header;
    OutArray[3] = footer;
    int16 conversion = 1;
    int16 converter = 1000;
    float outtempconv;
    
    uint8_t TemperatureData[2];
    int16 OutTemp;
    
    for(;;)
    {
        //Save the last frequency on the EEPROM 
        if (FlagInterrupt==1)
        {
            FlagInterrupt=0;
            
            switch (contatore)
            {
                case (1):
                    EEPROM_WriteByte(FREQ_1Hz, 0x00);
                    frequency=EEPROM_ReadByte(0x00);
                break;
                
                case (2):
                    EEPROM_WriteByte(FREQ_10Hz, 0x00);
                    frequency=EEPROM_ReadByte(0x00);
                break;
                    
                case (3):
                    EEPROM_WriteByte(FREQ_25Hz, 0x00);
                    frequency=EEPROM_ReadByte(0x00);
                break;
                    
                case (4):
                    EEPROM_WriteByte(FREQ_50Hz, 0x00);
                    frequency=EEPROM_ReadByte(0x00);
                break;
                    
                case (5):
                    EEPROM_WriteByte(FREQ_100Hz, 0x00);
                    frequency=EEPROM_ReadByte(0x00);
                break;
                    
                case (6):
                    EEPROM_WriteByte(FREQ_200Hz, 0x00);
                    frequency=EEPROM_ReadByte(0x00);
                break;
            }
           

        CyDelay(100);

        error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                LIS3DH_OUT_ADC_3L,
                                                2,
                                                TemperatureData);
        if(error == NO_ERROR)
        {  
            OutTemp = (int16)((TemperatureData[0] | (TemperatureData[1]<<8)))>>6;
            outtempconv = OutTemp * conversion;
            
            OutTemp = (int16) (outtempconv * converter);
            
            OutArray[1] = (uint8_t)(OutTemp & 0xFF);
            OutArray[2] = (uint8_t)(OutTemp >> 8);
            UART_PutArray(OutArray, 4);
        }
        else
        {
            UART_PutString("Error occurred during I2C comm to read ADC 3 / temperature output registers\r\n");   
        }
        
        
        }   
    }
}
