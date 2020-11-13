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
*   \brief Address of the output lower register of the x axis. 
*   
*   the higher register of x axis and the higher and lower registers 
*   of the y and z axis are adiacent to this one
*/
#define LIS3DH_OUT_X_L 0x28

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
*   \brief Address of the Control register 4
*/
#define LIS3DH_CTRL_REG4 0x23

#define LIS3DH_CTRL_REG4_BDU_ACTIVE 0x98 //In high resolution mode HR=1 



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
    char message[50];
    
    CyDelay(5); //"The boot procedure is complete about 5 milliseconds after device power-up."
    
    // Check which devices are present on the I2C bus
    for (int i = 0 ; i < 128; i++)
    {
        if (I2C_Peripheral_IsDeviceConnected(i))
        {
            // print out the address is hex format
            sprintf(message, "\r\nDevice 0x%02X is connected\r\n", i);
            UART_PutString(message); 
        }
        
    }
    
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
    
    
    // Read control register4 
    
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
    
    /******************************************/
    /*            I2C Writing                 */
    /******************************************/
    
    if (ctrl_reg4 != frequency)
    {
        UART_PutString("Updating the register..\r\n");
        
        ctrl_reg4 = frequency; // must be changed to the appropriate value

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
    }
   
        
    
    uint8_t AxisData[6];
    uint8_t DataBuffer[14]; 
    int16_t DataOut;
    
    DataBuffer[0] = 0xA0;                        //header
    DataBuffer[13] = 0xC0;                      //footer
    
   
    float Acc_ms2;                               //axis output in m/s^2
    int32 Acc_int;                               //axis output integer
    
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
                    
                return frequency;
            }
        
        }  

        //read the status register
            ErrorCode error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                                          LIS3DH_STATUS_REG,
                                                          &status_register);
            if (error == NO_ERROR)
            {
                if (status_register & 0x08)     //if (ZYXDA==1). To check if new data are available 
                {   
                //read the x, y, z axis output
                error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                LIS3DH_OUT_X_L,
                                                6, &AxisData[0]);
                 if(error == NO_ERROR)
                    {
                        
                         for (int i=0; i < 6; i += 2) //we read the 3 axis acc and convert them
                        {
                            //trasforming the 3 axial outputs in 3 right-justified 16-bit integers
                            DataOut = (int16)((AxisData[i] | (AxisData[i+1]<<8)))>>4; 
                            //converting the output values in m/s^2 
                            Acc_ms2 = (float) DataOut * 2 * 9.806 * 0.001;
                            //to keep 3 decimals of the acceleration value (Acc_ms2) it is converted into mm/s^2
                            Acc_int = Acc_ms2 * 1000;
                            //storing each axis data in 4 bytes to send them through UART 
                          
                            DataBuffer[i*2+1] = (uint8_t)(Acc_int >> 24);
                            DataBuffer[i*2+2] = (uint8_t)(Acc_int >> 16);
                            DataBuffer[i*2+3] = (uint8_t)(Acc_int >> 8);
                            DataBuffer[i*2+4] = (uint8_t)(Acc_int & 0xFF);
                          
                        }
                    //the samples are sent through UART communication
                    UART_PutArray(DataBuffer, 14);
                    }
                }
            }
    }
}

