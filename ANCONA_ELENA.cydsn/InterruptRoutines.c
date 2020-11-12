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
#include "InterruptRoutines.h"
#include "project.h"
#include "I2C_Interface.h"
#include "stdio.h"

uint8 counter=0;

CY_ISR(Custom_isr)
{      
    FlagInterrupt=1; 
    counter++;
    
    if (counter==7)
    {
        counter=0;
    }
}
/* [] END OF FILE */
