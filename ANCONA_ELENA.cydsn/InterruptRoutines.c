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

#include "project.h"
#include "I2C_Interface.h"
#include "stdio.h"
#include "InterruptRoutines.h"


CY_ISR(Custom_isr)
{      
    FlagInterrupt=1; 
    contatore++;
    
    if (contatore==7)
    {
        contatore=0;
    }
}
/* [] END OF FILE */
