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
#ifndef _INTERRUPT_ROUTINES_H
    #define _INTERRUPT_ROUTINES_H
    
    #include "cytypes.h"
    #include "stdio.h"
    #include "project.h"

    CY_ISR_PROTO(Custom_isr);
    
    volatile uint8 ReadPacketFlag;
    
#endif
/* [] END OF FILE */
