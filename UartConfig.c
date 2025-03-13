#include <stdint.h>
#include <xc.h>
#include <sys/attribs.h>

#include "UART.h"
#include "UARTconfig.h"

//register map for the pic32mx1xx and pic32mx2xx series chips
const UartDescriptor_t Uart_moduleMap[UART_NUM_MODULES] = 
{   
    [0] = {.registerMap = &U1MODE, .iecReg = &IEC1, .ifsReg = &IFS1, .errorMask = _IEC1_U1EIE_MASK, .rxMask = _IEC1_U1RXIE_MASK, .txMask = _IEC1_U1TXIE_MASK, .ipcReg = &IPC8, .ipcOffset = _IPC8_U1IS_POSITION, _UART1_VECTOR, _UART1_ERR_IRQ}, 
    [1] = {.registerMap = &U2MODE, .iecReg = &IEC1, .ifsReg = &IFS1, .errorMask = _IEC1_U2EIE_MASK, .rxMask = _IEC1_U2RXIE_MASK, .txMask = _IEC1_U2TXIE_MASK, .ipcReg = &IPC9, .ipcOffset = _IPC9_U2IS_POSITION, _UART2_VECTOR, _UART2_ERR_IRQ}
};

#ifdef U1MODE
    void __ISR(_UART1_VECTOR) U1ISR(){ 
        uint32_t ifsState = IFS1;
        IFS1CLR = _IFS1_U1RXIF_MASK;
        UART_isrHandler(1, ifsState);
    } 
#endif 

#ifdef U2MODE
    void __ISR(_UART2_VECTOR) U2ISR(){ 
        uint32_t ifsState = IFS1;
        IFS1CLR = _IFS1_U2RXIF_MASK;
        UART_isrHandler(2, ifsState);
    } 
#endif 