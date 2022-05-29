/*
        UART Driver for pic32
        Copyright Sep. 2018 TMAX-Electronics.de
 */

#include <stdio.h>
#include <xc.h>
#include <string.h>
#include <stdarg.h>
#include <sys/attribs.h>
#include <sys/kmem.h>

#include "UART.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "TTerm.h"

static void UART_populateDescriptor(uint32_t module, UART_PortHandle * descriptor);

//initializes a UART module and returns a handle for it
//NOT: module will stay disabled until UART_setModuleOn(handle, 1); is called
UART_PortHandle * UART_init(uint32_t module, uint32_t baud, volatile uint32_t* TXPinReg, uint8_t RXPinReg){
    UART_PortHandle * handle = pvPortMalloc(sizeof(UART_PortHandle));
    
    //prepare descriptor data
    if(!UART_populateDescriptor(module, handle)){
        vPortFree(handle);
        return 0;
    }
    handle->TXR = TXPinReg;
    URXPinValue = RXPinReg;
    
    //assign IO
    UTXR = UTXPinValue;
    URXR = URXPinValue;
    
    //initialise module
    UMODE = 0b0000000000001000;                //UART Module ON, U1RX and U1TX only, Autobaud off, 8bit Data no parity, High Speed mode off
    USTA = 0b1001010000000000;                 //Tx interrupt when all is transmitted, Rx & Tx enabled, Rx interrupt when buffer is full
    
    //set baud rate gen
    if(baud > 1000000){
        UMODEbits.BRGH = 1;
        UBRG = (configPERIPHERAL_CLOCK_HZ / (4 * baud)) - 1;
    }else{
        UMODEbits.BRGH = 0;
        UBRG = (configPERIPHERAL_CLOCK_HZ / (16 * baud)) - 1;
    }
    
    return handle;
}

void UART_setModuleOn(UART_PortHandle * handle, uint32_t on){
    UMODEbits.ON = on ? 1 : 0;
}

void UART_setBaud(UART_PortHandle * handle, uint64_t newBaud){
    if(newBaud > 250000){
        UMODEbits.BRGH = 1;
        UBRG = (configPERIPHERAL_CLOCK_HZ / (4 * newBaud)) - 1;
    }else{
        UMODEbits.BRGH = 0;
        UBRG = (configPERIPHERAL_CLOCK_HZ / (16 * newBaud)) - 1;
    }
}

uint32_t UART_getBaud(UART_PortHandle * handle){
    return (configPERIPHERAL_CLOCK_HZ/ (((UMODEbits.BRGH) ? 4 : 16) * (UBRG+1)));
}

inline unsigned UART_isOERR(UART_PortHandle * handle){
    return (USTA & _U2STA_OERR_MASK) != 0;
}

inline unsigned UART_isFERR(UART_PortHandle * handle){
    return (USTA & _U2STA_FERR_MASK) != 0;
}

inline void UART_clearOERR(UART_PortHandle * handle){
    USTAbits.OERR = 0;
}

inline void UART_clearFERR(UART_PortHandle * handle){
    USTAbits.FERR = 0;
}

inline uint32_t UART_available(UART_PortHandle * handle){
    return USTAbits.URXDA;
}

inline uint8_t UART_readChar(UART_PortHandle * handle){
    return URXReg;
}

static uint32_t UART_populateDescriptor(uint32_t module, UART_PortHandle * descriptor){
    switch(module){
#ifdef U1BRG
        case 1:
            descriptor->MODE    = (UxMODE_t*) &U1MODE;
            descriptor->STA     = (UxSTA_t*) &U1STA;
            descriptor->BRG     = &U1BRG;
            descriptor->RXREG   = &U1RXREG;
            descriptor->TXREG   = &U1TXREG;
            descriptor->RXR     = &U1RXR;
            descriptor->TXPV    = 0b0001;
            return 1;
#endif
#ifdef U1BRG
        case 2:
            descriptor->MODE    = (UxMODE_t*) &U2MODE;
            descriptor->STA     = (UxSTA_t*) &U2STA;
            descriptor->BRG     = &U2BRG;
            descriptor->RXREG   = &U2RXREG;
            descriptor->TXREG   = &U2TXREG;
            descriptor->RXR     = &U2RXR;
            descriptor->TXPV    = 0b0010;
            return 1;
#endif
#ifdef U1BRG
        case 3:
            descriptor->MODE    = (UxMODE_t*) &U3MODE;
            descriptor->STA     = (UxSTA_t*) &U3STA;
            descriptor->BRG     = &U3BRG;
            descriptor->RXREG   = &U3RXREG;
            descriptor->TXREG   = &U3TXREG;
            descriptor->RXR     = &U3RXR;
            descriptor->TXPV    = 0b0001;
            return 1;
#endif
#ifdef U1BRG
        case 4:
            descriptor->MODE    = (UxMODE_t*) &U4MODE;
            descriptor->STA     = (UxSTA_t*) &U4STA;
            descriptor->BRG     = &U4BRG;
            descriptor->RXREG   = &U4RXREG;
            descriptor->TXREG   = &U4TXREG;
            descriptor->RXR     = &U4RXR;
            descriptor->TXPV    = 0b0010;
            return 1;
#endif
#ifdef U1BRG
        case 5:
            descriptor->MODE    = (UxMODE_t*) &U5MODE;
            descriptor->STA     = (UxSTA_t*) &U5STA;
            descriptor->BRG     = &U5BRG;
            descriptor->RXREG   = &U5RXREG;
            descriptor->TXREG   = &U5TXREG;
            descriptor->RXR     = &U5RXR;
            descriptor->TXPV    = 0b0011;
            return 1;
#endif
#ifdef U1BRG
        case 6:
            descriptor->MODE    = (UxMODE_t*) &U6MODE;
            descriptor->STA     = (UxSTA_t*) &U6STA;
            descriptor->BRG     = &U6BRG;
            descriptor->RXREG   = &U6RXREG;
            descriptor->TXREG   = &U6TXREG;
            descriptor->RXR     = &U6RXR;
            descriptor->TXPV    = 0b0100;
            return 1;
#endif
        default:
            memset(descriptor, 0, sizeof(UART_PortHandle));
            return 0;
    }
}

//sends a string directly to the output buffer, incase there is some dramatic error
//we don't use the semaphore here as this will only be called from a context where an error prevents the semaphore from being released (tx error, _general_exception_handler, vAssert, etc.)
void UART_sendString(UART_PortHandle * handle, char *data){
    while((*data) != 0){
        while(USTAbits.UTXBF);
        UTXReg = *data++;
    }
}

uint32_t UART_termPrint(void * port, char * format, ...){
    va_list arg;
    va_start (arg, format);
    
    
    uint8_t * buff = (uint8_t*) pvPortMalloc(256);
    uint32_t length = vsprintf(buff, format, arg);
    
    configASSERT(length < 256);
    
    UART_sendString((UART_PortHandle *) port, buff);
    
    vPortFree(buff);
    
    va_end (arg);
    return length;
}