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
#include "UARTconfig.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "TTerm.h"
#include "DMAutils.h"
#include "System.h"

#define UART_SENDEVENT(X) {uint32_t evtId = X; xStreamBufferSend(handle->eventStream, &evtId, 1, 0);}

static uint32_t UART_populateDescriptor(uint32_t module, UART_PortHandle * descriptor);

typedef struct{
    uint16_t dataLength;
    unsigned freeAfterSend;
    uint8_t * data;
} UART_SENDQUE_ELEMENT;

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
    if(TXPinReg != NULL) UTXR = UTXPinValue;
    if(RXPinReg != NULL) URXR = URXPinValue;
    
    //initialise module    //TODO proper config functions!
    UMODE = 0;
    USTA = 0;
    
    USTAbits.UTXEN = TXPinReg != -1;
    USTAbits.URXEN = RXPinReg != -1;
    
    handle->rxEnabled = (RXPinReg != -1);
    handle->txEnabled = (TXPinReg != -1);
    
    UART_setBaud(handle, baud);
    
    handle->rxStream = xStreamBufferCreate(UART_BUFFERSIZE,0);
    handle->txStream = xStreamBufferCreate(UART_BUFFERSIZE,0);
    handle->eventStream = xStreamBufferCreate(UART_EVENTBUFFER_SIZE,0);
    
    return handle;
}

uint32_t UART_setRxDMAEnabled(UART_PortHandle * handle, uint32_t enabled){
    if(handle->rxRunning) return 0;
    
    if(enabled){
        if(handle->rxDMAHandle == NULL){
            //initialize ringbuffer
            handle->rxDMAHandle = DMA_createRingBuffer(UART_BUFFERSIZE, 1, handle->RXREG, handle->rxVector, 1, RINGBUFFER_DIRECTION_RX);
            //setup error interrupt for transfer abort
            DMA_RB_setAbortIRQ(handle->rxDMAHandle, handle->fltVector, 0);
        }
    }else{
        if(handle->rxDMAHandle != NULL){
            DMA_freeRingBuffer(handle->rxDMAHandle);
        }
    }
}

/*void UART_setRxIRQMode(UART_PortHandle * handle, uint32_t mode){
    
}*/

static void UART_rxTask(void * params){
    UART_PortHandle * handle = (UART_PortHandle *) params;
    
    handle->rxRunning = 1;
    //rx DMA enabled? aka do we need to check a ring buffer or the register itself
    if(handle->rxDMAHandle == NULL){
        //also some interrupt notify?
        while(UMODEbits.ON){
            if(UART_isOERR(handle)){
                UART_clearOERR(handle);
            }
            
            while(UART_available(handle)){
                xStreamBufferSend(handle->rxStream, URXReg, 1, portMAX_DELAY);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }else{
        while(UMODEbits.ON){
            //check if UART is in an error state
            if(UART_isOERR(handle)){
                //overrun occurred, which is weird because DMA should take care of it. Clear it but assert if debug is active
                configASSERT(0);
                
                UART_clearOERR(handle);
                
                UART_SENDEVENT(UART_EVT_RX_ERROR_OVERFLOW);
            }
            
            if(UART_isFERR(handle)){
                //framing error occurred, clear it
                
                UART_clearFERR(handle);
                
                UART_SENDEVENT(UART_EVT_RX_ERROR_FRAMING);
            }
            
            //check if dma is still running
            if(!DMA_isEnabled(handle->rxDMAHandle->channelHandle)){
                //some error happened and the channel was disabled... re-enable it as all errors should have been dealt with at this point
                DMA_setEnabled(handle->rxDMAHandle->channelHandle, 1);
            }
            
            //wait for data to be written to the buffer
            if(DMA_RB_waitForData(handle->rxDMAHandle, 10000)){
                //cool, some data was received, now write it to the buffer
                DMA_RB_readSB(handle->rxDMAHandle, handle->rxStream, UART_BUFFERSIZE);
            }else{
                //timeout occured :( do nothing
            }
        }
    }
    handle->rxRunning = 0;
    
    //uart module was disabled => shutdown the task. There __should__ be no memory to clean up TODO make sure thats the case
    vTaskDelete(xTaskGetCurrentTaskHandle());
    while(1);
}

static void UART_txTask(void * params){
    UART_PortHandle * handle = (UART_PortHandle *) params;
    
    while(UMODEbits.ON){
        
    }
}

void UART_setModuleOn(UART_PortHandle * handle, uint32_t on){
    //would this even change anything?
    if(on == UMODEbits.ON) return; //lol no, just return as everything is already in the correct state anyway
    
    //yes, check if we are enabling or disabling the module
    if(on){
        //first enable the module to make sure the tasks won't just quit themselves
        UMODEbits.ON = 1;
        
        //enabling, check which modes are currently enabled
        if(handle->txEnabled){
            //tx is on, check if txdma is in use
            //TODO enable TX Task & DMA
            USTAbits.UTXEN = 1;
        }
        
        if(handle->rxEnabled){
            //rx is enabled, do we need dma?
            if(handle->rxDMAHandle != NULL){
                //yes, reset the buffer and enable the channel
                DMA_RB_flush(handle->rxDMAHandle);
                DMA_setEnabled(handle->rxDMAHandle->channelHandle, 1);
            }
            
            //is the rx task still running for some reason?
            if(handle->rxRunning == 0){
                //no, start it TODO evaluate stack size :3
                xTaskCreate(UART_rxTask, "UART rx", configMINIMAL_STACK_SIZE + 100, handle, tskIDLE_PRIORITY + 2, NULL);
            }
            
            USTAbits.URXEN = 1;
        }
    }else{
        //module is to be shut down, do so. All tasks will automatically quit once the module enable gets cleared
        UMODEbits.ON = 0;
    }
}

void UART_setIRQConfig(){
    
}

void UART_updateBaudForSleep(UART_PortHandle * handle, uint32_t sleep){
    if(sleep){
        UMODEbits.BRGH = 1;
        UBRG = (configSECONDARY_CLOCK_HZ / (4 * handle->currentBaudrate)) - 1;
    }else{
        UART_setBaud(handle, handle->currentBaudrate);
    }
}

void UART_setBaud(UART_PortHandle * handle, uint64_t newBaud){
    handle->currentBaudrate = newBaud;
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

inline unsigned UART_isOn(UART_PortHandle * handle){
    return (UMODE & _U2MODE_ON_MASK) != 0;
}

inline unsigned UART_isFERR(UART_PortHandle * handle){
    return (USTA & _U2STA_FERR_MASK) != 0;
}

inline void UART_clearOERR(UART_PortHandle * handle){
    //TODO: evaluate if this is actually correct
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
    memset(descriptor, 0, sizeof(UART_PortHandle));
    switch(module){
#ifdef U1MODE
        case 1:
            descriptor->MODE    = (UxMODE_t*) &U1MODE;
            descriptor->STA     = (UxSTA_t*) &U1STA;
            descriptor->BRG     = &U1BRG;
            descriptor->RXREG   = &U1RXREG;
            descriptor->TXREG   = &U1TXREG;
            descriptor->RXR     = &U1RXR;
            descriptor->TXPV    = 0b0001;
            descriptor->rxVector  = _UART1_RX_IRQ;
            descriptor->txVector  = _UART1_TX_IRQ;
            descriptor->fltVector = _UART1_ERR_IRQ;
            return 1;
#endif
#ifdef U2MODE
        case 2:
            descriptor->MODE    = (UxMODE_t*) &U2MODE;
            descriptor->STA     = (UxSTA_t*) &U2STA;
            descriptor->BRG     = &U2BRG;
            descriptor->RXREG   = &U2RXREG;
            descriptor->TXREG   = &U2TXREG;
            descriptor->RXR     = &U2RXR;
            descriptor->TXPV    = 0b0010;
            descriptor->rxVector  = _UART2_RX_IRQ;
            descriptor->txVector  = _UART2_TX_IRQ;
            descriptor->fltVector = _UART2_ERR_IRQ;
            return 1;
#endif
#ifdef U3MODE
        case 3:
            descriptor->MODE    = (UxMODE_t*) &U3MODE;
            descriptor->STA     = (UxSTA_t*) &U3STA;
            descriptor->BRG     = &U3BRG;
            descriptor->RXREG   = &U3RXREG;
            descriptor->TXREG   = &U3TXREG;
            descriptor->RXR     = &U3RXR;
            descriptor->rxVector  = _UART3_RX_VECTOR;
            descriptor->txVector  = _UART3_TX_VECTOR;
            descriptor->fltVector = _UART3_FAULT_VECTOR;
            return 1;
#endif
#ifdef U4MODE
        case 4:
            descriptor->MODE    = (UxMODE_t*) &U4MODE;
            descriptor->STA     = (UxSTA_t*) &U4STA;
            descriptor->BRG     = &U4BRG;
            descriptor->RXREG   = &U4RXREG;
            descriptor->TXREG   = &U4TXREG;
            descriptor->RXR     = &U4RXR;
            descriptor->TXPV    = 0b0010;
            descriptor->rxVector  = _UART4_RX_VECTOR;
            descriptor->txVector  = _UART4_TX_VECTOR;
            descriptor->fltVector = _UART4_FAULT_VECTOR;
            return 1;
#endif
#ifdef U5MODE
        case 5:
            descriptor->MODE    = (UxMODE_t*) &U5MODE;
            descriptor->STA     = (UxSTA_t*) &U5STA;
            descriptor->BRG     = &U5BRG;
            descriptor->RXREG   = &U5RXREG;
            descriptor->TXREG   = &U5TXREG;
            descriptor->RXR     = &U5RXR;
            descriptor->TXPV    = 0b0011;
            descriptor->rxVector  = _UART5_RX_VECTOR;
            descriptor->txVector  = _UART5_TX_VECTOR;
            descriptor->fltVector = _UART5_FAULT_VECTOR;
            return 1;
#endif
#ifdef U6MODE
        case 6:
            descriptor->MODE    = (UxMODE_t*) &U6MODE;
            descriptor->STA     = (UxSTA_t*) &U6STA;
            descriptor->BRG     = &U6BRG;
            descriptor->RXREG   = &U6RXREG;
            descriptor->TXREG   = &U6TXREG;
            descriptor->RXR     = &U6RXR;
            descriptor->TXPV    = 0b0100;
            descriptor->rxVector  = _UART6_RX_VECTOR;
            descriptor->txVector  = _UART6_TX_VECTOR;
            descriptor->fltVector = _UART6_FAULT_VECTOR;
            return 1;
#endif
        default:
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

static char UART_printBuffer[256];

uint32_t UART_termPrint(void * port, char * format, ...){
    va_list arg;
    va_start (arg, format);
    
    uint32_t length = 0;
    
    UART_PortHandle * handle = (UART_PortHandle *) port;
    if(UART_isOn(handle)){
        length = vsnprintf(UART_printBuffer, 256, format, arg);

        UART_sendString(handle, UART_printBuffer);
        while(!(handle->STA->TRMT));
    }
    
    va_end (arg);
    return length;
}