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
#include "printf.h"
#include "Timer.h"

typedef struct{
    UartISR_t function;
    UartHandle_t * handle;
    uint32_t enabledEventFlags;
} UartISRDescriptor_t;

static UartISRDescriptor_t isrDescriptors[UART_NUM_MODULES] = {[0 ... (UART_NUM_MODULES - 1)] = {.function = NULL, .handle = NULL}};

//initializes a UART module and returns a handle for it
//NOT: module will stay disabled until UART_setModuleOn(handle, 1); is called
UartHandle_t * UART_init(uint32_t module, uint32_t baudRate){
    //try to get memory for the handle
    UartHandle_t * ret = UART_MALLOC(sizeof(UartHandle_t));
    
    //did we get memory?
    if(ret == NULL){
        return NULL;
    }
    
    //assign register map
    ret->descriptor = &Uart_moduleMap[module - 1];
    ret->number = module;
    
    //baud rate 
    UART_setBaud(ret, baudRate);
    
    //generate an event stream
    ret->eventStream = xStreamBufferCreate(UART_EVENTBUFFER_SIZE,0);
    
    //generate semaphores for tx and reset the flags
    ret->txSemaphore = xSemaphoreCreateBinary();
    ret->txDmaSemaphore = xSemaphoreCreateBinary();
    ret->txISRFlags = 0;
    
    //rx and tx dma is disabled by default, clear the pointers
    ret->rxDMAHandle = NULL;
    
    ret->txDMAHandle = NULL;
    ret->txBuffer;
    ret->txBufferPosition = 0;
    
    return ret;
}

void UART_updateBaudForSleep(UartHandle_t * handle, uint32_t sleep){
    if(sleep){
        UART_REGS->UMODE.BRGH = 1;
        UART_REGS->BRG = (configSECONDARY_CLOCK_HZ / (4 * handle->currentBaudrate)) - 1;
    }else{
        UART_setBaud(handle, handle->currentBaudrate);
    }
}


//TODO make this function make sense... we want the maximum time resolution possible to just a fixed threshold of 250kbaud
void UART_setBaud(UartHandle_t * handle, uint64_t newBaud){
    handle->currentBaudrate = newBaud;
    if(newBaud > 250000){
        UART_REGS->UMODE.BRGH = 1;
        UART_REGS->BRG = (UART_CLK_Hz / (4 * newBaud)) - 1;
    }else{
        UART_REGS->UMODE.BRGH = 0;
        UART_REGS->BRG = (UART_CLK_Hz / (16 * newBaud)) - 1;
    }
}

uint32_t UART_getBaud(UartHandle_t * handle){
    return (UART_CLK_Hz/ (((UART_REGS->UMODE.BRGH) ? 4 : 16) * (UART_REGS->BRG+1)));
}


//sends a string directly to the output buffer, incase there is some dramatic error
//we don't use the semaphore here as this will only be called from a context where an error prevents the semaphore from being released (tx error, _general_exception_handler, vAssert, etc.)
void UART_sendString(UartHandle_t * handle, char *data){
    while((*data) != 0){
        while(!UART_isTxBufferFull(handle));
        UART_REGS->TXREG = *data++;
    }
}

//enable or disable the internal rx and tx routines
//WARNING: when disabling the rx dma the user must ensure that no task is currently trying to read from the dma buffer as it will be freed!
uint32_t UART_setInternalRWEnabled(UartHandle_t * handle, uint32_t rxEnabled, uint32_t txEnabled){
    
    //should we change something about the rx routines?
    if(rxEnabled != UART_RW_LEAVE_UNCHANGED){
        //are we switching the rx routines on?
        if(rxEnabled){
            //yes, check if it is already on. if so we can return as there's nothing to do
            if(handle->rxDMAHandle != NULL) return pdPASS;

            //rx is not enabled but user wants it to be. Allocate a new dma ringbuffer
            DMA_RingBufferHandle_t * newBuffer = DMA_createRingBuffer(UART_BUFFERSIZE, 1, UART_getRXRegPtr(handle), Uart_getRxIRQNum(handle), 1, RINGBUFFER_DIRECTION_RX);

            //we used to set the abort irq to the fault interrupt, but don't do this anymore as in case of a fault occuring all data is flushed from the buffer and lost
            //a uart error would cause our own interrupt handler to be invoked, where we could deal with the issue ourselves
            //DMA_RB_setAbortIRQ(handle->rxDMAHandle, handle->fltVector, 1);

            //enable the dma channel
            DMA_setEnabled(newBuffer->channelHandle, 1);
            
            //set up the rx interrupt to occur as long as there is data in the buffer
            UART_setRxIrqMode(handle, UART_RX_IRQ_WHEN_DATA_AVAILABLE);
            
            //assign the buffer
            handle->rxDMAHandle = newBuffer;
        }else{
            //no, free the currently present ringbuffer

            if(handle->rxDMAHandle == NULL) return pdPASS; //already free'd

            //free the buffer
            DMA_freeRingBuffer(handle->rxDMAHandle);
            handle->rxDMAHandle = NULL;
        }
    }
    
    //should we change something about the tx routines?
    if(txEnabled != UART_RW_LEAVE_UNCHANGED){
        //are we switching the rx routines on?
        if(txEnabled){
            //yes, check if it is already on. if so we can return as there's nothing to do
            if(handle->txDMAHandle != NULL) return pdPASS;
            
            //generate a tx buffer for printing
            handle->txBuffer = UART_MALLOC(UART_BUFFERSIZE);
            
            //tx is currently off and needs to be enabled. Allocate a dma channel
            DmaHandle_t * newChannel = DMA_allocateChannel();
            
            //set constant transfer attributes (src will be set once a transfer is initiated)
            DMA_setDestConfig(newChannel, UART_getTXRegPtr(handle), 1);
            DMA_setChannelAttributes(newChannel, 0, 0, 0, 0, 2);
            
            //set up interrupts
            DMA_setIRQHandler(newChannel, UART_txDmaISR, handle);
            DMA_setInterruptConfig(newChannel, 0, 0, 0, 0, 1, 0, 1, 1); //block complete, abort and error irqs enabled
            
            //and finally set up the tx interrupt to occur as long as there is space in the tx buffer
            UART_setTxIrqMode(handle, UART_TX_IRQ_WHEN_SPACE_AVAILABLE);
            
            //finally assign the channel
            handle->txDMAHandle = newChannel;
        }else{
            //no, free the currently present ringbuffer
            
            if(handle->txDMAHandle == NULL) return pdPASS; //already free'd
            
            //make sure no transmission is currently going on
            if(!xSemaphoreTake(handle->txSemaphore, portMAX_DELAY)){
                //what somehow this didn't work... we need to return
                return pdFAIL;
            }
            
            //no transmission going on, its safe to free the dma
            DMA_freeChannel(handle->txDMAHandle);
            handle->txDMAHandle = NULL;
            
            UART_FREE(handle->txBuffer);
            handle->txBuffer = NULL;
            
            //return the semaphore
            xSemaphoreGive(handle->txSemaphore);
        }
    }
}
 
void UART_isrHandler(uint32_t moduleNumber, uint32_t ifsState){
    //a uart irq just occurred, check what we need to do to handle it
    
    //is there a handle defined for the module?
    if(isrDescriptors[moduleNumber-1].handle == NULL){
        //no, switch off the timer and return
        UART_setIRQsEnabledInternal(moduleNumber-1, 0);
        
        return;
    }
    
    //first get the handle
    UartHandle_t * handle = isrDescriptors[moduleNumber-1].handle;
    
    //check what interrupt occured 
    
    uint32_t evtFlags = 0;
    
    if(ifsState & handle->descriptor->errorMask){
        //error interrupt. get the error type(s) and push a notification into the handle's event queue
        
        evtFlags |= UART_EVENTFLAG_ERROR_IRQ;
        
        //figure out what errors occurred and deal with them
        
        if(UART_REGS->USTA.FERR){ //framing error, no need to reset this
            evtFlags |= UART_EVENTFLAG_ERROR_FRAMING;
        }
        
        if(UART_REGS->USTA.PERR){ //parity error, also no need to reset this 
            evtFlags |= UART_EVENTFLAG_ERROR_PARITY;
        }
        
        if(UART_REGS->USTA.OERR){ //overflow error, this will need to be reset loosing the data currently in the buffer
            evtFlags |= UART_EVENTFLAG_ERROR_OVERFLOW;
            
            //clear the error
            UART_clearOERR(handle);
        }
        
        //then we send any events to the eventStream. We do this now as rx and tx events don't need to go there TODO: evaluate if we might not need them after all
        xStreamBufferSendFromISR(handle->eventStream, &evtFlags, 1, NULL);
    }
    
    if(ifsState & handle->descriptor->txMask){
        //tx interrupt
        evtFlags |= UART_EVENTFLAG_TX_IRQ;
    }
    
    if(ifsState & handle->descriptor->rxMask){
        //rx interrupt
        evtFlags |= UART_EVENTFLAG_RX_IRQ;
    }
    
    //now check if an isr is assigned to this module and we have an event for it
    if(isrDescriptors[moduleNumber-1].function != NULL && (evtFlags & isrDescriptors[moduleNumber-1].enabledEventFlags) > 0){
        //yep we have an isr and and event to dispatch to it
        (*isrDescriptors[moduleNumber-1].function)(handle, evtFlags);
    }
}

//interrupt related functions


//assign an interrupt routine to a module. To de-assign call with isr* = NULL
uint32_t UART_setISR(UartHandle_t * handle, UartISR_t isr){
    //are we assigning or de-assigning?
    if(isr == NULL){
        //write NULL into the isr list for the function and we're done
        isrDescriptors[handle->number-1].function = NULL;
    }else{
        //is there already a function assigned? If so we won't overwrite it
        if(isrDescriptors[handle->number-1].function != NULL) return pdFAIL;
        
        //assign the functions
        isrDescriptors[handle->number-1].function = isr;
    }
    
    return pdPASS;
}

void UART_setInterruptPriority(UartHandle_t * handle, uint32_t priority, uint32_t subPriority){
    //we will have to perform a read-modify-write on the register to update the priority
    //since this would take forever though we utilise the pics SET and CLR registers together with a mask
    
    //first we need to select the correct descriptor. If the timer is in 32bit mode all interrupt related settings come from the slave timer
    UartDescriptor_t * desc = handle->descriptor;
    
    //first we make sure the interrupt is off
    uint32_t irqsEnabled = UART_getIRQsEnabledInternal(handle->number);
    UART_setIRQsEnabledInternal(handle, 0);
    
    //then we clear the priority bits. There are 5 (hence 0b11111) and located at the offset pointed to by ipcOffset
    desc->ipcReg->CLR = 0b11111 << desc->ipcOffset;
    
    //then we generate the new priority bits
    Pic32PrioBits_t map = {.priority = priority, .subPriority = subPriority};
    
    //after that we assign the shifted mask to the SET register to complete the operation
    desc->ipcReg->SET = map.map << desc->ipcOffset;
   
    UART_setIRQsEnabledInternal(handle, irqsEnabled);
}

//external function, this only returns what events will cause the set isr to be called
uint32_t UART_getIRQsEnabled(UartHandle_t * handle){
    return isrDescriptors[handle->number-1].enabledEventFlags;
}

//function to get what iec bits are set
static uint32_t UART_getIRQsEnabledInternal(uint32_t moduleNumber){
    uint32_t ret = 0;
    
    if(Uart_moduleMap[moduleNumber - 1].iecReg->w & Uart_moduleMap[moduleNumber - 1].rxMask) ret |= UART_EVENTFLAG_RX_IRQ;
    if(Uart_moduleMap[moduleNumber - 1].iecReg->w & Uart_moduleMap[moduleNumber - 1].txMask) ret |= UART_EVENTFLAG_TX_IRQ;
    if(Uart_moduleMap[moduleNumber - 1].iecReg->w & Uart_moduleMap[moduleNumber - 1].errorMask) ret |= UART_EVENTFLAG_ERROR_IRQ;
    
    return ret;
}

//sets which interrupts are enabled. The error interrupt is always on and will always call the libraries internal isr
void UART_setIRQsEnabled(UartHandle_t * handle, uint32_t eventIrqsEnabled){
    //call the internal function, but make sure to not disable the error interrupt
    UART_setIRQsEnabledInternal(handle->number, eventIrqsEnabled | UART_EVENTFLAG_ERROR_IRQ);
    
    //then update the enabled events in the isr descriptor
    isrDescriptors[handle->number-1].enabledEventFlags = eventIrqsEnabled;
}

//internal function that can also set error irq
static void UART_setIRQsEnabledInternal(uint32_t moduleNumber, uint32_t eventIrqsEnabled){
    //write the required bits into the iec register
    if(eventIrqsEnabled & UART_EVENTFLAG_RX_IRQ) Uart_moduleMap[moduleNumber - 1].iecReg->SET = Uart_moduleMap[moduleNumber - 1].rxMask; else Uart_moduleMap[moduleNumber - 1].iecReg->CLR = Uart_moduleMap[moduleNumber - 1].rxMask;
    if(eventIrqsEnabled & UART_EVENTFLAG_TX_IRQ) Uart_moduleMap[moduleNumber - 1].iecReg->SET = Uart_moduleMap[moduleNumber - 1].txMask; else Uart_moduleMap[moduleNumber - 1].iecReg->CLR = Uart_moduleMap[moduleNumber - 1].txMask;
    if(eventIrqsEnabled & UART_EVENTFLAG_ERROR_IRQ) Uart_moduleMap[moduleNumber - 1].iecReg->SET = Uart_moduleMap[moduleNumber - 1].errorMask; else Uart_moduleMap[moduleNumber - 1].iecReg->CLR = Uart_moduleMap[moduleNumber - 1].errorMask;
}

//function that handles the DMA interrupt when transmitting data
static uint32_t UART_txDmaISR(uint32_t evt, void * data){
    
    //freeRtos magic :3
    BaseType_t wake1 = pdFALSE;
    BaseType_t wake2 = pdFALSE;

    //tx DMA interrupt just occured
    
    //first get our handle
    UartHandle_t * handle = (UartHandle_t *) data;
    
    //we don't need to check which event occured, as all would be treated almost identically anyway

    //return the dma semaphore
    xSemaphoreGiveFromISR(handle->txDmaSemaphore, &wake1);

    //clean up dma
    DMA_setIRQEnabled(handle->txDMAHandle, 0);
    if(evt & DMA_EVTFLAG_ADDRERR) DMA_abortTransfer(handle->txDMAHandle); //if some address error was detected we want to abort the transfer. Any other event thats enabled already does this by itself

    //then check if we also need to return the normal txSemaphore
    if(handle->txISRFlags & UART_TXFLAG_NON_BLOCKING_TRANSFER){
        //last transmission was a non-blocking one so we need to do the cleanup

        //clear the txEn pin if needed
        //TODO: clear txEn

        //and finally return the txSemaphore
        xSemaphoreGiveFromISR(handle->txSemaphore, &wake2);
    }
    
    //TODO: the way this is implemented right now the user won't be able to know if something went wrong with the transmission after dma was started unless a timeout occurs instead of a dma error (which is pretty unlikely)
    
    portEND_SWITCHING_ISR(wake1 || wake2);
}

//this function sends bytes from a buffer out on the uart line
//IMPORTANT: if you use this function in non-blocking mode you must make sure that the buffer does not get accessed or (even worse) freed immediately after this function returns as it is still busy sending bytes
uint32_t UART_sendBuffer(UartHandle_t * handle, char * buffer, uint32_t size, uint32_t waitTimeout, uint32_t txFlags){
    //is there anything to send and tx even enabled? If not we can just return as we can't do anything anyway
    if(!(UART_isTxEnabled(handle) && UART_isTxDMAEnabled(handle))) return 0;
    
    //next we need to get the TX semaphore if the transfer was triggered from outside the library. If this times out we also just return without transmitting anything
    //We will also only return it in the following code if the function call was made from outside of the library
    if(!(txFlags & UART_TXFLAG_INTERNAL_CALL) &&  !xSemaphoreTake(handle->txSemaphore, waitTimeout)) return 0; //TODO evaluate if the second condition is actually not run!
    
    //WARNING TO ME: from this point on any return from this function must ensure that the txSemaphore gets returned!
    
    
    
    //we also have to take the txDmaSemaphore, which is used to signal completion of a transmission
    if(!xSemaphoreTake(handle->txDmaSemaphore, waitTimeout)){ 
        //This is a state that should never be reached! The only time that the txSemaphore is returned is when a transmission has completed, at which point the txDmaSemaphore MUST also be returned
        configASSERT(0);
        
        //return the txSemaphore and go have a cry :(
        if(!(txFlags & UART_TXFLAG_INTERNAL_CALL)) if(!(txFlags & UART_TXFLAG_INTERNAL_CALL)) xSemaphoreGive(handle->txSemaphore);
        return 0;
    }
    
    //tell the dma interrupt any flags that might be set. We can safely write this here as the interrupt is still disabled
    handle->txISRFlags = txFlags;
    
    //is software txEn control enabled? If so we now set the txEn pin
    //TODO: set txEn pin
    
    //now we set up the txDMA
    DMA_setSrcConfig(handle->txDMAHandle, buffer, size);    //transmit all bytes out of our buffer
    DMA_setTransferAttributes(handle, 1, Uart_getTxIRQNum(handle), DMA_IRQ_DISABLED);    //transfer one byte on every assertion of our uart interrupt
    
    DMA_clearGloablIF(handle->txDMAHandle);     //reset the interrupt flag just incase even if it isn't persistent
    DMA_setIRQEnabled(handle->txDMAHandle, 1);  //enable interrupts
    
    DMA_setEnabled(handle->txDMAHandle, 1);     //ready the dma for a transfer
    
    //start the transfer. Further bit transmission will be triggered automatically by the tx interrupt
    DMA_forceTransfer(handle->txDMAHandle);
    
    //are we supposed to block until the transfer has finished?
    if(txFlags & UART_TXFLAG_NON_BLOCKING_TRANSFER){
        //no, just return
        //NOTE: at this point pretty much no bytes have been transferred yet as we just started the DMA.
        //      we do return as if we had sent all bytes though anyway, as they are queued for transmission and will be transmitted no matter what happens
        //      (except for a reset i guess but that would cause entirely different problems anyway lol :P)
        //
        //      We also don't return the txSemaphore as the transmission is still going on. If another task is waiting to take it it would mess up this transmission if we do
        //
        //      txEnable will also be cleared by the DMA isr
        return size;
    }
    
    //yep we should block. Just wait until the semaphore is returned
    if(!xSemaphoreTake(handle->txDmaSemaphore, UART_TIMEOUT_TX)){
        //whatever we do we want to clear the txEn asap! If txEn is used we have probably already blocked for a while at this point.
        //TODO: clear txEn pin
        
        //This state should also never be reachable! If it is reached make sure your timeout is sufficient for the longest transmission that will happen
        //We should never get here as the txDmaSemaphore is returned in the dma complete or dma error interrupt, which MUST occur after the transmission completes or fails
        configASSERT(0);
        
        //TODO: maybe check what happened to the DMA module before aborting the transfer?
        uint32_t bytesWrittenToTxBuffer = DMA_getSourcePointerValue(handle->txDMAHandle);
        
        //clean up the dma to make sure it won't accidentally start transmitting anything
        DMA_setIRQEnabled(handle->txDMAHandle, 0);  //disable interrupts, as the abort will cause a abort interrupt to be triggered
        DMA_abortTransfer(handle->txDMAHandle); //abort transfer and clear the enable bit
        
        //return the txDmaSemaphore to release the dma for another transfer
        xSemaphoreGive(handle->txDmaSemaphore);
        
        //and also return the txSemaphore
        if(!(txFlags & UART_TXFLAG_INTERNAL_CALL)) xSemaphoreGive(handle->txSemaphore);
        
        //finally we return the number of bytes written to the buffer
        //NOTE: this isn't necessarily the number of bytes transmitted. If there are still bytes in the UART buffer when we get here they would count as transmitted even though they were not (yet)
        return bytesWrittenToTxBuffer;
    }
    
    //txDma interrupt triggered :D Now check if the transfer was successful
    //TODO: add flag that states wether a transfer was successful or not
    
    //all done, clear txEn and return both semaphores :)
    //TODO: clear txEn pin
    
    xSemaphoreGive(handle->txDmaSemaphore);
    if(!(txFlags & UART_TXFLAG_INTERNAL_CALL)) xSemaphoreGive(handle->txSemaphore);
    
    return size;
}

void UART_transmitBreak(UartHandle_t * handle){
    //is tx enabled?
    if(!UART_isTxEnabled(handle)) return;
    
    //get the tx semaphore
    if(!xSemaphoreTake(handle->txSemaphore, UART_DEFAULT_SEND_TIMEOUT)){ 
        return;
    }
    
    //transmit the break
    UART_REGS->USTA.UTXBRK = 1;
    UART_REGS->TXREG = 0;
    
    //return the semaphore
    xSemaphoreGive(handle->txSemaphore);
}


//function to write a byte to the buffer
static void UART_printfOutputFunction(char character, void* arg){
    //get the handle
    UartHandle_t * handle = (UartHandle_t *) arg;
    
    //write the data to the buffer
    handle->txBuffer[handle->txBufferPosition++] = character;
    
    //did we fill it up? TODO check this when entering to allow for transmission of a full buffer from here only if another character comes after the current one
    if(handle->txBufferPosition == UART_BUFFERSIZE){
        //yes! We need to transmit the data now
        UART_sendBuffer(handle, handle->txBuffer, handle->txBufferPosition, UART_DEFAULT_SEND_TIMEOUT, UART_TXFLAG_INTERNAL_CALL);
        
        //reset the buffer pointer (data does not need to be cleared as only the number of bytes written to will be sent)
        handle->txBufferPosition = 0;
    }
}

//printf function for the UART modules
uint32_t UART_printf(UartHandle_t * handle, char * format, ...){
    //start the va list
    va_list arg;
    va_start (arg, format);
    
    //is tx even enabled?
    if(!(UART_isTxEnabled(handle) && UART_isTxDMAEnabled(handle))) return 0;
    
    //try to get the semaphore
    if(!xSemaphoreTake(handle->txSemaphore, UART_DEFAULT_SEND_TIMEOUT)) return 0;
    
    //reset the buffer
    handle->txBufferPosition = 0;
    
    //fill the buffer with the data to send. If a buffer overflow happens as we do this the contents of the buffer will be automatically sent out and the buffer reset
    uint32_t length = vfctprintf(UART_printfOutputFunction, (void*) handle, format, arg);
    
    //finally check if there is any data left in the buffer that needs to be sent out
    if(handle->txBufferPosition != 0){ 
        //yes. We can do this non-blocking and have the dma isr handle the semaphore returning
        UART_sendBuffer(handle, handle->txBuffer, handle->txBufferPosition, UART_DEFAULT_SEND_TIMEOUT, UART_TXFLAG_INTERNAL_CALL | UART_TXFLAG_NON_BLOCKING_TRANSFER);
    }else{
        //no, we can just return the semaphore right away
        xSemaphoreGive(handle->txSemaphore);
    }
    
    //end the vaList again
    va_end(arg);
    
    return length;
}




//function to write a byte to the buffer
static void UART_immediatePrintfOutputFunction(char character, void* arg){
    //get the handle
    UartHandle_t * handle = (UartHandle_t *) arg;
    
    //wait for a space in the buffer
    while(!UART_isTxBufferFull(handle));
    
    //write byte into the buffer and return
    UART_REGS->TXREG = character;
}

//printf function for the UART modules that ignores any freertos functions and just transmits out the byte right away. (Safe to use even when the rtos died)
uint32_t UART_printfImmediate(UartHandle_t * handle, char * format, ...){
    //start the va list
    va_list arg;
    va_start (arg, format);
    
    //is tx even enabled?
    if(!UART_isTxEnabled(handle)) return 0;
    
    
    //fill the buffer with the data to send. If a buffer overflow happens as we do this the contents of the buffer will be automatically sent out and the buffer reset
    uint32_t length = vfctprintf(UART_immediatePrintfOutputFunction, (void*) handle, format, arg);
    
    //end the vaList again
    va_end(arg);
    
    return length;
}

//read data out of the dma buffer into a user specified buffer
//NOTE: this function behaves slightly differently from for example a freeRtos streambuffer read. It will wait until some data is available and read it, but not wait until {size} bytes have been read
uint32_t UART_read(UartHandle_t * handle, char * buffer, uint32_t size, uint32_t timeout){
    //wait for any data to be available. If some already is it will continue right away
    if(DMA_RB_waitForData(handle->rxDMAHandle, timeout)){
        //cool, some data was received, now write it to the buffer
        return DMA_RB_read(handle->rxDMAHandle, buffer, size);
    }else{
        //timeout occured :(
        return 0;
    }
}


//definition of putchar for the printf lib, not used as there is no default module
void putchar_(char c){}