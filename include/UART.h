#ifndef uart_H
#define uart_H

#include <stdint.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "DMA.h"
#include "DMAutils.h"
#include "Pic32Types.h"

//UART Event defines
#define UART_EVT_INVALID 0x00
//module enabling/disabling events (0x1X)
#define UART_EVT_MODULE_ENABLED 0x10
#define UART_EVT_MODULE_DISABLED 0x11
//error events (0x2X)
#define UART_EVT_RX_ERROR_OVERFLOW 0x20
#define UART_EVT_RX_ERROR_FRAMING 0x21


//translate macros to get the interrupt numbers for different events
#define Uart_getErrorIRQNum(X) X->descriptor->errorInterruptNumber
#define Uart_getRxIRQNum(X) X->descriptor->errorInterruptNumber + 1
#define Uart_getTxIRQNum(X) X->descriptor->errorInterruptNumber + 2

#define UART_DEFAULT_SEND_TIMEOUT pdMS_TO_TICKS(1000)

//timeout for waiting for a transmission to finish. This MUST be longer than the longest expected transmission time
#define UART_TIMEOUT_TX pdMS_TO_TICKS(1000)

//this flag signals the transmitter routine that the transfer should be non-blocking
#define UART_TXFLAG_NON_BLOCKING_TRANSFER 0x0001
//this flag signals the transmitter routine that it was called from inside the library and the txSemaphore is already taken/won't need to be taken by it.
//it also won't be returned by the dma routine unless a non-blocking transfer was requested
#define UART_TXFLAG_INTERNAL_CALL 0x0002

#define UART_REGS handle->descriptor->registerMap
#define UART_getRegs(handle) handle->descriptor->registerMap

#define UART_EVENTFLAG_ERROR_FRAMING  0x01
#define UART_EVENTFLAG_ERROR_PARITY   0x02
#define UART_EVENTFLAG_ERROR_OVERFLOW 0x04
#define UART_EVENTFLAG_ERROR_IRQ      0x08

#define UART_EVENTFLAG_TX_IRQ 0x10
#define UART_EVENTFLAG_RX_IRQ 0x20



#define UART_setTxIrqMode(handle, interruptMode)                UART_getRegs(handle)->USTA.UTXISEL = interruptMode

#define UART_setRxIrqMode(handle, interruptMode)                UART_getRegs(handle)->USTA.URXISEL = interruptMode

#define UART_setFlowControl(handle, pinMode, flowControlMode)   UART_getRegs(handle)->UMODE.UEN = flowControlMode; UART_getRegs(handle)->UMODE.RTSMD = pinMode
            
#define UART_setOutputInvert(handle, invertRx, invertTx)        UART_getRegs(handle).USTA.UTXINV = invertTx; UART_getRegs(handle).UMODE.RXINV = invertRx
            
#define UART_setParity(handle, parityMode)                      UART_getRegs(handle)->UMODE.PDSEL = parityMode
          
#define UART_setStopBits(handle, stopBits)                      { if(stopBits == 2) UART_getRegs(handle)->UMODE.STSEL = 1; else  UART_getRegs(handle)->UMODE.STSEL = 0; }

#define UART_isOn(handle) (UART_getRegs(handle)->UMODE.w & _U2MODE_ON_MASK)
#define UART_setOn(handle, on) (UART_getRegs(handle)->UMODE.ON = on)

#define UART_isOERR(handle) (UART_getRegs(handle)->USTA.w & _U2STA_OERR_MASK)
#define UART_isFERR(handle) (UART_getRegs(handle)->USTA.w & _U1STA_FERR_MASK)
#define UART_isPERR(handle) (UART_getRegs(handle)->USTA.w & _U1STA_PERR_MASK)

#define UART_clearOERR(handle) UART_getRegs(handle)->USTACLR = _U1STA_OERR_MASK
#define UART_clearFERR(handle) UART_getRegs(handle)->USTACLR = _U1STA_FERR_MASK
#define UART_clearPERR(handle) UART_getRegs(handle)->USTACLR = _U1STA_PERR_MASK


#define UART_getRXRegPtr(handle) &(UART_getRegs(handle)->RXREG)
#define UART_getTXRegPtr(handle) &(UART_getRegs(handle)->TXREG)

#define UART_isRxDataAvailable(handle) (UART_getRegs(handle)->USTA.w & _U1STA_URXDA_MASK)
#define UART_isTxBufferFull(handle) (UART_getRegs(handle)->USTA.w & _U1STA_UTXBF_MASK)

#define UART_readChar(handle) UART_getRegs(handle)->RXREG

#define UART_setRxEnabled(handle, enabled) UART_getRegs(handle)->USTA.URXEN = enabled
#define UART_isRxEnabled(handle) UART_getRegs(handle)->USTA.URXEN
#define UART_isRxDMAEnabled(handle) (handle->rxDMAHandle != NULL)

#define UART_setTxEnabled(handle, enabled) UART_getRegs(handle)->USTA.UTXEN = enabled;
#define UART_isTxEnabled(handle) UART_getRegs(handle)->USTA.UTXEN
#define UART_isTxDMAEnabled(handle) (handle->txDMAHandle != NULL)


#define UART_RX_IRQ_WHEN_DATA_AVAILABLE     0b00
#define UART_RX_IRQ_WHEN_HALF_FULL          0b01
#define UART_RX_IRQ_WHEN_THREE_QUARTER_FULL 0b10

#define UART_TX_IRQ_WHEN_SPACE_AVAILABLE    0b00
#define UART_TX_IRQ_WHEN_FINAL_BYTE_SENT    0b01
#define UART_TX_IRQ_WHEN_BUFFER_EMPTY       0b10

#define UART_FC_UBLCK       0b11
#define UART_FC_CTS_AND_RTS 0b10
#define UART_FC_RTS         0b01
#define UART_FC_NONE        0b00

#define UART_RW_LEAVE_UNCHANGED 0xffffffff


#define UART_PARITY_NONE 0b00
#define UART_PARITY_EVEN 0b01
#define UART_PARITY_ODD  0b10
#define UART_PARITY_9BIT 0b11


extern volatile uint8_t UART_bootloader;
extern volatile uint32_t lastScanPosition;
extern uint8_t * UART_rxBuffer;

typedef struct __UART_PortHandle__ UartHandle_t;

UartHandle_t * UART_init(uint32_t module, uint32_t baudRate);


//prototype of a function that can be used as an intterupt service routine
typedef uint32_t (*UartISR_t)(UartHandle_t * handle, uint32_t flags);


//initializes a UART module and returns a handle for it
//NOT: module will stay disabled until UART_setModuleOn(handle, 1); is called
UartHandle_t * UART_init(uint32_t module, uint32_t baudRate);

uint32_t UART_read(UartHandle_t * handle, char * buffer, uint32_t size, uint32_t timeout);
uint32_t UART_printf(UartHandle_t * handle, char * format, ...);
uint32_t UART_printfImmediate(UartHandle_t * handle, char * format, ...);
static void UART_printfOutputFunction(char character, void* arg);
void UART_transmitBreak(UartHandle_t * handle);
//this function sends bytes from a buffer out on the uart line
//IMPORTANT: if you use this function in non-blocking mode you must make sure that the buffer does not get accessed or (even worse) freed immediately after this function returns as it is still busy sending bytes
uint32_t UART_sendBuffer(UartHandle_t * handle, char * buffer, uint32_t size, uint32_t waitTimeout, uint32_t txFlags);
//function that handles the DMA interrupt when transmitting data
static uint32_t UART_txDmaISR(uint32_t evt, void * data);
//internal function that can also set error irq
static void UART_setIRQsEnabledInternal(uint32_t moduleNumber, uint32_t eventIrqsEnabled);
//sets which interrupts are enabled. The error interrupt is always on and will always call the libraries internal isr
void UART_setIRQsEnabled(UartHandle_t * handle, uint32_t eventIrqsEnabled);
//function to get what iec bits are set
static uint32_t UART_getIRQsEnabledInternal(uint32_t moduleNumber);
void UART_setInterruptPriority(UartHandle_t * handle, uint32_t priority, uint32_t subPriority);
//assign an interrupt routine to a module. To de-assign call with isr* = NULL
uint32_t UART_setISR(UartHandle_t * handle, UartISR_t isr);
static void UART_handleInterrupt(uint32_t moduleNumber, uint32_t ifsState);
//enable or disable the internal rx and tx routines
//WARNING: when disabling the rx dma the user must ensure that no task is currently trying to read from the dma buffer as it will be freed!
uint32_t UART_setInternalRWEnabled(UartHandle_t * handle, uint32_t rxEnabled, uint32_t txEnabled);
//sends a string directly to the output buffer, incase there is some dramatic error
//we don't use the semaphore here as this will only be called from a context where an error prevents the semaphore from being released (tx error, _general_exception_handler, vAssert, etc.)
void UART_sendString(UartHandle_t * handle, char *data);
uint32_t UART_getBaud(UartHandle_t * handle);
//TODO make this function make sense... we want the maximum time resolution possible to just a fixed threshold of 250kbaud
void UART_setBaud(UartHandle_t * handle, uint64_t newBaud);
void UART_updateBaudForSleep(UartHandle_t * handle, uint32_t sleep);
void UART_isrHandler(uint32_t moduleNumber, uint32_t ifsState);





typedef union {
  struct {
    uint32_t STSEL:1;
    uint32_t PDSEL:2;
    uint32_t BRGH:1;
    uint32_t RXINV:1;
    uint32_t ABAUD:1;
    uint32_t LPBACK:1;
    uint32_t WAKE:1;
    uint32_t UEN:2;
    uint32_t :1;
    uint32_t RTSMD:1;
    uint32_t IREN:1;
    uint32_t SIDL:1;
    uint32_t :1;
    uint32_t ON:1;
  };
  struct {
    uint32_t w:32;
  };
} UartMODEbits_t;

typedef union {
  struct {
    uint32_t URXDA:1;
    uint32_t OERR:1;
    uint32_t FERR:1;
    uint32_t PERR:1;
    uint32_t RIDLE:1;
    uint32_t ADDEN:1;
    uint32_t URXISEL:2;
    uint32_t TRMT:1;
    uint32_t UTXBF:1;
    uint32_t UTXEN:1;
    uint32_t UTXBRK:1;
    uint32_t URXEN:1;
    uint32_t UTXINV:1;
    uint32_t UTXISEL:2;
    uint32_t ADDR:8;
    uint32_t ADM_EN:1;
  };
  struct {
    uint32_t w:32;
  };
} UartSTAbits_t;




//UART module register map
typedef struct{
    UartMODEbits_t UMODE;
    uint32_t UMODECLR;
    uint32_t UMODESET;
    uint32_t UMODEINV;
    
    UartSTAbits_t USTA;
    uint32_t USTACLR;
    uint32_t USTASET;
    uint32_t USTAINV;
    
    uint32_t TXREG;
    uint32_t; uint32_t; uint32_t;
    
    uint32_t RXREG;
    uint32_t; uint32_t; uint32_t;
    
    uint32_t BRG;
    uint32_t BRGCLR; 
    uint32_t BRGSET; 
    uint32_t BRGINV;
} UartMap_t;





//timer descriptor, used in the timer array in UartConfig.c
typedef struct{
	UartMap_t * registerMap;
    
    Pic32SetClearMap_t * iecReg;
    Pic32SetClearMap_t * ifsReg;
    uint32_t   errorMask;
    uint32_t   txMask;
    uint32_t   rxMask;
    
    Pic32SetClearMap_t * ipcReg;
    uint32_t ipcOffset;
    
	uint32_t interruptVector;
	uint32_t errorInterruptNumber;
} UartDescriptor_t;


struct __UART_PortHandle__{
    UartDescriptor_t    *   descriptor;
    uint32_t                number;
    
    uint32_t                currentBaudrate;
    
    StreamBufferHandle_t    eventStream;
    
    SemaphoreHandle_t       txSemaphore;
    SemaphoreHandle_t       txDmaSemaphore;
    uint32_t                txISRFlags;
    
    DMA_RingBufferHandle_t * rxDMAHandle;
    DmaHandle_t *          txDMAHandle;
    
    char *                  txBuffer;
    uint32_t                txBufferPosition;
};

#endif