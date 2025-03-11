#ifndef uart_H
#define uart_H

#include <stdint.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "UARTconfig.h"
#include "DMA.h"
#include "DMAutils.h"

//UART Event defines
#define UART_EVT_INVALID 0x00
//module enabling/disabling events (0x1X)
#define UART_EVT_MODULE_ENABLED 0x10
#define UART_EVT_MODULE_DISABLED 0x11
//error events (0x2X)
#define UART_EVT_RX_ERROR_OVERFLOW 0x20
#define UART_EVT_RX_ERROR_FRAMING 0x21

extern volatile uint8_t UART_bootloader;
extern volatile uint32_t lastScanPosition;
extern uint8_t * UART_rxBuffer;

typedef struct __UART_PortHandle__ UartHandle_t;

UartHandle_t * UART_init(uint32_t module, uint32_t baud, volatile uint32_t* TXPinReg, uint8_t RXPinReg);


//prototype of a function that can be used as an intterupt service routine
typedef uint32_t (*UartISR_t)(UartHandle_t * handle, uint32_t flags);

uint32_t UART_setRxDMAEnabled(UartHandle_t * handle, uint32_t enabled);

uint32_t UART_termPrint(void * port, char * format, ...);
extern inline uint8_t UART_readChar(UartHandle_t * handle);
extern inline uint32_t UART_isRxDataAvailable(UartHandle_t * handle);
extern inline void UART_clearFERR(UartHandle_t * handle);
extern inline void UART_clearOERR(UartHandle_t * handle);
extern inline unsigned UART_isFERR(UartHandle_t * handle);
extern inline unsigned UART_isOERR(UartHandle_t * handle);
extern inline unsigned UART_isOn(UartHandle_t * handle);
uint32_t UART_getBaud(UartHandle_t * handle);
void UART_setBaud(UartHandle_t * handle, uint64_t newBaud);
void UART_setModuleOn(UartHandle_t * handle, uint32_t on);








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
    UartMODEbits_t UMODECLR;
    UartMODEbits_t UMODESET;
    UartMODEbits_t UMODEINV;
    
    UartSTAbits_t USTA;
    UartSTAbits_t USTACLR;
    UartSTAbits_t USTASET;
    UartSTAbits_t USTAINV;
    
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

















/*


#define UMODE handle->MODE->w
#define UMODEbits (*handle->MODE)

#define USTA handle->STA->w
#define USTAbits (*handle->STA)

#define UBRG *(handle->BRG)
#define URXR *(handle->RXR)
#define UTXR *(handle->TXR)

#define URXPinValue handle->RXPV
#define UTXPinValue handle->TXPV

#define URXReg *(handle->RXREG)
#define UTXReg *(handle->TXREG)*/

#endif