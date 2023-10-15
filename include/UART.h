#ifndef uart_H
#define uart_H

#include <stdint.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "UARTconfig.h"
#include "DMA.h"
#include "DMAutils.h"

extern volatile uint8_t UART_bootloader;
extern volatile uint32_t lastScanPosition;
extern uint8_t * UART_rxBuffer;

typedef struct __UART_PortDescriptor__ UART_PortHandle;

UART_PortHandle * UART_init(uint32_t module, uint32_t baud, volatile uint32_t* TXPinReg, uint8_t RXPinReg);

uint32_t UART_setRxDMAEnabled(UART_PortHandle * handle, uint32_t enabled);

uint32_t UART_termPrint(void * port, char * format, ...);
extern inline uint8_t UART_readChar(UART_PortHandle * handle);
extern inline uint32_t UART_available(UART_PortHandle * handle);
extern inline void UART_clearFERR(UART_PortHandle * handle);
extern inline void UART_clearOERR(UART_PortHandle * handle);
extern inline unsigned UART_isFERR(UART_PortHandle * handle);
extern inline unsigned UART_isOERR(UART_PortHandle * handle);
uint32_t UART_getBaud(UART_PortHandle * handle);
void UART_setBaud(UART_PortHandle * handle, uint64_t newBaud);
void UART_setModuleOn(UART_PortHandle * handle, uint32_t on);

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
    uint32_t :1;
    uint32_t PDSEL0:1;
    uint32_t PDSEL1:1;
    uint32_t :5;
    uint32_t UEN0:1;
    uint32_t UEN1:1;
  };
  struct {
    uint32_t :13;
    uint32_t USIDL:1;
    uint32_t :1;
    uint32_t UARTEN:1;
  };
  struct {
    uint32_t w:32;
  };
} UxMODE_t;

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
} UxSTA_t;

struct __UART_PortDescriptor__{
    volatile UxMODE_t *     MODE;
    volatile UxSTA_t  *     STA;
    
    volatile uint32_t *     BRG;
    volatile uint32_t *     RXREG;
    volatile uint32_t *     TXREG;
    
    volatile uint32_t *     RXR;
    volatile uint32_t *     TXR;
    
    uint32_t rxVector;
    uint32_t txVector;
    uint32_t fltVector;
    
    uint32_t RXPV;
    uint32_t TXPV;
    
    StreamBufferHandle_t    rxStream;
    StreamBufferHandle_t    txStream;
    
    uint32_t                rxRunning;
    uint32_t                txRunning;
    
    uint32_t                currentBaudrate;
    
    DMA_RINGBUFFERHANDLE_t * rxDMAHandle;
    DMA_HANDLE_t *          txDMAHandle;
};

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
#define UTXReg *(handle->TXREG)

#endif