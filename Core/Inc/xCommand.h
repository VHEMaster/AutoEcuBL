/*
 * xCommand.h
 *
 *  Created on: Dec 6, 2020
 *      Author: denys.prokhorov
 */


#ifndef XCOMMAND_H_
#define XCOMMAND_H_

#include "main.h"
#include "defines.h"
#include "xProFIFO.h"

#define TASK_SLEEP  { osDelay(1); } // Task must give it's time to another process or just skip some time
#define MAX_PACK_LEN (768)
#define UART_DMA_BUFFER (MAX_PACK_LEN * 2)

typedef enum {
    etrNone = 0,
    etrPC,
    etrECU,
    etrCTRL,

    etrCount
} eTransChannels;

typedef struct
{
    uint8_t BufRx[UART_DMA_BUFFER];
    uint8_t BufTx[MAX_PACK_LEN];
    uint8_t xRxFifoBuf[MAX_PACK_LEN*4];
    uint8_t xTxFifoBuf[MAX_PACK_LEN*4];
    uint8_t BufSender[MAX_PACK_LEN];
    uint8_t BufParser[MAX_PACK_LEN];
    UART_HandleTypeDef * xUart;
    eTransChannels xChannels[2];

    volatile uint16_t ReceivedAckPacket;
    volatile uint16_t RetriesPacket;
    volatile uint16_t NeedAckPacket;
    volatile uint16_t NeededAckPacketId;
    volatile uint32_t LastNotAckedTime;
    volatile uint8_t TxBusy;
    volatile eTransChannels TxDest;

    uint16_t ReceivedPackets[etrCount][10];
    uint16_t ReceivedPacketId[etrCount];
    sProFIFO xTxFifo;
    sProFIFO xRxFifo;
    uint32_t dataReceiving;
    uint32_t dataLen;
    uint16_t packetId;
    uint32_t RxPointer;
    uint8_t ErrorFlag;
}sGetterHandle ALIGNED(32);

void xCommandInit(void);
void xFifosInit(void);
void xGetterInit(void);
void xGetterLoop(void);
int8_t xSender(sGetterHandle *handle, eTransChannels xChaDest, const uint8_t* xMsgPtr, uint32_t xMsgLen);
void xDmaTxIrqHandler(UART_HandleTypeDef *huart);
void xDmaRxIrqHandler(UART_HandleTypeDef *huart);
void xDmaErIrqHandler(UART_HandleTypeDef *huart);

void xSenderRaw(sGetterHandle *handle, eTransChannels xChaDest, const uint8_t* xMsgPtr, uint32_t xMsgLen);


#endif /* XCOMMAND_H_ */
