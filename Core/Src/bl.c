/*
 * bl.c
 *
 *  Created on: Jun 18, 2023
 *      Author: VHEMaster
 */

#include "bl.h"
#include "crc.h"
#include "defines.h"
#include "packets.h"
#include "delay.h"
#include "xCommand.h"
#include "failures.h"

#include "stm32f7xx_ll_system.h"

#include <string.h>

#define FW_ADDRESS  0x08010000
#define FW_SIZE     0x00070000
#define FW_CRC_POS  0x0807FF00

#define BOOTFLAG_BOOTLOADER 0xDEADBEEF
#define BOOTFLAG_NORMAL     0xABBADEAD
#define BL_CTX_CHECK        0xABCDEF89

static __IO uint32_t *bl_boot_flag_pointer = (__IO uint32_t *)0x20000000;
static uint8_t *bl_fw_pointer = (uint8_t *)FW_ADDRESS;
static bl_ctx_t *bl_ctx_pointer = (bl_ctx_t *)FW_CRC_POS;
static bl_reset_func_t bl_reset_func = NULL;
static uint32_t bl_flash_size = 0;
static uint32_t bl_reset_request_time = 0;
static uint32_t bl_reset_request_mode = 0;
static uint8_t bl_flashing = 0;
static uint8_t bl_check_bitmap[CHECK_BITMAP_SIZE] = {0};

typedef struct {
    uint32_t idcode;
    eTransChannels channel;
} bl_idcode_to_channel_t;

static bl_idcode_to_channel_t bl_idcode_to_channel[] = {
    { 0x449, etrECU },
    { 0x452, etrCTRL },
};


#define CHECK_STATUS(iserror, cod, link) \
  if((link)) { \
    bl_check_bitmap[cod >> 3] |= 1 << (cod & 7); \
    iserror |= 1; \
  }

static HAL_StatusTypeDef bl_check_fw(void)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  bl_ctx_t ctx = *bl_ctx_pointer;
  uint32_t crc;

  do {
    if(ctx.check != BL_CTX_CHECK) {
      ret = HAL_ERROR;
      break;
    }

    if(ctx.size > FW_SIZE) {
      ret = HAL_ERROR;
      break;
    }

    crc = CRC32_Generate((uint8_t *)bl_fw_pointer, ctx.size);

    if(ctx.crc != crc) {
      ret = HAL_ERROR;
      break;
    }

  } while(0);

  return ret;
}

static HAL_StatusTypeDef bl_switch_to_fw(void)
{
  HAL_StatusTypeDef ret;
  bl_reset_func_t appfunc;
  uint32_t resetisr = *(__IO uint32_t*) (FW_ADDRESS + 4);

  ret = bl_check_fw();

  if(ret != HAL_OK)
    return ret;

  if(bl_reset_func)
    bl_reset_func();

  appfunc = (bl_reset_func_t)resetisr;
  __set_MSP(*(__IO uint32_t*)FW_ADDRESS);
  appfunc();

  while(1) {}
}

void bl_init(void)
{
  uint32_t bootflag = *bl_boot_flag_pointer;
  uint32_t idcode;
  uint8_t iserror = 0;
  eTransChannels channel = etrNone;

  if(bootflag != BOOTFLAG_BOOTLOADER) {
    *bl_boot_flag_pointer = BOOTFLAG_NORMAL;
    bl_switch_to_fw();
  }

  idcode = LL_DBGMCU_GetDeviceID();
  for(int i = 0; i < ITEMSOF(bl_idcode_to_channel); i++) {
    if(bl_idcode_to_channel[i].idcode == idcode) {
      channel = bl_idcode_to_channel[i].channel;
      break;
    }
  }

  if(channel != etrNone) {
    xCommandSetChannel(channel);
  }

  memset(bl_check_bitmap, 0, sizeof(bl_check_bitmap));
  CHECK_STATUS(iserror, CheckBootLoaderMode, 1);

}

void bl_loop(void)
{

}

void bl_irq_slow_loop(void)
{
  uint32_t now = Delay_Tick;

  if(bl_reset_request_time) {
    if(DelayDiff(now, bl_reset_request_time) >= 500000) {
      if(bl_reset_request_mode == 0)
        *bl_boot_flag_pointer = BOOTFLAG_NORMAL;
      else *bl_boot_flag_pointer = BOOTFLAG_BOOTLOADER;
      NVIC_SystemReset();
      bl_reset_request_time = 0;
    }
  }
}

void bl_irq_fast_loop(void)
{

}

void bl_register_reset_callback(bl_reset_func_t func)
{
  bl_reset_func = func;
}


void bl_parse_command(sGetterHandle *xHandle, eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{
  bl_ctx_t local_bl_ctx;
  uint32_t pointer;
  uint32_t offset;
  uint32_t size;
  uint64_t data;
  HAL_StatusTypeDef res;

  switch(msgBuf[0]) {
    case PK_PingID :
      PK_Copy(&PK_Ping, msgBuf);
      PK_Pong.RandomPong = PK_Ping.RandomPing;
      PK_SendCommand(xHandle, xChaSrc, &PK_Pong, sizeof(PK_Pong));
      break;

    case PK_StatusRequestID :
      PK_Copy(&PK_StatusRequest, msgBuf);
      memcpy(PK_StatusResponse.CheckBitmap, bl_check_bitmap, CHECK_BITMAP_SIZE);
      memcpy(PK_StatusResponse.CheckBitmapRecorded, bl_check_bitmap, CHECK_BITMAP_SIZE);
      PK_SendCommand(xHandle, xChaSrc, &PK_StatusResponse, sizeof(PK_StatusResponse));
      break;

    case PK_ParametersRequestID :
      PK_Copy(&PK_ParametersRequest, msgBuf);
      memset(&PK_ParametersResponse.Parameters, 0, sizeof(PK_ParametersResponse.Parameters));
      PK_SendCommand(xHandle, xChaSrc, &PK_ParametersResponse, sizeof(PK_ParametersResponse));
      break;

    case PK_ResetStatusRequestID :
      PK_Copy(&PK_StatusRequest, msgBuf);
      PK_ResetStatusResponse.ErrorCode = 0;
      PK_SendCommand(xHandle, xChaSrc, &PK_ResetStatusResponse, sizeof(PK_ResetStatusResponse));
      break;

    case PK_ForceParametersDataID :
      PK_Copy(&PK_ForceParametersData, msgBuf);
      PK_ForceParametersDataAcknowledge.ErrorCode = 0;
      PK_SendCommand(xHandle, xChaSrc, &PK_ForceParametersDataAcknowledge, sizeof(PK_ForceParametersDataAcknowledge));
      break;

    case PK_FlashMemoryRequestID :
      PK_Copy(&PK_FlashMemoryRequest, msgBuf);
      offset = PK_FlashMemoryData.offset = PK_FlashMemoryRequest.offset;
      size = PK_FlashMemoryData.size = PK_FlashMemoryRequest.size;
      PK_FlashMemoryData.ErrorCode = 0;

      if(size + offset > FW_SIZE)
        PK_FlashMemoryData.ErrorCode = 2;

      if(size > PACKET_TABLE_MAX_SIZE)
        PK_FlashMemoryData.ErrorCode = 3;

      if(PK_FlashMemoryData.ErrorCode == 0)
      {
        memcpy(&PK_FlashMemoryData.data[0], (void *)&bl_fw_pointer[offset], size);
        memset(&PK_FlashMemoryData.data[size], 0, sizeof(PK_FlashMemoryData.data) - size);
      }
      else
      {
        memset(&PK_FlashMemoryData.data[0], 0, sizeof(PK_FlashMemoryData.data));
      }
      PK_SendCommand(xHandle, xChaSrc, &PK_FlashMemoryData, sizeof(PK_FlashMemoryData));
      break;

    case PK_FlashMemoryDataID :
      PK_Copy(&PK_FlashMemoryData, msgBuf);
      offset = PK_FlashMemoryAcknowledge.offset = PK_FlashMemoryData.offset;
      size = PK_FlashMemoryAcknowledge.size = PK_FlashMemoryData.size;
      PK_FlashMemoryAcknowledge.ErrorCode = 0;

      if(size + offset > FW_SIZE)
        PK_FlashMemoryAcknowledge.ErrorCode = 2;

      if(size > PACKET_TABLE_MAX_SIZE)
        PK_FlashMemoryAcknowledge.ErrorCode = 3;

      if(!bl_flashing)
        PK_FlashMemoryAcknowledge.ErrorCode = 4;

      if(size + offset > bl_flash_size)
        PK_FlashMemoryAcknowledge.ErrorCode = 5;

      if(PK_FlashMemoryAcknowledge.ErrorCode == 0) {
        pointer = 0;
        res = HAL_OK;
        for(; pointer < size && res == HAL_OK; pointer += sizeof(uint64_t)) {
          memcpy(&data, &PK_FlashMemoryData.data[pointer], sizeof(uint64_t));
          res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)&bl_fw_pointer[offset + pointer], data);
        }
        for(; pointer < size && res == HAL_OK; pointer += sizeof(uint32_t)) {
          memcpy(&data, &PK_FlashMemoryData.data[pointer], sizeof(uint32_t));
          res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&bl_fw_pointer[offset + pointer], (uint32_t)data);
        }
        for(; pointer < size && res == HAL_OK; pointer += sizeof(uint16_t)) {
          memcpy(&data, &PK_FlashMemoryData.data[pointer], sizeof(uint16_t));
          res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&bl_fw_pointer[offset + pointer], (uint16_t)data);
        }
        for(; pointer < size && res == HAL_OK; pointer += sizeof(uint8_t)) {
          memcpy(&data, &PK_FlashMemoryData.data[pointer], sizeof(uint8_t));
          res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)&bl_fw_pointer[offset + pointer], (uint8_t)data);
        }
        if(res != HAL_OK) {
          PK_FlashMemoryAcknowledge.ErrorCode = 10 + res;
        }
      }

      PK_SendCommand(xHandle, xChaSrc, &PK_FlashMemoryAcknowledge, sizeof(PK_FlashMemoryAcknowledge));
      break;

    case PK_FlashBeginRequestID :
      PK_Copy(&PK_FlashBeginRequest, msgBuf);
      size = PK_FlashBeginResponse.size = PK_FlashBeginRequest.size;
      PK_FlashBeginResponse.ErrorCode = 0;

      if(size > FW_SIZE)
        PK_FlashBeginResponse.ErrorCode = 2;

      if(bl_flashing)
        PK_FlashBeginResponse.ErrorCode = 4;

      if(PK_FlashBeginResponse.ErrorCode == 0) {
        bl_flashing = 1;
        bl_flash_size = size;
      }

      PK_SendCommand(xHandle, xChaSrc, &PK_FlashBeginResponse, sizeof(PK_FlashBeginResponse));
      break;

    case PK_FlashFinishRequestID :
      PK_Copy(&PK_FlashFinishRequest, msgBuf);
      PK_FlashFinishResponse.ErrorCode = 0;

      if(!bl_flashing)
        PK_FlashFinishResponse.ErrorCode = 4;

      if(PK_FlashFinishResponse.ErrorCode == 0) {
        PK_FlashFinishResponse.crc = CRC32_Generate((uint8_t *)bl_fw_pointer, bl_flash_size);


        local_bl_ctx.check = BL_CTX_CHECK;
        local_bl_ctx.size = bl_flash_size;
        local_bl_ctx.crc = PK_FlashFinishResponse.crc;

        res = HAL_OK;
        for(pointer = 0; pointer < sizeof(bl_ctx_t) && res == HAL_OK; pointer += sizeof(uint32_t)) {
          memcpy(&data, &((uint8_t *)&local_bl_ctx)[pointer], sizeof(uint32_t));
          res = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&((uint8_t *)bl_ctx_pointer)[pointer], (uint32_t)data);
        }

        if(res != HAL_OK) {
          PK_FlashFinishResponse.ErrorCode = 10 + res;
        }
      }

      if(PK_FlashFinishResponse.ErrorCode == 0) {
        bl_flashing = 0;
        bl_flash_size = 0;
      }

      PK_SendCommand(xHandle, xChaSrc, &PK_FlashFinishResponse, sizeof(PK_FlashFinishResponse));
      break;

    case PK_ResetRequestID :
      PK_Copy(&PK_ResetRequest, msgBuf);
      PK_ResetResponse.mode = PK_ResetRequest.mode;
      PK_ResetResponse.ErrorCode = 0;

      bl_reset_request_mode = PK_ResetResponse.mode;
      bl_reset_request_time = Delay_Tick;
      if(!bl_reset_request_time)
        bl_reset_request_time++;

      PK_SendCommand(xHandle, xChaSrc, &PK_ResetResponse, sizeof(PK_ResetResponse));
      break;

    default:
      break;
  }
}
