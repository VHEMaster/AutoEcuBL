/*
 * bl.c
 *
 *  Created on: Jun 18, 2023
 *      Author: VHEMaster
 */

#include "bl.h"
#include "crc.h"

#define FW_ADDRESS 0x08010000

static bl_reset_func_t bl_reset_func = NULL;

static void bl_switch_to_fw(void)
{
  bl_reset_func_t appfunc;
  uint32_t resetisr = *(__IO uint32_t*) (FW_ADDRESS + 4);

  if(bl_reset_func)
    bl_reset_func();

  appfunc = (bl_reset_func_t)resetisr;
  __set_MSP(*(__IO uint32_t*)FW_ADDRESS);
  appfunc();

  while(1) {}
}

void bl_init(void)
{

}

void bl_loop(void)
{

}

void bl_irq_slow_loop(void)
{

}

void bl_irq_fast_loop(void)
{

}

void bl_register_reset_callback(bl_reset_func_t func)
{
  bl_reset_func = func;
}

void bl_parse_command(sGetterHandle *handle, eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{

}
