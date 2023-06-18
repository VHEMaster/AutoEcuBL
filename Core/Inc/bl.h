/*
 * bl.h
 *
 *  Created on: Jun 18, 2023
 *      Author: VHEMaster
 */

#ifndef INC_BL_H_
#define INC_BL_H_

#include "main.h"
#include "xCommand.h"

typedef struct {
    uint32_t crc;
    uint32_t size;
    uint32_t check;
    uint8_t rsvd[244];
} bl_ctx_t;

typedef void (*bl_reset_func_t)(void);

void bl_init(void);
void bl_loop(void);
void bl_irq_slow_loop(void);
void bl_irq_fast_loop(void);

void bl_register_reset_callback(bl_reset_func_t func);

void bl_parse_command(sGetterHandle *xHandle, eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length);

#endif /* INC_BL_H_ */
