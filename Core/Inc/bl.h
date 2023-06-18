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

void bl_init(void);
void bl_loop(void);
void bl_irq_slow_loop(void);
void bl_irq_fast_loop(void);
void bl_parse_command(sGetterHandle *handle, eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length);

#endif /* INC_BL_H_ */
