/*
 * board.c
 *
 * Board functions for IMAGO VisionSensor PV
 *
 * Copyright (C) IMAGO Technologies GmbH - <http://www.imago-technologies.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include <asm/arch/omap.h>

#define DEV_ATTR_MAX_OFFSET    5
#define DEV_ATTR_MIN_OFFSET    0

void enable_uart0_pin_mux(void);
void enable_board_pin_mux(void);
void enable_i2c0_pin_mux(void);
#endif
