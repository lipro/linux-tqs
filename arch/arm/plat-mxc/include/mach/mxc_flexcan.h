/*
 *      Copyright (C) 2009 Jan Weitzel <armlinux@phytec.de>
 *	Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_MXC_FLEXCAN
#define __ASM_ARCH_MXC_FLEXCAN

struct flexcan_platform_data {
        char *core_reg;
        char *io_reg;
        void (*xcvr_enable) (int id, int en);
        void (*active) (int id);
        void (*inactive) (int id);
};

#endif /* __ASM_ARCH_MXC_FLEXCAN  */
