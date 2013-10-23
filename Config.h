/****************************************************************************
*
*   Copyright (c) 2006 Dave Hylands     <dhylands@gmail.com>
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*   See README and COPYING for more details.
*
****************************************************************************/

#if !defined( CONFIG_H )
#define CONFIG_H

#define CFG_USE_UART1               1
#define CFG_UART1_BAUD_RATE         57600
#define CFG_UART1_RX_BUFFER_SIZE    64
#define CFG_UART1_TX_BUFFER_SIZE    64
#define CFG_UART1_LF_TO_CRLF        1

#define CFG_ADC_8_BIT               0
#define CFG_ADC_POLL                0
#define CFG_ADC_AVERAGE             1

#define CFG_TIMER_MS_TICK 1

#endif  // CONFIG_H
