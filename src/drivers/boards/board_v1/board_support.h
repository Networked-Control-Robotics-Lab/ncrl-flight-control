#ifndef __BOARD_SUPPORT_H__
#define __BOARD_SUPPORT_H__

/* function ports */
#define mavlink_puts    uart3_puts
#define mavlink_getc    uart3_getc

#define debug_link_puts uart1_puts
#define debug_link_getc uart1_getc

#define vins_mono_puts  uart6_puts

void board_init(void);

#endif
