/*
 * SuperH on-chip serial module support.  (SCI with no FIFO / with FIFO)
 *
 *  Copyright (C) 2002 - 2011  Paul Mundt
 *  Modified to support SH7720 SCIF. Markus Brunner, Mark Jonas (Jul 2007).
 *
 * based off of the old drivers/char/sh-scif.c by:
 *
 *   Copyright (C) 1999, 2000  Niibe Yutaka
 *   Copyright (C) 2000  Sugioka Toshinobu
 *   Modified to support multiple serial ports. Stuart Menefy (May 2000).
 *   Modified to support SecureEdge. David McCullough (2002)
 *   Modified to support SH7300 SCIF. Takashi Kusuda (Jun 2003).
 *   Removed SH7300 support (Jul 2007).
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#if defined(CONFIG_SERIAL_SH_SCI_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#undef DEBUG

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/sysrq.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/platform_device.h>
#include <linux/serial_sci.h>
#include <linux/notifier.h>
#include <linux/pm_runtime.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#ifdef CONFIG_SUPERH
#include <asm/sh_bios.h>
#endif

#include <asm/sc1000.h>
#include "sh-sci-rs485.h"

struct scif_port {
	struct uart_port	port;

	/* Platform configuration */
	struct plat_sci_port	*cfg;

	/* Break timer */
	struct timer_list	break_timer;
	int			break_flag;

	/* Interface clock */
	struct clk		*iclk;
	/* Function clock */
	struct clk		*fclk;

	char			*irqstr[SCIx_NR_IRQS];

	struct dma_chan			*chan_tx;
	struct dma_chan			*chan_rx;

#ifdef CONFIG_SERIAL_SH_SCI_DMA
	struct dma_async_tx_descriptor	*desc_tx;
	struct dma_async_tx_descriptor	*desc_rx[2];
	dma_cookie_t			cookie_tx;
	dma_cookie_t			cookie_rx[2];
	dma_cookie_t			active_rx;
	struct scatterlist		sg_tx;
	unsigned int			sg_len_tx;
	struct scatterlist		sg_rx[2];
	size_t				buf_len_rx;
	struct sh_dmae_slave		param_tx;
	struct sh_dmae_slave		param_rx;
	struct work_struct		work_tx;
	struct work_struct		work_rx;
	struct timer_list		rx_timer;
	unsigned int			rx_timeout;
#endif

	struct notifier_block		freq_transition;

#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
	unsigned short saved_smr;
	unsigned short saved_fcr;
	unsigned char saved_brr;
#endif
};

/* Function prototypes */
static void scif_start_tx(struct uart_port *port);
static void scif_stop_tx(struct uart_port *port);
static void scif_start_rx(struct uart_port *port);




/* only one rs485 port */
#define SCI_NPORTS 2 /* CONFIG_SERIAL_SH_SCI_NR_UARTS */

static struct scif_port scif_ports[SCI_NPORTS];
static struct uart_driver scif_uart_driver;

static inline struct scif_port *
to_scif_port(struct uart_port *uart)
{
	return container_of(uart, struct scif_port, port);
}

struct plat_scif_reg {
	u8 offset, size;
};

/* Helper for invalidating specific entries of an inherited map. */
#define scif_reg_invalid	{ .offset = 0, .size = 0 }

static struct plat_scif_reg scif_regmap[SCIx_NR_REGTYPES][SCIx_NR_REGS] = {
	[SCIx_PROBE_REGTYPE] = {
		[0 ... SCIx_NR_REGS - 1] = scif_reg_invalid,
	},

	/*
	 * Common SCI definitions, dependent on the port's regshift
	 * value.
	 */
        [SCIx_SCI_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00,  8 },
                [SH_SCI_SCBRR]          = { 0x01,  8 },
                [SH_SCI_SCSCR]          = { 0x02,  8 },
                [SH_SCI_SCxTDR]         = { 0x03,  8 },
                [SH_SCI_SCxSR]          = { 0x04,  8 },
                [SH_SCI_SCxRDR]         = { 0x05,  8 },
                [SH_SCI_SCFCR]          = scif_reg_invalid,
                [SH_SCI_SCFDR]          = scif_reg_invalid,
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = scif_reg_invalid,
                [SH_SCI_SCLSR]          = scif_reg_invalid,
        },

        /*
         * Common definitions for legacy IrDA ports, dependent on
         * regshift value.
         */
        [SCIx_IRDA_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00,  8 },
                [SH_SCI_SCBRR]          = { 0x01,  8 },
                [SH_SCI_SCSCR]          = { 0x02,  8 },
                [SH_SCI_SCxTDR]         = { 0x03,  8 },
                [SH_SCI_SCxSR]          = { 0x04,  8 },
                [SH_SCI_SCxRDR]         = { 0x05,  8 },
                [SH_SCI_SCFCR]          = { 0x06,  8 },
                [SH_SCI_SCFDR]          = { 0x07, 16 },
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = scif_reg_invalid,
                [SH_SCI_SCLSR]          = scif_reg_invalid,
        },

        /*
         * Common SCIFA definitions.
         */
        [SCIx_SCIFA_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00, 16 },
                [SH_SCI_SCBRR]          = { 0x04,  8 },
                [SH_SCI_SCSCR]          = { 0x08, 16 },
                [SH_SCI_SCxTDR]         = { 0x20,  8 },
                [SH_SCI_SCxSR]          = { 0x14, 16 },
                [SH_SCI_SCxRDR]         = { 0x24,  8 },
                [SH_SCI_SCFCR]          = { 0x18, 16 },
                [SH_SCI_SCFDR]          = { 0x1c, 16 },
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = scif_reg_invalid,
                [SH_SCI_SCLSR]          = scif_reg_invalid,
        },

        /*
         * Common SCIFB definitions.
         */
        [SCIx_SCIFB_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00, 16 },
                [SH_SCI_SCBRR]          = { 0x04,  8 },
                [SH_SCI_SCSCR]          = { 0x08, 16 },
                [SH_SCI_SCxTDR]         = { 0x40,  8 },
                [SH_SCI_SCxSR]          = { 0x14, 16 },
                [SH_SCI_SCxRDR]         = { 0x60,  8 },
                [SH_SCI_SCFCR]          = { 0x18, 16 },
                [SH_SCI_SCFDR]          = { 0x1c, 16 },
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = scif_reg_invalid,
                [SH_SCI_SCLSR]          = scif_reg_invalid,
        },

        /*
         * Common SH-2(A) SCIF definitions for ports with FIFO data
         * count registers.
         */
        [SCIx_SH2_SCIF_FIFODATA_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00, 16 },
                [SH_SCI_SCBRR]          = { 0x04,  8 },
                [SH_SCI_SCSCR]          = { 0x08, 16 },
                [SH_SCI_SCxTDR]         = { 0x0c,  8 },
                [SH_SCI_SCxSR]          = { 0x10, 16 },
                [SH_SCI_SCxRDR]         = { 0x14,  8 },
                [SH_SCI_SCFCR]          = { 0x18, 16 },
                [SH_SCI_SCFDR]          = { 0x1c, 16 },
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = { 0x20, 16 },
                [SH_SCI_SCLSR]          = { 0x24, 16 },
        },

        /*
         * Common SH-3 SCIF definitions.
         */
        [SCIx_SH3_SCIF_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00,  8 },
                [SH_SCI_SCBRR]          = { 0x02,  8 },
                [SH_SCI_SCSCR]          = { 0x04,  8 },
                [SH_SCI_SCxTDR]         = { 0x06,  8 },
                [SH_SCI_SCxSR]          = { 0x08, 16 },
                [SH_SCI_SCxRDR]         = { 0x0a,  8 },
                [SH_SCI_SCFCR]          = { 0x0c,  8 },
                [SH_SCI_SCFDR]          = { 0x0e, 16 },
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = scif_reg_invalid,
                [SH_SCI_SCLSR]          = scif_reg_invalid,
        },

        /*
         * Common SH-4(A) SCIF(B) definitions.
         */
        [SCIx_SH4_SCIF_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00, 16 },
                [SH_SCI_SCBRR]          = { 0x04,  8 },
                [SH_SCI_SCSCR]          = { 0x08, 16 },
                [SH_SCI_SCxTDR]         = { 0x0c,  8 },
                [SH_SCI_SCxSR]          = { 0x10, 16 },
                [SH_SCI_SCxRDR]         = { 0x14,  8 },
                [SH_SCI_SCFCR]          = { 0x18, 16 },
                [SH_SCI_SCFDR]          = { 0x1c, 16 },
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = { 0x20, 16 },
                [SH_SCI_SCLSR]          = { 0x24, 16 },
        },

        /*
         * Common SH-4(A) SCIF(B) definitions for ports without an SH_SCI_SCSPTR
         * register.
         */
        [SCIx_SH4_SCIF_NO_SCSPTR_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00, 16 },
                [SH_SCI_SCBRR]          = { 0x04,  8 },
                [SH_SCI_SCSCR]          = { 0x08, 16 },
                [SH_SCI_SCxTDR]         = { 0x0c,  8 },
                [SH_SCI_SCxSR]          = { 0x10, 16 },
                [SH_SCI_SCxRDR]         = { 0x14,  8 },
                [SH_SCI_SCFCR]          = { 0x18, 16 },
                [SH_SCI_SCFDR]          = { 0x1c, 16 },
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = scif_reg_invalid,
                [SH_SCI_SCLSR]          = { 0x24, 16 },
        },

        /*
         * Common SH-4(A) SCIF(B) definitions for ports with FIFO data
         * count registers.
         */
        [SCIx_SH4_SCIF_FIFODATA_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00, 16 },
                [SH_SCI_SCBRR]          = { 0x04,  8 },
                [SH_SCI_SCSCR]          = { 0x08, 16 },
                [SH_SCI_SCxTDR]         = { 0x0c,  8 },
                [SH_SCI_SCxSR]          = { 0x10, 16 },
                [SH_SCI_SCxRDR]         = { 0x14,  8 },
                [SH_SCI_SCFCR]          = { 0x18, 16 },
                [SH_SCI_SCFDR]          = { 0x1c, 16 },
                [SH_SCI_SCTFDR]         = { 0x1c, 16 }, /* aliased to SCFDR */
                [SH_SCI_SCRFDR]         = { 0x20, 16 },
                [SH_SCI_SCSPTR]         = { 0x24, 16 },
                [SH_SCI_SCLSR]          = { 0x28, 16 },
        },

        /*
         * SH7705-style SCIF(B) ports, lacking both SH_SCI_SCSPTR and SH_SCI_SCLSR
         * registers.
         */
        [SCIx_SH7705_SCIF_REGTYPE] = {
                [SH_SCI_SCSMR]          = { 0x00, 16 },
                [SH_SCI_SCBRR]          = { 0x04,  8 },
                [SH_SCI_SCSCR]          = { 0x08, 16 },
                [SH_SCI_SCxTDR]         = { 0x20,  8 },
                [SH_SCI_SCxSR]          = { 0x14, 16 },
                [SH_SCI_SCxRDR]         = { 0x24,  8 },
                [SH_SCI_SCFCR]          = { 0x18, 16 },
                [SH_SCI_SCFDR]          = { 0x1c, 16 },
                [SH_SCI_SCTFDR]         = scif_reg_invalid,
                [SH_SCI_SCRFDR]         = scif_reg_invalid,
                [SH_SCI_SCSPTR]         = scif_reg_invalid,
                [SH_SCI_SCLSR]          = scif_reg_invalid,
        },
};



// position of 485 driver enable pin
#define BIT_485TRM      (1 << 5)        
// 485 driver enable register_console
#define REG_485_SW      PORT_PEDR
// Bit5 Port E output
#define PIN_INIT_485TR  ctrl_outw((ctrl_inw(PORT_PECR) & 0xf3ff) | 0x0400, PORT_PECR)
// enable RS485 transmitter
#define PIN_ENAB_485TR  ctrl_outb(ctrl_inb(REG_485_SW) & ~BIT_485TRM, REG_485_SW)
// disable RS485 transmitter
#define PIN_DISA_485TR  ctrl_outb(ctrl_inb(REG_485_SW) |  BIT_485TRM, REG_485_SW)
// check if transmitter is enabled
#define PIN_IS_ENAB_485 (!(ctrl_inb(REG_485_SW) & BIT_485TRM))

/* Timer Register */
#define TIMER1_IRQ              17
// Timer Start Register, Byte access
#define TMU_TSTR                0xfffffe92
// Timer Constant Register, Long access
#define TMU1_TCOR               0xfffffea0
// Timer Count Regiser, Long access
#define TMU1_TCNT               0xfffffea4
// Timer Control Register, Word access
#define TMU1_TCR                0xfffffea8
// Timer 1 start bit
#define TMU_TSTR_BIT_TMU1       (1 << 1)
// Interrupt enable
//#define TMU1_TCR_INIT           0x0020
#define TMU1_TCR_INIT           0x0021
// Interrupt disable
#define TMU1_TCR_DISAB_INT      0x0001

// number of timer ticks that are NOT counted during timer count updates
#define TMTICKS_FOR_UPDATE      (-69*4)
// number of timer ticks after last bytes has been sent
#define TMTICKS_KEEPENAB        0
static int tmticks_per_byte = -1;



#define LOW 0
#define HIGH 1


#define SET_PIN(port,pin,state) do{ \
    uint8_t val=ctrl_inb(PORT_P ## port ## DR); \
    if(state){ \
        ctrl_outb(val|(1<<(pin)), PORT_P ## port ## DR); \
    } else { \
        ctrl_outb(val&~(1<<(pin)), PORT_P ## port ## DR); \
    } \
} while(0)

#define DBG_TGL(u) {int i;for(i=0;i<u;i++){SET_PIN(E,0,HIGH);SET_PIN(E,0,LOW);}udelay(2);}







#define scif_getreg(up, offset)		(scif_regmap[to_scif_port(up)->cfg->regtype] + offset)

/*
 * The "offset" here is rather misleading, in that it refers to an enum
 * value relative to the port mapping rather than the fixed offset
 * itself, which needs to be manually retrieved from the platform's
 * register map for the given port.
 */
static unsigned int scif_serial_in(struct uart_port *p, int offset)
{
	struct plat_scif_reg *reg = scif_getreg(p, offset);
//printk(KERN_DEBUG"offset %d membase %p regtype %d\n",offset,p->membase,to_scif_port(p)->cfg->regtype);
	if (reg->size == 8)
		return ioread8(p->membase + (reg->offset << p->regshift));
	else if (reg->size == 16)
		return ioread16(p->membase + (reg->offset << p->regshift));
	else
		WARN(1, "Invalid register access\n");

	return 0;
}

static void scif_serial_out(struct uart_port *p, int offset, int value)
{
	struct plat_scif_reg *reg = scif_getreg(p, offset);
//printk(KERN_DEBUG"offset %d membase %p\n",offset,p->membase);
	if (reg->size == 8)
		iowrite8(value, p->membase + (reg->offset << p->regshift));
	else if (reg->size == 16)
		iowrite16(value, p->membase + (reg->offset << p->regshift));
	else
		WARN(1, "Invalid register access\n");
}

#define scif_in(up, offset)		(up->serial_in(up, offset))
#define scif_out(up, offset, value)	(up->serial_out(up, offset, value))

static int scif_probe_regmap(struct plat_sci_port *cfg)
{
	switch (cfg->type) {
	case PORT_SCI:
		cfg->regtype = SCIx_SCI_REGTYPE;
		break;
	case PORT_IRDA:
		cfg->regtype = SCIx_IRDA_REGTYPE;
		break;
	case PORT_SCIFA:
		cfg->regtype = SCIx_SCIFA_REGTYPE;
		break;
	case PORT_SCIFB:
		cfg->regtype = SCIx_SCIFB_REGTYPE;
		break;
	case PORT_SCIF:
		/*
		 * The SH-4 is a bit of a misnomer here, although that's
		 * where this particular port layout originated. This
		 * configuration (or some slight variation thereof)
		 * remains the dominant model for all SCIFs.
		 */
		cfg->regtype = SCIx_SH4_SCIF_REGTYPE;
		break;
	default:
		printk(KERN_ERR "Can't probe register map for given port\n");
		return -EINVAL;
	}

	return 0;
}

static void scif_port_enable(struct scif_port *scif_port)
{
	if (!scif_port->port.dev)
		return;

	pm_runtime_get_sync(scif_port->port.dev);

	clk_enable(scif_port->iclk);
	scif_port->port.uartclk = clk_get_rate(scif_port->iclk);
	clk_enable(scif_port->fclk);
}

static void scif_port_disable(struct scif_port *scif_port)
{
	if (!scif_port->port.dev)
		return;

	clk_disable(scif_port->fclk);
	clk_disable(scif_port->iclk);

	pm_runtime_put_sync(scif_port->port.dev);
}

#if defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_SERIAL_SH_SCI_CONSOLE)

#ifdef CONFIG_CONSOLE_POLL
static int scif_poll_get_char(struct uart_port *port)
{
	unsigned short status;
	int c;

	do {
		status = scif_in(port, SH_SCI_SCxSR);
		if (status & SCxSR_ERRORS(port)) {
			scif_out(port, SH_SCI_SCxSR, SCxSR_ERROR_CLEAR(port));
			continue;
		}
		break;
	} while (1);

	if (!(status & SCxSR_RDxF(port)))
		return NO_POLL_CHAR;

	c = scif_in(port, SH_SCI_SCxRDR);

	/* Dummy read */
	scif_in(port, SH_SCI_SCxSR);
	scif_out(port, SH_SCI_SCxSR, SCxSR_RDxF_CLEAR(port));

	return c;
}
#endif

static void scif_poll_put_char(struct uart_port *port, unsigned char c)
{
	unsigned short status;

	do {
		status = scif_in(port, SH_SCI_SCxSR);
	} while (!(status & SCxSR_TDxE(port)));

	scif_out(port, SH_SCI_SCxTDR, c);
	scif_out(port, SH_SCI_SCxSR, SCxSR_TDxE_CLEAR(port) & ~SCxSR_TEND(port));
}
#endif /* CONFIG_CONSOLE_POLL || CONFIG_SERIAL_SH_SCI_CONSOLE */

static void scif_init_pins(struct uart_port *port, unsigned int cflag)
{
	struct scif_port *s = to_scif_port(port);
	struct plat_scif_reg *reg = scif_regmap[s->cfg->regtype] + SH_SCI_SCSPTR;

	/*
	 * Use port-specific handler if provided.
	 */
	if (s->cfg->ops && s->cfg->ops->init_pins) {
		s->cfg->ops->init_pins(port, cflag);
		return;
	}

	/*
	 * For the generic path SH_SCI_SCSPTR is necessary. Bail out if that's
	 * unavailable, too.
	 */
	if (!reg->size)
		return;

	if (!(cflag & CRTSCTS))
		scif_out(port, SH_SCI_SCSPTR, 0x0080); /* Set RTS = 1 */
}

static int scif_txfill(struct uart_port *port)
{
	struct plat_scif_reg *reg;

	reg = scif_getreg(port, SH_SCI_SCTFDR);
	if (reg->size)
		return scif_in(port, SH_SCI_SCTFDR) & 0xff;

	reg = scif_getreg(port, SH_SCI_SCFDR);
	if (reg->size)
		return scif_in(port, SH_SCI_SCFDR) >> 8;

	return !(scif_in(port, SH_SCI_SCxSR) & SCI_TDRE);
}

static int scif_txroom(struct uart_port *port)
{
	return port->fifosize - scif_txfill(port);
}

static int scif_rxfill(struct uart_port *port)
{
	struct plat_scif_reg *reg;

	reg = scif_getreg(port, SH_SCI_SCRFDR);
	if (reg->size)
		return scif_in(port, SH_SCI_SCRFDR) & 0xff;

	reg = scif_getreg(port, SH_SCI_SCFDR);
	if (reg->size)
		return scif_in(port, SH_SCI_SCFDR) & ((port->fifosize << 1) - 1);

	return (scif_in(port, SH_SCI_SCxSR) & SCxSR_RDxF(port)) != 0;
}

/*
 * SCI helper for checking the state of the muxed port/RXD pins.
 */
static inline int scif_rxd_in(struct uart_port *port)
{
	struct scif_port *s = to_scif_port(port);

	if (s->cfg->port_reg <= 0)
		return 1;

	return !!__raw_readb(s->cfg->port_reg);
}

/* ********************************************************************** *
 *                   the interrupt related routines                       *
 * ********************************************************************** */

static void scif_transmit_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int stopped = uart_tx_stopped(port);
	unsigned short status;
	unsigned short ctrl;
	int count,cnt_Tx;
        unsigned long tctele;

	status = scif_in(port, SH_SCI_SCxSR);
	if (!(status & SCxSR_TDxE(port))) {
		ctrl = scif_in(port, SH_SCI_SCSCR);
		if (uart_circ_empty(xmit))
			ctrl &= ~SCSCR_TIE;
		else
			ctrl |= SCSCR_TIE;
		scif_out(port, SH_SCI_SCSCR, ctrl);
		return;
	}

	count = scif_txroom(port);
        cnt_Tx = 0;
        
        PIN_ENAB_485TR; // enable RS485 trans
        //ctrl_outw(TMU1_TCR_INIT, TMU1_TCR);     // init timer control
        tctele = ctrl_inl(TMU1_TCNT);
        
        ctrl_outb(ctrl_inb(TMU_TSTR) & ~TMU_TSTR_BIT_TMU1, TMU_TSTR);   // Stop Timer 1 for update (assume this takes TMTICKS_FOR_UPDATE)
	do {
		unsigned char c;
                
		if (port->x_char) {
			c = port->x_char;
			port->x_char = 0;
		} else if (!uart_circ_empty(xmit) && !stopped) {
			c = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		} else {
			break;
		}
		
                scif_out(port, SH_SCI_SCxTDR, c);
                
                tctele+=tmticks_per_byte;
		port->icount.tx++;
                
	} while (--count > 0);

        ctrl_outl(tctele/* + *//*79*tc *//*4266 *//* tmticks_per_byte*/ /*4320 */ , TMU1_TCNT); // write to timer count
        ctrl_outw(TMU1_TCR_INIT, TMU1_TCR);
        ctrl_outb(ctrl_inb(TMU_TSTR) | TMU_TSTR_BIT_TMU1, TMU_TSTR);    // Start Timer 1 again after update
        
	scif_out(port, SH_SCI_SCxSR, SCxSR_TDxE_CLEAR(port));

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
	if (uart_circ_empty(xmit)) {
		scif_stop_tx(port);
	} else {
		ctrl = scif_in(port, SH_SCI_SCSCR);

		if (port->type != PORT_SCI) {
			scif_in(port, SH_SCI_SCxSR); /* Dummy read */
			scif_out(port, SH_SCI_SCxSR, SCxSR_TDxE_CLEAR(port));
		}

		ctrl |= SCSCR_TIE;
		scif_out(port, SH_SCI_SCSCR, ctrl);
	}
}

/* On SH3, SCIF may read end-of-break as a space->mark char */
#define STEPFN(c)  ({int __c = (c); (((__c-1)|(__c)) == -1); })

static void scif_receive_chars(struct uart_port *port)
{
	struct scif_port *scif_port = to_scif_port(port);
	struct tty_struct *tty = port->state->port.tty;
	int i, count, copied = 0;
	unsigned short status;
	unsigned char flag;

	status = scif_in(port, SH_SCI_SCxSR);
	if (!(status & SCxSR_RDxF(port)))
		return;

	while (1) {
		/* Don't copy more bytes than there is room for in the buffer */
		count = tty_buffer_request_room(tty, scif_rxfill(port));

		/* If for any reason we can't copy more data, we're done! */
		if (count == 0)
			break;

		if (port->type == PORT_SCI) {
			char c = scif_in(port, SH_SCI_SCxRDR);
			if (uart_handle_sysrq_char(port, c) ||
			    scif_port->break_flag)
				count = 0;
			else
				tty_insert_flip_char(tty, c, TTY_NORMAL);
		} else {
			for (i = 0; i < count; i++) {
				char c = scif_in(port, SH_SCI_SCxRDR);
				status = scif_in(port, SH_SCI_SCxSR);
#if defined(CONFIG_CPU_SH3)
				/* Skip "chars" during break */
				if (scif_port->break_flag) {
					if ((c == 0) &&
					    (status & SCxSR_FER(port))) {
						count--; i--;
						continue;
					}

					/* Nonzero => end-of-break */
					dev_dbg(port->dev, "debounce<%02x>\n", c);
					scif_port->break_flag = 0;

					if (STEPFN(c)) {
						count--; i--;
						continue;
					}
				}
#endif /* CONFIG_CPU_SH3 */
				if (uart_handle_sysrq_char(port, c)) {
					count--; i--;
					continue;
				}

				/* Store data and status */
				if (status & SCxSR_FER(port)) {
					flag = TTY_FRAME;
					dev_notice(port->dev, "frame error\n");
				} else if (status & SCxSR_PER(port)) {
					flag = TTY_PARITY;
					dev_notice(port->dev, "parity error\n");
				} else
					flag = TTY_NORMAL;

				tty_insert_flip_char(tty, c, flag);
			}
		}

		scif_in(port, SH_SCI_SCxSR); /* dummy read */
		scif_out(port, SH_SCI_SCxSR, SCxSR_RDxF_CLEAR(port));

		copied += count;
		port->icount.rx += count;
	}

	if (copied) {
		/* Tell the rest of the system the news. New characters! */
		tty_flip_buffer_push(tty);
	} else {
		scif_in(port, SH_SCI_SCxSR); /* dummy read */
		scif_out(port, SH_SCI_SCxSR, SCxSR_RDxF_CLEAR(port));
	}
}

#define SCI_BREAK_JIFFIES (HZ/20)

/*
 * The scif generates interrupts during the break,
 * 1 per millisecond or so during the break period, for 9600 baud.
 * So dont bother disabling interrupts.
 * But dont want more than 1 break event.
 * Use a kernel timer to periodically poll the rx line until
 * the break is finished.
 */
static inline void scif_schedule_break_timer(struct scif_port *port)
{
	mod_timer(&port->break_timer, jiffies + SCI_BREAK_JIFFIES);
}

/* Ensure that two consecutive samples find the break over. */
static void scif_break_timer(unsigned long data)
{
	struct scif_port *port = (struct scif_port *)data;

	scif_port_enable(port);

	if (scif_rxd_in(&port->port) == 0) {
		port->break_flag = 1;
		scif_schedule_break_timer(port);
	} else if (port->break_flag == 1) {
		/* break is over. */
		port->break_flag = 2;
		scif_schedule_break_timer(port);
	} else
		port->break_flag = 0;

	scif_port_disable(port);
}

static int scif_handle_errors(struct uart_port *port)
{
	int copied = 0;
	unsigned short status = scif_in(port, SH_SCI_SCxSR);
	struct tty_struct *tty = port->state->port.tty;
	struct scif_port *s = to_scif_port(port);

	/*
	 * Handle overruns, if supported.
	 */
	if (s->cfg->overrun_bit != SCIx_NOT_SUPPORTED) {
		if (status & (1 << s->cfg->overrun_bit)) {
			/* overrun error */
			if (tty_insert_flip_char(tty, 0, TTY_OVERRUN))
				copied++;

			dev_notice(port->dev, "overrun error");
		}
	}

	if (status & SCxSR_FER(port)) {
		if (scif_rxd_in(port) == 0) {
			/* Notify of BREAK */
			struct scif_port *scif_port = to_scif_port(port);

			if (!scif_port->break_flag) {
				scif_port->break_flag = 1;
				scif_schedule_break_timer(scif_port);

				/* Do sysrq handling. */
				if (uart_handle_break(port))
					return 0;

				dev_dbg(port->dev, "BREAK detected\n");

				if (tty_insert_flip_char(tty, 0, TTY_BREAK))
					copied++;
			}

		} else {
			/* frame error */
			if (tty_insert_flip_char(tty, 0, TTY_FRAME))
				copied++;

			dev_notice(port->dev, "frame error\n");
		}
	}

	if (status & SCxSR_PER(port)) {
		/* parity error */
		if (tty_insert_flip_char(tty, 0, TTY_PARITY))
			copied++;

		dev_notice(port->dev, "parity error");
	}

	if (copied)
		tty_flip_buffer_push(tty);

	return copied;
}

static int scif_handle_fifo_overrun(struct uart_port *port)
{
	struct tty_struct *tty = port->state->port.tty;
	struct scif_port *s = to_scif_port(port);
	struct plat_scif_reg *reg;
	int copied = 0;

	reg = scif_getreg(port, SH_SCI_SCLSR);
	if (!reg->size)
		return 0;

	if ((scif_in(port, SH_SCI_SCLSR) & (1 << s->cfg->overrun_bit))) {
		scif_out(port, SH_SCI_SCLSR, 0);

		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		tty_flip_buffer_push(tty);

		dev_notice(port->dev, "overrun error\n");
		copied++;
	}

	return copied;
}

static int scif_handle_breaks(struct uart_port *port)
{
	int copied = 0;
	unsigned short status = scif_in(port, SH_SCI_SCxSR);
	struct tty_struct *tty = port->state->port.tty;
	struct scif_port *s = to_scif_port(port);

	if (uart_handle_break(port))
		return 0;

	if (!s->break_flag && status & SCxSR_BRK(port)) {
#if defined(CONFIG_CPU_SH3)
		/* Debounce break */
		s->break_flag = 1;
#endif
		/* Notify of BREAK */
		if (tty_insert_flip_char(tty, 0, TTY_BREAK))
			copied++;

		dev_dbg(port->dev, "BREAK detected\n");
	}

	if (copied)
		tty_flip_buffer_push(tty);

	copied += scif_handle_fifo_overrun(port);

	return copied;
}

static irqreturn_t scif_rx_interrupt(int irq, void *ptr)
{
#ifdef CONFIG_SERIAL_SH_SCI_DMA
	struct uart_port *port = ptr;
	struct scif_port *s = to_scif_port(port);

	if (s->chan_rx) {
		u16 scr = scif_in(port, SH_SCI_SCSCR);
		u16 ssr = scif_in(port, SH_SCI_SCxSR);

		/* Disable future Rx interrupts */
		if (port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
			disable_irq_nosync(irq);
			scr |= 0x4000;
		} else {
			scr &= ~SCSCR_RIE;
		}
		scif_out(port, SH_SCI_SCSCR, scr);
		/* Clear current interrupt */
		scif_out(port, SH_SCI_SCxSR, ssr & ~(1 | SCxSR_RDxF(port)));
		dev_dbg(port->dev, "Rx IRQ %lu: setup t-out in %u jiffies\n",
			jiffies, s->rx_timeout);
		mod_timer(&s->rx_timer, jiffies + s->rx_timeout);

		return IRQ_HANDLED;
	}
#endif

	/* I think scif_receive_chars has to be called irrespective
	 * of whether the I_IXOFF is set, otherwise, how is the interrupt
	 * to be disabled?
	 */

        scif_receive_chars(ptr);

	return IRQ_HANDLED;
}

static irqreturn_t scif_tx_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
	unsigned long flags;

        spin_lock_irqsave(&port->lock, flags);
	scif_transmit_chars(port);
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t scif_er_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;

        /* Handle errors */
	if (port->type == PORT_SCI) {
		if (scif_handle_errors(port)) {
			/* discard character in rx buffer */
			scif_in(port, SH_SCI_SCxSR);
			scif_out(port, SH_SCI_SCxSR, SCxSR_RDxF_CLEAR(port));
		}
	} else {
		scif_handle_fifo_overrun(port);
		scif_rx_interrupt(irq, ptr);
	}

	scif_out(port, SH_SCI_SCxSR, SCxSR_ERROR_CLEAR(port));

	/* Kick the transmission */
	scif_tx_interrupt(irq, ptr);

	return IRQ_HANDLED;
}

static irqreturn_t scif_br_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;

	/* Handle BREAKs */
	scif_handle_breaks(port);

	scif_out(port, SH_SCI_SCxSR, SCxSR_BREAK_CLEAR(port));

	return IRQ_HANDLED;
}

static inline unsigned long port_rx_irq_mask(struct uart_port *port)
{
	/*
	 * Not all ports (such as SCIFA) will support REIE. Rather than
	 * special-casing the port type, we check the port initialization
	 * IRQ enable mask to see whether the IRQ is desired at all. If
	 * it's unset, it's logically inferred that there's no point in
	 * testing for it.
	 */
	return SCSCR_RIE | (to_scif_port(port)->cfg->scscr & SCSCR_REIE);
}

static irqreturn_t scif_mpxed_interrupt(int irq, void *ptr)
{
	unsigned short ssr_status, scr_status, err_enabled;
	struct uart_port *port = ptr;
	struct scif_port *s = to_scif_port(port);
	irqreturn_t ret = IRQ_NONE;

	ssr_status = scif_in(port, SH_SCI_SCxSR);
	scr_status = scif_in(port, SH_SCI_SCSCR);
	err_enabled = scr_status & port_rx_irq_mask(port);

	/* Tx Interrupt */
	if ((ssr_status & SCxSR_TDxE(port)) && (scr_status & SCSCR_TIE) &&
	    !s->chan_tx)
		ret = scif_tx_interrupt(irq, ptr);

	/*
	 * Rx Interrupt: if we're using DMA, the DMA controller clears RDF /
	 * DR flags
	 */
	if (((ssr_status & SCxSR_RDxF(port)) || s->chan_rx) &&
	    (scr_status & SCSCR_RIE))
		ret = scif_rx_interrupt(irq, ptr);

	/* Error Interrupt */
	if ((ssr_status & SCxSR_ERRORS(port)) && err_enabled)
		ret = scif_er_interrupt(irq, ptr);

	/* Break Interrupt */
	if ((ssr_status & SCxSR_BRK(port)) && err_enabled)
		ret = scif_br_interrupt(irq, ptr);

	return ret;
}

static irqreturn_t rs485_transmitter_off_timer(int irq, void *ptr)
{
    PIN_DISA_485TR;         // disable rs485 transmitter
    ctrl_outw(TMU1_TCR_DISAB_INT, TMU1_TCR);        // disable interrupt
    ctrl_outb(ctrl_inb(TMU_TSTR) & ~TMU_TSTR_BIT_TMU1, TMU_TSTR);   // Stop Timer 1
    ctrl_outl(TMTICKS_KEEPENAB, TMU1_TCNT); // write to timer count
    return IRQ_HANDLED;
}

/*
 * Here we define a transition notifier so that we can update all of our
 * ports' baud rate when the peripheral clock changes.
 */
static int scif_notifier(struct notifier_block *self,
			unsigned long phase, void *p)
{
	struct scif_port *scif_port;
	unsigned long flags;

	scif_port = container_of(self, struct scif_port, freq_transition);

	if ((phase == CPUFREQ_POSTCHANGE) ||
	    (phase == CPUFREQ_RESUMECHANGE)) {
		struct uart_port *port = &scif_port->port;

		spin_lock_irqsave(&port->lock, flags);
		port->uartclk = clk_get_rate(scif_port->iclk);
		spin_unlock_irqrestore(&port->lock, flags);
	}

	return NOTIFY_OK;
}

static struct scif_irq_desc {
	const char	*desc;
	irq_handler_t	handler;
} scif_irq_desc[] = {
	/*
	 * Split out handlers, the default case.
	 */
	[SCIx_ERI_IRQ] = {
		.desc = "rx err",
		.handler = scif_er_interrupt,
	},

	[SCIx_RXI_IRQ] = {
		.desc = "rx full",
		.handler = scif_rx_interrupt,
	},

	[SCIx_TXI_IRQ] = {
		.desc = "tx empty",
		.handler = scif_tx_interrupt,
	},

	[SCIx_BRI_IRQ] = {
		.desc = "break",
		.handler = scif_br_interrupt,
	},

	/*
	 * Special muxed handler.
	 */
	[SCIx_MUX_IRQ] = {
		.desc = "mux",
		.handler = scif_mpxed_interrupt,
	},
};

static int scif_request_irq(struct scif_port *port)
{
	struct uart_port *up = &port->port;
        
	int i, j, ret = 0;

	for (i = j = 0; i < SCIx_NR_IRQS; i++, j++) {
		struct scif_irq_desc *desc;
                unsigned int irq;

		if (SCIx_IRQ_IS_MUXED(port)) {
			i = SCIx_MUX_IRQ;
			irq = up->irq;
		} else
			irq = port->cfg->irqs[i];

		desc = scif_irq_desc + i;
		port->irqstr[j] = kasprintf(GFP_KERNEL, "%s:%s",
					    dev_name(up->dev), desc->desc);
		if (!port->irqstr[j]) {
			dev_err(up->dev, "Failed to allocate %s IRQ string\n",
				desc->desc);
			goto out_nomem;
		}
		
		ret = request_irq(irq, desc->handler, up->irqflags,
				  port->irqstr[j], port);
		if (unlikely(ret)) {
			dev_err(up->dev, "Can't allocate %s IRQ\n", desc->desc);
			goto out_noirq;
		}
		
	}
	
	ret = request_irq(TIMER1_IRQ, rs485_transmitter_off_timer, 
                            IRQF_DISABLED /*SA_INTERRUPT */ , "SCIF RS485", 
                            port);
        
        if (unlikely(ret)) {
                dev_err(up->dev, "scif: Cannot allocate timer1 IRQ\n");
                goto out_noirq;
        }

	return 0;

out_noirq:
	while (--i >= 0)
		free_irq(port->cfg->irqs[i], port);

out_nomem:
	while (--j >= 0)
		kfree(port->irqstr[j]);

	return ret;
}

static void scif_free_irq(struct scif_port *port)
{
	int i;

	/*
	 * Intentionally in reverse order so we iterate over the muxed
	 * IRQ first.
	 */
        
        
	for (i = 0; i < SCIx_NR_IRQS; i++) {
		free_irq(port->cfg->irqs[i], port);
		kfree(port->irqstr[i]);

		if (SCIx_IRQ_IS_MUXED(port)) {
			/* If there's only one IRQ, we're done. */
			return;
		}
	}
	
	free_irq(TIMER1_IRQ, port);
	
}

static unsigned int scif_tx_empty(struct uart_port *port)
{
	unsigned short status = scif_in(port, SH_SCI_SCxSR);
	unsigned short in_tx_fifo = scif_txfill(port);

	return (status & SCxSR_TEND(port)) && !in_tx_fifo ? TIOCSER_TEMT : 0;
}

static void scif_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* This routine is used for seting signals of: DTR, DCD, CTS/RTS */
	/* We use SCIF's hardware for CTS/RTS, so don't need any for that. */
	/* If you have signals for DTR and DCD, please implement here. */
}

static unsigned int scif_get_mctrl(struct uart_port *port)
{
	/* This routine is used for getting signals of: DTR, DCD, DSR, RI,
	   and CTS/RTS */

	return TIOCM_DTR | TIOCM_RTS | TIOCM_CTS | TIOCM_DSR;
}

#ifdef CONFIG_SERIAL_SH_SCI_DMA
static void scif_dma_tx_complete(void *arg)
{
	struct scif_port *s = arg;
	struct uart_port *port = &s->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long flags;

	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);

	spin_lock_irqsave(&port->lock, flags);

	xmit->tail += sg_dma_len(&s->sg_tx);
	xmit->tail &= UART_XMIT_SIZE - 1;

	port->icount.tx += sg_dma_len(&s->sg_tx);

	async_tx_ack(s->desc_tx);
	s->desc_tx = NULL;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (!uart_circ_empty(xmit)) {
		s->cookie_tx = 0;
		schedule_work(&s->work_tx);
	} else {
		s->cookie_tx = -EINVAL;
		if (port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
			u16 ctrl = scif_in(port, SH_SCI_SCSCR);
			scif_out(port, SH_SCI_SCSCR, ctrl & ~SCSCR_TIE);
		}
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

/* Locking: called with port lock held */
static int scif_dma_rx_push(struct scif_port *s, struct tty_struct *tty,
			   size_t count)
{
	struct uart_port *port = &s->port;
	int i, active, room;

	room = tty_buffer_request_room(tty, count);

	if (s->active_rx == s->cookie_rx[0]) {
		active = 0;
	} else if (s->active_rx == s->cookie_rx[1]) {
		active = 1;
	} else {
		dev_err(port->dev, "cookie %d not found!\n", s->active_rx);
		return 0;
	}

	if (room < count)
		dev_warn(port->dev, "Rx overrun: dropping %u bytes\n",
			 count - room);
	if (!room)
		return room;

	for (i = 0; i < room; i++)
		tty_insert_flip_char(tty, ((u8 *)sg_virt(&s->sg_rx[active]))[i],
				     TTY_NORMAL);

	port->icount.rx += room;

	return room;
}

static void scif_dma_rx_complete(void *arg)
{
	struct scif_port *s = arg;
	struct uart_port *port = &s->port;
	struct tty_struct *tty = port->state->port.tty;
	unsigned long flags;
	int count;

	dev_dbg(port->dev, "%s(%d) active #%d\n", __func__, port->line, s->active_rx);

	spin_lock_irqsave(&port->lock, flags);

	count = scif_dma_rx_push(s, tty, s->buf_len_rx);

	mod_timer(&s->rx_timer, jiffies + s->rx_timeout);

	spin_unlock_irqrestore(&port->lock, flags);

	if (count)
		tty_flip_buffer_push(tty);

	schedule_work(&s->work_rx);
}

static void scif_rx_dma_release(struct scif_port *s, bool enable_pio)
{
	struct dma_chan *chan = s->chan_rx;
	struct uart_port *port = &s->port;

	s->chan_rx = NULL;
	s->cookie_rx[0] = s->cookie_rx[1] = -EINVAL;
	dma_release_channel(chan);
	if (sg_dma_address(&s->sg_rx[0]))
		dma_free_coherent(port->dev, s->buf_len_rx * 2,
				  sg_virt(&s->sg_rx[0]), sg_dma_address(&s->sg_rx[0]));
	if (enable_pio)
		scif_start_rx(port);
}

static void scif_tx_dma_release(struct scif_port *s, bool enable_pio)
{
	struct dma_chan *chan = s->chan_tx;
	struct uart_port *port = &s->port;

	s->chan_tx = NULL;
	s->cookie_tx = -EINVAL;
	dma_release_channel(chan);
	if (enable_pio)
		scif_start_tx(port);
}

static void scif_submit_rx(struct scif_port *s)
{
	struct dma_chan *chan = s->chan_rx;
	int i;

	for (i = 0; i < 2; i++) {
		struct scatterlist *sg = &s->sg_rx[i];
		struct dma_async_tx_descriptor *desc;

		desc = chan->device->device_prep_slave_sg(chan,
			sg, 1, DMA_FROM_DEVICE, DMA_PREP_INTERRUPT);

		if (desc) {
			s->desc_rx[i] = desc;
			desc->callback = scif_dma_rx_complete;
			desc->callback_param = s;
			s->cookie_rx[i] = desc->tx_submit(desc);
		}

		if (!desc || s->cookie_rx[i] < 0) {
			if (i) {
				async_tx_ack(s->desc_rx[0]);
				s->cookie_rx[0] = -EINVAL;
			}
			if (desc) {
				async_tx_ack(desc);
				s->cookie_rx[i] = -EINVAL;
			}
			dev_warn(s->port.dev,
				 "failed to re-start DMA, using PIO\n");
			scif_rx_dma_release(s, true);
			return;
		}
		dev_dbg(s->port.dev, "%s(): cookie %d to #%d\n", __func__,
			s->cookie_rx[i], i);
	}

	s->active_rx = s->cookie_rx[0];

	dma_async_issue_pending(chan);
}

static void work_fn_rx(struct work_struct *work)
{
	struct scif_port *s = container_of(work, struct scif_port, work_rx);
	struct uart_port *port = &s->port;
	struct dma_async_tx_descriptor *desc;
	int new;

	if (s->active_rx == s->cookie_rx[0]) {
		new = 0;
	} else if (s->active_rx == s->cookie_rx[1]) {
		new = 1;
	} else {
		dev_err(port->dev, "cookie %d not found!\n", s->active_rx);
		return;
	}
	desc = s->desc_rx[new];

	if (dma_async_is_tx_complete(s->chan_rx, s->active_rx, NULL, NULL) !=
	    DMA_SUCCESS) {
		/* Handle incomplete DMA receive */
		struct tty_struct *tty = port->state->port.tty;
		struct dma_chan *chan = s->chan_rx;
		struct sh_desc *sh_desc = container_of(desc, struct sh_desc,
						       async_tx);
		unsigned long flags;
		int count;

		chan->device->device_control(chan, DMA_TERMINATE_ALL, 0);
		dev_dbg(port->dev, "Read %u bytes with cookie %d\n",
			sh_desc->partial, sh_desc->cookie);

		spin_lock_irqsave(&port->lock, flags);
		count = scif_dma_rx_push(s, tty, sh_desc->partial);
		spin_unlock_irqrestore(&port->lock, flags);

		if (count)
			tty_flip_buffer_push(tty);

		scif_submit_rx(s);

		return;
	}

	s->cookie_rx[new] = desc->tx_submit(desc);
	if (s->cookie_rx[new] < 0) {
		dev_warn(port->dev, "Failed submitting Rx DMA descriptor\n");
		scif_rx_dma_release(s, true);
		return;
	}

	s->active_rx = s->cookie_rx[!new];

	dev_dbg(port->dev, "%s: cookie %d #%d, new active #%d\n", __func__,
		s->cookie_rx[new], new, s->active_rx);
}

static void work_fn_tx(struct work_struct *work)
{
	struct scif_port *s = container_of(work, struct scif_port, work_tx);
	struct dma_async_tx_descriptor *desc;
	struct dma_chan *chan = s->chan_tx;
	struct uart_port *port = &s->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct scatterlist *sg = &s->sg_tx;

	/*
	 * DMA is idle now.
	 * Port xmit buffer is already mapped, and it is one page... Just adjust
	 * offsets and lengths. Since it is a circular buffer, we have to
	 * transmit till the end, and then the rest. Take the port lock to get a
	 * consistent xmit buffer state.
	 */
	spin_lock_irq(&port->lock);
	sg->offset = xmit->tail & (UART_XMIT_SIZE - 1);
	sg_dma_address(sg) = (sg_dma_address(sg) & ~(UART_XMIT_SIZE - 1)) +
		sg->offset;
	sg_dma_len(sg) = min((int)CIRC_CNT(xmit->head, xmit->tail, UART_XMIT_SIZE),
		CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE));
	spin_unlock_irq(&port->lock);

	BUG_ON(!sg_dma_len(sg));

	desc = chan->device->device_prep_slave_sg(chan,
			sg, s->sg_len_tx, DMA_TO_DEVICE,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		/* switch to PIO */
		scif_tx_dma_release(s, true);
		return;
	}

	dma_sync_sg_for_device(port->dev, sg, 1, DMA_TO_DEVICE);

	spin_lock_irq(&port->lock);
	s->desc_tx = desc;
	desc->callback = scif_dma_tx_complete;
	desc->callback_param = s;
	spin_unlock_irq(&port->lock);
	s->cookie_tx = desc->tx_submit(desc);
	if (s->cookie_tx < 0) {
		dev_warn(port->dev, "Failed submitting Tx DMA descriptor\n");
		/* switch to PIO */
		scif_tx_dma_release(s, true);
		return;
	}

	dev_dbg(port->dev, "%s: %p: %d...%d, cookie %d\n", __func__,
		xmit->buf, xmit->tail, xmit->head, s->cookie_tx);

	dma_async_issue_pending(chan);
}
#endif

static void scif_start_tx(struct uart_port *port)
{
	struct scif_port *s = to_scif_port(port);
	unsigned short ctrl;

#ifdef CONFIG_SERIAL_SH_SCI_DMA
	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
		u16 new, scr = scif_in(port, SH_SCI_SCSCR);
		if (s->chan_tx)
			new = scr | 0x8000;
		else
			new = scr & ~0x8000;
		if (new != scr)
			scif_out(port, SH_SCI_SCSCR, new);
	}

	if (s->chan_tx && !uart_circ_empty(&s->port.state->xmit) &&
	    s->cookie_tx < 0) {
		s->cookie_tx = 0;
		schedule_work(&s->work_tx);
	}
#endif
        
        ctrl_outl(tmticks_per_byte*2 , TMU1_TCNT); // write to timer count
	if (!s->chan_tx || port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
		/* Set TIE (Transmit Interrupt Enable) bit in SCSCR */
		ctrl = scif_in(port, SH_SCI_SCSCR);
		scif_out(port, SH_SCI_SCSCR, ctrl | SCSCR_TIE);
	}
}

static void scif_stop_tx(struct uart_port *port)
{
	unsigned short ctrl;

	/* Clear TIE (Transmit Interrupt Enable) bit in SCSCR */
	ctrl = scif_in(port, SH_SCI_SCSCR);

	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB)
		ctrl &= ~0x8000;

	ctrl &= ~SCSCR_TIE;

	scif_out(port, SH_SCI_SCSCR, ctrl);
}

static void scif_start_rx(struct uart_port *port)
{
	unsigned short ctrl;

	ctrl = scif_in(port, SH_SCI_SCSCR) | port_rx_irq_mask(port);

	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB)
		ctrl &= ~0x4000;

	scif_out(port, SH_SCI_SCSCR, ctrl);
}

static void scif_stop_rx(struct uart_port *port)
{
	unsigned short ctrl;

	ctrl = scif_in(port, SH_SCI_SCSCR);

	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB)
		ctrl &= ~0x4000;

	ctrl &= ~port_rx_irq_mask(port);

	scif_out(port, SH_SCI_SCSCR, ctrl);
}

static void scif_enable_ms(struct uart_port *port)
{
	/* Nothing here yet .. */
}

static void scif_break_ctl(struct uart_port *port, int break_state)
{
	/* Nothing here yet .. */
}

#ifdef CONFIG_SERIAL_SH_SCI_DMA
static bool filter(struct dma_chan *chan, void *slave)
{
	struct sh_dmae_slave *param = slave;

	dev_dbg(chan->device->dev, "%s: slave ID %d\n", __func__,
		param->slave_id);

	chan->private = param;
	return true;
}

static void rx_timer_fn(unsigned long arg)
{
	struct scif_port *s = (struct scif_port *)arg;
	struct uart_port *port = &s->port;
	u16 scr = scif_in(port, SH_SCI_SCSCR);

	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
		scr &= ~0x4000;
		enable_irq(s->cfg->irqs[1]);
	}
	scif_out(port, SH_SCI_SCSCR, scr | SCSCR_RIE);
	dev_dbg(port->dev, "DMA Rx timed out\n");
	schedule_work(&s->work_rx);
}

static void scif_request_dma(struct uart_port *port)
{
	struct scif_port *s = to_scif_port(port);
	struct sh_dmae_slave *param;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	int nent;

	dev_dbg(port->dev, "%s: port %d\n", __func__,
		port->line);

	if (s->cfg->dma_slave_tx <= 0 || s->cfg->dma_slave_rx <= 0)
		return;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	param = &s->param_tx;

	/* Slave ID, e.g., SHDMA_SLAVE_SCIF0_TX */
	param->slave_id = s->cfg->dma_slave_tx;

	s->cookie_tx = -EINVAL;
	chan = dma_request_channel(mask, filter, param);
	dev_dbg(port->dev, "%s: TX: got channel %p\n", __func__, chan);
	if (chan) {
		s->chan_tx = chan;
		sg_init_table(&s->sg_tx, 1);
		/* UART circular tx buffer is an aligned page. */
		BUG_ON((int)port->state->xmit.buf & ~PAGE_MASK);
		sg_set_page(&s->sg_tx, virt_to_page(port->state->xmit.buf),
			    UART_XMIT_SIZE, (int)port->state->xmit.buf & ~PAGE_MASK);
		nent = dma_map_sg(port->dev, &s->sg_tx, 1, DMA_TO_DEVICE);
		if (!nent)
			scif_tx_dma_release(s, false);
		else
			dev_dbg(port->dev, "%s: mapped %d@%p to %x\n", __func__,
				sg_dma_len(&s->sg_tx),
				port->state->xmit.buf, sg_dma_address(&s->sg_tx));

		s->sg_len_tx = nent;

		INIT_WORK(&s->work_tx, work_fn_tx);
	}

	param = &s->param_rx;

	/* Slave ID, e.g., SHDMA_SLAVE_SCIF0_RX */
	param->slave_id = s->cfg->dma_slave_rx;

	chan = dma_request_channel(mask, filter, param);
	dev_dbg(port->dev, "%s: RX: got channel %p\n", __func__, chan);
	if (chan) {
		dma_addr_t dma[2];
		void *buf[2];
		int i;

		s->chan_rx = chan;

		s->buf_len_rx = 2 * max(16, (int)port->fifosize);
		buf[0] = dma_alloc_coherent(port->dev, s->buf_len_rx * 2,
					    &dma[0], GFP_KERNEL);

		if (!buf[0]) {
			dev_warn(port->dev,
				 "failed to allocate dma buffer, using PIO\n");
			scif_rx_dma_release(s, true);
			return;
		}

		buf[1] = buf[0] + s->buf_len_rx;
		dma[1] = dma[0] + s->buf_len_rx;

		for (i = 0; i < 2; i++) {
			struct scatterlist *sg = &s->sg_rx[i];

			sg_init_table(sg, 1);
			sg_set_page(sg, virt_to_page(buf[i]), s->buf_len_rx,
				    (int)buf[i] & ~PAGE_MASK);
			sg_dma_address(sg) = dma[i];
		}

		INIT_WORK(&s->work_rx, work_fn_rx);
		setup_timer(&s->rx_timer, rx_timer_fn, (unsigned long)s);

		scif_submit_rx(s);
	}
}

static void scif_free_dma(struct uart_port *port)
{
	struct scif_port *s = to_scif_port(port);

	if (s->chan_tx)
		scif_tx_dma_release(s, false);
	if (s->chan_rx)
		scif_rx_dma_release(s, false);
}
#else
static inline void scif_request_dma(struct uart_port *port)
{
}

static inline void scif_free_dma(struct uart_port *port)
{
}
#endif

static int scif_startup(struct uart_port *port)
{
	struct scif_port *s = to_scif_port(port);
	int ret;

	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);

	scif_port_enable(s);

	ret = scif_request_irq(s);
	if (unlikely(ret < 0))
		return ret;

	scif_request_dma(port);

	scif_start_tx(port);
	scif_start_rx(port);

	return 0;
}

static void scif_shutdown(struct uart_port *port)
{
	struct scif_port *s = to_scif_port(port);

	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);
        
        
	scif_stop_rx(port);
	scif_stop_tx(port);

	scif_free_dma(port);
	scif_free_irq(s);
        
        PIN_DISA_485TR;  

	scif_port_disable(s);
}

static unsigned int scif_scbrr_calc(unsigned int algo_id, unsigned int bps,
				   unsigned long freq)
{
	switch (algo_id) {
	case SCBRR_ALGO_1:
		return ((freq + 16 * bps) / (16 * bps) - 1);
	case SCBRR_ALGO_2:
		return ((freq + 16 * bps) / (32 * bps) - 1);
	case SCBRR_ALGO_3:
		return (((freq * 2) + 16 * bps) / (16 * bps) - 1);
	case SCBRR_ALGO_4:
		return (((freq * 2) + 16 * bps) / (32 * bps) - 1);
	case SCBRR_ALGO_5:
		return (((freq * 1000 / 32) / bps) - 1);
	}

	/* Warn, but use a safe default */
	WARN_ON(1);

	return ((freq + 16 * bps) / (32 * bps) - 1);
}

static void scif_reset(struct uart_port *port)
{
	unsigned int status;

	do {
		status = scif_in(port, SH_SCI_SCxSR);
	} while (!(status & SCxSR_TEND(port)));

	scif_out(port, SH_SCI_SCSCR, 0x00);	/* TE=0, RE=0, CKE1=0 */

	if (port->type != PORT_SCI)
		scif_out(port, SH_SCI_SCFCR, SCFCR_RFRST | SCFCR_TFRST);
}

static void scif_set_termios(struct uart_port *port, struct ktermios *termios,
			    struct ktermios *old)
{
	struct scif_port *s = to_scif_port(port);
	unsigned int baud, smr_val, max_baud;
	int t = -1;
	u16 scfcr = 0;

	/*
	 * earlyprintk comes here early on with port->uartclk set to zero.
	 * the clock framework is not up and running at this point so here
	 * we assume that 115200 is the maximum baud rate. please note that
	 * the baud rate is not programmed during earlyprintk - it is assumed
	 * that the previous boot loader has enabled required clocks and
	 * setup the baud rate generator hardware for us already.
	 */
	max_baud = port->uartclk ? port->uartclk / 16 : 115200;

	baud = uart_get_baud_rate(port, termios, old, 0, max_baud);
	if (likely(baud && port->uartclk))
		t = scif_scbrr_calc(s->cfg->scbrr_algo_id, baud, port->uartclk);

        
        printk(KERN_DEBUG"%s():%d t= %d",__FUNCTION__,__LINE__,t);
        tmticks_per_byte = (t * 80 - TMTICKS_FOR_UPDATE)/4;
        
	scif_port_enable(s);

	scif_reset(port);

	smr_val = scif_in(port, SH_SCI_SCSMR) & 3;

	if ((termios->c_cflag & CSIZE) == CS7)
		smr_val |= 0x40;
	if (termios->c_cflag & PARENB)
		smr_val |= 0x20;
	if (termios->c_cflag & PARODD)
		smr_val |= 0x30;
	if (termios->c_cflag & CSTOPB)
		smr_val |= 0x08;

	uart_update_timeout(port, termios->c_cflag, baud);

	scif_out(port, SH_SCI_SCSMR, smr_val);

	dev_dbg(port->dev, "%s: SMR %x, t %x, SCSCR %x\n", __func__, smr_val, t,
		s->cfg->scscr);

	if (t > 0) {
		if (t >= 256) {
			scif_out(port, SH_SCI_SCSMR, (scif_in(port, SH_SCI_SCSMR) & ~3) | 1);
			t >>= 2;
		} else
			scif_out(port, SH_SCI_SCSMR, scif_in(port, SH_SCI_SCSMR) & ~3);

		scif_out(port, SH_SCI_SCBRR, t);
		udelay((1000000+(baud-1)) / baud); /* Wait one bit interval */
	}

	PIN_DISA_485TR;
        PIN_INIT_485TR;
        PIN_DISA_485TR;

        ctrl_outb(ctrl_inb(TMU_TSTR) & ~TMU_TSTR_BIT_TMU1, TMU_TSTR);   /* Stop Timer 1 */
        ctrl_outw(TMU1_TCR_INIT, TMU1_TCR);                             /* init timer control */
        ctrl_outl(0, TMU1_TCOR);                                        /* constant register max. */
        ctrl_outl(TMTICKS_KEEPENAB, TMU1_TCNT);                         /* write to timer count */

        
	
	
	scif_init_pins(port, termios->c_cflag);
        scif_out(port, SH_SCI_SCFCR, scfcr | ((termios->c_cflag & CRTSCTS) ? SCFCR_MCE : 0));

	scif_out(port, SH_SCI_SCSCR, s->cfg->scscr);

#ifdef CONFIG_SERIAL_SH_SCI_DMA
	/*
	 * Calculate delay for 1.5 DMA buffers: see
	 * drivers/serial/serial_core.c::uart_update_timeout(). With 10 bits
	 * (CS8), 250Hz, 115200 baud and 64 bytes FIFO, the above function
	 * calculates 1 jiffie for the data plus 5 jiffies for the "slop(e)."
	 * Then below we calculate 3 jiffies (12ms) for 1.5 DMA buffers (3 FIFO
	 * sizes), but it has been found out experimentally, that this is not
	 * enough: the driver too often needlessly runs on a DMA timeout. 20ms
	 * as a minimum seem to work perfectly.
	 */
	if (s->chan_rx) {
		s->rx_timeout = (port->timeout - HZ / 50) * s->buf_len_rx * 3 /
			port->fifosize / 2;
		dev_dbg(port->dev,
			"DMA Rx t-out %ums, tty t-out %u jiffies\n",
			s->rx_timeout * 1000 / HZ, port->timeout);
		if (s->rx_timeout < msecs_to_jiffies(20))
			s->rx_timeout = msecs_to_jiffies(20);
	}
#endif

	if ((termios->c_cflag & CREAD) != 0)
		scif_start_rx(port);

	scif_port_disable(s);
}




static const char *scif_type(struct uart_port *port)
{
	switch (port->type) {
	case PORT_IRDA:
		return "irda";
	case PORT_SCI:
		return "scif";
	case PORT_SCIF:
		return "scif";
	case PORT_SCIFA:
		return "scifa";
	case PORT_SCIFB:
		return "scifb";
	}

	return NULL;
}

static inline unsigned long scif_port_size(struct uart_port *port)
{
	/*
	 * Pick an arbitrary size that encapsulates all of the base
	 * registers by default. This can be optimized later, or derived
	 * from platform resource data at such a time that ports begin to
	 * behave more erratically.
	 */
	return 64;
}

static int scif_remap_port(struct uart_port *port)
{
	unsigned long size = scif_port_size(port);

	/*
	 * Nothing to do if there's already an established membase.
	 */
	if (port->membase)
		return 0;

	if (port->flags & UPF_IOREMAP) {
		port->membase = ioremap_nocache(port->mapbase, size);
		if (unlikely(!port->membase)) {
			dev_err(port->dev, "can't remap port#%d\n", port->line);
			return -ENXIO;
		}
	} else {
		/*
		 * For the simple (and majority of) cases where we don't
		 * need to do any remapping, just cast the cookie
		 * directly.
		 */
		port->membase = (void __iomem *)port->mapbase;
	}

	return 0;
}

static void scif_release_port(struct uart_port *port)
{
	if (port->flags & UPF_IOREMAP) {
		iounmap(port->membase);
		port->membase = NULL;
	}

	release_mem_region(port->mapbase, scif_port_size(port));
}

static int scif_request_port(struct uart_port *port)
{
	unsigned long size = scif_port_size(port);
	struct resource *res;
	int ret;

	res = request_mem_region(port->mapbase, size, dev_name(port->dev));
	if (unlikely(res == NULL))
		return -EBUSY;

	ret = scif_remap_port(port);
	if (unlikely(ret != 0)) {
		release_resource(res);
		return ret;
	}

	return 0;
}

static void scif_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		struct scif_port *sport = to_scif_port(port);

		port->type = sport->cfg->type;
		scif_request_port(port);
	}
}

static int scif_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct scif_port *s = to_scif_port(port);

	if (ser->irq != s->cfg->irqs[SCIx_TXI_IRQ] || ser->irq > nr_irqs)
		return -EINVAL;
	if (ser->baud_base < 2400)
		/* No paper tape reader for Mitch.. */
		return -EINVAL;

	return 0;
}

static struct uart_ops scif_uart_ops = {
	.tx_empty	= scif_tx_empty,
	.set_mctrl	= scif_set_mctrl,
	.get_mctrl	= scif_get_mctrl,
	.start_tx	= scif_start_tx,
	.stop_tx	= scif_stop_tx,
	.stop_rx	= scif_stop_rx,
	.enable_ms	= scif_enable_ms,
	.break_ctl	= scif_break_ctl,
	.startup	= scif_startup,
	.shutdown	= scif_shutdown,
	.set_termios	= scif_set_termios,
	.type		= scif_type,
	.release_port	= scif_release_port,
	.request_port	= scif_request_port,
	.config_port	= scif_config_port,
	.verify_port	= scif_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= scif_poll_get_char,
	.poll_put_char	= scif_poll_put_char,
#endif
};

static int __devinit scif_init_single(struct platform_device *dev,
				     struct scif_port *scif_port,
				     unsigned int index,
				     struct plat_sci_port *p)
{
	struct uart_port *port = &scif_port->port;
	int ret;

	port->ops	= &scif_uart_ops;
	port->iotype	= UPIO_MEM;
	port->line	= index;

	switch (p->type) {
	case PORT_SCIFB:
		port->fifosize = 256;
		break;
	case PORT_SCIFA:
		port->fifosize = 64;
		break;
	case PORT_SCIF:
		port->fifosize = 16;
		break;
	default:
		port->fifosize = 1;
		break;
	}

	if (p->regtype == SCIx_PROBE_REGTYPE) {
		ret = scif_probe_regmap(p);
		if (unlikely(ret))
			return ret;
	}

	if (dev) {
		scif_port->iclk = clk_get(&dev->dev, "scif_ick");
		if (IS_ERR(scif_port->iclk)) {
			scif_port->iclk = clk_get(&dev->dev, "peripheral_clk");
			if (IS_ERR(scif_port->iclk)) {
				dev_err(&dev->dev, "can't get iclk\n");
				return PTR_ERR(scif_port->iclk);
			}
		}

		/*
		 * The function clock is optional, ignore it if we can't
		 * find it.
		 */
		scif_port->fclk = clk_get(&dev->dev, "scif_fck");
		if (IS_ERR(scif_port->fclk))
			scif_port->fclk = NULL;

		port->dev = &dev->dev;

		pm_runtime_irq_safe(&dev->dev);
		pm_runtime_enable(&dev->dev);
	}

	scif_port->break_timer.data = (unsigned long)scif_port;
	scif_port->break_timer.function = scif_break_timer;
	init_timer(&scif_port->break_timer);

	/*
	 * Establish some sensible defaults for the error detection.
	 */
	if (!p->error_mask)
		p->error_mask = (p->type == PORT_SCI) ?
			SCI_DEFAULT_ERROR_MASK : SCIF_DEFAULT_ERROR_MASK;

	/*
	 * Establish sensible defaults for the overrun detection, unless
	 * the part has explicitly disabled support for it.
	 */
	if (p->overrun_bit != SCIx_NOT_SUPPORTED) {
		if (p->type == PORT_SCI)
			p->overrun_bit = 5;
		else if (p->scbrr_algo_id == SCBRR_ALGO_4)
			p->overrun_bit = 9;
		else
			p->overrun_bit = 0;

		/*
		 * Make the error mask inclusive of overrun detection, if
		 * supported.
		 */
		p->error_mask |= (1 << p->overrun_bit);
	}

	scif_port->cfg		= p;

	port->mapbase		= p->mapbase;
	port->type		= p->type;
	port->flags		= p->flags;
	port->regshift		= p->regshift;

	/*
	 * The UART port needs an IRQ value, so we peg this to the RX IRQ
	 * for the multi-IRQ ports, which is where we are primarily
	 * concerned with the shutdown path synchronization.
	 *
	 * For the muxed case there's nothing more to do.
	 */
	port->irq		= p->irqs[SCIx_RXI_IRQ];
	port->irqflags		= 0;

	port->serial_in		= scif_serial_in;
	port->serial_out	= scif_serial_out;

	if (p->dma_slave_tx > 0 && p->dma_slave_rx > 0)
		dev_dbg(port->dev, "DMA tx %d, rx %d\n",
			p->dma_slave_tx, p->dma_slave_rx);

	return 0;
}

#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
static void serial_console_putchar(struct uart_port *port, int ch)
{
	scif_poll_put_char(port, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 */
static void serial_console_write(struct console *co, const char *s,
				 unsigned count)
{
	struct scif_port *scif_port = &scif_ports[co->index];
	struct uart_port *port = &scif_port->port;
	unsigned short bits;

	scif_port_enable(scif_port);

	uart_console_write(port, s, count, serial_console_putchar);

	/* wait until fifo is empty and last bit has been transmitted */
	bits = SCxSR_TDxE(port) | SCxSR_TEND(port);
	while ((scif_in(port, SH_SCI_SCxSR) & bits) != bits)
		cpu_relax();

	scif_port_disable(scif_port);
}

static int __devinit serial_console_setup(struct console *co, char *options)
{
	struct scif_port *scif_port;
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	/*
	 * Refuse to handle any bogus ports.
	 */
	if (co->index < 0 || co->index >= SCI_NPORTS)
		return -ENODEV;

	scif_port = &scif_ports[co->index];
	port = &scif_port->port;

	/*
	 * Refuse to handle uninitialized ports.
	 */
	if (!port->ops)
		return -ENODEV;

	ret = scif_remap_port(port);
	if (unlikely(ret != 0))
		return ret;

	scif_port_enable(scif_port);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	scif_port_disable(scif_port);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console serial_console = {
	.name		= "ttyRS485",
	.device		= uart_console_device,
	.write		= serial_console_write,
	.setup		= serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &scif_uart_driver,
};

static struct console early_serial_console = {
	.name           = "early_ttySC",
	.write          = serial_console_write,
	.flags          = CON_PRINTBUFFER,
	.index		= -1,
};

static char early_serial_buf[32];

static int __devinit scif_probe_earlyprintk(struct platform_device *pdev)
{
	struct plat_sci_port *cfg = pdev->dev.platform_data;

	if (early_serial_console.data)
		return -EEXIST;

	early_serial_console.index = pdev->id;

	scif_init_single(NULL, &scif_ports[pdev->id], pdev->id, cfg);

	serial_console_setup(&early_serial_console, early_serial_buf);

	if (!strstr(early_serial_buf, "keep"))
		early_serial_console.flags |= CON_BOOT;

	register_console(&early_serial_console);
	return 0;
}

#define uart_console(port)	((port)->cons->index == (port)->line)

static int scif_runtime_suspend(struct device *dev)
{
	struct scif_port *scif_port = dev_get_drvdata(dev);
	struct uart_port *port = &scif_port->port;

	if (uart_console(port)) {
		scif_port->saved_smr = scif_in(port, SH_SCI_SCSMR);
		scif_port->saved_brr = scif_in(port, SH_SCI_SCBRR);
		scif_port->saved_fcr = scif_in(port, SH_SCI_SCFCR);
	}
	return 0;
}

static int scif_runtime_resume(struct device *dev)
{
	struct scif_port *scif_port = dev_get_drvdata(dev);
	struct uart_port *port = &scif_port->port;

	if (uart_console(port)) {
		scif_reset(port);
		scif_out(port, SH_SCI_SCSMR, scif_port->saved_smr);
		scif_out(port, SH_SCI_SCBRR, scif_port->saved_brr);
		scif_out(port, SH_SCI_SCFCR, scif_port->saved_fcr);
		scif_out(port, SH_SCI_SCSCR, scif_port->cfg->scscr);
	}
	return 0;
}

#define SCI_CONSOLE	(&serial_console)

#else
static inline int __devinit scif_probe_earlyprintk(struct platform_device *pdev)
{
	return -EINVAL;
}

#define SCI_CONSOLE	NULL
#define scif_runtime_suspend	NULL
#define scif_runtime_resume	NULL

#endif /* CONFIG_SERIAL_SH_SCI_CONSOLE */

static char banner[] __initdata =
	KERN_INFO "SuperH SCI(F) driver initialized\n";

static struct uart_driver scif_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "scif",
	.dev_name	= "ttyRS485",
	.major		= SCI_MAJOR,
	.minor		= SCI_MINOR_START,
	.nr		= SCI_NPORTS,
	.cons		= SCI_CONSOLE,
};

static int scif_remove(struct platform_device *dev)
{
	struct scif_port *port = platform_get_drvdata(dev);

	cpufreq_unregister_notifier(&port->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);

	uart_remove_one_port(&scif_uart_driver, &port->port);

	clk_put(port->iclk);
	clk_put(port->fclk);

	pm_runtime_disable(&dev->dev);
	return 0;
}

static int __devinit scif_probe_single(struct platform_device *dev,
				      unsigned int index,
				      struct plat_sci_port *p,
				      struct scif_port *scifport)
{
	int ret;

	/* Sanity check */
	if (unlikely(index >= SCI_NPORTS)) {
		dev_notice(&dev->dev, "Attempting to register port "
			   "%d when only %d are available.\n",
			   index+1, SCI_NPORTS);
		dev_notice(&dev->dev, "Consider bumping "
			   "CONFIG_SERIAL_SH_SCI_NR_UARTS!\n");
		return 0;
	}

	ret = scif_init_single(dev, scifport, index, p);
	if (ret)
		return ret;

	return uart_add_one_port(&scif_uart_driver, &scifport->port);
}

static int __devinit scif_probe(struct platform_device *dev)
{
	struct plat_sci_port *p = dev->dev.platform_data;
	struct scif_port *sp = &scif_ports[dev->id];
	int ret;

	/*
	 * If we've come here via earlyprintk initialization, head off to
	 * the special early probe. We don't have sufficient device state
	 * to make it beyond this yet.
	 */
	if (is_early_platform_device(dev))
		return scif_probe_earlyprintk(dev);

	platform_set_drvdata(dev, sp);

	ret = scif_probe_single(dev, dev->id, p, sp);
	if (ret)
		goto err_unreg;

	sp->freq_transition.notifier_call = scif_notifier;

	ret = cpufreq_register_notifier(&sp->freq_transition,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (unlikely(ret < 0))
		goto err_unreg;

#ifdef CONFIG_SH_STANDARD_BIOS
	sh_bios_gdb_detach();
#endif

	return 0;

err_unreg:
	scif_remove(dev);
	return ret;
}

static int scif_suspend(struct device *dev)
{
	struct scif_port *sport = dev_get_drvdata(dev);

	if (sport)
		uart_suspend_port(&scif_uart_driver, &sport->port);

	return 0;
}

static int scif_resume(struct device *dev)
{
	struct scif_port *sport = dev_get_drvdata(dev);

	if (sport)
		uart_resume_port(&scif_uart_driver, &sport->port);

	return 0;
}

static const struct dev_pm_ops scif_dev_pm_ops = {
	.runtime_suspend = scif_runtime_suspend,
	.runtime_resume = scif_runtime_resume,
	.suspend	= scif_suspend,
	.resume		= scif_resume,
};

static struct platform_driver scif_driver = {
	.probe		= scif_probe,
	.remove		= scif_remove,
	.driver		= {
		.name	= "sh-scif-rs485",
		.owner	= THIS_MODULE,
		.pm	= &scif_dev_pm_ops,
	},
};

static int __init scif_init(void)
{
	int ret;

	printk(banner);

	ret = uart_register_driver(&scif_uart_driver);
	if (likely(ret == 0)) {
		ret = platform_driver_register(&scif_driver);
		if (unlikely(ret))
			uart_unregister_driver(&scif_uart_driver);
	}

	return ret;
}

static void __exit scif_exit(void)
{
	platform_driver_unregister(&scif_driver);
	uart_unregister_driver(&scif_uart_driver);
}

#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
early_platform_init_buffer("earlyprintk", &scif_driver,
			   early_serial_buf, ARRAY_SIZE(early_serial_buf));
#endif
module_init(scif_init);
module_exit(scif_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sh-scif-rs485");
MODULE_AUTHOR("Paul Mundt");
MODULE_DESCRIPTION("SuperH SCI(F) serial driver");
