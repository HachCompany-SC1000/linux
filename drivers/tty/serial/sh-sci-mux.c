/*
 * SuperH on-chip serial module support.  (SCI with no FIFO / with FIFO)
 *
 *  Copyright (C) 2002 - 2011  Paul Mundt
 *  Modified to support SH7720 SCIF. Markus Brunner, Mark Jonas (Jul 2007).
 *
 * based off of the old drivers/char/sh-sci.c by:
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
#include "sh-sci-mux.h"



#define USE_MULTIPLEXER  1


static wait_queue_head_t wait;
static unsigned char wait_end = 0;


struct sci_port {
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
static void sci_start_tx(struct uart_port *port);
static void sci_stop_tx(struct uart_port *port);
static void sci_start_rx(struct uart_port *port);


/* only one multiplexed port */
#define SCI_NPORTS 1 /*CONFIG_SERIAL_SH_SCI_NR_UARTS*/

static struct sci_port sci_ports[SCI_NPORTS];
static struct uart_driver sci_uart_driver;

static inline struct sci_port *
to_sci_port(struct uart_port *uart)
{
	return container_of(uart, struct sci_port, port);
}

struct plat_sci_reg {
	u8 offset, size;
};

/* Helper for invalidating specific entries of an inherited map. */
#define sci_reg_invalid	{ .offset = 0, .size = 0 }

static struct plat_sci_reg sci_regmap[SCIx_NR_REGTYPES][SCIx_NR_REGS] = {
	[SCIx_PROBE_REGTYPE] = {
		[0 ... SCIx_NR_REGS - 1] = sci_reg_invalid,
	},

	/*
	 * Common SCI definitions, dependent on the port's regshift
	 * value.
	 */
	[SCIx_SCI_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00,  8 },
		[SH_SCI_SCBRR]		= { 0x01,  8 },
		[SH_SCI_SCSCR]		= { 0x02,  8 },
		[SH_SCI_SCxTDR]	        = { 0x03,  8 },
		[SH_SCI_SCxSR]		= { 0x04,  8 },
		[SH_SCI_SCxRDR]	        = { 0x05,  8 },
		[SH_SCI_SCFCR]		= sci_reg_invalid,
		[SH_SCI_SCFDR]		= sci_reg_invalid,
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = sci_reg_invalid,
		[SH_SCI_SCLSR]		= sci_reg_invalid,
	},

	/*
	 * Common definitions for legacy IrDA ports, dependent on
	 * regshift value.
	 */
	[SCIx_IRDA_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00,  8 },
		[SH_SCI_SCBRR]		= { 0x01,  8 },
		[SH_SCI_SCSCR]		= { 0x02,  8 },
		[SH_SCI_SCxTDR]	        = { 0x03,  8 },
		[SH_SCI_SCxSR]		= { 0x04,  8 },
		[SH_SCI_SCxRDR]	        = { 0x05,  8 },
		[SH_SCI_SCFCR]		= { 0x06,  8 },
		[SH_SCI_SCFDR]		= { 0x07, 16 },
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = sci_reg_invalid,
		[SH_SCI_SCLSR]		= sci_reg_invalid,
	},

	/*
	 * Common SCIFA definitions.
	 */
	[SCIx_SCIFA_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00, 16 },
		[SH_SCI_SCBRR]		= { 0x04,  8 },
		[SH_SCI_SCSCR]		= { 0x08, 16 },
		[SH_SCI_SCxTDR]	        = { 0x20,  8 },
		[SH_SCI_SCxSR]		= { 0x14, 16 },
		[SH_SCI_SCxRDR]	        = { 0x24,  8 },
		[SH_SCI_SCFCR]		= { 0x18, 16 },
		[SH_SCI_SCFDR]		= { 0x1c, 16 },
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = sci_reg_invalid,
		[SH_SCI_SCLSR]		= sci_reg_invalid,
	},

	/*
	 * Common SCIFB definitions.
	 */
	[SCIx_SCIFB_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00, 16 },
		[SH_SCI_SCBRR]		= { 0x04,  8 },
		[SH_SCI_SCSCR]		= { 0x08, 16 },
		[SH_SCI_SCxTDR]	        = { 0x40,  8 },
		[SH_SCI_SCxSR]		= { 0x14, 16 },
		[SH_SCI_SCxRDR]	        = { 0x60,  8 },
		[SH_SCI_SCFCR]		= { 0x18, 16 },
		[SH_SCI_SCFDR]		= { 0x1c, 16 },
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = sci_reg_invalid,
		[SH_SCI_SCLSR]		= sci_reg_invalid,
	},

	/*
	 * Common SH-2(A) SCIF definitions for ports with FIFO data
	 * count registers.
	 */
	[SCIx_SH2_SCIF_FIFODATA_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00, 16 },
		[SH_SCI_SCBRR]		= { 0x04,  8 },
		[SH_SCI_SCSCR]		= { 0x08, 16 },
		[SH_SCI_SCxTDR]	        = { 0x0c,  8 },
		[SH_SCI_SCxSR]		= { 0x10, 16 },
		[SH_SCI_SCxRDR]	        = { 0x14,  8 },
		[SH_SCI_SCFCR]		= { 0x18, 16 },
		[SH_SCI_SCFDR]		= { 0x1c, 16 },
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = { 0x20, 16 },
		[SH_SCI_SCLSR]		= { 0x24, 16 },
	},

	/*
	 * Common SH-3 SCIF definitions.
	 */
	[SCIx_SH3_SCIF_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00,  8 },
		[SH_SCI_SCBRR]		= { 0x02,  8 },
		[SH_SCI_SCSCR]		= { 0x04,  8 },
		[SH_SCI_SCxTDR]	        = { 0x06,  8 },
		[SH_SCI_SCxSR]		= { 0x08, 16 },
		[SH_SCI_SCxRDR]	        = { 0x0a,  8 },
		[SH_SCI_SCFCR]		= { 0x0c,  8 },
		[SH_SCI_SCFDR]		= { 0x0e, 16 },
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = sci_reg_invalid,
		[SH_SCI_SCLSR]		= sci_reg_invalid,
	},

	/*
	 * Common SH-4(A) SCIF(B) definitions.
	 */
	[SCIx_SH4_SCIF_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00, 16 },
		[SH_SCI_SCBRR]		= { 0x04,  8 },
		[SH_SCI_SCSCR]		= { 0x08, 16 },
		[SH_SCI_SCxTDR]	        = { 0x0c,  8 },
		[SH_SCI_SCxSR]		= { 0x10, 16 },
		[SH_SCI_SCxRDR]	        = { 0x14,  8 },
		[SH_SCI_SCFCR]		= { 0x18, 16 },
		[SH_SCI_SCFDR]		= { 0x1c, 16 },
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = { 0x20, 16 },
		[SH_SCI_SCLSR]		= { 0x24, 16 },
	},

	/*
	 * Common SH-4(A) SCIF(B) definitions for ports without an SCSPTR
	 * register.
	 */
	[SCIx_SH4_SCIF_NO_SCSPTR_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00, 16 },
		[SH_SCI_SCBRR]		= { 0x04,  8 },
		[SH_SCI_SCSCR]		= { 0x08, 16 },
		[SH_SCI_SCxTDR]	        = { 0x0c,  8 },
		[SH_SCI_SCxSR]		= { 0x10, 16 },
		[SH_SCI_SCxRDR]	        = { 0x14,  8 },
		[SH_SCI_SCFCR]		= { 0x18, 16 },
		[SH_SCI_SCFDR]		= { 0x1c, 16 },
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = sci_reg_invalid,
		[SH_SCI_SCLSR]		= { 0x24, 16 },
	},

	/*
	 * Common SH-4(A) SCIF(B) definitions for ports with FIFO data
	 * count registers.
	 */
	[SCIx_SH4_SCIF_FIFODATA_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00, 16 },
		[SH_SCI_SCBRR]		= { 0x04,  8 },
		[SH_SCI_SCSCR]		= { 0x08, 16 },
		[SH_SCI_SCxTDR]	        = { 0x0c,  8 },
		[SH_SCI_SCxSR]		= { 0x10, 16 },
		[SH_SCI_SCxRDR]	        = { 0x14,  8 },
		[SH_SCI_SCFCR]		= { 0x18, 16 },
		[SH_SCI_SCFDR]		= { 0x1c, 16 },
		[SH_SCI_SCTFDR]	        = { 0x1c, 16 },	/* aliased to SCFDR */
		[SH_SCI_SCRFDR]	        = { 0x20, 16 },
		[SH_SCI_SCSPTR]	        = { 0x24, 16 },
		[SH_SCI_SCLSR]		= { 0x28, 16 },
	},

	/*
	 * SH7705-style SCIF(B) ports, lacking both SCSPTR and SCLSR
	 * registers.
	 */
	[SCIx_SH7705_SCIF_REGTYPE] = {
		[SH_SCI_SCSMR]		= { 0x00, 16 },
		[SH_SCI_SCBRR]		= { 0x04,  8 },
		[SH_SCI_SCSCR]		= { 0x08, 16 },
		[SH_SCI_SCxTDR]	        = { 0x20,  8 },
		[SH_SCI_SCxSR]		= { 0x14, 16 },
		[SH_SCI_SCxRDR]	        = { 0x24,  8 },
		[SH_SCI_SCFCR]		= { 0x18, 16 },
		[SH_SCI_SCFDR]		= { 0x1c, 16 },
		[SH_SCI_SCTFDR]	        = sci_reg_invalid,
		[SH_SCI_SCRFDR]	        = sci_reg_invalid,
		[SH_SCI_SCSPTR]	        = sci_reg_invalid,
		[SH_SCI_SCLSR]		= sci_reg_invalid,
	},
};

#define sci_getreg(up, offset)		(sci_regmap[to_sci_port(up)->cfg->regtype] + offset)





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


/* ---------------------------------------- */
/* ------------- multiplexer: ------------- */
/* ---------------------------------------- */


#ifdef USE_MULTIPLEXER


#include <linux/syscalls.h>
#include <linux/blkdev.h>
#include <linux/kthread.h>

/* ------------- sd command buffer queue: ------------- */

/**
 *  These commands are handed over to the hardware driver/muxer.
 */
#define CMD_SD_WELCOME (0)
#define CMD_SD_READ    (1)
#define CMD_SD_WRITE   (2)


struct sci_port *_port;

struct sd_queue_t {
        int                   cmd;       /* CMD_SD_READ,.. , CMD_SD_CSLOW */
        struct sd_host       *host;
        struct request_queue *queue;
        struct completion    *compl;
        struct request       *request;
};

#define MAX_SD_REQUESTS      (128)

#define BIT_PSWITCH     (1 << 7)        // position ttySC0 port switch
#define BIT_485TRM      (1 << 5)        // position of 485 driver enable pin
#define BIT_IGT         (1 << 5)        // position IGT
#define BIT_DBG         (1 << 6)        // position DBG bit
#define BIT_GSM         (1 << 3)        // position GSM pugged in
#define BIT_MODEM_POWER (1 << 4)        // position 12V Data Tele
#define PIN_CHK_GSM     (~ctrl_inb(REG_IGT_SW) & BIT_GSM)       // GSM
#define REG_IGT_SW      SCPDR   // IGT enable/disable
#define REG_12V_DATATELE PORT_PKDR

/* IGT switch */
// IGT on
#define PIN_SWI_IGT_ON  ctrl_outb(ctrl_inb(REG_IGT_SW) |  BIT_IGT, REG_IGT_SW)
// IGT off
#define PIN_SWI_IGT_OFF ctrl_outb(ctrl_inb(REG_IGT_SW) & ~BIT_IGT, REG_IGT_SW)

/* MODEM Power */
// switch modem power on
#define PIN_SWI_MODEM_ON  ctrl_outb(ctrl_inb(REG_12V_DATATELE) |  BIT_MODEM_POWER, REG_12V_DATATELE)
// switch modem power off
#define PIN_SWI_MODEM_OFF ctrl_outb(ctrl_inb(REG_12V_DATATELE) & ~BIT_MODEM_POWER, REG_12V_DATATELE)

extern const u8 byte_rev_table[256];

/* Function prototypes */
static void do_sd_init_hw(int brr_value);
static int sdmodem_muxhandler(void* pv_data);



static inline void sci_transmit_chars(struct uart_port *port);

//static inline void sci_start_tx(struct uart_port *port);
static inline void sci_stop_tx(struct uart_port *port);
#define sci_stop_tx__(p)  ((void)0)
static inline void sci_stop_rx(struct uart_port *port);
#define sci_stop_rx__(p)  ((void)0)

static void sci_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old);

static void sci_set_stopped_hw(void);

static int sci_request_irq(struct sci_port *port);

static void sci_free_dma(struct uart_port *port);
static void sci_port_disable(struct sci_port *sci_port);





#define SDMMC_PORT_INDEX  (0)

#ifdef DEBUG
#define PRINTK_ECHO()   do{if(port->type==PORT_SCI){int rx=rx_count,tx=tx_count;printk(KERN_INFO "%s(%d)(%6d/%6d)\n",__FUNCTION__,__LINE__,tx,rx);}} while(0)
#else
#define PRINTK_ECHO()   do{} while(0)
#endif

#define MODEM   (1)
#define SDCARD  (0)


/* ------------- modem command buffer queue: ------------- */

#define CMD_MODEM_START_TX    (0)
#define CMD_MODEM_STOP_TX     (1)
#define CMD_MODEM_STOP_RX     (2)
#define CMD_MODEM_STARTUP     (3)
#define CMD_MODEM_SHUTDOWN    (4)
#define CMD_MODEM_SET_TERMIOS (5)
#define CMD_MODEM_START_RX    (6)


struct modem_queue_t {
        int                 cmd;       /* CMD_MODEM_START_TX,.. , CMD_MODEM_SET_TERMIOS */
        struct completion  *compl;     /* used to wait for CMD_MODEM_SHUTDOWN handling */
};



#define MAX_MODEM_REQUESTS      (64)

#define BUF_CAN_READ(wx,rx)      ((rx)!=(wx))
#define BUF_CAN_WRITE(wx,rx,sz)  ((((wx)+1)%(sz))!=(rx))
#define BUF_INC(x,sz)            (x++,(x)>=(sz)?x=0:(x))

static int modem_qerr_cnt = 0;

#define to_modem_queue(m, c)   do { \
  if (BUF_CAN_WRITE(modem_queue_wx, modem_queue_rx, MAX_MODEM_REQUESTS)) { \
    /*printk(KERN_ERR "%-20s to buf  %d\n", __FUNCTION__, modem_qerr_cnt);*/ \
    modem_queue[modem_queue_wx].cmd=(m); \
    modem_queue[modem_queue_wx].compl=(c); \
    BUF_INC(modem_queue_wx, MAX_MODEM_REQUESTS);} \
  else {printk(KERN_ERR "%s no buf space  %d\n", __FUNCTION__, modem_qerr_cnt); modem_qerr_cnt++;}wait_end=1;wake_up(&wait);} while(0)


struct modem_queue_t modem_queue[MAX_MODEM_REQUESTS];
volatile int modem_queue_wx = 0;
volatile int modem_queue_rx = 0;


/* ------------- sd command buffer queue: ------------- */

struct sd_queue_t sd_queue[MAX_SD_REQUESTS];
volatile int sd_queue_wx;
volatile int sd_queue_rx;


/* ------------- terminal+uart settings: ------------- */
struct ktermios used_termios, old_termios;
unsigned int used_baud = 0;
unsigned int used_c_cflag = 0;
//static unsigned long clk_rate = 0;
volatile int tx_count = 0;
volatile int rx_count = 0;
volatile int rx_tx_calls = 0;


/* ------------- time slice state machine: ------------- */


#define DEFAULT_SLICE               (7) // (300)        // jiffies - TODO: choose appropriate
#define MINWAIT_SLICE               (7) // ((100*264*10)/38400+1)  == 7
#define MODEM_GETREADY_SLICE        ((100*2*10)/9600+1)
#define MODEM_SLICE                 (3*DEFAULT_SLICE)
#define SD_SLICE                    DEFAULT_SLICE
#define MAX(A,B)                    ((A)>=(B)?(A):(B))
#define MIN(A,B)                    ((A)<(B)?(A):(B))
#define WAIT_SLICE                  (MAX(DEFAULT_SLICE,MINWAIT_SLICE))
#define POLLS_PER_MODEM_SLICE       (12)/*(6)*/

/* states for round robin time slices to switch between modem + sd card */
typedef enum {
        sdmodem_state_initializing  = 0,
        sdmodem_state_sdstarting    = 1,
        sdmodem_state_sdchecking2   = 2,
        sdmodem_state_sdready       = 3,
        sdmodem_state_modemstarting = 4,
        sdmodem_state_modemready    = 5,
        sdmodem_state_modempausing  = 6,
        sdmodem_state_modemwaiting  = 7,
        sdmodem_state_closing       = 8,
} sdmodem_state_t;

struct t_spi_sh7727_ops{
    irqreturn_t (*tx_irq)       (int, void *);
    irqreturn_t (*rx_irq)       (int, void *);
    irqreturn_t (*tei_irq)      (int, void *);
    irqreturn_t (*err_irq)      (int, void *);
    void        (*spi_sh_work)  (struct list_head *);
};

static struct t_spi_sh7727_ops spi_sh7727_ops;

struct sdmodem_share_t {
        struct uart_port *port;

        sdmodem_state_t state;

        unsigned int sd_jiffies;       /* _max_ period to act for sd */
        unsigned int modem_jiffies;    /* period to act for modem - this period may be prolonged */

        volatile int modem_is_selected;  /* how the muxer is switched */
        volatile int modem_is_opened;    /* an app has opend the device /dev/ttySC1 */
        //volatile int modem_is_active;    /* we switched to modem between 2 send irq chains: sending must be retriggered */
        struct list_head * spi_queue;
};

static struct sdmodem_share_t sdmodem_share = {
        .state             = sdmodem_state_initializing,
        .sd_jiffies        = SD_SLICE,
        .modem_jiffies     = MODEM_SLICE,
        .modem_is_selected = 0,
        //.modem_is_active   = 0,
        .modem_is_opened   = 0,
};

#define MAX_MODEMXOFF_CHARS     (280)   /* modem can send up to this # of chars **after** it got an XOFF */
#define BITS                    (11)    /* we have 8N1 + 1 space ==> 10. Use 11 to make timing a bit longer */

#define IDX_9600    (0)
#define IDX_19200   (1)
#define IDX_38400   (2)
#define IDX_57600   (3)
#define IDX_115200  (4)
#define IDX_DEFAULT  IDX_9600
static struct timespec wait_slice_ktimes[] = {
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/MAX_MODEMXOFF_CHARS/(9600/BITS),        },
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/MAX_MODEMXOFF_CHARS/(19200/BITS),       },
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/MAX_MODEMXOFF_CHARS/(38400/BITS),       },
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/MAX_MODEMXOFF_CHARS/(57600/BITS),       },
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/MAX_MODEMXOFF_CHARS/(115200/BITS),      },
};
static struct timespec *wait_slice_ktime = &(wait_slice_ktimes[IDX_DEFAULT]);




static struct timespec onechar_ktimes[] = {
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/(9600/BITS),    },
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/(19200/BITS),   },
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/(38400/BITS),   },
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/(57600/BITS),   },
        {       .tv_sec = 0,
                .tv_nsec = NSEC_PER_SEC/(115200/BITS),  },
};
static struct timespec *onechar_ktime = &(onechar_ktimes[IDX_DEFAULT]);

static struct timespec sdmodem_poll_ktime = {
        .tv_sec = (MODEM_SLICE/POLLS_PER_MODEM_SLICE)/HZ,
        .tv_nsec = ((MODEM_SLICE/POLLS_PER_MODEM_SLICE)%HZ)*(NSEC_PER_SEC/HZ),
};


/* ------------- Xon/Xoff: ------------- */

#define XON     ((unsigned char)17)
#define XOFF    ((unsigned char)19)

/* These are the requesters of Xon/Xoff:*/
static volatile unsigned char last_tty_x_request = 0;
static volatile unsigned char last_mux_x_request = 0;
static volatile unsigned char last_mod_x_request = 0;

static volatile unsigned int sci_stop_rx_request = 0;    /* handle muxer switch request */
static volatile unsigned int sci_stop_tx_request = 0;    /* handle muxer switch request */
static volatile unsigned int sci_stopped = 1;            /* handle shutdown */

static volatile unsigned char mux_x_request = 0;

inline void modem_flow_ctrl(unsigned char flow_ctrl, struct uart_port *port)
{
        mux_x_request = flow_ctrl;
        sci_start_tx(port);
        return;
}

void tty_flow_ctrl(unsigned char flow_ctrl, struct uart_port *port, struct tty_struct *tty)
{
        if (!sci_stopped) {
                tty_insert_flip_char(tty, flow_ctrl, TTY_NORMAL);
        }
        else {
        }
        return;
}



/* locking the muxer towards modem : */
DECLARE_WAIT_QUEUE_HEAD(modemlock_queue);
static char may_continue = 0;


static int modem_connected = 0;

static volatile int modem_lock_requested = 0;
static volatile int modem_lock_acknowleged = 1;

//void modemlock_callback(int dummy);  /* scisdmon.c */
void modemlock_callback(int dummy)
{
        //printk(KERN_INFOmay_continue "%s\n", __FUNCTION__);
        may_continue = 1;
        wake_up_interruptible(&modemlock_queue);
}


/* request exclusive access to modem/disable(enable) muxer */
int modem_request_lock(int lock)
{
        if (!modem_connected)
                return -ENXIO;

        if (lock)
                modem_lock_acknowleged = 0;
        modem_lock_requested = lock;
        return 0;
}


static int sdmodem_muxhandler(void* pv_data)
{
    int i;
        struct timespec ts_end, ts_now;
        struct sdmodem_share_t *p_share = (struct sdmodem_share_t *) pv_data;
        modem_connected = PIN_CHK_GSM;

        init_waitqueue_head (&wait);
printk(KERN_DEBUG"sdmodem_poll_ktime.tv_sec %ld  sdmodem_poll_ktime.tv_nsec %ld\n",sdmodem_poll_ktime.tv_sec, sdmodem_poll_ktime.tv_nsec);
for ( i=0;i<5;i++)
    printk(KERN_DEBUG"%d: wait_slice_ktime.tv_sec %ld  wait_slice_ktime.tv_nsec %ld\n",i,wait_slice_ktime[i].tv_sec, wait_slice_ktime[i].tv_nsec);
for ( i=0;i<5;i++)
    printk(KERN_DEBUG"%d: onechar_ktime.tv_sec %ld  onechar_ktime.tv_nsec %ld\n",i,onechar_ktime[i].tv_sec, onechar_ktime[i].tv_nsec);



        while (1) {
                switch (p_share->state) {
                case sdmodem_state_initializing:
                        sci_set_stopped_hw();
                        p_share->modem_is_opened = 0;
                        //p_share->modem_is_active = 0;
                        /* Disable MMC in the MUX */
                        ctrl_outb(ctrl_inb(PORT_PEDR) | (1<<7), PORT_PEDR);
                        /*   switch back and forth */
                        /* Enable MMC in the MUX */
                        ctrl_outb(ctrl_inb(PORT_PEDR) & ~(1<<7), PORT_PEDR);
                        do_sd_init_hw(0);

                        /* Disable MMC in the MUX */
                        ctrl_outb(ctrl_inb(PORT_PEDR) | (1<<7), PORT_PEDR);
                        sci_set_termios(p_share->port, &used_termios, &old_termios);

                        if (modem_connected) {
                            printk(KERN_INFO "%s() - sdmodem_state_initializing (modem detected)\n", __FUNCTION__);

                                 p_share->modem_is_selected = 0;  /* force switching */
                                 p_share->state = sdmodem_state_modemstarting;
                        }
                        else {  /* no modem is connected */
                            printk(KERN_INFO "%s() - sdmodem_state_initializing (no modem)\n", __FUNCTION__);

                                /* Enable MMC in the MUX */
                                ctrl_outb(ctrl_inb(PORT_PEDR) & ~(1<<7), PORT_PEDR);
                                do_sd_init_hw(0);
                                p_share->modem_is_selected = 0;
                                p_share->state = sdmodem_state_sdready;
                        }
                        break;

                case sdmodem_state_sdstarting:
                         if (last_mod_x_request==XOFF) { /* abort and get a modem again: wait until modem has sent XON */
//                                TRACE(0x8e);
                                 sci_start_rx(p_share->port /*, 0*/);
                                 p_share->state = sdmodem_state_modemstarting;
                                 break;
                         }
                        //printk(KERN_INFO "%s() - sdmodem_state_sdstarting\n", __FUNCTION__);
                         ktime_get_ts(&ts_end);
                         set_normalized_timespec(&ts_end, ts_end.tv_sec, ts_end.tv_nsec+p_share->sd_jiffies*(NSEC_PER_SEC/HZ));
                        /* break; */

                case sdmodem_state_sdchecking2:
                         //if (BUF_CAN_READ(sd_queue_wx, sd_queue_rx)) { /* sd wants access */
                         if (!list_empty(sdmodem_share.spi_queue)){
                                 if (p_share->modem_is_selected) { /* switch to sd: */
//                                        TRACE(0x87);
                                        /* Enable MMC in the MUX */
                                         ctrl_outb(ctrl_inb(PORT_PEDR) & ~(1<<7), PORT_PEDR);
                                         do_sd_init_hw(0);
                                         p_share->modem_is_selected = 0;
                                 }
                                 p_share->state = sdmodem_state_sdready;
                         }
                         else {
                                 ktime_get_ts(&ts_now);
                                 if (timespec_compare(&ts_now, &ts_end) < 0) {
                                         sys_nanosleep(&sdmodem_poll_ktime, NULL);
                                         p_share->state = sdmodem_state_sdchecking2;
                                         break;
                                 }
                                 else {
                                        p_share->state = sdmodem_state_modemstarting;
                                         break;
                                 }
                         }
                        /* break; */

                case sdmodem_state_sdready:
                         if (modem_connected || BUF_CAN_READ(modem_queue_wx, modem_queue_rx)) {
                                /* only try to switch to modem, when 1) a modem is connected
                                 * or 2) commands from ioctl s are found.
                                 */
                                 ktime_get_ts(&ts_now);
                                 if (timespec_compare(&ts_now, &ts_end) >= 0) {
//                                                ktime_get_ts(&ts_end);
//                                                set_normalized_timespec(&ts_end, ts_end.tv_sec, ts_end.tv_nsec+p_share->modem_jiffies*(NSEC_PER_SEC/HZ));
                                         p_share->state = sdmodem_state_modemstarting;
                                                break;
                                 }
                         }
//                         if (BUF_CAN_READ(sd_queue_wx, sd_queue_rx)) {
                         if ( !list_empty(sdmodem_share.spi_queue)){
                             spi_sh7727_ops.spi_sh_work(sdmodem_share.spi_queue);
                             schedule();
                         }
                         else{
                             sys_nanosleep(&sdmodem_poll_ktime, NULL);
                         }
                        /* keep this state:  p_share->state = sdmodem_state_sdready; */
                       break;

                case sdmodem_state_modemstarting:
                        //printk(KERN_INFO "%s() - sdmodem_state_modemstarting\n", __FUNCTION__);
                         ktime_get_ts(&ts_end);
                         set_normalized_timespec(&ts_end, ts_end.tv_sec, ts_end.tv_nsec+p_share->modem_jiffies*(NSEC_PER_SEC/HZ));
                         if (!p_share->modem_is_selected) {
//                                TRACE(0x9c);
                                /* Disable MMC in the MUX */
                                 ctrl_outb(ctrl_inb(PORT_PEDR) | (1<<7), PORT_PEDR);
                                 sci_set_termios(p_share->port, &used_termios, &old_termios);
                                 p_share->modem_is_selected = 1;
                         }
                         if (p_share->modem_is_opened) {
                                //TRACE(0x9d);
                                //sci_start_rx(p_share->port, 0);
sci_stop_tx_request = 0;
//sci_stop_rx_request = 0;
                                tty_flow_ctrl(XON, p_share->port, p_share->port->state->port.tty/* p_share->port->info->tty */);
                                modem_flow_ctrl(XON, p_share->port);
//                                TRACE(0x9e);
                                uart_write_wakeup(p_share->port);
//                                TRACE(0x9f);
                         }
                        //p_share->modem_is_active = 1;

                        p_share->state = sdmodem_state_modemready;
                        /* break; */

                case sdmodem_state_modemready:
                            if (modem_lock_requested) {
                                    if (!modem_lock_acknowleged) {
                                        modemlock_callback(0/*dummy*/);
                                        modem_lock_acknowleged = 1;
                                    }
                            }
                             while (BUF_CAN_READ(modem_queue_wx, modem_queue_rx)) {
                                //printk(KERN_INFO "%s() - sdmodem_state_modemready cmd=%s\n", __FUNCTION__, show_command(modem_queue[modem_queue_rx].cmd));
                                 switch (modem_queue[modem_queue_rx].cmd) {
                                 case CMD_MODEM_START_TX:
                                         sci_start_tx(p_share->port);
                                         break;
                                         
                                 case CMD_MODEM_START_RX:
                                         sci_start_rx(p_share->port);
                                         break;

                                 case CMD_MODEM_STOP_TX:
                                         sci_stop_tx__(p_share->port);
                                         break;

                                 case CMD_MODEM_STOP_RX:
                                         sci_stop_rx(p_share->port);
                                         break;

                                 case CMD_MODEM_STARTUP:
                                         sci_stopped = 0;
                                         p_share->modem_is_opened = 1;
                                         break;

                                 case CMD_MODEM_SHUTDOWN:
                                         p_share->modem_is_opened = 0;
                                         sci_stop_rx(p_share->port);
                                         sci_stop_tx(p_share->port);
                                         
                                         { /* for sci_shutdown */ 
                                            struct sci_port *s = to_sci_port(p_share->port);
                                            sci_free_dma(p_share->port);
                                            sci_port_disable(s);
                                         }
                                         
                                         tx_count = 0;
                                         rx_count = 0;
                                         rx_tx_calls = 0;

                                        /* let tty layer continue */
                                         complete(modem_queue[modem_queue_rx].compl);

                                         BUF_INC(modem_queue_rx, MAX_MODEM_REQUESTS);
                                         p_share->state = sdmodem_state_modemwaiting;
                                         goto l_sdmodem_state_modemwaiting;
                                        /*break;*/

                                 case CMD_MODEM_SET_TERMIOS:
                                         sci_set_termios(p_share->port, &used_termios, &old_termios);
                                         p_share->modem_is_selected = 1;
                                         break;

                                 default:
                                         break;
                                 }  /* switch (modem_queue[modem_queue_rx].cmd) */
                                 BUF_INC(modem_queue_rx, MAX_MODEM_REQUESTS);
                                 if (modem_lock_requested) {
                                     
                                         if (!modem_lock_acknowleged) {
                                                modemlock_callback(0/*dummy*/);
                                                modem_lock_acknowleged = 1;
                                         }
                                 }
                                 else {
                                         ktime_get_ts(&ts_now);
                                         if (timespec_compare(&ts_now, &ts_end) >= 0) {
                                                 //if (!BUF_CAN_READ(sd_queue_wx, sd_queue_rx)) {
                                                 if (list_empty(sdmodem_share.spi_queue)){
                                                     continue /*while*/;
                                                 }
                                                 p_share->state = sdmodem_state_modemwaiting;
                                                 goto l_sdmodem_state_modemwaiting;
                                         }
                                 }
                         }  /* while (BUF_CAN_READ(modem_queue_wx, modem_queue_rx)) */
                        p_share->state = sdmodem_state_modempausing;
                        break;

                case sdmodem_state_modempausing:
                         ktime_get_ts(&ts_now);
                         if (modem_lock_requested ||
                             (timespec_compare(&ts_now, &ts_end) < 0) ||
                             (list_empty(sdmodem_share.spi_queue)) ) {
                                 sys_nanosleep(&sdmodem_poll_ktime, NULL);/* t= ???? */
                                 //wait_event_timeout( wait, wait_end==1, 1 );
                                 //wait_end=0;
                                 p_share->state = sdmodem_state_modemready;
                                 break;
                         }
                         p_share->state = sdmodem_state_modemwaiting;
                        /* break; */

                case sdmodem_state_modemwaiting:
                    l_sdmodem_state_modemwaiting:
                         if (p_share->modem_is_opened ) {
                             /* stop sending to modem + wait for returned chars (up to 275 expected) */
                                 modem_flow_ctrl(XOFF, p_share->port);
                                 tty_flow_ctrl(XOFF, p_share->port, p_share->port->state->port.tty/* p_share->port->info->tty */);

                                 /* stop sending to modem + wait for returned chars (up to 275 expected) */
                                  sci_stop_tx_request = 1;
                                 sys_nanosleep(wait_slice_ktime, NULL);
                                 if (last_mod_x_request==XOFF) { /* abort and get a modem again: wait until modem has sent XON */
                                        p_share->state = sdmodem_state_modemstarting;
                                         break;
                                 }
                                 else {
                                        /* stop receiving and wait until receiving has stopped */
                                         sci_stop_rx_request = 1;
                                         sys_nanosleep(onechar_ktime, NULL);
                                 }
                         }
                         p_share->state = sdmodem_state_sdstarting;
                         break;

                 case sdmodem_state_closing:     // still unused
                 default:
                         return 1;

                }  /* switch( p_share->state ) */

        }  /* while (1) */
        return 0;
}

#endif  /* #if USE_MULTIPLEXER */

static void do_sd_init_hw(int brr_value)
{
#if 1
/* -------------------------------------------------------------------------- */
        ctrl_outb(0, SCSCR);
        ctrl_outb(0, SCSCR);

        /*  SCSMR -  setting clock rates:
         *      Bit 1: CKS1 Bit 0: CKS0 Description
         *      0           0           Pphi
         *      0           1           Pphi/4
         *      1           0           Pphi/16
         *      1           1           Pphi/64
         */
        ctrl_outb((1<<7/*C/A*/) | (0<<1/*CKS1*/) | (0<<0/*CKS0*/), SCSMR);

        /* Note: No usage of SCI hardware "SD mode" */

        //ctrl_outb(0, SCBBR);   /* Set baudrate */    /* fastest setting:     8clcks/960ns  = 8.33MHz */
        //ctrl_outb(1, SCBBR);   /* Set baudrate */    /* 2nd fastest setting: 8clcks/1920ns = 4.17MHz */
        //ctrl_outb(15, SCBBR);    /* Set baudrate */    /* Nylunds setting:     8clcks/14.4us = 0.556MHz */

        ctrl_outb(brr_value, SCBBR);    /* Set baudrate */

        /* All SD cards have at least 20MHz and a R2W_Factor of 32 (at most). So 20MHz/32 = 0.625MHz ist
         * the lowest admittable clck frequency.
         */
        udelay(2);   /* should be enough to wait for minimum one bit period */
        //mdelay(1);              /* wait for minimum one bit period */

        ctrl_outb((0<<1/*CKE1*/) | (0<<0/*CKE0*/), SCSCR);      /* CKE1 and CKE0 */
        ctrl_outb(ctrl_inb(SCSR) & ~(SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSR);      /* clear any error flags */
/* -------------------------------------------------------------------------- */
#else
        /* Initialise SCI for synchronous mode */
        ctrl_outb(0, SCSCR);    /* clear TE and RE */
        ctrl_outb(0, SCSCR);    /* CKE1 and CKE0 */
        ctrl_outb((1<<7/*C/A*/), SCSMR);        /* synchronous mode, clock divider=1 */

        /* Refer to table 17.5 "Bitrates and SCBRR Settings in clock
        * synchronous mode" in sh7727 datasheet */

        ctrl_outb(15, SCBBR);   /* Set baudrate */

        mdelay(1);              /* wait for minimum one bit period */
        ctrl_outb(ctrl_inb(SCSCR) | (SCSCR_RE | SCSCR_TE), SCSCR);      /* enable transmitter and receiver.  */
        ctrl_outb(ctrl_inb(SCSR) & ~(SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSR);      /* clear any error flags */

#warning using Nylunds init

/* -------------------------------------------------------------------------- */
#endif

}




/*----------------------------------------------------------------------------*/
/* function to support spi-sh7727                                             */
/*----------------------------------------------------------------------------*/


/* !!! fixed irq numbering, special for SC1000 !!! */
#define SCI_ERI_IRQ 23
#define SCI_RXI_IRQ 24
#define SCI_TXI_IRQ 25
#define SCI_TEI_IRQ 26

int shsci_request_irq(unsigned int irq, irq_handler_t handler)
{
    switch (irq){
        case SCI_ERI_IRQ:
            spi_sh7727_ops.err_irq = handler;
            break;
        case SCI_RXI_IRQ:
            spi_sh7727_ops.rx_irq = handler;
            break;
        case SCI_TXI_IRQ:
            spi_sh7727_ops.tx_irq = handler;
            break;
        case SCI_TEI_IRQ:
            spi_sh7727_ops.tei_irq = handler;
            break;
        default:
            return -ENODEV;
    }
    return 0;
}
int shsci_free_irq(unsigned int irq)
{
    switch (irq){
        case SCI_ERI_IRQ:
            spi_sh7727_ops.err_irq = NULL;
            break;
        case SCI_RXI_IRQ:
            spi_sh7727_ops.rx_irq = NULL;
            break;
        case SCI_TXI_IRQ:
            spi_sh7727_ops.tx_irq = NULL;
            break;
        case SCI_TEI_IRQ:
            spi_sh7727_ops.tei_irq = NULL;
            break;
        default:
            return -ENODEV;
    }
    return 0;
}


/* called from the mmc_spi driver */
/* start multiplexer after the spi driver is ready */
int shsci_init_work(struct list_head *spi_queue,void (*spi_sh_work)(struct list_head * spi_queue))
{
        spi_sh7727_ops.spi_sh_work   = spi_sh_work;
        sdmodem_share.spi_queue      = spi_queue;
        /* start multiplexer */
        kthread_run(sdmodem_muxhandler, (void*)&sdmodem_share, "sdmodemd");
        return 0;
}



//------------------ original functions ---------------------------------------

/*
 * The "offset" here is rather misleading, in that it refers to an enum
 * value relative to the port mapping rather than the fixed offset
 * itself, which needs to be manually retrieved from the platform's
 * register map for the given port.
 */
static unsigned int sci_serial_in(struct uart_port *p, int offset)
{
	struct plat_sci_reg *reg = sci_getreg(p, offset);
	if (reg->size == 8)
		return ioread8(p->membase + (reg->offset << p->regshift));
	else if (reg->size == 16)
		return ioread16(p->membase + (reg->offset << p->regshift));
	else
		WARN(1, "Invalid register access\n");

	return 0;
}

static void sci_serial_out(struct uart_port *p, int offset, int value)
{
	struct plat_sci_reg *reg = sci_getreg(p, offset);

        if (reg->size == 8)
		iowrite8(value, p->membase + (reg->offset << p->regshift));
	else if (reg->size == 16)
		iowrite16(value, p->membase + (reg->offset << p->regshift));
	else
		WARN(1, "Invalid register access\n");
}

#define sci_in(up, offset)		(up->serial_in(up, offset))
#define sci_out(up, offset, value)	(up->serial_out(up, offset, value))

static int sci_probe_regmap(struct plat_sci_port *cfg)
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

static void sci_port_enable(struct sci_port *sci_port)
{
	if (!sci_port->port.dev)
		return;

	pm_runtime_get_sync(sci_port->port.dev);

	clk_enable(sci_port->iclk);
	sci_port->port.uartclk = clk_get_rate(sci_port->iclk);
	clk_enable(sci_port->fclk);
}

static void sci_port_disable(struct sci_port *sci_port)
{
	if (!sci_port->port.dev)
		return;

	clk_disable(sci_port->fclk);
	clk_disable(sci_port->iclk);

	pm_runtime_put_sync(sci_port->port.dev);
}

#if defined(CONFIG_CONSOLE_POLL) || defined(CONFIG_SERIAL_SH_SCI_CONSOLE)

#ifdef CONFIG_CONSOLE_POLL
static int sci_poll_get_char(struct uart_port *port)
{
	unsigned short status;
	int c;

	do {
		status = sci_in(port, SH_SCI_SCxSR);
		if (status & SCxSR_ERRORS(port)) {
			sci_out(port, SH_SCI_SCxSR, SCxSR_ERROR_CLEAR(port));
			continue;
		}
		break;
	} while (1);

	if (!(status & SCxSR_RDxF(port)))
		return NO_POLL_CHAR;

	c = sci_in(port, SH_SCI_SCxRDR);

	/* Dummy read */
	sci_in(port, SH_SCI_SCxSR);
	sci_out(port, SH_SCI_SCxSR, SCxSR_RDxF_CLEAR(port));

	return c;
}
#endif

static void sci_poll_put_char(struct uart_port *port, unsigned char c)
{
	unsigned short status;

	do {
		status = sci_in(port, SH_SCI_SCxSR);
	} while (!(status & SCxSR_TDxE(port)));

	sci_out(port, SH_SCI_SCxTDR, c);
	sci_out(port, SH_SCI_SCxSR, SCxSR_TDxE_CLEAR(port) & ~SCxSR_TEND(port));
}
#endif /* CONFIG_CONSOLE_POLL || CONFIG_SERIAL_SH_SCI_CONSOLE */

static void sci_init_pins(struct uart_port *port, unsigned int cflag)
{
	struct sci_port *s = to_sci_port(port);
	struct plat_sci_reg *reg = sci_regmap[s->cfg->regtype] + SH_SCI_SCSPTR;

	/*
	 * Use port-specific handler if provided.
	 */
	if (s->cfg->ops && s->cfg->ops->init_pins) {
		s->cfg->ops->init_pins(port, cflag);
		return;
	}

	/*
	 * For the generic path SCSPTR is necessary. Bail out if that's
	 * unavailable, too.
	 */
	if (!reg->size)
		return;

	if (!(cflag & CRTSCTS))
		sci_out(port, SH_SCI_SCSPTR, 0x0080); /* Set RTS = 1 */
}

// static int sci_txfill(struct uart_port *port)
// {
// 	struct plat_sci_reg *reg;
// 
// 	reg = sci_getreg(port, SH_SCI_SCTFDR);
// 	if (reg->size)
// 		return sci_in(port, SH_SCI_SCTFDR) & 0xff;
// 
// 	reg = sci_getreg(port, SH_SCI_SCFDR);
// 	if (reg->size)
// 		return sci_in(port, SH_SCI_SCFDR) >> 8;
// 
// 	return !(sci_in(port, SH_SCI_SCxSR) & SCI_TDRE);
// }

// static int sci_txroom(struct uart_port *port)
// {
// 	return port->fifosize - sci_txfill(port);
// }

// static int sci_rxfill(struct uart_port *port)
// {
// 	struct plat_sci_reg *reg;
// 
// 	reg = sci_getreg(port, SH_SCI_SCRFDR);
// 	if (reg->size)
// 		return sci_in(port, SH_SCI_SCRFDR) & 0xff;
// 
// 	reg = sci_getreg(port, SH_SCI_SCFDR);
// 	if (reg->size)
// 		return sci_in(port, SH_SCI_SCFDR) & ((port->fifosize << 1) - 1);
// 
// 	return (sci_in(port, SH_SCI_SCxSR) & SCxSR_RDxF(port)) != 0;
// }

/*
 * SCI helper for checking the state of the muxed port/RXD pins.
 */
static inline int sci_rxd_in(struct uart_port *port)
{
	struct sci_port *s = to_sci_port(port);

	if (s->cfg->port_reg <= 0)
		return 1;

	return !!__raw_readb(s->cfg->port_reg);
}

/* ********************************************************************** *
 *                   the interrupt related routines                       *
 * ********************************************************************** */

// static void sci_transmit_chars(struct uart_port *port)
// {
// 	struct circ_buf *xmit = &port->state->xmit;
// 	unsigned int stopped = uart_tx_stopped(port);
// 	unsigned short status;
// 	unsigned short ctrl;
// 	int count;
// 
// 	status = sci_in(port, SH_SCI_SCxSR);
// 	if (!(status & SCxSR_TDxE(port))) {
// 		ctrl = sci_in(port, SH_SCI_SCSCR);
// 		if (uart_circ_empty(xmit))
// 			ctrl &= ~SCSCR_TIE;
// 		else
// 			ctrl |= SCSCR_TIE;
// 		sci_out(port, SH_SCI_SCSCR, ctrl);
// 		return;
// 	}
// 
// 	count = sci_txroom(port);
// 
// 	do {
// 		unsigned char c;
// 
// 		if (port->x_char) {
// 			c = port->x_char;
// 			port->x_char = 0;
// 		} else if (!uart_circ_empty(xmit) && !stopped) {
// 			c = xmit->buf[xmit->tail];
// 			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
// 		} else {
// 			break;
// 		}
// 
// 		sci_out(port, SH_SCI_SCxTDR, c);
// 
// 		port->icount.tx++;
// 	} while (--count > 0);
// 
// 	sci_out(port, SH_SCI_SCxSR, SCxSR_TDxE_CLEAR(port));
// 
// 	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
// 		uart_write_wakeup(port);
// 	if (uart_circ_empty(xmit)) {
// 		sci_stop_tx(port);
// 	} else {
// 		ctrl = sci_in(port, SH_SCI_SCSCR);
// 
// 		if (port->type != PORT_SCI) {
// 			sci_in(port, SH_SCI_SCxSR); /* Dummy read */
// 			sci_out(port, SH_SCI_SCxSR, SCxSR_TDxE_CLEAR(port));
// 		}
// 
// 		ctrl |= SCSCR_TIE;
// 		sci_out(port, SH_SCI_SCSCR, ctrl);
// 	}
// }

/* On SH3, SCIF may read end-of-break as a space->mark char */
#define STEPFN(c)  ({int __c = (c); (((__c-1)|(__c)) == -1); })

static void sci_receive_chars(struct uart_port *port)
{
        struct sci_port *sci_port = to_sci_port(port);
        struct tty_struct *tty = port->state->port.tty;
        int count, copied = 0;
        unsigned char status;
        unsigned char c;
        if (sci_stop_rx_request) { /* muxer request */
                sci_stop_rx_request = 0;
                sci_stop_rx(port);
                return;
        }

        if (sci_stopped) {
                //TRACE(0xc6);
                sci_stop_rx(port);
                return;
        }

        status = ctrl_inb(SCSR);

        if (!(status & SCI_RDRF)) {
                return;
        }

        do {
                /* Don't copy more bytes than there is room for in the buffer */
                count = tty_buffer_request_room(tty, 1);
                /* If for any reason we can't copy more data, we're done! */
                if (count == 0) {
                        //TRACE(0xb5);
                        c = ctrl_inb(SCRDR);
                        break;
                }

                c = ctrl_inb(SCRDR);

                rx_count++;
                if (uart_handle_sysrq_char(port, c) || sci_port->break_flag) {
                        count = 0;
                }
                else {
                        tty_insert_flip_char(tty, c, TTY_NORMAL);
                        if (c==XON || c==XOFF) {
                                last_mod_x_request = c;
                        }
                }
                ctrl_inb(SCSR);        /* dummy read */
                ctrl_outb(ctrl_inb(SCSR) & ~SCI_RDRF, SCSR);

                copied += count;
                port->icount.rx += count;
        } while (0);

        if (copied) {
                /* Tell the rest of the system the news. New characters! */
                tty_flip_buffer_push(tty);
        }
        else {
                ctrl_inb(SCSCR);        /* dummy read */
                ctrl_outb(ctrl_inb(SCSCR) & ~SCI_RDRF, SCSCR);
        }

// 	struct sci_port *sci_port = to_sci_port(port);
// 	struct tty_struct *tty = port->state->port.tty;
// 	int i, count, copied = 0;
// 	unsigned short status;
// 	unsigned char flag;
// 
// 	status = sci_in(port, SH_SCI_SCxSR);
// 	if (!(status & SCxSR_RDxF(port)))
// 		return;
// 
// 	while (1) {
// 		/* Don't copy more bytes than there is room for in the buffer */
// 		count = tty_buffer_request_room(tty, sci_rxfill(port));
// 
// 		/* If for any reason we can't copy more data, we're done! */
// 		if (count == 0)
// 			break;
// 
// 		if (port->type == PORT_SCI) {
// 			char c = sci_in(port, SH_SCI_SCxRDR);
// 			if (uart_handle_sysrq_char(port, c) ||
// 			    sci_port->break_flag)
// 				count = 0;
// 			else
// 				tty_insert_flip_char(tty, c, TTY_NORMAL);
// 		} else {
// 			for (i = 0; i < count; i++) {
// 				char c = sci_in(port, SH_SCI_SCxRDR);
// 				status = sci_in(port, SH_SCI_SCxSR);
// #if defined(CONFIG_CPU_SH3)
// 				/* Skip "chars" during break */
// 				if (sci_port->break_flag) {
// 					if ((c == 0) &&
// 					    (status & SCxSR_FER(port))) {
// 						count--; i--;
// 						continue;
// 					}
// 
// 					/* Nonzero => end-of-break */
// 					dev_dbg(port->dev, "debounce<%02x>\n", c);
// 					sci_port->break_flag = 0;
// 
// 					if (STEPFN(c)) {
// 						count--; i--;
// 						continue;
// 					}
// 				}
// #endif /* CONFIG_CPU_SH3 */
// 				if (uart_handle_sysrq_char(port, c)) {
// 					count--; i--;
// 					continue;
// 				}
// 
// 				/* Store data and status */
// 				if (status & SCxSR_FER(port)) {
// 					flag = TTY_FRAME;
// 					dev_notice(port->dev, "frame error\n");
// 				} else if (status & SCxSR_PER(port)) {
// 					flag = TTY_PARITY;
// 					dev_notice(port->dev, "parity error\n");
// 				} else
// 					flag = TTY_NORMAL;
// 
// 				tty_insert_flip_char(tty, c, flag);
// 			}
// 		}
// 
// 		sci_in(port, SH_SCI_SCxSR); /* dummy read */
// 		sci_out(port, SH_SCI_SCxSR, SCxSR_RDxF_CLEAR(port));
// 
// 		copied += count;
// 		port->icount.rx += count;
// 	}
// 
// 	if (copied) {
// 		/* Tell the rest of the system the news. New characters! */
// 		tty_flip_buffer_push(tty);
// 	} else {
// 		sci_in(port, SH_SCI_SCxSR); /* dummy read */
// 		sci_out(port, SH_SCI_SCxSR, SCxSR_RDxF_CLEAR(port));
// 	}
}

#define SCI_BREAK_JIFFIES (HZ/20)

/*
 * The sci generates interrupts during the break,
 * 1 per millisecond or so during the break period, for 9600 baud.
 * So dont bother disabling interrupts.
 * But dont want more than 1 break event.
 * Use a kernel timer to periodically poll the rx line until
 * the break is finished.
 */
static inline void sci_schedule_break_timer(struct sci_port *port)
{
	mod_timer(&port->break_timer, jiffies + SCI_BREAK_JIFFIES);
}

/* Ensure that two consecutive samples find the break over. */
static void sci_break_timer(unsigned long data)
{
	struct sci_port *port = (struct sci_port *)data;

	sci_port_enable(port);

	if (sci_rxd_in(&port->port) == 0) {
		port->break_flag = 1;
		sci_schedule_break_timer(port);
	} else if (port->break_flag == 1) {
		/* break is over. */
		port->break_flag = 2;
		sci_schedule_break_timer(port);
	} else
		port->break_flag = 0;

	sci_port_disable(port);
}

static int sci_handle_errors(struct uart_port *port)
{
	int copied = 0;
	unsigned short status = sci_in(port, SH_SCI_SCxSR);
	struct tty_struct *tty = port->state->port.tty;
	struct sci_port *s = to_sci_port(port);

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
		if (sci_rxd_in(port) == 0) {
			/* Notify of BREAK */
			struct sci_port *sci_port = to_sci_port(port);

			if (!sci_port->break_flag) {
				sci_port->break_flag = 1;
				sci_schedule_break_timer(sci_port);

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

static int sci_handle_fifo_overrun(struct uart_port *port)
{
	struct tty_struct *tty = port->state->port.tty;
	struct sci_port *s = to_sci_port(port);
	struct plat_sci_reg *reg;
	int copied = 0;

	reg = sci_getreg(port, SH_SCI_SCLSR);
	if (!reg->size)
		return 0;

	if ((sci_in(port, SH_SCI_SCLSR) & (1 << s->cfg->overrun_bit))) {
		sci_out(port, SH_SCI_SCLSR, 0);

		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		tty_flip_buffer_push(tty);

		dev_notice(port->dev, "overrun error\n");
		copied++;
	}

	return copied;
}

static int sci_handle_breaks(struct uart_port *port)
{
    sci_transmit_chars(port);
        return 0;
// 	int copied = 0;
// 	unsigned short status = sci_in(port, SH_SCI_SCxSR);
// 	struct tty_struct *tty = port->state->port.tty;
// 	struct sci_port *s = to_sci_port(port);
// 
// 	if (uart_handle_break(port))
// 		return 0;
// 
// 	if (!s->break_flag && status & SCxSR_BRK(port)) {
// #if defined(CONFIG_CPU_SH3)
// 		/* Debounce break */
// 		s->break_flag = 1;
// #endif
// 		/* Notify of BREAK */
// 		if (tty_insert_flip_char(tty, 0, TTY_BREAK))
// 			copied++;
// 
// 		dev_dbg(port->dev, "BREAK detected\n");
// 	}
// 
// 	if (copied)
// 		tty_flip_buffer_push(tty);
// 
// 	copied += sci_handle_fifo_overrun(port);
// 
// 	return copied;
}

static irqreturn_t sci_rx_interrupt(int irq, void *ptr)
{
#ifdef CONFIG_SERIAL_SH_SCI_DMA
	struct uart_port *port;
        struct sci_port *s;
   
        port = ptr;
        s = to_sci_port(port);
        
	if (s->chan_rx) {
		u16 scr = sci_in(port, SH_SCI_SCSCR);
		u16 ssr = sci_in(port, SH_SCI_SCxSR);

		/* Disable future Rx interrupts */
		if (port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
			disable_irq_nosync(irq);
			scr |= 0x4000;
		} else {
			scr &= ~SCSCR_RIE;
		}
		sci_out(port, SH_SCI_SCSCR, scr);
		/* Clear current interrupt */
		sci_out(port, SH_SCI_SCxSR, ssr & ~(1 | SCxSR_RDxF(port)));
		dev_dbg(port->dev, "Rx IRQ %lu: setup t-out in %u jiffies\n",
			jiffies, s->rx_timeout);
		mod_timer(&s->rx_timer, jiffies + s->rx_timeout);

		return IRQ_HANDLED;

         }
#endif
	/* I think sci_receive_chars has to be called irrespective
	 * of whether the I_IXOFF is set, otherwise, how is the interrupt
	 * to be disabled?
	 */
        
        if (sdmodem_share.modem_is_selected){
                sci_receive_chars(ptr);
                //DBG_TGL(2);
                return IRQ_HANDLED;
        }
        else{
            return spi_sh7727_ops.rx_irq(irq,ptr);            
        }
}
static inline void sci_transmit_chars(struct uart_port *port)
{
        unsigned int empty;
        int do_tx_stop = 0;
        unsigned char c;
        struct circ_buf *xmit = &port->state->xmit;
        unsigned int stopped = uart_tx_stopped(port);
        unsigned short status;

        if (sci_stop_tx_request) { /* muxer request */
                sci_stop_tx_request = 0;
                sci_stop_tx(port);
                return;
        }
        if (sci_stopped) {
                sci_stop_tx(port);
                return;
        }
        status = ctrl_inb(SCSR);
        if (!(status & SCSSR_TDRE)) {
                //TRACE(0xaa);
                if (uart_circ_empty(xmit)) {
                        sci_stop_tx(port);
                }
                else {
                        sci_start_tx(port);
                }
                return;
        }

        empty = uart_circ_empty(xmit);
        stopped = uart_tx_stopped(port);

        if ((empty || stopped) && !mux_x_request) {
                sci_stop_tx(port);
                return;
        }
        if (last_mod_x_request == XOFF) { /* we must not send except our XON */
                if (mux_x_request != XON) {
                        sci_stop_tx(port);
                        return;
                }
        }
        if (!mux_x_request && !port->x_char) {
                c = xmit->buf[xmit->tail];
                xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        }
        else {
                if (mux_x_request) {
                        last_mux_x_request = mux_x_request;
                        c = mux_x_request;
                        if (mux_x_request==XOFF) {
                                do_tx_stop = 1;
                        }
                        mux_x_request = 0;
                }
                else {  /* (port->x_char) */
                        last_tty_x_request = port->x_char;
                        if (port->x_char==XOFF || last_mux_x_request==XON) {
                                c = port->x_char;
                                if (port->x_char==XOFF) {
                                        do_tx_stop = 1;
                                }
                                port->x_char = 0;
                        }
                        else {
                                port->x_char = 0;
                                sci_stop_tx(port);
                                return;
                        }
                }
        }

        ctrl_outb(c, SCTDR);

        tx_count++;
        port->icount.tx++;

        ctrl_outb(ctrl_inb(SCSR) & ~SCSSR_TDRE, SCSR);

        if (uart_circ_chars_pending(xmit)<WAKEUP_CHARS && !do_tx_stop) {
                uart_write_wakeup(port);
        }

        if (uart_circ_empty(xmit) || do_tx_stop) {
                sci_stop_tx(port);
        }
}


static irqreturn_t sci_tx_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
	unsigned long flags;
        if (sdmodem_share.modem_is_selected){
            spin_lock_irqsave(&port->lock, flags);
            sci_transmit_chars(port);
            spin_unlock_irqrestore(&port->lock, flags);

            return IRQ_HANDLED;
        }
        else{
            return spi_sh7727_ops.tx_irq(irq,ptr);            
        }
}

static irqreturn_t sci_er_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;
        if (sdmodem_share.modem_is_selected){
            /* Handle errors */
            if (port->type == PORT_SCI) {
                    if (sci_handle_errors(port)) {
                            /* discard character in rx buffer */
                            sci_in(port, SH_SCI_SCxSR);
                            sci_out(port, SH_SCI_SCxSR, SCxSR_RDxF_CLEAR(port));
                    }
            } else {
                    sci_handle_fifo_overrun(port);
                    sci_rx_interrupt(irq, ptr);
            }

            sci_out(port, SH_SCI_SCxSR, SCxSR_ERROR_CLEAR(port));

            /* Kick the transmission */
            sci_tx_interrupt(irq, ptr);

            return IRQ_HANDLED;
        }
        else{
            return spi_sh7727_ops.err_irq(irq,ptr);            
        }
}

static irqreturn_t sci_br_interrupt(int irq, void *ptr)
{
	struct uart_port *port = ptr;

	/* Handle BREAKs */
	sci_handle_breaks(port);
	sci_out(port, SH_SCI_SCxSR, SCxSR_BREAK_CLEAR(port));

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
	return SCSCR_RIE | (to_sci_port(port)->cfg->scscr & SCSCR_REIE);
}

static irqreturn_t sci_mpxed_interrupt(int irq, void *ptr)
{
	unsigned short ssr_status, scr_status, err_enabled;
	struct uart_port *port = ptr;
	struct sci_port *s = to_sci_port(port);
	irqreturn_t ret = IRQ_NONE;

	ssr_status = sci_in(port, SH_SCI_SCxSR);
	scr_status = sci_in(port, SH_SCI_SCSCR);
	err_enabled = scr_status & port_rx_irq_mask(port);

	/* Tx Interrupt */
	if ((ssr_status & SCxSR_TDxE(port)) && (scr_status & SCSCR_TIE) &&
	    !s->chan_tx)
		ret = sci_tx_interrupt(irq, ptr);

	/*
	 * Rx Interrupt: if we're using DMA, the DMA controller clears RDF /
	 * DR flags
	 */
	if (((ssr_status & SCxSR_RDxF(port)) || s->chan_rx) &&
	    (scr_status & SCSCR_RIE))
		ret = sci_rx_interrupt(irq, ptr);

	/* Error Interrupt */
	if ((ssr_status & SCxSR_ERRORS(port)) && err_enabled)
		ret = sci_er_interrupt(irq, ptr);

	/* Break Interrupt */
	if ((ssr_status & SCxSR_BRK(port)) && err_enabled)
		ret = sci_br_interrupt(irq, ptr);

	return ret;
}

/*
 * Here we define a transition notifier so that we can update all of our
 * ports' baud rate when the peripheral clock changes.
 */
static int sci_notifier(struct notifier_block *self,
			unsigned long phase, void *p)
{
	struct sci_port *sci_port;
	unsigned long flags;

	sci_port = container_of(self, struct sci_port, freq_transition);

	if ((phase == CPUFREQ_POSTCHANGE) ||
	    (phase == CPUFREQ_RESUMECHANGE)) {
		struct uart_port *port = &sci_port->port;

		spin_lock_irqsave(&port->lock, flags);
		port->uartclk = clk_get_rate(sci_port->iclk);
		spin_unlock_irqrestore(&port->lock, flags);
	}

	return NOTIFY_OK;
}

static struct sci_irq_desc {
	const char	*desc;
	irq_handler_t	handler;
} sci_irq_desc[] = {
	/*
	 * Split out handlers, the default case.
	 */
	[SCIx_ERI_IRQ] = {
		.desc = "rx err",
		.handler = sci_er_interrupt,
	},

	[SCIx_RXI_IRQ] = {
		.desc = "rx full",
		.handler = sci_rx_interrupt,
	},

	[SCIx_TXI_IRQ] = {
		.desc = "tx empty",
		.handler = sci_tx_interrupt,
	},

	[SCIx_BRI_IRQ] = {
		.desc = "break",
		.handler = sci_br_interrupt,
	},

	/*
	 * Special muxed handler.
	 */
	[SCIx_MUX_IRQ] = {
		.desc = "mux",
		.handler = sci_mpxed_interrupt,
	},
};

static int sci_request_irq(struct sci_port *port)
{
	struct uart_port *up = &port->port;
	int i, j, ret = 0;

	for (i = j = 0; i < SCIx_NR_IRQS; i++, j++) {
		struct sci_irq_desc *desc;
		unsigned int irq;

		if (SCIx_IRQ_IS_MUXED(port)) {
			i = SCIx_MUX_IRQ;
			irq = up->irq;
		} else
			irq = port->cfg->irqs[i];

		desc = sci_irq_desc + i;
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

	return 0;

out_noirq:
	while (--i >= 0)
		free_irq(port->cfg->irqs[i], port);

out_nomem:
	while (--j >= 0)
		kfree(port->irqstr[j]);

	return ret;
}

// static void sci_free_irq(struct sci_port *port)
// {
// 	int i;
// 
// 	/*
// 	 * Intentionally in reverse order so we iterate over the muxed
// 	 * IRQ first.
// 	 */
// 	for (i = 0; i < SCIx_NR_IRQS; i++) {
// 		free_irq(port->cfg->irqs[i], port);
// 		kfree(port->irqstr[i]);
// 
// 		if (SCIx_IRQ_IS_MUXED(port)) {
// 			/* If there's only one IRQ, we're done. */
// 			return;
// 		}
// 	}
// }



// static unsigned int sci_tx_empty(struct uart_port *port)
// {
// 	unsigned short status = sci_in(port, SH_SCI_SCxSR);
// 	unsigned short in_tx_fifo = sci_txfill(port);
// 
// 	return (status & SCxSR_TEND(port)) && !in_tx_fifo ? TIOCSER_TEMT : 0;
// }


static unsigned int modem_tx_empty(struct uart_port *port)
{
        PRINTK_ECHO();
        /* Can't detect */

        return TIOCSER_TEMT;
}


static void sci_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* This routine is used for seting signals of: DTR, DCD, CTS/RTS */
	/* We use SCIF's hardware for CTS/RTS, so don't need any for that. */
	/* If you have signals for DTR and DCD, please implement here. */
}

static unsigned int sci_get_mctrl(struct uart_port *port)
{
	/* This routine is used for getting signals of: DTR, DCD, DSR, RI,
	   and CTS/RTS */

	return TIOCM_DTR | TIOCM_RTS | TIOCM_CTS | TIOCM_DSR;
}

#ifdef CONFIG_SERIAL_SH_SCI_DMA
static void sci_dma_tx_complete(void *arg)
{
	struct sci_port *s = arg;
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
			u16 ctrl = sci_in(port, SH_SCI_SCSCR);
			sci_out(port, SH_SCI_SCSCR, ctrl & ~SCSCR_TIE);
		}
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

/* Locking: called with port lock held */
static int sci_dma_rx_push(struct sci_port *s, struct tty_struct *tty,
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

static void sci_dma_rx_complete(void *arg)
{
	struct sci_port *s = arg;
	struct uart_port *port = &s->port;
	struct tty_struct *tty = port->state->port.tty;
	unsigned long flags;
	int count;

	dev_dbg(port->dev, "%s(%d) active #%d\n", __func__, port->line, s->active_rx);

	spin_lock_irqsave(&port->lock, flags);

	count = sci_dma_rx_push(s, tty, s->buf_len_rx);

	mod_timer(&s->rx_timer, jiffies + s->rx_timeout);

	spin_unlock_irqrestore(&port->lock, flags);

	if (count)
		tty_flip_buffer_push(tty);

	schedule_work(&s->work_rx);
}

static void sci_rx_dma_release(struct sci_port *s, bool enable_pio)
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
		sci_start_rx(port);
}

static void sci_tx_dma_release(struct sci_port *s, bool enable_pio)
{
	struct dma_chan *chan = s->chan_tx;
	struct uart_port *port = &s->port;

	s->chan_tx = NULL;
	s->cookie_tx = -EINVAL;
	dma_release_channel(chan);
	if (enable_pio)
		sci_start_tx(port);
}

static void sci_submit_rx(struct sci_port *s)
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
			desc->callback = sci_dma_rx_complete;
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
			sci_rx_dma_release(s, true);
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
	struct sci_port *s = container_of(work, struct sci_port, work_rx);
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
		count = sci_dma_rx_push(s, tty, sh_desc->partial);
		spin_unlock_irqrestore(&port->lock, flags);

		if (count)
			tty_flip_buffer_push(tty);

		sci_submit_rx(s);

		return;
	}

	s->cookie_rx[new] = desc->tx_submit(desc);
	if (s->cookie_rx[new] < 0) {
		dev_warn(port->dev, "Failed submitting Rx DMA descriptor\n");
		sci_rx_dma_release(s, true);
		return;
	}

	s->active_rx = s->cookie_rx[!new];

	dev_dbg(port->dev, "%s: cookie %d #%d, new active #%d\n", __func__,
		s->cookie_rx[new], new, s->active_rx);
}

static void work_fn_tx(struct work_struct *work)
{
	struct sci_port *s = container_of(work, struct sci_port, work_tx);
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
		sci_tx_dma_release(s, true);
		return;
	}

	dma_sync_sg_for_device(port->dev, sg, 1, DMA_TO_DEVICE);

	spin_lock_irq(&port->lock);
	s->desc_tx = desc;
	desc->callback = sci_dma_tx_complete;
	desc->callback_param = s;
	spin_unlock_irq(&port->lock);
	s->cookie_tx = desc->tx_submit(desc);
	if (s->cookie_tx < 0) {
		dev_warn(port->dev, "Failed submitting Tx DMA descriptor\n");
		/* switch to PIO */
		sci_tx_dma_release(s, true);
		return;
	}

	dev_dbg(port->dev, "%s: %p: %d...%d, cookie %d\n", __func__,
		xmit->buf, xmit->tail, xmit->head, s->cookie_tx);

	dma_async_issue_pending(chan);
}
#endif

static void sci_start_tx(struct uart_port *port)
{
	struct sci_port *s = to_sci_port(port);
	unsigned short ctrl;
#ifdef CONFIG_SERIAL_SH_SCI_DMA

        if (port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
		u16 new, scr = sci_in(port, SH_SCI_SCSCR);
		if (s->chan_tx)
			new = scr | 0x8000;
		else
			new = scr & ~0x8000;
		if (new != scr)
			sci_out(port, SH_SCI_SCSCR, new);
	}

	if (s->chan_tx && !uart_circ_empty(&s->port.state->xmit) &&
	    s->cookie_tx < 0) {
		s->cookie_tx = 0;
		schedule_work(&s->work_tx);
	}
#endif

        if (!s->chan_tx || port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
		/* Set TIE (Transmit Interrupt Enable) bit in SCSCR */
                ctrl = sci_in(port, SH_SCI_SCSCR);
		sci_out(port, SH_SCI_SCSCR, ctrl | SCSCR_TIE);
	}
}

static void queue_start_tx(struct uart_port *port)
{
    to_modem_queue(CMD_MODEM_START_TX, NULL);
}




static void sci_stop_tx(struct uart_port *port)
{
	unsigned short ctrl;
	/* Clear TIE (Transmit Interrupt Enable) bit in SCSCR */
	ctrl = sci_in(port, SH_SCI_SCSCR);

	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB)
		ctrl &= ~0x8000;

	ctrl &= ~SCSCR_TIE;

	sci_out(port, SH_SCI_SCSCR, ctrl);
}


static void queue_stop_tx(struct uart_port *port)
{
        PRINTK_ECHO();
        /* do nothing */
}

static void sci_start_rx(struct uart_port *port)
{
	unsigned short ctrl;
	ctrl = sci_in(port, SH_SCI_SCSCR) | port_rx_irq_mask(port);
        sci_stop_rx_request = 0;
	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB)
		ctrl &= ~0x4000;

	sci_out(port, SH_SCI_SCSCR, ctrl);
}

static void sci_stop_rx(struct uart_port *port)
{
	unsigned short ctrl;
	ctrl = sci_in(port, SH_SCI_SCSCR);

	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB)
		ctrl &= ~0x4000;

	ctrl &= ~port_rx_irq_mask(port);

	sci_out(port, SH_SCI_SCSCR, ctrl);
}

static void queue_stop_rx(struct uart_port *port)
{
        to_modem_queue(CMD_MODEM_STOP_RX, NULL);
        PRINTK_ECHO();
}


static void sci_enable_ms(struct uart_port *port)
{
	/* Nothing here yet .. */
}

static void sci_break_ctl(struct uart_port *port, int break_state)
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
	struct sci_port *s = (struct sci_port *)arg;
	struct uart_port *port = &s->port;
	u16 scr = sci_in(port, SH_SCI_SCSCR);

	if (port->type == PORT_SCIFA || port->type == PORT_SCIFB) {
		scr &= ~0x4000;
		enable_irq(s->cfg->irqs[1]);
	}
	sci_out(port, SH_SCI_SCSCR, scr | SCSCR_RIE);
	dev_dbg(port->dev, "DMA Rx timed out\n");
	schedule_work(&s->work_rx);
}

static void sci_request_dma(struct uart_port *port)
{
	struct sci_port *s = to_sci_port(port);
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
			sci_tx_dma_release(s, false);
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
			sci_rx_dma_release(s, true);
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

		sci_submit_rx(s);
	}
}

static void sci_free_dma(struct uart_port *port)
{
	struct sci_port *s = to_sci_port(port);

	if (s->chan_tx)
		sci_tx_dma_release(s, false);
	if (s->chan_rx)
		sci_rx_dma_release(s, false);
}
#else
static inline void sci_request_dma(struct uart_port *port)
{
}

static inline void sci_free_dma(struct uart_port *port)
{
}
#endif

// static int sci_startup(struct uart_port *port)
// {
// 	struct sci_port *s = to_sci_port(port);
// 	int ret;
// 
// 	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);
// 
// 	sci_port_enable(s);
// //	ret = sci_request_irq(s);
// 	if (unlikely(ret < 0))
// 		return ret;
// 
// 	sci_request_dma(port);
// 
// 	sci_start_tx(port);
// 	sci_start_rx(port);
// 
// 	return 0;
// }



static int queue_startup(struct uart_port *port)
{
        //struct sci_port *s = &sci_ports[port->line];
        struct sci_port *s = to_sci_port(port);
        int ret=0;
        if (sdmodem_share.modem_is_opened) {
                /* TODO: use spinlock + refcount here */
                printk(KERN_INFO "%s() - port already open(pid=%d)\n", __FUNCTION__, current->pid);
                return -ENODEV;
        }


//         if (s->enable)
//                 s->enable(port);
        sci_port_enable(s);

//             ret = sci_request_irq(s);


        to_modem_queue(CMD_MODEM_STARTUP, NULL);
        queue_start_tx(port);
        to_modem_queue(CMD_MODEM_START_RX, NULL);
        return ret;
}


static void queue_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
        static int run = 0;
        if (old) {
                if (run) {
                        *old = used_termios;
                }
                else {
                        *old = *termios;
                        run = 1;
                }
        }
        used_termios = *termios;

        to_modem_queue(CMD_MODEM_SET_TERMIOS, NULL);
}




static int sci_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
        int rc = 0;
//        struct sci_port *s = to_sci_port(port);
        int ival;

 

        switch (cmd) {
#if USE_MULTIPLEXER
                case TIO_SET_GSM:
                case TIO_SET_MMC:
                        printk(KERN_ERR "sci_ioctl() - auto switching by mux (not ioctl)\n");
                        break;


                case TIO_GET_GSM_RING:
                        if (access_ok(VERIFY_WRITE, (void *) arg, sizeof(unsigned int))) {
                                ival = 0;       // TODO: read RING from port
                                put_user(ival, (unsigned int *) arg);
                                printk("sci_ioctl() - TIO_GET_GSM_RING %d->(0x%08x)\n", ival, (unsigned int) arg);
                        }
                        break;
                case TIO_SET_GSM_ACTIVETIME:
                        if (access_ok(VERIFY_READ, (void *) arg, sizeof(unsigned int))) {
                                get_user(ival, (unsigned int *) arg);
                                sdmodem_share.modem_jiffies = ival;
                                printk("sci_ioctl() - TIO_SET_GSM_ACTIVETIME %d<-(0x%08x)\n", ival, (unsigned int) arg);
                        }
                        break;
                case TIO_SET_GSM_WAITTIME:  /* we misuse this ioctl to display some status info */
                        printk(KERN_ERR "sci_ioctl() - TIO_SET_GSM_WAITTIME is obsolete/misused\n");
#ifdef FLUGSCHREIBER
                        printk(KERN_INFO "enable SCIF TE for FLUGSCHREIBER\n");
                        ctrl_outb((ctrl_inb(SCSCR2) & 3) | (1<<5/*TE*/), SCSCR2);
#else
                        {
                                unsigned char status, control;
                                status = ctrl_inb(SCSR);
                                control = ctrl_inb(SCSCR);
                                printk(KERN_INFO "status: RDRF=%d TDRE=%d ORER=%d FER=%d PER=%d\n", (status&SCSSR_RDRF)?1:0, (status&SCSSR_TDRE)?1:0, (status&SCSSR_ORER)?1:0, (status&SCSSR_FER)?1:0, (status&SCSSR_PER)?1:0);
                                printk(KERN_INFO "control: RIE=%d TIE=%d TEIE=%d RE=%d TE=%d\n", (control&SCSCR_RIE)?1:0, (control&SCSCR_TIE)?1:0, (control&SCSCR_TEIE)?1:0, (control&SCSCR_RE)?1:0, (control&SCSCR_TE)?1:0);
                        }
#endif
                        break;
                case TIO_SET_MMC_ACTIVETIME:
                        if (access_ok(VERIFY_READ, (void *) arg, sizeof(unsigned int))) {
                                get_user(ival, (unsigned int *) arg);
                                sdmodem_share.sd_jiffies = ival;
                                printk("sci_ioctl() - TIO_SET_MMC_ACTIVETIME %d<-(0x%08x)\n", ival, (unsigned int) arg);
                        }
                        break;
#else
                case TIO_SET_GSM:
                        PIN_SWI_GSM;
                        break;
                case TIO_SET_MMC:
                        PIN_SWI_MMC;
                        break;
#endif
                case TIO_SET_IGT:
                        if (arg) {
                                PIN_SWI_IGT_ON;   /* Port E5 */
                        }
                        else {
                                PIN_SWI_IGT_OFF;  /* Port E5 */
                        }
                        break;
                case TIO_CHK_GSM:
                        if (access_ok(VERIFY_WRITE, (void *) arg, sizeof(unsigned int))) {
                                ival = PIN_CHK_GSM ? 1 : 0;
                                put_user(ival, (unsigned int *) arg);
                        }
                        else
                                rc = -EFAULT;
                        break;



                case TIO_SET_MODEMPWR:
                        if (arg)
                                PIN_SWI_MODEM_ON;
                        else
                                PIN_SWI_MODEM_OFF;
                        printk("sci_ioctl() - TIO_SET_MODEMPWR (%lu)\n", arg);
                        break;

                default:
                        rc = -ENOIOCTLCMD;
                        break;
                        
                /*
                 *  ioctls for locking the muxer towards modem :
                 */
                case MODEM_IOCTL_LOCK:
                        if (access_ok(VERIFY_WRITE, (void __user*)arg, sizeof(int))) {
                                DECLARE_WAITQUEUE(wait, current);
                                ival = 0;
                                rc = modem_request_lock(1);
                                printk(KERN_INFO "%s() - MODEM_IOCTL_LOCK rc=%d\n", __FUNCTION__, rc);
                                if (rc)
                                        break;

                                add_wait_queue(&modemlock_queue, &wait);
                                do {
                                        __set_current_state(TASK_INTERRUPTIBLE);
                                        if (may_continue != 0) {
                                                may_continue = 0;
                                                rc = 0;
                                                break;
                                        }
                                        if (signal_pending(current)) {
                                                rc = -ERESTARTSYS;
                                                break;
                                        }
                                        schedule();
                                } while (1);
                                __set_current_state(TASK_RUNNING);  /* must we use set_current_state ? */
                                remove_wait_queue(&modemlock_queue, &wait);

                                copy_to_user((void __user*)arg, &ival, sizeof(int));
                        }
                        break;
                case MODEM_IOCTL_UNLOCK:
                        if (access_ok(VERIFY_WRITE, (void __user*)arg, sizeof(int))) {
                                ival = 0;
                                rc = modem_request_lock(0);
                                printk(KERN_INFO "%s() - MODEM_IOCTL_UNLOCK rc=%d\n", __FUNCTION__, rc);
                                if (rc)
                                        break;
                                copy_to_user((void __user*)arg, &ival, sizeof(int));
                        }
                        break;                        
                        
                        
                        
        }

        return rc;
}




// static void sci_shutdown(struct uart_port *port)
// {
// 	struct sci_port *s = to_sci_port(port);
// 	dev_dbg(port->dev, "%s(%d)\n", __func__, port->line);
// 
// 	sci_stop_rx(port);
// 	sci_stop_tx(port);
// 
// 	sci_free_dma(port);
// 	sci_free_irq(s);
// 
// 	sci_port_disable(s);
// }


static void queue_shutdown(struct uart_port *port)
{
        struct completion done;

        init_completion(&done);
        to_modem_queue(CMD_MODEM_SHUTDOWN , &done);
        wait_for_completion(&done);
}

static unsigned int sci_scbrr_calc(unsigned int algo_id, unsigned int bps,
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

static void sci_reset(struct uart_port *port)
{
	unsigned int status;

	do {
		status = sci_in(port, SH_SCI_SCxSR);
	} while (!(status & SCxSR_TEND(port)));

        sci_out(port, SH_SCI_SCSCR, 0x00);	/* TE=0, RE=0, CKE1=0 */

	if (port->type != PORT_SCI)
		sci_out(port, SH_SCI_SCFCR, SCFCR_RFRST | SCFCR_TFRST);
}

#define SCBRR_VALUE(bps, clk) ((clk+16*bps)/(32*bps)-1)

static void sci_set_termios(struct uart_port *port, struct ktermios *termios,
			    struct ktermios *old)
{
	struct sci_port *s = to_sci_port(port);
	unsigned int baud, smr_val, max_baud;
	int t = -1;
	
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
		t = sci_scbrr_calc(s->cfg->scbrr_algo_id, baud, port->uartclk);

	sci_port_enable(s);

	sci_reset(port);

	smr_val = sci_in(port, SH_SCI_SCSMR) & 3;

	if ((termios->c_cflag & CSIZE) == CS7)
		smr_val |= 0x40;
	if (termios->c_cflag & PARENB)
		smr_val |= 0x20;
	if (termios->c_cflag & PARODD)
		smr_val |= 0x30;
	if (termios->c_cflag & CSTOPB)
		smr_val |= 0x08;

	uart_update_timeout(port, termios->c_cflag, baud);

	sci_out(port, SH_SCI_SCSMR, smr_val);

	dev_dbg(port->dev, "%s: SMR %x, t %x, SH_SCI_SCSCR %x\n", __func__, smr_val, t,
		s->cfg->scscr);

	if (t > 0) {
                switch (baud) {
                    case 9600:
                            wait_slice_ktime = &(wait_slice_ktimes[IDX_9600]);
                            onechar_ktime = &(onechar_ktimes[IDX_9600]);
                            break;
                    case 19200:
                            wait_slice_ktime = &(wait_slice_ktimes[IDX_19200]);
                            onechar_ktime = &(onechar_ktimes[IDX_19200]);
                            break;
                    case 38400:
                            wait_slice_ktime = &(wait_slice_ktimes[IDX_38400]);
                            onechar_ktime = &(onechar_ktimes[IDX_38400]);
                            break;
                    case 57600:
                            wait_slice_ktime = &(wait_slice_ktimes[IDX_57600]);
                            onechar_ktime = &(onechar_ktimes[IDX_57600]);
                            break;
                    case 115200:
                            wait_slice_ktime = &(wait_slice_ktimes[IDX_115200]);
                            onechar_ktime = &(onechar_ktimes[IDX_115200]);
                            break;
                    default:
                            printk(KERN_ERR "Invalid baud rate for muxer (%d)\n", baud);
                            wait_slice_ktime = &(wait_slice_ktimes[IDX_DEFAULT]);
                            onechar_ktime = &(onechar_ktimes[IDX_DEFAULT]);
                            break;
                }
		if (t >= 256) {
			sci_out(port, SH_SCI_SCSMR, (sci_in(port, SH_SCI_SCSMR) & ~3) | 1);
			t >>= 2;
		} else
			sci_out(port, SH_SCI_SCSMR, sci_in(port, SH_SCI_SCSMR) & ~3);

		sci_out(port, SH_SCI_SCBRR, t);
		udelay((1000000+(baud-1)) / baud); /* Wait one bit interval */
	}

	sci_init_pins(port, termios->c_cflag);
#ifdef CONFIG_CPU_SUBTYPE_SH7727
#warning "SH7727 does not support SCFCR register. (S.Minke)"
#else
        sci_out(port, SH_SCI_SCFCR, scfcr | ((termios->c_cflag & CRTSCTS) ? SH_SCI_SCFCR_MCE : 0));
#endif        

	sci_out(port, SH_SCI_SCSCR, s->cfg->scscr);

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
		sci_start_rx(port);

//	sci_port_disable(s);
}

static const char *sci_type(struct uart_port *port)
{
	switch (port->type) {
	case PORT_IRDA:
		return "irda";
	case PORT_SCI:
		return "sci";
	case PORT_SCIF:
		return "scif";
	case PORT_SCIFA:
		return "scifa";
	case PORT_SCIFB:
		return "scifb";
	}

	return NULL;
}

static inline unsigned long sci_port_size(struct uart_port *port)
{
	/*
	 * Pick an arbitrary size that encapsulates all of the base
	 * registers by default. This can be optimized later, or derived
	 * from platform resource data at such a time that ports begin to
	 * behave more erratically.
	 */
	//return 64;
	return 14;
}

static int sci_remap_port(struct uart_port *port)
{
	unsigned long size = sci_port_size(port);
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

static void sci_release_port(struct uart_port *port)
{
	if (port->flags & UPF_IOREMAP) {
		iounmap(port->membase);
		port->membase = NULL;
	}

	release_mem_region(port->mapbase, sci_port_size(port));
}

static int sci_request_port(struct uart_port *port)
{
	unsigned long size = sci_port_size(port);
	struct resource *res;
	int ret;

        res = request_mem_region(port->mapbase, size, dev_name(port->dev));
	if (unlikely(res == NULL)){
	    return -EBUSY;
        }

	ret = sci_remap_port(port);
	if (unlikely(ret != 0)) {
		release_resource(res);
		return ret;
	}

	return 0;
}

static void sci_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		struct sci_port *sport = to_sci_port(port);

		port->type = sport->cfg->type;
		sci_request_port(port);
	}
}

static int sci_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct sci_port *s = to_sci_port(port);

	if (ser->irq != s->cfg->irqs[SCIx_TXI_IRQ] || ser->irq > nr_irqs)
		return -EINVAL;
	if (ser->baud_base < 2400)
		/* No paper tape reader for Mitch.. */
		return -EINVAL;

	return 0;
}

static struct uart_ops sci_uart_ops = {
	.tx_empty	= modem_tx_empty, //sci_tx_empty,
	.set_mctrl	= sci_set_mctrl,
	.get_mctrl	= sci_get_mctrl,
	.start_tx	= queue_start_tx, // sci_start_tx,
	.stop_tx	= queue_stop_tx, // sci_stop_tx,
	.stop_rx	= queue_stop_rx, // sci_stop_rx,
	.enable_ms	= sci_enable_ms,
	.break_ctl	= sci_break_ctl,
	.startup	= queue_startup, // sci_startup,
	.shutdown	= queue_shutdown, //sci_shutdown,
	.set_termios	= queue_set_termios, // sci_set_termios,
	.type		= sci_type,
	.release_port	= sci_release_port,
	.request_port	= sci_request_port,
	.config_port	= sci_config_port,
	.verify_port	= sci_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= sci_poll_get_char,
	.poll_put_char	= sci_poll_put_char,
#endif
        .ioctl =        sci_ioctl,     /* extra for SCI [ttySC0] */
};

static int __devinit sci_init_single(struct platform_device *dev,
				     struct sci_port *sci_port,
				     unsigned int index,
				     struct plat_sci_port *p)
{
	struct uart_port *port = &sci_port->port;
	int ret;

	port->ops	= &sci_uart_ops;
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
		ret = sci_probe_regmap(p);
		if (unlikely(ret))
			return ret;
	}

	if (dev) {
		sci_port->iclk = clk_get(&dev->dev, "sci_ick");
		if (IS_ERR(sci_port->iclk)) {
			sci_port->iclk = clk_get(&dev->dev, "peripheral_clk");
			if (IS_ERR(sci_port->iclk)) {
				dev_err(&dev->dev, "can't get iclk\n");
				return PTR_ERR(sci_port->iclk);
			}
		}

		/*
		 * The function clock is optional, ignore it if we can't
		 * find it.
		 */
		sci_port->fclk = clk_get(&dev->dev, "sci_fck");
		if (IS_ERR(sci_port->fclk))
			sci_port->fclk = NULL;

		port->dev = &dev->dev;

		pm_runtime_irq_safe(&dev->dev);
		pm_runtime_enable(&dev->dev);
	}

	sci_port->break_timer.data = (unsigned long)sci_port;
	sci_port->break_timer.function = sci_break_timer;
	init_timer(&sci_port->break_timer);

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

	sci_port->cfg		= p;

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

	port->serial_in		= sci_serial_in;
	port->serial_out	= sci_serial_out;

	if (p->dma_slave_tx > 0 && p->dma_slave_rx > 0)
		dev_dbg(port->dev, "DMA tx %d, rx %d\n",
			p->dma_slave_tx, p->dma_slave_rx);
                
        sdmodem_share.port = port;        

	return 0;
}

#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
static void serial_console_putchar(struct uart_port *port, int ch)
{
	sci_poll_put_char(port, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 */
static void serial_console_write(struct console *co, const char *s,
				 unsigned count)
{
	struct sci_port *sci_port = &sci_ports[co->index];
	struct uart_port *port = &sci_port->port;
	unsigned short bits;

	sci_port_enable(sci_port);

	uart_console_write(port, s, count, serial_console_putchar);

	/* wait until fifo is empty and last bit has been transmitted */
	bits = SCxSR_TDxE(port) | SCxSR_TEND(port);
	while ((sci_in(port, SH_SCI_SCxSR) & bits) != bits)
		cpu_relax();

	sci_port_disable(sci_port);
}

static int __devinit serial_console_setup(struct console *co, char *options)
{
	struct sci_port *sci_port;
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

	sci_port = &sci_ports[co->index];
	port = &sci_port->port;

	/*
	 * Refuse to handle uninitialized ports.
	 */
	if (!port->ops)
		return -ENODEV;

	ret = sci_remap_port(port);
	if (unlikely(ret != 0))
		return ret;

	sci_port_enable(sci_port);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	sci_port_disable(sci_port);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console serial_console = {
	.name		= "ttySC",
	.device		= uart_console_device,
	.write		= serial_console_write,
	.setup		= serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &sci_uart_driver,
};

static struct console early_serial_console = {
	.name           = "early_ttySC",
	.write          = serial_console_write,
	.flags          = CON_PRINTBUFFER,
	.index		= -1,
};

static char early_serial_buf[32];

static int __devinit sci_probe_earlyprintk(struct platform_device *pdev)
{
	struct plat_sci_port *cfg = pdev->dev.platform_data;

	if (early_serial_console.data)
		return -EEXIST;

	early_serial_console.index = pdev->id;

	sci_init_single(NULL, &sci_ports[pdev->id], pdev->id, cfg);

	serial_console_setup(&early_serial_console, early_serial_buf);

	if (!strstr(early_serial_buf, "keep"))
		early_serial_console.flags |= CON_BOOT;

	register_console(&early_serial_console);
	return 0;
}

#define uart_console(port)	((port)->cons->index == (port)->line)

static int sci_runtime_suspend(struct device *dev)
{
	struct sci_port *sci_port = dev_get_drvdata(dev);
	struct uart_port *port = &sci_port->port;

	if (uart_console(port)) {
		sci_port->saved_smr = sci_in(port, SH_SCI_SCSMR);
		sci_port->saved_brr = sci_in(port, SH_SCI_SCBRR);
		sci_port->saved_fcr = sci_in(port, SH_SCI_SCFCR);
	}
	return 0;
}

static int sci_runtime_resume(struct device *dev)
{
	struct sci_port *sci_port = dev_get_drvdata(dev);
	struct uart_port *port = &sci_port->port;

	if (uart_console(port)) {
		sci_reset(port);
		sci_out(port, SH_SCI_SCSMR, sci_port->saved_smr);
		sci_out(port, SH_SCI_SCBRR, sci_port->saved_brr);
		sci_out(port, SH_SCI_SCFCR, sci_port->saved_fcr);
		sci_out(port, SH_SCI_SCSCR, sci_port->cfg->scscr);
	}
	return 0;
}

#define SCI_CONSOLE	(&serial_console)

#else
static inline int __devinit sci_probe_earlyprintk(struct platform_device *pdev)
{
	return -EINVAL;
}

#define SCI_CONSOLE	NULL
#define sci_runtime_suspend	NULL
#define sci_runtime_resume	NULL

#endif /* CONFIG_SERIAL_SH_SCI_CONSOLE */

static char banner[] __initdata =
	KERN_INFO "SuperH SCI(F) MUX driver initialized\n";

static struct uart_driver sci_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "sci-mux",
	.dev_name	= "ttySC",
	.major		= SCI_MAJOR,
	.minor		= SCI_MINOR_START,
	.nr		= SCI_NPORTS,
	.cons		= SCI_CONSOLE,
};

static int sci_remove(struct platform_device *dev)
{
	struct sci_port *port = platform_get_drvdata(dev);

	cpufreq_unregister_notifier(&port->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);

	uart_remove_one_port(&sci_uart_driver, &port->port);

	clk_put(port->iclk);
	clk_put(port->fclk);

	pm_runtime_disable(&dev->dev);
	return 0;
}

static int __devinit sci_probe_single(struct platform_device *dev,
				      unsigned int index,
				      struct plat_sci_port *p,
				      struct sci_port *sciport)
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

	ret = sci_init_single(dev, sciport, index, p);
	if (ret)
		return ret;
        ret = sci_request_irq(sciport);//SM
	return uart_add_one_port(&sci_uart_driver, &sciport->port);
}

static int __devinit sci_probe(struct platform_device *dev)
{
	struct plat_sci_port *p = dev->dev.platform_data;
	struct sci_port *sp = &sci_ports[dev->id];
	int ret;

	/*
	 * If we've come here via earlyprintk initialization, head off to
	 * the special early probe. We don't have sufficient device state
	 * to make it beyond this yet.
	 */
	if (is_early_platform_device(dev))
		return sci_probe_earlyprintk(dev);

	platform_set_drvdata(dev, sp);

	ret = sci_probe_single(dev, dev->id, p, sp);
	if (ret)
		goto err_unreg;

	sp->freq_transition.notifier_call = sci_notifier;

	ret = cpufreq_register_notifier(&sp->freq_transition,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (unlikely(ret < 0))
		goto err_unreg;

#ifdef CONFIG_SH_STANDARD_BIOS
	sh_bios_gdb_detach();
#endif

	return 0;

err_unreg:
	sci_remove(dev);
	return ret;
}

static int sci_suspend(struct device *dev)
{
	struct sci_port *sport = dev_get_drvdata(dev);

	if (sport)
		uart_suspend_port(&sci_uart_driver, &sport->port);

	return 0;
}

static int sci_resume(struct device *dev)
{
	struct sci_port *sport = dev_get_drvdata(dev);

	if (sport)
		uart_resume_port(&sci_uart_driver, &sport->port);

	return 0;
}



static void sci_set_stopped_hw(void)
{
        ctrl_outb(ctrl_inb(SCSR) & ~(SCSCR_RE | SCSCR_TE), SCSR);      /* disable transmitter and receiver */
        ctrl_outb(ctrl_inb(SCSR) & ~(SCSCR_TIE | SCSCR_TEIE | SCSCR_RIE), SCSR);  /* disable interrupts */
        ctrl_outb(ctrl_inb(SCSR) & ~(SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSR);  /* clear flags*/
}

// static struct plat_sci_port pseudo_cfg ={
//     .type = PORT_SCI,
//     .irqs = {23, 24, 25, 26},
// };

// static struct sci_port sd_pseudoport = {
//         .cfg = &pseudo_cfg,
// };


static const struct dev_pm_ops sci_dev_pm_ops = {
	.runtime_suspend = sci_runtime_suspend,
	.runtime_resume = sci_runtime_resume,
	.suspend	= sci_suspend,
	.resume		= sci_resume,
};

static struct platform_driver sci_driver = {
	.probe		= sci_probe,
	.remove		= sci_remove,
	.driver		= {
		.name	= "sh-sci-mux",
		.owner	= THIS_MODULE,
		.pm	= &sci_dev_pm_ops,
	},
};

static int __init sci_init(void)
{
	int ret;

	printk(banner);

	ret = uart_register_driver(&sci_uart_driver);
	if (likely(ret == 0)) {
		ret = platform_driver_register(&sci_driver);
		if (unlikely(ret))
			uart_unregister_driver(&sci_uart_driver);
	}
#ifdef USE_MULTIPLEXER	
        sdmodem_share.state = sdmodem_state_initializing;
        printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
	//kthread_run(sdmodem_muxhandler, (void*)&sdmodem_share, "sdmodemd");
#endif

	return ret;
}

static void __exit sci_exit(void)
{
	platform_driver_unregister(&sci_driver);
	uart_unregister_driver(&sci_uart_driver);
}







#ifdef CONFIG_SERIAL_SH_SCI_CONSOLE
early_platform_init_buffer("earlyprintk", &sci_driver,
			   early_serial_buf, ARRAY_SIZE(early_serial_buf));
#endif
module_init(sci_init);
module_exit(sci_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sh-sci");
MODULE_AUTHOR("Paul Mundt");
MODULE_DESCRIPTION("SuperH SCI(F) serial driver");
