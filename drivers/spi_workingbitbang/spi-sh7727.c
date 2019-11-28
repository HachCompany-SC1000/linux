/*
 * SH SPI bus driver
 *
 * Copyright (C) 2011  Renesas Solutions Corp.
 *
 * Based on pxa2xx_spi.c:
 * Copyright (C) 2005 Stephen Street / StreetFire Sound Labs
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/spi/spi.h>

#define LOW 1
#define HIGH 0
#define PORTKDR 0xA4000132
#define SCPCR   0xA4000116
#define CS(t) (t? \
    writeb( (readb(PORTKDR)&~(1<<2)),PORTKDR) :\
    writeb( (readb(PORTKDR)|(1<<2)), PORTKDR) )
//#define CS(t) (t?0:1);



//#define BASE_SCS                0xFFFFFE80
#define SCSMR                   0xFFFFFE80
#define SCBRR                   0xFFFFFE82
#define SCSCR                   0xFFFFFE84
#define SCTDR                   0xFFFFFE86
#define SCSSR                   0xFFFFFE88
#define SCRDR                   0xFFFFFE8A
#define SCSCMR                  0xFFFFFE8C

#define SCSSR_TDRE              0x80
#define SCSSR_RDRF              0x40
#define SCSSR_ORER              0x20 

#define SCSSR_FER               0x10
#define SCSSR_PER               0x08
#define SCSSR_TEND              0x04    

#define SCSCMR_CA               0x80

#define SCSCR_TIE               0x80
#define SCSCR_RIE               0x40
#define SCSCR_TE                0x20
#define SCSCR_RE                0x10

#define SCSCR_TEIE              0x04
#define SCI_CTRL_FLAGS_RIE      0x40 /* all */
#define SCI_CTRL_FLAGS_TIE      0x80
#define SCI_CTRL_FLAGS_TEIE     0x04

#define SPI_SH_TBR		0x00
#define SPI_SH_RBR		0x00
#define SPI_SH_CR1		0x08
#define SPI_SH_CR2		0x10
#define SPI_SH_CR3		0x18
#define SPI_SH_CR4		0x20
#define SPI_SH_CR5		0x28

/* CR1 */
#define SPI_SH_TBE		0x80
#define SPI_SH_TBF		0x40
#define SPI_SH_RBE		0x20
#define SPI_SH_RBF		0x10
#define SPI_SH_PFONRD		0x08
#define SPI_SH_SSDB		0x04
#define SPI_SH_SSD		0x02
#define SPI_SH_SSA		0x01

/* CR2 */
#define SPI_SH_RSTF		0x80
#define SPI_SH_LOOPBK		0x40
#define SPI_SH_CPOL		0x20
#define SPI_SH_CPHA		0x10
#define SPI_SH_L1M0		0x08

/* CR3 */
#define SPI_SH_MAX_BYTE		0xFF

/* CR4 */
#define SPI_SH_TBEI		0x80
#define SPI_SH_TBFI		0x40
#define SPI_SH_RBEI		0x20
#define SPI_SH_RBFI		0x10
#define SPI_SH_WPABRT		0x04
#define SPI_SH_SSS		0x01

/* CR8 */
#define SPI_SH_P1L0		0x80
#define SPI_SH_PP1L0		0x40
#define SPI_SH_MUXI		0x20
#define SPI_SH_MUXIRQ		0x10

#define SPI_SH_FIFO_SIZE	1/*32*/
#define SPI_SH_SEND_TIMEOUT	(3 * HZ)
#define SPI_SH_RECEIVE_TIMEOUT	(HZ >> 3)

#undef DEBUG



#define TRACE_SD_CHARS  (2)  // trace this number of sd data characters

#define  REGISTER_ERROR(sdIrqData, val) { \
    sdIrqData.error |= (val); \
    if (!sdIrqData.errorCnt) {sdIrqData.errorPos =  sdIrqData.done; printk(KERN_ERR "%s - ERR 0x%02x\n", __FUNCTION__, (unsigned int)val); } \
    sdIrqData.errorCnt++; }
    
extern const u8 byte_rev_table[256];

static int log = 1;
struct spi_sh_data {
	void __iomem *addr;
	int irq;
	struct spi_master *master;
	struct list_head queue;
	struct workqueue_struct *workqueue;
	struct work_struct ws;
	unsigned long cr1;
	wait_queue_head_t wait;
	spinlock_t lock;
};







/* used by sd card r/w irq handlers */
typedef struct {
        volatile int          isSdIrq;    // flag to choose irq handling method for sd
        int                   command;    // CMD_SD_READ/CMD_SD_WRITE/..
        int                   source;     // block layer / command
        struct completion    *complete;   // used for command blocking
        struct request_queue *queue;      // used for block layer blocking
        unsigned char        *data;       // r/w data
        unsigned int          size;       // number of bytes to read/write
        volatile unsigned int done;       // number of bytes already read/written
        int                   error;      // still unused
        unsigned int          errorCnt;   // still unused
        unsigned int          errorPos;   // still unused
        volatile int          finished;   // flag last irq has been requested
        struct completion    *irqComplete;// used for blocking of the "scmodemd" thread

} sdIrqData_t;
static sdIrqData_t sdIrqData;







static void sd_transmit_chars(struct uart_port *port)
{
        /* Note: port == NULL */
        unsigned char c;
        unsigned char status;
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
        /* finished ? */
        if (sdIrqData.done >= sdIrqData.size) {
                //TRACE(0xa1);
                ctrl_outb(ctrl_inb(SCSCR)&~(SCI_CTRL_FLAGS_TIE), SCSCR);
                ctrl_outb(ctrl_inb(SCSSR)&~(SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSSR);

                return;
        }


        c = *(sdIrqData.data+sdIrqData.done);
        sdIrqData.done++;

        // print first TRACE_SD_CHARS chars or last TRACE_SD_CHARS chars
        if (sdIrqData.done<=TRACE_SD_CHARS) {
                //TRACE(0xa2);
                //TRACE(c);
        }
        else if (sdIrqData.size-sdIrqData.done<TRACE_SD_CHARS) {
                //TRACE(0xac);
                //TRACE(c);
        }

        status = ctrl_inb(SCSSR);
        if (!(status & SCSSR_TDRE)) {
                REGISTER_ERROR(sdIrqData, 0x01);

                return;
        }
        if (status & (SCSSR_ORER | SCSSR_FER | SCSSR_PER)) {
                REGISTER_ERROR(sdIrqData, 0x02);
        }

        ctrl_outb(byte_rev_table[c], SCTDR);
        /* Clear the transmit register empty flag. */

        ctrl_outb(ctrl_inb(SCSSR) & ~(SCSSR_ORER | SCSSR_FER | SCSSR_PER | SCSSR_TDRE), SCSSR);

        return;
}


static void sd_handle_transmit_end(struct uart_port *port)
{
        /* Note: port == NULL */
        unsigned char status;
        status = ctrl_inb(SCSSR);
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
        if (!(status & SCSSR_TEND)) {
                //TRACE(0xa3);
                REGISTER_ERROR(sdIrqData, 0x04);
                return;
        }

        /* finished ? */
        if (sdIrqData.done >= sdIrqData.size) {
                //TRACE(0xa4);
                ctrl_outb(ctrl_inb(SCSCR)&~(SCI_CTRL_FLAGS_TEIE), SCSCR);
                ctrl_outb(ctrl_inb(SCSSR)&~(SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSSR);

                sdIrqData.isSdIrq = 0;
                sdIrqData.finished = 1;

                complete(sdIrqData.irqComplete);

                return;
        }
        //TRACE(0xa5);

        return;
}


static void sd_receive_chars(struct uart_port *port)
{
        /* Note: port == NULL */
        unsigned char c;
        unsigned char status;

        status = ctrl_inb(SCSSR);
        if (!(status & SCSSR_RDRF)) {
                //TRACE(0xa6);
                REGISTER_ERROR(sdIrqData, 0x10);

                return;
        }
        if (status & (SCSSR_ORER | SCSSR_FER | SCSSR_PER)) {
                //TRACE(0xa7);
                REGISTER_ERROR(sdIrqData, 0x20);
        }

        c = byte_rev_table[ctrl_inb(SCRDR)];

        /* Clear the "receive data ready" flag */
        ctrl_outb(ctrl_inb(SCSSR) & ~SCSSR_RDRF, SCSSR);

        *(sdIrqData.data+sdIrqData.done) = c;
        sdIrqData.done++;

        // print first TRACE_SD_CHARS chars or last TRACE_SD_CHARS chars
        if (sdIrqData.done<=TRACE_SD_CHARS) {
                //TRACE(0xa9);
                //TRACE(c);
        }
        else if (sdIrqData.size-sdIrqData.done<TRACE_SD_CHARS) {
                //TRACE(c);
        }

        /* finished ? */
        if (sdIrqData.done >= sdIrqData.size) {
                //TRACE(0xa8);
                ctrl_outb(ctrl_inb(SCSCR)&~(SCSCR_RE|SCI_CTRL_FLAGS_RIE), SCSCR);
                ctrl_outb(ctrl_inb(SCSSR) & ~(SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSSR);

                sdIrqData.isSdIrq = 0;
                sdIrqData.finished = 1;

                complete(sdIrqData.irqComplete);
                return;
        }

        ctrl_outb(0xff, SCTDR);

        /* Clear the transmit register empty flag. */
        ctrl_outb(ctrl_inb(SCSSR) & ~SCSSR_TDRE, SCSSR);

        return;
}































int sd_start_irq(int is_read, unsigned char *data, unsigned int len)
{

        struct completion done;
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
        sdIrqData.data = data;
        sdIrqData.size = len;
        sdIrqData.done = 0;



        sdIrqData.isSdIrq = 1;
        sdIrqData.finished = 0;

        init_completion(&done);
        sdIrqData.irqComplete = &done;


        /* enable interrupts */
        ctrl_outb(ctrl_inb(SCSSR) & ~(SCSSR_RDRF | SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSSR);

        if (is_read) {
            printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
                ctrl_outb(ctrl_inb(SCSCR) &~(SCSCR_RIE | SCSCR_TIE | SCSCR_TEIE), SCSCR);
                ctrl_outb(ctrl_inb(SCSCR) &~(SCSCR_RE | SCSCR_TE), SCSCR);

                ctrl_outb((SCSCR_RE | SCSCR_TE), SCSCR);

                ctrl_outb(ctrl_inb(SCSCR) | SCI_CTRL_FLAGS_RIE, SCSCR);

                /* send 1st chararcter to kick reading */
                ctrl_outb(0xff, SCTDR);
                /* Clear the transmit register empty flag. */
                ctrl_outb(ctrl_inb(SCSSR) & ~SCSSR_TDRE, SCSSR);

        }
        else { /* !is_read */
            printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
             //   ctrl_outb(ctrl_inb(SCSCR) &~(SCSCR_RE | SCSCR_TE), SCSCR);
//                ctrl_outb(ctrl_inb(SCSCR) &~(SCSCR_RIE | SCSCR_TIE | SCSCR_TEIE), SCSCR);
                

//                ctrl_outb(ctrl_inb(SCSCR) | (SCI_CTRL_FLAGS_TIE | SCI_CTRL_FLAGS_TEIE), SCSCR);
//                ctrl_outb((SCSCR_TE), SCSCR);
            ctrl_outw( (ctrl_inw(SCPCR) & (~0xFF)),SCPCR);
                ctrl_outb(0xA5,  SCTDR);
        }


        /* wait for end of irq actions */
        wait_for_completion(&done);


        return 0;
}



static irqreturn_t sci_rx_interrupt(int irq, void *ptr)
{
    printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
//         struct uart_port *port = ptr;
// 
//         if (port->type == PORT_SCIF) {
//                 scif_receive_chars(port);
//         }
//        else if (sdIrqData.isSdIrq) {
                sd_receive_chars(NULL);
//        }
//         else if (sdmodem_share.modem_is_opened) {
//                 sci_receive_chars(port);
//         }
//         else {
//                 TRACE(0xb8);
//         }
        return IRQ_HANDLED;
}

static irqreturn_t sci_tx_interrupt(int irq, void *ptr)
{
    printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
//         struct uart_port *port = ptr;
// 
//         if (port->type == PORT_SCIF) {
//                 spin_lock_irq(&port->lock);
//                 scif_transmit_chars(port);
//                 spin_unlock_irq(&port->lock);
//         }
//         else if (sdIrqData.isSdIrq) {
                sd_transmit_chars(NULL);
//         }
//         else if (sdmodem_share.modem_is_opened) {
//                 spin_lock_irq(&port->lock);
//                 sci_transmit_chars(port);
//                 spin_unlock_irq(&port->lock);
//         }
//         else {
//                 TRACE(0xb9);
//         }
        return IRQ_HANDLED;
}








































static void spi_sh_write(struct spi_sh_data *ss, unsigned long data,
			     unsigned long offset)
{
 printk(KERN_DEBUG"%s() data=0x%lx address=0x%lx\n",__FUNCTION__,data, (unsigned long)(ss->addr + offset));
    writel(data, ss->addr + offset);
}
static void spi_sh_writeb(struct spi_sh_data *ss, unsigned long data,
                             unsigned long offset)
{
//      if (log) printk(KERN_DEBUG"%s() data=0x%lx address=0x%lx\n",__FUNCTION__,data, (unsigned long)(ss->addr + offset));
    ctrl_outb(data, (long unsigned int)(ss->addr + offset) );
}

static unsigned long spi_sh_read(struct spi_sh_data *ss, unsigned long offset)
{
    unsigned long data;
    data = readl(ss->addr + offset);
printk(KERN_DEBUG"%s() data=0x%lx address=0x%lx\n",__FUNCTION__,data, (unsigned long)(ss->addr + offset));
    //return readl(ss->addr + offset);
        return data;
}

static void spi_sh_set_bit(struct spi_sh_data *ss, unsigned long val,
				unsigned long offset)
{
	unsigned long tmp;
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
	tmp = spi_sh_read(ss, offset);
	tmp |= val;
	spi_sh_write(ss, tmp, offset);
}

static void spi_sh_clear_bit(struct spi_sh_data *ss, unsigned long val,
				unsigned long offset)
{
	unsigned long tmp;
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
	tmp = spi_sh_read(ss, offset);
	tmp &= ~val;
	spi_sh_write(ss, tmp, offset);
}

static void clear_fifo(struct spi_sh_data *ss)
{
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);	
        spi_sh_set_bit(ss, SPI_SH_RSTF, SPI_SH_CR2);
	spi_sh_clear_bit(ss, SPI_SH_RSTF, SPI_SH_CR2);
}

static int spi_sh_wait_receive_buffer(struct spi_sh_data *ss)
{
	int timeout = 100000;
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
	while (spi_sh_read(ss, SPI_SH_CR1) & SPI_SH_RBE) {
		udelay(10);
		if (timeout-- < 0)
			return -ETIMEDOUT;
	}
	return 0;
}

static int spi_sh_wait_write_buffer_empty(struct spi_sh_data *ss)
{
	int timeout = 100000;
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
	while (!(spi_sh_read(ss, SPI_SH_CR1) & SPI_SH_TBE)) {
		udelay(10);
		if (timeout-- < 0)
			return -ETIMEDOUT;
	}
	return 0;
}

static int spi_sh_send(struct spi_sh_data *ss, struct spi_message *mesg,
			struct spi_transfer *t)
{
 	int i, retval = 0;
// 	int remain = t->len;
// 	int cur_len;
 	unsigned char *data;
// 	unsigned long tmp;
// 	long ret;
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
        data = (unsigned char *)t->tx_buf;
        sd_start_irq(0/*is_read*/, data, t->len);

// #ifdef ORIGINAL
//  if (t->len)
// 		spi_sh_set_bit(ss, SPI_SH_SSA, SPI_SH_CR1); // ??? chip select ???
// #else
// //        printk(KERN_DEBUG"portk=0x%x\n",ctrl_inb(PORTKDR));
//                 if (t->len)
//                 CS(LOW); // ??? chip select ???
// #endif
// 	data = (unsigned char *)t->tx_buf;
// 	while (remain > 0) {
// 		cur_len = min(SPI_SH_FIFO_SIZE, remain);
// 		for (i = 0; i < cur_len &&
// 				!(spi_sh_read(ss, SPI_SH_CR4) &
// 							SPI_SH_WPABRT) &&
// 				!(spi_sh_read(ss, SPI_SH_CR1) & SPI_SH_TBF);
// 				i++)
// #ifdef ORIGINAL
//                     spi_sh_write(ss, (unsigned long)data[i], SPI_SH_TBR);
// #else
//                         printk(KERN_DEBUG"SCTDR=0x%x\n",data[i]);
//                         spi_sh_writeb(ss, (unsigned long)data[i], SCTDR);
// #endif                        
// 
// 		if (spi_sh_read(ss, SPI_SH_CR4) & SPI_SH_WPABRT) {
// 			/* Abort SPI operation */
//                         printk(KERN_DEBUG"%s():%d unsupported\n",__FUNCTION__,__LINE__);
// //			spi_sh_set_bit(ss, SPI_SH_WPABRT, SPI_SH_CR4);
// 			retval = -EIO;
// 			break;
// 		}
// 
// 		cur_len = i;
// 
// 		remain -= cur_len;
// 		data += cur_len;
// 
// 		if (remain > 0) {
// 			ss->cr1 &= ~SPI_SH_TBE;
// //			spi_sh_set_bit(ss, SPI_SH_TBE, SPI_SH_CR4);
//                         printk(KERN_DEBUG"%s():%d unsupported\n",__FUNCTION__,__LINE__);
// 			ret = wait_event_interruptible_timeout(ss->wait,
// 						 ss->cr1 & SPI_SH_TBE,
// 						 SPI_SH_SEND_TIMEOUT);
// 			if (ret == 0 && !(ss->cr1 & SPI_SH_TBE)) {
// 				printk(KERN_ERR "%s: timeout\n", __func__);
// 				return -ETIMEDOUT;
// 			}
// 		}
// 	}
// 
// 	if (list_is_last(&t->transfer_list, &mesg->transfers)) {
// 		tmp = spi_sh_read(ss, SPI_SH_CR1);
// 		tmp = tmp & ~(SPI_SH_SSD | SPI_SH_SSDB);
// //		spi_sh_write(ss, tmp, SPI_SH_CR1);
// //		spi_sh_set_bit(ss, SPI_SH_SSA, SPI_SH_CR1);
//                 CS(LOW);
// 
// 		ss->cr1 &= ~SPI_SH_TBE;
// 		spi_sh_set_bit(ss, SPI_SH_TBE, SPI_SH_CR4);
// 		ret = wait_event_interruptible_timeout(ss->wait,
// 					 ss->cr1 & SPI_SH_TBE,
// 					 SPI_SH_SEND_TIMEOUT);
// 		if (ret == 0 && (ss->cr1 & SPI_SH_TBE)) {
// 			printk(KERN_ERR "%s: timeout\n", __func__);
// 			return -ETIMEDOUT;
// 		}
// 	}
// 
 	return retval;
}

static int spi_sh_receive(struct spi_sh_data *ss, struct spi_message *mesg,
			  struct spi_transfer *t)
{
// 	int i;
// 	int remain = t->len;
// 	int cur_len;
 	unsigned char *data;
// 	unsigned long tmp;
// 	long ret;
  printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
        data = (unsigned char *)t->rx_buf;
        sd_start_irq(1/*is_read*/, data, t->len);
// 	if (t->len > SPI_SH_MAX_BYTE)
// 		spi_sh_write(ss, SPI_SH_MAX_BYTE, SPI_SH_CR3);
// 	else
// 		spi_sh_write(ss, t->len, SPI_SH_CR3);
// 
// //	tmp = spi_sh_read(ss, SPI_SH_CR1);
// //	tmp = tmp & ~(SPI_SH_SSD | SPI_SH_SSDB);
// //	spi_sh_write(ss, tmp, SPI_SH_CR1);
// //	spi_sh_set_bit(ss, SPI_SH_SSA, SPI_SH_CR1);
//         CS(LOW);
// 
// 	spi_sh_wait_write_buffer_empty(ss);
// 
// 	data = (unsigned char *)t->rx_buf;
// 	while (remain > 0) {
// 		if (remain >= SPI_SH_FIFO_SIZE) {
// 			ss->cr1 &= ~SPI_SH_RBF;
// 			spi_sh_set_bit(ss, SPI_SH_RBF, SPI_SH_CR4);
// 			ret = wait_event_interruptible_timeout(ss->wait,
// 						 ss->cr1 & SPI_SH_RBF,
// 						 SPI_SH_RECEIVE_TIMEOUT);
// 			if (ret == 0 &&
// 			    spi_sh_read(ss, SPI_SH_CR1) & SPI_SH_RBE) {
// 				printk(KERN_ERR "%s: timeout\n", __func__);
// 				return -ETIMEDOUT;
// 			}
// 		}
// 
// 		cur_len = min(SPI_SH_FIFO_SIZE, remain);
// 		for (i = 0; i < cur_len; i++) {
// 			if (spi_sh_wait_receive_buffer(ss))
// 				break;
// 			data[i] = (unsigned char)spi_sh_read(ss, SPI_SH_RBR);
// 		}
// 
// 		remain -= cur_len;
// 		data += cur_len;
// 	}
// 
// 	/* deassert CS when SPI is receiving. */
// 	if (t->len > SPI_SH_MAX_BYTE) {
// 		clear_fifo(ss);
// 		spi_sh_write(ss, 1, SPI_SH_CR3);
// 	} else {
// 		spi_sh_write(ss, 0, SPI_SH_CR3);
// 	}
// 
 	return 0;
}

static void spi_sh_work(struct work_struct *work)
{
	struct spi_sh_data *ss = container_of(work, struct spi_sh_data, ws);
	struct spi_message *mesg;
	struct spi_transfer *t;
	unsigned long flags;
	int ret;
    printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
	pr_debug("%s: enter\n", __func__);

	spin_lock_irqsave(&ss->lock, flags);
	while (!list_empty(&ss->queue)) {
		mesg = list_entry(ss->queue.next, struct spi_message, queue);
		list_del_init(&mesg->queue);

		spin_unlock_irqrestore(&ss->lock, flags);
		list_for_each_entry(t, &mesg->transfers, transfer_list) {
			pr_debug("tx_buf = %p, rx_buf = %p\n",
					t->tx_buf, t->rx_buf);
			pr_debug("len = %d, delay_usecs = %d\n",
					t->len, t->delay_usecs);

			if (t->tx_buf) {
				ret = spi_sh_send(ss, mesg, t);
				if (ret < 0)
					goto error;
			}
			if (t->rx_buf) {
				ret = spi_sh_receive(ss, mesg, t);
				if (ret < 0)
					goto error;
			}
			mesg->actual_length += t->len;
		}
		spin_lock_irqsave(&ss->lock, flags);

		mesg->status = 0;
		mesg->complete(mesg->context);
	}

	clear_fifo(ss);
	spi_sh_set_bit(ss, SPI_SH_SSD, SPI_SH_CR1);
	udelay(100);

//	spi_sh_clear_bit(ss, SPI_SH_SSA | SPI_SH_SSDB | SPI_SH_SSD,
//			 SPI_SH_CR1);

	clear_fifo(ss);

	spin_unlock_irqrestore(&ss->lock, flags);

	return;

 error:
	mesg->status = ret;
	mesg->complete(mesg->context);

	spi_sh_clear_bit(ss, SPI_SH_SSA | SPI_SH_SSDB | SPI_SH_SSD,
			 SPI_SH_CR1);
        CS(HIGH);
	clear_fifo(ss);

}










































static int spi_sh_setup(struct spi_device *spi)
{
	struct spi_sh_data *ss = spi_master_get_devdata(spi->master);
        int i,ret,irq;
        
        irq = ss->irq;
    printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
#ifdef ORIGINAL
    if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	pr_debug("%s: enter\n", __func__);

	spi_sh_write(ss, 0xfe, SPI_SH_CR1);	/* SPI sycle stop */
	spi_sh_write(ss, 0x00, SPI_SH_CR1);	/* CR1 init */
	spi_sh_write(ss, 0x00, SPI_SH_CR3);	/* CR3 init */

	clear_fifo(ss);

	/* 1/8 clock */
	spi_sh_write(ss, spi_sh_read(ss, SPI_SH_CR2) | 0x07, SPI_SH_CR2);
	udelay(10);
#else
    
        ctrl_outw( (ctrl_inw(SCPCR) & (~0x0F)),SCPCR);
        ctrl_outb(0x00,SCSCR);
        ctrl_outb(SCSCMR_CA,SCSMR);
        ctrl_outb(0x20,SCBRR); 
        udelay(20);
        ctrl_outb( (SCSCR_RE|SCSCR_TE), SCSCR);
        
//#define TEST_SPI
#ifdef TEST_SPI
        {/* test spi */ 
                int i;
                volatile unsigned char status;
                for (i=0;i<10;i++)
                {
                    do {} while ((status = ctrl_inb( SCSSR ) & SCSSR_TDRE)!=SCSSR_TDRE);
                    ctrl_outb(i,  SCTDR);  
                    status &= ~SCSSR_TDRE;
                    ctrl_outb(status, SCSSR);
                }
        }  
#endif        
#endif
        log = 0;

        
	return 0;
        
        
error3:
    for (i=i-1 ;i>=0;i--)
           free_irq(irq+i,ss);
        
}

static int spi_sh_transfer(struct spi_device *spi, struct spi_message *mesg)
{
	struct spi_sh_data *ss = spi_master_get_devdata(spi->master);
	unsigned long flags;
        printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
	pr_debug("%s: enter\n", __func__);
	pr_debug("\tmode = %02x\n", spi->mode);

	spin_lock_irqsave(&ss->lock, flags);

	mesg->actual_length = 0;
	mesg->status = -EINPROGRESS;

//	spi_sh_clear_bit(ss, SPI_SH_SSA, SPI_SH_CR1);


	list_add_tail(&mesg->queue, &ss->queue);
	queue_work(ss->workqueue, &ss->ws);

	spin_unlock_irqrestore(&ss->lock, flags);

	return 0;
}

static void spi_sh_cleanup(struct spi_device *spi)
{
	struct spi_sh_data *ss = spi_master_get_devdata(spi->master);
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
	pr_debug("%s: enter\n", __func__);

	//spi_sh_clear_bit(ss, SPI_SH_SSA | SPI_SH_SSDB | SPI_SH_SSD,
//			 SPI_SH_CR1);
        CS(HIGH);
}



static int __devexit spi_sh_remove(struct platform_device *pdev)
{
	struct spi_sh_data *ss = dev_get_drvdata(&pdev->dev);

	spi_unregister_master(ss->master);
	destroy_workqueue(ss->workqueue);
	free_irq(ss->irq, ss);
	iounmap(ss->addr);

	return 0;
}




























static irqreturn_t spi_sh_irq(int irq, void *_ss)
{
        struct spi_sh_data *ss = (struct spi_sh_data *)_ss;
        unsigned long cr1;
        irqreturn_t ret = IRQ_NONE;
printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
        //cr1 = spi_sh_read(ss, SCSSR);
        cr1 = readb(SCSSR);
        if (cr1 & SCSSR_TDRE)
               ret = sci_tx_interrupt(irq,ss);
        //if (cr1 & SPI_SH_TBF)
        //        ss->cr1 |= SPI_SH_TBF;
        //if (cr1 & SPI_SH_RBE)
        //        ss->cr1 |= SPI_SH_RBE;
        if (cr1 & SCSSR_RDRF)
               ret = sci_rx_interrupt(irq,ss);

        if (ss->cr1) {
               // spi_sh_clear_bit(ss, ss->cr1, SPI_SH_CR4);
                wake_up(&ss->wait);
        }

        return ret;
}





enum {
        SCIx_ERI_IRQ = 0,
        SCIx_RXI_IRQ,
        SCIx_TXI_IRQ,
        SCIx_BRI_IRQ,
        SCIx_NR_IRQS,
};


static struct sci_irq_desc {
        const char      *desc;
        irq_handler_t   handler;
} sci_irq_desc[] = {
        /*
         * Split out handlers, the default case.
         */
        [SCIx_ERI_IRQ] = {
                .desc = "rx_err",
                .handler = spi_sh_irq,
        },

        [SCIx_RXI_IRQ] = {
                .desc = "rx_full",
                .handler = spi_sh_irq,
        },

        [SCIx_TXI_IRQ] = {
                .desc = "tx_empty",
                .handler = spi_sh_irq,
        },

        [SCIx_BRI_IRQ] = {
                .desc = "break",
                .handler = spi_sh_irq,
        },

//         /*
//          * Special muxed handler.
//          */
//         [SCIx_MUX_IRQ] = {
//                 .desc = "mux",
//                 .handler = spi_sh_irq,
//         },
};











static int __devinit spi_sh_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct spi_master *master;
	struct spi_sh_data *ss;
	int ret, irq, i;
    printk(KERN_DEBUG"%s() %d\n",__FUNCTION__,__LINE__);
	/* get base addr */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(res == NULL)) {
		dev_err(&pdev->dev, "invalid resource\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq error\n");
		return -ENODEV;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_sh_data));
	if (master == NULL) {
		dev_err(&pdev->dev, "spi_alloc_master error.\n");
		return -ENOMEM;
	}

	ss = spi_master_get_devdata(master);
	dev_set_drvdata(&pdev->dev, ss);

	ss->irq = irq;
	ss->master = master;
	ss->addr = ioremap(res->start, resource_size(res));
	if (ss->addr == NULL) {
		dev_err(&pdev->dev, "ioremap error.\n");
		ret = -ENOMEM;
		goto error1;
	}
	INIT_LIST_HEAD(&ss->queue);
	spin_lock_init(&ss->lock);
	INIT_WORK(&ss->ws, spi_sh_work);
	init_waitqueue_head(&ss->wait);
	ss->workqueue = create_singlethread_workqueue(
					dev_name(master->dev.parent));
	if (ss->workqueue == NULL) {
		dev_err(&pdev->dev, "create workqueue error\n");
		ret = -EBUSY;
		goto error2;
	}

//	for ( i=0; i<4; i++){
    i=0;
            ret = request_irq(irq+i, sci_irq_desc[i].handler /*spi_sh_irq */, 0, sci_irq_desc[i].desc /*"spi_sh"*/, ss);
//            if (ret < 0) {
//		dev_err(&pdev->dev, "request_irq error whith irq=%d\n",irq+i);
//		goto error3;
//            }
//        }

	master->num_chipselect = 2;
	master->bus_num = pdev->id;
	master->setup = spi_sh_setup;
	master->transfer = spi_sh_transfer;
	master->cleanup = spi_sh_cleanup;

	ret = spi_register_master(master);
	if (ret < 0) {
		printk(KERN_ERR "spi_register_master error.\n");
		goto error4;
	}

	return 0;

 error4:
       for ( i=3 ;i>=0;i--)
           free_irq(irq+i, ss);
 error3:
       for (i=i-1 ;i>=0;i--)
           free_irq(irq+i,ss);
	destroy_workqueue(ss->workqueue);
 error2:
	iounmap(ss->addr);
 error1:
	spi_master_put(master);

	return ret;
}

static struct platform_driver spi_sh_driver = {
	.probe = spi_sh_probe,
	.remove = __devexit_p(spi_sh_remove),
	.driver = {
		.name = "sh_spi",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(spi_sh_driver);

MODULE_DESCRIPTION("SH SPI bus driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yoshihiro Shimoda");
MODULE_ALIAS("platform:sh_spi");
