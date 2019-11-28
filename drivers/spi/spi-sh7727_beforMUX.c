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
#include <asm/sc1000.h>

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
#include <linux/bitrev.h>


#define LOW 0
#define HIGH 1



#define CS(t) (t? \
    writeb( (readb(PORT_PKDR)|(1<<2)), PORT_PKDR): \
    writeb( (readb(PORT_PKDR)&~(1<<2)),PORT_PKDR) ) 


#define SCI_ERI_IRQ 23
#define SCI_RXI_IRQ 24
#define SCI_TXI_IRQ 25
#define SCI_TEI_IRQ 26


#define SET_PIN(port,pin,state) do{ \
    uint8_t val=ctrl_inb(PORT_P ## port ## DR); \
    if(state){ \
        ctrl_outb(val|(1<<(pin)), PORT_P ## port ## DR); \
    } else { \
        ctrl_outb(val&~(1<<(pin)), PORT_P ## port ## DR); \
    } \
} while(0)

#define GET_PIN(port,pin) (ctrl_inb(PORT_P ## port ## DR)>>pin&1)

#undef DEBUG


#define TRACE_SD_CHARS  (2)  // trace this number of sd data characters

#define  REGISTER_ERROR(sdIrqData, val) { \
    sdIrqData.error |= (val); \
    if (!sdIrqData.errorCnt) {sdIrqData.errorPos =  sdIrqData.done; printk(KERN_ERR "%s - ERR 0x%02x\n", __FUNCTION__, (unsigned int)val); } \
    sdIrqData.errorCnt++; }
    
extern const u8 byte_rev_table[256];

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
        unsigned char        *tx_data;    // w data
        unsigned char        *rx_data;    // r data
        unsigned int          size;       // number of bytes to read/write
        volatile unsigned int done;       // number of bytes already read/written
        int                   error;      // still unused
        unsigned int          errorCnt;   // still unused
        unsigned int          errorPos;   // still unused
        volatile int          finished;   // flag last irq has been requested
        struct completion    *irqComplete;// used for blocking of the "scmodemd" thread

} sdIrqData_t;
static sdIrqData_t sdIrqData;



#define DBG_TGL(u) {int i;for(i=0;i<u;i++){SET_PIN(E,0,HIGH);SET_PIN(E,0,LOW);}udelay(2);}

static irqreturn_t (*cd_handler)(int, void *);
static struct mmc_host         *_mmc;

static irqreturn_t _spi_detect_irq(int irq, void *mmc)
{
    unsigned short icr2;
    int ret = IRQ_NONE;
    icr2 = ctrl_inw(INTC_ICR2);

    if (icr2 & 0x0800){
        /* high level --> undetect */
        ret = cd_handler( irq, _mmc);
        ctrl_outw(icr2 & ((unsigned short)~(1<<11)), INTC_ICR2);
    }
    else{
        /* low level --> detect */
        ret = cd_handler( irq, _mmc);
        ctrl_outw(icr2 | ((unsigned short)(1<<11)) , INTC_ICR2);
    }
    
    return ret;
}


int spi_sh7727_init(struct device *dev,
                irqreturn_t (*handler)(int, void *),
                void *mmc)
{
    int ret=0;
    
    ret = request_irq(41, _spi_detect_irq, 0, "mmc_cd", mmc);
    
    if (ret<0)
        return ret;
    
    if (GET_PIN(F,3)){
        /* card not in slot */
        ctrl_outw(ctrl_inw(INTC_ICR2) & ((unsigned short)~(1<<11)), INTC_ICR2);
    }
    else{
        /* card in slot */
        ctrl_outw(ctrl_inw(INTC_ICR2) | ((unsigned short)(1<<11)) , INTC_ICR2);
    }
    
    ctrl_outw (ctrl_inw(INTC_PINTER) | (1<<11), INTC_PINTER);
    
    cd_handler = handler;
    _mmc=mmc;
    
 
    return ret;
}

int spi_sh7727_getcd(struct device *dev)
{
    int ret;
    ret = !GET_PIN(F,3);
    
 
    return ret;
}



/* combined character send an transmit routine */

static void sd_receive_chars()
{
        /* Note: port == NULL */
        unsigned char c;
        unsigned char status;

        status = ctrl_inb(SCSR);
        if (!(status & SCSSR_RDRF)) {
                REGISTER_ERROR(sdIrqData, 0x10);

                return;
        }
        if (status & (SCSSR_ORER | SCSSR_FER | SCSSR_PER)) {
                REGISTER_ERROR(sdIrqData, 0x20);
        }

        c = byte_rev_table[ctrl_inb(SCRDR)];

        /* Clear the "receive data ready" flag */
        ctrl_outb(ctrl_inb(SCSR) & ~SCSSR_RDRF, SCSR);

        /* capture received character only if necessary */
        if (sdIrqData.rx_data)
            *(sdIrqData.rx_data+sdIrqData.done) = c;
        
        sdIrqData.done++;

        /* finished ? */
        if (sdIrqData.done >= sdIrqData.size) {
                ctrl_outb(ctrl_inb(SCSCR)&~(SCSCR_RE|SCSCR_TE|SCSCR_RIE), SCSCR);
                ctrl_outb(ctrl_inb(SCSR) & ~(SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSR);

                sdIrqData.isSdIrq = 0;
                sdIrqData.finished = 1;

                complete(sdIrqData.irqComplete);
                /* Clear the transmit register empty flag. */
                ctrl_outb(ctrl_inb(SCSR) & ~SCSSR_TDRE, SCSR);
                return;
        }

        /* send next character */
        if (sdIrqData.tx_data)
            ctrl_outb(byte_rev_table[*(sdIrqData.tx_data+sdIrqData.done)], SCTDR);
        else
            ctrl_outb(0xFF, SCTDR);

        /* Clear the transmit register empty flag. */
        ctrl_outb(ctrl_inb(SCSR) & ~SCSSR_TDRE, SCSR);

        return;
}




int sd_start_rxtx_irq(unsigned char *data, unsigned int len, unsigned char *rx_data)
{
        struct completion done;

        sdIrqData.tx_data = data;
        sdIrqData.rx_data = rx_data;
        sdIrqData.size = len;

        sdIrqData.done = 0;

        sdIrqData.isSdIrq = 1;
        sdIrqData.finished = 0;

        init_completion(&done);
        sdIrqData.irqComplete = &done;

        /* enable interrupts */
        CS(LOW);  
        ctrl_outb(ctrl_inb(SCSR) & ~(SCSSR_RDRF | SCSSR_ORER | SCSSR_FER | SCSSR_PER), SCSR); 

        ctrl_outb(ctrl_inb(SCSCR) &~(SCSCR_RIE | SCSCR_TIE | SCSCR_TEIE), SCSCR);
        ctrl_outb(ctrl_inb(SCSCR) &~(SCSCR_RE | SCSCR_TE), SCSCR);

        ctrl_outb((SCSCR_RE | SCSCR_TE), SCSCR);

        ctrl_outb(ctrl_inb(SCSCR) | SCSCR_RIE , SCSCR);

        /* send 1st chararcter to kick reading */
        /* don' t increment sdIrqData.done, that will be done on RDRF */
        if (sdIrqData.tx_data)
            ctrl_outb(byte_rev_table[*(sdIrqData.tx_data+sdIrqData.done)], SCTDR);
        else
            ctrl_outb(0xFF, SCTDR);
        /* Clear the transmit register empty flag. */
        ctrl_outb(ctrl_inb(SCSR) & ~SCSSR_TDRE, SCSR);


        /* wait for end of irq actions */
        wait_for_completion(&done);

        return 0;
}


static irqreturn_t sci_rx_interrupt(int irq, void *ptr)
{
  //  printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
//         struct uart_port *port = ptr;
// 
//         if (port->type == PORT_SCIF) {
//                 scif_receive_chars(port);
//         }
//        else if (sdIrqData.isSdIrq) {
       sd_receive_chars(/*NULL*/);
//        }
//         else if (sdmodem_share.modem_is_opened) {
//                 sci_receive_chars(port);
//         }
//         else {
//                 TRACE(0xb8);
//         }
        return IRQ_HANDLED;
}

//static irqreturn_t sci_tx_interrupt(int irq, void *ptr)
//{
//printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
//         struct uart_port *port = ptr;
// 
//         if (port->type == PORT_SCIF) {
//                 spin_lock_irq(&port->lock);
//                 scif_transmit_chars(port);
//                 spin_unlock_irq(&port->lock);
//         }
//         else if (sdIrqData.isSdIrq) {
//         }
//         else if (sdmodem_share.modem_is_opened) {
//                 spin_lock_irq(&port->lock);
//                 sci_transmit_chars(port);
//                 spin_unlock_irq(&port->lock);
//         }
//         else {
//                 TRACE(0xb9);
//         }
//        return IRQ_HANDLED;
//}



//static irqreturn_t sci_txend_interrupt(int irq, void *ptr)
//{
//printk(KERN_DEBUG"%s():%d\n",__FUNCTION__,__LINE__);
//         struct uart_port *port = ptr;
// 
//         if (port->type == PORT_SCIF) {
//                 spin_lock_irq(&port->lock);
//                 scif_transmit_chars(port);
//                 spin_unlock_irq(&port->lock);
//         }
//         else if (sdIrqData.isSdIrq) {
//                sd_handle_transmit_end(NULL);
//         }
//         else if (sdmodem_share.modem_is_opened) {
//                 spin_lock_irq(&port->lock);
//                 sci_transmit_chars(port);
//                 spin_unlock_irq(&port->lock);
//         }
//         else {
//                 TRACE(0xb9);
//         }
//        return IRQ_HANDLED;
//}


static int spi_sh_send(struct spi_sh_data *ss, struct spi_message *mesg,
			struct spi_transfer *t)
{
 	int retval = 0;
 	unsigned char *tx_data,*rx_data;

        tx_data =    (unsigned char *)(t->tx_buf);
        rx_data = (unsigned char *)(t->rx_buf);
    
        sd_start_rxtx_irq(tx_data, t->len,rx_data);
 
 	return retval;
}

static void spi_sh_work(struct work_struct *work)
{
	struct spi_sh_data *ss = container_of(work, struct spi_sh_data, ws);
	struct spi_message *mesg;
	struct spi_transfer *t;
	unsigned long flags;
	int ret;
        
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
			mesg->actual_length += t->len;
		}
		spin_lock_irqsave(&ss->lock, flags);

		mesg->status = 0;
		mesg->complete(mesg->context);
	}

	udelay(100);

        spin_unlock_irqrestore(&ss->lock, flags);

	return;

 error:
        pr_debug("%s: error\n", __func__);
	mesg->status = ret;
	mesg->complete(mesg->context);
}


static int spi_sh_setup(struct spi_device *spi)
{
	struct spi_sh_data *ss = spi_master_get_devdata(spi->master);
        int irq;
        
        irq = ss->irq;
        ctrl_outw( (ctrl_inw(SCPCR) & (~0x0F)),SCPCR);
        ctrl_outb(0x00,SCSCR);
        ctrl_outb(SCSMR_CA,SCSMR);
        ctrl_outb(0x00,SCBBR); 
        udelay(20);
        ctrl_outb( (SCSCR_RE|SCSCR_TE), SCSCR);
        
//#define TEST_SPI
#ifdef TEST_SPI
        {/* test spi */ 
                int i;
                volatile unsigned char status;
                for (i=0;i<10;i++)
                {
                    do {} while ((status = ctrl_inb( SCSR ) & SCSSR_TDRE)!=SCSSR_TDRE);
                    ctrl_outb(i,  SCTDR);  
                    status &= ~SCSSR_TDRE;
                    ctrl_outb(status, SCSR);
                }
        }  
#endif        
       
        CS(HIGH);
	return 0;

}

static int spi_sh_transfer(struct spi_device *spi, struct spi_message *mesg)
{
	struct spi_sh_data *ss = spi_master_get_devdata(spi->master);
	unsigned long flags;
	pr_debug("%s: enter\n", __func__);
	pr_debug("\tmode = %02x\n", spi->mode);
	spin_lock_irqsave(&ss->lock, flags);

	mesg->actual_length = 0;
	mesg->status = -EINPROGRESS;

        list_add_tail(&mesg->queue, &ss->queue);
	queue_work(ss->workqueue, &ss->ws);

	spin_unlock_irqrestore(&ss->lock, flags);

	return 0;
}

static void spi_sh_cleanup(struct spi_device *spi)
{
	//struct spi_sh_data *ss = spi_master_get_devdata(spi->master);
	pr_debug("%s: enter\n", __func__);
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
        cr1 = readb(SCSR); 

       
        switch (irq){
            case SCI_ERI_IRQ:
                ctrl_outb(cr1&~(SCSSR_FER | SCSSR_ORER | SCSSR_PER),SCSR); 
                ret = IRQ_HANDLED;
                break;
            case SCI_RXI_IRQ:
                ret = sci_rx_interrupt(irq,ss);
                ctrl_outb(cr1&~SCSSR_RDRF,SCSR); 
                break;
            case SCI_TXI_IRQ:
                ctrl_outb(cr1&~SCSSR_TDRE,SCSR);
                ret = IRQ_HANDLED;
                break;
            case SCI_TEI_IRQ:
                ctrl_outb(cr1&~SCSSR_TEND,SCSR); 
                ret= IRQ_HANDLED;
                break;
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
                .desc = "spi_sh7727_ERI",
                .handler = spi_sh_irq,
        },

        [SCIx_RXI_IRQ] = {
                .desc = "spi_sh7727_RXI",
                .handler = spi_sh_irq,
        },

        [SCIx_TXI_IRQ] = {
                .desc = "spi_sh7727_TXI",
                .handler = spi_sh_irq,
        },

        [SCIx_BRI_IRQ] = {
                .desc = "spi_sh7727_TEI",
                .handler = spi_sh_irq,
        },
};



static int __devinit spi_sh_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct spi_master *master;
	struct spi_sh_data *ss;
	int ret, irq, i;

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

	for ( i=0; i<4; i++){
            ret = request_irq(irq+i, sci_irq_desc[i].handler /*spi_sh_irq */, 0, sci_irq_desc[i].desc /*"spi_sh"*/, ss);
            if (ret < 0) {
		dev_err(&pdev->dev, "request_irq error whith irq=%d\n",irq+i);
		goto error3;
            }
        }
        
        
        
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
           free_irq(irq, ss);
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
