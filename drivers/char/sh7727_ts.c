/*
 *  linux/drivers/block/sh7727_ts.c
 *
 *  Copyright (c) 2001 Lineo Japan, Inc.
 *
 *  May be copied or modified under the terms of the GNU General Public
 *  License.  See linux/COPYING for more information.
 *
 *  SH7727 touch screen drvier.
 *
 *  last changes:
 *	Rainer, 13.8.03
 *	Rainer, 14.8.04 (use calibr. of calcIntersectioncoordinates)
 *	jsp, 20.09.2007 Screen ON/OFF from touch, added backlight_mode_timer
 *      jsp, 12.11.2007 remove udelay(200) from irq handlers + use NUM_CYCLES ad conversions instead
 *      jsp, 13.11.2007 remove SA_INTERRUPT
 *      TODO: remove float/double arithmetics (jsp)
 */

//#include <linux/autoconf.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sh7727_ts.h>

#include <asm/sc1000.h>
#include <asm/uaccess.h>

#include <asm/ioctls.h>
#include "floating_point.h"
#include <linux/kobject.h>
#include <linux/cdev.h>


#define LOW 0
#define HIGH 1

//static int debug=0;

#define SET_PIN(port,pin,state) do{ \
    uint8_t val=ctrl_inb(PORT_P ## port ## DR); \
    if(state){ \
        ctrl_outb(val|(1<<(pin)), PORT_P ## port ## DR); \
    } else { \
        ctrl_outb(val&~(1<<(pin)), PORT_P ## port ## DR); \
    } \
} while(0)
#define DBG_TGL(u) if(debug){int i;for(i=0;i<u;i++){SET_PIN(E,0,HIGH);SET_PIN(E,0,LOW);}udelay(2);}




/* -------------------------------------------------- */
#define USE_MMCDRV_AS_MONITOR 0   // setting this to 1 sets MMCDRV while waiting for voltage settle
#if (USE_MMCDRV_AS_MONITOR)
#warning using MMCDRV as monitor
#endif

#if USE_MMCDRV_AS_MONITOR
static unsigned char mmcdrv_toggle = 0;
#endif
/* -------------------------------------------------- */

typedef int (*adc_callback_t)(int, void*);
#define MAX_ADC_CHANNELS 7
#define ADC_AN1 0x01
#define ADC_AN2 0x02
#define ADC_AN3 0x03
#define ADC_AN4 0x04
#define ADC_AN5 0x05
#define ADC_AN6 0x06
#define ADC_AN7 0x07

#define ADC_TPY ADC_AN2
#define ADC_TPX ADC_AN3

extern int32_t sc1000_mac[];

/**********************************************************************/
/* Taken from sh7727_adc.c */
/**********************************************************************/

spinlock_t my_lock;

static int ts_state = 0;

#define BEEP_ON  1
#define BEEP_OFF 0
const char info[]="echo 1 > /sys/beeper/beep to switch beeper on\n"\
                     "echo 0 > /sys/beeper/beep to switch beeper off\0";
static unsigned char bBeep=1;
static int beepcnt = 0;


static adc_callback_t sh7727_adc_callback_handler[MAX_ADC_CHANNELS+1];

static irqreturn_t sh7727_adc_interrupt(int irq, void* ptr)
{
	int ch;
	ch = (ctrl_inb(ADCSR) & 0x07);
	if (sh7727_adc_callback_handler[ch]) {
		//printk(KERN_INFO "%s irq=%d ch=%d ts_state=%d\n", __FUNCTION__, irq, ch, ts_state);
		(*sh7727_adc_callback_handler[ch])(irq, ptr);
	}
	else {
		//printk(KERN_INFO "%s [irq=%d ch=%d ts_state=%d]\n", __FUNCTION__, irq, ch, ts_state);
	}
	return IRQ_HANDLED;
}

void sh7727_adc_set_callback(unsigned int ch,
			     int (*handler)(int, void*))
{
	printk(KERN_INFO "sh7727_adc_set_callback() - ch %u\n", ch);
	if (ch > MAX_ADC_CHANNELS)
		return;
	sh7727_adc_callback_handler[ch] = handler;
}

int sh7727_adc_start(unsigned int ch)
{
 	int ret = 0;

	ctrl_outb(ADCSR_ADIE|ADCSR_ADST|ch, ADCSR);

	return ret;
}

void sh7727_adc_stop(void)
{
	ctrl_outb(0, ADCSR);
}

unsigned int sh7727_adc_read(unsigned int ch)
{
	unsigned int value = 0;

        switch (ch) {
	case ADC_AN4:
		return ((((unsigned int)ctrl_inb(ADDRAH)) << 2) +
			(((unsigned int)ctrl_inb(ADDRAL)) >> 6));
	case ADC_AN5:
		return ((((unsigned int)ctrl_inb(ADDRBH)) << 2) +
			(((unsigned int)ctrl_inb(ADDRBL)) >> 6));
	case ADC_AN2:
	case ADC_AN6:
		return ((((unsigned int)ctrl_inb(ADDRCH)) << 2) +
			(((unsigned int)ctrl_inb(ADDRCL)) >> 6));
	case ADC_AN3:
	case ADC_AN7:
		return ((((unsigned int)ctrl_inb(ADDRDH)) << 2) +
			(((unsigned int)ctrl_inb(ADDRDL)) >> 6));
	}
	return value;
}

int __init sh7727_adc_init(void)
{
	int i,ret;
   
	printk(KERN_INFO "sh7727_adc_init()\n");

	for (i = 0; i <= MAX_ADC_CHANNELS; i++) {
		sh7727_adc_callback_handler[i] = NULL;
	}
	ctrl_outb(ADCR_TRGE1|ADCR_TRGE0, ADCR);

	ret = request_irq(ADC_IRQ, sh7727_adc_interrupt,
			IRQF_NOBALANCING/*SA_INTERRUPT*/, "adc", NULL);
	if(ret){
		printk(KERN_ERR "%s: failed to register ADC_IRQ %d\n",
			__FUNCTION__,ADC_IRQ);
		return ret;
	}
	//ctrl_outb(0, ADCR);

	return 0;
}

/**********************************************************************/

#define TS_NAME "ts"
#define TS_MAJOR 11

#define BUFSIZE 128

#ifdef CONFIG_PM
static struct pm_dev* sh7727_ts_pm_dev;
#endif


#ifdef CONFIG_PM
#include <linux/pm.h>
static struct pm_dev* sh7727_ts_pm_dev;
#endif


// Port Control for beeper
#define REG_BEEP_SW	SCPDR								// beep on/off
#define PIN_INIT_BEEP	ctrl_outw((ctrl_inw(SCPCR) & 0xffcf) | 0x10, SCPCR)		// Bit2 Port SCP
#define PIN_REINIT_BEEP	do{unsigned short w=ctrl_inw(SCPCR); if((w & ~0xffcf)!=0x0010){ctrl_outw((w & 0xffcf)|0x0010, SCPCR);printk("sh7727_ts SCPCR|=0x0010 (found: 0x%04X)\n", w);}}while(0)	// Bit2 Port SCP
// Beep switch
#define BIT_BEEP	(1 << 2)							// position beep
#define PIN_SW_BEEP_OFF	ctrl_outb(ctrl_inb(REG_BEEP_SW) |  BIT_BEEP, REG_BEEP_SW)	// beep on
#define PIN_SW_BEEP_ON	ctrl_outb(ctrl_inb(REG_BEEP_SW) & ~BIT_BEEP, REG_BEEP_SW)	// beep off



// 13.8.03 Rainer
// 10.8.04 Rainer
#define DO_CORRECTION
//#define DO_TRANSFORM


/* From Compaq's Touch Screen Specification version 0.2 (draft) */
typedef struct {
	short pressure;
	short x;
	short y;
	short millisecs;
} TS_EVENT;


#define PRESSED      0
#define X_WAIT       1   /* wait some time before digitizing starts */
#define X_DIGITIZE   2   /* the digitize */
#define Y_WAIT       3
#define Y_DIGITIZE   4
#define UP_WAIT      5
#define RELEASED     6
#define PENDOWN      7


static DECLARE_WAIT_QUEUE_HEAD(queue);
static struct fasync_struct* fasync;
static int raw_max_x, raw_max_y;
static int res_x, res_y;
static int raw_min_x, raw_min_y;
static int cal_ok, x_rev, y_rev, xyswap;
static int volatile head = 0, tail = 0;
static volatile TS_EVENT cur_data, buf[BUFSIZE];


static struct timer_list timer;
static struct fasync_struct *fasync;
static unsigned long in_timehandle = 0;
static int refcount = 0;





/* --------------------------------------------------------------- */
/* backlight_mode_timer                                            */
/* --------------------------------------------------------------- */

int fb_handle_fbioset_lcd( unsigned long arg );  /* see drivers/video/fbmem.c */

#define  PWR_OFF    ((unsigned char)0x80)
static inline void put_backlight_on( void )
{
       fb_handle_fbioset_lcd(0x02);  /* FBIOSETLCD_POWERON */
}

static inline void put_backlight_mode( void )
{
       fb_handle_fbioset_lcd(0x10);  /* FBIOSETLCD_DEFAULT */
}


static struct timer_list backlight_mode_timer;
#define BACKLIGHT_OFF_TIMER_INTERVALL_S  (10*60)

static void schedule_backlight_mode_timer(struct timer_list *timer)
{
        del_timer(timer);
	//printk("schedule_backlight_mode_timer()\n");
	timer->expires = jiffies + BACKLIGHT_OFF_TIMER_INTERVALL_S*HZ;
	add_timer(timer);
}

static void handle_backlight_mode_timer(unsigned long data)
{
	//printk("handle_backlight_mode_timer()\n");
	put_backlight_mode();
}

static void init_backlight_mode_timer(struct timer_list *timer)
{
        timer->data = (unsigned long)timer;
	timer->function = handle_backlight_mode_timer;
	init_timer(timer);
	//printk("init_backlight_mode_timer()\n");
}
/* --------------------------------------------------------------- */

#ifdef DO_CORRECTION
static int use_pq = 1;
static int qflag = 1;
static int pflag = 1;
static double qx, qy, px, py;
// SC1000 Display (nxcal points)
static double x[4] = {200, 236, 696, 733};	// down right, top right, top left, down left
static double y[4] = {169, 712, 611, 147};	// down right, top right, top left, down left
static double x0, y0;				// down right

static void calcIntersectioncoordinates(void)
{
	double a, b, c, d;
        x0 = x[0]; y0 = y[0];

/*
	Q
	y = a*x + b
	y[0] = a*x[0] + b;
	y[1] = a*x[1] + b;
	y[1] - y[0] = a*(x[1] - x[0]);
*/
	/* bad data ? */
	if (y[0] == y[1] || y[2] == y[3]) {
		qflag = 0;
		pflag = 0;
		return;
	}

	if (x[0] != x[1] && x[2] != x[3]) {
		a = (y[1] - y[0]) / (x[1] - x[0]);
		b = y[0] - a * x[0];

/*	y = c*x + d;
	y[2] = c*x[2] + d;
	y[3] = c*x[3] + d;
	y[3] - y[2] = c*(x[3] - x[2])
*/
		c = (y[3] - y[2]) / (x[3] - x[2]);
		d = y[2] - c * x[2];

/*
	a*qx + b = c*qx + d
	qx*(a - c) = (d - b)
*/
		qx = (d - b) / (a - c);
		qy = a*qx + b;
	}
	else if (x[0] == x[1] && x[2] != x[3]) {
		c = (y[3] - y[2]) / (x[3] - x[2]);
		d = y[2] - c * x[2];
		qx = x[0];
		qy = c*qx + d;
	}
	else if (x[0] != x[1] && x[2] == x[3]) {
		a = (y[1] - y[0]) / (x[1] - x[0]);
		b = y[0] - a * x[0];

		qx = x[2];
		qy = a*qx + b;
	}
	else {
		/* Parallel lines */
		qflag = 0;
	}
/*
	P

	y = a*x + b
	y[1] = a*x[1] + b
	y[2] = a*x[2] + b
	y[2] - y[1] = a*(x[2] - x[1])
*/
	/* bad data ? */
	if (x[1] == x[2] || x[3] == x[0]) {
		qflag = 0;
		pflag = 0;
		return;
	}

	if (y[1] != y[2] && y[3] != y[0]) {
		a = (y[2] - y[1]) / (x[2] - x[1]);
		b = y[1] - a * x[1];
/*
	y = c*x + d
	y[3] = c*x[3] + d
	y[0] = c*x[0] + d
	y[0] - y[3] = c*(x[0] - x[3])
*/
		c = (y[0] - y[3]) / (x[0] - x[3]);
		d = y[3] - c * x[3];
/*
	a*px + b = c*px + d
	px*(a - c) = (d - b)
*/
		px = (d - b) / (a - c);
		py = a*px + b;
	}
	else if (y[1] == y[2] && y[3] != y[0]) {
		c = (y[0] - y[3]) / (x[0] - x[3]);
		d = y[3] - c * x[3];

		py = y[1];
		px = (py - d) / c;
	}
	else if (y[1] != y[2] && y[3] == y[0]) {
		a = (y[2] - y[1]) / (x[2] - x[1]);
		b = y[1] - a * x[1];

		py = y[3];
		px = (py - b) / a;
	}
	else {
		/* Parallel lines */
		pflag = 0;
	}

#if 1
	printk(KERN_INFO "calcIntersectioncoordinates()  qx=%d qy=%d  px=%d py=%d\n", (int)qx, (int)qy, (int)px, (int)py);
#endif
}


static int transX( int x, int y)
{
	double a, b;
	double sx=0., sy=0.;
	double mx;
	sx = (double)x; 
        sy = (double)y;
	mx = sx;
	/* X coordinate */
  	if (qflag == 1 && (sx != qx && sy != qy)) {
		a = (qy - sy) / (qx - sx);
		b = sy - a*sx;
		mx = (y0 - b) / a;
	}
	return (short)mx;
}

static int transY( int x, int y)
{
	double a, b;
	double sx, sy;
	double my=1;

	sx = x; sy = y;
	my = sy;
	/* Y coordinate */
	if (pflag == 1 && (sx != px && sy != py)) {
		a = (py - sy) / (px - sx);
		b = sy - a*sx;
		my = a*x0 + b;
	}
	return (int)my;
}
#endif



static void print_TestPoints(void)
{
 int ind;

	printk(" Kernel ==> Testpoints:\n");
	for(ind = 0; ind < 4; ind++)
		printk(" Kernel ==> x[%d]: %d, y[%d]: %d\n", ind, (int)x[ind], ind, (int)y[ind]);
	printk(" Kernel ==> Use Testpoints: %d\n", use_pq);
	printk(" Kernel ==> pflag: %d, qflag: %d\n", pflag, qflag);
}



static void print_par(void)
{
	printk(" Kernel ==> cal_ok = %d\n",cal_ok);
	printk(" Kernel ==> raw_max_x = %d\n",raw_max_x);
	printk(" Kernel ==> raw_max_y = %d\n",raw_max_y);
	printk(" Kernel ==> res_x = %d\n",res_x);
	printk(" Kernel ==> res_y = %d\n",res_y);
	printk(" Kernel ==> raw_min_x = %d\n",raw_min_x);
	printk(" Kernel ==> raw_min_y = %d\n",raw_min_y);
	printk(" Kernel ==> xyswap = %d\n",xyswap);
	printk(" Kernel ==> x_rev = %d\n",x_rev);
	printk(" Kernel ==> y_rev = %d\n",y_rev);
}

static void ts_clear(void)
{
	int i;

	head = tail = 0;
	for (i = 0; i < BUFSIZE; i++) {
		buf[i].pressure = 0;
		buf[i].x = 0;
		buf[i].y = 0;
		buf[i].millisecs = 0;
	}
}



/*
 * ioctl replaced by unlocked_ioctl, if locking is required for the ioctl
 * lock_kernel() and unlock_kernel() can be added at top and end. (SM)
 */
static long /*int*/ sh7727_ts_ioctl(/*struct inode* inode, */struct file* filp, unsigned int cmd, unsigned long arg)
{
	printk(KERN_INFO "sh7727_ts_ioctl()  cmd=%u  arg=%lu\n", cmd, arg);

        switch (cmd) {
	case 3:
	  	raw_max_x = arg;
		break;
	case 4:
	  	raw_max_y = arg;
		break;
	case 5:
	  	res_x = arg;
		break;
	case 6:
	  	res_y = arg;
		break;
	case 10:
	  	raw_min_x = arg;
		break;
	case 11:
	  	raw_min_y = arg;
		break;
	case 12:
		xyswap = arg;
	case 13:
	  	/* 0 = Enable calibration ; 1 = Calibration OK */
	  	cal_ok = arg;
	case 14:
		ts_clear();
		break;
	case 15:
		x_rev = arg;
		break;
	case 16:
	  	y_rev = arg;
		break;
	case 17:
		print_par();
		break;
	// switch on / off correction with intersectioncoordinates
	case 20:
		use_pq = arg;
		if(use_pq)
			calcIntersectioncoordinates();
		break;
	// fill 4 test points in edges (go clockwise from left/top)
	case 21:
		x[2] = arg;
		break;
	case 22:
		y[2] = arg;
		break;
	case 23:
		x[1] = arg;
		break;
	case 24:
		y[1] = arg;
		break;
	case 25:
		x[0] = arg;
		break;
	case 26:
		y[0] = arg;
		break;
	case 27:
		x[3] = arg;
		break;
	case 28:
		y[3] = arg;
		break;
	case 29:
		print_TestPoints();
		break;
	case 30:
                if (bBeep)
                    beepcnt = 1 + arg;
                else
                    printk(KERN_INFO"%s(): beep received, but beeper switched off\n",__FUNCTION__);
		break;
	// get handheld serial#
	case 31:  /* TSIOCTL_GET_SERIAL */
		{
			int			ind;
			unsigned long long	llval = 0;

			if(!__SC1000Magic(SC1000CF_MAGIC_NUM)) {
				printk(KERN_ERR "%s: cant access handheld serial#\n", __FUNCTION__);
				return -EFAULT;
			}
			for (ind = 0; ind < 8; ind++) {
				llval <<= 8;
				llval += ctrl_inb(SC1000CF_BASE + SC1000CF_SER_OFFS + ind);
			}

			if ((access_ok(VERIFY_WRITE, (void *)arg, 8))){
				put_user(llval, (unsigned int *)arg);
				put_user(llval >> 32, (unsigned int *)arg + 1);
			} else {
				printk(KERN_ERR "%s: cant access user space address 0x%08x\n",
					__FUNCTION__,(uint32_t)arg);
				return -EFAULT;
			}
			return 0;
		}
	// get handheld mac#
	case 32:  /* TSIOCTL_GET_MACADDR */
		{
			unsigned long long     llval = 0;
			int i;
			if(!__SC1000Magic(SC1000CF_MAGIC_NUM)) {
				printk(KERN_ERR "%s: cant access handheld mac#\n", __FUNCTION__);
				return -EFAULT;
			}
			for (i=0; i<6; i++)
				llval |= ((unsigned long long)(ctrl_inb(SC1000CF_BASE+SC1000CF_MAC_OFFS+i))) << (8*(5-i));

			if ((access_ok(VERIFY_WRITE, (void *)arg, 8))){
				put_user(llval, (unsigned int *)arg);
				put_user(llval >> 32, (unsigned int *)arg + 1);
			} else {
				printk(KERN_ERR "%s: cant access user space address 0x%08x\n",
					__FUNCTION__,(uint32_t)arg);
				return -EFAULT;
			}
			return 0;
		}
	// get handheld "is eco version" bit
	case 33:  /* TSIOCTL_GET_ECOVERS */
		{
			unsigned long     lval = 0;
			if(!__SC1000Magic(SC1000CF_MAGIC_NUM)) {
				printk(KERN_ERR "%s: cant access handheld eco\n", __FUNCTION__);
				return -EFAULT;
			}
			lval = (unsigned long)((ctrl_inb(SC1000CF_BASE+SC1000CF_XDATA_OFFS) & 0x01));

			if ((access_ok(VERIFY_WRITE, (void *)arg, 4))){
				put_user(lval, (unsigned int *)arg);
			} else {
				printk(KERN_ERR "%s: cant access user space address 0x%08x\n",
				       __FUNCTION__,(uint32_t)arg);
				return -EFAULT;
			}
			return 0;
		}
	}  /* switch (cmd) */


	return 0;
}


static int cycle_cnt = 0;
#define NUM_CYCLES (7) // (25/3) (25)

static void set_read_x_pos(void)
{
#if USE_MMCDRV_AS_MONITOR
		// toggle MMCDRV pin to see irqs on scope:
		// mmcdrv_toggle = mmcdrv_toggle ? 0 : 1<<7;
		mmcdrv_toggle = (1<<7);  // show start
		ctrl_outb( (ctrl_inb(PORT_PJDR) & ~(1<<7)) | mmcdrv_toggle, PORT_PJDR );
#endif
	ts_state = X_WAIT;
	cycle_cnt = NUM_CYCLES;
	ctrl_outb((ctrl_inb(PORT_PJDR)&0xC7)|0x38, PORT_PJDR);
	/*udelay(200);*/
	sh7727_adc_start(ADC_TPX);
	enable_irq(ADC_IRQ);
}

static void set_read_y_pos(void)
{
	ts_state = Y_WAIT;
	cycle_cnt = NUM_CYCLES;
	ctrl_outb((ctrl_inb(PORT_PJDR)&0xC7)|0x08, PORT_PJDR);
	/*udelay(200);*/
	sh7727_adc_start(ADC_TPY);
}



#define AVERAGE 3
static int avg = 0;
static int avx = 0;
static int avy = 0;
static int b_state = 0;
// Rainer
#define MAX_CMP_DIF	10
static int cmp_x = 0;

static void new_data(void)
{
//#ifdef DO_CORRECTION
	//int x=0, y=0;
//#endif
	int cmp_dif_x;

	if (cur_data.pressure) {
		b_state = 1;
		avx += cur_data.x;
		avy += cur_data.y;
		avg++;
		//printk(KERN_INFO "new_data()  x=%d  y=%d  (avg=%d)\n", cur_data.x, cur_data.y, avg );
		if (avg == AVERAGE) {
			cur_data.x = avx / AVERAGE;
			cur_data.y = avy / AVERAGE;
			avx = 0;
			avy = 0;
			avg = 0;
			// check difference, Rainer 27.1.04
			cmp_dif_x = cmp_x - cur_data.x;
			cmp_x = cur_data.x;
			if(abs(cmp_dif_x) > MAX_CMP_DIF) {
				//printk(KERN_ERR "new_data()  cmp_x=%d\n", cmp_x );
				return;
			}
		}
		else
			return;
	}
	else {
		if (b_state == 0)
			return;

		b_state = 0;
		avx = 0;
		avy = 0;
		avg = 0;

		cmp_x = 0;
	}
//#ifdef DO_CORRECTION
// 	if(use_pq) {
// 		x = cur_data.x;
// 		y = cur_data.y;
//  		cur_data.x = transX(x, y);
// 		cur_data.y = transY(x, y);
// 	}
//#endif
/*
	cur_data.y += ((880 - cur_data.x) * (455 - cur_data.y) * 50)
			/ (255 * 800);
	cur_data.x -= 20 * (700 - cur_data.y) * (cur_data.x - 100)
			/ (500 * 770);
	cur_data.y -= 20 * (850 - cur_data.x) / 760;
*/
	cur_data.millisecs = (short)jiffies;
	buf[head] = cur_data;

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#if 0
	printk("P[%d]  +X[%d](%d)  Y[%d](%d)  M[%d]\n",
	    cur_data.pressure, cur_data.x, x, cur_data.y, y, (int)cur_data.millisecs);
#endif

	if (++head >= BUFSIZE) {
		head = 0;
	}
	if ((head == tail) && (++tail >= BUFSIZE)) {
	  	tail = 0;
	}

	if (fasync) {
		kill_fasync(&fasync, SIGIO, POLL_IN);
	}

	wake_up_interruptible(&queue);

}

static TS_EVENT get_data(void)
{
	int last;
        short x,y;
        TS_EVENT tempEvent;
    

#ifdef DO_CORRECTION
        spin_lock_irq(&my_lock);
        last = tail;
        tempEvent = buf[tail];
        spin_unlock_irq(&my_lock);
        if (++tail == BUFSIZE) {
                tail = 0;
        }
        if(use_pq) {  
            x = tempEvent.x;
            y = tempEvent.y;
            tempEvent.x = transX((int)x, (int)y);
            tempEvent.y = transY(x, y);
        }
#endif
	return tempEvent;
}

static int pen_down_first = 1;
static void pen_down_continue(void)
{
	pen_down_first = 0;
	ts_state = PENDOWN;
	disable_irq_nosync(ADC_IRQ);//disable_irq(ADC_IRQ);
	//disable_irq(IRQ0_IRQ);
}

static void wait_for_action(void)
{
	pen_down_first = 1;
	ts_state = PRESSED;
	disable_irq_nosync(ADC_IRQ);//disable_irq(ADC_IRQ);

	/* 00-00-01-01-01-00-00-00 */
	ctrl_outb((ctrl_inb(PORT_PJDR)&0xC7)|0x28, PORT_PJDR);
	enable_irq(IRQ0_IRQ);
}

static ssize_t sh7727_ts_read(struct file* filp, char* buf, size_t count, loff_t* l)
{
	TS_EVENT t, out;
	DECLARE_WAITQUEUE(wait, current);
	int i;

        if (head == tail) {
		if (filp->f_flags & O_NONBLOCK)
		  	return -EAGAIN;
		add_wait_queue(&queue, &wait);
		current->state = TASK_INTERRUPTIBLE;
		while ((head == tail) && !signal_pending(current)) {
		  schedule();
		  current->state = TASK_INTERRUPTIBLE;
		}
		current->state = TASK_RUNNING;
		remove_wait_queue(&queue, &wait);
	}

	for (i = count; i >= sizeof(out);
	     i -= sizeof(out), buf += sizeof(out)) {
		if (head == tail)
		  	break;
		t = get_data();
#if 0
		// !!!
		printk("P[%d] X[%d] Y[%d] M[%d]\n", t.pressure, t.x, t.y, (int)cur_data.millisecs);
#endif
		out.pressure = t.pressure;
#ifdef DO_TRANSFORM
		if (cal_ok) {
		  out.x = (x_rev) ?
		    ((raw_max_x - t.x) * res_x) / (raw_max_x - raw_min_x) :
		    ((t.x - raw_min_x) * res_x) / (raw_max_x - raw_min_x);
		  out.y = (y_rev) ?
		    ((raw_max_y - t.y) * res_y) / (raw_max_y - raw_min_y) :
		    ((t.y - raw_min_y) * res_y) / (raw_max_y - raw_min_y);
		}
		else
#endif
		{
			out.x = t.x;
			out.y = t.y;
		}
		out.millisecs = t.millisecs;
		copy_to_user(buf, &out, sizeof(out));
	}

	return count - i;
}

static unsigned int sh7727_ts_poll(struct file* filp, poll_table* wait)
{
	poll_wait(filp, &queue, wait);
	if (head != tail) {
		return POLLIN | POLLRDNORM;
	}
	return 0;
}

static int sh7727_ts_fasync(int fd, struct file* filp, int on)
{
	int ret = fasync_helper(fd, filp, on, &fasync);
	if (ret < 0) {
		return ret;
	}
 	return 0;
}

/////////////////////////////////////////



static int pen_down = 0;
static int pen_opened = 0;

static void ts_timer(unsigned long);

static int ts_starttimer(void)
{
      in_timehandle++;
      init_timer(&timer);
      timer.function = ts_timer;
      timer.expires = jiffies + HZ / 100;
      add_timer(&timer);
      return 0;
}



#if 0
static unsigned int bt_cancel = 0;
#endif
static void ts_timer(unsigned long data)
{
 	if (pen_opened) {
             if (pen_down && ts_state == PENDOWN) {
			if (pen_down_first) {
				disable_irq_nosync(IRQ0_IRQ);//disable_irq(IRQ0_IRQ);
			}
			set_read_x_pos();
		}
	}

	if(beepcnt) {
		PIN_REINIT_BEEP;
		if(--beepcnt){
			PIN_SW_BEEP_ON;
                }
		else
			PIN_SW_BEEP_OFF;
	}

// SWK 22.10.02
//	/* Jog and Button check */
//	if (!(bt_cancel % 20)) {
//		int vmb = ctrl_inb(PFDR)&0x03;
//		int jog = ctrl_inb(PMDR)&0xe;

//		if (!(jog & 8)) {
//			sh7727fb_set_contrast(1);
//		}
//		else if (!(jog & 4)) {
//			/* push */
//		}
//		else if (!(jog & 2)) {
//			/* down */
//			sh7727fb_set_contrast(-1);
//		}
//
//		if (vmb == 1) {
//			/* bottom */
//		}
//		else if (vmb == 2) {
//			/* middle */
//		}
//
//		if (vh == 0) {
//			/* top */
//		}
//
//		if (!(vmb|vh|jog)) {
//			/* 2 button pressed */
//		}
//	}
//	bt_cancel++;



	ts_starttimer();
}

static void set_ts_timer(void)
{
  	if (refcount++ == 0) {
		in_timehandle = 0;
		ts_starttimer();
	}
}

#ifdef MODULE
static void del_ts_timer(void)
{
 
	if (--refcount == 0) {
		if (in_timehandle)
			del_timer(&timer);
	}
}
#endif

static int sh7727_ts_open(struct inode* inode, struct file* filp)
{
       
       ts_clear();
	pen_opened = 1;
#ifdef MODULE
	MOD_INC_USE_COUNT;
#endif
	return 0;
}

static int sh7727_ts_release(struct inode* inode, struct file* filp)
{
       
	ts_clear();
	sh7727_ts_fasync(-1, filp, 0);
//!!!	pen_opened = 0;
#ifdef MODULE
	MOD_DEC_USE_COUNT;
#endif
	return 0;
}




irqreturn_t sh7727_ts_interrupt(int irq, void *ptr/*, struct pt_regs *regs*/)
{
	int ret_pen_up;
	short adc_value, dummy;
	int status;

	static int bx = 0;
	static int by = 0;
// 	static char a_action[32+1];
// 	static int  c_action=0;
// #define a_printk()     do{a_action[c_action]=0;printk(KERN_INFO "%s() [%s]\n",__FUNCTION__,a_action);c_action=0;}while(0)
// #define if_printk(c)   do{if(c_action<32)a_action[c_action++]=c;else{a_printk();a_action[c_action++]=c;}}while(0)
        
	if (irq == IRQ0_IRQ) {
		pen_down = 1;
		cur_data.pressure = 10;
		ts_state = PENDOWN;

		/* Acknowledge the interrupt */
		// this sequence may delete other irqs (jsp): ctrl_outb(ctrl_inb(INTC_IRR0)&~0x1, INTC_IRR0);
		if (ctrl_inb(INTC_IRR0)&0x1)
			ctrl_outb(0xFE, INTC_IRR0);//ctrl_outb(~0x1, INTC_IRR0);
		//disable_irq_nosync(IRQ0_IRQ);
                //if_printk('T');
		//printk("%s - irq=%d\n", __FUNCTION__, irq);
		return IRQ_HANDLED;
	}

	switch (ts_state) {
	case X_WAIT:
		sh7727_adc_stop();
		dummy = sh7727_adc_read(ADC_TPX);
		if (cycle_cnt > 0) {
			cycle_cnt--;
			sh7727_adc_start(ADC_TPX);
			break;
		}
		ts_state = X_DIGITIZE;
		sh7727_adc_start(ADC_TPX);
		break;
	case X_DIGITIZE:
#if USE_MMCDRV_AS_MONITOR
		// toggle MMCDRV pin to see irqs on scope:
		// mmcdrv_toggle = mmcdrv_toggle ? 0 : 1<<7;
		mmcdrv_toggle = 0;  // show stop
		ctrl_outb( (ctrl_inb(PORT_PJDR) & ~(1<<7)) | mmcdrv_toggle, PORT_PJDR );
#endif
		sh7727_adc_stop();
		adc_value = sh7727_adc_read(ADC_TPX);
		if(xyswap)	//!!! Rainer
			cur_data.x = adc_value;
		else
			cur_data.y = adc_value;
		set_read_y_pos();
		break;
	case Y_WAIT:
		sh7727_adc_stop();
		dummy = sh7727_adc_read(ADC_TPY);
		if (cycle_cnt > 0) {
			cycle_cnt--;
			sh7727_adc_start(ADC_TPY);
			break;
		}
		ts_state = Y_DIGITIZE;
		sh7727_adc_start(ADC_TPY);
		break;
	case Y_DIGITIZE:
		sh7727_adc_stop();
		adc_value =  sh7727_adc_read(ADC_TPY);
		if(xyswap)	//!!! Rainer
			cur_data.y = adc_value;
		else
			cur_data.x = adc_value;

		ctrl_outb((ctrl_inb(PORT_PJDR)&0xC7)|0x28, PORT_PJDR);
		/*udelay(200);*/

		ts_state = UP_WAIT;
		cycle_cnt = NUM_CYCLES;
		sh7727_adc_start(ADC_TPX);  /*ADC_TPY*/
		break;
	case UP_WAIT:
		sh7727_adc_stop();
		dummy = sh7727_adc_read(ADC_TPX);
		if (cycle_cnt > 0) {
			cycle_cnt--;
			sh7727_adc_start(ADC_TPX);
			break;
		}

		if (bx == 0 && by == 0) {
			bx = cur_data.x;
			by = cur_data.y;
		}
		status = ctrl_inb(PORT_PLDR);
		if (!(status & 0x08)) {
			cur_data.x = bx;
			cur_data.y = by;
			ret_pen_up = 1;
		}
		else
			ret_pen_up = 0;

		bx = cur_data.x;
		by = cur_data.y;

		if (ret_pen_up) {
			//printk("sh7727_ts_interrupt::pen_up    X=%d   Y=%d\n", cur_data.x, cur_data.y);

			put_backlight_on(); /* jsp */
			schedule_backlight_mode_timer(&backlight_mode_timer);  /* jsp */

	                //if_printk('w');

			pen_down = 0;
			cur_data.pressure = 0;
			new_data();
			wait_for_action();
		}
		else {
	                //if_printk('c');
			new_data();
			pen_down_continue();
		}
		break;
	}
	return IRQ_HANDLED;
}

static struct file_operations sh7727_ts_fops = {
	read:	        sh7727_ts_read,
	poll:	        sh7727_ts_poll,
	unlocked_ioctl:	sh7727_ts_ioctl,
	fasync:	        sh7727_ts_fasync,
	open:	        sh7727_ts_open,
	release:        sh7727_ts_release,
};

#ifdef CONFIG_PM
static int sh7727_ts_pm_callback(struct pm_dev* pm_dev,
				 pm_request_t req, void* data)
{
   
    switch (req) {
	case PM_SUSPEND:
		sh7727_adc_stop();
	        disable_irq_nosync(ADC_IRQ);//disable_irq(ADC_IRQ);
		disable_irq_nosync(IRQ0_IRC);//disable_irq(IRQ0_IRC);
		break;
	case PM_RESUME:
		head = tail = 0;
		sh7727_adc_stop();
		wait_for_action();
                break;
	}
	return 0;
}
#endif



//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// nur zum Test!
#define SH7727_STBCR		0xffffff82
irqreturn_t sh7727_pwdown_interrupt(int irq, void *ptr/*, struct pt_regs *regs*/)
{
	
    /* Acknowledge the interrupt */
	// this sequence may delete other irqs (jsp): ctrl_outb(ctrl_inb(INTC_IRR0)&~0x8, INTC_IRR0);
	if (ctrl_inb(INTC_IRR0)&0x8)
		ctrl_outb(~0x8, INTC_IRR0);

	printk("*** power down ***\nCPU enters standby mode.\n - good bye!!!\n");
	mdelay(10);				// wait 10 ms
	// Standby Mode
	ctrl_outw(0, INTC_INTER);		// disable all Interrupts
	ctrl_outb(0x80, SH7727_STBCR);		// Standby mode
	asm volatile("sleep" : : : "memory");	// Sleep Instruction
 	return IRQ_HANDLED;

}
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



/*
 * beeper on/off
 */

ssize_t show_attr(struct kobject *kobj, struct attribute *attr, 
                    char *buffer)
{
    sprintf(buffer,"%s\nbeeper: %s\n",info,bBeep?("ON"):("OFF"));
    return 128;
}
ssize_t store_attr(struct kobject *kobj, struct attribute *attr, 
                     const char *buffer, size_t size)
{
    switch (buffer[0]){
        case '1': 
            bBeep=1;
            break;
        case '0':
            bBeep=0;
            break;
        default:
            printk(KERN_INFO"%s(): %s --- unknown command string\n",__FUNCTION__, buffer);
            break;
    }
    return size;
}

static struct sysfs_ops sops ={
    .show = show_attr,
    .store = store_attr,
};

static struct attribute  kAttr = {
    .name = "beep",
    .mode = ( S_IRUGO | S_IWUSR ),
};
static struct kobj_type ktype_default = {
    .sysfs_ops = &sops,
};
static struct kobject kobj;

/*
 * ************************************
 */


int __init sh7727_ts_init(void)
{
	int ret;
        
         
// #warning DISABLE sh7727_ts_init()
// 	return -1;  // test
	ret = register_chrdev(TS_MAJOR, TS_NAME, &sh7727_ts_fops);
	
        memset(&kobj, 0, sizeof(struct kobject));
        kobject_init(&kobj,&ktype_default);
        ret = kobject_add(&kobj,NULL,"beeper");
        if (ret<0)
            printk(KERN_ERR"%s(): could not add kobject\n",__FUNCTION__);
        ret = sysfs_create_file(&kobj, &kAttr);
        if (ret<0)
            printk(KERN_ERR"%s(): could not create file\n",__FUNCTION__);

        
        sh7727_adc_init();


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// nur zum Test!
	if ((ret = request_irq(IRQ3_IRQ, sh7727_pwdown_interrupt,
			       IRQF_NOBALANCING/*SA_INTERRUPT*/, "power down", NULL)))	{
		printk("sh7727_PWDOWN_init: failed to register IRQ3!\n");
		return ret;
	}
	// Init beeper
	PIN_INIT_BEEP;
	PIN_SW_BEEP_OFF;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	if ((ret = request_irq(IRQ0_IRQ, sh7727_ts_interrupt,
			       IRQF_NOBALANCING/*SA_INTERRUPT*/, TS_NAME, NULL)))	{
		printk(KERN_ERR "%s: failed to register IRQ0_IRQ (%d)\n",
				__FUNCTION__,IRQ0_IRQ);
		return ret;
	}

#warning EXTRA disable_irq_nosync()
	disable_irq_nosync(IRQ0_IRQ);

	sh7727_adc_set_callback(ADC_TPX, (void *)sh7727_ts_interrupt);
	sh7727_adc_set_callback(ADC_TPY, (void *)sh7727_ts_interrupt);

	raw_min_x = 140;
	raw_max_x = 806;

	raw_min_y = 120;
	raw_max_y = 870;

	res_x     = 320;
	res_y     = 240;

	head = 0;
	tail = 0;
	xyswap = 1;

	cal_ok = 1;
	x_rev = 1;
	y_rev = 1;

	ts_clear();

	init_waitqueue_head(&queue);
        spin_lock_init(&my_lock);

#ifdef CONFIG_PM
	sh7727_ts_pm_dev = pm_register(PM_SYS_DEV, 0, sh7727_ts_pm_callback);
#endif

	ctrl_outw((ctrl_inw(PORT_PJCR) & 0xF03F) | 0x0540, PORT_PJCR);
	ctrl_outw((ctrl_inw(PORT_PLCR) & 0xFF0F) | 0x00A0, PORT_PLCR);
	ctrl_outb((ctrl_inb(PORT_PJDR)&0xC7)|0x28, PORT_PJDR);
	wait_for_action();

	/* JOG init */
//  	ctrl_outw((ctrl_inw(PMCR) & 0xFF03) | 0xA8, PMCR);
//	udelay(200);
	/* VM, VB buttom */
// 	ctrl_outw((ctrl_inw(PFCR) & 0xFFF0) | 0x0A, PFCR);
//	udelay(200);
	/* VH buttom */
//  	ctrl_outw((ctrl_inw(PGCR) & 0xFCFF) | 0x0200, PGCR);
//	udelay(200);

	set_ts_timer();
	init_backlight_mode_timer(&backlight_mode_timer);


#ifdef DO_CORRECTION
	/***************/
	calcIntersectioncoordinates();
#endif
	emX(ctrl_inw(INTC_ICR1));
	emX(ctrl_inb(INTC_IRR0));
	printk(KERN_INFO "sh7727 touch screen driver initialized.\n");

	return 0;
}

#ifdef MODULE
void __exit sh7727_ts_cleanup(void)
{
    del_ts_timer();
	unregister_chrdev(TS_MAJOR, TS_NAME);

	free_irq(IRQ0_IRQ, NULL);

	printk("sh7727 touch screen driver removed\n");
}
#endif

module_init(sh7727_ts_init);
#ifdef MODULE
module_exit(sh7727_ts_cleanup);
#endif

