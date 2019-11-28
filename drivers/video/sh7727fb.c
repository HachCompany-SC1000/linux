/*
 *  linux/drivers/video/sh7727fb.c
 *
 *  Copyright (c) 2001 Lineo Japan, Inc.
 *
 *  May be copied or modified under the terms of the GNU General Public
 *  License.  See linux/COPYING for more information.
 *
 *  LCD Frame Buffer Device Driver for SH7727

 Changelog:

 Andreas Schwenk     	Changes for LANGE HCD Board
         		set dispkay type in command line (Parameter: "disptype=")
                	0 for NEC NL3224BC35-20
			16 (0x10) for Sharp LCY99073

			adapt sh7727fb_enable_lcd_controller() to used displays

 Jochen Sparbier	[2008-07-21]  Added sysfs support via DEVICE_ATTR()
			/sys/class/graphics/fb0/vsync+hsync and /sys/class/graphics/fb0/topdown
			to adjust parameters.


*/
//#include <linux/autoconf.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bootmem.h>
#include <linux/proc_fs.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sc1000.h>

#define USE_PEGUIN  1              /* use a startup image (jsp) */
#define USE_REVERSED_TOP_DOWN  1   /* reversed display direction (jsp) */


#ifdef CONFIG_FB_SH7727_ROTATE /* SWK */
#define XRES 240
#define YRES 320
#define BPP  16
#else
#define XRES 320
#define YRES 240
#define BPP  16
#endif

#ifdef CONFIG_PM
static struct pm_dev *fb_pm_dev;
#endif

/* Use of statical mem needed for using SLAB memory allocator */
#define USE_STATIC_FBMEM

/* The framebuffer memory is at the end of physical memory. Kernel doesn't
 * know anything about it */
#define PHYS_FB_MEM_START (CONFIG_MEMORY_START+CONFIG_MEMORY_SIZE)

#define SH7727FB_MEM_SIZE ((XRES * YRES * BPP) / 8)
#define LINE_LENGTH        ((XRES * BPP) / 8)

void __udelay(unsigned long usecs);

int fb_handle_fbioset_lcd( unsigned long arg );

unsigned int videotype = 0x00;  /* not static - early access in setup.c */
static unsigned int hsync   = 0x5027;
static unsigned int vsync   = 0xae0;
static unsigned int topdown = 0;
static unsigned int inverse = 0;


int set_video_cfg(int videotype)
{
	switch (videotype) {
	default:  /* old display used until 2009/09 */
		hsync   = 0x5027;
		vsync   = 0xae0;
		topdown = 0;
		inverse = 0;
		break;
	case 1:
		hsync   = 0x5027;
		vsync   = 0xae0;
		topdown = 1;
		inverse = 1;
		break;
	}
	//printk(KERN_INFO "%s - videotype=%u, hsync=0x%x, vsync=0x%x, topdown=%u, inverse=%u\n", __FUNCTION__, videotype, hsync, vsync, topdown, inverse);
	return 0;
}


static int load_video_cfg(void)
{
	unsigned char  uc;
	if (!__SC1000Magic(SC1000CF_MAGIC_NUM)) {
		printk(KERN_ERR "%s SC1000Magic not found\n", __FUNCTION__);
		return -ENODATA;
	}
	uc = ctrl_inb(SC1000CF_BASE+SC1000CF_XDATA_OFFS+1);
	videotype = uc&0x03;
	set_video_cfg(videotype);
	return 0;
}


static int sh7727fb_enable_lcd_controller(void);

ssize_t show_videotype(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", videotype);
}
ssize_t store_videotype(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	videotype = simple_strtoul(buf, NULL, 0)&0x03;
	set_video_cfg(videotype);
	sh7727fb_enable_lcd_controller();
	return strnlen(buf, PAGE_SIZE);
}

ssize_t show_hsync(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", hsync);
}
ssize_t store_hsync(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	hsync = simple_strtoul(buf, NULL, 0);
	sh7727fb_enable_lcd_controller();
	return strnlen(buf, PAGE_SIZE);
}

ssize_t show_vsync(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", vsync);
}
ssize_t store_vsync(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	vsync = simple_strtoul(buf, NULL, 0);
	sh7727fb_enable_lcd_controller();
	return strnlen(buf, PAGE_SIZE);
}

ssize_t show_topdown(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", topdown);
}
ssize_t store_topdown(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	topdown = simple_strtoul(buf, NULL, 0);
	sh7727fb_enable_lcd_controller();
	return strnlen(buf, PAGE_SIZE);
}

ssize_t show_inverse(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", inverse);
}
ssize_t store_inverse(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	inverse = simple_strtoul(buf, NULL, 0);
	sh7727fb_enable_lcd_controller();
	return strnlen(buf, PAGE_SIZE);
}

static DEVICE_ATTR(videotype, S_IWUSR | S_IRUGO, &show_videotype, &store_videotype);
static DEVICE_ATTR(hsync,     S_IWUSR | S_IRUGO, &show_hsync,     &store_hsync);
static DEVICE_ATTR(vsync,     S_IWUSR | S_IRUGO, &show_vsync,     &store_vsync);
static DEVICE_ATTR(topdown,   S_IWUSR | S_IRUGO, &show_topdown,   &store_topdown);
static DEVICE_ATTR(inverse,   S_IWUSR | S_IRUGO, &show_inverse,   &store_inverse);


u32 pseudo_palette[16];

unsigned long vram_base_phys = 0;

static u16 fbcon_cmap_cfb16[16];

static int sh7727fb_contrast_value  = 0x80;
static int sh7727fb_backlight_value = 0x30;


static int sh7727fb_setcolreg(unsigned, unsigned, unsigned,
                              unsigned, unsigned, struct fb_info*);
static int sh7727fb_blank(int blank, struct fb_info* info);

static struct fb_ops sh7727fb_ops = {
    .owner = THIS_MODULE,
    .fb_setcolreg = sh7727fb_setcolreg,
    .fb_blank = sh7727fb_blank,
    .fb_fillrect = cfb_fillrect,
    .fb_copyarea = cfb_copyarea,
    .fb_imageblit = cfb_imageblit,
};

static struct fb_fix_screeninfo __devinitdata sh7727fb_fix = {

    .id = "sh7727fb",
    .type = FB_TYPE_PACKED_PIXELS,
    .visual = FB_VISUAL_TRUECOLOR,
    .xpanstep = 0,
    .ypanstep = 0,
    .ywrapstep = 0,
    .line_length = LINE_LENGTH,
    .smem_len = SH7727FB_MEM_SIZE,
    .accel = FB_ACCEL_NONE,

};




/*
 * Some display types have inverted control signal for backlight-enable.
 * This function accounts for this: pwr_off!=0 always means switch backlight off (dark).
 */
unsigned char sh7727fb_power_off_value(int pwr_off)
{
	//return (inverse ? (pwr_off ? 0x00: 0x80) : (pwr_off ? 0x80: 0x00));
	return (pwr_off ? 0x80: 0x00);
}

/*
 * Some display types have inverted control signal for contrast (/brightness)
 * This function accounts for this: darkness=0 always means max brightness.
 */
int sh7727fb_scale_contrast_value(int darkness)
{
	return (inverse ? 0xff - (darkness&0xff) : (darkness&0xff));
}



static int sh7727fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			     unsigned blue, unsigned transp,
			     struct fb_info* info)
{
	red   >>= 11;
	green >>= 11;
	blue  >>= 10;

	if (regno < 16)
		fbcon_cmap_cfb16[regno] = ((red & 31) << 6) |
		                          ((green & 31) << 11) |
					  ((blue & 63));

	return 0;
}


static int sh7727fb_disable_lcd_controller(void)
{
	ctrl_outw(0, LDCNTR); /* Display OFF */
	return 0;
}

#define emX(var) do{printk("DBG %s:%d: ",__FUNCTION__,__LINE__);printk("%s=0x%08x\n",#var,(uint32_t)var);}while(0);


// Logical Values are defined for NEC-Display
#define VGAMODE		(1 << 7)	// TFT-Displaycontrol VGA/QVGA
#define HSCANREV	(1 << 6)	// TFT-Displaycontrol Horizontal Scan Direction Reverse
#define VSCANREV	(1 << 4)	// TFT-Displaycontrol Vertical Scan Direction Reverse

static int sh7727fb_enable_lcd_controller(void)
{
	int i;

	/* Set display mode and orientation using comandline parameter "disptype" to adapt to used display */
	/* Example:0 for NEC NL3224BC35-20 or 8 for Sharp LCY99073 */

// 	ctrl_outw(0, LDCNTR); /* Display OFF */
// 	for(i=0; (ctrl_inw(LDPMMR) & (LDPMMR_LPS1+LDPMMR_LPS0)) != 0x00; i++){
// 		mdelay(1);
// 		if(i>1000){
// 			printk(KERN_ERR "%s: Could not turn display off\n",
// 					__FUNCTION__);
// 			return -EIO;
// 		}
// 	}

//---------------------------------------------------------------------------------
	// PIN initialisation belongs to setup.c - not here!

	/* Set Display Mode QVGA (QVGA/VGA) */
	// ctrl_outw((ctrl_inw(PORT_PDCR) & 0x3fff) | 0x4000, PORT_PDCR);	// Bit7 Port D output
	ctrl_outb((ctrl_inb(PORT_PDDR) & ~VGAMODE), PORT_PDDR);

	// ny> PTD7 is "Display on" - DON signal. What does it has to do
	// ny> with VGAMODE!?

	/* Set Scan Direction Horizontal */
	// ctrl_outw((ctrl_inw(PORT_PJCR) & 0xcfff) | 0x1000, PORT_PJCR);	// Bit6 Port J output
	ctrl_outb((ctrl_inb(PORT_PJDR) & ~HSCANREV), PORT_PJDR);

	// ny> According to schematics PTJ7 goes to "heizung" !!?

	/* Set Scan Direction Vertical */
	// ctrl_outw((ctrl_inw(PORT_PECR) & 0xfcff) | 0x0100, PORT_PECR);	// Bit4 Port E output
	if (topdown) {
		//#warning using Reversed Scan Direction Vertical
		ctrl_outb(ctrl_inb(PORT_PEDR) | VSCANREV, PORT_PEDR);
	}
	else {
		//#warning using normal Scan Direction Vertical
		ctrl_outb((ctrl_inb(PORT_PEDR) & ~VSCANREV), PORT_PEDR);
	}

	// ny> Why is signal M_DISP (SHMSIG, PTE6) not driven here?
//---------------------------------------------------------------------------------


#ifdef CONFIG_FB_SH7727_ROTATE
	/* LCDC Horizontal Character Number Register 320/8-1 // 320/8-1+3 */
	ctrl_outw(0x272d, LDHCNR);
	/* LCDC Horizontal Synchronization Signal Register */
	ctrl_outw(40, LDHSYNR);
	/* LCDC Vertical Display Line Number Register */
	ctrl_outw(XRES-1, LDVDLNR);
	/* LCDC Vertical Total Line Number Register */
	ctrl_outw(XRES-1, LDVTLNR);
	/* LCDC Vertical Synchronization Signal Register */
	/*ctrl_outw(0x30ef, LDVSYNR);*/
	ctrl_outw(0x3020, LDVSYNR);
#else
	/* LCDC Horizontal Character Number Register */
	ctrl_outw(0x2731, LDHCNR);
	/* LCDC Horizontal Synchronization Signal Register */
	//ctrl_outw(0x5027, LDHSYNR);
	ctrl_outw(hsync, LDHSYNR);
	/* LCDC Vertical Display Line Number Register */
	ctrl_outw(0xef, LDVDLNR);
	/* LCDC Vertical Total Line Number Register */
	ctrl_outw(0xfb, LDVTLNR);
	/* LCDC Vertical Synchronization Signal Register */
	//ctrl_outw(0xae0, LDVSYNR);	/* NEC */
	ctrl_outw(vsync, LDVSYNR);	/* NEC */
	printk(KERN_INFO "setting LDVSYNR = 0x%x\n", vsync);
#endif

	/* Select Input base clock */
	ctrl_outw(LDICKR_E_CLK + 0x8, LDICKR); /* E_CLOCK, 1/2 */
	/* LCDC Module Type SWK */
	/*ctrl_outw(LDMTR_FLMPOL|LDMTR_CL1POL|LDMTR_CL1CNT|LDMTR_CL2CNT|LDMTR_TFT_COLOR_16, LDMTR);*/ /* VSync LOW,HSync LOW,DE HIGH,LC-Data HIGH, */
	ctrl_outw(LDMTR_FLMPOL|LDMTR_CL1POL|LDMTR_TFT_COLOR_16, LDMTR);
	/* LCDC Data Format */
	ctrl_outw(LDDFR_PABD + LDDFR_COLOR_64K, LDDFR);


#ifdef CONFIG_FB_SH7727_ROTATE
	/* LCDC Scan Mode */
	ctrl_outw(LDSMR_ROT, LDSMR);
	/* LCD Start Address Register for Upper */
	ctrl_outl((vram_base_phys & 0x1FFFFFFF) + 0x0027e00,LDSARU);
	ctrl_outl(0, LDSARL);
	/* LDC Line Address Offset Register for Display Data */
	ctrl_outw(0x0200, LDLAOR);
#else
	/* LCDC Scan Mode */
	ctrl_outw(0, LDSMR);
	/* LCD Start Address Register for Upper */
	emX(vram_base_phys);
	ctrl_outl((vram_base_phys & 0x1FFFFFFF), LDSARU);
	ctrl_outl(0, LDSARL);
	/* LDC Line Address Offset Register for Display Data */
	/*ctrl_outw(0x01e0, LDLAOR);*/
	ctrl_outw(0x0280, LDLAOR);

#endif

	/* LCDC AC Line Number Register */
	ctrl_outw(0x0000, LDACLNR);
	/* LCDC Interrupt Control Register */
	ctrl_outw(0x0000, LDINTR); /* 0x1100 */
	/* LCDC Power Sequence Period Register SWK */
	ctrl_outw(0x0ff0, LDPSPR);
	/* LCDC Power Management Mode Register SWK  */
	ctrl_outw(0xff40, LDPMMR);


// // ----------------------- try order to avoid blinking at startup
// 	/* default contrast set */
// #if 1
// 	ctrl_outb(DACR_DAOE0, DACR);	// CCFL-Helligkeit auf DA0
// 	ctrl_outb(sh7727fb_scale_contrast_value(0), DADR0);		// max. Helligkeit
// #else
// 	ctrl_outb(DACR_DAOE1+DACR_DAOE0+DACR_DAE, DACR);
// 	ctrl_outb(sh7727fb_scale_contrast_value(sh7727fb_contrast_value), DADR0); /*SWK*/
// #endif
// // -----------------------


	ctrl_outw(LDCNTR_DON|LDCNTR_DON2, LDCNTR); /* Display ON */
	/* Wait until power on status bit set */
	for (i=0;(ctrl_inw(LDPMMR) & (LDPMMR_LPS1+LDPMMR_LPS0))!= (LDPMMR_LPS1+LDPMMR_LPS0); i++){
		mdelay(1);  // gcc 4.1 seems to be too fast for the HW...
		if(i>1000){
			printk(KERN_ERR "%s: Could not turn display on\n", __FUNCTION__);
			return -EIO;
		}
	}

	return 0;
}


int sh7727fb_get_contrast_value(void)
{
	return sh7727fb_contrast_value;
}

int sh7727fb_get_backlight_value(void)
{
	return sh7727fb_backlight_value;
}

void sh7727fb_set_contrast(int value)
{
#if 0
	sh7727fb_contrast_value += value;

	ctrl_outw(0, LDCNTR); /* Display OFF */
	while ((ctrl_inw(LDPMMR) & (LDPMMR_LPS1+LDPMMR_LPS0)) != 0x00) {}

	ctrl_outb(DACR_DAOE1+DACR_DAOE0+DACR_DAE, DACR);
	ctrl_outb(sh7727fb_contrast_value, DADR0);

	ctrl_outw(LDCNTR_DON|LDCNTR_DON2, LDCNTR); /* Display ON */
	while ((ctrl_inw(LDPMMR) & (LDPMMR_LPS1+LDPMMR_LPS0))
	       != (LDPMMR_LPS1+LDPMMR_LPS0)) {}
#endif
}

void sh7727fb_set_backlight(int value)
{
#if 0
	sh7727fb_backlight_value = value;

	ctrl_outw(0, LDCNTR); /* Display OFF */
	while ((ctrl_inw(LDPMMR) & (LDPMMR_LPS1+LDPMMR_LPS0)) != 0x00) {}

	/* backlight ON */
	ctrl_outb(sh7727fb_power_off_value(sh7727fb_backlight_value), PORT_PDDR);

	ctrl_outw(LDCNTR_DON|LDCNTR_DON2, LDCNTR); /* Display ON */
	while ((ctrl_inw(LDPMMR) & (LDPMMR_LPS1+LDPMMR_LPS0))
	       != (LDPMMR_LPS1+LDPMMR_LPS0)) {}
#endif
}


static int sh7727fb_blank(int blank, struct fb_info* info)
{
	if (blank) {
		return sh7727fb_disable_lcd_controller();
	}
	else {
		return sh7727fb_enable_lcd_controller();
	}
	return 0;
}


#ifdef CONFIG_PM
static int sh7727fb_pm_callback(struct pm_dev* pm_dev,
			       pm_request_t req, void* data)
{
	switch (req) {
	case PM_BLANK:
		return sh7727fb_disable_lcd_controller();
		break;
	case PM_UNBLANK:
		return sh7727fb_enable_lcd_controller();
		break;
	case PM_SUSPEND:
		return sh7727fb_disable_lcd_controller();
		break;
	case PM_RESUME:
		return sh7727fb_enable_lcd_controller();
		break;
	}

	return 0;
}
#endif





/* Fill the info structure with correct settings. A true programmer would of
 * course parse the values from the controller registers... */
void sh7727fb_init_fb_info(struct fb_info *info)
{
	info->var.xres           = XRES;
	info->var.yres           = YRES;
	info->var.xres_virtual   = XRES;
	info->var.yres_virtual   = YRES;
	info->var.bits_per_pixel = BPP;
	info->flags              = FBINFO_DEFAULT;

	info->var.red.length     = 5;
	info->var.blue.length    = 5;
	info->var.green.length   = 6;
	info->var.red.offset     = 11;
	info->var.green.offset   = 5;
	info->var.blue.offset    = 0;
}




void draw_test_pattern(unsigned char* video_base)
{
	int x, yy, y, l;
	unsigned char colbg, colgr, rgb[2*20];
	unsigned char *pos, *bp;
	unsigned long prod;
	for (y=0; y<30; y++) {
		for (x=0; x<16; x++) {
			prod = (x+13)*(y+17);
			colbg = ((unsigned char)((x+prod)*456273)) & 0x3F;
			colgr = ((unsigned char)((y+prod)*977213));
			pos = rgb;
			bp = video_base + y*8*LINE_LENGTH + x*2*20;
			for (l=0; l<20; l++) {
				*(pos++) = colgr;
				*(pos++) = colbg;
			}
			for (yy=0; yy<8; yy++) {
				if (yy<2 || yy>=6)
					memcpy(bp, rgb, 2*20);
				bp += LINE_LENGTH;
			}
		}
	}
}

#ifdef DEBUG_RECTANGLES

void sh7727fb_paint_rect(int row/*0..29*/, int column/*0..15*/)
{
	int x=column, yy, y=row, l;
	unsigned char colbg, colgr, rgb[2*20];
	unsigned char *pos, *bp;
	static unsigned char *base = 0;

//  	if (inited)
//  		__udelay(100000);

	if (!base)
		base =  ioremap(PHYS_FB_MEM_START, SH7727FB_MEM_SIZE);

	//printk(KERN_INFO "sh7727fb_paint_rect(%d, %d)\n", row, column);

	if ( row==-1 && column==-1) {
		draw_test_pattern(base);
		return;
	}

	y += x/16;
	x %= 16;

	colbg = 0xF1;
	colgr = 0x00;
	pos = rgb;
	bp = base + y*8*LINE_LENGTH + x*2*20;
	for (l=0; l<20; l++) {
		*(pos++) = colgr;
		*(pos++) = colbg;
	}
	for (yy=0; yy<8; yy++) {
		memcpy(bp, rgb, 2*20);
		bp += LINE_LENGTH;
	}
}
#else
#define sh7727fb_paint_rect(row,column) do{}while(0)
#endif




#if USE_PEGUIN
#include "logo/logo_linux_clut224.c"
#endif
int __init sh7727fb_init(void)
{
	struct fb_info *info;
	int ret;

	printk("%s\n",__FUNCTION__);

	info = framebuffer_alloc(0, NULL);
	if(!info){
		return -ENOMEM;
	}

#ifdef USE_STATIC_FBMEM
	vram_base_phys=PHYS_FB_MEM_START;
	info->screen_base = ioremap(vram_base_phys, SH7727FB_MEM_SIZE);
#else

        info->screen_base = kmalloc(SH7727FB_MEM_SIZE,GFP_KERNEL|__GFP_DMA);
	if(!info->screen_base){
		printk(KERN_ERR "%s: Could not allocate frame buffer memory\n",
				__FUNCTION__);
		return -ENOMEM;
	}
        vram_base_phys = virt_to_phys(info->screen_base);
#endif

	sh7727fb_fix.smem_start  = vram_base_phys;
	sh7727fb_fix.smem_len    = SH7727FB_MEM_SIZE;
	info->fix = sh7727fb_fix;

	fb_alloc_cmap(&info->cmap, 256, 0);

	info->pseudo_palette = pseudo_palette;

	info->fbops = &sh7727fb_ops;

	sh7727fb_init_fb_info(info);


	// show intial startup pattern:

#if 1
#warning TEST PATTERN
	memset(info->screen_base, 0x00, info->fix.smem_len);      // clear every thing
	draw_test_pattern((unsigned char*)info->screen_base);
#endif

#if 0
 	/* Clear the video memory */
 	memset(info->screen_base, 0xff, info->fix.smem_len);
#endif


	/* Set video memory to blue colour */
	//hellblau:      memset(info->screen_base, 0x1C, info->fix.smem_len);
	//dunkeltuerkis: memset(info->screen_base, 0x0C, info->fix.smem_len);
	//dunkelrot:     memset(info->screen_base, 0x40, info->fix.smem_len);
	//dunkelblau - sc1000 default:  memset(info->screen_base, 0x08, info->fix.smem_len);
	memset(info->screen_base, 0x08, info->fix.smem_len);      // dunkelblau - sc1000 default

#if USE_PEGUIN
#warning USE_PEGUIN  image must be 80x80
	{
		int x, y;
		unsigned short *us_from = imagebits, *us_to;
		for (y=0; y<80; y++) {
			us_to = (unsigned short*)(info->screen_base + (80-y)*LINE_LENGTH);
			for (x=0; x<80; x++) {
				if (*us_from)
					*us_to = *us_from;
				us_from++;
				us_to++;
			}
		}
        }
#endif

	sh7727fb_paint_rect(0, 0);

	load_video_cfg();  /* load from flash */
	ret = sh7727fb_enable_lcd_controller();

	if(ret){
		kfree(info->screen_base);
		return ret;
	}

	if (register_framebuffer(info) < 0) {
		printk(KERN_ERR "%s: unable to register SH7727 frame buffer\n",
				__FUNCTION__);
		return -EINVAL;
	}

#ifdef CONFIG_PM
	fb_pm_dev = pm_register(PM_SYS_DEV, 0, sh7727fb_pm_callback);
#endif

	printk(KERN_INFO "%s:%d %dx%dx%d (%dps/pixel), %ldK of video memory at 0x%08lx.\n",
			__FUNCTION__, info->node, info->var.xres,
			info->var.yres, info->var.bits_per_pixel,
			info->var.pixclock, (long) info->fix.smem_len >> 10,
			info->fix.smem_start);

	// already done by register_framebuffer:
	/*
	ret = device_register(info->dev);
 	if (ret)
 		printk(KERN_ERR "device_register() failed\n");
	*/
	get_device(info->dev);

	ret = 0;
 	ret |= device_create_file(info->dev, &dev_attr_videotype);
 	ret |= device_create_file(info->dev, &dev_attr_hsync);
 	ret |= device_create_file(info->dev, &dev_attr_vsync);
 	ret |= device_create_file(info->dev, &dev_attr_topdown);
 	ret |= device_create_file(info->dev, &dev_attr_inverse);
	if (ret) {
		printk(KERN_ERR "%s() device_create_file failed\n", __FUNCTION__);
		return -ENODATA;
	}

	return 0;
}

int __init sh7727fb_module_init(void)
{
	int ret;
        fb_handle_fbioset_lcd(FBIOSETLCD_POWEROFF);

	ret = sh7727fb_init();

 	ctrl_outb(DACR_DAOE0, DACR);                         /* CCFL-Helligkeit auf DA0 */
 	ctrl_outb(sh7727fb_scale_contrast_value(0), DADR0);  /* max. Helligkeit */

        fb_handle_fbioset_lcd(FBIOSETLCD_POWERON);

	return ret;
}

static void __exit sh7727fb_module_exit(void)
{
	/* Not supprted for now... */

	/* unmap and free the framebuffer memory and unregister framebuffer */
        fb_handle_fbioset_lcd(FBIOSETLCD_POWEROFF);
}

module_init(sh7727fb_module_init);
module_exit(sh7727fb_module_exit);
