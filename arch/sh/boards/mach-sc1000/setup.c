/*
 * linux/arch/sh/boards/unknown/setup.c
 *
 * Copyright (C) 2002 Paul Mundt
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * Setup code for an unknown machine (internal peripherials only)
 *
 * This is the simplest of all boards, and serves only as a quick and dirty
 * method to start debugging a new board during bring-up until proper board
 * setup code is written.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/bootmem.h>
#include <linux/debugfs.h>
#include <asm/irq.h>
#include <asm/machvec.h>
#include <asm/sc1000.h>
//#include <asm/cpu/timer.h>
#include <cpu/timer.h>

#include <linux/spi/spi.h>
// #include <linux/spi/spidev.h>
#include <linux/spi/mmc_spi.h>
// #include <linux/spi/sh_msiof.h>
#include <linux/mmc/host.h>
// #include <linux/mmc/sh_mmcif.h>



/* global variable for SC1000 MAC address, which is read from flash */
uint32_t sc1000_mac[]={0x08,0x88,0x12,0x34,0x56,0x78};
EXPORT_SYMBOL(sc1000_mac);

uint8_t sc1000_videocfg[] = {0x50,0x27,  0x0a,0xe0,  0,0,  0,0};  /*hsync, vsync, topdown, inverse*/
EXPORT_SYMBOL(sc1000_videocfg);


// static struct platform_device scisd_device = {
// 	.name		= "scisd",
// 	.id		= -1,
// };


static struct resource rtc_resources[] = {
	[0] = {
		.start	= 0xFFFFFEC0,
		.end	= 0xFFFFFEDF - 1,
		.flags	= IORESOURCE_IO,
	},
	[1] = {
		/* Period IRQ */
		.start	= 21,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		/* Carry IRQ */
		.start	= 22,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		/* Alarm IRQ */
		.start	= 20,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct platform_device sh7727_rtc_device = {
	.name		= "sh-rtc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(rtc_resources),
	.resource	= rtc_resources,
};



 static struct resource eth_resources[] = {
 	[0] = {
 		.start	= 0xB4000000,
 		.end	= 0xB4000000 + 0x20 -1,
 		.flags	= IORESOURCE_MEM,
 	},
 	[1] = {
 		/* ethernet IRQ */
 		.start	= 34,
 		.flags	= IORESOURCE_IRQ,
 	},
 };
 static struct platform_device cs89_eth_device = {
 	.name		= "cs89x0",
 	.id		= -1,
 	.num_resources	= ARRAY_SIZE(eth_resources),
 	.resource	= eth_resources,
 };
 
 
 
 
 
 
 
 /******************************* S P I ***************************************/
 
 static struct resource spi_resources[] __initdata = {
        [0] = {
                .start  = 0xfffffe80,
                .end    = 0xfffffe80 + 0xd,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                 /*  */
                 .name = "re",
                 .start  = 23,
                 .end    = 23,
                 .flags  = IORESOURCE_IRQ,
         },
         [2] = {
                 /*  */
                 .name = "rx",
                 .start  = 24,
                 .end    = 24,
                 .flags  = IORESOURCE_IRQ,
         },

        [3] = {
                .name = "tx",
                .start  = 25,
                .flags  = IORESOURCE_IRQ,
        },
        [4] = {
                /*  */
                .name = "te",
                .start = 26,
                .flags  = IORESOURCE_IRQ,
        },
         [5] = {
                 /*  */
                 .name = "cd",
                 .start = 41,
                 .flags  = IORESOURCE_IRQ,
         },
};

int spi_sh7727_init(struct device *dev,
                irqreturn_t (*handler)(int, void *),
                void *mmc); 
int spi_sh7727_getcd(struct device *dev); 

static struct mmc_spi_platform_data mmc_spi_pdata ={
    //e.g.: .caps = MMC_CAP_NEEDS_POLL,
    .init         = &spi_sh7727_init,
    .get_cd       = &spi_sh7727_getcd,
};
 
static struct platform_device spi_device __refdata = {
      .name      = "sh_spi",
      .id        = 0,

      .resource  = spi_resources,
      .num_resources  = ARRAY_SIZE(spi_resources),
};
 
static struct spi_board_info spi_board_info[]  = {
    [0] = {
        .modalias = "mmc_spi",
        .irq      = 23,
        .bus_num  = 0,
        .mode     = SPI_MODE_0,
        .chip_select = 1,
        .max_speed_hz = 1e6,
        .platform_data = &mmc_spi_pdata,
    },
};

/******************************** S P I ***************************************/



/*
 * watchdog device:
 *   - system reset 
 * 
 */

static struct resource wdt_resources[] = {
        [0] = {
                .start  = 0xFFFFFF84,
                .end    = 0xFFFFFF87,
                .flags  = IORESOURCE_MEM,
        },

};
static struct platform_device wdt_device = {
        .name           = "sh_wdt",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(wdt_resources),
        .resource       = wdt_resources,
};


 
static struct platform_device *sc1000_platform_devices[] __initdata = {
//	&sh7727_mmc_device,  //  Device 'sh7727_mmc' does not have a release() function, it is broken and must be fixed.
//	&scisd_device,       //  Device 'scisd' does not have a release() function, it is broken and must be fixed.
//	&sh7727_rtc_device,  //       -- see setup-sh770x.c
	&cs89_eth_device,
        &spi_device,
        &wdt_device,
};


static void start_sh_hwtimer_2( void )
{
#define TMU_TSTR_BIT_TMU2  (1<<2)
	/* timer 2: */
	//tctele = ctrl_inl(TMU2_TCNT);
	ctrl_outb(ctrl_inb(TMU_TSTR) & ~TMU_TSTR_BIT_TMU2, TMU_TSTR);	// Stop Timer 2 for update (assume this takes TMTICKS_FOR_UPDATE)
	ctrl_outw( 0x0000, TMU2_TCR );					// disable interrupts, use Phi/4
	ctrl_outl(0xFFFFFFFF, TMU2_TCNT);				// write to timer count
	ctrl_outb(ctrl_inb(TMU_TSTR) | TMU_TSTR_BIT_TMU2, TMU_TSTR);	// Start Timer 2 again after update

	return;
}


#ifdef CONFIG_DEBUG_FS
/**** Example how to put an entry into the debugfs ****/
u8 dummy_switch=0;
EXPORT_SYMBOL(dummy_switch);
static int __init sc1000_init_debugfs(void){
    struct dentry *d;
    d=debugfs_create_u8("dummy_switch", 0666, NULL, &dummy_switch);

    if(!d){
	return -1;
    } else {
	return 0;
    }
}
late_initcall(sc1000_init_debugfs);
#endif /* #ifdef CONFIG_DEBUG_FS */
/*********************************************/

static int __init sc1000_devices_setup(void)
{
    int ret;
    ret = platform_add_devices(sc1000_platform_devices, ARRAY_SIZE(sc1000_platform_devices));
    ret |= spi_register_board_info ( spi_board_info, ARRAY_SIZE(spi_board_info));
    return ret;
}
device_initcall(sc1000_devices_setup);


int __init setup_early_printk(char *buf);

#define OTHER_FUNCTION 0
#define OUTPUT 1
#define INPUT_PU_ON 2
#define INPUT_PU_OFF 3

#define CFG_PIN(port, pin, type) do{ \
    uint16_t val = ctrl_inw(PORT_P ## port ## CR); \
    val&=~(0x3<<((pin)*2)); \
    val|=(type<<((pin)*2)); \
    ctrl_outw(val, PORT_P ## port ## CR); \
}while(0);

#define HIGH 1
#define LOW 0

#define SET_PIN(port,pin,state) do{ \
    uint8_t val=ctrl_inb(PORT_P ## port ## DR); \
    if(state){ \
	ctrl_outb(val|(1<<(pin)), PORT_P ## port ## DR); \
    } else { \
	ctrl_outb(val&~(1<<(pin)), PORT_P ## port ## DR); \
    } \
} while(0)


#define USE_INITIAL_MMC_AT_SCI   1    // defines whether MMC or Modem is usable after sc1000_setup()  ### JSP
#if USE_INITIAL_MMC_AT_SCI
#define SWITCH_SCI(val)  (val)
#else
#define SWITCH_SCI(val)  (1-(val))
#endif

#define PIN_INIT_IGT    ctrl_outw((ctrl_inw(SCPCR) & 0xf33f) | 0x0480, SCPCR)   // Bit5 Port SCP output, Bit3 Input, Pullup - moved to setup.c

/* some initializations require early access to  videotype */
int set_video_cfg(int videotype);
extern unsigned int videotype;
unsigned char sh7727fb_power_off_value(int pwr_off);

#define CKIO2CR		0xA400023A  /* CKIO2 Control Register */

static void __init sc1000_setup(char **cmdline_p)
{
    int i;
    //setup_early_printk("serial");

    /******************************************************************/
    /* This is a leftover from the earlier pin initialisation. I tried to make
     * a bit more readable below, but just in case some other HW on SC1000
     * (which I'm not aware of) needs this, I'll just leave it here (ny). */
	ctrl_outw(0x0000, PORT_PCCR);
	ctrl_outw(0x4000, PORT_PDCR);
	ctrl_outw(0x4100, PORT_PECR);
	ctrl_outw(0x0000, PORT_PFCR);
	ctrl_outw(0x0000, PORT_PGCR);
	ctrl_outw(0x0000, PORT_PHCR);
	ctrl_outw(0x0000, PORT_PJCR);
	ctrl_outw(0x0000, PORT_PKCR);
	ctrl_outw(0x0000, PORT_PLCR);
	ctrl_outw(0x0000, PORT_PMCR);
    /***************************************************************************/

    /* Port H pin control register. PH0-PH3 correspond IRQ0-IRQ3. Value 0
     * ("other function") sets them to be interrupt inputs */
    CFG_PIN(H,0,OTHER_FUNCTION);
    CFG_PIN(H,1,OTHER_FUNCTION);
    CFG_PIN(H,2,OTHER_FUNCTION);
    CFG_PIN(H,3,OTHER_FUNCTION);

    /* Switch off CKIO2 to minimize rf emission: */
    ctrl_outw(0x0001, CKIO2CR);

    // ctrl_outw(0x8000, SCPCR);  // is this needed for SCI synchronous mode?

    /* Set the MMC enable pin as output and set the initial value (signal is
     * low active)*/
    CFG_PIN(J,7,OUTPUT);
    SET_PIN(J,7,SWITCH_SCI(LOW));


    /* Set the "Sense Modem is Present" pin as input */
    CFG_PIN(SC,3,INPUT_PU_ON);


    /* Set the MMC power pin as output and set the initial value */
    CFG_PIN(SC,5,OUTPUT);
    SET_PIN(SC,5,LOW);

    /* Set the RTS0 modem pin as debug output and set the initial value */
    CFG_PIN(SC,6,OUTPUT);
    SET_PIN(SC,6,HIGH);

    /* Set TX pin of SCI to high */
    CFG_PIN(SC,0,OUTPUT);
    SET_PIN(SC,0,HIGH);

    /* Set MMC MUX pin as output and set the initial value (signal is low
     * active) */
    CFG_PIN(E,7,OUTPUT);
    SET_PIN(E,7,SWITCH_SCI(LOW));

    /* Set MMC CS as output and set the initial value (signal is low active) */
    SET_PIN(K,2,SWITCH_SCI(HIGH));
    CFG_PIN(K,2,OUTPUT);

    /* Set 12V DataTele as output and set the initial value */
    CFG_PIN(K,4,OUTPUT);
    SET_PIN(K,4,HIGH);

    
    /* debug */
    
    CFG_PIN(E,0,OUTPUT);
    SET_PIN(E,0,HIGH);
    
    /* LCDC Signals */


    /* ny: I saw from the old pin configuration (from the sc1000 2.4 bsp) that
     * the DON was set up as an output pin and driven manually low. I think
     * this should be done by the lcd controller. Below is the 'old
     * configuration just in case it its needed */
#if 1
    CFG_PIN(D,7,OUTPUT);         /* DON */
    SET_PIN(D,7,HIGH);           /* turn display off */
#endif


    /* Values are set in the frame buffer driver */
    CFG_PIN(E,4,OUTPUT);  /* Scan direction vertical (signal SHPTE4) */
    CFG_PIN(E,6,OUTPUT);  /* M_DISP ("SHMSIG" on the schematics) */

    //CFG_PIN(D,7,OTHER_FUNCTION); /* DON  */
    CFG_PIN(D,5,OTHER_FUNCTION); /* CL1  */
    CFG_PIN(H,7,OTHER_FUNCTION); /* CL2  */
    CFG_PIN(E,3,OTHER_FUNCTION); /* FLM */
    CFG_PIN(E,6,OTHER_FUNCTION); /* M_DISP */
    CFG_PIN(D,6,OTHER_FUNCTION); /* LCK (lcd clock input)  */

    CFG_PIN(F,3,INPUT_PU_ON);    /* SD insert irq PINT11 */

    CFG_PIN(D,0,OTHER_FUNCTION);
    CFG_PIN(D,1,OTHER_FUNCTION);
    CFG_PIN(D,2,OTHER_FUNCTION);
    CFG_PIN(D,3,OTHER_FUNCTION);

    CFG_PIN(C,0,OTHER_FUNCTION);
    CFG_PIN(C,1,OTHER_FUNCTION);
    CFG_PIN(C,2,OTHER_FUNCTION);
    CFG_PIN(C,3,OTHER_FUNCTION);
    CFG_PIN(C,4,OTHER_FUNCTION);
    CFG_PIN(C,5,OTHER_FUNCTION);
    CFG_PIN(C,6,OTHER_FUNCTION);
    CFG_PIN(C,7,OTHER_FUNCTION);

    CFG_PIN(M,0,OTHER_FUNCTION);
    CFG_PIN(M,1,OTHER_FUNCTION);
    CFG_PIN(M,2,OTHER_FUNCTION);
    CFG_PIN(M,3,OTHER_FUNCTION);

#if 0
    ctrl_outb(DACR_DAOE0, DACR); // CCFL-Helligkeit auf DA0
    ctrl_outb(0, DADR0);         // min. Helligkeit
#endif

    /* Read the MAC address from the special flash location and save it into a
     * global variable.
     *
     * Dies wurde nicht getestet weil es weder Magic number noch MAC addresse
     * in meinem Entwicklungsboard gibt. Nylund */
    if(__SC1000Magic(SC1000CF_MAGIC_NUM)) {
	videotype =  ctrl_inb(SC1000CF_BASE+SC1000CF_XDATA_OFFS+1) & 0x03;
	set_video_cfg(videotype);

	/* handle backlight: */
	ctrl_outb(DACR_DAOE0, DACR);            // CCFL-Helligkeit auf DA0
	ctrl_outb(videotype?0:255, DADR0);      // min. Helligkeit

	ctrl_outb((ctrl_inb(PORT_PDDR)&~0x80) | sh7727fb_power_off_value(0x80), PORT_PDDR); /* Display OFF */

	/* handle MAC address: */
	//printk("%s: Found MAC address ",__FUNCTION__);
	for (i = 0; i < 6; i++) {
	    sc1000_mac[i]=ctrl_inb(SC1000CF_BASE+SC1000CF_MAC_OFFS + i);
	    //printk("%02x ",sc1000_mac[i]);
	}
	//printk("\n");
    } else {
        //printk(KERN_ERR "%s: SC1000 Magic not found! Using a dummy MAC address\n", __FUNCTION__);
    }

    //start_sh_hwtimer_2();		/* start TMU2 as free running counter for latency measurements */
}



//////////////////////////////////////////////////////////////////////////
/*
 * Test function to toggle PortE5 bit (sc1000:485Enable)
 */
#define BIT_PSWITCH	(1 << 7)	// position ttySC0 port switch
#define BIT_485TRM	(1 << 5)	// position of 485 driver enable pin
#define BIT_IGT		(1 << 5)	// position IGT
#define BIT_DBG		(1 << 6)	// position DBG bit
#define BIT_GSM		(1 << 3)	// position GSM pugged in
#define BIT_MODEM_POWER (1 << 4)	// position 12V Data Tele
#define REG_485_SW	PORT_PEDR	// 485 driver enable register
#define REG_IGT_SW	SCPDR	// IGT enable/disable
#define REG_CTL_IGT_SW	SCPCR	// IGT enable/disable
#define REG_12V_DATATELE PORT_PKDR
#define PIN_INIT_485TR	ctrl_outw((ctrl_inw(PORT_PECR) & 0xf3ff) | 0x0400, PORT_PECR)	// Bit5 Port E output
#define PIN_INIT_PTSWI	ctrl_outw((ctrl_inw(PORT_PECR) & 0x3fff) | 0x4000, PORT_PECR)	// Bit7 Port E output
//#define PIN_INIT_IGT	ctrl_outw((ctrl_inw(SCPCR) & 0xf33f) | 0x0480, SCPCR)	// Bit5 Port SCP output, Bit3 Input, Pullup - moved to setup.c
/* IGT switch */
#define PIN_SWI_IGT_ON	ctrl_outb(ctrl_inb(REG_IGT_SW) |  BIT_IGT, REG_IGT_SW)	// IGT on
#define PIN_SWI_IGT_OFF	ctrl_outb(ctrl_inb(REG_IGT_SW) & ~BIT_IGT, REG_IGT_SW)	// IGT off
/* Modem RTS0 switch for debug purpose */

#define CFG_PIN(port, pin, type) do{ \
    uint16_t val = ctrl_inw(PORT_P ## port ## CR); \
    val&=~(0x3<<((pin)*2)); \
    val|=(type<<((pin)*2)); \
    ctrl_outw(val, PORT_P ## port ## CR); \
}while(0);
/* Set the RTS0 modem pin as debug output and set the initial value */
#define OUTPUT 1
#define PIN_SWI_DBG_ENA	    CFG_PIN(SC,6,OUTPUT)
#define PIN_SWI_DBG_ON	ctrl_outb(ctrl_inb(REG_IGT_SW) |  BIT_DBG, REG_IGT_SW)	// IGT on
#define PIN_SWI_DBG_OFF	ctrl_outb(ctrl_inb(REG_IGT_SW) & ~BIT_DBG, REG_IGT_SW)	// IGT off
/* GSM pugged in */
#define PIN_CHK_GSM	(~ctrl_inb(REG_IGT_SW) & BIT_GSM)	// GSM
/* Port Switch */
#define PIN_SWI_GSM	ctrl_outb(ctrl_inb(REG_485_SW) |  BIT_PSWITCH, REG_485_SW)	// switch to GSM
#define PIN_SWI_MMC	ctrl_outb(ctrl_inb(REG_485_SW) & ~BIT_PSWITCH, REG_485_SW)	// switch to MMC
/* MODEM Power */
#define PIN_SWI_MODEM_ON  ctrl_outb(ctrl_inb(REG_12V_DATATELE) |  BIT_MODEM_POWER, REG_12V_DATATELE)	// switch modem power on
#define PIN_SWI_MODEM_OFF ctrl_outb(ctrl_inb(REG_12V_DATATELE) & ~BIT_MODEM_POWER, REG_12V_DATATELE)	// switch modem power off
/* RS485 Transmitter */
#define PIN_ENAB_485TR	ctrl_outb(ctrl_inb(REG_485_SW) & ~BIT_485TRM, REG_485_SW)	// enable RS485 transmitter
#define PIN_DISA_485TR	ctrl_outb(ctrl_inb(REG_485_SW) |  BIT_485TRM, REG_485_SW)	// disable RS485 transmitter
#define PIN_IS_ENAB_485	(!(ctrl_inb(REG_485_SW) & BIT_485TRM))	// check if transmitter is enabled
//////////////////////////////////////////////////////////////////////////

void sc1000_blinkwith_485Enable(int cycles)
{
	static int inited = 0;
	if (!inited) {
		PIN_INIT_485TR;
		inited = 1;
	}
	while(cycles-- > 0) {
		PIN_ENAB_485TR;
		PIN_DISA_485TR;
	}
}
//////////////////////////////////////////////////////////////////////////
extern void __init init_sh7727_IRQ(void);

struct sh_machine_vector mv_sc1000  __initmv = {
	.mv_name		= "sc1000",
	.mv_setup               = sc1000_setup,
	//.mv_init_irq		= init_sh7727_IRQ,

};

#define ALIAS_MV(system) \
  asm(".global sh_mv\nsh_mv = mv_"#system );

ALIAS_MV(sc1000)
