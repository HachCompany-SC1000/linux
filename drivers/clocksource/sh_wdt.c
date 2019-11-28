/*
 * SuperH Watchdog Support - WDT
 *
 *  Copyright (C) 2012 Hach-Lange GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/sh_timer.h>
#include <linux/slab.h>
#include <linux/module.h>


#define WTCNT           0xFFFFFF84
#define WTCNT_PASS      0x5A00
#define WTCSR           0xFFFFFF86
#define WTCSR_PASS      0xA500

/* bit 7: TME           :       en-/disable timer                       */
/* bit 6: WT /IT        :       0 = use as interval timer               */
/* bit 5: RSTS          :       0 = Power on reset; 1 = Manual reset    */
/* bit 4: WOVF          :       Watchdog timer overflow                 */
/* bit 3: IOVF          :       overflow in interval timer mode         */
/* bit 2-0              :       Clock select                            */
#define WTCSR_TME       7
#define WTCSR_WT        6
#define WTCSR_RSTS      5
#define WTCSR_WOVF      4
#define WTCSR_IOVF      3
#define WTCSR_CKS_2     2
#define WTCSR_CKS_1     1
#define WTCSR_CKS_0     0

#define WTCSR_VALUE     0xC1


struct sh_wdt_priv {
        struct platform_device *pdev;
};


static void do_Reset( void )
{
    printk(KERN_INFO"system will be restarted, bye ...\n");
    ctrl_outw( 0 | WTCNT_PASS,WTCNT);
    ctrl_outw( (1<<WTCSR_CKS_2) | (1<<WTCSR_CKS_1) | (1<<WTCSR_CKS_0) |\
               (1<<WTCSR_TME) | (1<<WTCSR_WT) | WTCSR_PASS ,WTCSR);
}



static ssize_t show_attr(struct kobject *kobj, struct attribute *attr, 
                    char *buffer)
{
    return 0;
}


static ssize_t store_attr(struct kobject *kobj, struct attribute *attr, 
                     const char *buffer, size_t size)
{
    switch (buffer[0]){
        case '1':
            do_Reset();
            break;
    }
    return 128;
}

static struct sysfs_ops sops ={
    .show = show_attr,
    .store = store_attr,
};

static struct attribute  kAttr = {
    .name = "reset",
    .mode = ( S_IRUGO | S_IWUSR ),
};
static struct kobj_type ktype_default = {
    .sysfs_ops = &sops,
};

static struct kobject kobj;


static int sh_wdt_setup(struct sh_wdt_priv *p, struct platform_device *pdev)
{
	struct resource *res;

	int ret;
	ret = -ENXIO;

	memset(p, 0, sizeof(*p));
	p->pdev = pdev;

	platform_set_drvdata(pdev, p);

	res = platform_get_resource(p->pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&p->pdev->dev, "failed to get I/O memory\n");
		goto err0;
	}
	
	/* create new sysfs entry */
        memset(&kobj, 0, sizeof(struct kobject));
        kobject_init(&kobj,&ktype_default);
        ret = kobject_add(&kobj,&(pdev->dev.kobj),"attributes");
        if (ret) {
                dev_err(&p->pdev->dev, "failed to create attributes\n");
                goto err0;
        }
        
        ret = sysfs_create_file(&kobj,&kAttr);
        if (ret) {
                dev_err(&p->pdev->dev, "failed to create file in sysfs\n");
                goto err0;
        }
	return 0;
    
 err0:
	return ret;
}

static int __devinit sh_wdt_probe(struct platform_device *pdev)
{
	struct sh_wdt_priv *p = platform_get_drvdata(pdev);
	int ret;

        p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (p == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	ret = sh_wdt_setup(p, pdev);
	if (ret) {
		kfree(p);
		platform_set_drvdata(pdev, NULL);
	}
	return ret;
}

static int __devexit sh_wdt_remove(struct platform_device *pdev)
{
	return -EBUSY; /* cannot unregister clockevent and clocksource */
}

static struct platform_driver sh_wdt_device_driver = {
	.probe		= sh_wdt_probe,
	.remove		= __devexit_p(sh_wdt_remove),
	.driver		= {
		.name	= "sh_wdt",
                .owner = THIS_MODULE,
	}
};

static int __init sh_wdt_init(void)
{
	return platform_driver_register(&sh_wdt_device_driver);
}

static void __exit sh_wdt_exit(void)
{
	platform_driver_unregister(&sh_wdt_device_driver);
}


module_init(sh_wdt_init);
module_exit(sh_wdt_exit);

MODULE_AUTHOR("Sebastian Minke");
MODULE_DESCRIPTION("SuperH WDT Timer Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sh_wdt");
