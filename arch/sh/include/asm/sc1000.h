/*
 * include/asm-sh/hardware-sh7727.h
 *
 * Definitions for the SH7727 SH3-DSP processor. I'm hoping this is accurate
 * as I cut and pasted the values from the datasheet PDF!
 *
 * Copyright (c) 2004, MPC Data Limited (http://www.mpc-data.co.uk)
 * Author : Dave Peverley <dpeverley at mpc-data.co.uk>
 *
 * This file may be copied or modified under the terms of the GNU
 * General Public License.  See linux/COPYING for more information.
 *
 */




#ifndef __INCLUDED_ASM_HARDWARE_SH7727_H_
#define __INCLUDED_ASM_HARDWARE_SH7727_H_

/* External IRQ lines. These values are also defined as numbers in
 * sh7727-setup.c !!  I'm not sure what's the correct way to make the irq
 * number defines and that's why the double definition. */
#define IRQ0_IRQ 32
// #define IRQ1_IRQ 33
// #define IRQ2_IRQ 34
#define IRQ3_IRQ 35
// #define IRQ4_IRQ 36
// #define IRQ5_IRQ 37
//
#define ADC_IRQ 60



/* Tagen from the old sh7727_SC1000_Config.h.
 *
 * Generation of a config file for SC1000 with MAC-Address, serial number,
 * magic number The numbers are stored in network byte order which is big
 * endian. Output format is srec which can be loaded directly by Redboot as an
 * image file.  Startaddress is 0x8c200000 a free area before the Kernel.  Mem
 * address of Kernel is 0x8c201000 */


// SC1000 Config area
#define SC1000CF_MAGIC_NUM      0x5a61575a   /* magic number - cfg block is present */
#define SC1000CF_BASE           0x8c200000
#define SC1000CF_MAGIC1_OFFS    0
#define SC1000CF_XDATA_OFFS     4            /* 4:????????  5:??????xx   xx: video type */
#define SC1000CF_MAC_OFFS       6            /* MAC */
#define SC1000CF_SER_OFFS       12
#define SC1000CF_CFGORG_END     20           /* from here no memory is mapped.
                                              * Mapping is done by "fis load cfgORG".
                                              * Mapped size visible by "fis list -d".
                                              */


// Check if magic number for SC1000 config data exists
static __inline int
__SC1000Magic(unsigned long magic)
{
 int i;

	for (i = 3; i >= 0; i--, magic >>= 8)
		if(ctrl_inb(SC1000CF_BASE + i) != (0xff & magic))
			return 0;
	return 1;
}




/***************************************************************************
 * Rest of the definitions are generic for SH7727 CPU (i.e. no sc1000 specific
 * stuff after this line )
 * *************************************************************************/

/* DAC - D/A Converter */
#define DADR0		0xa40000a0 /* D/A Data Register 0 */
#define DADR1		0xa40000a2 /* D/A Data Register 1 */
#define DACR		0xa40000a4 /* D/A Control Register */
#define   DACR_DAOE1		(1 << 7)
#define   DACR_DAOE0		(1 << 6)
#define   DACR_DAE		(1 << 5)

/***************************************************************************
 * SH7727 Bus State Controller (BSC Module)
 **************************************************************************/

#define SH7727_SDMR                0xFFFFD000     // 8 bit access
#define SH7727_BCR1                0xFFFFFF60     // 16 bit access
#define SH7727_BCR2                0xFFFFFF62     // 16 bit access
#define SH7727_WCR1                0xFFFFFF64     // 16 bit access
#define SH7727_WCR2                0xFFFFFF66     // 16 bit access
#define SH7727_MCR                 0xFFFFFF68     // 16 bit access
#define SH7727_PCR                 0xFFFFFF6C     // 16 bit access
#define SH7727_RTCSR               0xFFFFFF6E     // 16 bit access
#define SH7727_RTCNT               0xFFFFFF70     // 16 bit access
#define SH7727_RTCOR               0xFFFFFF72     // 16 bit access
#define SH7727_RFCR                0xFFFFFF74     // 16 bit access



/***************************************************************************
 * SH7727 Interrupt Controller (INTC Module)
 * These defs used to be in irq.h but have migrated here for the SH7727
 **************************************************************************/

#define INTC_IRR0                  0xA4000004
#define INTC_IRR1                  0xA4000006
#define INTC_IRR2                  0xA4000008
#define INTC_IRR3                  0xA4000224
#define INTC_IRR4                  0xA4000226

#define INTC_ICR0                  0xFFFFFEE0
#define INTC_ICR1                  0xA4000010
#define INTC_ICR2                  0xA4000012
#define INTC_ICR3                  0xA4000228
#define INTC_INTER                 0xA4000014

#define INTC_IPRA                  0xFFFFFEE2
#define INTC_IPRB                  0xFFFFFEE4
#define INTC_IPRC                  0xA4000016
#define INTC_IPRD                  0xA4000018
#define INTC_IPRE                  0xA400001A
#define INTC_IPRF                  0xA4000220  // NOT: 0xA400001C
#define INTC_IPRG                  0xA4000222

#define INTC_PINTER                0xA4000014


/***************************************************************************
 * PFC - Pin Function Controller
 **************************************************************************/

#define SH7727_PFC_BASE            0xA4000100

#define PORT_PACR                  (SH7727_PFC_BASE + 0x00)
#define PORT_PBCR                  (SH7727_PFC_BASE + 0x02)
#define PORT_PCCR                  (SH7727_PFC_BASE + 0x04)
#define PORT_PDCR                  (SH7727_PFC_BASE + 0x06)
#define PORT_PECR                  (SH7727_PFC_BASE + 0x08)
#define PORT_PFCR                  (SH7727_PFC_BASE + 0x0A)
#define PORT_PGCR                  (SH7727_PFC_BASE + 0x0C)
#define PORT_PHCR                  (SH7727_PFC_BASE + 0x0E)
#define PORT_PJCR                  (SH7727_PFC_BASE + 0x10)
#define PORT_PKCR                  (SH7727_PFC_BASE + 0x12)
#define PORT_PLCR                  (SH7727_PFC_BASE + 0x14)
#define PORT_PSCCR                 (SH7727_PFC_BASE + 0x16)
#define PORT_PMCR                  (SH7727_PFC_BASE + 0x18)

/* For compatibility... */
#ifndef SCPCR
#define SCPCR                      PORT_PSCCR /* 16 bit SCI and SCIF */
#endif


/*
 * Mux values and a handy macro
 */

#define MD_OTHER_FUNCTION          0
#define MD_PORT_OUTPUT             1
#define MD_PORT_INPUT_PU           2
#define MD_PORT_INPUT              3

#define PFC_MUX(port_name, group, group_mode)  {       \
    unsigned short value;                              \
    value = ctrl_inw(PORT_P##port_name##CR);           \
    value &= ~(0x3 << (group * 2));                    \
    value |= MD_##group_mode << (group * 2);           \
    ctrl_outw(value, PORT_P##port_name##CR);           \
}



/***************************************************************************
 * PORTS - I/O ports
 **************************************************************************/

#define SH7727_PORT_BASE           0xA4000120

#define PORT_PADR                  (SH7727_PORT_BASE + 0x00)
#define PORT_PBDR                  (SH7727_PORT_BASE + 0x02)
#define PORT_PCDR                  (SH7727_PORT_BASE + 0x04)
#define PORT_PDDR                  (SH7727_PORT_BASE + 0x06)
#define PORT_PEDR                  (SH7727_PORT_BASE + 0x08)
#define PORT_PFDR                  (SH7727_PORT_BASE + 0x0A)
#define PORT_PGDR                  (SH7727_PORT_BASE + 0x0C)
#define PORT_PHDR                  (SH7727_PORT_BASE + 0x0E)
#define PORT_PJDR                  (SH7727_PORT_BASE + 0x10)
#define PORT_PKDR                  (SH7727_PORT_BASE + 0x12)
#define PORT_PLDR                  (SH7727_PORT_BASE + 0x14)
#define PORT_PSCDR                 (SH7727_PORT_BASE + 0x16)
#define PORT_PMDR                  (SH7727_PORT_BASE + 0x18)

/* For compatibility... */
#ifndef SCPDR
#define SCPDR  PORT_PSCDR /* 8  bit SCI and SCIF */
#endif

/* Port Data Register Definitions (Valid for A -> M) */
#define Px7DT                      (1 << 7)
#define Px6DT                      (1 << 6)
#define Px5DT                      (1 << 5)
#define Px4DT                      (1 << 4)
#define Px3DT                      (1 << 3)
#define Px2DT                      (1 << 2)
#define Px1DT                      (1 << 1)
#define Px0DT                      (1 << 0)



/***************************************************************************
 * SH7727 Serial Communication Interface (SCI Module)
 ***************************************************************************/

#define SH7727_SCI_BASE            0xFFFFFE80

#define SCSMR                      (SH7727_SCI_BASE + 0x00)
#define SCBBR                      (SH7727_SCI_BASE + 0x02)
#define SCSCR                      (SH7727_SCI_BASE + 0x04)
#define SCTDR                     (SH7727_SCI_BASE + 0x06)
#define SCSR                      (SH7727_SCI_BASE + 0x08)
#define SCRDR                     (SH7727_SCI_BASE + 0x0A)

#define SCSMR_CA                   (1 << 7)
#define SCSMR_CHR                  (1 << 6)
#define SCSMR_PE                   (1 << 5)
#define SCSMR_OE                   (1 << 4)
#define SCSMR_STOP                 (1 << 3)
#define SCSMR_MP                   (1 << 2)
#define SCSMR_CKS1                 (1 << 1)
#define SCSMR_CKS0                 (1 << 0)

#define SCSCR_TIE                  (1 << 7)
#define SCSCR_RIE                  (1 << 6)
#define SCSCR_TE                   (1 << 5)
#define SCSCR_RE                   (1 << 4)
#define SCSCR_MPIE                 (1 << 3)
#define SCSCR_TEIE                 (1 << 2)
#define SCSCR_CKE1                 (1 << 1)
#define SCSCR_CKE0                 (1 << 0)

#define SCSSR_TDRE                 (1 << 7)
#define SCSSR_RDRF                 (1 << 6)
#define SCSSR_ORER                 (1 << 5)
#define SCSSR_FER                  (1 << 4)
#define SCSSR_PER                  (1 << 3)
#define SCSSR_TEND                 (1 << 2)
#define SCSSR_MPB                  (1 << 1)
#define SCSSR_MPBT                 (1 << 0)



/***************************************************************************
 * SH7727 Serial Communication Interface with FIFO (SCIF Module)
 ***************************************************************************/

#define SH7727_SCIF_BASE           0xA4000150

#define SCSMR2                     (SH7727_SCIF_BASE + 0x00) // 8 bit access
#define SCBRR2                     (SH7727_SCIF_BASE + 0x02) // 8 bit access
#define SCSCR2                     (SH7727_SCIF_BASE + 0x04) // 8 bit access
#define SCFTDR2                    (SH7727_SCIF_BASE + 0x06) // 8 bit access
#define SCSSR2                     (SH7727_SCIF_BASE + 0x08) // 16 bit access
#define SCFRDR2                    (SH7727_SCIF_BASE + 0x0A) // 8 bit access
#define SCFCR2                     (SH7727_SCIF_BASE + 0x0C) // 8 bit access
#define SCFDR2                     (SH7727_SCIF_BASE + 0x0E) // 16 bit access

/* SCSSR2 Register Definitions */
#define SCSSR2_ER                   (1 << 7)
#define SCSSR2_TEND                 (1 << 6)
#define SCSSR2_TDFE                 (1 << 5)
#define SCSSR2_BRK                  (1 << 4)
#define SCSSR2_FER                  (1 << 3)
#define SCSSR2_PER                  (1 << 2)
#define SCSSR2_RDF                  (1 << 1)
#define SCSSR2_DR                   (1 << 0)



/***************************************************************************
 * LCDC - LDC Controller
 **************************************************************************/

#define SH7727_LDC_BASE            0xA4000C00

#define LDICKR                     (SH7727_LDC_BASE + 0x00)
#define LDMTR                      (SH7727_LDC_BASE + 0x02)
#define LDDFR                      (SH7727_LDC_BASE + 0x04)
#define LDSMR                      (SH7727_LDC_BASE + 0x06)
#define LDSARU                     (SH7727_LDC_BASE + 0x08)
#define LDSARL                     (SH7727_LDC_BASE + 0x0C)
#define LDLAOR                     (SH7727_LDC_BASE + 0x10)
#define LDPALCR                    (SH7727_LDC_BASE + 0x12)
#define LDHCNR                     (SH7727_LDC_BASE + 0x14)
#define LDHSYNR                    (SH7727_LDC_BASE + 0x16)
#define LDVDLNR                    (SH7727_LDC_BASE + 0x18)
#define LDVTLNR                    (SH7727_LDC_BASE + 0x1A)
#define LDVSYNR                    (SH7727_LDC_BASE + 0x1C)
#define LDACLNR                    (SH7727_LDC_BASE + 0x1E)
#define LDINTR                     (SH7727_LDC_BASE + 0x20)
#define LDPMMR                     (SH7727_LDC_BASE + 0x24)
#define LDPSPR                     (SH7727_LDC_BASE + 0x26)
#define LDCNTR                     (SH7727_LDC_BASE + 0x28)

#define LDPR                       0xA4000800 /* Palette Data Register */


/* LDICKR Register definitions */
#define LDICKR_B_CLK               0x0000  /* CKIO */
#define LDICKR_P_CLK               0x1000
#define LDICKR_E_CLK               0x2000

/* LDMTR Register definitions */
#define LDMTR_FLMPOL               (1 << 15)
#define LDMTR_CL1POL               (1 << 14)
#define LDMTR_DISPPOL              (1 << 13)
#define LDMTR_DPOL                 (1 << 12)
#define LDMTR_MCNT                 (1 << 10)
#define LDMTR_CL1CNT               (1 << 9)
#define LDMTR_CL2CNT               (1 << 8)
#define LDMTR_STN_MONO_4           0x00
#define LDMTR_STN_MONO_8           0x01
#define LDMTR_STN_COLOR_4          0x08
#define LDMTR_STN_COLOR_8          0x09
#define LDMTR_STN_COLOR_12         0x0A
#define LDMTR_STN_COLOR_16         0x0B
#define LDMTR_DSTN_MONO_8          0x11
#define LDMTR_DSTN_MONO_16         0x13
#define LDMTR_DSTN_COLOR_8         0x19
#define LDMTR_DSTN_COLOR_12        0x1A
#define LDMTR_DSTN_COLOR_16        0x1B
#define LDMTR_TFT_COLOR_16         0x2B

/* LDDFR Register definitions */
#define LDDFR_PABD                 (1 << 8)
#define LDDFR_MONO_2               0x00  /* 1bpp */
#define LDDFR_MONO_4               0x01  /* 2bpp */
#define LDDFR_MONO_16              0x02  /* 4bpp */
#define LDDFR_MONO_64              0x04  /* 6bpp */
#define LDDFR_COLOR_16             0x0A  /* 16 */
#define LDDFR_COLOR_256            0x0C  /* 256 */
#define LDDFR_COLOR_32K            0x1D  /* RGB:555 */
#define LDDFR_COLOR_64K            0x2D  /* RGB:565 */

/* LDSMR Register definitions */
#define LDSMR_ROT                  (1 << 13)
#define LDSMR_BURST_4              (0 << 8)
#define LDSMR_BURST_8              (1 << 8)
#define LDSMR_BURST_16             (2 << 8)
#define LDSMR_BURST_32             (3 << 8)

/* LDINTR Register definitions */
#define LDINTR_VINTSEL             (1 << 12)
#define LDINTR_VINTE               (1 << 8)
#define LDINTR_VINTS               (1 << 0)

/* LDPMMR Register definitions */
#define LDPMMR_VCPE                (1 << 6)
#define LDPMMR_VEPE                (1 << 5)
#define LDPMMR_DONE                (1 << 4)
#define LDPMMR_LPS1                (1 << 1)
#define LDPMMR_LPS0                (1 << 0)

/* LDCNTR Register definitions */
#define LDCNTR_DON                 (1 << 0)
#define LDCNTR_DON2                (1 << 4)



/***************************************************************************
 * SH7727 USB device controller registers
 **************************************************************************/

#define SH7727_USB_BASE            0xA4000240

#define USBIFR0                    (SH7727_USB_BASE + 0x00)
#define USBIFR1                    (SH7727_USB_BASE + 0x01)
#define USBEPDR0I                  (SH7727_USB_BASE + 0x02)
#define USBEPDR0O                  (SH7727_USB_BASE + 0x03)
#define USBTRG                     (SH7727_USB_BASE + 0x04)
#define USBFCLR                    (SH7727_USB_BASE + 0x05)
#define USBEPSZ0O                  (SH7727_USB_BASE + 0x06)
#define USBEPDR0S                  (SH7727_USB_BASE + 0x07)
#define USBDASTS                   (SH7727_USB_BASE + 0x08)
#define USBEPDR2                   (SH7727_USB_BASE + 0x09)
#define USBISR0                    (SH7727_USB_BASE + 0x0A)
#define USBEPSTL                   (SH7727_USB_BASE + 0x0B)
#define USBIER0                    (SH7727_USB_BASE + 0x0C)
#define USBIER1                    (SH7727_USB_BASE + 0x0D)
#define USBEPDR1                   (SH7727_USB_BASE + 0x0E)
#define USBEPSZ1                   (SH7727_USB_BASE + 0x0F)
#define USBISR1                    (SH7727_USB_BASE + 0x10)
#define USBDMA                     (SH7727_USB_BASE + 0x11)
#define USBEPDR3                   (SH7727_USB_BASE + 0x12)



/***************************************************************************
 * SH7727 A/D Converter
 **************************************************************************/

#define SH7727_AD_BASE             0xA4000080

#define ADDRAH                     (SH7727_AD_BASE + 0x00)
#define ADDRAL                     (SH7727_AD_BASE + 0x02)
#define ADDRBH                     (SH7727_AD_BASE + 0x04)
#define ADDRBL                     (SH7727_AD_BASE + 0x06)
#define ADDRCH                     (SH7727_AD_BASE + 0x08)
#define ADDRCL                     (SH7727_AD_BASE + 0x0A)
#define ADDRDH                     (SH7727_AD_BASE + 0x0C)
#define ADDRDL                     (SH7727_AD_BASE + 0x0E)
#define ADCSR                      (SH7727_AD_BASE + 0x10)
#define ADCR                       (SH7727_AD_BASE + 0x12)

#define ADCSR_ADF                  (1 << 7)
#define ADCSR_ADIE                 (1 << 6)
#define ADCSR_ADST                 (1 << 5)
#define ADCSR_MULTI                (1 << 4)
#define ADCSR_CKS                  (1 << 3)
#define ADCSR_CH2                  (1 << 2)
#define ADCSR_CH1                  (1 << 1)
#define ADCSR_CH0                  (1 << 0)

#define ADCR_TRGE1                 (1 << 7)
#define ADCR_TRGE0                 (1 << 6)
#define ADCR_SCN                   (1 << 5)



/***************************************************************************
 * SIOF Registers
 **************************************************************************/

#define SH7727_SIOF_BASE           0xA40000C0
#define SIOF_BASE                  SH7727_SIOF_BASE

#define SIMDR                      (SH7727_SIOF_BASE + 0x00)
#define SISCR                      (SH7727_SIOF_BASE + 0x02)
#define SITDAR                     (SH7727_SIOF_BASE + 0x04)
#define SIRDAR                     (SH7727_SIOF_BASE + 0x06)
#define SICDAR                     (SH7727_SIOF_BASE + 0x08)
#define SICTR                      (SH7727_SIOF_BASE + 0x0C)
#define SIFCTR                     (SH7727_SIOF_BASE + 0x10)
#define SISTR                      (SH7727_SIOF_BASE + 0x14)
#define SIIER                      (SH7727_SIOF_BASE + 0x16)
#define SITDR                      (SH7727_SIOF_BASE + 0x20)
#define SIRDR                      (SH7727_SIOF_BASE + 0x24)
#define SITCR                      (SH7727_SIOF_BASE + 0x28)
#define SIRCR                      (SH7727_SIOF_BASE + 0x2C)

#define SIMDR_TRMD1                (1 << 15)
#define SIMDR_TRMD0                (1 << 14)
#define SIMDR_REDG                 (1 << 12)
#define SIMDR_FL3                  (1 << 11)
#define SIMDR_FL2                  (1 << 10)
#define SIMDR_FL1                  (1 << 9)
#define SIMDR_FL0                  (1 << 8)
#define SIMDR_TXDIZ                (1 << 7)
#define SIMDR_LSBF                 (1 << 6)
#define SIMDR_RCIM                 (1 << 5)

#define SISCR_MSSEL                (1 << 15)
#define SISCR_MSIMM                (1 << 14)
#define SISCR_BRPS4                (1 << 12)
#define SISCR_BRPS3                (1 << 11)
#define SISCR_BRPS2                (1 << 10)
#define SISCR_BRPS1                (1 << 9)
#define SISCR_BRPS0                (1 << 8)
#define SISCR_BRDV2                (1 << 2)
#define SISCR_BRDV1                (1 << 1)
#define SISCR_BRDV0                (1 << 0)

#define SITDAR_TDLE                (1 << 15)
#define SITDAR_TDLA3               (1 << 11)
#define SITDAR_TDLA2               (1 << 10)
#define SITDAR_TDLA1               (1 << 9)
#define SITDAR_TDLA0               (1 << 8)
#define SITDAR_TDRE                (1 << 7)
#define SITDAR_TLREP               (1 << 6)
#define SITDAR_TDRA3               (1 << 3)
#define SITDAR_TDRA2               (1 << 2)
#define SITDAR_TDRA1               (1 << 1)
#define SITDAR_TDRA0               (1 << 0)

#define SIRDAR_RDLE                (1 << 15)
#define SIRDAR_RDLA3               (1 << 11)
#define SIRDAR_RDLA2               (1 << 10)
#define SIRDAR_RDLA1               (1 << 9)
#define SIRDAR_RDLA0               (1 << 8)
#define SIRDAR_RDRE                (1 << 7)
#define SIRDAR_RDRA3               (1 << 3)
#define SIRDAR_RDRA2               (1 << 2)
#define SIRDAR_RDRA1               (1 << 1)
#define SIRDAR_RDRA0               (1 << 0)

#define SICDAR_CD0E                (1 << 15)
#define SICDAR_CD0A3               (1 << 11)
#define SICDAR_CD0A2               (1 << 10)
#define SICDAR_CD0A1               (1 << 9)
#define SICDAR_CD0A0               (1 << 8)
#define SICDAR_CD1E                (1 << 7)
#define SICDAR_CD1A3               (1 << 3)
#define SICDAR_CD1A2               (1 << 2)
#define SICDAR_CD1A1               (1 << 1)
#define SICDAR_CD1A0               (1 << 0)

#define SICTR_SCKE                 (1 << 15)
#define SICTR_FSE                  (1 << 14)
#define SICTR_TXE                  (1 << 9)
#define SICTR_RXE                  (1 << 8)
#define SICTR_TXRST                (1 << 1)
#define SICTR_RXRST                (1 << 0)

#define SIFCTR_TFWM2               (1 << 15)
#define SIFCTR_TFWM1               (1 << 14)
#define SIFCTR_TFWM0               (1 << 13)
#define SIFCTR_TFUA4               (1 << 12)
#define SIFCTR_TFUA3               (1 << 11)
#define SIFCTR_TFUA2               (1 << 10)
#define SIFCTR_TFUA1               (1 << 9)
#define SIFCTR_TFUA0               (1 << 8)
#define SIFCTR_RFWM2               (1 << 7)
#define SIFCTR_RFWM1               (1 << 6)
#define SIFCTR_RFWM0               (1 << 5)
#define SIFCTR_RFUA4               (1 << 4)
#define SIFCTR_RFUA3               (1 << 3)
#define SIFCTR_RFUA2               (1 << 2)
#define SIFCTR_RFUA1               (1 << 1)
#define SIFCTR_RFUA0               (1 << 0)

#define SISTR_TCRDY                (1 << 14)
#define SISTR_TFEMP                (1 << 13)
#define SISTR_TDREQ                (1 << 12)
#define SISTR_RCRDY                (1 << 10)
#define SISTR_RFFUL                (1 << 9)
#define SISTR_RDREQ                (1 << 8)
#define SISTR_FSERR                (1 << 4)
#define SISTR_TFOVR                (1 << 3)
#define SISTR_TFUDR                (1 << 2)
#define SISTR_RFUDR                (1 << 1)
#define SISTR_RFOVR                (1 << 0)

#define SIIER_TCRDYE               (1 << 14)
#define SIIER_TFEMPE               (1 << 13)
#define SIIER_TDREQE               (1 << 12)
#define SIIER_RCRDYE               (1 << 10)
#define SIIER_RFFULE               (1 << 9)
#define SIIER_RDREQE               (1 << 8)
#define SIIER_FSERRE               (1 << 4)
#define SIIER_TFOVRE               (1 << 3)
#define SIIER_TFUDRE               (1 << 2)
#define SIIER_RFUDRE               (1 << 1)
#define SIIER_RFOVRE               (1 << 0)



/***************************************************************************
 * DMAC Registers
 **************************************************************************/

#define SH7727_DMAC_BASE           0xA4000020

#define SAR0                       (SH7727_DMAC_BASE + 0x00)
#define DAR0                       (SH7727_DMAC_BASE + 0x04)
#define DMATCR0                    (SH7727_DMAC_BASE + 0x08)
#define CHCR0                      (SH7727_DMAC_BASE + 0x0C)
#define SAR1                       (SH7727_DMAC_BASE + 0x10)
#define DAR1                       (SH7727_DMAC_BASE + 0x14)
#define DMATCR1                    (SH7727_DMAC_BASE + 0x18)
#define CHCR1                      (SH7727_DMAC_BASE + 0x1C)
#define SAR2                       (SH7727_DMAC_BASE + 0x20)
#define DAR2                       (SH7727_DMAC_BASE + 0x24)
#define DMATCR2                    (SH7727_DMAC_BASE + 0x28)
#define CHCR2                      (SH7727_DMAC_BASE + 0x2C)
#define SAR3                       (SH7727_DMAC_BASE + 0x30)
#define DAR3                       (SH7727_DMAC_BASE + 0x34)
#define DMATCR3                    (SH7727_DMAC_BASE + 0x38)
#define CHCR3                      (SH7727_DMAC_BASE + 0x3C)
//#define DMAOR                      (SH7727_DMAC_BASE + 0x40) - in dma.h
#define CHRAR                      (SH7727_DMAC_BASE + 0x020A)

#define CHCR_DI                    (1 << 20)
#define CHCR_RO                    (1 << 19)
#define CHCR_RL                    (1 << 18)
#define CHCR_AM                    (1 << 17)
#define CHCR_AL                    (1 << 16)
#define CHCR_DM1                   (1 << 15)
#define CHCR_DM0                   (1 << 14)
#define CHCR_SM1                   (1 << 13)
#define CHCR_SM0                   (1 << 12)
#define CHCR_RS3                   (1 << 11)
#define CHCR_RS2                   (1 << 10)
#define CHCR_RS1                   (1 << 9)
#define CHCR_RS0                   (1 << 8)
#define CHCR_DS                    (1 << 6)
#define CHCR_RM                    (1 << 5)
#define CHCR_TS1                   (1 << 4)
#define CHCR_TS0                   (1 << 3)
//#define CHCR_IE                    (1 << 2) - in dma.h
//#define CHCR_TE                    (1 << 1) - in dma.h
//#define CHCR_DE                    (1 << 0) - in dma.h

#define DMAOR_PR1                  (1 << 9)
#define DMAOR_PR0                  (1 << 8)
//#define DMAOR_AE                   (1 << 2) - in dma.h
//#define DMAOR_NMIF                 (1 << 1) - in dma.h
//#define DMAOR_DME                  (1 << 0) - in dma.h



/***************************************************************************
 * Misc - Definitions that need sorting out / tidying
 **************************************************************************/

#define STBCR3        0xA4000230
#define EXPFC         0xA4000234
#define EXCPGCR       0xA4000236






#endif   /* !  __INCLUDED_ASM_HARDWARE_SH7727_H_ */



