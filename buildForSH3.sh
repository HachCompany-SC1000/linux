#!/bin/sh


OBJDIR=linux-3.2.18-sh3object
TOOLS=/usr/bin/sh3-linux-
#TOOLS=/home/sminke/opt/buildroot-2012.02/output/host/usr/bin/sh3-unknown-linux-uclibc-

echo  "--------------------------------------------------  TOOLS   = ${TOOLS}"
echo  "--------------------------------------------------  OBJDIR  = ${OBJDIR}"
echo  "--------------------------------------------------  __weak  = ''"

make "__weak=" O=../${OBJDIR}  ARCH=sh CROSS_COMPILE=${TOOLS} HOSTCC=gcc CC=${TOOLS}gcc $@

if test "X$1X" != "XX" -o "X$?X" != "X0X" ; then
	exit;
fi

${TOOLS}objcopy -v -O binary ../${OBJDIR}/vmlinux  ../${OBJDIR}/ImageKernORG.bin
cp -v ../${OBJDIR}/ImageKernORG.bin   /tftpboot/ImageKernORG.bin

