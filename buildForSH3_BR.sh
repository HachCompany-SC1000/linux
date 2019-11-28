#!/bin/sh


OBJDIR=linux-3.2.18-sh3object
#TOOLS=/usr/bin/sh3-linux-
TOOLS=/home/sminke/opt/buildroot-2012.02/output/host/usr/bin/sh3-unknown-linux-uclibc-
#TOOLS=/home/sminke/opt/buildroot-2012.02/output/host/usr/bin/sh3-linux-

echo  "--------------------------------------------------  TOOLS   = ${TOOLS}"
echo  "--------------------------------------------------  OBJDIR  = ${OBJDIR}"




if [ "$1" = "used" ] ; then

	rm -fv list.L? list.collect list.all
	find ../${OBJDIR} -name "*.cmd" > list.cmdfiles
#	for L in /home/jochen/tmp/linux-3.2.18-sh3hlobj/arch/sh/kernel/cpu/sh3/.serial-sh770x.o.cmd ; do
	for L in `cat list.cmdfiles` ; do
		echo "			$L"
		grep -Eoh --text '(wildcard )?[a-z_0-9/.-]+\.[cho]' $L > list.L0
		sed "s#wildcard ##"  list.L0 >  list.L1
		sed "s#$THISDIR/##"  list.L1 >  list.L2
		grep -Ev '^/' list.L2  > list.L3
		grep -Ev '/\.[a-z]' list.L3  > list.L4
		grep -Ev '^(source|cmd|deps)_' list.L4  > list.L5
		sed -r 's#\.o$#.c#'  list.L5 >  list.L6
		cat list.L6 >> list.collect
	done
	echo "			sorting"
	sort -u  list.collect > list.all

else
	make O=../${OBJDIR}  ARCH=sh CROSS_COMPILE=${TOOLS} HOSTCC=gcc CC=${TOOLS}gcc $@



	if test "X$1X" != "XX" -o "X$?X" != "X0X" ; then
		exit;
	fi

	${TOOLS}objcopy -v -O binary ../${OBJDIR}/vmlinux  ../${OBJDIR}/ImageKernORG.bin
	cp -v /home/sminke/temp/${OBJDIR}/ImageKernORG.bin   /tftpboot/ImageKernORG.bin

fi
