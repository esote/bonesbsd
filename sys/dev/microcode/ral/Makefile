# $OpenBSD: Makefile,v 1.12 2018/10/02 02:05:34 kevlo Exp $

NOPROG=
NOMAN=
CC=${HOSTCC}

# PCI capable systems only
.if (${MACHINE} == "amd64" || \
     ${MACHINE} == "i386"

FIRM=	ral-rt2561 ral-rt2561s ral-rt2661 ral-rt2860 ral-rt3290

PROG=	build

CLEANFILES+= ${FIRM} ${PROG}

all: ${FIRM}

${FIRM}: build
	${.OBJDIR}/build

realinstall:

afterinstall:
	${INSTALL} -c -o ${BINOWN} -g ${BINGRP} -m 644 \
	    ${FIRM} ${DESTDIR}/etc/firmware
	${INSTALL} -c -o ${BINOWN} -g ${BINGRP} -m 644 \
	    ${.CURDIR}/ral-license ${DESTDIR}/etc/firmware

.endif

.include <bsd.prog.mk>
