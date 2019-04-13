#	$OpenBSD: Makefile,v 1.49 2017/01/11 20:43:15 deraadt Exp $
#	$NetBSD: Makefile,v 1.5 1995/09/15 21:05:21 pk Exp $

SUBDIR=	dev/microcode \
	arch/amd64 arch/arm64 arch/armv7 \
	arch/i386 \
	arch/landisk

tags:
	cd ${.CURDIR}/kern; make tags

.include <bsd.subdir.mk>
