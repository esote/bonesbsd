/*	$OpenBSD: if_media.h,v 1.41 2018/02/04 10:06:51 stsp Exp $	*/
/*	$NetBSD: if_media.h,v 1.22 2000/02/17 21:53:16 sommerfeld Exp $	*/

/*-
 * Copyright (c) 1998, 2000 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe of the Numerical Aerospace Simulation Facility,
 * NASA Ames Research Center.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 1997
 *	Jonathan Stone and Jason R. Thorpe.  All rights reserved.
 *
 * This software is derived from information provided by Matt Thomas.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Jonathan Stone
 *	and Jason R. Thorpe for the NetBSD Project.
 * 4. The names of the authors may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _NET_IF_MEDIA_H_
#define _NET_IF_MEDIA_H_

#ifdef _KERNEL

struct ifnet;

#include <sys/queue.h>
/*
 * Driver callbacks for media status and change requests.
 */
typedef	int (*ifm_change_cb_t)(struct ifnet *);
typedef	void (*ifm_stat_cb_t)(struct ifnet *, struct ifmediareq *);

/*
 * In-kernel representation of a single supported media type.
 */
struct ifmedia_entry {
	TAILQ_ENTRY(ifmedia_entry) ifm_list;
	uint64_t	ifm_media;	/* description of this media attachment */
	u_int	ifm_data;	/* for driver-specific use */
	void	*ifm_aux;	/* for driver-specific use */
};

/*
 * One of these goes into a network interface's softc structure.
 * It is used to keep general media state.
 */
struct ifmedia {
	uint64_t	ifm_mask;	/* mask of changes we don't care about */
	uint64_t	ifm_media;	/* current user-set media word */
	struct ifmedia_entry *ifm_cur;	/* currently selected media */
	TAILQ_HEAD(, ifmedia_entry) ifm_list; /* list of all supported media */
	ifm_change_cb_t	ifm_change;	/* media change driver callback */
	ifm_stat_cb_t	ifm_status;	/* media status driver callback */
};

/* Initialize an interface's struct if_media field. */
void	ifmedia_init(struct ifmedia *, uint64_t, ifm_change_cb_t,
	     ifm_stat_cb_t);

/* Add one supported medium to a struct ifmedia. */
void	ifmedia_add(struct ifmedia *, uint64_t, int, void *);

/* Add an array (of ifmedia_entry) media to a struct ifmedia. */
void	ifmedia_list_add(struct ifmedia *, struct ifmedia_entry *,
	    int);

/* Set default media type on initialization. */
void	ifmedia_set(struct ifmedia *, uint64_t);

/* Common ioctl function for getting/setting media, called by driver. */
int	ifmedia_ioctl(struct ifnet *, struct ifreq *, struct ifmedia *,
	    u_long);

/* Locate a media entry */
struct	ifmedia_entry *ifmedia_match(struct ifmedia *, uint64_t, uint64_t);

/* Delete all media for a given media instance */
void	ifmedia_delete_instance(struct ifmedia *, uint64_t);

/* Compute baudrate for a given media. */
uint64_t	ifmedia_baudrate(uint64_t);
#endif /*_KERNEL */

/*
 * if_media Options word:
 *	Bits	Use
 *	----	-------
 *	0-7	Media subtype		MAX SUBTYPE == 255!
 *	8-15	Media type
 *	16-31	Type specific options
 *	32-39	Mode (for multi-mode devices)
 *	40-55	Shared (global) options
 *	56-63	Instance
 */

/*
 * Ethernet
 */
#define IFM_ETHER	0x0000000000000100ULL
#define	IFM_10_T	3		/* 10BaseT - RJ45 */
#define	IFM_10_2	4		/* 10Base2 - Thinnet */
#define	IFM_10_5	5		/* 10Base5 - AUI */
#define	IFM_100_TX	6		/* 100BaseTX - RJ45 */
#define	IFM_100_FX	7		/* 100BaseFX - Fiber */
#define	IFM_100_T4	8		/* 100BaseT4 - 4 pair cat 3 */
#define	IFM_100_VG	9		/* 100VG-AnyLAN */
#define	IFM_100_T2	10		/* 100BaseT2 */
#define	IFM_1000_SX	11		/* 1000BaseSX - multi-mode fiber */
#define	IFM_10_STP	12		/* 10BaseT over shielded TP */
#define	IFM_10_FL	13		/* 10BaseFL - Fiber */
#define	IFM_1000_LX	14		/* 1000baseLX - single-mode fiber */
#define	IFM_1000_CX	15		/* 1000baseCX - 150ohm STP */
#define	IFM_1000_T	16		/* 1000baseT - 4 pair cat 5 */
#define	IFM_1000_TX	IFM_1000_T	/* for backwards compatibility */
#define	IFM_HPNA_1	17		/* HomePNA 1.0 (1Mb/s) */
#define	IFM_10G_LR	18		/* 10GBase-LR - single-mode fiber */
#define	IFM_10G_SR	19		/* 10GBase-SR - multi-mode fiber */
#define	IFM_10G_CX4	20		/* 10GBase-CX4 - copper */
#define	IFM_2500_SX	21		/* 2500baseSX - multi-mode fiber */
#define	IFM_10G_T	22		/* 10GbaseT cat 6 */
#define	IFM_10G_SFP_CU	23		/* 10G SFP+ direct attached cable */
#define	IFM_10G_LRM	24		/* 10GBase-LRM 850nm Multi-mode */
#define	IFM_40G_CR4	25		/* 40GBase-CR4 */
#define	IFM_40G_SR4	26		/* 40GBase-SR4 */
#define	IFM_40G_LR4	27		/* 40GBase-LR4 */
#define	IFM_1000_KX	28		/* 1000Base-KX backplane */
#define	IFM_10G_KX4	29		/* 10GBase-KX4 backplane */
#define	IFM_10G_KR	30		/* 10GBase-KR backplane */
#define	IFM_10G_CR1	31		/* 10GBase-CR1 Twinax splitter */
#define	IFM_20G_KR2	32		/* 20GBase-KR2 backplane */
#define	IFM_2500_KX	33		/* 2500Base-KX backplane */
#define	IFM_2500_T	34		/* 2500Base-T - RJ45 (NBaseT) */
#define	IFM_5000_T	35		/* 5000Base-T - RJ45 (NBaseT) */
#define	IFM_1000_SGMII	36		/* 1G media interface */
#define	IFM_10G_SFI	37		/* 10G media interface */
#define	IFM_40G_XLPPI	38		/* 40G media interface */
#define	IFM_1000_CX_SGMII 39		/* 1000Base-CX-SGMII */
#define	IFM_40G_KR4	40		/* 40GBase-KR4 */
#define	IFM_10G_ER	41		/* 10GBase-ER */
#define	IFM_100G_CR4	42		/* 100GBase-CR4 */
#define	IFM_100G_SR4	43		/* 100GBase-SR4 */
#define	IFM_100G_KR4	44		/* 100GBase-KR4 */
#define	IFM_100G_LR4	45		/* 100GBase-LR4 */
#define	IFM_56G_R4	46		/* 56GBase-R4 */
#define	IFM_25G_CR	47		/* 25GBase-CR */
#define	IFM_25G_KR	48		/* 25GBase-KR */
#define	IFM_25G_SR	49		/* 25GBase-SR */
#define	IFM_50G_CR2	50		/* 50GBase-CR2 */
#define	IFM_50G_KR2	51		/* 50GBase-KR2 */
#define	IFM_25G_LR	52		/* 25GBase-LR */
#define	IFM_25G_ER	53		/* 25GBase-ER */
#define	IFM_10G_AOC	54		/* 10G Active Optical Cable */
#define	IFM_25G_AOC	55		/* 25G Active Optical Cable */
#define	IFM_40G_AOC	56		/* 40G Active Optical Cable */
#define	IFM_100G_AOC	57		/* 100G Active Optical Cable */

#define	IFM_ETH_MASTER	0x0000000000010000ULL	/* master mode (1000baseT) */
#define	IFM_ETH_RXPAUSE	0x0000000000020000ULL	/* receive PAUSE frames */
#define	IFM_ETH_TXPAUSE	0x0000000000040000ULL	/* transmit PAUSE frames */

/*
 * FDDI
 */
#define	IFM_FDDI	0x0000000000000300ULL
#define	IFM_FDDI_SMF	3		/* Single-mode fiber */
#define	IFM_FDDI_MMF	4		/* Multi-mode fiber */
#define IFM_FDDI_UTP	5		/* CDDI / UTP */
#define IFM_FDDI_DA	0x00000100	/* Dual attach / single attach */

/*
 * Digitally multiplexed "Carrier" Serial Interfaces
 */
#define	IFM_TDM		0x0000000000000500ULL
#define IFM_TDM_T1		3	/* T1 B8ZS+ESF 24 ts */
#define IFM_TDM_T1_AMI		4	/* T1 AMI+SF 24 ts */
#define IFM_TDM_E1		5	/* E1 HDB3+G.703 clearchannel 32 ts */
#define IFM_TDM_E1_G704		6	/* E1 HDB3+G.703+G.704 channelized 31 ts */
#define IFM_TDM_E1_AMI		7	/* E1 AMI+G.703 32 ts */
#define IFM_TDM_E1_AMI_G704	8	/* E1 AMI+G.703+G.704 31 ts */
#define IFM_TDM_T3		9	/* T3 B3ZS+C-bit 672 ts */
#define IFM_TDM_T3_M13		10	/* T3 B3ZS+M13 672 ts */
#define IFM_TDM_E3		11	/* E3 HDB3+G.751 512? ts */
#define IFM_TDM_E3_G751		12	/* E3 G.751 512 ts */
#define IFM_TDM_E3_G832		13	/* E3 G.832 512 ts */
#define IFM_TDM_E1_G704_CRC4	14	/* E1 HDB3+G.703+G.704 31 ts + CRC4 */
/*
 * 6 major ways that networks talk: Drivers enforce independent selection,
 * meaning, a driver will ensure that only one of these is set at a time.
 * Default is cisco hdlc mode with 32 bit CRC.
 */
#define IFM_TDM_HDLC_CRC16	0x0100	/* Use 16-bit CRC for HDLC instead */
#define IFM_TDM_PPP		0x0200	/* SPPP (dumb) */
#define IFM_TDM_FR_ANSI		0x0400	/* Frame Relay + LMI ANSI "Annex D" */
#define IFM_TDM_FR_CISCO	0x0800	/* Frame Relay + LMI Cisco */
#define IFM_TDM_FR_ITU		0x1000	/* Frame Relay + LMI ITU "Q933A" */

/* operating mode */
#define IFM_TDM_MASTER		0x0000000100000000ULL	/* aka clock source internal */

/*
 * Common Access Redundancy Protocol
 */
#define	IFM_CARP		0x0000000000000600ULL

/*
 * Shared media sub-types
 */
#define	IFM_AUTO	0ULL		/* Autoselect best media */
#define	IFM_MANUAL	1ULL		/* Jumper/dipswitch selects media */
#define	IFM_NONE	2ULL		/* Deselect all media */

/*
 * Shared options
 */
#define IFM_FDX		0x0000010000000000ULL	/* Force full duplex */
#define	IFM_HDX		0x0000020000000000ULL	/* Force half duplex */
#define	IFM_FLOW	0x0000040000000000ULL	/* enable hardware flow control */
#define IFM_FLAG0	0x0000100000000000ULL	/* Driver defined flag */
#define IFM_FLAG1	0x0000200000000000ULL	/* Driver defined flag */
#define IFM_FLAG2	0x0000400000000000ULL	/* Driver defined flag */
#define	IFM_LOOP	0x0000800000000000ULL	/* Put hardware in loopback */

/*
 * Masks
 */
#define	IFM_NMASK	0x000000000000ff00ULL	/* Network type */
#define	IFM_NSHIFT	8			/* Network type shift */
#define	IFM_TMASK	0x00000000000000ffULL	/* Media sub-type */
#define	IFM_TSHIFT	0			/* Sub-type shift */
#define	IFM_IMASK	0xff00000000000000ULL	/* Instance */
#define	IFM_ISHIFT	56			/* Instance shift */
#define	IFM_OMASK	0x00000000ffff0000ULL	/* Type specific options */
#define	IFM_OSHIFT	16			/* Specific options shift */
#define	IFM_MMASK	0x000000ff00000000ULL	/* Mode */
#define	IFM_MSHIFT	32			/* Mode shift */
#define	IFM_GMASK	0x00ffff0000000000ULL	/* Global options */
#define	IFM_GSHIFT	40			/* Global options shift */

/* Ethernet flow control mask */
#define	IFM_ETH_FMASK	(IFM_FLOW|IFM_ETH_RXPAUSE|IFM_ETH_TXPAUSE)

#define	IFM_NMIN	IFM_ETHER	/* lowest Network type */
#define	IFM_NMAX	IFM_NMASK	/* highest Network type */

/*
 * Status bits
 */
#define	IFM_AVALID	0x0000000000000001ULL	/* Active bit valid */
#define	IFM_ACTIVE	0x0000000000000002ULL	/* Interface attached to working net */

/* Mask of "status valid" bits, for ifconfig(8). */
#define	IFM_STATUS_VALID	IFM_AVALID

/* List of "status valid" bits, for ifconfig(8). */
#define	IFM_STATUS_VALID_LIST {						\
	IFM_AVALID,							\
	0								\
}

/*
 * Macros to extract various bits of information from the media word.
 */
#define	IFM_TYPE(x)	((x) & IFM_NMASK)
#define	IFM_SUBTYPE(x)	((x) & IFM_TMASK)
#define	IFM_INST(x)	(((x) & IFM_IMASK) >> IFM_ISHIFT)
#define	IFM_OPTIONS(x)	((x) & (IFM_OMASK|IFM_GMASK))
#define	IFM_MODE(x)	((x) & IFM_MMASK)

#define	IFM_INST_MAX	IFM_INST(IFM_IMASK)
#define	IFM_INST_ANY	((uint64_t) -1)

/*
 * Macro to create a media word.
 * All arguments are IFM_* macros, except 'instance' which is a 64-bit integer.
 * XXX 'operating mode' is not included here?!?
 */
#define	IFM_MAKEWORD(type, subtype, options, instance)			\
	((type) | (subtype) | (options) | \
	((uint64_t)(instance) << IFM_ISHIFT))

/*
 * NetBSD extension not defined in the BSDI API.  This is used in various
 * places to get the canonical description for a given type/subtype.
 *
 * In the subtype and mediaopt descriptions, the valid TYPE bits are OR'd
 * in to indicate which TYPE the subtype/option corresponds to.  If no
 * TYPE is present, it is a shared media/mediaopt.
 *
 * Note that these are parsed case-insensitive.
 *
 * Order is important.  The first matching entry is the canonical name
 * for a media type; subsequent matches are aliases.
 */
struct ifmedia_description {
	uint64_t	ifmt_word;	/* word value; may be masked */
	const char	*ifmt_string;	/* description */
};

#define	IFM_TYPE_DESCRIPTIONS {						\
	{ IFM_ETHER,			"Ethernet" },			\
	{ IFM_ETHER,			"ether" },			\
	{ IFM_FDDI,			"FDDI" },			\
	{ IFM_TDM,			"TDM" },			\
	{ IFM_CARP,			"CARP" },			\
	{ 0, NULL },							\
}

#define	IFM_TYPE_MATCH(dt, t)						\
	(IFM_TYPE((dt)) == 0 || IFM_TYPE((dt)) == IFM_TYPE((t)))

#define	IFM_SUBTYPE_DESCRIPTIONS {					\
	{ IFM_AUTO,			"autoselect" },			\
	{ IFM_AUTO,			"auto" },			\
	{ IFM_MANUAL,			"manual" },			\
	{ IFM_NONE,			"none" },			\
									\
	{ IFM_ETHER|IFM_10_T,		"10baseT" },			\
	{ IFM_ETHER|IFM_10_T,		"10baseT/UTP" },		\
	{ IFM_ETHER|IFM_10_T,		"UTP" },			\
	{ IFM_ETHER|IFM_10_T,		"10UTP" },			\
	{ IFM_ETHER|IFM_10_2,		"10base2" },			\
	{ IFM_ETHER|IFM_10_2,		"10base2/BNC" },		\
	{ IFM_ETHER|IFM_10_2,		"BNC" },			\
	{ IFM_ETHER|IFM_10_2,		"10BNC" },			\
	{ IFM_ETHER|IFM_10_5,		"10base5" },			\
	{ IFM_ETHER|IFM_10_5,		"10base5/AUI" },		\
	{ IFM_ETHER|IFM_10_5,		"AUI" },			\
	{ IFM_ETHER|IFM_10_5,		"10AUI" },			\
	{ IFM_ETHER|IFM_100_TX,		"100baseTX" },			\
	{ IFM_ETHER|IFM_100_TX,		"100TX" },			\
	{ IFM_ETHER|IFM_100_FX,		"100baseFX" },			\
	{ IFM_ETHER|IFM_100_FX,		"100FX" },			\
	{ IFM_ETHER|IFM_100_T4,		"100baseT4" },			\
	{ IFM_ETHER|IFM_100_T4,		"100T4" },			\
	{ IFM_ETHER|IFM_100_VG,		"100baseVG" },			\
	{ IFM_ETHER|IFM_100_VG,		"100VG" },			\
	{ IFM_ETHER|IFM_100_T2,		"100baseT2" },			\
	{ IFM_ETHER|IFM_100_T2,		"100T2" },			\
	{ IFM_ETHER|IFM_1000_SX,	"1000baseSX" },			\
	{ IFM_ETHER|IFM_1000_SX,	"1000SX" },			\
	{ IFM_ETHER|IFM_10_STP,		"10baseSTP" },			\
	{ IFM_ETHER|IFM_10_STP,		"STP" },			\
	{ IFM_ETHER|IFM_10_STP,		"10STP" },			\
	{ IFM_ETHER|IFM_10_FL,		"10baseFL" },			\
	{ IFM_ETHER|IFM_10_FL,		"FL" },				\
	{ IFM_ETHER|IFM_10_FL,		"10FL" },			\
	{ IFM_ETHER|IFM_1000_LX,	"1000baseLX" },			\
	{ IFM_ETHER|IFM_1000_LX,	"1000LX" },			\
	{ IFM_ETHER|IFM_1000_CX,	"1000baseCX" },			\
	{ IFM_ETHER|IFM_1000_CX,	"1000CX" },			\
	{ IFM_ETHER|IFM_1000_T,		"1000baseT" },			\
	{ IFM_ETHER|IFM_1000_T,		"1000T" },			\
	{ IFM_ETHER|IFM_1000_T,		"1000baseTX" },			\
	{ IFM_ETHER|IFM_1000_T,		"1000TX" },			\
	{ IFM_ETHER|IFM_HPNA_1,		"HomePNA1" },			\
	{ IFM_ETHER|IFM_HPNA_1,		"HPNA1" },			\
	{ IFM_ETHER|IFM_10G_LR,		"10GbaseLR" },			\
	{ IFM_ETHER|IFM_10G_LR,		"10GLR" },			\
	{ IFM_ETHER|IFM_10G_LR,		"10GBASE-LR" },			\
	{ IFM_ETHER|IFM_10G_SR,		"10GbaseSR" },			\
	{ IFM_ETHER|IFM_10G_SR,		"10GSR" },			\
	{ IFM_ETHER|IFM_10G_SR,		"10GBASE-SR" },			\
	{ IFM_ETHER|IFM_10G_CX4,	"10GbaseCX4" },			\
	{ IFM_ETHER|IFM_10G_CX4,	"10GCX4" },			\
	{ IFM_ETHER|IFM_10G_CX4,	"10GBASE-CX4" },		\
	{ IFM_ETHER|IFM_2500_SX,	"2500baseSX" },			\
	{ IFM_ETHER|IFM_2500_SX,	"2500SX" },			\
	{ IFM_ETHER|IFM_10G_T,		"10GbaseT" },			\
	{ IFM_ETHER|IFM_10G_T,		"10GT" },			\
	{ IFM_ETHER|IFM_10G_T,		"10GBASE-T" },			\
	{ IFM_ETHER|IFM_10G_SFP_CU,	"10GSFP+Cu" },			\
	{ IFM_ETHER|IFM_10G_SFP_CU,	"10GCu" },			\
	{ IFM_ETHER|IFM_10G_LRM,	"10GbaseLRM" },			\
	{ IFM_ETHER|IFM_10G_LRM,	"10GBASE-LRM" },		\
	{ IFM_ETHER|IFM_40G_CR4,	"40GbaseCR4" },			\
	{ IFM_ETHER|IFM_40G_CR4,	"40GBASE-CR4" },		\
	{ IFM_ETHER|IFM_40G_SR4,	"40GbaseSR4" },			\
	{ IFM_ETHER|IFM_40G_SR4,	"40GBASE-SR4" },		\
	{ IFM_ETHER|IFM_40G_LR4,	"40GbaseLR4" },			\
	{ IFM_ETHER|IFM_40G_LR4,	"40GBASE-LR4" },		\
	{ IFM_ETHER|IFM_1000_KX,	"1000base-KX" },		\
	{ IFM_ETHER|IFM_1000_KX,	"1000BASE-KX" },		\
	{ IFM_ETHER|IFM_10G_KX4,	"10GbaseKX4" },			\
	{ IFM_ETHER|IFM_10G_KX4,	"10GBASE-KX4" },		\
	{ IFM_ETHER|IFM_10G_KR,		"10GbaseKR" },			\
	{ IFM_ETHER|IFM_10G_KR,		"10GBASE-KR" },			\
	{ IFM_ETHER|IFM_10G_CR1,	"10GbaseCR1" },			\
	{ IFM_ETHER|IFM_10G_CR1,	"10GBASE-CR1" },		\
	{ IFM_ETHER|IFM_10G_AOC,	"10G-AOC" },			\
	{ IFM_ETHER|IFM_20G_KR2,	"20GbaseKR2" },			\
	{ IFM_ETHER|IFM_20G_KR2,	"20GBASE-KR2" },		\
	{ IFM_ETHER|IFM_2500_KX,	"2500baseKX" },			\
	{ IFM_ETHER|IFM_2500_KX,	"2500BASE-KX" },		\
	{ IFM_ETHER|IFM_2500_T,		"2500baseT" },			\
	{ IFM_ETHER|IFM_2500_T,		"2500BASE-T" },			\
	{ IFM_ETHER|IFM_5000_T,		"5000baseT" },			\
	{ IFM_ETHER|IFM_5000_T,		"5000BASE-T" },			\
	{ IFM_ETHER|IFM_1000_SGMII,	"1000base-SGMII" },		\
	{ IFM_ETHER|IFM_1000_SGMII,	"1000BASE-SGMII" },		\
	{ IFM_ETHER|IFM_10G_SFI,	"10GbaseSFI" },			\
	{ IFM_ETHER|IFM_10G_SFI,	"10GBASE-SFI" },		\
	{ IFM_ETHER|IFM_40G_XLPPI,	"40GbaseXLPPI" },		\
	{ IFM_ETHER|IFM_40G_XLPPI,	"40GBASE-XLPPI" },		\
	{ IFM_ETHER|IFM_1000_CX_SGMII,	"1000baseCX-SGMII" },		\
	{ IFM_ETHER|IFM_1000_CX_SGMII,	"1000BASE-CX-SGMII" },		\
	{ IFM_ETHER|IFM_40G_KR4,	"40GbaseKR4" },			\
	{ IFM_ETHER|IFM_40G_KR4,	"40GBASE-KR4" },		\
	{ IFM_ETHER|IFM_40G_AOC,	"40G-AOC" },			\
	{ IFM_ETHER|IFM_10G_ER,		"10GbaseER" },			\
	{ IFM_ETHER|IFM_10G_ER,		"10GBASE-ER" },			\
	{ IFM_ETHER|IFM_100G_CR4,	"100GbaseCR4" },		\
	{ IFM_ETHER|IFM_100G_CR4,	"100GBASE-CR4" },		\
	{ IFM_ETHER|IFM_100G_SR4,	"100GbaseSR4" },		\
	{ IFM_ETHER|IFM_100G_SR4,	"100GBASE-SR4" },		\
	{ IFM_ETHER|IFM_100G_KR4,	"100GbaseKR4" },		\
	{ IFM_ETHER|IFM_100G_KR4,	"100GBASE-KR4" },		\
	{ IFM_ETHER|IFM_100G_LR4,	"100GbaseLR4" },		\
	{ IFM_ETHER|IFM_100G_LR4,	"100GBASE-LR4" },		\
	{ IFM_ETHER|IFM_100G_AOC,	"100G-AOC" },			\
	{ IFM_ETHER|IFM_56G_R4,		"56GbaseR4" },			\
	{ IFM_ETHER|IFM_56G_R4,		"56GBASE-R4" },			\
	{ IFM_ETHER|IFM_25G_CR,		"25GbaseCR" },			\
	{ IFM_ETHER|IFM_25G_CR,		"25GBASE-CR" },			\
	{ IFM_ETHER|IFM_25G_KR,		"25GbaseKR" },			\
	{ IFM_ETHER|IFM_25G_KR,		"25GBASE-KR" },			\
	{ IFM_ETHER|IFM_25G_SR,		"25GbaseSR" },			\
	{ IFM_ETHER|IFM_25G_SR,		"25GBASE-SR" },			\
	{ IFM_ETHER|IFM_25G_LR,		"25GbaseLR" },			\
	{ IFM_ETHER|IFM_25G_LR,		"25GBASE-LR" },			\
	{ IFM_ETHER|IFM_25G_ER,		"25GbaseER" },			\
	{ IFM_ETHER|IFM_25G_ER,		"25GBASE-ER" },			\
	{ IFM_ETHER|IFM_25G_AOC,	"25G-AOC" },			\
	{ IFM_ETHER|IFM_50G_CR2,	"50GbaseCR2" },			\
	{ IFM_ETHER|IFM_50G_CR2,	"50GBASE-CR2" },		\
	{ IFM_ETHER|IFM_50G_KR2,	"50GbaseKR2" },			\
	{ IFM_ETHER|IFM_50G_KR2,	"50GBASE-KR2" },		\
									\
	{ IFM_FDDI|IFM_FDDI_SMF,	"Single-mode" },		\
	{ IFM_FDDI|IFM_FDDI_SMF,	"SMF" },			\
	{ IFM_FDDI|IFM_FDDI_MMF,	"Multi-mode" },			\
	{ IFM_FDDI|IFM_FDDI_MMF,	"MMF" },			\
	{ IFM_FDDI|IFM_FDDI_UTP,	"UTP" },			\
	{ IFM_FDDI|IFM_FDDI_UTP,	"CDDI" },			\
									\
	{ IFM_TDM|IFM_TDM_T1,		"t1" },				\
	{ IFM_TDM|IFM_TDM_T1_AMI,	"t1-ami" },			\
	{ IFM_TDM|IFM_TDM_E1,		"e1" },				\
	{ IFM_TDM|IFM_TDM_E1_G704,	"e1-g.704" },			\
	{ IFM_TDM|IFM_TDM_E1_AMI,	"e1-ami" },			\
	{ IFM_TDM|IFM_TDM_E1_AMI_G704,	"e1-ami-g.704" },		\
	{ IFM_TDM|IFM_TDM_T3,		"t3" },				\
	{ IFM_TDM|IFM_TDM_T3_M13,	"t3-m13" },			\
	{ IFM_TDM|IFM_TDM_E3,		"e3" },				\
	{ IFM_TDM|IFM_TDM_E3_G751,	"e3-g.751" },			\
	{ IFM_TDM|IFM_TDM_E3_G832,	"e3-g.832" },			\
	{ IFM_TDM|IFM_TDM_E1_G704_CRC4,	"e1-g.704-crc4" },		\
									\
	{ 0, NULL },							\
}

#define IFM_MODE_DESCRIPTIONS {						\
	{ IFM_AUTO,				"autoselect" },		\
	{ IFM_AUTO,				"auto" },		\
	{ IFM_TDM|IFM_TDM_MASTER,		"master" },		\
	{ 0, NULL },							\
}

#define	IFM_OPTION_DESCRIPTIONS {					\
	{ IFM_FDX,			"full-duplex" },		\
	{ IFM_FDX,			"fdx" },			\
	{ IFM_HDX,			"half-duplex" },		\
	{ IFM_HDX,			"hdx" },			\
	{ IFM_FLAG0,			"flag0" },			\
	{ IFM_FLAG1,			"flag1" },			\
	{ IFM_FLAG2,			"flag2" },			\
	{ IFM_LOOP,			"loopback" },			\
	{ IFM_LOOP,			"hw-loopback"},			\
	{ IFM_LOOP,			"loop" },			\
									\
	{ IFM_ETHER|IFM_ETH_MASTER,	"master" },			\
	{ IFM_ETHER|IFM_ETH_RXPAUSE,	"rxpause" },			\
	{ IFM_ETHER|IFM_ETH_TXPAUSE,	"txpause" },			\
									\
	{ IFM_FDDI|IFM_FDDI_DA,		"dual-attach" },		\
	{ IFM_FDDI|IFM_FDDI_DA,		"das" },			\
									\
	{ IFM_TDM|IFM_TDM_HDLC_CRC16,	"hdlc-crc16" },			\
	{ IFM_TDM|IFM_TDM_PPP,		"ppp" },			\
	{ IFM_TDM|IFM_TDM_FR_ANSI,	"framerelay-ansi" },		\
	{ IFM_TDM|IFM_TDM_FR_CISCO,	"framerelay-cisco" },		\
	{ IFM_TDM|IFM_TDM_FR_ANSI,	"framerelay-itu" },		\
									\
	{ 0, NULL },							\
}

/*
 * Baudrate descriptions for the various media types.
 */
struct ifmedia_baudrate {
	uint64_t	ifmb_word;		/* media word */
	uint64_t	ifmb_baudrate;		/* corresponding baudrate */
};

#define	IFM_BAUDRATE_DESCRIPTIONS {					\
	{ IFM_ETHER|IFM_10_T,		IF_Mbps(10) },			\
	{ IFM_ETHER|IFM_10_2,		IF_Mbps(10) },			\
	{ IFM_ETHER|IFM_10_5,		IF_Mbps(10) },			\
	{ IFM_ETHER|IFM_100_TX,		IF_Mbps(100) },			\
	{ IFM_ETHER|IFM_100_FX,		IF_Mbps(100) },			\
	{ IFM_ETHER|IFM_100_T4,		IF_Mbps(100) },			\
	{ IFM_ETHER|IFM_100_VG,		IF_Mbps(100) },			\
	{ IFM_ETHER|IFM_100_T2,		IF_Mbps(100) },			\
	{ IFM_ETHER|IFM_1000_SX,	IF_Mbps(1000) },		\
	{ IFM_ETHER|IFM_10_STP,		IF_Mbps(10) },			\
	{ IFM_ETHER|IFM_10_FL,		IF_Mbps(10) },			\
	{ IFM_ETHER|IFM_1000_LX,	IF_Mbps(1000) },		\
	{ IFM_ETHER|IFM_1000_CX,	IF_Mbps(1000) },		\
	{ IFM_ETHER|IFM_1000_T,		IF_Mbps(1000) },		\
	{ IFM_ETHER|IFM_HPNA_1,		IF_Mbps(1) },			\
	{ IFM_ETHER|IFM_10G_LR,		IF_Gbps(10) },			\
	{ IFM_ETHER|IFM_10G_SR,		IF_Gbps(10) },			\
	{ IFM_ETHER|IFM_10G_CX4,	IF_Gbps(10) },			\
	{ IFM_ETHER|IFM_2500_SX,	IF_Mbps(2500) },		\
	{ IFM_ETHER|IFM_10G_T,		IF_Gbps(10) },			\
	{ IFM_ETHER|IFM_10G_SFP_CU,	IF_Gbps(10) },			\
									\
	{ IFM_FDDI|IFM_FDDI_SMF,	IF_Mbps(100) },			\
	{ IFM_FDDI|IFM_FDDI_MMF,	IF_Mbps(100) },			\
	{ IFM_FDDI|IFM_FDDI_UTP,	IF_Mbps(100) },			\
									\
	{ IFM_TDM|IFM_TDM_T1,		IF_Kbps(1536) },		\
	{ IFM_TDM|IFM_TDM_T1_AMI,	IF_Kbps(1536) },		\
	{ IFM_TDM|IFM_TDM_E1,		IF_Kbps(2048) },		\
	{ IFM_TDM|IFM_TDM_E1_G704,	IF_Kbps(2048) },		\
	{ IFM_TDM|IFM_TDM_E1_AMI,	IF_Kbps(2048) },		\
	{ IFM_TDM|IFM_TDM_E1_AMI_G704,	IF_Kbps(2048) },		\
	{ IFM_TDM|IFM_TDM_T3,		IF_Kbps(44736) },		\
	{ IFM_TDM|IFM_TDM_T3_M13,	IF_Kbps(44736) },		\
	{ IFM_TDM|IFM_TDM_E3,		IF_Kbps(34368) },		\
	{ IFM_TDM|IFM_TDM_E3_G751,	IF_Kbps(34368) },		\
	{ IFM_TDM|IFM_TDM_E3_G832,	IF_Kbps(34368) },		\
	{ IFM_TDM|IFM_TDM_E1_G704_CRC4,	IF_Kbps(2048) },		\
									\
	{ 0, 0 },							\
}

/*
 * Status bit descriptions for the various media types.
 */
struct ifmedia_status_description {
	uint64_t	ifms_type;
	uint64_t	ifms_valid;
	uint64_t	ifms_bit;
	const char *ifms_string[2];
};

#define	IFM_STATUS_DESC(ifms, bit)					\
	(ifms)->ifms_string[((ifms)->ifms_bit & (bit)) ? 1 : 0]

#define	IFM_STATUS_DESCRIPTIONS {					\
	{ IFM_ETHER,		IFM_AVALID,	IFM_ACTIVE,		\
	    { "no carrier", "active" } },				\
	{ IFM_FDDI,		IFM_AVALID,	IFM_ACTIVE,		\
	    { "no ring", "inserted" } },				\
	{ IFM_TDM,		IFM_AVALID,	IFM_ACTIVE,		\
	    { "no carrier", "active" } },				\
	{ IFM_CARP,		IFM_AVALID,	IFM_ACTIVE,		\
	    { "backup", "master" } },					\
	{ 0,			0,		0,			\
	    { NULL, NULL } }						\
}
#endif	/* _NET_IF_MEDIA_H_ */
