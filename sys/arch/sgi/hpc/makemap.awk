#! /usr/bin/awk -f
#	$OpenBSD: makemap.awk,v 1.1 2014/05/22 19:39:37 miod Exp $
#
# Copyright (c) 2005, 2014, Miodrag Vallat
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# This script attempts to convert, with minimal hacks and losses, the
# regular PS/2 keyboard (pckbd) layout tables into SGI serial keyboard (zskbd)
# layout tables.
#

BEGIN {
	rcsid = "$OpenBSD: makemap.awk,v 1.1 2014/05/22 19:39:37 miod Exp $"
	ifdepth = 0
	ignore = 0
	haskeys = 0
	summary = 0

	# PS/2 -> SGI conversion table
	for (i = 0; i < 256; i++)
		conv[i] = -1

	conv[1] = 6
	conv[2] = 7
	conv[3] = 13
	conv[4] = 14
	conv[5] = 21
	conv[6] = 22
	conv[7] = 29
	conv[8] = 30
	conv[9] = 37
	conv[10] = 38
	conv[11] = 45
	conv[12] = 46
	conv[13] = 53
	conv[14] = 60
	conv[15] = 8
	conv[16] = 9
	conv[17] = 15
	conv[18] = 16
	conv[19] = 23
	conv[20] = 24
	conv[21] = 31
	conv[22] = 32
	conv[23] = 39
	conv[24] = 40
	conv[25] = 47
	conv[26] = 48
	conv[27] = 55
	conv[28] = 50
	conv[29] = 2
	conv[30] = 10
	conv[31] = 11
	conv[32] = 17
	conv[33] = 18
	conv[34] = 25
	conv[35] = 26
	conv[36] = 33
	conv[37] = 34
	conv[38] = 41
	conv[39] = 42
	conv[40] = 49
	conv[41] = 54
	conv[42] = 5
	conv[43] = 56
	conv[44] = 19
	conv[45] = 20
	conv[46] = 27
	conv[47] = 28
	conv[48] = 35
	conv[49] = 36
	conv[50] = 43
	conv[51] = 44
	conv[52] = 51
	conv[53] = 52
	conv[54] = 4
	conv[55] = 108
	conv[56] = 83
	conv[57] = 82
	conv[58] = 3
	conv[59] = 86
	conv[60] = 87
	conv[61] = 88
	conv[62] = 89
	conv[63] = 90
	conv[64] = 91
	conv[65] = 92
	conv[66] = 93
	conv[67] = 94
	conv[68] = 95
	conv[69] = 106
	conv[70] = 99
	conv[71] = 66
	conv[72] = 67
	conv[73] = 74
	conv[74] = 75
	conv[75] = 62
	conv[76] = 68
	conv[77] = 69
	conv[78] = 109
	conv[79] = 57
	conv[80] = 63
	conv[81] = 64
	conv[82] = 58
	conv[83] = 65
	conv[86] = 111
	conv[87] = 96
	conv[88] = 97
	conv[127] = 100
	conv[156] = 81
	conv[157] = 85
	conv[170] = 98
	conv[181] = 107
	conv[183] = 98
	conv[184] = 84
	conv[199] = 102
	conv[200] = 80
	conv[201] = 103
	conv[203] = 72
	conv[205] = 79
	conv[207] = 104
	conv[208] = 73
	conv[209] = 105
	conv[210] = 101
	conv[211] = 61
}
NR == 1 {
	VERSION = $0
	gsub("\\$", "", VERSION)
	gsub("\\$", "", rcsid)

	printf("/*\t\$OpenBSD\$\t*/\n\n")
	printf("/*\n")
	printf(" * THIS FILE IS AUTOMAGICALLY GENERATED.  DO NOT EDIT.\n")
	printf(" *\n")
	printf(" * generated by:\n")
	printf(" *\t%s\n", rcsid)
	printf(" * generated from:\n")
	printf(" */\n")
	print VERSION

	next
}

#
# A very limited #if ... #endif parser. We only want to correctly detect
# ``#if 0'' constructs, so as not to process their contents. This is necessary
# since our output is out-of-order from our input.
#
# Note that this does NOT handle ``#ifdef notyet'' correctly - please only use
# ``#if 0'' constructs in the input.
#

/^#if/ {
	ignores[ifdepth] = ignore
	if ($2 == "0")
		ignore = 1
	#else
	#	ignore = 0
	ifdepth++
	if (ignore)
		next
}
/^#endif/ {
	oldignore = ignore
	ifdepth--
	ignore = ignores[ifdepth]
	ignores[ifdepth] = 0
	if (oldignore)
		next
}

$1 == "#include" {
	if (ignore)
		next
	if ($2 == "<dev/pckbc/wskbdmap_mfii.h>")
		next
	printf("#include %s\n", $2)
	next
}
$1 == "#define" || $1 == "#undef" {
	if (ignore)
		next
	print $0
	next
}

/pckbd/ {
	gsub("pckbd", "zskbd", $0)
}
/zskbd_keydesctab/ {
	gsub("zskbd", "wssgi", $0)
}

/zskbd_keydesc_/ {
	mapname = substr($0, index($0, "zskbd_keydesc_") + length("zskbd_keydesc_"))
	sub("\\[\\].*", "", mapname)
	sub("\\).*", "", mapname)
	shortname = mapname
	sub("_nodead", "", shortname)	# _nodead ok if main layout ok
	if (shortname != "be" &&
	    shortname != "de" &&
	    shortname != "dk" &&
	    shortname != "es" &&
	    shortname != "fi" &&	# missing from PS/2 source...
	    shortname != "fr" &&
	    shortname != "gr" &&	# missing from PS/2 source...
	    shortname != "it" &&
	    shortname != "nl" &&
	    shortname != "no" &&
	    shortname != "pt" &&
	    shortname != "sf" &&
	    shortname != "sg" &&
	    shortname != "sv" &&
	    shortname != "uk" &&
	    shortname != "us") {
		nolayout = 1
	}
}

/zskbd_keydesc_.*\[\]/ {
	if (nolayout)
		printf("/* %s not applicable */\n", mapname)
}

/KC/ {
	if (ignore)
		next

	if (nolayout)
		next

	haskeys = 1

	sidx = substr($1, 4, length($1) - 5)
	orig = int(sidx)
	id = conv[orig]

	if (id == -1) {
		# printf("/* initially KC(%d),", orig)
		# for (f = 2; f <= NF; f++) {
		# 	if ($f != "/*" && $f != "*/")
		# 		printf("\t%s", $f)
		# }
		# printf("\t*/\n")
	} else {
		lines[id] = sprintf("    KC(%d),\t", id)
		#
		# This makes sure that the non-comment part of the output
		# ends up with a trailing comma. This is necessary since
		# the last line of an input block might not have a trailing
		# comma, but might not be the last line of an output block
		# due to sorting.
		#
		comma = 0
		for (f = 2; f <= NF; f++) {
			l = length($f)
			if ($f == "/*")
				comma++
			if (comma == 0 && substr($f, l) != ",") {
				lines[id] = sprintf("%s%s,", lines[id], $f)
				l++
			} else {
				lines[id] = sprintf("%s%s", lines[id], $f)
			}
			if (comma == 0 && f != NF) {
				if (l < 2 * 8)
					lines[id] = lines[id] "\t"
				if (l < 8)
					lines[id] = lines[id] "\t"
			}
			if ($f == "*/")
				comma--
		}
	}

	next
}
/};/ {
	if (ignore)
		next

	if (nolayout) {
		nolayout = 0
		next
	}

	if (haskeys) {
		for (i = 0; i < 256; i++)
			if (lines[i]) {
				print lines[i]
				lines[i] = ""
			}

		haskeys = 0
	}
}
/KBD_MAP/ {
	summary = 1
}
# hack to eat two line KBD_MAP() - we want to ignore all of them, and
# the second line will get rejected because ``nolayout'' will be set.
/KBD_MAP[^)]*,$/ { next }

{
	if (ignore)
		next
	if (nolayout) {
		if (summary)
			nolayout = 0
		next
	}
	print $0
}