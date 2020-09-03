/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "mtk_cpufreq_struct.h"
#include "mtk_cpufreq_config.h"

#if defined (CONFIG_CPU_UNDERVOLT)
#define CPU_DVFS_FREQ0_LL_FY	910000
#define CPU_DVFS_FREQ1_LL_FY	910000
#define CPU_DVFS_FREQ2_LL_FY	910000
#define CPU_DVFS_FREQ3_LL_FY	910000
#define CPU_DVFS_FREQ4_LL_FY	910000
#define CPU_DVFS_FREQ5_LL_FY	910000
#define CPU_DVFS_FREQ6_LL_FY	910000
#define CPU_DVFS_FREQ7_LL_FY	910000
#define CPU_DVFS_FREQ8_LL_FY	910000
#define CPU_DVFS_FREQ9_LL_FY	845000
#define CPU_DVFS_FREQ10_LL_FY	702000
#define CPU_DVFS_FREQ11_LL_FY	624000
#define CPU_DVFS_FREQ12_LL_FY	546000
#define CPU_DVFS_FREQ13_LL_FY	416000
#define CPU_DVFS_FREQ14_LL_FY	338000
#define CPU_DVFS_FREQ15_LL_FY	299000

#define CPUFREQ_BOUNDARY_FOR_FHCTL (CPU_DVFS_FREQ8_LL_FY)

#define CPU_DVFS_VOLT0_VPROC1_FY	100000
#define CPU_DVFS_VOLT1_VPROC1_FY	100000
#define CPU_DVFS_VOLT2_VPROC1_FY	100000
#define CPU_DVFS_VOLT3_VPROC1_FY	100000
#define CPU_DVFS_VOLT4_VPROC1_FY	100000
#define CPU_DVFS_VOLT5_VPROC1_FY	100000
#define CPU_DVFS_VOLT6_VPROC1_FY	100000
#define CPU_DVFS_VOLT7_VPROC1_FY	100000
#define CPU_DVFS_VOLT8_VPROC1_FY	100000
#define CPU_DVFS_VOLT9_VPROC1_FY	100000
#define CPU_DVFS_VOLT10_VPROC1_FY	100000
#define CPU_DVFS_VOLT11_VPROC1_FY	100000
#define CPU_DVFS_VOLT12_VPROC1_FY	97500
#define CPU_DVFS_VOLT13_VPROC1_FY	95000
#define CPU_DVFS_VOLT14_VPROC1_FY	95000
#define CPU_DVFS_VOLT15_VPROC1_FY	92500

#else // Default CPUFREQ
#define CPU_DVFS_FREQ0_LL_FY	1001000
#define CPU_DVFS_FREQ1_LL_FY	1001000
#define CPU_DVFS_FREQ2_LL_FY	1001000
#define CPU_DVFS_FREQ3_LL_FY	1001000
#define CPU_DVFS_FREQ4_LL_FY	1001000
#define CPU_DVFS_FREQ5_LL_FY	1001000
#define CPU_DVFS_FREQ6_LL_FY	1001000
#define CPU_DVFS_FREQ7_LL_FY	962000
#define CPU_DVFS_FREQ8_LL_FY	910000
#define CPU_DVFS_FREQ9_LL_FY	845000
#define CPU_DVFS_FREQ10_LL_FY	702000
#define CPU_DVFS_FREQ11_LL_FY	624000
#define CPU_DVFS_FREQ12_LL_FY	546000
#define CPU_DVFS_FREQ13_LL_FY	416000
#define CPU_DVFS_FREQ14_LL_FY	338000
#define CPU_DVFS_FREQ15_LL_FY	299000

#define CPUFREQ_BOUNDARY_FOR_FHCTL (CPU_DVFS_FREQ6_LL_FY)

#define CPU_DVFS_VOLT0_VPROC1_FY	113750
#define CPU_DVFS_VOLT1_VPROC1_FY	113750
#define CPU_DVFS_VOLT2_VPROC1_FY	113750
#define CPU_DVFS_VOLT3_VPROC1_FY	113750
#define CPU_DVFS_VOLT4_VPROC1_FY	113750
#define CPU_DVFS_VOLT5_VPROC1_FY	113750
#define CPU_DVFS_VOLT6_VPROC1_FY	113750
#define CPU_DVFS_VOLT7_VPROC1_FY	111875
#define CPU_DVFS_VOLT8_VPROC1_FY	107500
#define CPU_DVFS_VOLT9_VPROC1_FY	105000
#define CPU_DVFS_VOLT10_VPROC1_FY	102500
#define CPU_DVFS_VOLT11_VPROC1_FY	100000
#define CPU_DVFS_VOLT12_VPROC1_FY	97500
#define CPU_DVFS_VOLT13_VPROC1_FY	95000
#define CPU_DVFS_VOLT14_VPROC1_FY	95000
#define CPU_DVFS_VOLT15_VPROC1_FY	92500
#endif

/* DVFS OPP table */
#define OPP_TBL(cluster, seg, lv, vol)	\
static struct mt_cpu_freq_info opp_tbl_##cluster##_e##lv##_0[] = {	\
	OP(CPU_DVFS_FREQ0_##cluster##_##seg, CPU_DVFS_VOLT0_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ1_##cluster##_##seg, CPU_DVFS_VOLT1_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ2_##cluster##_##seg, CPU_DVFS_VOLT2_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ3_##cluster##_##seg, CPU_DVFS_VOLT3_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ4_##cluster##_##seg, CPU_DVFS_VOLT4_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ5_##cluster##_##seg, CPU_DVFS_VOLT5_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ6_##cluster##_##seg, CPU_DVFS_VOLT6_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ7_##cluster##_##seg, CPU_DVFS_VOLT7_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ8_##cluster##_##seg, CPU_DVFS_VOLT8_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ9_##cluster##_##seg, CPU_DVFS_VOLT9_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ10_##cluster##_##seg, CPU_DVFS_VOLT10_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ11_##cluster##_##seg, CPU_DVFS_VOLT11_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ12_##cluster##_##seg, CPU_DVFS_VOLT12_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ13_##cluster##_##seg, CPU_DVFS_VOLT13_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ14_##cluster##_##seg, CPU_DVFS_VOLT14_VPROC##vol##_##seg),	\
	OP(CPU_DVFS_FREQ15_##cluster##_##seg, CPU_DVFS_VOLT15_VPROC##vol##_##seg),	\
}

OPP_TBL(LL, FY, 0, 1);

struct opp_tbl_info opp_tbls[NR_MT_CPU_DVFS][NUM_CPU_LEVEL] = {		/* v1.1 */
	/* LL */
	{
		[CPU_LEVEL_0] = { opp_tbl_LL_e0_0, ARRAY_SIZE(opp_tbl_LL_e0_0) },
	},
};

/* 16 steps OPP table */
static struct mt_cpu_freq_method opp_tbl_method_LL_e0[] = {	/* FY */
	/* POS,	CLK */
	FP(1,	1),
	FP(1,	1),
	FP(1,	1),
	FP(1,	1),
	FP(1,	1),
	FP(1,	1),
	FP(1,	1),
	FP(2,	1),
	FP(2,	1),
	FP(2,	1),
	FP(2,	1),
	FP(2,	1),
	FP(2,	1),
	FP(2,	2),
	FP(2,	2),
	FP(2,	2),
};

struct opp_tbl_m_info opp_tbls_m[NR_MT_CPU_DVFS][NUM_CPU_LEVEL] = {	/* v1.1 */
	/* LL */
	{
		[CPU_LEVEL_0] = { opp_tbl_method_LL_e0 },
	},
};
