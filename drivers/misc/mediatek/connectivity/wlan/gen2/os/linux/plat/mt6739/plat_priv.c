/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include "mach/mtk_ppm_api.h"
#include "mtk_spm_resource_req.h"
#include "mtk_vcorefs_manager.h"

#define MAX_CPU_FREQ     23400000

int kalBoostCpu(unsigned int level)
{
	unsigned long freq = MAX_CPU_FREQ;

	if (level >= 1) {
		spm_resource_req(SPM_RESOURCE_USER_CONN, SPM_RESOURCE_ALL);  /* Disable deepidle/SODI */
		if (level >= 7) {
			/* Setting ddr to 1333 */
			if (vcorefs_request_dvfs_opp(KIR_WIFI, OPP_0) != 0)
				pr_info("vcorefs_request_dvfs_opp@KIR_WIFI,OPP_0 fail!\n");
		}
	} else {
		spm_resource_req(SPM_RESOURCE_USER_CONN, 0);/* Enable deepidle/SODI */
		/* Setting ddr to default */
		if (vcorefs_request_dvfs_opp(KIR_WIFI, OPP_UNREQ) != 0)
			pr_info("vcorefs_request_dvfs_opp@KIR_WIFI,OPP_UNREQ fail!\n");
	}

	freq = level == 0 ? 0 : freq;

	mt_ppm_sysboost_freq(BOOST_BY_WIFI, freq);

	return 0;
}


