/** \file 	drbcc_core.h	
*   \brief	hidden header file with definitions for drbcc core
*   \author 	Christina Quast
*
* (C) 2013 DResearch Digital Media Systems GmbH
*
*/
#ifndef __DRBCC_CORE_H__
#define __DRBCC_CORE_H__

#include "drbcc.h"

struct parse_work {
	unsigned char buf[MSG_MAX_BUF];
	int cnt;

	struct work_struct work;
};

#endif	/* __DRBCC_CORE_H__ */
