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

/* Timeout in jiffies; here: ?00ms */
#define BCC_PKT_TIMEOUT			HZ/2

#include "drbcc_core.h"

#define BCC_MAGIC			22
#define BCC_TTY_MAJOR		123

#define DEBUG_DRBCC
#define DEBUG

#define RESEND_SYNC_THRESHOLD	2	
#define RESEND_THRESHOLD	2	
#define RX_CURR_BUF_CNT		2

#define DRIVER_THROTTLING

#ifndef BCC
#define BCC "[DRBCC_CORE] "
#endif


typedef enum {
	RQ_STD, RQ_STD_ANS, RQ_WAIT_ANS, RQ_SYNC, NONE
} GLOBAL_L2_STATES;

static char async_cmd[]  = { DRBCC_IND_STATUS, DRBCC_IND_ACCEL_EVENT };
static char l2_state = NONE;

struct parse_work {
	unsigned char buf[MSG_MAX_BUF];
	int cnt;

	struct work_struct work;
};

/* Mapping request to response message type */
/* DRBCC_CMD_ILLEGAL means in our context that we don't expect a response to the message */
const uint8_t cmd_responses[] = {
	[DRBCC_ACK]				= DRBCC_CMD_ILLEGAL,			/* 0 */
	[DRBCC_SYNC]				= DRBCC_CMD_ILLEGAL,			/* 1 */
	[DRBCC_CMD_REQ_PROTOCOL_VERSION] 	= DRBCC_CMD_IND_PROTOCOL_VERSION,	/* 3 */
	[DRBCC_REQ_RTC_READ]			= DRBCC_IND_RTC_READ, 			/* 5 */
	[DRBCC_REQ_RTC_SET]			= DRBCC_IND_RTC_READ,			/* 7 */
	[DRBCC_REQ_EXTFLASH_ID]			= DRBCC_IND_EXTFLASH_ID,		/* 8 */
	[DRBCC_REQ_EXTFLASH_READ]		= DRBCC_IND_EXTFLASH_READ,		/* 10 */
	[DRBCC_REQ_EXTFLASH_WRITE]		= DRBCC_IND_EXTFLASH_WRITE_RESULT,	/* 12 */
	[DRBCC_REQ_EXTFLASH_BLOCKERASE]		= DRBCC_IND_EXTFLASH_BLOCKERASE_RESULT,	/* 14 */
	[DRBCC_REQ_FW_INVALIDATE]		= DRBCC_IND_FW_INVALIDATED,		/* 16 */
	[DRBCC_REQ_BCTRL_RESTART]		= DRBCC_IND_BCTRL_RESTART_ACCEPTED,	/* 18 */
	[DRBCC_REQ_SET_LED]			= DRBCC_CMD_ILLEGAL,			/* 20 */
	[DRBCC_REQ_BL_UPDATE]			= DRBCC_IND_BL_UPDATE,			/* 22 */
	[DRBCC_REQ_HEARTBEAT]			= DRBCC_CMD_ILLEGAL,			/* 24 */ 
	[DRBCC_REQ_STATUS]			= DRBCC_IND_STATUS,			/* 25 */
	[DRBCC_REQ_HD_EJECT]			= DRBCC_CMD_ILLEGAL,			/* 27 */
	[DRBCC_REQ_HD_ONOFF]			= DRBCC_CMD_ILLEGAL,			/* 28 */
	[DRBCC_REQ_GPI_POWER]			= DRBCC_CMD_ILLEGAL,			/* 29 */
	[DRBCC_REQ_PUT_LOG]			= DRBCC_IND_PUT_LOG,			/* 30 */
	[DRBCC_REQ_RINGLOG_POS]			= DRBCC_IND_RINGLOG_POS,		/* 32 */
	[DRBCC_REQ_AES_TEST]			= DRBCC_IND_AES_TEST,			/* 34 */
	[DRBCC_REQ_SET_GPO]			= DRBCC_CMD_ILLEGAL,			/* 36 */
	[DRBCC_REQ_SHUTDOWN]			= DRBCC_CMD_ILLEGAL, 			/* 37 */
	[DRBCC_REQ_ID_DATA]			= DRBCC_IND_ID_DATA,			/* 38 */
	[DRBCC_REQ_MEMDEV_READ]			= DRBCC_IND_MEMDEV_READ,		/* 40 */
	[DRBCC_REQ_MEMDEV_WRITE]		= DRBCC_IND_MEMDEV_WRITE_RESULT,	/* 42 */
	[DRBCC_CLEAR_RINGLOG_REQ]		= DRBCC_IND_RINGLOG_POS,		/* 45 */
	[DRBCC_OXE_UART_BOOT_CALCCRC_IND]	= DRBCC_OXE_UART_BOOT_CALCCRC_REQ,	/* 47 */
	[DRBCC_OXE_UART_BOOT_RUN_REQ]		= DRBCC_CMD_ILLEGAL,			/* 48 */
	[DRBCC_OXE_BOOT_MODE_SELECT_REQ]	= DRBCC_CMD_ILLEGAL,			/* 49 */
	[DRBCC_HDD_OFF_REQ]			= DRBCC_CMD_ILLEGAL,			/* 50 */
	[DRBCC_REQ_DEBUG_SET]			= DRBCC_CMD_ILLEGAL,			/* 51 */ 
	[DRBCC_REQ_DEBUG_GET]			= DRBCC_IND_DEBUG_GET,			/* 52 */
	[DRBCC_SYNC_CMD_ERROR]			= DRBCC_CMD_ILLEGAL,			/* 127 */ /* Or what should be done here? */
 

	[54 ... 126] 				= DRBCC_CMD_ILLEGAL,
};

/*
* Layer 2 functions:
* Transactionbased: Request-response-logic
* Keeps track of the toggle bits
*/
static int perform_transaction(struct bcc_packet *pkt);
//static int perform_transaction_ans(void);

/*
* Layer 1 functions:
* Parsing: char[] <==> struct bcc_struct
*/
static int synchronize(void);
static int transmit_msg(struct bcc_packet *pkt);
static void transmit_ack(void); 
static void receive_msg(unsigned char *buf, uint8_t len);


#endif	/* __DRBCC_CORE_H__ */
