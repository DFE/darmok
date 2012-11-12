/** \file 	drbcc.h	
*   \brief	header file with definitions for drbcc device driver functionality
*   \author 	Christina Quast
*
*	Includes packet structure for board controller message represenation,
*	request-response-mapping, toggle bit state machine, various time and 
*	length defines.
*
* (C) 2009 DResearch Digital Media Systems GmbH
*
*/
#ifndef __DRBCC_H__
#define __DRBCC_H__

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/cdev.h>
#include <asm/semaphore.h>
#include <linux/workqueue.h>
#endif /* __KERNEL__ */

#include "debug.h"
#include "drbcc_ll.h"

#define MSG_MAX_LEN 	140
#define MSG_MAX_BUFF 	2*MSG_MAX_LEN
#define MSG_MIN_LEN	5
#define SYNC_LEN 	MSG_MIN_LEN
#define ACK_LEN 	MSG_MIN_LEN

/* FIXME: Delete this define? */
#define BCC_TTY_BUFF_SIZE	4096	
#define BCC_TTY_FLIPBUF_SIZE	TTY_FLIPBUF_SIZE
#define BCC_PKT_MIN_SIZE	5 	/* <DRBCC_START_CHAR> <DRBCC_CMD> [<DATA:1> .. <DATA:n>] <CRC_LO> <CRC_HI> <DRBCC_STOP_CHAR> */

// FIXME: will man wirklich auf pointer in einer struct in einem macro
// zugreifen?
#define CMD_DEL_TBIT(x) (x->cmd = (x->cmd & ~TOGGLE_BITMASK)) // remove toggle bit from cmd
// #define TOGGLEB(x) (x->cmd = (x->cmd & ~TOGGLE_BITMASK))
#define SHIFT_TBIT(toggle) (toggle << 7)	// for &ing with cmd
// #define TOGGLE_SHIFT(toggle) (toggle << 7)
// #define TOGGLE TOGGLE_SHIFT(toggle)
// #define TOGGLE_BIT(toggle) (toggle += 1)
#define CMD_NO_TBIT(x) (x & ~SHIFT_TBIT(1))	// can be 0x00 or 0x80
// #define T(x)	(x & ~TOGGLE_SHIFT(1))
#define SET_TBIT(pkt, toggle) (pkt->cmd = ((pkt->cmd & ~TOGGLE_BITMASK) | SHIFT_TBIT(toggle)) ) 
#define CMD_TBIT(cmd) (cmd & TOGGLE_BITMASK)

/**
*  \struct	toggle
*  \brief	state machine for one bit sequence number in both directions (receive and transmit)
*/
struct toggle {
	uint8_t rx:1;	/**< current toggle bit of packets to be received */
	uint8_t tx:1;	/**< current toggle bit of packets to be transmitted */
};

/* Mapping request to response message type */
/* DRBCC_CMD_ILLEGAL means in our context that we don't expect a response to the message */
/* TODO: Implement the missing messages */
// static const DRBCC_COMMANDS_t cmd_responses[] = {
/* TODO: struktur verstecken!*/
static const uint8_t cmd_responses[] = {
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
	[DRBCC_IND_FW_UPDATE_STARTED]		= DRBCC_CMD_ILLEGAL,			/* 21 */
	[DRBCC_REQ_BL_UPDATE]			= DRBCC_IND_BL_UPDATE,			/* 22 */
	[DRBCC_REQ_HEARTBEAT]			= DRBCC_CMD_ILLEGAL,			/* 24 */ /* TODO: really right answer or is this message send by the booard controller? */
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
	[DRBCC_IND_KEY_PROCESSING]		= DRBCC_CMD_ILLEGAL,			/* 44 */ /* TODO: really right answer or is this message send by the booard controller? */
	[DRBCC_CLEAR_RINGLOG_REQ]		= DRBCC_IND_RINGLOG_POS,		/* 45 */
	[DRBCC_OXE_UART_BOOT_CALCCRC_IND]	= DRBCC_OXE_UART_BOOT_CALCCRC_REQ,	/* 47 */
	[DRBCC_OXE_UART_BOOT_RUN_REQ]		= DRBCC_CMD_ILLEGAL,			/* 48 */
	[DRBCC_OXE_BOOT_MODE_SELECT_REQ]	= DRBCC_CMD_ILLEGAL,			/* 49 */
	[DRBCC_HDD_OFF_REQ]			= DRBCC_CMD_ILLEGAL,			/* 50 */
	[DRBCC_REQ_DEBUG_SET]			= DRBCC_CMD_ILLEGAL,			/* 51 */ 
	[DRBCC_REQ_DEBUG_GET]			= DRBCC_IND_DEBUG_GET,			/* 52 */
	[DRBCC_SYNC_CMD_ERROR]			= DRBCC_CMD_ILLEGAL,			/* 127 */ /* Or what should be done here? */
	[DRBCC_CMD_FORWARD]			= DRBCC_CMD_ILLEGAL,			/* No number */ /* Or what should be done here? */

 

	[54 ... 126] 				= DRBCC_CMD_ILLEGAL,
};

#define DRBCC_TIMEOUT		126

#define RSP_CMD(cmd)		(cmd_responses[(cmd & ~TOGGLE_BITMASK)])
/* How-To on using the enum above us
* DRBCC_COMMANDS_t cmd 	   = DRBCC_OXE_BOOT_MODE_SELECT_REQ,
*		 response  = cmd_responses[cmd];
*/


struct drbcc_driver {
	/* TODO to be implemented */
};

/**
*  \struct	bcc_packet
*  \brief	board controller message struct
*  \note	data buffer should initially be filled with zeors
*/
/* FIXME: integrate pkt_len into struct, because it's safer than checking for NULL reference */
struct bcc_packet {
	uint8_t cmd;						/**< message type */
	/* Substracted start and stop byte, command, two bytes for the CRC */
	unsigned char data[MSG_MAX_LEN-MSG_MIN_LEN];		/**< data array of packet */
	uint8_t payloadlen;					/**< number of data bytes in data array */

        /* Used in serialize and deserialize functions */
	uint8_t curr_idx;					/**< current position pointer in the process of parsing  */
	uint16_t crc; /* TODO: default should be 0*/

	struct semaphore 	*sem;
	struct work_struct	work;
};

int serialize_packet(struct bcc_packet * pkt, unsigned char tx_buff[MSG_MAX_BUFF]);
int deserialize_packet(const unsigned char *cp, struct bcc_packet *pkt, int size);

unsigned char *create_ack_buf(uint8_t toggle, unsigned char *tx_buff);
unsigned char *create_sync_buf(unsigned char *tx_buff);

int transmit_packet(struct bcc_packet *pkt,  uint8_t resp_cmd);
int register_async_callback(void *func_ptr);

int add_device_entry(struct cdev *cdev, int minor, char *dev_name);
void remove_device_entry(int minor);

#endif /* __DRBCC_H__ */
