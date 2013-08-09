/** \file 	drbcc.h	
 *  \brief	header file with definitions for drbcc device driver functionality
 *  \author 	Christina Quast
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
#include <linux/workqueue.h>
#endif /* __KERNEL__ */

#include "debug.h"
#include "drbcc_ll.h"

#define MSG_MAX_LEN 	140
#define MSG_MAX_BUF 	2*MSG_MAX_LEN
#define MSG_MIN_LEN	5
#define SYNC_LEN 	MSG_MIN_LEN
#define ACK_LEN 	MSG_MIN_LEN
#define DRBCC_TIMEOUT		126

#define ACK_BUF_RX_0 (const char[]){0xfa, 0x00, 0x87, 0x0f, 0xfb}
#define ACK_BUF_RX_1 (const char[]){0xfa, 0x80, 0x8f, 0x8b, 0xfb}

/* FIXME: Delete this define? */
#define BCC_TTY_BUFF_SIZE	4096	
#define BCC_TTY_FLIPBUF_SIZE	TTY_FLIPBUF_SIZE
#define BCC_PKT_MIN_SIZE	5 	/* <DRBCC_START_CHAR> <DRBCC_CMD> [<DATA:1> .. <DATA:n>] <CRC_LO> <CRC_HI> <DRBCC_STOP_CHAR> */

// FIXME: will man wirklich auf pointer in einer struct in einem macro
// zugreifen?
#define CMD_DEL_TBIT(x) (x->cmd = (x->cmd & ~TOGGLE_BITMASK)) // remove toggle bit from cmd
#define SHIFT_TBIT(toggle) (toggle << 7)	// for &ing with cmd
#define CMD_WITHOUT_TBIT(x) (x & ~SHIFT_TBIT(1))	// can be 0x00 or 0x80
#define SET_TBIT(pkt, toggle) (pkt->cmd = ((pkt->cmd & ~TOGGLE_BITMASK) | SHIFT_TBIT(toggle)) ) 
#define TBIT_OF_CMD(cmd) (cmd & TOGGLE_BITMASK)

/**
 *  \struct	toggle
 *  \brief	state machine for one bit sequence number in both directions (receive and transmit)
 */
struct toggle {
	uint8_t rx;	/**< current toggle bit of packets to be received */
	uint8_t tx;	/**< current toggle bit of packets to be transmitted */
};

extern const uint8_t cmd_responses[];
#define RSP_CMD(cmd)		(cmd_responses[(cmd & ~TOGGLE_BITMASK)])
/* How-To on using the enum above us
 * DRBCC_COMMANDS_t cmd 	   = DRBCC_OXE_BOOT_MODE_SELECT_REQ,
 *		 response  = cmd_responses[cmd];
 */


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

	struct work_struct	work;
};

int serialize_packet(struct bcc_packet * pkt, unsigned char tx_buff[MSG_MAX_BUF]);
int deserialize_packet(const unsigned char *cp, struct bcc_packet *pkt, int size);

int transmit_packet(struct bcc_packet *pkt);
int register_async_callback(void *func_ptr);

int add_device_entry(struct cdev *cdev, int minor, char *dev_name);
void remove_device_entry(int minor);

#endif /* __DRBCC_H__ */
