#include <linux/module.h>
#include <asm/errno.h>

#include "drbcc.h"

// FIXME:
#ifndef BCC
#define BCC "[DRBCC_PACKET] "
#endif

/**
*	Fill byte into packet data array. If byte is a control character, escape it with escape byte first.
*	\param	pkt		packet data array
*	\param	b		byte to escape and fill in
*	\param 	index		integer at which position to fill byte in packet data array
*	\return	new index of next byte postition
* 	\warning index will be changed through the function
*/
static int bcc_esc_byte(unsigned char *cp, uint8_t b)
{
	int i = 0;
	if (DRBCC_START_CHAR == b || DRBCC_STOP_CHAR == b || DRBCC_ESC_CHAR == b)
	{
		*cp = DRBCC_ESC_CHAR; 
		cp++;
		b = ~b;
		i++;
	}
	*cp = b;
//	DBGF("escape Nr. %d: %x\n", index, b); 
	i++;
	return i;
}

/**
*	Ignore escape byte, if present, and fill data byte into packet data array at the given postion.
*	\param 	cp		pointer on packet buffer at current position	
*	\param	pkt_data	pointer to data buffer in the packet structure where to put byte
*	\return pointer to next position in packet buffer to parse
* 	\warning cp will be changed through the function
*/
static uint8_t bcc_unesc_byte(const unsigned char ** cp, unsigned char *pkt_data)
{
	if (DRBCC_ESC_CHAR == **cp)
        {
		(*cp)++;
//		DBGF("Escaping Byte: %c (%x)", *cp, *cp);
		*pkt_data = ~(**cp);
		(*cp)++;
		return 2;	// 2 Bytes read in buffer 
	} else {
		*pkt_data = **cp;
		(*cp)++;
		return 1;	// 1 Bytes read in buffer 
	}
}

/* TODO: steht das noch in nem header? */
/* Macros needed for CRC calculation */
#define lo8(x) ((x) & 0xff)
#define hi8(x) ((x) >> 8)

/* TODO: wenn die CRC nach einem RFC standard berechnet wird, ist es wahrscheinlich,
* dass es dafür bereits eine kernelfunktion gibt.
*/
/**
*	Update CRC value based on data byte passed to it
*	\param	crc	current CRC value
*	\param	data	byte in packet to base the CRC calculation on 
*	\return	newly calculated CRC value
*/
static uint16_t libdrbcc_crc_ccitt_update(uint16_t crc, uint8_t data)
{
        data ^= lo8 (crc);
        data ^= data << 4;

        return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4)
                ^ ((uint16_t)data << 3));
}

/* FIXME: Better return values needed. Currently: 
* 1 --> SUCCESS
* -EAGAIN --> Come back to parsing paket after more data in buffer arrived 
* < 0 --> FAILURE */
/* FIXME: do I really need the size parameter? */

/**
*	Fill passed struct based on the values parsed out of passed buffer.
*	\param 	cp	buffer containing message
*	\param	pkt	packet struct where to fill parsing result
*	\param	size	buffer size
*	\return payload length on success, negative error value otherwise
*/
int deserialize_packet(const unsigned char *cp, struct bcc_packet *pkt, int size) 
{
	uint16_t crc;
	uint8_t i;
	int t_size = size;
	int crc_postition;

	const unsigned char *p = cp;

	DBGF("%s (l. %d): pkt->curr_idx = %d, size = %d", __FUNCTION__, __LINE__, pkt->curr_idx, size);

start:
	if(pkt->curr_idx == 0) {
		if (*p == DRBCC_START_CHAR) {
			/* Start char, command, stop char */
			pkt->curr_idx = 1;
			DBG("Start char.");
			p++;
			t_size--;
		} else {
			return -EFAULT;
		}
	}

	if(!t_size)	
		return size;

	if(pkt->curr_idx == 1) {
		t_size -= bcc_unesc_byte(&p, &pkt->cmd);
		DBGF("Cmd: %c (%x)", pkt->cmd, pkt->cmd);
		pkt->curr_idx++;
	}
	
	if(!t_size)	
		return size;

/* 	Chars you will find in each message:
*	Two bytes (one start and one stop char), one char for command, two chars for the CRC 
* !ATTENTION!: The last two characters red are the CRC! 
*/
/* Here for the index holds: pkt->curr_idx >=2 */
	for(; t_size > 0 && pkt->curr_idx < MSG_MAX_LEN && *p != DRBCC_STOP_CHAR &&  *p != DRBCC_START_CHAR; pkt->curr_idx++) {
		t_size -= bcc_unesc_byte(&p, &pkt->data[pkt->curr_idx-2]);
		DBGF("Nr. %d: %x", pkt->curr_idx-2, pkt->data[pkt->curr_idx-2]);
	}

	if(*p == DRBCC_START_CHAR) {
		pkt->curr_idx = 0;
		goto start;
	}

	if (*p == DRBCC_STOP_CHAR) {
		t_size--;
/* 	Just from now on I know the length of the data in the packet.
* 	The start character is at index 0.
*	There are exactly MSG_MIN_LEN obligatory character in each message.
*/
		pkt->payloadlen = pkt->curr_idx-(MSG_MIN_LEN-1);	
		DBGF("%s Reached stop character. Struct filled (Payloadlen: %d).\n", BCC, pkt->payloadlen);
		/* The last two bytes in Buffer are the 2-Byte CRC value */
		crc_postition = pkt->payloadlen;	// data byte index: 0..payloadlen-1
		crc = 0xffff;
		crc = libdrbcc_crc_ccitt_update(crc, pkt->cmd);
		for (i = 0; i < crc_postition; i++){
			crc = libdrbcc_crc_ccitt_update(crc, pkt->data[i]);
		}

		pkt->crc = (((uint16_t)pkt->data[crc_postition]) | (((uint16_t)pkt->data[crc_postition+1]) << 8));

		if (crc == pkt->crc) {
/* Delete CRC from data buffer */
			pkt->data[crc_postition] = pkt->data[crc_postition+1] = '\0';
			DBGF("%s CRC of packet was right.\n", BCC);
			return size - t_size;
		} else {
			ERR("Received packet with wrong CRC: Expected CRC: 0x%x - CRC was: 0x%x", crc, pkt->crc);
#ifdef DEBUG
			PRINTKN(cp, size);
#endif
			return -EFAULT;
		}
	} else if(pkt->curr_idx >= MSG_MAX_LEN) {
		ERR("Message too long: %d", pkt->curr_idx);
		return -EFAULT;
	} else {
		DBGF("%s No stop character found at the end of the buffer. Trying again next time\n.", BCC);
#ifdef DEBUG
		PRINTKN(cp, size);
#endif
		return -EAGAIN;
	}
}
EXPORT_SYMBOL(deserialize_packet);

/**
*	Fill passed buffer based on the values in packet structure passed to function.
*	\param 	tx_buff	buffer to fill message in
*	\param	pkt	packet struct representing message
*	\return payload length on success, negative error value otherwise
*/
int serialize_packet(struct bcc_packet * pkt, unsigned char tx_buff[]) 
{
	int idx = 0;
	int j = 0;
	uint16_t crc = 0xffff;

	tx_buff[idx] = DRBCC_START_CHAR;
	idx++;

	idx += bcc_esc_byte(&tx_buff[idx], pkt->cmd);
        crc = libdrbcc_crc_ccitt_update(crc, pkt->cmd);

	/* FIXME: What if parameter is null?? */

	/* Notice: idx will be changed through the function bcc_esc_byte */
	/* FIXME: 140-5: Because there might be two escape seq for crc hi and crc lo */
	for(; idx < MSG_MAX_BUF-5 && j < pkt->payloadlen; j++) {
	                idx += bcc_esc_byte(&tx_buff[idx], pkt->data[j]);
                        crc = libdrbcc_crc_ccitt_update(crc, pkt->data[j]);
	}
	
	idx += bcc_esc_byte(&tx_buff[idx], crc & 0xff);
	idx += bcc_esc_byte(&tx_buff[idx], crc >> 8);

	tx_buff[idx] = DRBCC_STOP_CHAR;

	return idx+1;
}
EXPORT_SYMBOL(serialize_packet);
