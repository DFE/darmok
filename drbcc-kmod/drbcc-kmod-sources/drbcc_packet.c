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
int bcc_esc_byte(unsigned char pkt[], uint8_t b, int index)
{
        if (DRBCC_START_CHAR == b || DRBCC_STOP_CHAR == b || DRBCC_ESC_CHAR == b)
        {
                pkt[index] = DRBCC_ESC_CHAR; 
		b = ~b;
		index++;
        }
        pkt[index] = b;
//	DBGF("escape Nr. %d: %x\n", index, b); 
	index++;
	return index;
}

/**
*	Ignore escape byte, if present, and fill data byte into packet data array at the given postion.
*	\param 	cp		pointer on packet buffer at current position	
*	\param	pkt_data	pointer to data buffer in the packet structure where to put byte
*	\return pointer to next position in packet buffer to parse
* 	\warning cp will be changed through the function
*/
const unsigned char* bcc_unesc_byte(const unsigned char * cp, unsigned char *pkt_data)
{
	if (DRBCC_ESC_CHAR == *cp)
        {
		cp++;
//		DBGF("Escaping Byte: %c (%x)", *cp, *cp);
		*pkt_data = ~(*cp);
	} else {
		*pkt_data = *cp;
	}
	cp++;
	return cp;
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
uint16_t libdrbcc_crc_ccitt_update(uint16_t crc, uint8_t data)
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

#ifdef DEBUG
	const unsigned char *p = cp;
#endif
start:
	if (*cp == DRBCC_START_CHAR) {
		/* Start char, command, stop char */
		pkt->curr_idx = 0;
		DBG("Start char.");
		cp++;
	}
	
	if(pkt->curr_idx == 0) {
		cp = bcc_unesc_byte(cp, &pkt->cmd);
		DBGF("Cmd: %c (%x)", pkt->cmd, pkt->cmd);
		pkt->curr_idx++;
	}

/* Two bytes for start and stop char, one for command */
/* !ATTENTION!: The last two characters red are the CRC! */
	for(i = 0; i < size && pkt->curr_idx-1 < MSG_MAX_LEN-2 && *cp != DRBCC_STOP_CHAR &&  *cp != DRBCC_START_CHAR; pkt->curr_idx++, i++) {
/* FIXME: Komischer code hier, muss ganz andersch gehn mit dem de-escapen */
		cp = bcc_unesc_byte(cp, &pkt->data[pkt->curr_idx-1]);
		DBGF("Nr. %d: %x", pkt->curr_idx-1, pkt->data[pkt->curr_idx-1]);
	}

	if(*cp == DRBCC_START_CHAR) {
		goto start;
	}

	if (*cp == DRBCC_STOP_CHAR) {
		pkt->payloadlen = pkt->curr_idx-3;
		DBGF("%s Reached stop character. Struct filled (Payloadlen: %d).\n", BCC, pkt->payloadlen);
		
		crc = 0xffff;
		crc = libdrbcc_crc_ccitt_update(crc, pkt->cmd);
		for (i = 0; i < pkt->payloadlen; i++){
			crc = libdrbcc_crc_ccitt_update(crc, pkt->data[i]);
		}

		pkt->crc = (((uint16_t)pkt->data[pkt->payloadlen]) | (((uint16_t)pkt->data[pkt->payloadlen+1]) << 8));

		if (crc == pkt->crc) {
			pkt->data[pkt->payloadlen] = pkt->data[pkt->payloadlen+1] = 0;
			DBGF("%s CRC of packet was right.\n", BCC);
			return pkt->payloadlen;
		} else {
			ERR("Received packet with wrong CRC\n");
			DBGF("Expected CRC: 0x%x - CRC was: 0x%x", crc, pkt->crc);
			return -EFAULT;
		}
	} else {
		DBGF("%s No stop character found at the end of the buffer. Trying again next time\n.", BCC);
		PRINTKN(cp, size);
		return -EAGAIN;
	}
}
EXPORT_SYMBOL(deserialize_packet);

/* TODO: Makro draus machen!!!!!! */

/**
*	Create ACK packet and fill it into buffer passed to function
*	\param	toggle	one bit sequence number to use
*	\param	tx_buff	pointer to buffer, must be at least ACK_LEN in size
*	\return pointer to message buffer
*/
unsigned char *create_ack_buf(uint8_t toggle, unsigned char *tx_buff)
{
	int ret;
	struct bcc_packet pkt = {
		.cmd = (DRBCC_ACK | (toggle & TOGGLE_BITMASK)),	// FIXME: Macro?
	};
	
	DBGF("Toggle bit of ACK message: %d.", toggle);

	if ((ret = serialize_packet(&pkt, tx_buff)) < ACK_LEN) {
		DBGF("How could anything get wrong with creating an ACK message??");
		return NULL;
	}
	DBG("Created ack message.");
	return tx_buff;
}
EXPORT_SYMBOL(create_ack_buf);

/* FIXME: Als makros darstellen? */
/**
*	Create SYNC packet and fill it into buffer passed to function
*	\param	tx_buff	pointer to buffer, must be at least ACK_LEN in size
*	\return pointer to message buffer
*/
unsigned char *create_sync_buf(unsigned char *tx_buff)
{
	int ret;
	struct bcc_packet pkt = { DRBCC_SYNC | SHIFT_TBIT(1) };

	DBG("Create synchronisation message in buffer.");

	if ((ret = serialize_packet(&pkt, tx_buff)) < SYNC_LEN) {
		DBGF("How could anything get wrong with creating a static sync message??");
		return NULL;
	}
	DBG("Created sync message.");
	return tx_buff;
}
EXPORT_SYMBOL(create_sync_buf);


/* FIXME: adopt j to pkt->curr_idx 
* pointer to buffer?
*/
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

	idx = bcc_esc_byte(tx_buff, pkt->cmd, idx);
        crc = libdrbcc_crc_ccitt_update(crc, pkt->cmd);

	/* FIXME: What if parameter is null?? */

	/* Notice: idx will be changed through the function bcc_esc_byte */
	/* FIXME: 140-5: Because there might be two escape seq for crc hi and crc lo */
	for(; idx < MSG_MAX_BUFF-5 && j < pkt->payloadlen; j++) {
	                idx = bcc_esc_byte(tx_buff, pkt->data[j], idx);
                        crc = libdrbcc_crc_ccitt_update(crc, pkt->data[j]);
	}
	
	idx = bcc_esc_byte(tx_buff, crc & 0xff, idx);
	idx = bcc_esc_byte(tx_buff, crc >> 8, idx);

	tx_buff[idx] = DRBCC_STOP_CHAR;

	return idx+1;
}
EXPORT_SYMBOL(serialize_packet);
