/* Editor hints for vim
 * vim:set ts=4 sw=4 noexpandtab:  */

/* $Id: drbcc_ll.h 88712 2012-07-09 07:39:31Z rothe $ */

#ifndef _DRBCC_LL_
#define _DRBCC_LL_

#ifndef __KERNEL__
#ifndef __AVR__
#include <stdio.h>
#include <sys/time.h>
#endif /* __AVR__ */

#include <stdint.h>

#include "drbcc_com.h"
#endif /* __KERNEL__ */

#define TOGGLE_BITMASK 0x80

/*/////////////////////////////////////////////////////////////////////*/
/*                                                                     */
/* code compatibility definitions (HOST was formerly known as OXE)     */
/*                                                                     */
/* do not use in new code!                                             */
/*                                                                     */
/*/////////////////////////////////////////////////////////////////////*/
#define DRBCC_OXE_BOOT_MODE_SELECT_REQ DRBCC_HOST_BOOT_MODE_SELECT_REQ
/* end of code compatibility definitions */



// commands must be lower than 128
// toggle bit is 0x80
// message byte sequence is given as <byte> <byte> .... (framing is <STX> <<message bytes>> <crc hi> <crc lo> <EOT>)
typedef enum
{
	//<mid>: message id

	// semantic free acknowledge, contains toggle bit of libdrbcc_received cmd -> <ack>
	DRBCC_ACK = 0, // <mid>

	// message causes reset of rx and tx t-bits, t-bit of this message must be 1; message will be processed regardless of t-bit
	DRBCC_SYNC = 1, // <mid>

	// message is alternative answer to DRBCC_SYNC (instead of DRBCC_ACK), indicating simplified synchronous protocol mode
	DRBCC_SYNC_ANSWER = 2, // <mid>

	// Get DRBCC Protocol Version
	DRBCC_CMD_REQ_PROTOCOL_VERSION = 3, // <mid>

	// Give DRBCC Protocol Version
	DRBCC_CMD_IND_PROTOCOL_VERSION = 4, // <mid> <version major> <version minor> <firmware running>

	// read rtc request
	DRBCC_REQ_RTC_READ = 5, // <mid>

	// give rtc data
	// RTC is a Maxim DS3231, for format of the bytes see DS3231 datasheet
	DRBCC_IND_RTC_READ = 6, // <mid> <sec> <min> <hour> <day> <date> <month> <year> <epoch>

	// set rtc, epoch will be incremented automatically
	// set rtc result is a DRBCC_IND_RTC_READ message
	DRBCC_REQ_RTC_SET = 7, // <mid> <sec> <min> <hour> <day> <date> <month> <year>

	// read ext. flash id
	DRBCC_REQ_EXTFLASH_ID = 8, // <mid>

	// give ext. flash id
	DRBCC_IND_EXTFLASH_ID = 9, // <mid> <mid> <devid1> <devid2>

	// read ext. flash data
	DRBCC_REQ_EXTFLASH_READ = 10, // <mid> <adrh> <adrm> <adrl> <len of data>

	// give ext. flash data
	DRBCC_IND_EXTFLASH_READ = 11, // <mid> <adrh> <adrm> <adrl> <len of data> <data[0]> ... <data[len-1]>

	// write ext. flash data, adrl + len < 256, flash area must be empty before write
	DRBCC_REQ_EXTFLASH_WRITE = 12, // <mid> <adrh> <adrm> <adrl> <len of data> <data[0]> ... <data[len-1]>

	// ext. flash write result
	DRBCC_IND_EXTFLASH_WRITE_RESULT = 13, // <mid> <adrh> <adrm> <adrl> <len> <result>

	// erase ext. flash block (4k block)
	DRBCC_REQ_EXTFLASH_BLOCKERASE = 14, // <mid> <blkh> <blkl>

	// ext. flash erase block result
	DRBCC_IND_EXTFLASH_BLOCKERASE_RESULT = 15, // <mid> <blkh> <blkl> <result>

	// fw invalidate <mid>
	DRBCC_REQ_FW_INVALIDATE = 16,

	// fw invalidate done <mid>
	DRBCC_IND_FW_INVALIDATED = 17,

	// request board controller restart <mid> <when>
	// when=0: restart when host power is off
	// when=1: restart immediately (killing the host)
	DRBCC_REQ_BCTRL_RESTART = 18,

	// restart accepted <mid>
	DRBCC_IND_BCTRL_RESTART_ACCEPTED = 19,

	// set led to desired color <mid> <led> <color> led:1..4 color:0=off, 1=green, 2=red, 3=orange
	// optional extended parameter set: <on_time> <off_time> optional: <phase> 
	// if extended paramset is used, LED will flash: on for on_time (in 1/20s), the off for off_time (in 1/20s)
	// all flash interval are aligned to a minute counter
	// the phase parameter allow shifting relative to this alignment (in 1/20s)
	// if on_time or off_time is 0, LED will not flash; if on_time is 0, LED will be OFF!
	DRBCC_REQ_SET_LED = 20,

	// firmware update started <mid>
	DRBCC_IND_FW_UPDATE_STARTED = 21,

	// firmware updates the boot loader if any file present <mid>
	DRBCC_REQ_BL_UPDATE = 22,

	// boot loader update done <mid> <result> result=1: success
	DRBCC_IND_BL_UPDATE = 23,

	// heartbeat message with timeout in seconds <mid> <timeout high> <timeout low>
	// System will turn off if no new heartbeat message arrives before timeout is over
	DRBCC_REQ_HEARTBEAT = 24,

	// status request for GPIO, Ignition, HD-Power, GPI-Power  <mid>
	DRBCC_REQ_STATUS = 25,

	// answer to DRBCC_REQ_STATUS or asynchronous status changed indication (all input states with delta bit)
	// <mid> <SFI> <dSFI> <SFO> <GPI> <dGPI> <GPO> <RTC_Temp> <7 byte accel values: dataformat, xx, yy, zz>
	// <voltage values: 2 byte signed int, in 10mV units, interpretation depends 
	// on board_id value supplied in DRBCC_IND_ID_DATA message
	// board_id 0 (VAP boards): PWR_CAP, PWR_CAM, VKEY, SUPERCAP, 5V, 3V3, 1V8, 1V2, 1V0>
	// board_id 1 (STO system): PWR_CAP, PWR_CAM, VKEY, SUPERCAP, 5V, 3V3, 1V5, VTERM, 0>
	DRBCC_IND_STATUS = 26,

	// test message to eject hard disc <mid>
	DRBCC_REQ_HD_EJECT = 27,

	// message to switch hard disc power <mid> <onoff>
	DRBCC_REQ_HD_ONOFF = 28,

	// message to switch GPI power <mid> <onoff>
	DRBCC_REQ_GPI_POWER = 29,

	// Put log entry to flash <mid> <ring> <len of data> <data[0]> ... <data[len-1]>
	DRBCC_REQ_PUT_LOG = 30,

	// ACK <mid>
	DRBCC_IND_PUT_LOG = 31,

	// Get current position of ring log -> <mid>
	DRBCC_REQ_RINGLOG_POS = 32,

	// Tells current position of ring log -> <mid> <blockh> <blockl> <entry> <wrapflag>
	DRBCC_IND_RINGLOG_POS = 33,

	// perform AES test -> <mid> <dir: 0=enc 1=dec> <16 byte aes key> <16 byte data>
	DRBCC_REQ_AES_TEST = 34,

	// response to DRBCC_REQ_AES_TEST -> <mid> <16 byte result data>
	DRBCC_IND_AES_TEST = 35,

	// set a single gpo <mid> <gpo> <onoff>
	DRBCC_REQ_SET_GPO = 36,

	// announce shutdown with timeout in seconds <mid> <timeout high> <timeout low>
	// system will turn off after timeout is reached or if HOST_RUNNING signal line indicates shutdown complete
	// during shutdown, heartbeat checking is disabled
	// shutdown cannot be canceled, shutdown timer cannot be restarted (only timeout can be changed)
	DRBCC_REQ_SHUTDOWN = 37,

	// get ID data <mid>
	DRBCC_REQ_ID_DATA = 38,

	// answer to DRBCC_REQ_ID_DATA, <mid> <board-id> <slot-id> <8 byte serial number>
	DRBCC_IND_ID_DATA = 39,

	// read data from specified memory device
	DRBCC_REQ_MEMDEV_READ = 40, // <mid> <memdevtype> <adrh> <adrm> <adrl> <len of data>

	// give data from specified memory device
	DRBCC_IND_MEMDEV_READ = 41, // <mid> <memdevtype> <adrh> <adrm> <adrl> <len of data> <data[0]> ... <data[len-1]>

	// write data to specified memory device, adrl + len < 256
	DRBCC_REQ_MEMDEV_WRITE = 42, // <mid> <memdevtype> <adrh> <adrm> <adrl> <len of data> <data[0]> ... <data[len-1]>

	// memory device write result
	DRBCC_IND_MEMDEV_WRITE_RESULT = 43, // <mid> <memdevtype> <adrh> <adrm> <adrl> <len> <result>

	// key processing notification, may contain eject request --> if eject requested and HD in use: unmount and turn off HD
	DRBCC_IND_KEY_PROCESSING = 44, // <mid> <7 byte key serial number> <16 byte token>

	// reset ringlog, report new log position with DRBCC_IND_RINGLOG_POS message
	DRBCC_CLEAR_RINGLOG_REQ = 45, // <mid>

	// send check sum block command to OXE in UART-boot mode, adrh/m/l specifies start address 
	// and lenh/m/l specifies length of memory area for which CRC calculation is requested
	DRBCC_OXE_UART_BOOT_CALCCRC_REQ = 46, // <mid> <adrh> <adrm> <adrl> <lenh> <lenm> <lenl>

	// response to DRBCC_OXE_UART_BOOT_CALCCRC_REQ, result is a 32 bit checksum, high byte first
	DRBCC_OXE_UART_BOOT_CALCCRC_IND = 47, // <mid> <crcbyte3> <crcbyte2> <crcbyte1> <crcbyte0>

	// send run command to OXE in UART-boot mode, adrh/m/l specifies start address
	DRBCC_OXE_UART_BOOT_RUN_REQ = 48, // <mid> <adrh> <adrm> <adrl>

	// set HOST boot mode to the desired value, must be done before ignition, will be reset to BOOT_NOR after HOST shutdown
	// for bootmode values see drbcc_com.h
	DRBCC_HOST_BOOT_MODE_SELECT_REQ = 49, // <mid> <bootmode>

	// BCTRL requests HDD turn off (due to key eject request); HOST shall unmount HDD, then turn off HDD power
	// reason: 1 byte for bitmask indicating actions requested by BCTRL; currently bit 0 indicates HD power off request
	DRBCC_HDD_OFF_REQ = 50, // <mid> <reason> <16 byte key token data>

	// send generic debug/config data to BCTRL: 16bit identifier/address, length of following data, content of data depends on address
	DRBCC_REQ_DEBUG_SET = 51, // <mid> <adrh> <adrl> <len> <data...>

	// request generic debug/config data from BCTRL: 16bit identifier/address
	DRBCC_REQ_DEBUG_GET = 52, // <mid> <adrh> <adrl>

	// answer to generic debug/config data request: 16bit identifier/address, length of following data, content of data depends on identifier/address
	DRBCC_IND_DEBUG_GET = 53, // <mid> <adrh> <adrl> <len> <data...>

	// unsolicited acceleration event message: 1 byte event type + parameters (acceleration values: 1g is equivalent to value 256 or value 1 is equivalent to 3,90625mg(1000/256))
	DRBCC_IND_ACCEL_EVENT = 54, // <mid> <event-type> <3 x 2 byte int16 <low byte><high byte> accel values: xx, yy, zz>

	// default answer if an unspecified error occurred in sync message handling (unknown command, invalid parameters ...)
	DRBCC_SYNC_CMD_ERROR = 127, // <mid> 

	// extended message

	// forwarding message
	DRBCC_CMD_FORWARD,

	DRBCC_CMD_ILLEGAL = 0xff

} DRBCC_COMMANDS_t;

#ifdef  __AVR_ARCH__
#define AVR 1
#endif

// Msg definitions
#define DRBCC_MAX_MSG_LEN 140
#define DRBCC_CRC_LEN 2

#define DRBCC_CMD_t uint8_t
#define SRC_ADDR_t uint8_t
#define DST_ADDR_t uint8_t

#define CHECK_HANDLE(x) 	if (!libdrbcc_initialized) return DRBCC_RC_NOTINITIALIZED;  if (!((x) && ((DRBCC_t*) (x))->magic == 0xDAD1DADA)) return DRBCC_RC_INVALID_HANDLE;

#define DRBCC_MAX_STR_LEN 255

#define DRBCC_MAX_PAYLOAD (sizeof(DRBCC_CMD_t) + sizeof(SRC_ADDR_t) + sizeof(DST_ADDR_t) + DRBCC_MAX_MSG_LEN + DRBCC_CRC_LEN)

#define DRBCC_PART_MAGIC1 0xAF
#define DRBCC_PART_MAGIC2 0xFE

#define DRBCC_START_CHAR 0xFA
#define DRBCC_STOP_CHAR  0xFB
#define DRBCC_ESC_CHAR   0xFC


#ifndef __KERNEL__
typedef struct
{
#ifdef __AVR__

#else
	void * next;					// next pointer for queuing
	char str[DRBCC_MAX_STR_LEN+1];	// msg info, for error logging etc.
#endif
	uint8_t msg_len;
	uint8_t msg[DRBCC_MAX_PAYLOAD];
	// Message may contain a forwarding information + message + tmp space for crc bytes
} DRBCC_MESSAGE_t;

#ifndef __AVR__
extern void drbcc_dump_message(FILE *fp, const DRBCC_MESSAGE_t *mesg);

typedef enum
{
	DRBCC_STATE_USER,			// internal state is not used
	DRBCC_STATE_PARTITION_REQ,	// waiting for partition table
	DRBCC_STATE_DELETE_FILE,
	DRBCC_STATE_GET_FILE,		// read flash to file
	DRBCC_STATE_PUT_FILE,		// write file to flash
	DRBCC_STATE_GET_LOG,		// read log entries from flash
} DRBCC_STATES_t;

#endif


// -----------------------------------------------------------------------------------------


// timestamp uses DS3231 RTC register format; day-of-week register (byte 3) is replaced by epoch counter value
typedef struct
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t epoch;
	uint8_t date;
	uint8_t mon;
	uint8_t year;
} logtimestamp_t;

typedef struct
{
	logtimestamp_t timestamp;
	uint8_t datalen;
	uint8_t data[7];
} log_payload_first_t;

typedef struct
{
	uint8_t data[15];
} log_payload_extension_t;

typedef union
{
	log_payload_first_t     first;
	log_payload_extension_t extension;
} log_payload_t;

typedef struct
{
	DRBCC_LOG_EVENT_t event;
	log_payload_t payload;
} log_entry_t;

typedef struct
{
#ifdef __AVR__
	USART_t *usart;
#else
	int fd;
	int send_toggle;
	int expected_recv_toggle;
	int escaped;
	int wait_for_ack;
	int wait_for_answer;
	int wait_for_stop_char;
	int wait_for_first_sync_ack;
	int sync_mode;
	int repeatCount;
	void *context;
	DRBCC_SESSION_t session;
	int indClosesSession;
	uint16_t crc;
	char lockfile[32];
	uint8_t readbuf[DRBCC_MAX_PAYLOAD];
	uint8_t writebuf[DRBCC_MAX_PAYLOAD*2];
	int writepos;
	unsigned int bufstart;
	unsigned int bufend;
	unsigned int curFileIndex;	// partition table index to delete, read or write in current state
	unsigned int curFilelength;
	unsigned int curFilestart;
	unsigned int maxFilelength;
	int curFileType;
	int logtype;
	unsigned flashAddr;
	unsigned flashPos;
	unsigned flashLen;
	uint8_t *flashData;
	unsigned int start;
	int entries;
	log_payload_first_t dlpf;
	uint8_t *logdata;
	int logindex;
	int logrest;
	int logpos;
	uint8_t logentry;
	uint8_t logwrapflag;
	char curFilename[FILENAME_MAX];
	struct timeval ackTimeout;
	struct timeval nextTimeout;
	struct timeval resend;
	struct timeval sendnext;
	DRBCC_STATES_t state;
	unsigned int magic;
	DRBCC_MESSAGE_t * prioQueue;
	DRBCC_MESSAGE_t * secQueue;
	DRBCC_MESSAGE_t * repeatMsg;
	DRBCC_MESSAGE_t cur_msg;
	// EventHandler
	DRBCC_RTC_CB_t rtc_cb;
	DRBCC_STATUS_CB_t status_cb;
	DRBCC_STATUS_CB_t hdOffRequest_cb;
	DRBCC_ERROR_CB_t error_cb;
	DRBCC_SESSION_CB_t session_cb;
	DRBCC_PROTOCOL_CB_t protocol_cb;
	DRBCC_ID_CB_t id_cb;
	DRBCC_FLASH_ID_CB_t flash_id_cb;
	DRBCC_READ_FLASH_CB_t read_flash_cb;
	DRBCC_WRITE_FLASH_CB_t write_flash_cb;
	DRBCC_ERASE_FLASH_CB_t erase_flash_cb;
	DRBCC_PARTITION_TABLE_CB_t partitiontable_cb;
	DRBCC_PROGRESS_CB_t progress_cb;
	DRBCC_GETLOG_CB_t getlog_cb;
	DRBCC_GETPOS_CB_t getpos_cb;
	DRBCC_TEST_AES_CB_t testaes_cb;
	DRBCC_OXE_UART_BOOT_CALCCRC_CB_t oxe_uart_boot_calccrc_cb;
	DRBCC_DEBUG_GET_CB_t debug_get_cb;
	DRBCC_ACCEL_EVENT_CB_t accel_event_cb;
#endif
	DRBCC_RC_t last_error;
} DRBCC_t;

#ifdef __AVR__
//extern DRBCC_RC_t drbcc_init(DRBCC_t *drbcc, USART_t *usart);
#endif

extern DRBCC_RC_t libdrbcc_send_message(DRBCC_t *drbcc, DRBCC_MESSAGE_t *msg);

extern DRBCC_RC_t drbcc_rpc(DRBCC_t *drbcc, DRBCC_MESSAGE_t *request,
	DRBCC_MESSAGE_t *response, int retries);

#endif /* __KERNEL__ */

#endif /* _DRBCC_LL_ */

/* Editor hints for emacs
 *
 * Local Variables:
 * mode:c
 * c-basic-offset:4
 * indent-tabs-mode:t
 * tab-width:4
 * End:
 *
 * NO CODE BELOW THIS! */
