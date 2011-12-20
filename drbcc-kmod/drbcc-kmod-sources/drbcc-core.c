/*
 *  \file 	drbcc-core.c
*  \brief	board controller protocol communication backend
*  \author 	Christina Quast
*  drbcc-core.c:
 *	This code is the management part of the DResearch boardcontroller
 * 	protocol stack and manages access to the serial interface "/dev/ttyS0".
 * 	It implements a line discipline.
 *	It can be compiled as a module or straight into the kernel.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_ldisc.h>
#include <linux/delay.h>
#include <linux/slab_def.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <asm/semaphore.h>

/* Remove when not needed for dummy write function anymore, 
* since communication to userspace via /dev/ttyS0 is prohibited 
*/
#include <asm/uaccess.h>

/* Timeout in jiffies; here: ?00ms */
#define BCC_PKT_TIMEOUT			HZ/2
#include "drbcc.h"

/* A homepage said N_MOUSE might be unused by most systems */

#define BCC_MAGIC			22
#define BCC_TTY_MAJOR			123

/**
*	\enum 	ackstate_t
*	\brief	states of protocol state machine
*/
typedef enum {
	WAIT_FOR_ACK = 0,
	ACK_RECEIVED = 1,
	ACK_UNDEFINED = 2,
	WAIT_FOR_SYNC_ACK = 3,
} ackstate_t;

static uint8_t ackstate = ACK_UNDEFINED;
DECLARE_COMPLETION(rx_data);
DECLARE_COMPLETION(sync_ack);

#define RESEND_SYNC_THRESHOLD	2	
#define RESEND_THRESHOLD	2	

#ifndef BCC
#define BCC "[DRBCC_CORE] "
#endif

/**
*  \struct	bcc_struct
*  \brief	private data of open tty interface
*/
struct bcc_struct {
	unsigned int 		opened;		/**< int to assure our ldisc singleton pattern */
	unsigned int 		initial; 	/**< if initial, send synchronisation messages before first packet send */
	struct tty_struct	*tty;		/**< struct representing serial port */ 
	struct semaphore		access;
	
	struct bcc_packet 	*curr;		/**< Pointer to packet struct we receive from the frontend drivers */
/* FIXME: Müssen/Sollten die beiden folgenden Elemente static sein? */
	struct bcc_packet 	temp;		/**< Structure, where the parsing result of the packets from the serial driver are saved in */
};

struct class *drbcc_class;

static struct bcc_struct _the_bcc = {
	.opened = 0,
	.initial = 1,

	/* How shall this be initialised? */
/* TODO	.access.count = 1,
	.response.count = 0, */
};

static unsigned char tx_buff[MSG_MAX_BUFF] = { 0 };
uint8_t resp_cmd;
DECLARE_MUTEX(sem_access);
/* To signal the transmit that response was received, has timeout*/

static struct toggle toggle_t = {
	.rx = 0,
	.tx = 0,
};

void send_sync_msg(void);

/**
*	Give buffer to serial driver to be sent out.
*	\param	tty 		struct representing serial port 
*	\param	buf		char buffer of message to send
*	\param	pkt_len		integer to delimit buffer boundary 	
*
*	\return bytes sent on succes, negative value on failure
*/
int send_pkt(struct tty_struct *tty, unsigned char *buf, int pkt_len) {
	int ret = 0;
	
	if(_the_bcc.initial) {
		_the_bcc.initial--;
		send_sync_msg();
		ackstate = WAIT_FOR_ACK;
	}

	if (tty->driver && tty->driver->write) {
		printk(KERN_INFO "%s Transmitting packet (cmd = %x) through driver.\n", BCC, *(buf+1));
		ret = tty->driver->write(tty, buf, pkt_len);
		DBGF("Driver returned: %d", ret);
	} else {
		printk(KERN_INFO "%s No driver or driver write function defined in bcc struct.\n", BCC);
		/* TODO: Failure code */
		ret = -EFAULT;
	}
	return ret;	
}

/**
*	Create ack buffer and send out ACK message
*/
void send_ack_msg(void) 
{
	send_pkt(_the_bcc.tty, create_ack_buf(toggle_t.rx, tx_buff), ACK_LEN);
	DBG("Sent ACK message.");
}
	
/**
*	Create sync buffer and send out synchronistaion message
*/
void send_sync_msg(void)
{
	int ret, i; 

	printk(KERN_INFO "******** _the_bcc.tty->buf.head: %s", (_the_bcc.tty->buf.head == NULL)?"true":"false");
	
	DBG("Synchronize toggle bits.");
	DBGF("low latency: %x", _the_bcc.tty->low_latency);
	DBGF("minimum_to_wake: %x", _the_bcc.tty->minimum_to_wake);
	DBGF("MIN_CHAR: %x", MIN_CHAR(_the_bcc.tty));
	DBGF("tty->termios->c_lflag = %d", (_the_bcc.tty->termios->c_lflag & ICANON)?1:0);
	
	ackstate = WAIT_FOR_SYNC_ACK;
//	_the_bcc.tty->low_latency = 1;
	
	for (i = 0; i < RESEND_THRESHOLD;) {
		send_pkt(_the_bcc.tty, create_sync_buf(tx_buff), SYNC_LEN);
/*		_the_bcc.tty->driver->unthrottle(_the_bcc.tty);
		_the_bcc.tty->driver->flush_buffer(_the_bcc.tty);
		tty_flip_buffer_push(_the_bcc.tty);
*/	
	if (_the_bcc.tty->driver->flush_chars)
		_the_bcc.tty->driver->flush_chars(_the_bcc.tty);

		ret = wait_for_completion_interruptible_timeout(&sync_ack, 2*BCC_PKT_TIMEOUT);
		if (ret == 0) {
			ERR("Received no ACK message for my sync request. ");
			i++;
			if (i >= RESEND_SYNC_THRESHOLD) {
				if(_the_bcc.curr) {
					DBG("_the_bcc.curr");
					_the_bcc.curr->cmd = DRBCC_TIMEOUT;
				} else {
					DBG("_the_bcc.temp");
					_the_bcc.temp.cmd = DRBCC_TIMEOUT;
				}
				return;
			}
		} else {
			break;
		}	
	}
	toggle_t.tx = 0;
	toggle_t.rx = 0;
}

/**
*	Send out packet over serial port, does all the maintenance of the state machine (exported symbol)	
*	\param	pkt 		struct representing message to send out
*
*	\return passed return value of called functions or 0 on failure, sent bytes on success
*/
/* TODO: remove resp_cmd from parameter list */
int transmit_packet(struct bcc_packet * pkt, uint8_t resp_cmd)
{
	int pkt_len;
	int ret = 0;
	uint8_t send_cnt = 0;
	
	pkt->cmd = ((pkt->cmd & ~TOGGLE_BITMASK) | TOGGLE(toggle_t.tx));
	
	down_interruptible(&sem_access);
	ackstate = ACK_UNDEFINED;
	memset(&_the_bcc.temp, 0, sizeof(_the_bcc.temp));
	_the_bcc.temp = *pkt;
	_the_bcc.curr = pkt;
	
	DBGF("size of tx_buff array: %d", (sizeof(tx_buff)/sizeof(tx_buff[0])));
	
	memset(tx_buff, 0, (sizeof(tx_buff)/sizeof(tx_buff[0])));	
	pkt_len = serialize_packet(pkt, tx_buff);

	if (!_the_bcc.tty) {
		printk(KERN_INFO "%s TTY in bcc struct was null.\n", BCC);
		/* TODO: Failure code */
		ret = -EFAULT;
		goto out;
	}
	
	_the_bcc.tty->receive_room = MSG_MAX_BUFF; 

	for(send_cnt = 0; send_cnt < RESEND_THRESHOLD; ) {
		if(ackstate == WAIT_FOR_SYNC_ACK) {
			ret = wait_for_completion_interruptible_timeout(&sync_ack, 2*BCC_PKT_TIMEOUT);
			if(_the_bcc.curr->cmd == DRBCC_TIMEOUT) {
				goto out;
			}
		}
		pkt_len = serialize_packet(pkt, tx_buff);
	/* FIXME: toggle*/
		toggle_t.tx = ((pkt->cmd & TOGGLE_BITMASK) >> 7)?1:0;
		DBGF("Toggle bit: %x", toggle_t.tx);
		DBGF("(1): raw wants to send packet with cmd %d (%x) and to receive packet with cmd %d (%x)", pkt->cmd, pkt->cmd, 
		RSP_CMD(pkt->cmd), RSP_CMD(pkt->cmd));
		ret = send_pkt(_the_bcc.tty, tx_buff, pkt_len);
		ackstate = WAIT_FOR_ACK;
		if(ret < 0) {
			DBGF("Error %d returned by send_pkt function.", ret);
			goto out;
		}
		
		ret = wait_for_completion_interruptible_timeout(&rx_data, 2*BCC_PKT_TIMEOUT);
		if (ret == 0) {
			DBGF("Timeout (%d)!", send_cnt);
			send_cnt++;
			if(send_cnt >= RESEND_THRESHOLD) {
				ERR("Received no valid answer for my request.");
				pkt->cmd = DRBCC_TIMEOUT;
				goto out;
			} 
		} else {
			DBG("rx_data completed, therefore I was woken up.");
			goto out;
		}
	}
	memset(pkt, 0, sizeof(*pkt));

out:	
	up(&sem_access);
	DBG("sem_access semaphore is up now");

	return ret;
}
EXPORT_SYMBOL(transmit_packet);

void do_throttle(int do_throttle, struct tty_struct *tty) {
	if(do_throttle && !test_and_set_bit(TTY_THROTTLED, &tty->flags) 
			&& tty->driver->throttle) {
		DBG("Throttle tty.");
		tty->driver->throttle(tty);
	} else if (!do_throttle &&  test_and_clear_bit(TTY_THROTTLED, &tty->flags) 
			&& tty->driver->unthrottle){
		DBG("Unthrottle tty.");
		tty->driver->unthrottle(tty);
	}
}
/* TODO
static void bcc_set_room(struct tty_struct *tty)
{
	tty->receive_room = 
} */

/* Let's begin with implementing the ldisc interface functions now. */

/* Taken from the board controller rtc hack, see LINUX/rtc-drbcc */
void set_initial_termios(struct tty_struct *tty)
{
        speed_t speed = B921600;

        /* set terminal raw like cfmakeraw does (see manpage) */
	tty->termios->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty->termios->c_oflag &= ~OPOST;
	tty->termios->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty->termios->c_cflag &= ~(CSIZE | PARENB);
	tty->termios->c_cflag |= CS8;

                /* set baud rate like cfsetospeed does (see glibc sources) */
#ifdef _HAVE_STRUCT_TERMIOS_C_OSPEED
	tty->termios->c_ospeed = speed;
#endif
        tty->termios->c_cflag &= ~(CBAUD | CBAUDEX);
        tty->termios->c_cflag |= speed;

        /* set 1 stop bit */
        tty->termios->c_cflag &= ~CSTOPB;
}

static void bcc_set_termios (struct tty_struct *tty, struct ktermios * old) 
{

	DBG(".");

	if(!tty) {
		return;
	}

	tty->icanon = (L_ICANON(tty) != 0);

	//tty->termios = &tty_std_termios;
}


/**
*	Open ldisc, doing all the initialization	
*	\param	tty 		representing serial port 
*
*	\return 0 on succes, negative value on failure 
*/

/* TODO: The n_touch_receive_room() entry point for the line discipline returns the memory left in its read buffer, while the n_touch_chars_in_buffer() entry point returns the number of buffered processed characters that are ready to be delivered to user space. 
* TODO: hang up bla (also sync function, siehe slip.c)
* TODO: Register device?
*/
static int bcc_open (struct tty_struct *tty)
{
/* FIXME:	struct bcc_struct *bcc; */
	//int err;
	static char werwardas[TASK_COMM_LEN] = {"gar keiner"};
	DBG("Try to open line discipline.");
	
	if(!tty)
		return -EINVAL;
	
	if (_the_bcc.opened) {
		DBGF("Already opened by (%s) and pretty busy", werwardas);
		return -EBUSY;
	}

	tty->receive_room = MSG_MAX_BUFF;
	do_throttle(0, tty);
	// bcc_set_termios(tty, NULL);

	
	/* Every entry point will now have access to our private data structure */
	tty->disc_data = &_the_bcc;
	/* TODO: Needed? */
	_the_bcc.tty = tty;

	if (tty->driver) {
		printk("+++++++ driver set in tty struct. ");
		if (!tty->driver->write) {
			printk("But no driver->write function. ");
		} else {
			printk(" driver->write: %p ",  tty->driver->write);
		}
	} else {
		printk("++++++++ No driver set in tty struct. ");
	}
	printk("\n");
	/* Allocate the line discipline’s local read buffer that we
	* use for copying data out of the tty flip buffer --> FIXME: Needed? 
	*/
//	tty->low_latency = 1;
//	tty->minimum_to_wake = 3;
/*
	tty->termios->c_iflag = ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                | INLCR | IGNCR | ICRNL | IXON);
	tty->termios->c_oflag = ~OPOST;
	tty->termios->c_lflag = ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty->termios->c_cflag = ~(CSIZE | PARENB | CSTOPB);
	tty->termios->c_cflag |= CS8;
	tty->termios->c_ospeed = B921600;
*/

/*
struct ktermios *termios = tty->termios;

 termios->c_iflag = ICRNL | IXON;
termios->c_oflag = 0;
termios->c_lflag = ISIG | ICANON;
termios->c_iflag = 0;
termios->c_lflag &= ~ICANON;
termios->c_lflag |= ECHO | ECHOE | ECHOK |
ECHOCTL | ECHOKE | IEXTEN;
termios->c_oflag |= OPOST | ONLCR;
termios->c_iflag = 0;
termios->c_lflag &= ~(ISIG | ICANON);
termios->c_cc[VMIN] = 1;
termios->c_cc[VTIME] = 0;
*/


/*	tty->termios->c_cc[VMIN] = 1;
	tty->termios->c_cc[VTIME] = 0;
	tty->termios->c_lflag &= ~ICANON;*/
/*	DBGF("low latency: %x", tty->low_latency);

	DBGF("tty->termios->c_cc[VTIME] : %x", tty->termios->c_cc[VTIME] );
	DBGF("tty->termios->c_cc[VMIN] : %x", tty->termios->c_cc[VMIN]);
	DBGF("minimum_to_wake: %x", tty->minimum_to_wake);
	DBGF("MIN_CHAR: %x", MIN_CHAR(tty));*/
/*	tty->read_buf = kzalloc (BCC_TTY_BUFF_SIZE, GFP_KERNEL);
	if (!tty->read_buf) {
		ERR("malloc(%d) failed\n", sizeof(_the_bcc));
		err = -ENOMEM;
		goto err_free_read_buf;
	}*/

	if(tty->driver == NULL) {
		DBG("No driver set in tty");
	} else {
		DBGF("driver_name: %s, name: %s", tty->driver->driver_name, tty->driver->name);
		DBGF("(type == TTY_DRIVER_TYPE_SERIAL) ? %s , (subtype == SERIAL_TYPE_NORMAL) ? %s", 
			(tty->driver->type == TTY_DRIVER_TYPE_SERIAL)?"YES":"NO", 
			(tty->driver->subtype == SERIAL_TYPE_NORMAL)?"YES":"NO");
	}

	/* TFM FIXME: HACK ALERT */
	_the_bcc.opened++;
	strncpy(werwardas, current->comm, sizeof(current->comm));

	set_initial_termios(tty);
	
	return 0;
	
//	bcc_ldisc = tty_ldisc_get(N_TTY);
//TODO	bcc_set_termios(tty, NULL);	
	
/* err_free_read_buf:
	ERR("err_free_read_buf because allocating buffer for tty->read_buf.");
	tty->read_buf = NULL;	
	tty->disc_data = NULL;

	return err; */
}


/**
*	Close ldisc, doing all the cleanup	
*	\param	tty 		representing serial port 
*/
void bcc_close (struct tty_struct *tty)
{	
	struct bcc_struct *bcc;
	
	DBG("Close line discipline.");

/*
	DBGF("tty->termios->c_iflag = %u, %u", tty->termios->c_iflag, ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON));
	DBGF("tty->termios->c_oflag = %u, %u", tty->termios->c_oflag, ~OPOST);
	DBGF("tty->termios->c_lflag = %u, %u", tty->termios->c_lflag, ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN));
	DBGF("tty->termios->c_cflag = %u, %u", tty->termios->c_cflag, ~(CSIZE | PARENB | CSTOPB)|CS8);
	DBGF("tty->termios->c_ospeed = %u, %u", tty->termios->c_ospeed, B921600);
	
	DBGF("low latency: %x", tty->low_latency);
	DBGF("tty->termios->c_cc[VTIME] : %x", tty->termios->c_cc[VTIME] );
	DBGF("tty->termios->c_cc[VMIN] : %x", tty->termios->c_cc[VMIN]);
	DBGF("minimum_to_wake: %x", tty->minimum_to_wake);
	DBGF("MIN_CHAR: %x", MIN_CHAR(tty));
	DBGF("tty->termios->c_lflag & ICANON = %d", (tty->termios->c_lflag & ICANON)?1:0);
*/
/* TODO: Delete */
	if(tty->read_buf) {	
		kfree(tty->read_buf);
		tty->read_buf = NULL;
	}

	bcc = (struct bcc_struct*) tty->disc_data;
	tty->disc_data = NULL;

	DBG("Freed bcc_struct");
	/* TFM FIXME: HACK ALERT */
	bcc->opened--;
	bcc->tty = NULL;
}

/* is this interrupt context? shall one create a bottom half? */

/**
*	Called by serial driver passing data received on serial port for processing	
*	\param	tty 		representing serial port 
*	\param	buf		char buffer of message to received 
*	\param	count		integer for number of bytes in buffer 
*/
static void bcc_receive_buf(struct tty_struct *tty, const unsigned char *cp, char *fp, int count) 
{	
	struct bcc_struct 		*bcc;
	unsigned char 			newbuf[MSG_MAX_BUFF];
	const unsigned char 		*p;
	char				*f, flags = TTY_NORMAL;
	static struct bcc_packet 	resp = { 0 };
	int 				ret = 0;
	uint8_t 			cmd;
	uint8_t				i, j, readc = 0;

	printk(KERN_INFO "Received %d bytes from driver\n", count);
	PRINTKN(cp, count);

/*	DBGF("low latency: %x", tty->low_latency);
	DBGF("tty->termios->c_cc[VTIME] : %x", tty->termios->c_cc[VTIME] );
	DBGF("tty->termios->c_cc[VMIN] : %x", tty->termios->c_cc[VMIN]);
	DBGF("minimum_to_wake: %x", tty->minimum_to_wake);
	DBGF("MIN_CHAR: %x", MIN_CHAR(tty));	*/

	if (count > MSG_MAX_BUFF) {
		ERR("Received more bytes from serial driver than I can handle.");
		return;
	}

	bcc = tty->disc_data;
	if(!bcc) {
		ERR("bcc_struct could not be retrieved.");
		return;
	}

	if(!bcc->curr) {
		ERR("bcc->curr pointer was NULL.");
		goto out;
	} 

//	tty->receive_room -= count; 
	memset(newbuf, 0, MSG_MAX_BUFF);

	for(j = 0, i = count, p = cp, f = fp; i; i--, p++) {
		if(f)
			flags = *f++;
		switch (flags) {
		case TTY_NORMAL:	
			newbuf[j] = *p;
//			DBGF("Char %d (%x)", newbuf[j], newbuf[j]);
			j++;
			break;
		case TTY_OVERRUN:
			DBG("TTY-Overun occured");
//			do_throttle(1, tty);
			break;	
		default:
			DBGF("Flags at nr. %d was: %x", count-i, flags);
			break;
		}
	}
	
	do {	
	/* Randfälle überdenken: 
	* ESC was received, this buffer begins with STOP 
	* STOP received
	* last Paket was with failure; valid new paket beginning with START
	* --> throw away old parsed packet and begin with new one? retransmit? timeout?
	*/
		memset(&resp, 0, sizeof(struct bcc_packet));
		ret = deserialize_packet(&newbuf[readc], &resp, j-readc);

		if(ret >= 0) {
			DBG("Succeeded parsing message to struct bcc_packet");
		} else if(ret == -EAGAIN) {
			DBG("No full packet parsed, try again later");
			/* Throttle is still 0 here */
	//		do_throttle(1, tty);
			return;
		} else if(ret < 0) {
			DBG("Failure while parsing packet.");
			return;
		}	
		
		readc += (ret + MSG_MIN_LEN);
		DBGF("readc = %d", readc);
	
		cmd = resp.cmd & ~TOGGLE_BITMASK;
		printk(KERN_INFO "______Command: %d (%x)__________\n", cmd, cmd);

		//	down(bcc->curr->sem);
// TODO: Zugriff sichern über Semaphore? 
		if (bcc->temp.cmd == DRBCC_TIMEOUT) {
			ERR("Waiting for answer timed out.");	
			goto out;
		}
		switch(cmd) {	
			case DRBCC_ACK:
				DBG("Received ACK");
				// TODO: receive_ack(&resp);
				if(ackstate == WAIT_FOR_SYNC_ACK) {
					if ((resp.cmd & TOGGLE_BITMASK) == TOGGLE(1)) {
						DBG("Received sync ack.");
						complete_all(&sync_ack);
						ackstate = ACK_UNDEFINED;
						continue;
						//goto out;
					}
				}	
				if(ackstate == WAIT_FOR_ACK) {	
					if ((resp.cmd & TOGGLE_BITMASK) == TOGGLE(toggle_t.tx)) {
						DBG("Toggle bit: OK.");
						TOGGLE_BIT(toggle_t.tx);
						if(RSP_CMD(_the_bcc.curr->cmd) == DRBCC_CMD_ILLEGAL){
							DBG("No answer packets expected, therefore just returning.");
							ackstate = ACK_RECEIVED;
							_the_bcc.curr->cmd =  DRBCC_CMD_ILLEGAL;
							complete(&rx_data);
							goto out;
						} else {
							DBG("Received expected ack.");
							ackstate = ACK_RECEIVED;
							continue;
							//goto out;
						}	
					} else {
						DBGF("False toggle bit: %x, expected: %x", (resp.cmd & TOGGLE_BITMASK),  TOGGLE(toggle_t.tx));
						send_sync_msg();
						goto out;
					}
				}
				break;
			default:
				// TODO: receive_pkt(&resp);
					if ((resp.cmd & TOGGLE_BITMASK) == TOGGLE(toggle_t.rx)) {
						DBG("Toggle bit: OK.");
					} else {
						DBGF("False toggle bit: %x, expected: %x", (resp.cmd & TOGGLE_BITMASK), TOGGLE(toggle_t.rx));
						send_sync_msg();
						goto out;
					}
					if(bcc->temp.cmd != DRBCC_TIMEOUT) {
						if (cmd == RSP_CMD(bcc->curr->cmd)) {
							if(ackstate == ACK_RECEIVED) {
							/* TODO */
								printk(KERN_INFO "Received expected command: %x\n", cmd);
							/* TODO: copy pkt_len and other values */
								resp.cmd = cmd;
								*bcc->curr = resp;
								DBGF("bcc->curr->cmd = %x, bcc->curr->data = %x %x %x", bcc->curr->cmd , bcc->curr->data[0], bcc->curr->data[1], bcc->curr->data[2]);
								complete(&rx_data);
								bcc->curr = NULL;
								//do_throttle(1, tty);
								send_ack_msg();
							/*	send_pkt(tty, create_ack_buf(toggle_t.rx, tx_buff), ACK_LEN);*/
								TOGGLE_BIT(toggle_t.rx);		
								//do_throttle(0, tty);
							} else {
								DBG("No ACK received for request.");
								goto out;
							}
						} else if(DRBCC_IND_STATUS == cmd) {
							DBG("Received STATUS message or update");
							send_ack_msg();
							TOGGLE_BIT(toggle_t.rx);
							continue;		
						} else {
							DBGF("Received unknown or unexpected command: %x; expected command: %x", 
								cmd, RSP_CMD(bcc->curr->cmd));
							goto out;
						}
					} else {
						ERR("Waiting for answer timed out.");	
						goto out;
					}
				break;
			}
			DBG("Check for message type finished successfully.");

			//up(bcc->curr->sem);
	//		_the_bcc.curr = NULL;
	} while(readc < j); 
	out:
//		tty->receive_room = MSG_MAX_BUFF;
		return; 
}

/**
*	Called by userspace to set termios and other configuration work. 
*	Just passed over to the tty's generic ioctl function
*	\param	tty 		struct representing serial port 
*	\param	file			
*	\param	cmd		system call value	
*	\param  arg		arguments to system call	
*	\return value passed by the tty's generic ioctl function
*/
static int bcc_ioctl(struct tty_struct* tty, struct file * file, unsigned int cmd, unsigned long arg) {
	return tty_mode_ioctl(tty, file, cmd, arg);
}


static struct tty_ldisc bcc_ldisc = {
	.magic =		BCC_MAGIC,
	.name =			"bcc_ldisc",
	
	.open =			bcc_open,
	.close =		bcc_close,

	.ioctl = 		bcc_ioctl,

/* Note: there is no userspace functions write and read, because userspace communication should be managed by the next higher driver level */

/* driver communication */
	.receive_buf =		bcc_receive_buf,
};

/**
*	Register device with properties past to function and fixed major number under the drbcc class
*	\param	cdev 		struct representing character device
*	\param	minor		minor device number to be used			
*	\param	dev_name	char buffer containing name of device to create	
*	\return values returned by called functions 
*/
int add_device_entry(struct cdev *cdev, int minor, char *dev_name) {
	int ret;
	dev_t devno;

	devno = MKDEV(BCC_TTY_MAJOR, minor);

	/* Register device number in kernel */
	ret = register_chrdev_region(devno, 1, dev_name);
	
	if(ret < 0) {
		printk(KERN_WARNING "%s can't get major %d\n", dev_name, BCC_TTY_MAJOR);
		return ret;
	}
	
	/* FIXME: Warning by Linux Device Drivers:
	* cdev_add can fail with negative number;
	*/
	ret = cdev_add(cdev, devno, 1);
	if(ret < 0) {
                printk(KERN_WARNING "%s: Addind device failed.\n", dev_name);
                return ret;
	}

	/* Register device with it's name so that kernel can create dev entry */
	device_create(drbcc_class, NULL, devno, dev_name);

	return ret;	
}
EXPORT_SYMBOL(add_device_entry);

/**
*	Unregister device with minor number passed to us
*	\param	minor		minor device number to be used			
*/
void remove_device_entry(int minor) {
	dev_t devno;

	devno = MKDEV(BCC_TTY_MAJOR, minor);
	unregister_chrdev_region(devno, 1);
	DBGF("device number: %d, drbcc_class: %p", devno, drbcc_class);
	device_destroy(drbcc_class, devno);
}
EXPORT_SYMBOL(remove_device_entry);

int __drbcc_init(void) 
{
	int err;
	
	drbcc_class = class_create( THIS_MODULE, "drbcc");

	printk(KERN_INFO "Load ldisc.\n");

	if ((err = tty_register_ldisc(N_BCC, &bcc_ldisc))) {
		DBG("Registering line discipline failed.");
		return err;
	}

	DBG("Test debug");	
	return 0;
}

void __drbcc_exit(void) {
	int i;

	printk("HydraIP DRBCC driver unloaded...\n");

	/* TFM FIXME: HACK ALERT */
	while (0 != _the_bcc.opened ) {
		printk(KERN_WARNING "DRBCC: %d open file descriptors on line discipline's serial device file.\n", _the_bcc.opened);

		/* Hangup here ... */
		tty_hangup(_the_bcc.tty);		

		if (0 == _the_bcc.opened)
			break;

		msleep_interruptible(100);
	}
	/* then */
	if ((i = tty_unregister_ldisc(N_BCC)) < 0) {
		printk(KERN_ERR "DRBCC: can't unregister line discipline (err = %d)\n", i);
	}

	class_destroy(drbcc_class);     
	printk("HydraIP DRBCC driver: %s.\n", __FUNCTION__);
}

static int __init drbcc_init(void) {
	__drbcc_init();
	return 0;
}

static void __exit drbcc_exit(void) {
	__drbcc_exit();
}

MODULE_AUTHOR("Christina Quast <christina.dresearch@googlemail.com>");
MODULE_DESCRIPTION("HydraIP DRBCC driver");
MODULE_LICENSE("GPL");

module_init(drbcc_init);
module_exit(drbcc_exit);
