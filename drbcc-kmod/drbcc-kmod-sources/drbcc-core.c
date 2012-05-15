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

#define DEBUG

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
DECLARE_COMPLETION(transaction);

#define RESEND_SYNC_THRESHOLD	2	
#define RESEND_THRESHOLD	2	
#define RX_CURR_BUF_CNT		2

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
	struct semaphore	access;
	
	/****** For communication between Layer 1-and-2 only *****/
	struct bcc_packet 	*curr;		/**< Pointer to packet struct we receive from the frontend drivers */
	struct bcc_packet 	resp;		
/* FIXME: Müssen/Sollten die beiden folgenden Elemente static sein? */
	// TODO: array without count?
	struct bcc_packet 	temp_rx[RX_CURR_BUF_CNT];		/**< Structure, where the parsing result of the packets from the serial driver are saved in */
	// TODO struct buf_cnt 	{
	unsigned short 		temp_rx_read_cnt : RX_CURR_BUF_CNT;
	unsigned short 		temp_rx_write_cnt : RX_CURR_BUF_CNT;
	// TODO } rx_buf_cnt;
	struct bcc_packet 	temp_tx;		/**< Structure, where the parsing result of the packets to be send by the serial driver are saved in */
};

struct class *drbcc_class;

static struct bcc_struct _the_bcc = {
	.opened = 0,
	.initial = 1,

	.temp_rx_write_cnt = 0,
	.temp_rx_read_cnt = 0,
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

/*
* Layer 3 functions:
*/
int send_sync_msg(void);
int send_msg(void);
int send_msg_ans(void);

/*
* Layer 2 functions:
* Transactionbased: Request-response-logic
* Keeps track of the toggle bits
*/
int perform_transaction(void);
int perform_transaction_ans(void);

/*
* Layer 1 functions:
* Parsing: char[] <==> struct bcc_struct
*/
int synchronize(void);
int transmit_msg(void);
void transmit_ack(void); 
void receive_msg(unsigned char *buf, uint8_t len);

/*
* Return codes for inter-layer-communication
*/
#define ACK_RECEIVED 	10

/*
*
***********   LAYER 1  ***************
*/
int transmit_msg(void) 
{
	int pkt_len, ret = 0;
	struct tty_struct *tty = _the_bcc.tty;
	struct bcc_packet *pkt = &_the_bcc.temp_tx;

	if (tty->driver && tty->driver->write) {
		memset(tx_buff, 0, (sizeof(tx_buff)/sizeof(tx_buff[0])));	
		pkt_len = serialize_packet(pkt, tx_buff);
		
		pkt->cmd = ((pkt->cmd & ~TOGGLE_BITMASK) | TOGGLE_SHIFT(toggle_t.tx)); // FIXME: better macro or delete this comment
	
		printk(KERN_INFO "%s Transmitting packet (cmd = %x) through driver.\n", BCC, *(tx_buff+1));
		PRINTKN(tx_buff, 10);
		ret = tty->driver->write(tty, tx_buff, pkt_len);
		DBGF("Driver returned: %d", ret);
	} else {
		printk(KERN_INFO "%s No driver or driver write function defined in bcc struct.\n", BCC);
		/* TODO: Failure code */
		ret = -EFAULT;
	}
	
	return ret;	
}

/* Why a void function? Because I don't care! */
void transmit_ack(void) 
{
#define ACK_BUF_RX_0 (char[]){0xfa, 0x00, 0x87, 0x0f, 0xfb}
#define ACK_BUF_RX_1 (char[]){0xfa, 0x80, 0x8f, 0x8b, 0xfb}
	int ret = 0;
	struct tty_struct *tty = _the_bcc.tty;
	
	if (tty->driver && tty->driver->write) {
		printk(KERN_INFO "%s Transmitting ack through driver.\n", BCC);

		if(toggle_t.rx) {
/* TODO: I need an ACK Buffer!! */ 
			ret = tty->driver->write(tty, ACK_BUF_RX_0, 5);
		} else {
			ret = tty->driver->write(tty, ACK_BUF_RX_1, 5);
		}
		DBGF("Driver returned: %d", ret);
	} 
}

// TODO: Asynchronous status updates
void receive_msg(unsigned char *buf, uint8_t len)
{
	uint8_t		readc = 0;
	int 		ret = 0;

	do {
		memset(&_the_bcc.temp_rx[_the_bcc.temp_rx_write_cnt], 0, sizeof(struct bcc_packet));
		
		ret = deserialize_packet(&buf[readc], &_the_bcc.temp_rx[_the_bcc.temp_rx_write_cnt], len-readc);

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
		
// TODO: work queue? semaphor? 
		complete(&transaction);	
		schedule();	
	
		// FIXME: increase is atomic? or does it not matter, 
		// because the receive_msg function is kinda atomic?
		_the_bcc.temp_rx_write_cnt++;
		
		readc += (ret + MSG_MIN_LEN);
		DBGF("readc = %d", readc);

	
		printk(KERN_DEBUG "______Command: %d (%x)__________\n", 
			_the_bcc.temp_rx[_the_bcc.temp_rx_write_cnt-1].cmd, 
			_the_bcc.temp_rx[_the_bcc.temp_rx_write_cnt-1].cmd);

		
	} while(readc < len); 
}

/*	Possible buffers that can be received:
*	ACK
*	MSG
*	ACK + MSG
* 	--> ACK + MSG + MSG (Status update) ??
*/
static void bcc_receive_buf(struct tty_struct *tty, const unsigned char *cp, char *fp, int count) 
{
	unsigned char 			newbuf[MSG_MAX_BUFF];
	uint8_t				i, j;
	const unsigned char 		*p;
	char				*f, flags = TTY_NORMAL;

	printk(KERN_INFO "Received %d bytes from driver\n", count);
	PRINTKN(cp, count);

	if (count > MSG_MAX_BUFF) {
		ERR("Received more bytes from serial driver than I can handle.");
		return;
	}

//	tty->receive_room -= count; 
	memset(newbuf, 0, MSG_MAX_BUFF);

// FIXME: put into separate function?: 
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

// TODO: remove all PRINTKNs
	PRINTKN(newbuf, j);
	receive_msg(newbuf, j);
}

/*
*
***********   LAYER 2   ***************
*/
int synchronize(void)
{
	int ret = 0;
	struct bcc_packet *save = _the_bcc.curr;

	_the_bcc.curr = &((struct bcc_packet) { 
		.cmd = DRBCC_SYNC, 
		.payloadlen = 0 });

	ret = perform_transaction();
	printk("***** Transmitted sync \n");	
	
	if (ret < 0) {
		DBGF("Error %d occured on sync", ret);
	}
	
	DBGF("**** 1 _the_bcc.curr->cmd = %d\n", _the_bcc.curr->cmd);
	_the_bcc.curr = save;
	DBGF("**** 2 _the_bcc.curr->cmd = %d\n", _the_bcc.curr->cmd);
	return ret;
}


#define MAX_FAILED_PKT 3
int perform_transaction(void) 
{
	int i = 0;
	int ret = 0;

	DBGF("**** _the_bcc.curr->cmd = %d\n", _the_bcc.curr->cmd);

	if(_the_bcc.initial) {
		_the_bcc.initial--;
		ret = synchronize();
		if (ret < 0) {
                	return ret;
		}	
	}
	
	DBGF("**** _the_bcc.curr->cmd = %d\n", _the_bcc.curr->cmd);
	_the_bcc.temp_tx = *(_the_bcc.curr);

	ret = transmit_msg();
	if (ret < 0) {
		DBGF("Error %d returned by transmit_msg function.", ret);
		return ret;
	}
		
	DBGF("***** Transmitted msg with cmd %d \n", _the_bcc.temp_tx.cmd);	

/* TODO: Rly? hier eine do-while-Schleife? dann aber konsequent sein 
*	und eine auch in perform_transaction_ans implementieren...
*	bzw KANN sich jemals ein status-update zwischen ack und rsp drängen? --> Micha fragen */
	do {
		ret = wait_for_completion_interruptible_timeout(&transaction, 2*BCC_PKT_TIMEOUT);

		if (ret == 0) {
			ERR("Transaction failed (on 'send message').");
			return -EFAULT; 	// is there a better return value for a timeout? e.g. -EBUSY?
		}
		
		if (_the_bcc.temp_rx[_the_bcc.temp_rx_read_cnt].cmd == DRBCC_ACK) {
			/* Sending the packet seems to have succeeded */
			DBGF("*****Received ACK for cmd %d \n", _the_bcc.temp_tx.cmd);	
			toggle_t.tx = !toggle_t.tx;	
			ret = ACK_RECEIVED;
		} else {
			DBGF("*****No ACK received for cmd %d \n", _the_bcc.temp_tx.cmd);	
			ret = -EFAULT;
		}
		_the_bcc.temp_rx_read_cnt++;
		i++;

	} while(ret < 0 && i < MAX_FAILED_PKT);

	return ret;
}


/* TODO: check for return! if return code < 0, there is an invalid pkt in _the_bcc.resp */
int perform_transaction_ans(void) {
	int ret = 0;

	ret = perform_transaction();
	if (ret != ACK_RECEIVED) {
		DBGF("***** No ack received\n");
		return ret;
	}
	
	ret = wait_for_completion_interruptible_timeout(&transaction, 2*BCC_PKT_TIMEOUT);
	if (ret == 0) {
		ERR("Transaction failed (on 'receive ans')."); // FIXME: write into log instead of to stdout ?
		return -EFAULT; 	// is there a better return value for a timeout?
	}
	
	_the_bcc.resp = _the_bcc.temp_rx[_the_bcc.temp_rx_read_cnt];
	_the_bcc.temp_rx_read_cnt++;

/* TODO: Layer 1 or Layer 2 job? Bei falschen Toggle wirklich einfach zurückkehren...? */	
	if (TOGGLEB(_the_bcc.resp) == TOGGLE_SHIFT(toggle_t.rx)) {	
		toggle_t.rx = !toggle_t.rx;	
	} else {
		DBGF("**** Err on toggle bit!\n");
		ret = -EFAULT;
	}
		
	/* I don't care whether sending the ack worked */
	transmit_ack();
	
	return ret;	
}



int transmit_packet(struct bcc_packet * pkt, uint8_t resp_cmd)
{
	int ret = 0;

	_the_bcc.curr = pkt;

	if (resp_cmd != DRBCC_CMD_ILLEGAL) {
		printk("**** Send transaction and expect ans\n");
		ret = perform_transaction_ans();
	} else {
		printk("**** Send transaction without to expect ans\n");
		ret = perform_transaction();
	}

	if (ret < 0) {
		printk(KERN_WARNING "Transaction of packet with command %d failed\n", _the_bcc.curr->cmd);	
	}
	
	/* TODO: implement this? */
	if (ret != resp_cmd) {
		printk(KERN_WARNING "Received packet with wrong response command: %d\n", _the_bcc.resp.cmd);	
	}
	
	*(_the_bcc.curr) = _the_bcc.resp;
	return ret;
}
EXPORT_SYMBOL(transmit_packet);

/**
*	Send out packet over serial port, does all the maintenance of the state machine (exported symbol)	
*	\param	pkt 		struct representing message to send out
*
*	\return passed return value of called functions or 0 on failure, sent bytes on success
*/
/* TODO: remove resp_cmd from parameter list */
//int transmit_packet(struct bcc_packet * pkt, uint8_t resp_cmd)
//{
/*	int pkt_len;
	int ret = 0;
	uint8_t send_cnt = 0;
	
	pkt->cmd = ((pkt->cmd & ~TOGGLE_BITMASK) | TOGGLE(toggle_t.tx));
	
	down_interruptible(&sem_access);
	ackstate = ACK_UNDEFINED;
	memset(&_the_bcc.temp_tx, 0, sizeof(_the_bcc.temp_tx));
	_the_bcc.temp_tx = *pkt;
	_the_bcc.curr = pkt;
	
	DBGF("size of tx_buff array: %d", (sizeof(tx_buff)/sizeof(tx_buff[0])));
	
	memset(tx_buff, 0, (sizeof(tx_buff)/sizeof(tx_buff[0])));	
	pkt_len = serialize_packet(pkt, tx_buff);

	if (!_the_bcc.tty) {
		printk(KERN_INFO "%s TTY in bcc struct was null.\n", BCC);
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
*/

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

	dump_stack();	
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
	/* TODO: if(Sig___) { ..}  */
	printk("HydraIP DRBCC driver: %s (Faking IOCTL calls is fun!!).\n", __FUNCTION__);
	return 0;
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
