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
#include <linux/workqueue.h>
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

#define DEBUG_DRBCC
#define DEBUG

extern int chrdev_open(struct inode * inode, struct file * filp);
struct inode ino; 
struct file filp;
struct tty_struct *ttyp;

#define RESEND_SYNC_THRESHOLD	2	
#define RESEND_THRESHOLD	2	
#define RX_CURR_BUF_CNT		2

#define DRIVER_THROTTLING

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
	struct bcc_packet 	*resp;		
	struct bcc_packet 	temp_tx;		
	
	struct workqueue_struct	*wkq;

	void (*async_callback) (struct bcc_packet *);
};

struct class *drbcc_class;

static struct bcc_struct _the_bcc = {
	.opened = 0,
	.initial = 1,

	.async_callback = NULL,
	/* How shall this be initialised? */
/* TODO	.access.count = 1,
	.response.count = 0, */
};

static unsigned char tx_buff[MSG_MAX_BUFF] = { 0 };
static uint8_t resp_cmd;
DECLARE_MUTEX(sem_access);
static uint8_t transaction_ready = 0; 
DECLARE_WAIT_QUEUE_HEAD(wq);

static struct toggle toggle_t = {
	.rx = 1,
	.tx = 1,
};

typedef enum {
	RQ_STD, RQ_STD_ANS, RQ_WAIT_ANS, RQ_SYNC, NONE
} GLOBAL_L2_STATES;

static char async_cmd[]  = { DRBCC_IND_STATUS, DRBCC_IND_ACCEL_EVENT };
static char l2_state = NONE;

/*
* Layer 3 functions:
*/
// static int send_sync_msg(void);
// static int send_msg(void); 
// static int send_msg_ans(void); 

/*
* Layer 2 functions:
* Transactionbased: Request-response-logic
* Keeps track of the toggle bits
*/
static int perform_transaction(void);
//static int perform_transaction_ans(void);

/*
* Layer 1 functions:
* Parsing: char[] <==> struct bcc_struct
*/
static int synchronize(void);
static int transmit_msg(void);
static void transmit_ack(void); 
static void receive_msg(unsigned char *buf, uint8_t len);

/*
* Return codes for inter-layer-communication
*/
#define ACK_RECEIVED 	10

#ifdef DRIVER_THROTTLING
/* FIXME: rewrite code? copied from drivers/char/n_tty.c */
static void check_unthrottle(struct tty_struct * tty)
{
	if (tty->count &&
	    test_and_clear_bit(TTY_THROTTLED, &tty->flags) && 
	    tty->driver->unthrottle)
		tty->driver->unthrottle(tty);
}
#endif
/*
*
***********   LAYER 1  ***************
*/
static int transmit_msg(void) 
{
	int pkt_len, ret = 0;
	struct tty_struct *tty = _the_bcc.tty;
	struct bcc_packet *pkt = &_the_bcc.temp_tx;

	if (tty->driver && tty->driver->write) {
		memset(tx_buff, 0, (sizeof(tx_buff)/sizeof(tx_buff[0])));	
		
		DBGF("toggle = %d, pkt: 0x%x\n", toggle_t.tx, (pkt->cmd & ~TOGGLE_BITMASK) | TOGGLE_SHIFT(toggle_t.tx));
		pkt->cmd = ((pkt->cmd & ~TOGGLE_BITMASK) | TOGGLE_SHIFT(toggle_t.tx)); // FIXME: better macro or delete this comment
		
		pkt_len = serialize_packet(pkt, tx_buff);
	
		DBGF("%s Transmitting packet (cmd = 0x%x) through driver.\n", BCC, *(tx_buff+1));
		PRINTKN(tx_buff, 10);
		ret = tty->driver->write(tty, tx_buff, pkt_len);

// TODO: Do I need this?
		if (ttyp->driver->flush_chars)
			ttyp->driver->flush_chars(ttyp);

		DBGF("Driver returned: %d", ret);
	} else {
		DBGF("%s No driver or driver write function defined in bcc struct.\n", BCC);
		/* TODO: Failure code */
		ret = -EFAULT;
	}
	
	return ret;	
}

/* Why a void function? Because I don't care! */
static void transmit_ack(void) 
{
#define ACK_BUF_RX_0 (const char[]){0xfa, 0x00, 0x87, 0x0f, 0xfb}
#define ACK_BUF_RX_1 (const char[]){0xfa, 0x80, 0x8f, 0x8b, 0xfb}
	int ret = 0;
	struct tty_struct *tty = _the_bcc.tty;
	
	if (tty->driver && tty->driver->write) {
		DBGF("%s Transmitting ack through driver (toggle-bit: 0x%x).\n", BCC, toggle_t.rx);

		if(toggle_t.rx) {
			ret = tty->driver->write(tty, ACK_BUF_RX_0, 5);
		} else {
			ret = tty->driver->write(tty, ACK_BUF_RX_1, 5);
		}
		DBGF("Driver returned: %d", ret);
	} 
}

static int toggle_bit_save_rx(struct bcc_packet *pkt) {
	if ((pkt->cmd & TOGGLE_BITMASK) == TOGGLE_SHIFT(toggle_t.rx)) {	
		transmit_ack();
		toggle_t.rx = !toggle_t.rx;
		DBGF("New rx_toggle: 0x%x", toggle_t.rx);
		return 0;	
	} else {
		DBGF("**** Err on toggle bit (Expected: 0x%x, received: 0x%x (pkt->cmd = 0x%x))!\n", 
			TOGGLE_SHIFT(toggle_t.rx), (pkt->cmd & TOGGLE_BITMASK), pkt->cmd);
		// FIXME: Hack Alert!!! non-atomic toggling
		toggle_t.rx = !toggle_t.rx;
		transmit_ack();
		toggle_t.rx = !toggle_t.rx;
		return -EAGAIN;
	}
}

/* typedef enum {
	RQ_STD, RQ_STD_ANS, RQ_WAIT_ANS, NONE
} GLOBAL_L2_STATES;*/
/* Attention: resp pointer has to be kfree'd! */
static void rx_worker_thread(struct work_struct *work)
{
	int i = 0;
        struct bcc_packet *pkt = container_of(work, struct bcc_packet, work);

	if (!pkt) {
		printk(KERN_NOTICE "Pkt pointer was null, something went terribly wrong.\n");
	}

	if (l2_state == RQ_STD && T(pkt->cmd) == DRBCC_ACK) {
		DBG("*** l2_state = RQ_STD\n");
		TOGGLEB(pkt);
		DBGF("pkt->cmd: 0x%x", pkt->cmd);
		_the_bcc.resp = pkt;
		_the_bcc.resp->cmd = DRBCC_CMD_ILLEGAL; 
		pkt = NULL;
		DBGF("(1) rep>cmd: 0x%x", _the_bcc.resp->cmd);
		toggle_t.tx = !toggle_t.tx;	
		DBGF("transaction_ready++; new toggle = %d \n", toggle_t.tx );
		transaction_ready = 1;	
		DBG("transaction_ready = 1, RQ_WAIT");
		DBGF("(2) rep>cmd: 0x%x", _the_bcc.resp->cmd);
		wake_up_interruptible(&wq);	
		l2_state = NONE;
		return;
	}

	if (l2_state == RQ_STD_ANS && T(pkt->cmd) == DRBCC_ACK) {
		DBG("*** l2_state = RQ_STD_ANS\n");
		toggle_t.tx = !toggle_t.tx;	
		DBGF("new tx_toggle = %d\n", toggle_t.tx );
		l2_state = RQ_WAIT_ANS;
		goto freeptr;
	}

	if(l2_state == RQ_WAIT_ANS) {
		DBG("*** l2_state = RQ_WAIT_ANS\n");
		if (toggle_bit_save_rx(pkt) < 0) {
			goto freeptr;
		} 
		TOGGLEB(pkt);
		_the_bcc.resp = pkt; 
		pkt = NULL;
		l2_state = NONE;
		transaction_ready = 1;		
		DBG("transaction_ready = 1, RQ_WAIT_ANS");
		wake_up_interruptible(&wq);	
		return;
	}

/* Asynchronous messages */ 
	if ((toggle_bit_save_rx(pkt) < 0) || (_the_bcc.async_callback == NULL)) {
		if(_the_bcc.async_callback == NULL) {
			DBG("_the_bcc.async_callback == NULL");
		} else {
			DBGF("Msg with false toggle bit: 0x%x.", toggle_t.rx); 
		}
		goto freeptr;
	} else {
		DBG("Start async msg loop.");
		for(i = 0; i < sizeof(async_cmd)/sizeof(char); i++) {
			if (T(pkt->cmd) == async_cmd[i]) {
				/* TODO: pass to /proc/something or similar, or log
					in case nobody has registered for async messages */
				TOGGLEB(pkt);
				_the_bcc.async_callback(pkt);
				DBGF("Received async msg: %d (0x%x)", pkt->cmd, pkt->cmd);
				return;		// Callback Funktion kfree's memory
			}
		}
	}
	
/* If nobody waited for a packet, the packet memory is also just kfree'd */ 
freeptr:
	DBGF("State = %d, cmd = 0x%x", l2_state, pkt->cmd);
	DBGF("Free pointer to pkt (%p)\n", pkt);
	kfree(pkt);
	pkt = NULL;
	_the_bcc.resp = NULL;
}

static void receive_msg(unsigned char *buf, uint8_t len)
{
	uint8_t			readc = 0;
	int 			ret = 0;
	struct bcc_packet	*pkt = NULL;

	do {
		if (!pkt) {
			pkt = (struct bcc_packet *) kmalloc(sizeof(struct bcc_packet), GFP_KERNEL);
			DBGF("Kmalloced pkt (%p)\n", pkt);

			if (!pkt) {
				DBG("Out of memory!");
				return;
			}
			memset(pkt, 0, sizeof(struct bcc_packet));		
		} /* else: packet was already partly parsed */

		ret = deserialize_packet(&buf[readc], pkt, len-readc);

		if(ret >= 0) {
			DBG("Succeeded parsing message to struct bcc_packet");
		} else if(ret == -EAGAIN) {
			//TODO: just for testing: throw away party parsed packets
			//kfree(pkt);
			DBG("No full packet parsed, try again later");
			/* Throttle is still 0 here */
	//		do_throttle(1, tty);
			// FIXME: potential memory leak because I never free partly parsed packet when I don't encounter stop char
			//continue; --> did not work, because somehow I was stuck in an infinite loop
 			kfree(pkt);
			pkt = NULL;
//			return;
			break;
		} else if(ret < 0) {
			DBGF("%s: Free packet with adress %p", __FUNCTION__, pkt);
			kfree(pkt);
			pkt = NULL;
			DBG("Failure while parsing packet.");
//			return;
			break;
		}	

		if (!pkt)
			DBG ("pkt was Null!");

		INIT_WORK(&(pkt->work), rx_worker_thread);
		queue_work(_the_bcc.wkq, &pkt->work);
		readc += (ret + MSG_MIN_LEN);
		DBGF("readc = %d, len: %d", readc, len);
	
		DBGF("______Command before Schedule: %d (0x%x)__________\n", pkt->cmd, pkt->cmd);
		pkt = NULL;
		schedule();
	
	} while(readc < len); 

#ifdef DRIVER_THROTTLING
	check_unthrottle(_the_bcc.tty);
#endif
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

	DBGF("Received %d bytes from driver: \n", count);
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
//			DBGF("Char %d (0x%x)", newbuf[j], newbuf[j]);
			j++;
			break;
		case TTY_OVERRUN:
			DBG("TTY-Overun occured");
//			do_throttle(1, tty);
			break;	
		default:
			DBGF("Flags at nr. %d was: 0x%x", count-i, flags);
			break;
		}
	}

// TODO: remove all PRINTKNs
#ifdef DRIVER_THROTTLING
	if (!test_and_set_bit(TTY_THROTTLED, &tty->flags) &&
	    tty->driver->throttle)
		tty->driver->throttle(tty);
#endif
	receive_msg(newbuf, j);

}

/*
*
***********   LAYER 2   ***************
*/
static int synchronize(void)
{
	int ret = 0;
	struct bcc_packet *save = _the_bcc.curr;
	char l2_state_save = l2_state;

	_the_bcc.curr = &((struct bcc_packet) { 
		.cmd = DRBCC_SYNC | TOGGLE_SHIFT(1), 
		.payloadlen = 0 });

	
	l2_state = RQ_STD;
	ret = perform_transaction();
	DBG("***** Transmitted sync");	
	
	if (ret < 0) {
		DBGF("Error %d occured on sync", ret);
	}
	
	toggle_t.rx = 0;
	toggle_t.tx = 0;
 	
	DBGF("**** 1 _the_bcc.curr->cmd = 0x%x\n", _the_bcc.curr->cmd);
	DBGF("Free pkt with adress (%p)", _the_bcc.resp);
	kfree(_the_bcc.resp);
	_the_bcc.resp = NULL;
	_the_bcc.curr = save;
	l2_state = l2_state_save;
	DBGF("**** 2 _the_bcc.curr->cmd = 0x%x\n", _the_bcc.curr->cmd);
	return ret;
}

#define MAX_FAILED_PKT 3
static int perform_transaction(void) 
{
	int i = 0;
	int ret = 0;

	DBGF("**** _the_bcc.curr->cmd = 0x%x\n", _the_bcc.curr->cmd);

	if(_the_bcc.initial) {
		DBG("Initial call to perform transaction, therefore syncing");
		_the_bcc.initial--;
		ret = synchronize();
		if (ret < 0) {
			ERR("**** Syncing failed");
		//	_the_bcc.initial++;
                	return ret;
		}	
	}
	
	DBGF("**** _the_bcc.curr->cmd = 0x%x\n", _the_bcc.curr->cmd);
	_the_bcc.temp_tx = *(_the_bcc.curr);

	ret = transmit_msg();

	if (ret < 0) {
		DBGF("Error %d returned by transmit_msg function.", ret);
		return ret;
	}
		
	DBGF("***** Transmitted msg with cmd 0x%x \n", _the_bcc.temp_tx.cmd);	

/* TODO: Rly? hier eine do-while-Schleife? dann aber konsequent sein 
*	und eine auch in perform_transaction_ans implementieren...
*	bzw KANN sich jemals ein status-update zwischen ack und rsp drängen? --> Micha fragen */
	do {
/* TODO: Oooops: race condition could occure? Actually, we should be save, since no two threads should ever be on this same point on a single core processor */
		transaction_ready = 0;
		DBG("transaction_ready = 0");
		ret = wait_event_interruptible_timeout(wq, transaction_ready!=0, 2*BCC_PKT_TIMEOUT);
		DBGF("transaction_ready = %d", transaction_ready);
		transaction_ready = 0;
		DBGF("ret = %d", ret);

		if (ret == 0) {
			ERR("Transaction failed on 'send message' (Timeout).");	
			ret = -EFAULT; 	// is there a better return value for a timeout? e.g. -EBUSY?
		} else if (ret < 0) {
			ERR("Transaction failed on 'send message' (Error).");
			return -EFAULT;
		}
		i++;
	} while(ret <= 0 && i < MAX_FAILED_PKT);

	return ret;
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
	int ret = 0;

	if (!ttyp) {
		DBG("DRBCC-Core not ready yet.\n");
		return -EAGAIN;
	}

	down_interruptible(&sem_access);
	_the_bcc.curr = pkt;

	if (resp_cmd != DRBCC_CMD_ILLEGAL) {
		DBG("**** Send transaction and expect ans\n");
		l2_state = RQ_STD_ANS;
	} else {
		DBG("**** Send transaction without to expect ans\n");
		l2_state = RQ_STD;
	}
	ret = perform_transaction();
	DBGF("Ret: %d", ret);

	// FIXME: Mean hack, because actually !(_the_bcc.resp) should be signalised by a negative ret value :(	
	if ((ret < 0) || !(_the_bcc.resp)) {
		printk(KERN_NOTICE "Transaction of packet with command 0x%x failed\n", _the_bcc.curr->cmd);
		_the_bcc.curr->cmd = DRBCC_CMD_ILLEGAL;
		goto exit;	
	}
	
	if (resp_cmd == DRBCC_CMD_ILLEGAL) {
		DBG("Expected no answer for my command. Just returning after receiving Ack.");
		_the_bcc.curr->cmd = DRBCC_CMD_ILLEGAL;
		goto exit;
	} else if (_the_bcc.resp->cmd != resp_cmd) {
		printk(KERN_NOTICE "Received packet with wrong response command: 0x%x\n", _the_bcc.resp->cmd);
		_the_bcc.curr->cmd = DRBCC_CMD_ILLEGAL;
		ret = -EFAULT; 	/* TODO: is there a better err value for this? */
		goto exit;
	}
	
// TODO: How should I signal an invalid pkt to the upper layers?
	*(_the_bcc.curr) = *(_the_bcc.resp);

/* TODO: test auf memory leak */
	DBGF("%s: Transaction was success. Free packet with adress %p", __FUNCTION__, _the_bcc.resp);
	kfree(_the_bcc.resp);
	_the_bcc.resp = NULL;

exit:
	DBGF("Curr cmd: 0x%x", _the_bcc.curr->cmd);
	up(&sem_access);
	DBG("sem_access semaphore is up now");

	return ret;
}
EXPORT_SYMBOL(transmit_packet);

/* Every time another async function is registered,
*	the pointer to the old one is discarded
* ATTENTION: Can be exploited: 
*	- EVERYBODY can call this function
*	- what, if the module leaves us, but the pointer doesn't point to NULL?
*	--> the leaving module should always call register_async_callback(NULL) before leaving!
*/
int register_async_callback(void *func_ptr)
{
// TODO: do we need any checks here?
	_the_bcc.async_callback = func_ptr;
	return 0;
}
EXPORT_SYMBOL(register_async_callback);

static void bcc_set_termios (struct tty_struct *tty, struct ktermios * old) 
{

/*	DBG(".");
	if(!tty) {
		return;
	}
	tty->icanon = (L_ICANON(tty) != 0); 
*/
	tty->termios = &tty_std_termios;
}

static void set_default_termios(struct tty_struct *tty) {
	//tty->termios = &tty_std_termios;
	struct ktermios *tios = tty->termios;
	

	/* set terminal raw like cfmakeraw does (see manpage) */
	tios->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tios->c_oflag &= ~OPOST;
	tios->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tios->c_cflag &= ~(CSIZE | PARENB);
	tios->c_cflag |= CS8;

	/* set baud rate like cfsetospeed does (see glibc sources) */
//#ifdef _HAVE_STRUCT_TERMIOS_C_OSPEED
	tios->c_ospeed = B921600;
//#endif
	tios->c_cflag &= ~(CBAUD | CBAUDEX);
	tios->c_cflag |= B921600;

	/* set 1 stop bit */
	tios->c_cflag &= ~CSTOPB;

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
	DBG("Try to open line discipline.");

	if(!tty)
		return -EINVAL;
	
	if (_the_bcc.opened) {
		// TODO: Wait...this was secured by a semaphore, wasn't it?
		// tty->count--;
		return -EBUSY;
	}
	
	_the_bcc.opened++;	

	tty->receive_room = MSG_MAX_BUFF;
	
	/* Every entry point will now have access to our private data structure */
	tty->disc_data = &_the_bcc;
	/* TODO: Needed? */
	_the_bcc.tty = tty;
	ttyp = tty;

	if (tty->driver) {
		DBG("+++++++ driver set in tty struct. ");
		if (!tty->driver->write) {
			DBG("But no driver->write function. ");
		} else {
			DBGF(" driver->write: %p ",  tty->driver->write);
		}
	} else {
		DBG("++++++++ No driver set in tty struct. ");
	}
	DBG("\n");
	
	if(tty->driver == NULL) {
		DBG("No driver set in tty");
	} else {
		DBGF("driver_name: %s, name: %s", tty->driver->driver_name, tty->driver->name);
		DBGF("(type == TTY_DRIVER_TYPE_SERIAL) ? %s , (subtype == SERIAL_TYPE_NORMAL) ? %s", 
			(tty->driver->type == TTY_DRIVER_TYPE_SERIAL)?"YES":"NO", 
			(tty->driver->subtype == SERIAL_TYPE_NORMAL)?"YES":"NO");
	}

/*	mutex_lock(&tty->termios_mutex);
	if (test_and_clear_bit(TTY_THROTTLED, &tty->flags) && tty->ops->unthrottle)
			tty->ops->unthrottle(tty);
	mutex_unlock(&tty->termios_mutex);
*/
	set_default_termios(tty); 

	_the_bcc.wkq = create_singlethread_workqueue("drbcc_rx_wkq");

	return 0;
}


/**
*	Close ldisc, doing all the cleanup	
*	\param	tty 		representing serial port 
*/
void bcc_close (struct tty_struct *tty)
{	
	struct bcc_struct *bcc;
	
	DBG("Close line discipline.");
	
	_the_bcc.opened--;
	if(_the_bcc.opened)
		return;	

	flush_scheduled_work();
	destroy_workqueue(_the_bcc.wkq);

/* TODO: Delete */
	if(tty->read_buf) {	
		kfree(tty->read_buf);
		tty->read_buf = NULL;
	}

	bcc = (struct bcc_struct*) tty->disc_data;
	tty->disc_data = NULL;
	ttyp = NULL;
	_the_bcc.tty = NULL;

	DBG("Freed bcc_struct");
	/* TFM FIXME: HACK ALERT */
	//bcc->opened--;
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
int bcc_ioctl(struct tty_struct* tty, struct file * file, unsigned int cmd, unsigned long arg) {
	/* TODO: if(Sig___) { ..}  */
	DBGF("HydraIP DRBCC driver: %s\n", __FUNCTION__);
	return tty_mode_ioctl(_the_bcc.tty, file, cmd, arg);
}

static struct tty_ldisc bcc_ldisc = {
	.magic 		= BCC_MAGIC,
	.name 		= "bcc_ldisc",
	
	.open 		= bcc_open,
	.close 		= bcc_close,

	.ioctl 		= bcc_ioctl,
	.set_termios 	= bcc_set_termios,	
/* Note: there is no userspace functions write and read, because userspace communication should be managed by the next higher driver level */

/* driver communication */
	.receive_buf 	= bcc_receive_buf,
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
		printk(KERN_NOTICE "%s can't get major %d\n", dev_name, BCC_TTY_MAJOR);
		return ret;
	}
	
	/* FIXME: Warning by Linux Device Drivers:
	* cdev_add can fail with negative number;
	*/
	ret = cdev_add(cdev, devno, 1);
	if(ret < 0) {
                printk(KERN_NOTICE "%s: Addind device failed.\n", dev_name);
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

	DBGF("Load ldisc.\n");

	if ((err = tty_register_ldisc(N_BCC, &bcc_ldisc))) {
		DBG("Registering line discipline failed.");
		return err;
	}

	memset(&ino, 0, sizeof(ino));
	memset(&filp, 0, sizeof(filp));
	ino.i_rdev = MKDEV(4,64);
 	filp.f_u.fu_list.next = &filp.f_u.fu_list;
 	filp.f_u.fu_list.prev = &filp.f_u.fu_list;

	chrdev_open(&ino, &filp);	

	DBG("Test debug");	
	return 0;
}

void __drbcc_exit(void) {
	int i;

	printk(KERN_INFO "HydraIP DRBCC driver unloaded...\n");

	/* TFM FIXME: HACK ALERT */
	while (0 != _the_bcc.opened ) {
		printk(KERN_NOTICE "DRBCC: %d open file descriptors on line discipline's serial device file.\n", _the_bcc.opened);

		/* Hangup here ... */
		tty_hangup(ttyp);		
		ttyp->count--;

		if (0 == _the_bcc.opened)
			break;

		msleep_interruptible(100);
	}
	/* then */
	if ((i = tty_unregister_ldisc(N_BCC)) < 0) {
		printk(KERN_ERR "DRBCC: can't unregister line discipline (err = %d)\n", i);
	}

	// !!! TODO: Close File pointer
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
