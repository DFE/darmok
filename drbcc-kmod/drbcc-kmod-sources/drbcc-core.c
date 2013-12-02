/*
 *  \file 	drbcc-core.c
 *  \brief	board controller protocol communication backend
 *  \author 	Christina Quast
 *  drbcc-core.c:
 *	This code is the management part of the DResearch boardcontroller
 * 	protocol stack and manages access to the serial interface "/dev/ttyS0".
 * 	It implements a line discipline.
 *	The code can be compiled as a module or straight into the kernel.	
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_ldisc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include "drbcc_core.h"

static DEFINE_SPINLOCK(pkt_lock);
static unsigned long flags;

// TODO: maybe decrease count when closing the ldisc?
static void open_port(struct work_struct *work);
static DECLARE_DELAYED_WORK(open_port_work, open_port);
static struct file *filp = 0;

/*
 * Layer 2 functions:
 * Transactionbased: Request-response-logic
 * Keeps track of the toggle bits
 */
static int perform_transaction(struct bcc_packet *pkt, struct bcc_packet * pkt_resp);

/*
 * Layer 1 functions:
 * Parsing: char[] <==> struct bcc_struct
 */
static int synchronize(void);
static int transmit_msg(struct bcc_packet *pkt);
static void transmit_ack(int ack_toggle); 
static void receive_msg(unsigned char *buf, uint8_t len);

/*
 *  \struct	bcc_struct
 *  \brief	private data of open tty interface
 */
struct bcc_struct {
	unsigned int 		opened;		/**< int to assure our ldisc singleton pattern */
	unsigned int 		initial; 	/**< if initial, send synchronisation messages before first packet send */
	struct tty_struct	*tty;		/**< struct representing serial port */ 
	struct semaphore	access;

	/****** For communication between Layer 1-and-2 only *****/
	struct bcc_packet 	*resp;		

	struct workqueue_struct	*parser_wkq;

	void (*async_callback) (struct bcc_packet *);
};

static struct class *drbcc_class;

static struct bcc_struct _the_bcc = {
	.opened = 0,
	.initial = 1,

	.async_callback = NULL,
};

static DEFINE_SEMAPHORE(sem_access);
static uint8_t transaction_ready = 0; 
static DECLARE_WAIT_QUEUE_HEAD(wq);

static struct toggle toggle_t = {
	.rx = 1,
	.tx = 1,
};

/*
 *
 ***********   LAYER 1  ***************
 */
static int transmit_msg(struct bcc_packet *pkt) 
{
	int pkt_len, ret = 0;
	struct tty_struct *tty = _the_bcc.tty;
	unsigned char tx_buff[MSG_MAX_BUF] = { 0 };

	if (tty->driver && tty->driver->ops->write) {
		memset(tx_buff, 0, (sizeof(tx_buff)/sizeof(tx_buff[0])));	

		SET_TBIT(pkt, toggle_t.tx); 
		DBGF("toggle = %d, pkt_cmd: 0x%x\n", toggle_t.tx, pkt->cmd);

		pkt_len = serialize_packet(pkt, tx_buff);

		DBGF("%s Transmitting packet (cmd = 0x%x) through driver.\n", BCC, *(tx_buff+1));
		PRINTKN(tx_buff, 10);
		ret = tty->driver->ops->write(tty, tx_buff, pkt_len);

		// TODO: Do I need this?
		if (tty->driver->ops->flush_chars)
			tty->driver->ops->flush_chars(tty);

		DBGF("Driver returned: %d", ret);
	} else {
		DBGF("%s No driver or driver write function defined in bcc struct.\n", BCC);
		/* TODO: Failure code */
		ret = -EFAULT;
	}

	return ret;	
}

/* Why a void function? Because I don't care! */
static void transmit_ack(int ack_toggle) 
{
	int ret = 0;
	struct tty_struct *tty = _the_bcc.tty;

	if (tty->driver && tty->driver->ops->write) {
		DBGF("%s Transmitting ack through driver (toggle-bit: 0x%x).\n", BCC, toggle_t.rx);

		if(ack_toggle) {
			ret = tty->driver->ops->write(tty, ACK_BUF_RX_1, 5);
		} else {
			ret = tty->driver->ops->write(tty, ACK_BUF_RX_0, 5);
		}
		DBGF("Driver returned: %d", ret);
	} 
}

static int toggle_bit_save_rx(struct bcc_packet *pkt) {
	if (TBIT_OF_CMD(pkt->cmd) == SHIFT_TBIT(toggle_t.rx)) {	
		transmit_ack(toggle_t.rx);
		toggle_t.rx = !toggle_t.rx;
		DBGF("New rx_toggle: 0x%x", toggle_t.rx);
		return 0;	
	} else {
		DBGF("**** Err on toggle bit (Expected: 0x%x, received: 0x%x (pkt->cmd = 0x%x))!\n", 
				SHIFT_TBIT(toggle_t.rx), TBIT_OF_CMD(pkt->cmd), pkt->cmd);
		transmit_ack(!toggle_t.rx);
		return -EAGAIN;
	}
}

static void parser_worker_thread(struct work_struct *work)
{
	struct parse_work *pw = container_of(work, struct parse_work, work);

	receive_msg(pw->buf, pw->cnt);

	kfree(pw);
}

/* typedef enum {
   RQ_STD, RQ_STD_ANS, RQ_WAIT_ANS, NONE
   } GLOBAL_L2_STATES;*/
static void rx_packet(struct bcc_packet *pkt)
{
	int i = 0;

	if (!pkt) {
		printk(KERN_WARNING "DRBCC Packet pointer was null, something went terribly wrong!\n");
		return;
	}

	if (l2_state == RQ_STD && CMD_WITHOUT_TBIT(pkt->cmd) == DRBCC_ACK) {
		DBG("*** l2_state = RQ_STD\n");
		CMD_DEL_TBIT(pkt);
		DBGF("pkt->cmd: 0x%x", pkt->cmd);
		spin_lock_irqsave(&pkt_lock, flags);
		if (_the_bcc.resp) { 
			*_the_bcc.resp = *pkt;
			_the_bcc.resp->cmd = DRBCC_CMD_ILLEGAL; 
		}
		spin_unlock_irqrestore(&pkt_lock, flags);
		toggle_t.tx = !toggle_t.tx;	
		DBGF("transaction_ready++; new toggle = %d \n", toggle_t.tx );
		transaction_ready = 1;	
		DBG("transaction_ready = 1, RQ_WAIT");
		wake_up_interruptible(&wq);	
		l2_state = NONE;
	} else if (l2_state == RQ_STD_ANS && CMD_WITHOUT_TBIT(pkt->cmd) == DRBCC_ACK) {
		DBG("*** l2_state = RQ_STD_ANS\n");
		toggle_t.tx = !toggle_t.tx;	
		DBGF("new tx_toggle = %d\n", toggle_t.tx );
		l2_state = RQ_WAIT_ANS;
	} else if (l2_state == RQ_WAIT_ANS) {
		DBG("*** l2_state = RQ_WAIT_ANS\n");
		if (toggle_bit_save_rx(pkt) >= 0) {
			CMD_DEL_TBIT(pkt);
			spin_lock_irqsave(&pkt_lock, flags);
			if (_the_bcc.resp) { *_the_bcc.resp = *pkt; }
			spin_unlock_irqrestore(&pkt_lock, flags);
			pkt = NULL;
			l2_state = NONE;
			transaction_ready = 1;		
			DBG("transaction_ready = 1, RQ_WAIT_ANS");
			wake_up_interruptible(&wq);	
		} 
	/* Asynchronous messages */ 
	} else if ((toggle_bit_save_rx(pkt) < 0) || (_the_bcc.async_callback == NULL)) {
		if(_the_bcc.async_callback == NULL) {
			DBG("_the_bcc.async_callback == NULL");
		} else {
			DBGF("Msg with false toggle bit: 0x%x.", toggle_t.rx); 
		}
	} else {
		DBG("Start async msg loop.");
		for(i = 0; i < sizeof(async_cmd)/sizeof(char); i++) {
			if (CMD_WITHOUT_TBIT(pkt->cmd) == async_cmd[i]) {
				/* TODO: pass to /proc/something or similar, or log
				   in case nobody has registered for async messages */
				CMD_DEL_TBIT(pkt);
				_the_bcc.async_callback(pkt);
				DBGF("Received async msg: %d (0x%x)", pkt->cmd, pkt->cmd);
				return;
			}
		}
	}

	DBGF("State = %d, cmd = 0x%x", l2_state, pkt->cmd);
}

static void receive_msg(unsigned char *buf, uint8_t len)
{
	uint8_t			readc = 0;
	int 			ret = 0;
	static struct bcc_packet curr_pkt = { 0 };

	do {
		ret = deserialize_packet(&buf[readc], &curr_pkt, len-readc);

		if(ret > 0) {
			DBG("Succeeded parsing message to struct bcc_packet");
		} else if(ret == -EAGAIN) {
			DBG("No full packet parsed, try again later");
			break;
		} else if(ret < 0) {
			DBG("Failure while parsing packet.");
			memset(&curr_pkt, 0, sizeof(struct bcc_packet));		
			break;
		}	

		rx_packet(&curr_pkt);
		memset(&curr_pkt, 0, sizeof(struct bcc_packet));		
		readc += ret;
		DBGF("readc = %d, len: %d", readc, len);

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
	//	unsigned char 			newbuf[MSG_MAX_BUF];
	uint8_t				i, j;
	const unsigned char 		*p;
	char				*f, flags = TTY_NORMAL;
	struct parse_work	*pw;

	DBGF("Received %d bytes from driver: \n", count);
	PRINTKN(cp, count);

	if (count > MSG_MAX_BUF) {
		ERR("Received more bytes from serial driver than I can handle.");
		return;
	}

	//	tty->receive_room -= count; 
	/* TODO: free packet!*/
	pw = (struct parse_work *) kmalloc(sizeof(struct parse_work), GFP_KERNEL);
	if (!pw) {
		ERR("Out of memory");
	}
	memset(&pw->buf, 0, MSG_MAX_BUF);

	// FIXME: put into separate function?: 
	for(j = 0, i = count, p = cp, f = fp; i; i--, p++) {
		if(f)
			flags = *f++;
		switch (flags) {
			case TTY_NORMAL:	
				pw->buf[j] = *p;
				//			DBGF("Char %d (0x%x)", newbuf[j], newbuf[j]);
				j++;
				break;
			case TTY_OVERRUN:
				DBG("TTY-Overun occured");
				break;	
			default:
				DBGF("Flags at nr. %d was: 0x%x", count-i, flags);
				break;
		}
	}

	// TODO: remove all PRINTKNs
	if (j == 0) {
		DBG("No bytes to parse");
		return;
	}

	pw->cnt = j;
	INIT_WORK(&pw->work, parser_worker_thread);
	queue_work(_the_bcc.parser_wkq, &pw->work);
	schedule();
}

/*
 *
 ***********   LAYER 2   ***************
 */
static int synchronize(void)
{
	int ret = 0;
	struct bcc_packet pkt_resp = { 0 };
	struct bcc_packet pkt = { 
		.cmd = DRBCC_SYNC | SHIFT_TBIT(1), 
		.payloadlen = 0 };


	l2_state = RQ_STD;
	ret = perform_transaction(&pkt, &pkt_resp);
	DBG("***** Transmitted sync");	

	if (ret < 0) {
		DBGF("Error %d occured on sync", ret);
		return ret;
	}

	toggle_t.rx = 0;
	toggle_t.tx = 0;

	return ret;
}

#define MAX_FAILED_PKT 3
static int perform_transaction(struct bcc_packet * pkt, struct bcc_packet * pkt_resp) 
{
	int ret = 0;

	WARN_ON(transaction_ready != 0);
	transaction_ready = 0;

	WARN_ON(_the_bcc.resp != 0);
	spin_lock_irqsave(&pkt_lock, flags);
	_the_bcc.resp = pkt_resp;
	spin_unlock_irqrestore(&pkt_lock, flags);

	ret = transmit_msg(pkt);

	if (ret < 0) {
		DBGF("Error %d returned by transmit_msg function.", ret);
		return ret;
	}

	DBGF("***** Transmitted msg with cmd 0x%x \n", pkt->cmd);	

	DBGF("transaction_ready = %d", transaction_ready);
	ret = wait_event_interruptible_timeout(wq, transaction_ready!=0, 2*BCC_PKT_TIMEOUT*MAX_FAILED_PKT);
	DBGF("transaction_ready = %d", transaction_ready);
	DBGF("ret = %d", ret);

	if (ret == 0) {
		ERR("Transaction failed on 'send message' (Timeout).");	
		ret = -EFAULT; 	// is there a better return value for a timeout? e.g. -EBUSY?
	} else if (ret == -ERESTARTSYS) {
		ERR("Process was interrupted by a signal.");
		flush_workqueue(_the_bcc.parser_wkq);
	} else if (ret < 0) {	// Can this ever happen?
		ERR("Transaction failed on 'send message' (Error).");
		ret = -EFAULT;
	}
	transaction_ready = 0;

	spin_lock_irqsave(&pkt_lock, flags);
	_the_bcc.resp = 0;
	spin_unlock_irqrestore(&pkt_lock, flags);

	return ret;
}

/**
 *	Send out packet over serial port, does all the maintenance of the state machine (exported symbol)	
 *	\param	pkt 		struct representing message to send out
 *
 *	\return passed return value of called functions or 0 on failure, sent bytes on success
 */
/* TODO: remove resp_cmd from parameter list */
int transmit_packet(struct bcc_packet * pkt)
{
	uint8_t resp_cmd;
	int ret = 0;
	struct bcc_packet pkt_resp;

	if (!_the_bcc.tty) {
		DBG("DRBCC-Core not ready yet.\n");
		return -EAGAIN;
	}

	ret = down_interruptible(&sem_access);

	if (ret)	
		return ret;

	if(_the_bcc.initial) {
		DBG("Initial call to transmit_packet, therefore syncing");
		ret = synchronize();
		if (ret < 0) {
			ERR("**** Syncing failed");
			up(&sem_access);
			return ret;
		}
		_the_bcc.initial = 0;
	}

	resp_cmd = RSP_CMD(pkt->cmd);

	if (resp_cmd != DRBCC_CMD_ILLEGAL) {
		DBG("**** Send transaction and expect ans\n");
		l2_state = RQ_STD_ANS;
	} else {
		DBG("**** Send transaction without to expect ans\n");
		l2_state = RQ_STD;
	}
	ret = perform_transaction(pkt, &pkt_resp);
	DBGF("Ret: %d", ret);
	if (ret < 0) {
		printk(KERN_NOTICE "Transaction of packet with command 0x%x failed with error value %x\n", pkt->cmd, ret);
		pkt->cmd = DRBCC_CMD_ILLEGAL;
		goto exit;	
	}

	if (resp_cmd == DRBCC_CMD_ILLEGAL) {
		DBG("Expected no answer for my command. Just returning after receiving Ack.");
		pkt->cmd = DRBCC_CMD_ILLEGAL;
		goto exit;
	} else if (pkt_resp.cmd != resp_cmd) {
		printk(KERN_NOTICE "Received packet with wrong response command: 0x%x\n", pkt_resp.cmd);
		pkt->cmd = DRBCC_CMD_ILLEGAL;
		ret = -EFAULT; 	/* TODO: is there a better err value for this? */
		goto exit;
	}

	// TODO: How should I signal an invalid pkt to the upper layers?
	*pkt = pkt_resp;

	DBGF("%s: Transaction success.", __FUNCTION__);

exit:
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
	tty->termios = tty_std_termios;
}

static void set_default_termios(struct tty_struct *tty) {
	//tty->termios = &tty_std_termios;
	struct ktermios *tios = &tty->termios;


	/* set terminal raw like cfmakeraw does (see manpage) */
	tios->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tios->c_oflag &= ~OPOST;
	tios->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tios->c_cflag &= ~(CSIZE | PARENB);
	tios->c_cflag |= CS8;

	/* set baud rate like cfsetospeed does (see glibc sources) */
	//#ifdef _HAVE_STRUCT_TERMIOS_C_OSPEED
	tios->c_ispeed = B921600;
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

	tty->receive_room = MSG_MAX_BUF;

	_the_bcc.tty = tty;

	if (tty->driver) {
		DBG("+++++++ driver set in tty struct. ");
		if (!tty->driver->ops->write) {
			DBG("But no driver->ops->write function. ");
		} else {
			DBGF(" driver->ops->write: %p ",  tty->driver->ops->write);
		}
	} else {
		DBG("++++++++ No driver set in tty struct. ");
	}
	DBG("\n");

	if(tty->driver == NULL) {
		DBG("No driver set in tty");
	} 

	set_default_termios(tty); 

	_the_bcc.parser_wkq = create_singlethread_workqueue("drbcc_parser_wkq");

	return 0;
}


/**
 *	Close ldisc, doing all the cleanup	
 *	\param	tty 		representing serial port 
 */
static void bcc_close (struct tty_struct *tty)
{	
	DBG("Close line discipline.");

	_the_bcc.opened--;
	if(_the_bcc.opened)
		return;	

	flush_workqueue(_the_bcc.parser_wkq);
	destroy_workqueue(_the_bcc.parser_wkq);

	_the_bcc.tty = NULL;

	DBG("Freed bcc_struct");
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
	DBGF("HydraIP DRBCC driver: %s\n", __FUNCTION__);
	return tty_mode_ioctl(_the_bcc.tty, file, cmd, arg);
}

static struct tty_ldisc_ops bcc_ldisc = {
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
	device_create(drbcc_class, NULL, devno, NULL, dev_name);

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

static void open_port(struct work_struct *work)
{
	static int count = 0;
	char *bctrl_port = "/dev/ttyUSB1";

	if(IS_ERR_OR_NULL(filp)) {
		filp = filp_open(bctrl_port, O_RDWR, 0);
		if(IS_ERR(filp)) {
			count++;
			if((count%30) == 0) {
				printk(KERN_INFO "[drbcc] Open port '%s' failed (error: %ld). Try it on.\n", bctrl_port, PTR_ERR(filp));
			}
			schedule_delayed_work(&open_port_work, HZ/10);
		} else {
			printk(KERN_INFO "[drbcc] Open port '%s'\n", bctrl_port);
		}
	} else {
		printk(KERN_INFO "[drbcc] '%s' already open\n", bctrl_port);
	}
}

static int __drbcc_init(void) 
{
	int err;

	drbcc_class = class_create( THIS_MODULE, "drbcc");

	DBGF("Load ldisc.\n");

	if ((err = tty_register_ldisc(N_BCC, &bcc_ldisc))) {
		DBG("Registering line discipline failed.");
		return err;
	}

	/* hikirk machine specific boardcontroller connection */
	open_port(0);
	return 0;
}

static void __drbcc_exit(void) {
	int i;

	printk(KERN_INFO "HydraIP DRBCC driver unloaded...\n");

	/* TFM FIXME: HACK ALERT */
	while (0 != _the_bcc.opened ) {
		printk(KERN_NOTICE "DRBCC: %d open file descriptors on line discipline's serial device file.\n", _the_bcc.opened);

		/* Hangup here ... */
		tty_hangup(_the_bcc.tty);		
		_the_bcc.tty->count--;

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

MODULE_AUTHOR("DResearch Fahrzeugelektronik GmbH");
MODULE_DESCRIPTION("HydraIP DRBCC driver");
MODULE_LICENSE("GPL");

module_init(drbcc_init);
module_exit(drbcc_exit);
