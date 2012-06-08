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

#define DEBUG

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
	struct bcc_packet 	*resp;		
	struct bcc_packet 	temp_tx;		
	
	struct workqueue_struct	*wkq;
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
static uint8_t transaction_ready = 0; 
DECLARE_WAIT_QUEUE_HEAD(wq);

static struct toggle toggle_t = {
	.rx = 1,
	.tx = 1,
};

typedef enum {
	RQ_STD, RQ_STD_ANS, RQ_WAIT_ANS, RQ_SYNC, NONE
} GLOBAL_L2_STATES;

char async_cmd[]  = { DRBCC_REQ_STATUS, DRBCC_IND_ACCEL_EVENT };
static char l2_state = NONE;

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
		
		printk(KERN_DEBUG "toggle = %d, pkt: %x\n", toggle_t.tx, (pkt->cmd & ~TOGGLE_BITMASK) | TOGGLE_SHIFT(toggle_t.tx));
		pkt->cmd = ((pkt->cmd & ~TOGGLE_BITMASK) | TOGGLE_SHIFT(toggle_t.tx)); // FIXME: better macro or delete this comment
		
		pkt_len = serialize_packet(pkt, tx_buff);
	
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
			ret = tty->driver->write(tty, ACK_BUF_RX_0, 5);
		} else {
			ret = tty->driver->write(tty, ACK_BUF_RX_1, 5);
		}
		DBGF("Driver returned: %d", ret);
	} 
}

int toggle_bit_save_rx(struct bcc_packet *pkt) {
	if ((pkt->cmd & TOGGLE_BITMASK) == TOGGLE_SHIFT(toggle_t.rx)) {	
		toggle_t.rx = !toggle_t.rx;
		return 0;	
	} else {
		DBGF("**** Err on toggle bit (Expected: %d, received: %d (pkt->cmd = %d))!\n", 
			TOGGLE_SHIFT(toggle_t.rx), (pkt->cmd & TOGGLE_BITMASK), pkt->cmd);
		return -EAGAIN;
	}
}

/* typedef enum {
	RQ_STD, RQ_STD_ANS, RQ_WAIT_ANS, NONE
} GLOBAL_L2_STATES;*/
/* Attention: resp pointer has to be kfree'd! */
void rx_worker_thread(struct work_struct *work)
{
	int i = 0;
        struct bcc_packet *pkt = container_of(work, struct bcc_packet, work);

	if (!pkt) {
		printk(KERN_WARNING "Pkt pointer was null, something went terribly wrong.\n");
	}

/* Asynchronous messages */ 
	for(i = 0; i < sizeof(async_cmd)/sizeof(char); i++) {
		if (pkt->cmd == async_cmd[i]) {
			/* TODO: pass to /proc/something or similar, or log
				in case nobody has registered for async messages */
			/* TODO: don't forget to kfree memory !!!! */
			toggle_bit_save_rx(pkt);
			transmit_ack();
			return;
		}
	}

	if (l2_state == RQ_STD && T(pkt->cmd) == DRBCC_ACK) {
		printk(KERN_DEBUG "*** l2_state = RQ_STD\n");
		TOGGLEB(pkt);
		_the_bcc.resp = pkt;
		_the_bcc.resp->cmd = DRBCC_CMD_ILLEGAL; 
		toggle_t.tx = !toggle_t.tx;	
		printk(KERN_DEBUG "transaction_ready++; new toggle = %d \n", toggle_t.tx );
		transaction_ready = 1;	
		wake_up_interruptible(&wq);	
		l2_state = NONE;
		goto freeptr;
	}

	if (l2_state == RQ_STD_ANS && T(pkt->cmd) == DRBCC_ACK) {
		printk(KERN_DEBUG "*** l2_state = RQ_STD_ANS\n");
		toggle_t.tx = !toggle_t.tx;	
		printk(KERN_DEBUG "new toggle = %d\n", toggle_t.tx );
		l2_state = RQ_WAIT_ANS;
		goto freeptr;
	}

	if(l2_state == RQ_WAIT_ANS) {
		printk(KERN_DEBUG "*** l2_state = RQ_WAIT_ANS\n");
		transmit_ack();
		if (toggle_bit_save_rx(pkt) < 0) {
			goto freeptr;
		} 
		TOGGLEB(pkt);
		_the_bcc.resp = pkt; 
		l2_state = NONE;
		transaction_ready = 1;		
		wake_up_interruptible(&wq);	
		return;
	}

	printk(KERN_DEBUG "State = %d, cmd = %x", l2_state, pkt->cmd);
	transmit_ack();
	
/* If nobody waited for a packet, the packet memory is also just kfree'd */ 
freeptr:
	printk(KERN_DEBUG "Free pointer to pkt (%p)\n", pkt);
	kfree(pkt);
}

// TODO: Asynchronous status updates
void receive_msg(unsigned char *buf, uint8_t len)
{
	uint8_t			readc = 0;
	int 			ret = 0;
	struct bcc_packet	*pkt = NULL;

	do {
// TODO: free pkt
		pkt = (struct bcc_packet *) kmalloc(sizeof(struct bcc_packet), GFP_KERNEL);
		memset(pkt, 0, sizeof(struct bcc_packet));
		
		ret = deserialize_packet(&buf[readc], pkt, len-readc);

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
	
		INIT_WORK(&pkt->work, rx_worker_thread);
		queue_work(_the_bcc.wkq, &pkt->work);
		schedule();
	
		readc += (ret + MSG_MIN_LEN);
		DBGF("readc = %d", readc);
	
		printk(KERN_DEBUG "______Command: %d (%x)__________\n", pkt->cmd, pkt->cmd);
		
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
	char l2_state_save = l2_state;

	_the_bcc.curr = &((struct bcc_packet) { 
		.cmd = DRBCC_SYNC | TOGGLE_SHIFT(1), 
		.payloadlen = 0 });

	
	l2_state = RQ_STD;
	ret = perform_transaction();
	printk(KERN_DEBUG "***** Transmitted sync \n");	
	
	if (ret < 0) {
		DBGF("Error %d occured on sync", ret);
	}
	
	toggle_t.rx = 0;
	toggle_t.tx = 0;
 	
	DBGF("**** 1 _the_bcc.curr->cmd = %d\n", _the_bcc.curr->cmd);
	_the_bcc.curr = save;
	l2_state = l2_state_save;
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
			ERR("**** Syncing failed");
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
/* TODO: Oooops: race condition could occure? Actually, we should be save, since no two threads should ever be on this same point on a single core processor */
		transaction_ready = 0;
		ret = wait_event_interruptible_timeout(wq, transaction_ready!=0, 2*BCC_PKT_TIMEOUT);

		if (ret == 0) {
			ERR("Transaction failed on 'send message' (Timeout).");
//			return -EFAULT; 	// is there a better return value for a timeout? e.g. -EBUSY?
		}
		if (ret < 0) {
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

	down_interruptible(&sem_access);
	_the_bcc.curr = pkt;

	if (resp_cmd != DRBCC_CMD_ILLEGAL) {
		printk(KERN_DEBUG "**** Send transaction and expect ans\n");
		l2_state = RQ_STD_ANS;
	} else {
		printk(KERN_DEBUG "**** Send transaction without to expect ans\n");
		l2_state = RQ_STD;
	}
	ret = perform_transaction();
	
	if (ret < 0) {
		printk(KERN_WARNING "Transaction of packet with command %d failed\n", _the_bcc.curr->cmd);
		goto exit;	
	}
	
	/* TODO: implement this? */
	if (_the_bcc.resp->cmd != resp_cmd) {
		printk(KERN_WARNING "Received packet with wrong response command: %d\n", _the_bcc.resp->cmd);
		ret = -EFAULT; 	/* TODO: is there a better err value for this? */
		goto exit;
	}
	
exit:
// TODO: How should I signal an invalid pkt to the upper layers?
	*(_the_bcc.curr) = *(_the_bcc.resp);
	up(&sem_access);
	DBG("sem_access semaphore is up now");

/* TODO: test auf memory leak */
	kfree(_the_bcc.resp);

	return ret;
}
EXPORT_SYMBOL(transmit_packet);

static void bcc_set_termios (struct tty_struct *tty, struct ktermios * old) 
{

/*	DBG(".");
	if(!tty) {
		return;
	}
	tty->icanon = (L_ICANON(tty) != 0); 
*/
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

//	tty->receive_room = MSG_MAX_BUFF;
	
	/* Every entry point will now have access to our private data structure */
	tty->disc_data = &_the_bcc;
	/* TODO: Needed? */
	_the_bcc.tty = tty;

	if (tty->driver) {
		printk(KERN_DEBUG "+++++++ driver set in tty struct. ");
		if (!tty->driver->write) {
			printk(KERN_DEBUG "But no driver->write function. ");
		} else {
			printk(KERN_DEBUG " driver->write: %p ",  tty->driver->write);
		}
	} else {
		printk(KERN_DEBUG "++++++++ No driver set in tty struct. ");
	}
	printk(KERN_DEBUG "\n");
	
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

	flush_scheduled_work();
	destroy_workqueue(_the_bcc.wkq);

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
	printk(KERN_DEBUG "HydraIP DRBCC driver: %s (Faking IOCTL calls is fun!!).\n", __FUNCTION__);
	return 0;
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
