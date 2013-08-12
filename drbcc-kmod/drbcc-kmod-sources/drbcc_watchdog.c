/** 
 *  \file 	drbcc_wd.c	
 *  \brief	DRBCC Watchdog implementation, depends on the DRBCC CORE functionality/module
 *  \author 	Christina Quast, DResearch Fahrzeugelektronik GmbH
 *
 * (C) 2013 DResearch Fahrzeugelektronik GmbH
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h> 
#include <linux/workqueue.h>

#include "drbcc.h"

#define DEFAULT_TIMEOUT   CONFIG_DARMOK_WATCHDOG_TIMEOUT       /* default timeout in seconds */
#define WD_TIMEOUT_MAX    0xFFFF                               /* maximum watchdog timeout */
#define WD_KEEPALIVE_TIME CONFIG_DARMOK_WATCHDOG_KEEPALIVE     /* watchdog keepalive interval */

#define WD_MAGIC	'V'

#define BWD 		"[DRBCC-WATCHDOG] "

/* Function declarations */
static void blocking_wd_keepalive_thread(struct work_struct *work);


static uint16_t timeout = DEFAULT_TIMEOUT;
static struct workqueue_struct *timeout_keepalive_wq;	
static DECLARE_DELAYED_WORK(tm_work, blocking_wd_keepalive_thread);

/*
 * Kick the watchdog.
 * This is done by sending a heartbeat message to the board controller watchdog 
 * through drbcc-core.
 */
static int wd_keepalive(void) 
{
	int ret;

	struct bcc_packet pkt = {
		.cmd        =  DRBCC_REQ_HEARTBEAT,
		.payloadlen = 2,
	};

	pkt.data[0] = timeout >> 8;
	pkt.data[1] = timeout;

	DBGF(BWD "Send timeout to board controller (current timeout: %d).", timeout);

	if ((ret = transmit_packet(&pkt)) < 0) {
		ERR(BWD "Error while trying to send heartbeat to board controller.");
		return -EFAULT;
	}

	if(pkt.cmd != cmd_responses[DRBCC_REQ_HEARTBEAT]) {
		if(pkt.cmd == DRBCC_TIMEOUT) {
			DBG("Waiting for ACK for HEARTBEAT message timed out.");
		} else {
			DBGF("Received message with wrong response type: %x", pkt.cmd);
		}
		return -EFAULT;
	}

	return ret;
}

static void blocking_wd_keepalive_thread(struct work_struct *work)
{
	wd_keepalive();
	queue_delayed_work(timeout_keepalive_wq, &tm_work, WD_KEEPALIVE_TIME*HZ);
}

static int drbcc_wd_set_timeout(int t)
{
	/* Otherwise, what should we do with timeout = 0? */
	/* Should we als send a keepalive signal after changing the timeout? */
	if (t < 1 || t > WD_TIMEOUT_MAX) 
		return -EINVAL;

	timeout = t;
	DBGF("Darmok Watchdog: Set timeout to %d\n", timeout);
	return 0;
}

static int wd_timer_start(void)
{
	if(WD_KEEPALIVE_TIME >0) {
		DBG("The Darmok Watchdog Driver starts kicking the watchdog\n");
		timeout = DEFAULT_TIMEOUT;
		wd_keepalive();

		queue_delayed_work(timeout_keepalive_wq, &tm_work, WD_KEEPALIVE_TIME*HZ);
	}
	return 0;
}

static int wd_timer_stop(void) 
{
	cancel_delayed_work(&tm_work);
	flush_workqueue(timeout_keepalive_wq);
	return 0;
}

/**
 *  drbcc_wd_write - Write to watchdog device
 *  Any character but one that is written to the watchdog device is interpreted 
 *  as a keepalive signal. Just a 'V' is interpreted as a stop signal.
 */
static ssize_t drbcc_wd_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	int offset;
	char c;

	if (count) {
		wd_keepalive();
	}

	for (offset = 0; offset < count; offset++) {
		if (get_user(c, buf + offset)) {
			return -EFAULT;
		}
		if (c == WD_MAGIC) {
			wd_timer_start();
		}
	}

	return count;
}

static long drbcc_wd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) 
{
	int new_timeout;
	int options;

	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
		case WDIOC_SETOPTIONS:
			if (get_user(options, p)) {
				return -EFAULT;
			}

			switch(options) {
				case WDIOS_DISABLECARD:
					/* The kernel drbcc watchdog is responsible 
					 * for keeping track of the time now
					 */
					wd_timer_start();
					return 0;

				case WDIOS_ENABLECARD:
					/* The userspace wants to be responsible 
					 * of kicking the watchdog periodically.
					 */
					wd_timer_stop();
					return 0;

				default:
					return -EFAULT;
			}

		case WDIOC_KEEPALIVE:
			wd_keepalive();
			return 0;

		case WDIOC_SETTIMEOUT:
			if (get_user(new_timeout, p))
				return -EFAULT;

			if(drbcc_wd_set_timeout(new_timeout))
				return -EINVAL;

			wd_keepalive();

			/* Not having a break at this point is done on purpose */
		case WDIOC_GETTIMEOUT:
			return put_user(timeout, p);

		default:
			// TODO: why this error value? 
			return -ENOTTY;
	}	
}

static int drbcc_wd_open(struct inode *inode, struct file *file)
{
	/* The userspace is responsible 
	 * of kicking the watchdog periodically.
	 */
	wd_timer_stop();
	return 0;
}

static int drbcc_wd_release(struct inode *inode, struct file *file)
{
	/* The kernel drbcc watchdog is responsible for keeping track 
	 * of the time ONLY if the magic character 'V' has been received before,
	 * or the user space watchdog was via ioctl() disabled.
	 * No watchdog timer start here.
	 */
	return 0;
}

static const struct file_operations drbcc_wd_fops = {
	.owner   	= THIS_MODULE,
	.write		= drbcc_wd_write,
	.unlocked_ioctl	= drbcc_wd_ioctl,
	.open		= drbcc_wd_open,
	.release	= drbcc_wd_release,
};

static struct miscdevice drbcc_wd_miscdev = {
	.minor      = WATCHDOG_MINOR,
	.name       = "watchdog",
	.fops       = &drbcc_wd_fops,
};

static int __init drbcc_wd_init_module(void) 
{
	int ret = 0;

	DBGF("HydraIP DRBCC Watchdog driver: %s.\n", __FUNCTION__);

	ret = misc_register(&drbcc_wd_miscdev);

	if (ret) {
		ERR(BWD "Cannot register miscdev on minor=%d (err=%d)\n", drbcc_wd_miscdev.minor, ret);
	}

	timeout_keepalive_wq = create_singlethread_workqueue("timeout_blocking_keepalive_wq");
	wd_timer_start();

	return ret;
}

static void __exit drbcc_wd_cleanup_module(void) 
{
	DBGF("Unload HydraIP DRBCC Watchdog driver: %s.\n", __FUNCTION__);

	flush_workqueue(timeout_keepalive_wq);
	destroy_workqueue(timeout_keepalive_wq);

	misc_deregister(&drbcc_wd_miscdev);	
}

module_init(drbcc_wd_init_module);
module_exit(drbcc_wd_cleanup_module);
MODULE_AUTHOR("DResearch Fahrzeugelektronik GmbH");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DRBCC Watchdog driver");
