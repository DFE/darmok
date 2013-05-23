/** 
*   \file 	drbcc_wd.c	
*   \brief	DRBCC Watchdog implementation, depends on the DRBCC CORE functionality/module
*   \author 	Christina Quast
*
* (C) 2013 DResearch Digital Media Systems GmbH
*
*/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h> 

#include "drbcc.h"

#define DEFAULT_TIMEOUT 60         /* default timeout in seconds */
#define WD_TIMEOUT_MAX 0xFFFF

static uint16_t timeout = DEFAULT_TIMEOUT;

static uint8_t user_wd_active;

/* 
 * When the userspace stops kicking the watchdog this kernel driver 
 * has to take care for this, and the other way round. 
 */
#define us_wd_stop kernel_wd_start
#define us_wd_start kernel_wd_stop

#define BWD 		"[DRBCC-WATCHDOG] "

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

static int drbcc_wd_set_timeout(int t)
{
/* Otherwise, what should we do with timeout = 0? */
	if (t < 1 || t > WD_TIMEOUT_MAX) 
		return -EINVAL;
	
	timeout = t;
	return 0;
}

static int kernel_wd_start(void)
{
	/* Stopping the userspace watchdog.
 		This means the drbcc-watchdog driver is now responsible for
		kicking the watchdog from time to time 
	*/
	timeout = DEFAULT_TIMEOUT;
	user_wd_active = 0;
	return 0;
}

static int kernel_wd_stop(void) 
{
	user_wd_active = 1;
	return 0;
}

static long drbcc_wd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) 
{
	int ret = -ENOTTY;
	int new_timeout, options;

	switch (cmd) {
	case WDIOC_SETOPTIONS:
		if (get_user(options, (int __user *)arg)) {
			return -EFAULT;
		}

		if (options & WDIOS_DISABLECARD) {
			/* The kernel drbcc watchdog is responsible 
			 * for keeping track of the time now
			 */
			kernel_wd_start();
			ret = 0;
		}
		if (options & WDIOS_ENABLECARD) {
			/* The userspace wants to be responsible 
			 * of kicking the watchdog periodically.
			 */
			kernel_wd_stop();
			ret = 0;
		}

		return ret;

	case WDIOC_KEEPALIVE:
		printk("******WDIOC_KEEPALIVE\n");
		wd_keepalive();
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_timeout, (int __user *)arg))	
			return -EFAULT;
		
		if(drbcc_wd_set_timeout(new_timeout))
			return -EINVAL;

		wd_keepalive();

/* Not having a break at this point is done on purpose */
	case WDIOC_GETTIMEOUT:
		return put_user(timeout, (int __user *)arg);

	default:
// TODO: why this error value? 
		return -ENOTTY;
	}	
	return ret;
}


static int drbcc_wd_open(struct inode *inode, struct file *file)
{
	/* FIXME: Stop automatic watchdog functionality */
	us_wd_start();
	return 0;
}

static int drbcc_wd_release(struct inode *inode, struct file *file)
{
	/* FIXME: Start automatic watchdog functionality */
	us_wd_stop();
	return 0;
}

static const struct file_operations drbcc_wd_fops = {
	.owner   		= THIS_MODULE,
	.unlocked_ioctl	= drbcc_wd_ioctl,
	.open			= drbcc_wd_open,
	.release		= drbcc_wd_release,
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

	// TODO: Set timeout

	ret = misc_register(&drbcc_wd_miscdev);

	if (ret) {
		ERR(BWD "Cannot register miscdev on minor=%d (err=%d)\n", drbcc_wd_miscdev.minor, ret);
	}

	kernel_wd_start();

	return ret;
}

static void __exit drbcc_wd_cleanup_module(void) 
{
	DBGF("Unload HydraIP DRBCC Watchdog driver: %s.\n", __FUNCTION__);

	misc_deregister(&drbcc_wd_miscdev);	
}

module_init(drbcc_wd_init_module);
module_exit(drbcc_wd_cleanup_module);
MODULE_AUTHOR("Christina Quast");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DRBCC Watchdog driver");
