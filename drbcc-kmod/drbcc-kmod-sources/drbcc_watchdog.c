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

#define DEFAULT_TIMEOUT 60                      /* default timeout in seconds */
// TODO: Rly? Or rather delete it?
static  int timeout = DEFAULT_TIMEOUT;

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
	

    DBG(BWD "Send timeout to board controller.");
	
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
	if (t < 1 ||t > 0xFFFF) 
		return -EINVAL;
	
	timeout = t;
	return 0;
}

static long drbcc_wd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) 
{
	int ret = -ENOTTY;
	int new_timeout;

	switch (cmd) {
	case WDIOC_SETOPTIONS:

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

static const struct file_operations drbcc_wd_fops = {
	.owner   		= THIS_MODULE,
	.unlocked_ioctl	= drbcc_wd_ioctl,
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

//	TODO: DBGF("initialized. timeout=%d sec", timeout);	

	return ret;
}

static void __exit drbcc_wd_cleanup_module(void) 
{
	DBGF("Unload HydraIP DRBCC Watchdog driver: %s.\n", __FUNCTION__);

	// TODO: Stop wd ?
	misc_deregister(&drbcc_wd_miscdev);	
}

module_init(drbcc_wd_init_module);
module_exit(drbcc_wd_cleanup_module);
MODULE_AUTHOR("Christina Quast");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DRBCC Watchdog driver");
