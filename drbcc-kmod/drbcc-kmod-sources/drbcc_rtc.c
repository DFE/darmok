/** 
*   \file 	drbcc_rtc.c	
*   \brief	module converting time between userspace rtc representation and board controller packet struct 	
*   \author 	Christina Quast
*
* (C) 2009 DResearch Digital Media Systems GmbH
*
*/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/rtc.h>
#include <linux/capability.h>
#include <asm/uaccess.h>
#include <asm/string.h>
#include <asm/types.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>

#include "drbcc.h"
//#include "debug.h"


#define MODULE_NAME "drbcc-rtc"

#define MINOR_NR_RTC 	135

#define BRTC 		"[DRBCC-RTC] "

// struct cdev cdev;
static struct platform_device *drbcc_rtc = NULL;

/* RTC packet response format: <mid> <sec> <min> <hour> <day> <date> <month> <year> <epoch> */
/**
*  \enum PKTF_t
*  \brief mapping between postition in packet data array and signification
*/
typedef enum { SEC, MIN, HOUR, DAY, DATE, MONTH, YEAR, EPOCH } PKTF_t;	/* Field in packet.data */

/**
*	Get time from the board controller rtc and fill in into struct passed by hwclock or another userspace program.
*	\param	rtc_tm		struct representing time
*	\return negative value on failure, 0 on success 
*/
int get_rtc_time(struct device *dev, struct rtc_time *rtc_tm)
{
	int ret = 0;
	DEFINE_SEMAPHORE(sem);
	struct bcc_packet pkt = { 
		.cmd 		=  DRBCC_REQ_RTC_READ,
		.payloadlen	= 0,
		.sem		= &sem,
	};
	DBG("Request time from microcontroller.");	
		
/* TODO: remove second parameter in transmit_packet function */
	if ((ret = transmit_packet(&pkt, RSP_CMD(pkt.cmd))) < 0) {
		ERR(BRTC "Error while trying to send message.");
		return -EFAULT;
	}

	if(pkt.cmd != DRBCC_IND_RTC_READ) {
		if(pkt.cmd == DRBCC_TIMEOUT) {
			ERR("Waiting for answer timed out.");
		} else {
			DBGF("Received message with wrong response type: %x", pkt.cmd);
		}
		return -EFAULT;
	}

	rtc_tm->tm_sec =  bcd2bin(pkt.data[SEC]);
	rtc_tm->tm_min  = bcd2bin(pkt.data[MIN]);
	rtc_tm->tm_hour = bcd2bin(pkt.data[HOUR]);
	rtc_tm->tm_wday = bcd2bin(pkt.data[DAY]);
	rtc_tm->tm_mday = bcd2bin(pkt.data[DATE]);
	rtc_tm->tm_mon = bcd2bin(pkt.data[MONTH]) - 1; 
	rtc_tm->tm_year = bcd2bin(pkt.data[YEAR]) + 2000 - 1900;  
/* TODO: epoch?? */

	DBGF("______%d %d %d %d %d %d______", rtc_tm->tm_sec, rtc_tm->tm_min, rtc_tm->tm_hour,
		rtc_tm->tm_mday, rtc_tm->tm_mon, rtc_tm->tm_year);
	
	return 0;
}

/**
*	Set time passed by hwclock or another userspace program and set it in the board controller rtc
*	\param	rtc_tm		struct representing time
*	\return negative value on failure, 0 on success 
*/
int set_rtc_time(struct device *dev, struct rtc_time *rtc_tm)
{
	int month, year, ret = 0;
	DEFINE_SEMAPHORE(sem);
	struct bcc_packet pkt = { 
		.cmd 		=  DRBCC_REQ_RTC_SET,
		.payloadlen	= 0,
	/* TODO: put semaphore declaration into core */
	};

	DBG("Set RTC time through mircocontroller.");	

	pkt.data[SEC] = bin2bcd(rtc_tm->tm_sec);
	pkt.data[MIN] = bin2bcd(rtc_tm->tm_min);
	pkt.data[HOUR] = bin2bcd(rtc_tm->tm_hour);
	pkt.data[DAY] = bin2bcd(rtc_tm->tm_wday);
	pkt.data[DATE] = bin2bcd(rtc_tm->tm_mday);
	
	month = rtc_tm->tm_mon+1;
	pkt.data[MONTH] = bin2bcd(month);
	
	year = 	rtc_tm->tm_year + 1900 - 2000;
	pkt.data[YEAR] = bin2bcd(year);

/* TODO: remove second parameter in transmit_packet function */
	if ((ret = transmit_packet(&pkt, RSP_CMD(pkt.cmd))) < 0) {
		ERR(BRTC "Error while trying to send message.");
		return -EFAULT;
	}

	if(pkt.cmd != DRBCC_IND_RTC_READ) {
		DBGF("Received message with wrong response type: %x", pkt.cmd);
		return -EFAULT;
	}

	return 0;
}


/* int drbcc_rtc_open(struct inode * ino, struct file * file) {	
	DBG("Open Function of rtc module called.");	
	
	return 0;
}

int drbcc_rtc_close(struct inode * inode, struct file * file)
{
	DBG("Closing operation of rtc module called.");

	return 0;
} */

/**
*	Called by userspace to set and read time through a system call. 
*	\param	inode		pointer to metadata structure of file	
*	\param	file		structure representing open file in kernel space 
*	\param	cmd		system call value	
*	\param  arg		arguments to system call	
*	
*	\return value passed by the tty's generic ioctl function
*/
//static int drbcc_rtc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
static int drbcc_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
//	int ret = 0;
	/* TODO: if(Sig___) { ..}  */
	printk("HydraIP DRBCC driver: %s.\n", __FUNCTION__);

	switch(cmd) {
		/* case RTC_RD_TIME:	// read time and date from RTC 
		{
			struct rtc_time rtc_tm;

			memset(&rtc_tm, 0, sizeof(struct rtc_time));			
			// TODO: check return value 
			ret = get_rtc_time(NULL, &rtc_tm);
			if (copy_to_user((struct rtc_time*)arg, &rtc_tm, sizeof(struct rtc_time))) {
				ERR("Copying to user failed.");
				return -EFAULT;
			}
			return ret;	
		}

		case RTC_SET_TIME:	// set the RTC 
		{
			struct rtc_time rtc_tm;
			
			if(!capable(CAP_SYS_TIME)) {
				return -EPERM;				
			}			

			if(copy_from_user(&rtc_tm, (struct rtc_time*)arg, sizeof(struct rtc_time))) {
				ERR("Copying from user failed.");
				return -EFAULT;
			}
			// TODO: check return value 
			ret = set_rtc_time(NULL, &rtc_tm);
			return ret;
		} */
		default:
		{
			// TODO: we don't support ioctl calls?
			DBGF("IOCTL cmd called that I didn't know: %u", cmd);
			return -ENOIOCTLCMD;
		}
	}	

//	return 0;
}

static const struct rtc_class_ops drbcc_rtc_ops = {
        //roc 	= drbcc_rtc_proc,
//	.open    = drbcc_rtc_open,
//	.release = drbcc_rtc_close, 
	.read_time 	= get_rtc_time,
	.set_time	= set_rtc_time,
	.ioctl  	= drbcc_rtc_ioctl,
};

static int drbcc_rtc_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct rtc_device *rtc;

        rtc = rtc_device_register(MODULE_NAME, &plat_dev->dev, &drbcc_rtc_ops, THIS_MODULE);
        if (IS_ERR(rtc)) {
		printk(KERN_INFO "Failed to register RTC %s\n", MODULE_NAME);
                ret = PTR_ERR(rtc);
                return ret;
        }
	
	platform_set_drvdata(plat_dev, rtc);	

	return ret;
}

static int __devexit drbcc_rtc_remove(struct platform_device *plat_dev)
{
	struct rtc_device *rtc = platform_get_drvdata(plat_dev);

	rtc_device_unregister(rtc);

	return 0;
}


static struct platform_driver drbcc_rtc_drv = {
        .probe  = drbcc_rtc_probe,
        .remove = __devexit_p(drbcc_rtc_remove),
        .driver = {	
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
        },
};

static int __init drbcc_rtc_init_module(void) {
	int ret;

	printk("HydraIP DRBCC RTC driver: %s.\n", __FUNCTION__);

	if((ret = platform_driver_register(&drbcc_rtc_drv))) {
		return ret;
	}
	
	if((drbcc_rtc = platform_device_alloc(MODULE_NAME, 0)) == NULL) {
		ret = -ENOMEM;
		goto exit_driver_unregister;
	}

	if((ret = platform_device_add(drbcc_rtc))) {
		goto exit_device_unregister; 
	}

	return 0;

	/* Register device handle in kernel*/
/*	cdev_init(&cdev, &drbcc_rtc_fops);
	cdev.owner = THIS_MODULE;

	ret = add_device_entry(&cdev, MINOR_NR_RTC, "drbcc-rtc");	*/

exit_device_unregister:
	platform_device_unregister(drbcc_rtc);
	platform_device_put(drbcc_rtc);

exit_driver_unregister:
	platform_driver_unregister(&drbcc_rtc_drv);

	return ret;

}

static void __exit drbcc_rtc_cleanup_module(void) {
	printk("Unload HydraIP DRBCC RTC driver: %s.\n", __FUNCTION__);

	platform_device_unregister(drbcc_rtc);
	platform_driver_unregister(&drbcc_rtc_drv);

	/* cdev_del(&cdev);
	DBGF("cdev_del: %p", &cdev);

	remove_device_entry(MINOR_NR_RTC); */
}

module_init(drbcc_rtc_init_module);
module_exit(drbcc_rtc_cleanup_module);
MODULE_AUTHOR("Christina Quast");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DRBCC RAW driver");