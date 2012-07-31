/** 
*   \file 	drbcc_raw.c	
*   \brief	compatibility module converting userspace requests to board controller packet struct 	
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
#include <linux/kfifo.h>
#include <asm/uaccess.h>
#include <asm/string.h>
#include <asm/types.h>

#include "drbcc.h"

#define MINOR_NR_RAW 	123

#define BRAW 		"[DRBCC-RAW] "
#define READ_BUF	4096

static struct cdev cdev;
static struct kfifo *fifo;
DECLARE_MUTEX(transaction);

static struct toggle toggle_t = {
	.rx = 0,
	.tx = 0,
};

/* Note: Callback Method for asynchronous messages should kfree
* retrieved memory
*/
void drbcc_rcv_msg_async(struct bcc_packet *pkt)
{
	unsigned char buf[MSG_MAX_BUFF];
	int ret;
 	
	pkt->cmd |= TOGGLE_SHIFT(toggle_t.rx);
	ret = serialize_packet(pkt, buf);
	down_interruptible(&transaction);
	if (kfifo_put(fifo, buf, ret) < ret) {
		ERR("Putting data to fifo failed.");
	}
	up(&transaction);
	DBGF("Put asynchronous packet with cmd = %x into fifo. fifolen = %d", pkt->cmd, kfifo_len(fifo));

	kfree(pkt);
}

/**
*	Called on open call on device file. Does the initialization work for the drbcc-raw device. 
*	\param 	ino	struct representing the file's inode entry in file system
*	\param 	file 	struct representing file with it's current state
*	\return 0 on success, -EAGAIN if allocating memory for fifo buffer failed 	
*/
int drbcc_raw_open(struct inode * ino, struct file * file) {	
	DEFINE_SPINLOCK(spin);
	DBG("Open through drbcc_raw_open, init new kfifo.");
	fifo = kfifo_alloc(READ_BUF, GFP_KERNEL, &spin);
	if(!fifo) {
		DBG("Allocating fifo buffer in open function failed.");
		return -EAGAIN;
	}
	memset(fifo->buffer, 0,  kfifo_len(fifo));

	register_async_callback(drbcc_rcv_msg_async);
	
	return 0;
}

/**
*	Do cleanup on close call on device file.
*	\param 	ino	struct representing the file's inode entry in file system
*	\param 	file 	struct representing file with it's current state
*	\return negative error value if fifo buffer in private data pointer was null, 0 else
*/
int drbcc_raw_close(struct inode * inode, struct file * file)
{
	DBG("Closing operation for device called.");
	if(!fifo) {
		DBG("Fifo buffer was never initialized.");
	}
	kfifo_free(fifo);

	return 0;
}

/**
*	Passes data read from drbcc_core as buffer to userspace.
*	\param 	file 	struct representing file with it's current state
*	\param	user	pointer to userspace buffer
*	\param	num1	maximum bytes to fill into buffer	
*	\param	loff	file offset where to start reading from	
*	\return negative error value on failure retrieving data from fifo, bytes copied to userspace on success 
*/
ssize_t drbcc_raw_read(struct file * file, char __user * user, size_t num1, loff_t * loff) {
	size_t ret = 0;
	size_t count = 0;
	unsigned int fifo_len;
	unsigned char local_buf[READ_BUF];

	if(!fifo) {
		DBG("Fifo buffer could not be retrieved.");
	}

	if((fifo_len = kfifo_len(fifo)) <= 0) {
		//DBGF("Error No. %d: No bytes available in fifo.", fifo_len);
		goto out;
	}

	printk("HydraIP DRBCC driver: %s.\n", __FUNCTION__);
	//PRINTKN(fifo->buffer, fifo->size);
	DBGF("fifo contains %d elements in its buffer", kfifo_len(fifo));

	count = min(num1, fifo_len);

	if ((ret = kfifo_get(fifo, local_buf, count)) < 0) {
		ERR("Failed reading bytes from fifo buffer.");
		ret = -EFAULT;
		goto out;
	}
	
	DBGF("kfifo_Get returned: %d.", ret);

	if (copy_to_user(user, local_buf, ret)) {
		ERR("Copying to local buffer failed.");
		ret = -EFAULT;
		goto out;
	}	
	ret = count;
	kfifo_reset(fifo);
	
  out:
	return ret;
}

/**
*	Transforms data buffer into drbcc message struct and passes data to drbcc_core to send through serial driver.
*	Fills ACK messages and result of the request into local fifo buffer.
*	\param 	file 	struct representing file with it's current state
*	\param	user	pointer to userspace buffer
*	\param	num1	maximum bytes to fill into buffer	
*	\param	loff_t	file offset where to start reading from	
*	\return negative error on failure, 0 otherwise 
*/

ssize_t drbcc_raw_write (struct file * file, const char __user * user, size_t size, loff_t * loff) {
	struct bcc_packet pkt = { 0 };
	int ret = 0;
	unsigned char buf[MSG_MAX_BUFF];
 	
	if(!fifo) {
		DBG("Fifo buffer could not be retrieved.");
	}

	printk("HydraIP DRBCC driver: %s.\n", __FUNCTION__);

	if ( size > MSG_MAX_BUFF ) {
		ERR(BRAW "Can't send message that is bigger than maximal message size.");	
		return -EFAULT;
	}
	
	if ((ret = copy_from_user(&buf, user, size))) {
		ERR("Copying data from user failed: %d not copied.\n", ret);
		return -EFAULT;
	}

	DBG("Copied form user: ");
	PRINTKN(buf, size);

	if((ret = deserialize_packet(buf, &pkt, size)) < 0) {
		ERR("Deserializing packet failed because errno %x", ret);
		return -EFAULT;
	}

	if((pkt.cmd & ~TOGGLE_BITMASK) == DRBCC_ACK || (pkt.cmd & ~TOGGLE_BITMASK) == DRBCC_SYNC_ANSWER) {
		DBG("Received ACK message.");
		TOGGLE_BIT(toggle_t.rx);
		return size;
	}
	
	if((pkt.cmd & ~TOGGLE_BITMASK) == DRBCC_SYNC) {
		DBG("Received SYNC message.");
		if (kfifo_put(fifo, create_ack_buf(1, buf), ACK_LEN) < ACK_LEN) {
			ERR("Putting ACK to fifo failed.");
			return -EFAULT;
		}
		toggle_t.rx = 0;
		toggle_t.tx = 0;
		return size;
	}

	if(((pkt.cmd & ~TOGGLE_BITMASK) != DRBCC_SYNC) && (pkt.cmd & TOGGLE_BITMASK) != TOGGLE_SHIFT(toggle_t.tx)) {
		ERR("Packet with wrong toggle bit received, putting fake ACK into fifo.");
		if (kfifo_put(fifo, create_ack_buf(TOGGLE_SHIFT(toggle_t.tx), buf), ACK_LEN) < ACK_LEN) {
			ERR("Putting ACK to fifo failed.");
			return -EFAULT;
		}
/*		if (kfifo_put(fifo, create_sync_buf(buf), SYNC_LEN) < SYNC_LEN) {
			ERR("Putting sync message to fifo failed.");
			return -EFAULT;
		}*/
/*		toggle_t.rx = 0;
		toggle_t.tx = 1;*/
		return size;
	}
	
/*	pkt.cmd = buf[0];
	memcpy(pkt.data, &buf[1], size-1); */

	DBGF("Copied buf (%d elements): cmd = %x, data:", size, pkt.cmd);
	PRINTKN(pkt.data, pkt.payloadlen);

	/*  */
	pkt.cmd = (pkt.cmd & ~TOGGLE_BITMASK);
	DBGF("raw wants to send packet with cmd %d (%x) and to receive packet with cmd %d (%x)", pkt.cmd, pkt.cmd, 
		RSP_CMD(pkt.cmd), RSP_CMD(pkt.cmd));
	if ((ret = transmit_packet(&pkt, RSP_CMD(pkt.cmd))) < 0) {
		ERR(BRAW "Error while trying to send message.");
		DBGF("Error code: %d", ret);
		return -EAGAIN;
	}

	if (pkt.cmd == DRBCC_TIMEOUT) {
		ERR("Waiting for answer timed out.");
		return -EAGAIN;
	}

	if (kfifo_put(fifo, create_ack_buf(toggle_t.tx, buf), ACK_LEN) < ACK_LEN) {
		ERR("Putting ACK to fifo failed.");
		return -EFAULT;
	}

	DBGF("Put ACK packet into fifo. fifolen = %d", kfifo_len(fifo));
	TOGGLE_BIT(toggle_t.tx);

	if(pkt.cmd != DRBCC_CMD_ILLEGAL) {
		pkt.cmd |= TOGGLE_SHIFT(toggle_t.rx);
		ret = serialize_packet(&pkt, buf);
		if (kfifo_put(fifo, buf, ret) < ret) {
			ERR("Putting data to fifo failed.");
			return -EFAULT;
		}
		DBGF("Put packet with cmd = %x into fifo. fifolen = %d", pkt.cmd, kfifo_len(fifo));
	}	

//	DBG("Content of the fifo: ");	
//	PRINTKN(fifo->buffer, fifo->size);
	return size;
}


int drbcc_raw_ioctl(struct inode * ino, struct file * file, unsigned int cmd, unsigned long arg) {
	/* TODO: if(Sig___) { ..}  */
	printk("HydraIP DRBCC driver: %s.\n", __FUNCTION__);
	DBGF("Cmd: %d, arg: %ld ", cmd, arg );
	return 0;
}

static const struct file_operations drbcc_raw_fops = {
	.owner   = THIS_MODULE,
	.open    = drbcc_raw_open,
	.read    = drbcc_raw_read,
	.write   = drbcc_raw_write,
	.release = drbcc_raw_close, 
	.ioctl   = drbcc_raw_ioctl
};

int drbcc_raw_init_module(void) {
	int ret;
	printk("HydraIP DRBCC RAW driver: %s.\n", __FUNCTION__);

	/* Register device handle in kernel*/
	cdev_init(&cdev, &drbcc_raw_fops);
	cdev.owner = THIS_MODULE;

	ret = add_device_entry(&cdev, MINOR_NR_RAW, "drbcc-raw");

	return ret;

}

void drbcc_raw_cleanup_module(void) {
	printk("Unload HydraIP DRBCC RAW driver: %s.\n", __FUNCTION__);

	cdev_del(&cdev);
	DBGF("cdev_del: %p", &cdev);

	remove_device_entry(MINOR_NR_RAW);
}

module_init(drbcc_raw_init_module);
module_exit(drbcc_raw_cleanup_module);
MODULE_AUTHOR("Christina Quast");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DRBCC RAW driver");
