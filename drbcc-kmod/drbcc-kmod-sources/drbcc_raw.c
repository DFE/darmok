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
#include <linux/semaphore.h>
#include <linux/slab.h>

#include "drbcc.h"

#define MINOR_NR_RAW 	123

#define BRAW 		"[DRBCC-RAW] "
#define READ_BUF	4096

static struct cdev cdev;
static struct kfifo fifo;
DEFINE_SPINLOCK(spin);

static struct toggle toggle_t = {
	.rx = 0,
	.tx = 0,
};

#define ACK_BUF_BY_TBIT(t) (t==0)?(&(ACK_BUF_RX_0)):(&(ACK_BUF_RX_1))

/* Note: Callback Method for asynchronous messages should kfree
* retrieved memory
*/
void drbcc_rcv_msg_async(struct bcc_packet *pkt)
{
	unsigned char buf[MSG_MAX_BUF];
	int ret;
	
	pkt->cmd |= SHIFT_TBIT(toggle_t.rx);

	ret = serialize_packet(pkt, buf);
	if (kfifo_in_spinlocked(&fifo, buf, ret, &spin) < ret) {
		ERR("Putting data to fifo failed.");
	}
	DBGF("Put asynchronous packet with cmd = 0x%x into fifo. fifolen = %d", pkt->cmd, kfifo_len(&fifo));

	kfree(pkt);
}

/**
*	Called on open call on device file. Does the initialization work for the drbcc-raw device. 
*	\param 	ino	struct representing the file's inode entry in file system
*	\param 	file 	struct representing file with it's current state
*	\return 0 on success, -EAGAIN if allocating memory for fifo buffer failed 	
*/
int drbcc_raw_open(struct inode * ino, struct file * file) {	
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
	register_async_callback(NULL);	// Unregister callback method

	DBG("Closing operation for device called.");

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
	unsigned char local_buf[MSG_MAX_BUF];

	if((fifo_len = kfifo_len(&fifo)) <= 0) {
		//DBGF("Error No. %d: No bytes available in fifo.", fifo_len);
		goto out;
	}

	DBGF("HydraIP DRBCC driver: %s.\n", __FUNCTION__);
	//PRINTKN(fifo->buffer, fifo->size);
	DBGF("fifo contains %d elements in its buffer", kfifo_len(&fifo));

	count = min(num1, fifo_len);

	if ((ret = kfifo_out_spinlocked(&fifo, local_buf, count, &spin)) < 0) {
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
	kfifo_reset(&fifo);
	
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
	unsigned char buf[MSG_MAX_BUF];
 	
	DBGF("HydraIP DRBCC driver: %s.\n", __FUNCTION__);

	if ( size > MSG_MAX_BUF ) {
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
		ERR("Deserializing packet failed because errno 0x%x", ret);
		return -EFAULT;
	}
	

	if(((CMD_NO_TBIT(pkt.cmd) == DRBCC_ACK) && (CMD_TBIT(pkt.cmd) == SHIFT_TBIT(toggle_t.rx))) || (CMD_NO_TBIT(pkt.cmd) == DRBCC_SYNC_ANSWER)) {
		DBG("Received ACK message.");
		toggle_t.rx = !toggle_t.rx;
		return size;
	}
	
	if(pkt.cmd == (DRBCC_SYNC | TOGGLE_BITMASK)) {
		DBG("Received SYNC message.");
		if (kfifo_in_spinlocked(&fifo, &(ACK_BUF_RX_1), ACK_LEN, &spin) < ACK_LEN) {
			ERR("Putting ACK to fifo failed.");
			return -EFAULT;
		}
		toggle_t.rx = 0;
		toggle_t.tx = 0;
		return size;
	}


	if(CMD_TBIT(pkt.cmd) != SHIFT_TBIT(toggle_t.tx)) {
		ERR("Packet with wrong toggle bit received, putting fake ACK into fifo.");
		if (kfifo_in_spinlocked(&fifo, ACK_BUF_BY_TBIT(!toggle_t.tx), ACK_LEN, &spin) < ACK_LEN) {
			ERR("Putting ACK to fifo failed.");
			return size+2;
		}

		return size;
	}

	DBGF("Copied buf (%d elements): cmd = 0x%x, data:", size, pkt.cmd);
	PRINTKN(pkt.data, pkt.payloadlen);

	CMD_DEL_TBIT((&pkt));	// FIXME!!!!
	DBGF("raw wants to send packet with cmd %d (0x%x) and to receive packet with cmd %d (0x%x)", pkt.cmd, pkt.cmd, 
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

	if (kfifo_in_spinlocked(&fifo, ACK_BUF_BY_TBIT(toggle_t.tx), ACK_LEN, &spin) < ACK_LEN) {
		ERR("Putting ACK to fifo failed.");
		return -EFAULT;
	}

	DBGF("Put ACK packet into fifo. fifolen = %d", kfifo_len(&fifo));
	toggle_t.tx = !toggle_t.tx;

	if(pkt.cmd != DRBCC_CMD_ILLEGAL) {
		pkt.cmd |= SHIFT_TBIT(toggle_t.rx);
		ret = serialize_packet(&pkt, buf);
		if (kfifo_in_spinlocked(&fifo, buf, ret, &spin) < ret) {
			ERR("Putting data to fifo failed.");
			return -EFAULT;
		}
		DBGF("Put packet with cmd = 0x%x into fifo. fifolen = %d", pkt.cmd, kfifo_len(&fifo));
	}	

	return size;
}

long drbcc_raw_ioctl(struct file * file, unsigned int cmd, unsigned long arg) {
	/* TODO: if(Sig___) { ..}  */
	DBGF("HydraIP DRBCC driver: %s.\n", __FUNCTION__);
	DBGF("Cmd: %d, arg: %ld ", cmd, arg );
	return 0;
}

static const struct file_operations drbcc_raw_fops = {
	.owner   		= THIS_MODULE,
	.open    		= drbcc_raw_open,
	.read    		= drbcc_raw_read,
	.write   		= drbcc_raw_write,
	.release 		= drbcc_raw_close, 
	.unlocked_ioctl	= drbcc_raw_ioctl
};

int drbcc_raw_init_module(void) {
	int ret;
	DBGF("HydraIP DRBCC RAW driver: %s.\n", __FUNCTION__);

	/* Register device handle in kernel*/
	cdev_init(&cdev, &drbcc_raw_fops);
	cdev.owner = THIS_MODULE;

	ret = add_device_entry(&cdev, MINOR_NR_RAW, "drbcc-raw");

	DBG("Init new kfifo.");
	ret = kfifo_alloc(&fifo, MSG_MAX_BUF, GFP_KERNEL);
	if (ret) {
		DBG("Allocating fifo buffer in open function failed.");
	}

	return ret;
}

void drbcc_raw_cleanup_module(void) {
	DBGF("Unload HydraIP DRBCC RAW driver: %s.\n", __FUNCTION__);
	
	kfifo_free(&fifo);

	cdev_del(&cdev);
	DBGF("cdev_del: %p", &cdev);

	remove_device_entry(MINOR_NR_RAW);
}

module_init(drbcc_raw_init_module);
module_exit(drbcc_raw_cleanup_module);
MODULE_AUTHOR("Christina Quast");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DRBCC RAW driver");
