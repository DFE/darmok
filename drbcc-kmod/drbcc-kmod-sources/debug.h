/**
* \file 	debug.h
* \brief 	Some debug macros
*/

#ifndef __DEBUG_H__
#define __DEBUG_H__

//#ifndef DEBUG_DRBCC
//#define DEBUG_DRBCC
//#endif

#ifndef ERR
#define ERR(vfmt, args...) { 	(void)printk(KERN_NOTICE "*****AIEEEE: %s [%s:%d] " vfmt "\n", \
				__FUNCTION__, __FILE__, __LINE__, ##args); }
#endif

#ifdef DEBUG_DRBCC

#ifdef BCC_PKT_TIMEOUT
#undef BCC_PKT_TIMEOUT
#define BCC_PKT_TIMEOUT	HZ 
#endif

# ifndef __FUNCTION__
# define __FUNCTION__ __func__
# endif

# define DBGF(fmt, args...) { 	(void)printk(KERN_DEBUG "*****DEBUG: %s [%s:%d] " fmt "\n", \
				__FUNCTION__, __FILE__, __LINE__, ##args); }

#define DBG(x) 	DBGF("%s", x)

#define PRINTKN(buf, n) {	do {								\
					int i;							\
					for(i = 0; i < n; i++){					\
						if((buf[i] > 0x7f)||(buf[i] < 0x19)) {		\
							printk(KERN_DEBUG "? (%x) ", buf[i]);		\
						} else { printk(KERN_DEBUG "%c (%x) ", buf[i], buf[i]); }	\
					}							\
					printk(KERN_DEBUG "\n"); 						\
				} while (0); }

/* void printkn(const unsigned char *buf, int n) {
	int i;

	for(i = 0; i < n; i++){
		printk(KERN_DEBUG "%c (%x) ", buf[i], buf[i]);
	}
	printk(KERN_DEBUG "\n");
}*/
#else
/* void printkn(const unsigned char *buf, int n); */
#define PRINTKN(buf, n)

# define DBGF(fmt, args...)
# define DBG(x)
#endif /* DEBUG_DRBCC */

#endif /* __DEBUG_H__ */
