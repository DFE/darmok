/**
* \file 	debug.h
* \brief 	Some debug macros
*/

#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifndef DEBUG
#define DEBUG 
#endif

#ifndef ERR
#define ERR(vfmt, args...) { 	(void)printk(KERN_ERR "*****AIEEEE: %s [%s:%d] " vfmt "\n", \
				__FUNCTION__, __FILE__, __LINE__, ##args); }
#endif

#ifdef DEBUG

#ifdef BCC_PKT_TIMEOUT
#undef BCC_PKT_TIMEOUT
#define BCC_PKT_TIMEOUT	HZ 
#endif

# ifndef __FUNCTION__
# define __FUNCTION__ __func__
# endif

# define DBGF(fmt, args...) { 	(void)printk(KERN_INFO "*****DEBUG: %s [%s:%d] " fmt "\n", \
				__FUNCTION__, __FILE__, __LINE__, ##args); }

#define DBG(x) 	DBGF("%s", x)

#define PRINTKN(buf, n) {	do {								\
					int i;							\
					for(i = 0; i < n; i++){					\
						if((buf[i] > 0x7f)||(buf[i] < 0x19)) {		\
							printk("? (%x) ", buf[i]);		\
						} else { printk("%c (%x) ", buf[i], buf[i]); }	\
					}							\
					printk("\n"); 						\
				} while (0); }

/* void printkn(const unsigned char *buf, int n) {
	int i;

	for(i = 0; i < n; i++){
		printk("%c (%x) ", buf[i], buf[i]);
	}
	printk("\n");
}*/
#else
/* void printkn(const unsigned char *buf, int n); */
#define PRINTKN(buf, n)

# define DBGF(fmt, args...)
# define DBG(x)
#endif /* DEBUG */

#endif /* __DEBUG_H__ */
