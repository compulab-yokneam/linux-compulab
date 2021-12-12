/* mhitty.h */

#ifndef MHITTY_H_
#define MHITTY_H_

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>

#define DELAY_TIME		HZ * 2	/* 2 seconds per character */
#define TINY_DATA_CHARACTER	't'

#define MHI_TTY_MAJOR		240	/* experimental range */
#define MHI_TTY_MINORS		1	/* only have 1 device */

#define MHI_TTY_DRIVER_NAME "mhi_tty"

struct tty_buf {
	void *data;
	size_t len;
	struct list_head node;
};

struct tty_chan {
	wait_queue_head_t wq;
	spinlock_t lock;
	struct list_head pending; /* user space waiting to read */
	struct tty_buf *cur_buf; /* current buffer user space reading */
	size_t rx_size;
};

struct mhi_serial {
	struct tty_struct *tty;		/* pointer to the tty for this device */
	atomic_t open_count;	    /* number of times this port has been opened */
	struct semaphore sem;		/* locks this structure */

	/* for tiocmget and tiocmset functions */
	int			msr;		/* MSR shadow */
	int			mcr;		/* MCR shadow */

	/* for ioctl fun */
	struct serial_struct	serial;
	wait_queue_head_t	wait;
	struct async_icount	icount;
};

struct tty_dev {
	struct list_head node;
	dev_t devt;
	struct device *dev;
	struct mhi_device *mhi_dev;
	const char *chan;
	struct mutex mutex; /* sync open and close */
	struct tty_chan ul_chan;
	struct tty_chan dl_chan;
	size_t mtu;
	bool enabled;
	void *ipc_log;
    bool sahara; 

	struct tty_driver *mhi_tty_driver;

	struct mhi_serial tty_table;	/* initially all NULL */
	struct tty_port tport;

    int dev_seq;
	bool removed;
};

struct mhi_tty_drv {
	struct list_head head;
	struct mutex lock;
};

void mhi_tty_return_data(struct tty_dev *tty_dev, u8 *data, u32 data_size);

extern enum MHI_DEBUG_LEVEL msg_lvl;

#ifdef CONFIG_MHI_DEBUG

#define IPC_LOG_LVL (MHI_MSG_LVL_VERBOSE)
#define MHI_UCI_IPC_LOG_PAGES (25)

#else

#define IPC_LOG_LVL (MHI_MSG_LVL_ERROR)
#define MHI_UCI_IPC_LOG_PAGES (1)

#endif

#define MSG_VERB(fmt, ...)

extern bool debug;
extern int debug_level;

#define MSG_LOG(fmt, ...) do { \
		if (debug && debug_level <= MHI_MSG_LVL_INFO) \
			pr_err("[I][%s] " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define MSG_ERR(fmt, ...) do { \
		if (debug && debug_level <= MHI_MSG_LVL_ERROR) \
			pr_err("[E][%s] " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define DEFAULT 1

#if DEFAULT
#define MSG_DEFAULT(fmt, ...) do { \
		pr_info("[D][%s] " fmt, __func__, ##__VA_ARGS__);\
} while (0)
#else
#define MSG_DEFAULT(fmt, ...)
#endif

#endif /* MHITTY_H_ */
