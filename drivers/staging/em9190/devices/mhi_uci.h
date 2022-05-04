#ifndef _MHI_UCI_H
#define _MHI_UCI_H

struct uci_chan {
	wait_queue_head_t wq;
	spinlock_t lock;
	struct list_head pending; /* user space waiting to read */
	struct uci_buf *cur_buf; /* current buffer user space reading */
	size_t rx_size;
	struct mutex mutex_read;
};

struct uci_buf {
	void *data;
	size_t len;
	struct list_head node;
	bool sent;
};

struct uci_dev {
	struct list_head node;
	dev_t devt;
	struct device *dev;
	struct mhi_device *mhi_dev;
	const char *chan;
	struct mutex mutex; /* sync open and close */
	struct uci_chan ul_chan;
	struct uci_chan dl_chan;
	size_t mtu;
	int ref_count;
	bool enabled;
	void *ipc_log;
	u32 set_data_fmt_tid;
};

struct mhi_uci_drv {
	struct list_head head;
	struct mutex lock;
	struct class *class;
	int major;
	dev_t dev_t;
};

#define DEVICE_NAME "mhi"
#define MHI_UCI_DRIVER_NAME "mhi_uci"

extern enum MHI_DEBUG_LEVEL msg_lvl;

#ifdef CONFIG_MHI_DEBUG

#define IPC_LOG_LVL (MHI_MSG_LVL_VERBOSE)
#define MHI_UCI_IPC_LOG_PAGES (25)

#else

#define IPC_LOG_LVL (MHI_MSG_LVL_ERROR)
#define MHI_UCI_IPC_LOG_PAGES (1)

#endif

extern bool debug;
extern int debug_level;

#define MSG_VERB(fmt, ...)

#define MSG_LOG(fmt, ...) do { \
		if (debug && debug_level <= MHI_MSG_LVL_INFO) \
			pr_err("[I][%s] " fmt, __func__, ##__VA_ARGS__); \
		if (uci_dev->ipc_log && (IPC_LOG_LVL <= MHI_MSG_LVL_INFO)) \
			ipc_log_string(uci_dev->ipc_log, "[I][%s] " fmt, \
				       __func__, ##__VA_ARGS__); \
	} while (0)

#define MSG_ERR(fmt, ...) do { \
		if (debug && debug_level <= MHI_MSG_LVL_ERROR) \
			pr_err("[E][%s] " fmt, __func__, ##__VA_ARGS__); \
		if (uci_dev->ipc_log && (IPC_LOG_LVL <= MHI_MSG_LVL_ERROR)) \
			ipc_log_string(uci_dev->ipc_log, "[E][%s] " fmt, \
				       __func__, ##__VA_ARGS__); \
	} while (0)

#define DEFAULT 1

#if DEFAULT
#define MSG_DEFAULT(fmt, ...) do { \
		pr_info("[D][%s] " fmt, __func__, ##__VA_ARGS__);\
} while (0)
#else
#define MSG_DEFAULT(fmt, ...)
#endif

#define MAX_UCI_DEVICES (64)

#define ADB_HDR_LEN			24		/* ADB message header length */

#endif
