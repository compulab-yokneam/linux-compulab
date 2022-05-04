
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,12,0)
#include <linux/sched/signal.h>
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(5,6,0)
#include <linux/mod_devicetable.h>
#else
#include "../inc/devicetable.h"
#endif

#include "../inc/ipc_logging.h"
#include "../inc/mhi.h"
#include "../core/mhi_internal.h"
#include "../inc/msm_mhi_dev.h"

#include "mhi_tty.h"

bool debug = false;

module_param(debug, bool, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH );
MODULE_PARM_DESC(debug,"enable/disable driver logging");

int debug_level = MHI_MSG_LVL_INFO;
module_param(debug_level, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH );
MODULE_PARM_DESC(debug_level,"driver logging level");

static struct mhi_tty_drv mhi_tty_drv;

#define MAX_MHITTY_DEVICES (256)

static DECLARE_BITMAP(mhitty_seq, MAX_MHITTY_DEVICES);
static DECLARE_BITMAP(mhisahara_seq, MAX_MHITTY_DEVICES);

struct tty_dev *mhi_tty_get_dev(struct mhi_device *mhi_dev)
{
    struct tty_dev *tty_dev = NULL;

	mutex_lock(&mhi_tty_drv.lock);
    
    tty_dev = mhi_device_get_devdata(mhi_dev);
    if (tty_dev == NULL) {
       	MSG_LOG("device was removed!\n");
        mutex_unlock(&mhi_tty_drv.lock);
        return NULL;
    }

    mutex_unlock(&mhi_tty_drv.lock);

    return tty_dev;
}    

/**
 * @brief queue income messages 
 *
 * @param[in ]            tty_dev				struct tty_dev
 */
static int mhi_queue_inbound(struct tty_dev *tty_dev)
{
	struct mhi_device *mhi_dev = tty_dev->mhi_dev;
	int nr_trbs = mhi_get_no_free_descriptors(mhi_dev, DMA_FROM_DEVICE);
	size_t mtu = tty_dev->mtu;
	void *buf;
	struct tty_buf *tty_buf;
	int ret = -EIO, i;

	MSG_LOG("nr_trbs %d mtu %ld\n", nr_trbs, (unsigned long)mtu);

	for (i = 0; i < nr_trbs; i++) {
		buf = kmalloc(mtu + sizeof(*tty_buf), GFP_KERNEL);
		if (!buf) {
	        MSG_ERR("failed to allocate memory %ld\n", (long)(mtu + sizeof(*tty_buf)));
			return -ENOMEM;
        }

		tty_buf = buf + mtu;
		tty_buf->data = buf;

		MSG_LOG("Allocated buf %d of %d size %lu\n", i, nr_trbs, (unsigned long)mtu);

		ret = mhi_queue_transfer(mhi_dev, DMA_FROM_DEVICE, buf, mtu,
					 MHI_EOT);
	    
        MSG_LOG("mhi_queue_transfer i %d ret 0x%x\n", i, ret);
		
        if (ret) {
			kfree(buf);
			MSG_ERR("Failed to queue buffer %d\n", i);
			return ret;
		}
	}

	MSG_LOG("ret 0x%x\n", ret);
	return ret;
}

/**
 * @brief return rx data to TTY
 *
 * @param[in ]        tty_dev        TTY device
 * @param[in ]        data           data buffer
 * @param[in ]        data_size      data length
 */
void mhi_tty_return_data(struct tty_dev *tty_dev, u8 *data, u32 data_size)
{
	struct mhi_serial *tiny = NULL;
	struct tty_struct *tty;
	int copied = 0;

    MSG_LOG("mhi_tty_return_data %p len %d", data, data_size);

    if ((data == NULL) || (data_size == 0)) {
        
		MSG_LOG("no data return!");
        
        return;
    }

	tiny = &tty_dev->tty_table;

	if (!tiny)
		return;

#if 0
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
		data, data_size, true);
#endif

	tty = tiny->tty;

	/* check if the tty open or not*/
	if ((tty == NULL) || (tty->port == NULL))
		return;

	/* send the data to the tty layer for users to read.  This doesn't
	 * actually push the data through unless tty->low_latency is set */
	tty_buffer_request_room(tty->port, data_size);
	copied = tty_insert_flip_string(tty->port, data, data_size);
	// MSG_DEFAULT("copied %d data_size %\n", copied, data_size);
	if (copied != data_size) {
		MSG_ERR("copied %d != data_size %d!\n", copied, data_size);
	}
	tty_flip_buffer_push(tty->port);
}

/**
 * @brief open TTY device
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        file			file handle
 */
static int tiny_open(struct tty_struct *tty, struct file *file)
{
	struct mhi_serial *tiny;
	struct tty_dev *tty_dev;
    int ret, count;
	struct tty_chan *dl_chan;
	struct tty_buf *buf_itr, *tmp;

	tty_dev = (struct tty_dev *)dev_get_drvdata(tty->dev);

	tty->driver_data = NULL;
    if (tty_dev->removed) {
       	MSG_LOG("device was removed!\n");
        return -ENODEV;
    }

	/* get the serial object associated with this tty pointer */
	tiny = &tty_dev->tty_table;

	count = atomic_inc_return(&tiny->open_count);

#if 0
    /* opened? */
    if (count != 1) {
       	MSG_ERR("device was opened!\n");
        return -EBUSY;
    }     

	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;
#endif

	MSG_LOG("0x%p\n", tty->dev);

	sema_init(&tiny->sem, 1);

	down(&tiny->sem);

	/* save our structure within the tty structure */
	tty->driver_data = tiny;
	tiny->tty = tty;

	tty_dev->tport.tty = tty;
	tty->port = &tty_dev->tport;
    
	if (count == 1) {
        
        if (tty_dev->sahara) {
            /* for SAHARA fw downloading, without this, TTY chops the data in 2K each */
            set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);

	    tty->port->low_latency = 1;	
        }

		/* this is the first time this port is opened */
		/* do any hardwarelist initialization needed here */
		MSG_LOG("Starting channel\n");
		if (tty_dev->mhi_dev == NULL) {
			ret = -ENOTCONN;
			goto error_open_chan;
		}
		
		ret = mhi_prepare_for_transfer(tty_dev->mhi_dev);
		if (ret) {
			MSG_ERR("Error starting transfer channels\n");
			goto error_open_chan;
		}
        
		MSG_LOG("mhi_queue_inbound\n");
		ret = mhi_queue_inbound(tty_dev);
		if (ret)
			goto error_rx_queue;
	}

	up(&tiny->sem);
	return 0;

error_rx_queue:
	dl_chan = &tty_dev->dl_chan;
	mhi_unprepare_from_transfer(tty_dev->mhi_dev);
	list_for_each_entry_safe(buf_itr, tmp, &dl_chan->pending, node) {
		list_del(&buf_itr->node);
		kfree(buf_itr->data);
	}

error_open_chan:
	atomic_dec(&tiny->open_count);
	up(&tiny->sem);
	return ret;
}

/**
 * @brief close TTY device helper
 *
 * @param[in ]        tiny          TTY device
 */
static void do_close(struct mhi_serial *tiny, struct tty_dev *tty_dev)
{
    int count;

   	MSG_LOG("Enter!\n");

	down(&tiny->sem);

	if (!atomic_read(&tiny->open_count)) {
		/* port was never opened */
		goto exit;
	}

	count = atomic_dec_return(&tiny->open_count);

   	MSG_LOG("open_count %d\n", count);

	if (count == 0) {

		if (tty_dev->removed) {
   			MSG_LOG("free tty_dev\n");

			kfree(tty_dev);
		} else {
   			MSG_LOG("mhi_unprepare_from_transfer!\n");
			/* The port is being closed by the last user. */
			/* Do any hardware specific stuff here */
	    	mhi_unprepare_from_transfer(tty_dev->mhi_dev);
		}

	}
exit:
	up(&tiny->sem);
}

/**
 * @brief close TTY device
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        file			file handle
 */
static void tiny_close(struct tty_struct *tty, struct file *file)
{
	struct mhi_serial *tiny = tty->driver_data;
	struct tty_dev *tty_dev = NULL;

   	MSG_LOG("Enter!\n");

	tty_dev = (struct tty_dev *)dev_get_drvdata(tty->dev);

	if (tiny) 
		do_close(tiny, tty_dev);
}	

/**
 * @brief write TTY device
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        file			file handle
 */
static int tiny_write(struct tty_struct *tty,
	const unsigned char *buffer, int count)
{
	struct mhi_serial *tiny = tty->driver_data;
    struct mhi_device *mhi_dev;
	struct tty_dev *tty_dev = NULL;
	struct tty_chan *tty_chan;
	size_t bytes_xfered = 0;
	int ret = -EINVAL; 
	int nr_avail;

	tty_dev = (struct tty_dev *)dev_get_drvdata(tty->dev);

	MSG_LOG("enter tty_dev 0x%px\n", tty_dev);

    if (tty_dev->removed) {
       	MSG_LOG("device was removed!\n");
        return -ENODEV;
    }

	mhi_dev = tty_dev->mhi_dev;

	tty_chan = &tty_dev->ul_chan;

	if (!buffer || !count)
		return -EINVAL;

	if (!tiny)
		return -ENODEV;

	if (tty->index >= MHI_TTY_MINORS)
		return -ENODEV;

	down(&tiny->sem);

	if (!atomic_read(&tiny->open_count)) {
       	MSG_LOG("device not opened!\n");

		/* port was not opened */
		goto exit;
	}

	/* confirm channel is active */
	spin_lock_bh(&tty_chan->lock);
	if (!tty_dev->enabled) {
		spin_unlock_bh(&tty_chan->lock);
		ret = -ERESTARTSYS;
		goto exit;
	}

	MSG_LOG("Enter: to xfer:%u bytes\n", count);

	while (count) {
		size_t xfer_size;
		void *kbuf;
		enum MHI_FLAGS flags;

		spin_unlock_bh(&tty_chan->lock);

		/* wait for free descriptors */
		ret = wait_event_interruptible(tty_chan->wq,
			(!tty_dev->enabled) ||
			(nr_avail = mhi_get_no_free_descriptors(mhi_dev,
				DMA_TO_DEVICE)) > 0);

		if (ret == -ERESTARTSYS || !tty_dev->enabled) {
			MSG_LOG("Exit signal caught for node or not enabled\n");
			ret = -ERESTARTSYS;
			goto exit;
		}

		xfer_size = min_t(size_t, count, tty_dev->mtu);
		MSG_LOG("xfer_size:%lu bytes\n", (unsigned long)xfer_size);
		kbuf = kmalloc(xfer_size, GFP_KERNEL);
		if (!kbuf) {
			MSG_ERR("Failed to allocate memory %lu\n", (unsigned long)xfer_size);
			ret = -ENOMEM;
			goto exit;
		}

	    memcpy(kbuf, buffer, count);

		spin_lock_bh(&tty_chan->lock);

		/* if ring is full after this force EOT */
		if (nr_avail > 1 && (count - xfer_size))
			flags = MHI_CHAIN;			   

		else
			flags = MHI_EOT;

		MSG_LOG("dev 0x%p enabled %d\n", tty_dev, tty_dev->enabled);

		if (tty_dev->enabled) {
#if 0
			MSG_LOG("write buffer\n");
			print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1,
				kbuf, xfer_size, true);
#endif
			ret = mhi_queue_transfer(mhi_dev, DMA_TO_DEVICE, kbuf,
				xfer_size, flags);
			MSG_LOG("mhi_queue_transfer flags %d ret 0x%x\n", flags, ret);
		}
		else
			ret = -ERESTARTSYS;

		if (ret) {
			kfree(kbuf);
			goto sys_interrupt;
		}

		bytes_xfered += xfer_size;
		count -= xfer_size;
		buffer += xfer_size;
	}

	spin_unlock_bh(&tty_chan->lock);
	MSG_LOG("Exit: Number of bytes xferred:%lu\n", (unsigned long)bytes_xfered);

	up(&tiny->sem);

	return bytes_xfered;

sys_interrupt:
	spin_unlock_bh(&tty_chan->lock);

exit:
	up(&tiny->sem);
	return ret;
}

/**
 * @brief get writing space for TTY device
 *
 * @param[in ]        tty           TTY device
 */
static int tiny_write_room(struct tty_struct *tty) 
{
	struct mhi_serial *tiny = tty->driver_data;
    struct mhi_device *mhi_dev;
	struct tty_dev *tty_dev;
	int room = -EINVAL;
    int nr_trbs = 0;

	tty_dev = (struct tty_dev *)dev_get_drvdata(tty->dev);
	MSG_LOG("tty_dev 0x%px removed %d\n", tty_dev, tty_dev->removed);

    if (tty_dev->removed) {
       	MSG_LOG("device was removed!\n");
        return -ENODEV;
    }

    mhi_dev = tty_dev->mhi_dev;

	if (!tiny)
		return -ENODEV;

	down(&tiny->sem);
	
	if (!atomic_read(&tiny->open_count)) {
		/* port was not opened */
		goto exit;
	}
	nr_trbs = mhi_get_no_free_descriptors(mhi_dev, DMA_TO_DEVICE);
	MSG_LOG("nr_trbs %d\n", nr_trbs);

    if (nr_trbs) {
	    /* calculate how much room is left in the device */
	    room = tty_dev->mtu;
    } else {
        room = 0;
    }    
exit:
	up(&tiny->sem);
	MSG_LOG("room: %d\n", room);
	return room;
}

#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

/**
 * @brief set properties for TTY
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        old_termios   properties
 */
static void tiny_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
	unsigned int cflag;

	cflag = tty->termios.c_cflag;

	/* check that they really want us to change something */
	if (old_termios) {
		if ((cflag == old_termios->c_cflag) &&
		    (RELEVANT_IFLAG(tty->termios.c_iflag) == 
		     RELEVANT_IFLAG(old_termios->c_iflag))) {
			MSG_LOG(" - nothing to change...\n");
			return;
		}
	}

	/* get the byte size */
	switch (cflag & CSIZE) {
		case CS5:
			MSG_LOG(" - data bits = 5\n");
			break;
		case CS6:
			MSG_LOG(" - data bits = 6\n");
			break;
		case CS7:
			MSG_LOG(" - data bits = 7\n");
			break;
		default:
		case CS8:
			MSG_LOG(" - data bits = 8\n");
			break;
	}
	
	/* determine the parity */
	if (cflag & PARENB)
		if (cflag & PARODD) {
			MSG_LOG(" - parity = odd\n");
        }    
		else {
			MSG_LOG(" - parity = even\n");
        }     
	else {
		MSG_LOG(" - parity = none\n");
    }

	/* figure out the stop bits requested */
	if (cflag & CSTOPB) {
		MSG_LOG(" - stop bits = 2\n");
    }
    else {
		MSG_LOG(" - stop bits = 1\n");
    }

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS) {
		MSG_LOG(" - RTS/CTS is enabled\n");
    }     
	else {
		MSG_LOG(" - RTS/CTS is disabled\n");
    }

	/* determine software flow control */
	/* if we are implementing XON/XOFF, set the start and 
	 * stop character in the device */
	if (I_IXOFF(tty) || I_IXON(tty)) {
		//unsigned char stop_char  = STOP_CHAR(tty);
		//unsigned char start_char = START_CHAR(tty);

		/* if we are implementing INBOUND XON/XOFF */
		if (I_IXOFF(tty)) {
			MSG_LOG(" - INBOUND XON/XOFF is enabled");
        }
        else {
			MSG_LOG(" - INBOUND XON/XOFF is disabled");
        }

		/* if we are implementing OUTBOUND XON/XOFF */
		if (I_IXON(tty)) {
			MSG_LOG(" - OUTBOUND XON/XOFF is enabled");
        }
        else {
			MSG_LOG(" - OUTBOUND XON/XOFF is disabled");
        }    
	}

	/* get the baud rate wanted */
	MSG_LOG(" - baud rate = %d", tty_get_baud_rate(tty));
}

/* Our fake UART values */
#define MCR_DTR		0x01
#define MCR_RTS		0x02
#define MCR_LOOP	0x04
#define MSR_CTS		0x08
#define MSR_CD		0x10
#define MSR_RI		0x20
#define MSR_DSR		0x40

/**
 * @brief get signal settings for TTY
 *
 * @param[in ]        tty           TTY device
 */
static int tiny_tiocmget(struct tty_struct *tty)
{
	struct mhi_serial *tiny = tty->driver_data;

	unsigned int result = 0;
	unsigned int msr = tiny->msr;
	unsigned int mcr = tiny->mcr;

	result = ((mcr & MCR_DTR)  ? TIOCM_DTR  : 0) |	/* DTR is set */
             ((mcr & MCR_RTS)  ? TIOCM_RTS  : 0) |	/* RTS is set */
             ((mcr & MCR_LOOP) ? TIOCM_LOOP : 0) |	/* LOOP is set */
             ((msr & MSR_CTS)  ? TIOCM_CTS  : 0) |	/* CTS is set */
             ((msr & MSR_CD)   ? TIOCM_CAR  : 0) |	/* Carrier detect is set*/
             ((msr & MSR_RI)   ? TIOCM_RI   : 0) |	/* Ring Indicator is set */
             ((msr & MSR_DSR)  ? TIOCM_DSR  : 0);	/* DSR is set */

	return result;
}

/**
 * @brief set/clear signal for TTY
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        set           set
 * @param[in ]        clear         clear
 */
static int tiny_tiocmset(struct tty_struct *tty,
                         unsigned int set, unsigned int clear)
{
	struct mhi_serial *tiny = tty->driver_data;
	unsigned int mcr = tiny->mcr;

	if (set & TIOCM_RTS)
		mcr |= MCR_RTS;
	if (set & TIOCM_DTR)
		mcr |= MCR_RTS;

	if (clear & TIOCM_RTS)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_DTR)
		mcr &= ~MCR_RTS;

	/* set the new MCR value in the device */
	tiny->mcr = mcr;
	return 0;
}

/**
 * @brief device I/O control
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        cmd           command
 * @param[in ]        arg           argument
 */
#define tiny_ioctl tiny_ioctl_tiocgserial
static int tiny_ioctl(struct tty_struct *tty, 
                      unsigned int cmd, unsigned long arg)
{
	struct mhi_serial *tiny = tty->driver_data;

	if (cmd == TIOCGSERIAL) {
		struct serial_struct tmp;

		if (!arg)
			return -EFAULT;

		memset(&tmp, 0, sizeof(tmp));

		tmp.type		= tiny->serial.type;
		tmp.line		= tiny->serial.line;
		tmp.port		= tiny->serial.port;
		tmp.irq			= tiny->serial.irq;
		tmp.flags		= ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ;
		tmp.xmit_fifo_size	= tiny->serial.xmit_fifo_size;
		tmp.baud_base		= tiny->serial.baud_base;
		tmp.close_delay		= 5*HZ;
		tmp.closing_wait	= 30*HZ;
		tmp.custom_divisor	= tiny->serial.custom_divisor;
		tmp.hub6		= tiny->serial.hub6;
		tmp.io_type		= tiny->serial.io_type;

		if (copy_to_user((void __user *)arg, &tmp, sizeof(struct serial_struct)))
			return -EFAULT;
		return 0;
	}
	return -ENOIOCTLCMD;
}
#undef tiny_ioctl

/**
 * @brief device I/O control
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        cmd           command
 * @param[in ]        arg           argument
 */
#define tiny_ioctl tiny_ioctl_tiocmiwait
static int tiny_ioctl(struct tty_struct *tty, 
                      unsigned int cmd, unsigned long arg)
{
	struct mhi_serial *tiny = tty->driver_data;

	if (cmd == TIOCMIWAIT) {
		DECLARE_WAITQUEUE(wait, current);
		struct async_icount cnow;
		struct async_icount cprev;

		cprev = tiny->icount;
		while (1) {
			add_wait_queue(&tiny->wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			remove_wait_queue(&tiny->wait, &wait);

			/* see if a signal woke us up */
			if (signal_pending(current))
				return -ERESTARTSYS;

			cnow = tiny->icount;
			if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
			    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
				return -EIO; /* no change => error */
			if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
			    ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
			    ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
			    ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
				return 0;
			}
			cprev = cnow;
		}

	}
	return -ENOIOCTLCMD;
}
#undef tiny_ioctl

/**
 * @brief device I/O control
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        cmd           command
 * @param[in ]        arg           argument
 */
#define tiny_ioctl tiny_ioctl_tiocgicount
static int tiny_ioctl(struct tty_struct *tty, 
                      unsigned int cmd, unsigned long arg)
{
	struct mhi_serial *tiny = tty->driver_data;

	if (cmd == TIOCGICOUNT) {
		struct async_icount cnow = tiny->icount;
		struct serial_icounter_struct icount;

		icount.cts	= cnow.cts;
		icount.dsr	= cnow.dsr;
		icount.rng	= cnow.rng;
		icount.dcd	= cnow.dcd;
		icount.rx	= cnow.rx;
		icount.tx	= cnow.tx;
		icount.frame	= cnow.frame;
		icount.overrun	= cnow.overrun;
		icount.parity	= cnow.parity;
		icount.brk	= cnow.brk;
		icount.buf_overrun = cnow.buf_overrun;

		if (copy_to_user((void __user *)arg, &icount, sizeof(icount)))
			return -EFAULT;
		return 0;
	}
	return -ENOIOCTLCMD;
}
#undef tiny_ioctl

/**
 * @brief device I/O control
 *
 * the real tiny_ioctl function.  The above is done to get the small functions in the book
 *
 * @param[in ]        tty           TTY device
 * @param[in ]        cmd           command
 * @param[in ]        arg           argument
 */
static int tiny_ioctl(struct tty_struct *tty, 
                      unsigned int cmd, unsigned long arg)
{

	switch (cmd) {
	case TIOCGSERIAL:
		return tiny_ioctl_tiocgserial(tty, cmd, arg);
	case TIOCMIWAIT:
		return tiny_ioctl_tiocmiwait(tty, cmd, arg);
	case TIOCGICOUNT:
		return tiny_ioctl_tiocgicount(tty, cmd, arg);
	}

	return -ENOIOCTLCMD;
}

/**
 * @brief remove TTY device
 *
 * @param[in ]            mhi_dev				struct mhi_device
 */
static void mhi_tty_remove(struct mhi_device *mhi_dev)
{
	struct tty_dev *tty_dev;
	struct tty_driver *mhi_tty_driver;
	struct mhi_serial *tiny = NULL;

	MSG_DEFAULT("enter\n");

    tty_dev = mhi_tty_get_dev(mhi_dev);    

    if (tty_dev == NULL || tty_dev->removed) {
       	MSG_LOG("device was removed!\n");
        return;
    }

	mutex_lock(&mhi_tty_drv.lock);
	mutex_lock(&tty_dev->mutex);
    
    mhi_tty_driver = tty_dev->mhi_tty_driver;

	/* disable the node */
	spin_lock_irq(&tty_dev->dl_chan.lock);
	spin_lock_irq(&tty_dev->ul_chan.lock);
	tty_dev->enabled = false;
	spin_unlock_irq(&tty_dev->ul_chan.lock);
	spin_unlock_irq(&tty_dev->dl_chan.lock);

	wake_up(&tty_dev->dl_chan.wq);
	wake_up(&tty_dev->ul_chan.wq);

	/* delete the node to prevent new opens */
	tty_dev->dev = NULL;
	list_del(&tty_dev->node);

	MSG_LOG("mhi_tty_driver 0x%px\n", tty_dev->mhi_tty_driver);
	if (tty_dev->mhi_tty_driver) {

        tty_port_destroy(&tty_dev->tport);
		tty_unregister_device(mhi_tty_driver, 0);

		tty_unregister_driver(tty_dev->mhi_tty_driver);

		MSG_LOG("mhi_unprepare_from_transfer!\n");
		mhi_unprepare_from_transfer(mhi_dev);

		tty_dev->mhi_tty_driver = NULL;
	}

    if (tty_dev->sahara) {
    	clear_bit(tty_dev->dev_seq, mhisahara_seq);
    } else {
    	clear_bit(tty_dev->dev_seq, mhitty_seq);
    }

	/* set the removed flag */
	MSG_LOG("tty_dev 0x%px removed is true!\n", tty_dev);
	tty_dev->removed = true;

    mhi_device_set_devdata(mhi_dev, NULL);

	/* safe to free memory only if all file nodes are closed */
	tiny = &tty_dev->tty_table;
	MSG_LOG("open_count %d\n", atomic_read(&tiny->open_count));
	if (!atomic_read(&tiny->open_count)) {

		mutex_unlock(&tty_dev->mutex);
		mutex_destroy(&tty_dev->mutex);
		MSG_LOG("free tty_dev!\n");
		kfree(tty_dev);
		mutex_unlock(&mhi_tty_drv.lock);
		goto exit;
	}

	mutex_unlock(&tty_dev->mutex);
	mutex_unlock(&mhi_tty_drv.lock);

exit:
	MSG_DEFAULT("exit\n");
}

static struct tty_operations serial_ops = {
	.open = tiny_open,
	.close = tiny_close,
	.write = tiny_write,
	.write_room = tiny_write_room,
	.set_termios = tiny_set_termios,
	.tiocmget = tiny_tiocmget,
	.tiocmset = tiny_tiocmset,
	.ioctl = tiny_ioctl,
};

/**
 * @brief probe TTY device
 *
 * @param[in ]            mhi_dev				struct mhi_device
 * @param[in ]            id					struct mhi_device_id
 */
static int mhi_tty_probe(struct mhi_device *mhi_dev,
	const struct mhi_device_id *id)
{
	struct tty_dev *tty_dev;
	struct tty_driver *mhi_tty_driver;
	int dir, ret;
	struct device *dev;
    int seq;
	struct device *parent = mhi_dev->dev.parent;

	MSG_DEFAULT("enter\n");

	MSG_LOG("dev_id: 0x%x domain: 0x%x bus: 0x%x slot: 0x%x\n",
        mhi_dev->dev_id, mhi_dev->domain, mhi_dev->bus, mhi_dev->slot);

	tty_dev = kzalloc(sizeof(*tty_dev), GFP_KERNEL);
	if (!tty_dev) {
		MSG_ERR("no memory!\n");
        ret = -ENOMEM;
		goto exit0;
	}

	mutex_init(&tty_dev->mutex);
	tty_dev->mhi_dev = mhi_dev;

    if (strcmp(id->chan, "SAHARA") == 0) { 
        tty_dev->sahara = true;
    }

    /* try to get a sequence number for device */
	mutex_lock(&tty_dev->mutex);
    if (tty_dev->sahara) {
	    seq = find_first_zero_bit(mhisahara_seq, MAX_MHITTY_DEVICES);
    } else {
	    seq = find_first_zero_bit(mhitty_seq, MAX_MHITTY_DEVICES);
    }
    
	if (seq >= MAX_MHITTY_DEVICES) {
		MSG_ERR("No available tty device number %d!\n", seq);
        mutex_unlock(&tty_dev->mutex);
		ret = -ENOSPC;
        goto exit1;
	}

    tty_dev->dev_seq = seq;
    if (tty_dev->sahara) {
    	set_bit(seq, mhisahara_seq);
    } else {
    	set_bit(seq, mhitty_seq);
    }
	mutex_unlock(&tty_dev->mutex);

	mutex_lock(&tty_dev->mutex);
	mutex_lock(&mhi_tty_drv.lock);

	for (dir = 0; dir < 2; dir++) {
		struct tty_chan *tty_chan = (dir) ?
			&tty_dev->ul_chan : &tty_dev->dl_chan;
		spin_lock_init(&tty_chan->lock);
		init_waitqueue_head(&tty_chan->wq);
		INIT_LIST_HEAD(&tty_chan->pending);
	};

	tty_dev->mtu = min_t(size_t, id->driver_data, mhi_dev->mtu);
	mhi_device_set_devdata(mhi_dev, tty_dev);
	tty_dev->enabled = true;

	MSG_LOG("dev 0x%p enabled %d\n", tty_dev, tty_dev->enabled);

	list_add(&tty_dev->node, &mhi_tty_drv.head);
	mutex_unlock(&mhi_tty_drv.lock);
	mutex_unlock(&tty_dev->mutex);

	/* allocate the tty driver */
	mhi_tty_driver = tty_alloc_driver(MHI_TTY_MINORS, TTY_DRIVER_DYNAMIC_DEV);

	if (IS_ERR(mhi_tty_driver)) {
		MSG_ERR("failed to alloc memory for tty driver!\n");
		ret = -ENOMEM;
        goto exit2;
	}

	tty_dev->mhi_tty_driver = mhi_tty_driver;

	/* initialize the tty driver */
	mhi_tty_driver->owner = THIS_MODULE;
	mhi_tty_driver->driver_name = "mhi_tty";
	mhi_tty_driver->name = "mhitty";
    if (tty_dev->sahara)
	    mhi_tty_driver->name = "mhiqdl";
	mhi_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	mhi_tty_driver->flags |= TTY_DRIVER_REAL_RAW;
	mhi_tty_driver->name_base = tty_dev->dev_seq;
	mhi_tty_driver->init_termios = tty_std_termios;
	mhi_tty_driver->init_termios.c_lflag &= ~ECHO;
	mhi_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(mhi_tty_driver, &serial_ops);

	tty_port_init(&tty_dev->tport);

	/* set the limit to higher value as the recent kernel versions*/
	tty_buffer_set_limit(&tty_dev->tport, (640 * 1024UL));

	tty_port_link_device(&tty_dev->tport, mhi_tty_driver, 0);

	/* register the tty driver */
	ret = tty_register_driver(mhi_tty_driver);
	if (ret) {
		MSG_ERR("failed to register tiny tty driver 0x%x", ret);
		put_tty_driver(mhi_tty_driver);
		mhi_tty_driver = NULL;
		tty_dev->mhi_tty_driver = mhi_tty_driver;
		goto exit2;
	}

	sema_init(&tty_dev->tty_table.sem, 1);

    atomic_set(&tty_dev->tty_table.open_count, 0);

	dev = tty_register_device(mhi_tty_driver, 0, NULL);

	dev_set_drvdata(dev, tty_dev);

    ret = sysfs_create_link(&dev->kobj, &parent->kobj, mhi_dev->dev.kobj.name);

    MSG_LOG("sysfs_create_link 0x%x\n", ret);

    MSG_DEFAULT("probe chan %s successful!\n", mhi_dev->chan_name);

	return 0;

exit2:
    if (tty_dev->sahara) {
    	clear_bit(tty_dev->dev_seq, mhisahara_seq);
    } else {
    	clear_bit(tty_dev->dev_seq, mhitty_seq);
    }
exit1:
    kfree(tty_dev);
exit0:
    return ret;
};

static void mhi_ul_xfer_cb(struct mhi_device *mhi_dev,
	struct mhi_result *mhi_result)
{
	struct tty_dev *tty_dev = NULL;
	struct tty_chan *tty_chan;

    tty_dev = mhi_device_get_devdata(mhi_dev);
    if (tty_dev == NULL) {
       	MSG_LOG("device was removed!\n");
	    kfree(mhi_result->buf_addr);
        return;
    }

    tty_chan = &tty_dev->ul_chan;
	MSG_LOG("status:%d xfer_len:%zu\n", mhi_result->transaction_status,
		mhi_result->bytes_xferd);

	kfree(mhi_result->buf_addr);
	if (!mhi_result->transaction_status)
		wake_up(&tty_chan->wq);
}

/**
 * @brief device to host transfer callback
 *
 * @param[in ]            mhi_dev				struct mhi_device
 * @param[in ]            mhi_result			struct mhi_result
 */
static void mhi_dl_xfer_cb(struct mhi_device *mhi_dev,
	struct mhi_result *mhi_result)
{
	struct tty_dev *tty_dev;
	//struct tty_chan *tty_chan = &tty_dev->dl_chan;
	//unsigned long flags;
	//struct tty_buf *buf;
    int ret;

	if ((mhi_dev == NULL) || (mhi_result == NULL))
		return;

    tty_dev = mhi_device_get_devdata(mhi_dev);
    if (tty_dev == NULL) {
       	MSG_LOG("device was removed!\n");
        return;
    }

	MSG_LOG("status:%d receive_len:%zu\n", mhi_result->transaction_status,
		mhi_result->bytes_xferd);

	if (mhi_result->transaction_status == -ENOTCONN) {
		kfree(mhi_result->buf_addr);
		return;
	}

	MSG_LOG("chan %d\n", mhi_dev->dl_chan->chan);

	MSG_LOG("mhi_tty_return_data: buf 0x%px len %ld", mhi_result->buf_addr, 
		(long)mhi_result->bytes_xferd);

	if (mhi_result->buf_addr == NULL)
		return;

    if (tty_dev) {
        /* return data */
        mhi_tty_return_data(tty_dev, mhi_result->buf_addr, mhi_result->bytes_xferd);

        /* send read request */
        if (tty_dev->enabled)
        {
            ret = mhi_queue_transfer(mhi_dev, DMA_FROM_DEVICE,
                                     mhi_result->buf_addr, tty_dev->mtu,
                                     MHI_EOT);

            MSG_LOG("mhi_queue_transfer 0x%x\n", ret);
        }
    } else {
		kfree(mhi_result->buf_addr);
        MSG_LOG("incoming tty data dropped!\n");
    }    

    //spin_lock_irqsave(&tty_chan->lock, flags);
	//list_add_tail(&buf->node, &tty_chan->pending);
	//spin_unlock_irqrestore(&tty_chan->lock, flags);

	if (mhi_dev->dev.power.wakeup)
		__pm_wakeup_event(mhi_dev->dev.power.wakeup, 0);

	//wake_up(&tty_chan->wq);
}

/* .driver_data stores max mtu */
static const struct mhi_device_id mhi_tty_match_table[] = {
	{.chan = "DIAG",.driver_data = 0x1000 },
	{.chan = "DUN",.driver_data = 0x1000 },
	{.chan = "SAHARA",.driver_data = 0x8000 },
	{},
};
MODULE_DEVICE_TABLE(mhi, mhi_tty_match_table);

static struct mhi_driver mhi_tty_driver = {
	.id_table = mhi_tty_match_table,
	.remove = mhi_tty_remove,
	.probe = mhi_tty_probe,
	.ul_xfer_cb = mhi_ul_xfer_cb,
	.dl_xfer_cb = mhi_dl_xfer_cb,
	.driver = {
		.name = MHI_TTY_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

/**
 * @brief initialize TTY 
 *
 * @param[in ]        pdev           PCIe device
 */
static int __init
mhitty_init(void)
{
	int ret;

	mutex_init(&mhi_tty_drv.lock);
	INIT_LIST_HEAD(&mhi_tty_drv.head);

	MSG_DEFAULT("Enter\n");

	ret = mhi_driver_register(&mhi_tty_driver);

	MSG_DEFAULT("mhi_driver_register 0x%x\n", ret);

	return ret;
}

/**
 * @brief exit TTY
 *
 * @param[in ]        pdev           PCIe device
 */
static void __exit
mhitty_exit(void)
{
    mhi_driver_unregister(&mhi_tty_driver);

	MSG_DEFAULT("exit\n");
}

module_init(mhitty_init);
module_exit(mhitty_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("MHI_TTY");
MODULE_DESCRIPTION("MHI TTY Driver");
MODULE_VERSION("1.9.2101.1");