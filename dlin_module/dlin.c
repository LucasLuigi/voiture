/****************************************************************************************/
/*				 					dlin.c												*/
/*   dlin is a linux kernel drive (module) which is TTY line discipline implemented		*/
/*   for GNU/Linux operating system (kernel 3.10.17) that handles the interchange of 	*/
/*   LIN messages between the underlying UART driver and the CAN bus subsystem			*/
/*																						*/
/****************************************************************************************/

#include <linux/module.h> 		/* definitions of symbols and fonctions */
#include <linux/init.h> 		/* initialization and cleanup fonctions */
#include <linux/moduleparam.h> 	/* passing parameters */
#include <linux/skbuff.h> 		/* struct sk_buff */
#include <linux/netdevice.h> 	/* struct net_device */
#include <linux/kernel.h>		/* printk... */
#include <linux/slab.h> 		/* kmalloc() */
#include <linux/errno.h> 		/* error codes */
#include <linux/if_arp.h> 		/* CAN identifier */
#include <linux/if.h>
#include <linux/hrtimer.h>
#include <linux/rtnetlink.h> 	/*rtnl_lock and rtnl_unlock */
#include <linux/can.h>
#include <linux/sched.h> 		/*task_struct describe process/task in the system */
#include <linux/tty.h>
#include <linux/kthread.h> 		/*thread task such as stop,init... */
#include "lin_bus.h"

MODULE_AUTHOR("Egreteau Corentin, Faro Mathias, Gissot Lucas, Pleuron Jonathan");
MODULE_LICENSE("Dual BSD/GPL");

#define N_DLIN      25
#define DLIN_MAGIC	0x53CA	/* value of the variable magic in the struct dlin used as a flag */
static int baudrate;
static int maxdev =10;		/* max number of dlin channels */
module_param(maxdev,int,0); /* module's parameters */

#define DLIN_DATA_MAX			8	/* LIN data : 8 bytes max */
#define DLIN_BUFF_LEN			(1 /*break*/ + 1 /*sync*/ + 1 /*ID*/ + DLIN_DATA_MAX + 1 /*checksum*/)
#define DLIN_BUFF_BREAK			0
#define DLIN_BUFF_SYNC			1
#define DLIN_BUFF_ID			2	
#define DLIN_BUFF_DATA			3
#define DLIN_SAMPLES_PER_CHAR	10
#define DLIN_CHARS_TO_TIMEOUT	24

//// States of the DLIN channel /////
enum dstate {
	DSTATE_IDLE = 0,
	DSTATE_BREAK_SENT,
	DSTATE_ID_SENT,
	DSTATE_RESPONSE_WAIT, 		/* Wait for response */
	DSTATE_RESPONSE_WAIT_BUS, 	/* Wait for response from LIN bus
								only (CAN frames from network stack
								are not processed in this moment) */
	DSTATE_ID_RECEIVED,
	DSTATE_RESPONSE_SENT,
};

//// LIN frame structure ////
struct dlin_conf_entry {
	int dlc;		   	/* Length of data in LIN frame */
	canid_t frame_fl;	/* LIN frame flags. Passed from userspace as canid_t data type */
	u8 data[8];		    /* LIN frame data payload */
};

//// Main struct of the driver ////
struct dlin {
        int                 magic;
        struct net_device   *dev; 			/* our interface */
        spinlock_t          lock;
        
        /// LIN message buffer and actual processed data counts ///
        unsigned char		rx_buff[DLIN_BUFF_LEN]; /* LIN Rx buffer */
        unsigned char		tx_buff[DLIN_BUFF_LEN]; /* LIN Tx buffer */
	    int			        rx_expect;      /* expected number of Rx chars */
	    int			        rx_lim;         /* maximum Rx chars for current frame */
	    int			        rx_cnt;         /* message buffer Rx fill level  */
	    int			        tx_lim;         /* actual limit of bytes to Tx */
        int             	tx_cnt;         /* number of already Tx bytes */
		char            	lin_master;     /* node is a master node */
		int                 lin_baud;       /* LIN baudrate */
        int			        lin_state;		/* state */
        char			    id_to_send;		/* there is ID to be sent */
        char                data_to_send;   /* there are data to be sent */
        struct sk_buff      *tx_skb; 		/* socket buffer */
        struct task_struct  *kwthread; 		/* struct describing the kernel worker thread */
        dev_t               line; 			/* struct with major and minor numbers of the device */
        struct tty_struct 	*tty; 			/* ptr to TTY structure*/
		char				rx_len_unknown; /* We don't know how much data will be sent to us we (just guess) */
		char	            resp_len_known; /* Length of the response is known */
		
		/// FLAGS ///
		unsigned long        flags;
		#define DLF_INUSE           0     	/*channel in use */
		#define DLF_ERROR           1     	/* Parity, etc. error */
		#define DLF_RXEVENT	        2     	/* Rx wake event */
		#define DLF_TXEVENT	        3    	/* Tx wake event */
		#define DLF_MSGEVENT	  	4 		/* CAN message to sent */
		#define DLF_TMOUTEVENT		5		/* Timeout on received data */
		#define DLF_TXBUFF_RQ	    6  	  	/* Req. to send buffer to UART*/
		#define DLF_TXBUFF_INPR	    7   	/* Above request in progress */
        struct sk_buff      *tx_req_skb;	/* Socket buffer with CAN frame received from network stack*/

		
		struct dlin_conf_entry linfr_cache[LIN_ID_MAX + 1];	/*List with configurations for each of 0 to LIN_ID_MAX LIN IDs */
		wait_queue_head_t	  kwt_wq; 			/* Wait queue used by kwthread */
		struct hrtimer				rx_timer;	/* RX timeout timer */
		ktime_t	rx_timer_timeout; 				/* RX timeout timer value */
		spinlock_t           linframe_lock; 	/*frame cache and buffers lock*/
};



static struct net_device **dlin_devs; /*list of *net_devices */
static int dlin_configure_frame_cache(struct dlin *dl, struct can_frame *cf); /* important to mention */

/* Values of two parity bits in LIN Protected
   Identifier for each particular LIN ID */
const unsigned char dlin_id_parity_table[] = {
	0x80, 0xc0, 0x40, 0x00, 0xc0, 0x80, 0x00, 0x40,
	0x00, 0x40, 0xc0, 0x80, 0x40, 0x00, 0x80, 0xc0,
	0x40, 0x00, 0x80, 0xc0, 0x00, 0x40, 0xc0, 0x80,
	0xc0, 0x80, 0x00, 0x40, 0x80, 0xc0, 0x40, 0x00,
	0x00, 0x40, 0xc0, 0x80, 0x40, 0x00, 0x80, 0xc0,
	0x80, 0xc0, 0x40, 0x00, 0xc0, 0x80, 0x00, 0x40,
	0xc0, 0x80, 0x00, 0x40, 0x80, 0xc0, 0x40, 0x00,
	0x40, 0x00, 0x80, 0xc0, 0x00, 0x40, 0xc0, 0x80
};

/**
 * sltty_change_speed() -- Change baudrate of Serial device belonging
 *			   			   to particular @tty
 *
 * @tty:	Pointer to TTY to change speed for.
 * @speed:	Integer value of new speed. It is possible to
 *			assign non-standard values, i.e. those which
 *			are not defined in termbits.h.
 */
static int dlin_tty_change_speed(struct tty_struct *tty, unsigned speed)
{
	struct ktermios old_termios, termios;
	int cflag;
	mutex_lock(&tty->termios_mutex);	/*release a write lock*/
	//down_write(&tty->termios_rwsem); 	/*other version of linux*/
	old_termios = termios = tty->termios;


	cflag = CS8 | CREAD | CLOCAL | HUPCL;
	/* CREAD = Enable receiver
	   CS8 = Char size mask 8
	   CLOCAL = Ignore modem control lines
	   HUPCL = Lower modem control lines after last process closes the device (hang up)*/

	cflag &= ~(CBAUD | CIBAUD);
	/* CBAUD = Baud speed mask (4+1 bits)
	   CIBAUD = Mask for input speeds. The values for the CIBAUD bits are the same as the values for the CBAUD bits, shifted left IBSHIFT (16) bits */

	cflag |= BOTHER; // BOTHER = 0010000
	termios.c_cflag = cflag; // c_cflag => control mode flag
	termios.c_oflag = 0;     // c_oflag => output mode flag
	termios.c_lflag = 0;     // c_lflag => local mode flag

	/* Enable interrupt when UART-Break or Framing error received */
	termios.c_iflag = BRKINT | INPCK; //c_iflag => input mode flag
	/* BRKINT =  BRKINT is set, then a BREAK causes the input and output queues to be flushed
	   INPCK = Enable input parity checking */

	tty->termios = termios;

	tty_encode_baud_rate(tty, speed, speed); // Update the current termios data for the tty with the new speed settings

	if (tty->ops->set_termios)
	  tty->ops->set_termios(tty, &old_termios);//This routine allows the tty driver to be notified when device's termios settings have changed.

	mutex_unlock(&tty->termios_mutex);// Unlock a mutex that has been locked by this task previously
	//up_write(&tty->termios_rwsem); // other version of linux

	return 0;
}

/** 
 * dlin_send_canfr()	-- Send one can_frame to the network layer 
 * 
 * @dl:		pointer to LIN device 
 * @id:		CAN ID
 * @data:	CAN/LIN data
 * @len:	CAN data lenght
 */
static void dlin_send_canfr(struct dlin *dl, canid_t id, char *data, int len) {
	struct sk_buff *skb;
	struct can_frame cf;

	cf.can_id = id;
	cf.can_dlc = len;
	if (cf.can_dlc > 0){					/* if data_length code is not null */
		memcpy(&cf.data, data, cf.can_dlc); /* copy the data into the frame */
}
	skb = dev_alloc_skb(sizeof(struct can_frame)); /*allocation of a socket buffer */
	if (!skb)
		return;

  skb->dev = dl->dev;
	skb->protocol = htons(ETH_P_CAN);		/*setting up the CAN protocol*/
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	memcpy(skb_put(skb, sizeof(struct can_frame)),
	&cf, sizeof(struct can_frame));
	/**
    * skb_put - add data to a buffer
    * 
    * @skb: buffer to use
    * @len: amount of data to add
    *
    * This function extends the used data area of the buffer. If this would
    * exceed the total buffer size the kernel will panic. A pointer to the
    * first byte of the extra data is returned.
    */
	netif_rx(skb);

	/**
    * netif_rx	-- post buffer to the network code
    *      
    * @skb: buffer to post
    *
    * This function receives a packet from a device driver and queues it for
    * the upper (protocol) levels to process.  It always succeeds. The buffer
    * may be dropped during processing for congestion control or by the
    * protocol layers.
    *
    * return values:
    *      NET_RX_SUCCESS  (no congestion)
    *      NET_RX_DROP     (packet was dropped)
    *
    */

	dl->dev->stats.rx_packets++;
	dl->dev->stats.rx_bytes += cf.can_dlc;
}

/**
 * dlin_bump() -- Send data of received LIN frame (existing in dl->rx_buff)
 *		 		as CAN frame
 *
 * @dl::	pointer to LIN device
 * 
 * This function calls dlin_send_frame function.
 */
static void dlin_bump(struct dlin *dl)
{
	int len = dl->rx_cnt - DLIN_BUFF_DATA - 1;	/* without checksum */
	len = (len < 0) ? 0 : len;

	dlin_send_canfr(dl, dl->rx_buff[DLIN_BUFF_ID] & LIN_ID_MASK,
	dl->rx_buff + DLIN_BUFF_DATA, len);			/* send a can frame with data with the correct ID */
}

/*
 * Called by the driver when there's room for more data.  If we have
 * more packets to send, we send them here.
 */
 
/**
 * dlin_write_wakeup()	-- send a wake up signal
 * 
 * @ tty:	Pointer to TTY sending the signal
 * */
static void dlin_write_wakeup(struct tty_struct *tty)
{
	int actual = 0;
	int remains;
	struct dlin *dl = (struct dlin *) tty->disc_data;	/* dl takes the tty data */

	/* First make sure we're connected. */
	if (!dl || dl->magic != DLIN_MAGIC || !netif_running(dl->dev)) /* Check if we are not already connected (cf  dlin_tty_alloc) + Test if the device has been brought up. */
		return;

	set_bit(DLF_TXBUFF_RQ, &dl->flags); /* Request to send buffer to UART */
	do {
	  /* Set a bit and return its old value if the condition is false */
	  if (unlikely(test_and_set_bit(DLF_TXBUFF_INPR, &dl->flags)))
	    return;	 //the above request is in progress

		clear_bit(DLF_TXBUFF_RQ, &dl->flags);

	/* 	We need to barrier before modifying the word, since the _atomic_xxx()
		routines just tns the lock and then read/modify/write of the word.
		But after the word is updated, the routine issues an "mf" before returning,
		and since it's a function call, we don't even need a compiler barrier.*/

		smp_mb__after_clear_bit(); //< KERNEL_VERSION(3, 18, 0)
		//smp_mb__after_atomic();


		if (dl->lin_state != DSTATE_BREAK_SENT)
		  remains = dl->tx_lim - dl->tx_cnt;	/* Tx bytes remaining */

		else /*if BREAK SENT*/
		  remains = DLIN_BUFF_BREAK /* = 0 */ + 1 - dl->tx_cnt;
		  /*remains <= 0 if bytes have already been transmitted*/

		if (remains > 0) {

			actual = tty->ops->write(tty, dl->tx_buff + dl->tx_cnt,
						 dl->tx_cnt - dl->tx_lim); 	/* (tty, msg, length msg)
													actual = number of characters written */
			dl->tx_cnt += actual; 	/* bytes Tx += char written */
			remains -= actual; 		/* remains -= char written */
		}

		clear_bit(DLF_TXBUFF_INPR, &dl->flags); 	/* request finished */

		smp_mb__after_clear_bit();	/* < KERNEL_VERSION(3, 18, 0) */
		//smp_mb__after_atomic();


	} while (unlikely(test_bit(DLF_TXBUFF_RQ, &dl->flags)));	/* while no request sent */

	if ((remains > 0) && (actual >= 0)) {
		netdev_dbg(dl->dev, "dlin_write_wakeup sent %d, remains %d, waiting\n",
			   dl->tx_cnt, dl->tx_lim - dl->tx_cnt);
		return;
	}
	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);	/* TTY_DO_WRITE_WAKEUP = 5 when set call write_wakeup after queuing new */
	set_bit(DLF_TXEVENT, &dl->flags); 				/* a Tx wake event happened */
	wake_up(&dl->kwt_wq); 							/* wake up threads blocked on a waitqueue */
	netdev_dbg(dl->dev, "dlin_write_wakeup sent %d, wakeup\n", dl->tx_cnt);
}





/**
 * Dlin_tx() -- Send a can_frame to a TTY queue.
 *
 * @skb:	Pointer to Socket buffer to be sent.
 * @dev:	Network device where @skb will be sent.
 */

static netdev_tx_t dlin_tx(struct sk_buff *skb, struct net_device *dev)
{
    struct dlin *dl = netdev_priv(dev);
    struct can_frame *cf;

    if(skb->len != sizeof(struct can_frame))
    goto err_out;	/* check if our socket buffer has the good length */

    spin_lock(&dl->lock);
    if(!netif_running(dev))	/* Test if the device has been brought up*/
      {
        netdev_warn(dl->dev,"tx: interface is down\n");
        goto err_out_unlock;
      }

    cf = (struct can_frame *) skb->data;/* cast the data in lin packet to can_frame */
    if(cf->can_id & LIN_CTRL_FRAME) 	/* if we want to send a control frame ?? */
    {
      dlin_configure_frame_cache(dl,cf);
      goto free_out_unlock;
    }

    netif_stop_queue(dl->dev);

    dl->tx_req_skb = skb; 				/* link the device with th packet to txmit */
    set_bit(DLF_MSGEVENT, &dl->flags); 	/* inform that we send a frame CAN */
    wake_up(&dl->kwt_wq); 				/* wake up sleeping processes */
    spin_unlock(&dl->lock);
    return NETDEV_TX_OK;

    free_out_unlock :
    err_out_unlock :
      spin_unlock(&dl->lock);
    err_out :
      kfree_skb(skb);	/* free the skb struct that we used */
      return NETDEV_TX_OK;
}

/**
 * dlin_open()	-- Netdevice DOWN->UP
 *
 * @dev:	pointer to the network device 
 */
static int dlin_open(struct net_device *dev)
{
  struct dlin *dl = netdev_priv(dev);
  netdev_dbg(dl->dev, "%s() invoked\n",__func__); /* function for debug purpose*/

  if(dl->tty == NULL)	/* open the dev only if it is attached to tty */
  return -ENODEV;

  dl->flags &= (1 << DLF_INUSE);/* bit wise and to set the INUSE bit */
  netif_start_queue(dev); 		/* Allow upper layers to call the device hard_start_xmit routine */
    return 0;
}

/**
 * dlin_close()	-- Netdevice UP>DOWN
 *
 * @dev:	pointer to the network device 
 */
static int dlin_close(struct net_device *dev)
{
    struct dlin *dl = netdev_priv(dev);
    spin_lock_bh(&dl->lock);	/* Disables software interrupts before taking the lock, but leaves hardware interrupts enabled*/
    if(dl->tty)
    {
        /* TTY discipline is running. */
        clear_bit(TTY_DO_WRITE_WAKEUP, &dl->tty->flags); /* reset the bit of TTY */
    }
    netif_stop_queue(dev); /* Stop upper layers calling the device hard_start_xmit routine.
							  Used for flow control when transmit resources are unavailable. */
    dl->rx_expect = 0;
    dl->tx_lim    = 0;
    spin_unlock_bh(&dl->lock);

    return 0;
}

/** 
 * dlin_free_netdev()	-- Hook the destructor so we can free dlin devs at the right point in time
 * 
 * @dev: pointer to the network device 
 * */
static void dlin_free_netdev(struct net_device *dev)
{
	int i = dev->base_addr;
	free_netdev(dev);
	dlin_devs[i] = NULL;
}

/// struct of basic functions for struct net_device ///
static const struct net_device_ops dlin_netdev_ops = {
       .ndo_open            = dlin_open,
       .ndo_stop            = dlin_close,
       .ndo_start_xmit      = dlin_tx,
};

/** 
 * dlin_setup()	--struct for setting up the whole structure of the device
 * 
 * @dev: pointer to the network device 
 */
static void dlin_setup(struct net_device *dev)
{
        dev->netdev_ops     = &dlin_netdev_ops;
		dev->destructor     = dlin_free_netdev;
        dev->flags          = IFF_NOARP;		/* non ARP protocol (if.h) */
        dev->tx_queue_len   = 10;
        dev->mtu            = sizeof(struct can_frame);
        dev->type           = ARPHRD_CAN; 		/* identifier CAN for non ARP protocol (if_arp.h) */
        dev->features       = NETIF_F_HW_CSUM; 	/* checksum all pkt at hw level (netdevice.h) */
}

/**
* dlin_master_receive_buf()	-- ???
* 
* @tty: 
* @cp:	
* @fp:	
* @count:
*/
static void dlin_master_receive_buf(struct tty_struct *tty,
			      const unsigned char *cp, char *fp, int count)
{
	struct dlin *dl = (struct dlin *) tty->disc_data;

	/* Read the characters out of the buffer */
	while (count--) {
		if (fp && *fp++) {
			netdev_dbg(dl->dev, "dlin_master_receive_buf char 0x%02x ignored "
				"due marker 0x%02x, flags 0x%lx\n",
				*cp, *(fp-1), dl->flags);

			/* i.e. Real error -- not Break */
			if (dl->rx_cnt > DLIN_BUFF_BREAK) {
			  set_bit(DLF_ERROR, &dl->flags);	/* error flag set */
			  wake_up(&dl->kwt_wq);
			  return;
			}
		}

#ifndef BREAK_BY_BAUD
		/* We didn't receive Break character -- fake it! */
		if ((dl->rx_cnt == DLIN_BUFF_BREAK) && (*cp == 0x55)) {
			netdev_dbg(dl->dev, "LIN_RX[%d]: 0x00\n", dl->rx_cnt);
			dl->rx_buff[dl->rx_cnt++] = 0x00;
		}
#endif /* BREAK_BY_BAUD */

		if (dl->rx_cnt < DLIN_BUFF_LEN) {
			netdev_dbg(dl->dev, "LIN_RX[%d]: 0x%02x\n", dl->rx_cnt, *cp);
			dl->rx_buff[dl->rx_cnt++] = *cp++;
		}
	}


	if (dl->rx_cnt >= dl->rx_expect) {
	  set_bit(DLF_RXEVENT, &dl->flags);	/* RX wake event */
	  wake_up(&dl->kwt_wq);
	  netdev_dbg(dl->dev, "dlin_receive_buf count %d, wakeup\n", dl->rx_cnt);
	}
	else {
	  netdev_dbg(dl->dev, "sllin_receive_buf count %d, waiting\n", dl->rx_cnt);
	}
}


/*****************************************
 *  dlin message helper routines
 *****************************************/
 
/**
 *  dlin_report_error()	-- Report an error by sending CAN frame
 *						   with particular error flag set in can_id
 *
 * @dl:	
 * @err: Error flag to be sent.
 */
static void dlin_report_error(struct dlin *dl, int err)
{
	unsigned char *lin_buff;
	int lin_id;

	switch (err) {
	case LIN_ERR_CHECKSUM:
		dl->dev->stats.rx_crc_errors++;
		break;

	case LIN_ERR_RX_TIMEOUT:
		dl->dev->stats.rx_errors++;
		break;

	case LIN_ERR_FRAMING:
		dl->dev->stats.rx_frame_errors++;
		break;
	}

	lin_buff = (dl->lin_master) ? dl->tx_buff : dl->rx_buff;
	lin_id = lin_buff[DLIN_BUFF_ID] & LIN_ID_MASK;
	dlin_send_canfr(dl, lin_id | CAN_EFF_FLAG | (err & ~LIN_ID_MASK), NULL, 0);
}


/**
 * DLIN_configure_frame_cache() -- Configure particular entry in linfr_cache
 *
 * @dl:
 * @cf: Pointer to CAN frame sent to this driver
 *		holding configuration information
 */
static int dlin_configure_frame_cache(struct dlin *dl, struct can_frame *cf)
{
  //unsigned long flags;
  struct dlin_conf_entry *sce;

  sce =&dl->linfr_cache[cf->can_id & LIN_ID_MASK];
  /* the index corresponds to the ID of the trame CAN that we want to transmit (control ect...)
     in the array we find all the coresponding config for the LIN */
  netdev_dbg(dl->dev, "Setting frame cache with EFF CAN frame. LIN ID = %d\n",cf->can_id & LIN_ID_MASK);
  //spin lock ??
  sce->dlc = cf->can_dlc; 		/* make the correspondance between can and lin length */
  if(sce->dlc > DLIN_DATA_MAX) 	/* if the trame is too long dor the LIN protocol */
    {
      sce->dlc = DLIN_DATA_MAX; /* set up the maximum length */
    }
      sce->frame_fl = (cf->can_id & ~LIN_ID_MASK) & CAN_EFF_MASK; 	/* a voir ? + ne pas oublier les spin lock*/
      memcpy(sce->data, cf->data, cf->can_dlc); 					/* destination, source, size */
      return 0;
}

/**
 * dlin_checksum() -- Compute checksum for particular data
 *
 * @data:		Pointer to the buffer containing whole LIN
 *		 		frame (i.e. including break and sync bytes).
 * @length:	 	Length of the buffer.
 * @enhanced_fl:Flag determining whether Enhanced or Classic
 *		 		checksum should be counted.
 * 
 *CHECKSUM: There are two checksum-models available within LIN -
 *The first is the checksum including the data bytes only (specification up to Version 1.3),
 *the second one includes the identifier in addition (Version 2.0+).
 *The used checksum model is pre-defined by the application designer.
 */

static inline unsigned dlin_checksum(unsigned char *data, int length, int enhanced_fl)
{
	unsigned csum = 0;
	int i;

	if (enhanced_fl)
		i = DLIN_BUFF_ID;
	else
		i = DLIN_BUFF_DATA;

	for (; i < length; i++) { /* warning */
		csum += data[i];
		if (csum > 255)
			csum -= 255;
	}

	return ~csum & 0xff;
}

#define DLIN_STPMSG_RESPONLY		(1) /* Message will be LIN Response only */
#define DLIN_STPMSG_CHCKSUM_CLS	(1 << 1)
#define DLIN_STPMSG_CHCKSUM_ENH	(1 << 2)

/**
 * dlin_setup_msg()	-- ???
 * 
 * @dl:	
 * @mode:
 * @id:
 * @data:
 * @len:
 */
static int dlin_setup_msg(struct dlin *dl, int mode, int id,
		unsigned char *data, int len)
{
	if (id > LIN_ID_MASK)
		return -1;

	if (!(mode & DLIN_STPMSG_RESPONLY)) { /* select the responly mode, we don't expect to receive pckt*/
		dl->rx_cnt = 0; /* init counters */
		dl->tx_cnt = 0;
		dl->rx_expect = 0;
		dl->rx_lim = DLIN_BUFF_LEN;
	}

	dl->tx_buff[DLIN_BUFF_BREAK] = 0; 	/*those consts are made up to select one element of the tx_buff array */
	dl->tx_buff[DLIN_BUFF_SYNC]  = 0x55;/* SYNC: The SYNC is a standard data format byte with a value of hexadecimal 0x55.
										   LIN slaves running on RC oscillator will use the distance between a fixed amount of rising and falling edges
	                                       to measure the current bit time on the bus (the master's time normal) and to recalculate the internal baud rate.*/

	dl->tx_buff[DLIN_BUFF_ID] = id | dlin_id_parity_table[id];
	dl->tx_lim = DLIN_BUFF_DATA; 	/* = 3 because we already sent 3 bytes */

	if ((data != NULL) && len) {
		dl->tx_lim += len;	/* if we have some data to send, we extend the tx_lim by len */
		memcpy(dl->tx_buff + DLIN_BUFF_DATA, data, len);
		dl->tx_buff[dl->tx_lim] = dlin_checksum(dl->tx_buff,
		dl->tx_lim, mode & DLIN_STPMSG_CHCKSUM_ENH);	/* we add at the end of the buffer a checksum */
		dl->tx_lim++; 		/* the total length extends by one */
	}
	if (len != 0)
		dl->rx_lim = DLIN_BUFF_DATA + len + 1; /* set up the max receiving buffer */

	return 0;
}

/**
 * dlin_reset_buffs()	-- Reset buffer of the DLIN channel
 * 
 * @dl: 
 */
static void dlin_reset_buffs(struct dlin *dl)
{
	dl->rx_cnt = 0;
	dl->rx_expect = 0;
	dl->rx_lim = dl->lin_master ? 0 : DLIN_BUFF_LEN;
	dl->tx_cnt = 0;
	dl->tx_lim = 0;
	dl->id_to_send = false;
	dl->data_to_send = false;
}

/**
 * sllin_rx_validate() -- Validate received frame, i,e. check checksum
 *
 * @dl:
 */
static int dlin_rx_validate(struct dlin *dl)
{
	unsigned long flags;
	int actual_id;
	int ext_chcks_fl;
	int lin_dlc;
	unsigned char rec_chcksm = dl->rx_buff[dl->rx_cnt - 1]; /* we select the checksum in the array to validate with the calculus*/
	struct dlin_conf_entry *sce;

	actual_id = dl->rx_buff[DLIN_BUFF_ID] & LIN_ID_MASK;
	sce = &dl->linfr_cache[actual_id]; /*select the ID of the frame */

	spin_lock_irqsave(&dl->linframe_lock, flags);
	lin_dlc = sce->dlc; /*length of data in LIN frame */
	ext_chcks_fl = sce->frame_fl & LIN_CHECKSUM_EXTENDED;
	spin_unlock_irqrestore(&dl->linframe_lock, flags);

	if (dlin_checksum(dl->rx_buff, dl->rx_cnt - 1, ext_chcks_fl) !=
		rec_chcksm) {

		/* Type of checksum is configured for particular frame */
		/* We can test the checksum with the two methods (1.3) or (2.0)*/
		if (lin_dlc > 0) {
			return -1;
		} else {
			if (dlin_checksum(dl->rx_buff,	dl->rx_cnt - 1,
				!ext_chcks_fl) != rec_chcksm) {
				return -1;
			}
		}
	}
	return 0;
}

static int dlin_send_tx_buff(struct dlin *dl)
{
	struct tty_struct *tty = dl->tty;
	int remains;
	int res;

	set_bit(DLF_TXBUFF_RQ, &dl->flags); /* Request to send buffer to UART */
	do {
	    /* if false set the flag that the request above is in progress*/
		if (unlikely(test_and_set_bit(DLF_TXBUFF_INPR, &dl->flags)))
			return 0;

		clear_bit(DLF_TXBUFF_RQ, &dl->flags);

		smp_mb__after_clear_bit(); //< KERNEL_VERSION(3, 18, 0)
		//smp_mb__after_atomic();

#ifdef BREAK_BY_BAUD
		if (dl->lin_state != DSTATE_BREAK_SENT)
		  remains = dl->tx_lim - dl->tx_cnt; /*bytes remaining to Tx*/
		else
		  remains = 1;
#else
		remains = dl->tx_lim - dl->tx_cnt; 	/*bytes remaining to Tx*/
#endif

		res = tty->ops->write(tty, dl->tx_buff + dl->tx_cnt, remains);
		/* number of char written (tty, msg, length(msg)) */

		if (res < 0)
			goto error_in_write;

		remains -= res;
		dl->tx_cnt += res;	/* bytes already Tx + char just written */

		if (remains > 0) {
			set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
			res = tty->ops->write(tty, dl->tx_buff + dl->tx_cnt, remains);

			if (res < 0) {
				clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
				goto error_in_write;
			}
			remains -= res;
			dl->tx_cnt += res;
		}

		netdev_dbg(dl->dev, "dlin_send_tx_buff sent %d, remains %d\n",
				dl->tx_cnt, remains);

		clear_bit(DLF_TXBUFF_INPR, &dl->flags);	/*request process terminated*/

		smp_mb__after_clear_bit(); //< KERNEL_VERSION(3, 18, 0)
		//smp_mb__after_atomic();


	} while (unlikely(test_bit(DLF_TXBUFF_RQ, &dl->flags)));

	return 0;

error_in_write:
	clear_bit(DLF_TXBUFF_INPR, &dl->flags); /*request process terminated*/
	return -1;
}

/**
 * dlin_send_break()	--send a break field 
 * 
 * @dl:
 */
#ifdef BREAK_BY_BAUD

static int dlin_send_break(struct dlin *dl)
{
	struct tty_struct *tty = dl->tty;
	unsigned long break_baud;
	int res;

	break_baud = ((dl->lin_baud * 2) / 3);
	dlin_tty_change_speed(tty, break_baud);	/*change baudrate */
	tty->ops->flush_buffer(tty); 			/* Discard all data in the send buffer */

	/* Stop reception processus */
	dl->rx_cnt = DLIN_BUFF_BREAK;
	dl->rx_expect = DLIN_BUFF_BREAK + 1;

	/* Change the lin_state and start the sending processus */
	dl->lin_state = DSTATE_BREAK_SENT;
	res = dlin_send_tx_buff(dl);

	/* if an error occured in dlin_send_tx_buff */
	if (res < 0) {
		dl->lin_state = DSTATE_IDLE;
		return res;
	}
	return 0;
}

#else /* BREAK_BY_BAUD */

static int dlin_send_break(struct dlin *dl)
{
	struct tty_struct *tty = dl->tty;
	int retval;
	unsigned long break_baud;
	unsigned long usleep_range_min;
	unsigned long usleep_range_max;

	break_baud = ((dl->lin_baud * 2) / 3);
	dl->rx_cnt = DLIN_BUFF_BREAK;
	dl->rx_expect = DLIN_BUFF_BREAK + 1;
	dl->lin_state = DSTATE_BREAK_SENT;

	/* Do the break ourselves; Inspired by
	   http://lxr.linux.no/#linux+v3.1.2/drivers/tty/tty_io.c#L2452 */

	retval = tty->ops->break_ctl(tty, -1); /* Turn break on */
	if (retval)
		return retval;

	/* udelay(712); */
	usleep_range_min = (1000000l * DLIN_SAMPLES_PER_CHAR) / break_baud;
	usleep_range_max = usleep_range_min + 50;
	usleep_range(usleep_range_min, usleep_range_max);

	retval = tty->ops->break_ctl(tty, 0);	/* Turn break off */
	usleep_range_min = (1000000l * 1 /* 1 bit */) / break_baud;
	usleep_range_max = usleep_range_min + 30;
	usleep_range(usleep_range_min, usleep_range_max);

	tty->ops->flush_buffer(tty); 	/* Discard all data in the send buffer */

	dl->tx_cnt = DLIN_BUFF_SYNC;

	netdev_dbg(dl->dev, "Break sent.\n");
	set_bit(DLF_RXEVENT, &dl->flags);	/* Rx wake event */
	wake_up(&dl->kwt_wq);

	return 0;
}
#endif /* BREAK_BY_BAUD */

/**
 * dlin_rx_timeout_handler()
 * 
 * @hrtimer:
 */
static enum hrtimer_restart dlin_rx_timeout_handler(struct hrtimer *hrtimer)
{
	struct dlin *dl = container_of(hrtimer, struct dlin, rx_timer);

	/*
	 * Signal timeout when:
	 * master: We did not receive as much characters as expected
	 * slave: * we did not receive any data bytes at all
	 *        * we know the length and didn't receive enough            (remove)
	 */
	if ((dl->lin_master) || (dl->rx_cnt <= DLIN_BUFF_DATA) ||
		((!dl->rx_len_unknown) && (dl->rx_cnt < dl->rx_expect))) 
		{
		dlin_report_error(dl, LIN_ERR_RX_TIMEOUT);
		set_bit(DLF_TMOUTEVENT, &dl->flags);
		}

	wake_up(&dl->kwt_wq);

	return HRTIMER_NORESTART;
}

/**
 * dlin_kwthread()	-- kernel working thread
 */
static int dlin_kwthread(void *ptr)
{
	struct dlin *dl = (struct dlin *)ptr;
	struct tty_struct *tty = dl->tty;
	struct sched_param schparam = { .sched_priority = 40 };
	int tx_bytes = 0;	/* Used for Network statistics */
	unsigned long flags;
	int lin_id;
	struct dlin_conf_entry *sce;

	netdev_dbg(dl->dev, "dlin_kwthread started.\n");
	sched_setscheduler(current, SCHED_FIFO, &schparam);

	clear_bit(DLF_ERROR, &dl->flags);
	dlin_tty_change_speed(tty, dl->lin_baud);

	while (!kthread_should_stop()) {
		struct can_frame *cf;
		u8 *lin_data;
		int lin_dlc;
		u8 lin_data_buff[DLIN_DATA_MAX];


		if ((dl->lin_state == DSTATE_IDLE) && dl->lin_master &&
			dl->id_to_send) {
			if (dlin_send_break(dl) < 0) {
				/* error processing */
			}
		}

		wait_event_killable(dl->kwt_wq, kthread_should_stop() ||
			test_bit(DLF_RXEVENT, &dl->flags) ||
			test_bit(DLF_TXEVENT, &dl->flags) ||
			test_bit(DLF_TMOUTEVENT, &dl->flags) ||
			test_bit(DLF_ERROR, &dl->flags) ||
			(dl->lin_state == DSTATE_ID_RECEIVED) ||
			(((dl->lin_state == DSTATE_IDLE) ||
				(dl->lin_state == DSTATE_RESPONSE_WAIT))
				&& test_bit(DLF_MSGEVENT, &dl->flags)));

		if (test_and_clear_bit(DLF_RXEVENT, &dl->flags)) {
			netdev_dbg(dl->dev, "dlin_kthread RXEVENT\n");
		}

		if (test_and_clear_bit(DLF_ERROR, &dl->flags)) {
			unsigned long usleep_range_min;
			unsigned long usleep_range_max;
			hrtimer_cancel(&dl->rx_timer);
			netdev_dbg(dl->dev, "dlin_kthread ERROR\n");

			if (dl->lin_state != DSTATE_IDLE)
				dlin_report_error(dl, LIN_ERR_FRAMING);

			usleep_range_min = (1000000l * DLIN_SAMPLES_PER_CHAR * 10) /
						dl->lin_baud;
			usleep_range_max = usleep_range_min + 50;
			usleep_range(usleep_range_min, usleep_range_max);
			dlin_reset_buffs(dl);
			dl->lin_state = DSTATE_IDLE;
		}

		if (test_and_clear_bit(DLF_TXEVENT, &dl->flags)) {
			netdev_dbg(dl->dev, "dlin_kthread TXEVENT\n");
		}

		if (test_and_clear_bit(DLF_TMOUTEVENT, &dl->flags)) {
			netdev_dbg(dl->dev, "dlin_kthread TMOUTEVENT\n");
			dlin_reset_buffs(dl);

			dl->lin_state = DSTATE_IDLE;
		}

		switch (dl->lin_state) {
		case DSTATE_IDLE:
			if (!test_bit(DLF_MSGEVENT, &dl->flags))
				break;

			cf = (struct can_frame *)dl->tx_req_skb->data;

			/* SFF RTR CAN frame -> LIN header */
			if (cf->can_id & CAN_RTR_FLAG) {
				struct dlin_conf_entry *sce;

				netdev_dbg(dl->dev, "%s: RTR SFF CAN frame, ID = %x\n",
					__func__, cf->can_id & LIN_ID_MASK);

				sce = &dl->linfr_cache[cf->can_id & LIN_ID_MASK];
				spin_lock_irqsave(&dl->linframe_lock, flags);

				/* Is there Slave response in linfr_cache to be sent? */
				if ((sce->frame_fl & LIN_CACHE_RESPONSE)
					&& (sce->dlc > 0)) {

					netdev_dbg(dl->dev, "Sending LIN response from linfr_cache\n");

					lin_data = sce->data;
					lin_dlc = sce->dlc;
					if (lin_dlc > DLIN_DATA_MAX)
						lin_dlc = DLIN_DATA_MAX;
					memcpy(lin_data_buff, lin_data, lin_dlc);
					lin_data = lin_data_buff;
				} else {
					lin_data = NULL;
					lin_dlc = sce->dlc;
				}
				spin_unlock_irqrestore(&dl->linframe_lock, flags);

			} else { /* SFF NON-RTR CAN frame -> LIN header + LIN response */
				netdev_dbg(dl->dev, "%s: NON-RTR SFF CAN frame, ID = %x\n",
					__func__, (int)cf->can_id & LIN_ID_MASK);

				lin_data = cf->data;
				lin_dlc = cf->can_dlc;
				if (lin_dlc > DLIN_DATA_MAX)
					lin_dlc = DLIN_DATA_MAX;
				tx_bytes = lin_dlc;
			}

			if (dlin_setup_msg(dl, 0, cf->can_id & LIN_ID_MASK,
				lin_data, lin_dlc) != -1) {

				dl->id_to_send = true;
				dl->data_to_send = (lin_data != NULL) ? true : false;
				dl->resp_len_known = (lin_dlc > 0) ? true : false;
				dl->dev->stats.tx_packets++;
				dl->dev->stats.tx_bytes += tx_bytes;
			}

			clear_bit(DLF_MSGEVENT, &dl->flags);
			kfree_skb(dl->tx_req_skb);
			netif_wake_queue(dl->dev);
			hrtimer_start(&dl->rx_timer,
				ktime_add(ktime_get(), dl->rx_timer_timeout),
				HRTIMER_MODE_ABS);
			break;

		case DSTATE_BREAK_SENT:
#ifdef BREAK_BY_BAUD
			if (dl->rx_cnt <= DLIN_BUFF_BREAK)
				continue;

			res = dlin_tty_change_speed(tty, dl->lin_baud);
#endif

			dl->lin_state = DSTATE_ID_SENT;
			dlin_send_tx_buff(dl);
			break;

		case DSTATE_ID_SENT:
			hrtimer_cancel(&dl->rx_timer);
			dl->id_to_send = false;
			if (dl->data_to_send) {
				dlin_send_tx_buff(dl);
				dl->lin_state = DSTATE_RESPONSE_SENT;
				dl->rx_expect = dl->tx_lim;
				goto dstate_response_sent;
			} else {
				if (dl->resp_len_known) {
					dl->rx_expect = dl->rx_lim;
				} else {
					dl->rx_expect = DLIN_BUFF_DATA + 2;
				}
				dl->lin_state = DSTATE_RESPONSE_WAIT;
				/* If we don't receive anything, timer will "unblock" us */
				hrtimer_start(&dl->rx_timer,
					ktime_add(ktime_get(), dl->rx_timer_timeout),
					HRTIMER_MODE_ABS);
				goto DSTATE_response_wait;
			}
			break;

		case DSTATE_RESPONSE_WAIT:
DSTATE_response_wait:
			if (test_bit(DLF_MSGEVENT, &dl->flags)) {
				unsigned char *lin_buff;
				cf = (struct can_frame *)dl->tx_req_skb->data;

				lin_buff = (dl->lin_master) ? dl->tx_buff : dl->rx_buff;
				if (cf->can_id == (lin_buff[DLIN_BUFF_ID] & LIN_ID_MASK)) {
					hrtimer_cancel(&dl->rx_timer);
					netdev_dbg(dl->dev, "received LIN response in a CAN frame.\n");
					if (dlin_setup_msg(dl, DLIN_STPMSG_RESPONLY,
						cf->can_id & LIN_ID_MASK,
						cf->data, cf->can_dlc) != -1) {

						dl->rx_expect = dl->tx_lim;
						dl->data_to_send = true;
						dl->dev->stats.tx_packets++;
						dl->dev->stats.tx_bytes += tx_bytes;

						if (!dl->lin_master) {
							dl->tx_cnt = DLIN_BUFF_DATA;
						}

						dlin_send_tx_buff(dl);
						clear_bit(DLF_MSGEVENT, &dl->flags);
						kfree_skb(dl->tx_req_skb);
						netif_wake_queue(dl->dev);

						dl->lin_state = DSTATE_RESPONSE_SENT;
						goto dstate_response_sent;
					}
				} else {
					dl->lin_state = DSTATE_RESPONSE_WAIT_BUS;
				}
			}

			/* Be aware, no BREAK here */
		case DSTATE_RESPONSE_WAIT_BUS:
			if (dl->rx_cnt < dl->rx_expect)
				continue;

			hrtimer_cancel(&dl->rx_timer);
			netdev_dbg(dl->dev, "response received ID %d len %d\n",
				dl->rx_buff[DLIN_BUFF_ID], dl->rx_cnt - DLIN_BUFF_DATA - 1);

			if (dlin_rx_validate(dl) == -1) {
				netdev_dbg(dl->dev, "RX validation failed.\n");
				dlin_report_error(dl, LIN_ERR_CHECKSUM);
			} else {
				/* Send CAN non-RTR frame with data */
				netdev_dbg(dl->dev, "sending NON-RTR CAN frame with LIN payload.");
				dlin_bump(dl); /* send packet to the network layer */
			}

			dl->id_to_send = false;
			dl->lin_state = DSTATE_IDLE;
			break;

		case DSTATE_ID_RECEIVED:
			lin_id = dl->rx_buff[DLIN_BUFF_ID] & LIN_ID_MASK;
			sce = &dl->linfr_cache[lin_id];
			spin_lock_irqsave(&dl->linframe_lock, flags);

			if ((sce->frame_fl & LIN_CACHE_RESPONSE)
					&& (sce->dlc > 0)) {
				int mode;

				netdev_dbg(dl->dev, "Sending LIN response from linfr_cache\n");

				lin_data = sce->data;
				lin_dlc = sce->dlc;
				if (lin_dlc > DLIN_DATA_MAX)
					lin_dlc = DLIN_DATA_MAX;
				memcpy(lin_data_buff, lin_data, lin_dlc);
				lin_data = lin_data_buff;
				tx_bytes = lin_dlc;

				mode = DLIN_STPMSG_RESPONLY;
				if (sce->frame_fl & LIN_CHECKSUM_EXTENDED)
					mode |= DLIN_STPMSG_CHCKSUM_ENH;

				if (dlin_setup_msg(dl, mode, lin_id & LIN_ID_MASK,
					lin_data, lin_dlc) != -1) {

					dl->rx_expect = dl->tx_lim;
					dl->data_to_send = true;
					dl->dev->stats.tx_packets++;
					dl->dev->stats.tx_bytes += tx_bytes;
					dl->resp_len_known = true;

					if (!dl->lin_master) {
						dl->tx_cnt = DLIN_BUFF_DATA;
					}
					dlin_send_tx_buff(dl);
				}

				hrtimer_start(&dl->rx_timer,
					ktime_add(ktime_get(), dl->rx_timer_timeout),
					HRTIMER_MODE_ABS);
			}
			spin_unlock_irqrestore(&dl->linframe_lock, flags);
			dl->lin_state = DSTATE_IDLE;
			break;

		case DSTATE_RESPONSE_SENT:
dstate_response_sent:
			if (dl->rx_cnt < dl->tx_lim)
				continue;

			hrtimer_cancel(&dl->rx_timer);
			dlin_bump(dl); /* send packet to the network layer */
			netdev_dbg(dl->dev, "response sent ID %d len %d\n",
				dl->rx_buff[DLIN_BUFF_ID], dl->rx_cnt - DLIN_BUFF_DATA - 1);

			dl->id_to_send = false;
			dl->lin_state = DSTATE_IDLE;
			break;
		}
	}

	hrtimer_cancel(&dl->rx_timer);
	netdev_dbg(dl->dev, "sllin_kwthread stopped.\n");

	return 0;
}

/**
 * dlin_sync()	--Collect hanged up channels 
 */
static void dlin_sync(void)
{
	int i;
	struct net_device *dev;
	struct dlin	  *dl;

	for (i = 0; i < maxdev; i++) {
		dev = dlin_devs[i];
		if (dev == NULL)
			break;

		dl = netdev_priv(dev);
		if (dl->tty)
			continue;
		if (dev->flags & IFF_UP) //check if the device is UP or DOWN
		  dev_close(dev); //This function moves an active device into down state
	}
}

/**
 * dlin_tty_alloc()--Link the DLIN channel with the tty line discipline
 * 
 * @line:
 */
static struct dlin *dlin_tty_alloc(dev_t line)
{
        int                i;
        struct net_device *dev=NULL;
        struct dlin       *dl;

        for(i=0 ;i<maxdev ;i++)
        {
          dev= dlin_devs[i]; /* Test if one dlin channel is free */
            if (dev==NULL)
                  break;
        }

        if(i>= maxdev)
          return NULL; /*Too many arguments */

        if(dev) /* free a channel if Master array is full */
        {
          for(i=0 ;i<maxdev; i++)
          {
            dev=dlin_devs[i];
            dl=netdev_priv(dev);
              if(test_bit(DLF_INUSE, &dl->flags)) /* test if flag bit is set */
              {
                unregister_netdevice(dev);
                dev=NULL;
                dlin_devs[i]=NULL;
              }
          }
        }

        if(!dev)
        {
          char name[IFNAMSIZ]; /* IFNAMSIZ = 16 (if.h) */
          sprintf(name,"dlin%d",i);
          dev= alloc_netdev(sizeof(*dl),name,dlin_setup); /* allocation of each netdevice */
          dev->base_addr  = i; /* set the base addr I/O of the device */
        }

          dl                = netdev_priv(dev); /* copy of the private part into the device */
          dl->magic         = DLIN_MAGIC; 		/* connection between the line discipline and the high level part set */
          dl->dev           = dev;
          spin_lock_init(&dl->lock);
          spin_lock_init(&dl->linframe_lock);
          dlin_devs[i]=dev;
          return dl;
}


/**
 *  dlin_tty_open()	-- OPEN the high level part of the DLIN channel
 *					   link the tty line discipline with a free DLIN channel
 * 
 * @tty:
 */
static int dlin_tty_open(struct tty_struct *tty)
{
  struct dlin *dl;
	int err;


	pr_debug("dlin: %s() invoked\n", __func__); /* function for debug purpose */

	if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		if (tty->ops->write == NULL)
			return -EOPNOTSUPP;

		/* RTnetlink lock is misused here to serialize concurrent
		   opens of sllin channels. There are better ways, but it is
		   the simplest one. */
		rtnl_lock();

		/* Collect hanged up channels. */
		dlin_sync();

		dl=tty->disc_data;
		err = -EEXIST;

	if (dl && dl->magic == DLIN_MAGIC)	/* Check if we are not already connected (cf  dlin_tty_alloc) */
	  {
	    goto err_exit;
	  }
		err = -ENFILE;
  dl = dlin_tty_alloc(tty_devnum(tty)); /* call of tty_alloc to link DLIN channel with the tty */
	if (dl == NULL)
	  {
	    goto err_exit;
	  }
	dl  ->tty        = tty;
	tty ->disc_data  = dl;
	dl  ->line       = tty_devnum(tty);

	if(!test_bit(DLF_INUSE, &dl->flags)) /* test if flag bit is not set */
	  {
	    dlin_reset_buffs(dl);

		dl->lin_baud = (baudrate == 0) ? LIN_DEFAULT_BAUDRATE : baudrate;
		pr_debug("sllin: Baudrate set to %u\n", dl->lin_baud);

		dl->lin_state = DSTATE_IDLE;

		hrtimer_init(&dl->rx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		dl->rx_timer.function = dlin_rx_timeout_handler;
		/* timeval_to_ktime(msg_head->ival1); */
		dl->rx_timer_timeout = ns_to_ktime(
			(1000000000l / dl->lin_baud) *
			DLIN_SAMPLES_PER_CHAR * DLIN_CHARS_TO_TIMEOUT);
	    dl->lin_state = DSTATE_IDLE;
	    set_bit (DLF_INUSE, &dl->flags);
		init_waitqueue_head(&dl->kwt_wq);
		dl->kwthread = kthread_run(dlin_kwthread, dl, "sllin");
		if (dl->kwthread == NULL)
			goto err_free_chan;

		err = register_netdevice(dl->dev);
		if (err)
			goto err_free_chan_and_thread;

	  }
		rtnl_unlock();
		tty->receive_room = DLIN_BUFF_LEN * 40; /* bytes free for queue  ??*/
		return 0;

	err_free_chan_and_thread:
	kthread_stop(dl->kwthread);
	dl->kwthread = NULL;

	err_free_chan:
	dl->tty = NULL;
	tty->disc_data = NULL;
	clear_bit(DLF_INUSE, &dl->flags);
	err_exit:
	rtnl_unlock();
  return err;
}

/**
 * dlin_tty_release()	-- ??
 * 
 * @tty:
 */
static void dlin_tty_release(struct tty_struct *tty)
{
  struct dlin *dl = (struct dlin *) tty->disc_data; /*????*/

  kthread_stop(dl->kwthread); /*stop the active thread */
  dl->kwthread=NULL;

  unregister_netdev(dl->dev);
}

/**
 * dlin_tty_hangup()	-- ??
 * 
 * @tty:
 */
static int dlin_tty_hangup(struct tty_struct *tty) {
	dlin_tty_release(tty);
	return 0;
}

/**  
 * dlin_ioctl()	-- Perform I/O control on an active DLIN channel.
 * 
 * @tty:
 * @file:
 * @cmd:
 * @arg:
 */
static int dlin_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct dlin *dl = (struct dlin *) tty->disc_data;
	unsigned int tmp;

	/* First make sure we're connected. */
	if (!dl || dl->magic != DLIN_MAGIC)
	  return -EINVAL; //Invalide argument = 22

	switch (cmd) {
	case SIOCGIFNAME: //0x8910   get iface name
		tmp = strlen(dl->dev->name) + 1;
		if (copy_to_user((void __user *)arg, dl->dev->name, tmp))
		  return -EFAULT; // = 14 Bad address
		return 0;

	case SIOCSIFHWADDR: // 0x8924 set hardware address
		return -EINVAL; //Invalide argument = 22

	default:
		/**
		 * tty_mode_ioctl	-- mode related ioctls
		 * 
		 * @tty: tty for the ioctl
		 * @file: file pointer for the tty
		 * @cmd: command
		 * @arg: ioctl argument
		 *
		 * Perform non line discipline specific mode control ioctls. 
		 * This is designed to be called by line disciplines to ensure 
		 * they provide consistent mode setting.
		 */
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}

/// Line Discipline main structure ///
static struct tty_ldisc_ops dlin_ldisc = {
	.owner				= THIS_MODULE,
	.name		  		= "dlin",
	.open		  		= dlin_tty_open,
	.close				= dlin_tty_release,
	.hangup				= dlin_tty_hangup,
	.ioctl				= dlin_ioctl,
	.receive_buf	= dlin_master_receive_buf,
	.write_wakeup	= dlin_write_wakeup,
};

/// Module init function ///
static int __init dlin_init_module(void) /* allocate the array of devices */
{
        int status;
        int result=-ENOMEM;

        dlin_devs = kzalloc(sizeof(struct net_device *)*maxdev,GFP_KERNEL);/* allocation of array of devices in KERNEL memory */
  			printk(KERN_DEBUG "dlin : memory allocation\n");
				if(!dlin_devs)
        {
            printk(KERN_DEBUG "dlin : fail memory allocation\n");
            return result;
        }
      /* TTY part */
  	   status = tty_register_ldisc(N_DLIN, &dlin_ldisc);
			 printk(KERN_DEBUG "dlin : register tty ldisc\n");
  	if (status)  {
  	  pr_err("dlin: can't register line discipline\n");
  	  kfree(dlin_devs); 	/* deallocate dlin_devs if dlin can't register line discipline */
  	}
		printk(KERN_DEBUG "dlin : end of init\n");
    return status;
}

/// Exit module fonction ///
static void __exit dlin_exit_module(void)
{
	int i;
		struct net_device *dev;
		struct dlin *dl;
		unsigned long timeout = jiffies + HZ;
		int busy = 0;

		if (dlin_devs == NULL)
			return;

		/* First of all: check for active disciplines and hangup them. */
		do {
			if (busy)
				msleep_interruptible(100);

			busy = 0;
			for (i = 0; i < maxdev; i++) {
				dev = dlin_devs[i];
				if (!dev)
					continue;
				dl = netdev_priv(dev);
				spin_lock_bh(&dl->lock);
				if (dl->tty) {
					busy++;
					tty_hangup(dl->tty); /* if we detect an active tty we close it */
				}
				spin_unlock_bh(&dl->lock);
			}
		} while (busy && time_before(jiffies, timeout)); /* if a timeout is detected we close */

		for (i = 0; i < maxdev; i++) {
			dev = dlin_devs[i];
			if (!dev)
				continue;
			dlin_devs[i] = NULL; /* we erase the elemements of the dlin_devs array */

			dl = netdev_priv(dev);
			if (dl->tty) {
				netdev_dbg(dl->dev, "tty discipline still running\n");
				/* Intentionally leak the control block. */
				dev->destructor = NULL;
			}

			unregister_netdev(dev);
		}

		kfree(dlin_devs); /* dealloc the array of devices */
		dlin_devs = NULL;

		i = tty_unregister_ldisc(N_DLIN); /*unregister the line disciplines it should return 0 */
		if (i)
	pr_err("dlin: can't unregister ldisc (err %d)\n", i);

  return;
}

module_init(dlin_init_module);
module_exit(dlin_exit_module);
