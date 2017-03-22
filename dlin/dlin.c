#include <linux/module.h> /* definitions of symbols and fonctions */
#include <linux/init.h> /* initialization and cleanup fonctions */
#include <linux/moduleparam.h> /* passing parameters */

#include <linux/skbuff.h> /* struct sk_buff */
#include <linux/netdevice.h> /* struct net_device */
#include <linux/kernel.h> /* printk... */
#include <linux/slab.h> /* kmalloc() */
#include <linux/errno.h> /* error codes */
#include <linux/if_arp.h> /* CAN identifier */
#include <linux/if.h>
#include <linux/can.h>
#include <linux/sched.h> /*task_struct describe process/task in the system */
#include <linux/tty.h>
#include <linux/kthread.h> /*thread task such as stop,init... */
#include "lin_bus.h"

MODULE_AUTHOR("Egreteau Corentin, Faro Mathias, Gissot Lucas, Pleuron Jonathan");
MODULE_LICENSE("Dual BSD/GPL");

#define N_DLIN        25
#define DLIN_MAGIC    0x53CA /* value of the variable magic in the struct dlin used as a flag */

static int maxdev =10;
module_param(maxdev,int,0); /* max number of dlin channels */

/* maximum buffer len to store whole LIN message*/
#define DLIN_DATA_MAX		8
#define DLIN_BUFF_LEN		(1 /*break*/ + 1 /*sync*/ + 1 /*ID*/ + \
				                  DLIN_DATA_MAX + 1 /*checksum*/)
#define DLIN_BUFF_BREAK	0
#define DLIN_BUFF_SYNC		1
#define DLIN_BUFF_ID		  2
#define DLIN_BUFF_DATA		3

#define DLIN_SAMPLES_PER_CHAR	10
#define DLIN_CHARS_TO_TIMEOUT	24

///////////////////States of the DLIN channel/////////////////

enum dstate {
	DSTATE_IDLE = 0,
	DSTATE_BREAK_SENT,
	DSTATE_ID_SENT,
	DSTATE_RESPONSE_WAIT, /* Wait for response */
	DSTATE_RESPONSE_WAIT_BUS, /* Wait for response from LIN bus
				only (CAN frames from network stack
				are not processed in this moment) */
	DSTATE_ID_RECEIVED,
	DSTATE_RESPONSE_SENT,
};

struct dlin_conf_entry {
	int dlc;		       /* Length of data in LIN frame */
	canid_t frame_fl;	 /* LIN frame flags. Passed from userspace as
				              canid_t data type */
	u8 data[8];		     /* LIN frame data payload */
};

/* main struct of the driver */
struct dlin {
       int                  magic;
       struct net_device   *dev; /* our interface */
       spinlock_t           lock;
       /* LIN message buffer and actual processed data counts */
    	 unsigned char		    rx_buff[DLIN_BUFF_LEN]; /* LIN Rx buffer */
       unsigned char		    tx_buff[DLIN_BUFF_LEN]; /* LIN Tx buffer */
	     int			            rx_expect;      /* expected number of Rx chars */
	     int			            rx_lim;         /* maximum Rx chars for current frame */
	     int			            rx_cnt;         /* message buffer Rx fill level  */
	     int			            tx_lim;         /* actual limit of bytes to Tx */
       int             	    tx_cnt;         /* number of already Tx bytes */
       char            	    lin_master;     /* node is a master node */
       int			            lin_state;	/* state */
       char			            id_to_send;	/* there is ID to be sent */
       char                 data_to_send;   /* there are data to be sent */
       struct sk_buff      *tx_skb; /* socket buffer */
       struct task_struct  *kwthread; /* struct describing the kernel worker thread */
       dev_t                line; /* struct with major and minor numbers of the device */
       struct tty_struct    *tty; /* ptr to TTY structure*/
       unsigned long        flags;
#define DLF_INUSE           0     /*channel in use */
#define DLF_MSGEVENT	      4 /* CAN message to sent   */
        struct sk_buff      *tx_req_skb;	/* Socket buffer with CAN frame
						                                 received from network stack*/

       /* List with configurations for each of 0 to LIN_ID_MAX LIN IDs */
       struct dlin_conf_entry linfr_cache[LIN_ID_MAX + 1];
       wait_queue_head_t	  kwt_wq; /* Wait queue used by kwthread */
       spinlock_t           linframe_lock; /*frame cache and buffers lock*/
};

static struct net_device **dlin_devs; /*list of *net_devices */
//static int dlin_configure_frame_cache(struct dlin *dl, struct can_frame *cf); /* important to mention
                                                                                //  the name of the function to call it before write it */
/* Netdevice DOWN->UP */
static int dlin_open(struct net_device *dev)
{
  struct dlin *dl = netdev_priv(dev);
  netdev_dbg(dl->dev, "%s() invoked\n",__func__); /* function for debug purpose*/

  if(dl->tty == NULL) /* open the dev only if it is attached to tty */
  return -ENODEV;

  dl->flags &= (1 << DLF_INUSE); /* bit wise and to set the INUSE bit */
  netif_start_queue(dev); /*Allow upper layers to call the device hard_start_xmit routine */
    return 0;
}

static int dlin_close(struct net_device *dev)
{
    struct dlin *dl = netdev_priv(dev);
    spin_lock_bh(&dl->lock); /*Disables software interrupts before taking the lock, but leaves hardware interrupts enabled*/
    if(dl->tty)
    {
        /* TTY discipline is running. */
        clear_bit(TTY_DO_WRITE_WAKEUP, &dl->tty->flags); /* reset the bit of TTY */
    }
    netif_stop_queue(dev); /* Stop upper layers calling the device hard_start_xmit routine.
                            *	Used for flow control when transmit resources are unavailable. */
    dl->rx_expect = 0;
    dl->tx_lim    = 0;
    spin_unlock_bh(&dl->lock);

    return 0;
}

/**
 * DLIN_configure_frame_cache() -- Configure particular entry in linfr_cache
 *
 * @sl:
 * @cf: Pointer to CAN frame sent to this driver
 *	holding configuration information
 */
static int dlin_configure_fame_cache(struct dlin *dl, struct can_frame *cf)
{
//  unsigned long flags;
  struct dlin_conf_entry *sce;

  sce =&dl->linfr_cache[cf->can_id & LIN_ID_MASK];
  /* the index corresponds to the ID of the trame CAN
  that we want to transmit (control ect...)
  in the array we find all the coresponding config for the LIN */
  netdev_dbg(dl->dev, "Setting frame cache with EFF CAN frame. LIN ID = %d\n",cf->can_id & LIN_ID_MASK);
// spin lock ??
  sce->dlc = cf->can_dlc; /* make the correspondance between can and lin length */
  if(sce->dlc > DLIN_DATA_MAX) /* if the trame is too long dor the LIN protocol */
    {
      sce->dlc = DLIN_DATA_MAX; /* set up the maximum length */
    }
      sce->frame_fl = (cf->can_id & ~LIN_ID_MASK) & CAN_EFF_MASK; /* a voir ? + ne pas oublier les spin lock*/
      memcpy(sce->data, cf->data, cf->can_dlc); /* destination, source, size */
      return 0;

      /* struct can_frame {
        u32 can_id;  29 bit CAN_ID + flags
          u8  can_dlc;  data length code: 0 .. 8
          u8  data[8]; }; */
}

/*
 * DLl_xmit() -- Send a can_frame to a TTY queue.
 *
 * @skb: Pointer to Socket buffer to be sent.
 * @dev: Network device where @skb will be sent.
 */

static netdev_tx_t dlin_tx(struct sk_buff *skb, struct net_device *dev)
{
    struct dlin *dl = netdev_priv(dev);
    struct can_frame *cf;

    if(skb->len != sizeof(struct can_frame))
    goto err_out; /* check if our socket buffer has the good length */

    spin_lock(&dl->lock);
    if(!netif_running(dev))/* Test if the device has been brought up*/
      {
        netdev_warn(dl->dev,"tx: interface is down\n");
        goto err_out_unlock;
      }

    cf = (struct can_frame *) skb->data; /* cast the data in lin packet to can_frame */
    if(cf->can_id & LIN_CTRL_FRAME) /* if we want to send a control frame ?? */
    {
      dlin_configure_fame_cache(dl,cf);
      goto free_out_unlock;
    }

    netif_stop_queue(dl->dev);

    dl->tx_req_skb = skb; /* link the device with th packet to txmit */
    set_bit(DLF_MSGEVENT, &dl->flags); /* inform that we send a frame CAN */
    wake_up(&dl->kwt_wq); /* wake up sleeping processes */
    spin_unlock(&dl->lock);
    return NETDEV_TX_OK;

    free_out_unlock :
    err_out_unlock :
      spin_unlock(&dl->lock);
    err_out :
      kfree_skb(skb); /* free the skb struct that we used */
      return NETDEV_TX_OK;
}

/* struct of basic functions for struct net_device */
static const struct net_device_ops dlin_netdev_ops = {
       .ndo_open            = dlin_open,
       .ndo_stop            = dlin_close,
       .ndo_start_xmit      = dlin_tx,
};


/* struct for setting up the whole structure of the device */
static void dlin_setup(struct net_device *dev)
{
        dev->netdev_ops     = &dlin_netdev_ops;
        dev->flags          = IFF_NOARP;/* non ARP protocol (if.h) */
        dev->tx_queue_len   = 10;

        dev->mtu            = sizeof(struct can_frame);
        dev->type           = ARPHRD_CAN; /* identifier CAN for non ARP protocol (if_arp.h) */
        dev->features       = NETIF_F_HW_CSUM; /* checksum all pkt at hw level (netdevice.h) */
}

static struct dlin *dlin_tty_alloc(dev_t line)
{
  /* Link the DLIN channel with the tty line discipline */
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
          dev= alloc_netdev(sizeof(*dl),name,NET_NAME_UNKNOWN,dlin_setup); /* allocation of each netdevice */
          dev->base_addr  = i; /* set the base addr I/O of the device */
        }

          dl                = netdev_priv(dev); /*copy of the private part into the device */
          dl->magic         = DLIN_MAGIC; /* connection between the line discipline and the high level part set */
          dl->dev           = dev;
          spin_lock_init(&dl->lock);
          spin_lock_init(&dl->linframe_lock);
          dlin_devs[i]=dev;
          return dl;
}

/*Reset buffer of the DLIN channel*/
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


/* OPEN the high level part of the DLIN channel
link the tty line discipline with a free DLIN channel*/
static int dlin_tty_open(struct tty_struct *tty)
{
  struct dlin *dl;

	dl=tty->disc_data;
	pr_debug("dlin: %s() invoked\n", __func__); /* function for debug purpose*/

	if (dl && dl->magic == DLIN_MAGIC) /* Check if we are not already connected (cf  dlin_tty_alloc) */
	  {
	    goto err_exit;
	  }
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
	    dl->lin_state = DSTATE_IDLE;
	    set_bit (DLF_INUSE, &dl->flags);
	  }
	tty->receive_room = DLIN_BUFF_LEN * 40; /* bytes free for queue  ??*/
err_exit:
  return 0;
}

static void dlin_tty_release(struct tty_struct *tty)
{
  struct dlin *dl = (struct dlin *) tty->disc_data; /*????*/

  kthread_stop(dl->kwthread); /*stop the active thread */
  dl->kwthread=NULL;

  unregister_netdev(dl->dev);
}

static struct tty_ldisc_ops dlin_ldisc = {
	.owner		= THIS_MODULE,
	.name		  = "dlin",
	.open		  = dlin_tty_open,
	.close		= dlin_tty_release,
	//.hangup		= dlin_hangup,
	//.ioctl		= dlin_ioctl,
	//.receive_buf	= dlin_receive_buf,
	//.write_wakeup	= dlin_write_wakeup,
};


static int __init dlin_init_module(void) /* allocate the array of devices */
{
        int status;
        int result=-ENOMEM;

        dlin_devs = kzalloc(sizeof(struct net_device *)*maxdev,GFP_KERNEL);/* allocation of array of devices in KERNEL memory */
        if(!dlin_devs)
        {
            printk(KERN_DEBUG "dlin : fail memory allocation\n");
            return result;
        }
      //TTY part//
  	   status = tty_register_ldisc(N_DLIN, &dlin_ldisc);
  	if (status)  {
  	  pr_err("dlin: can't register line discipline\n");
  	  kfree(dlin_devs); //deallocate dlin_devs if dlin can't register line discipline
  	}
    return status;
}

static void __exit dlin_exit_module(void)
{


  return;
}

module_init(dlin_init_module);
module_exit(dlin_exit_module);
