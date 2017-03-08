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

MODULE_AUTHOR("Egreteau Corentin, Faro Mathias, Gissot Lucas, Pleuron Jonathan");
MODULE_LICENSE("Dual BSD/GPL");

static int maxdev =10;
module_param(maxdev,int,0); /* max number of dlin channels */


/* main struct of the driver */
struct dlin {
       struct net_device   *dev; /* our interface */
       spinlock_t           lock;
       struct sk_buff      *tx_skb; /* socket buffer */
       struct task_struct  *kwthread; /* struct describing the kernel worker thread */
       dev_t                line; /* struct with major and minor numbers of the device */
       struct tty_struct *tty; /* ptr to TTY structure*/
       unsigned long        flags;
#define SLF_INUSE           0     /*channel in use */
       spinlock_t           linframe_lock; /*frame cache and buffers lock*/
};

static struct net_device **dlin_devs; /*list of *net_devices */

/* struct of basic functions for struct net_device */
static const struct net_device_ops dlin_netdev_ops = {
       .ndo_open            = dlin_open,
       .ndo_stop            = dlin_release,
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
        int i;
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
              if(test_bit(SLF_INUSE, &dl->flags)) /* test if flag bit is set */
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
          dl->dev           = dev;
          spin_lock_init(&dl->lock);
          spin_lock_init(&dl->linframe_lock);
          dlin_devs[i]=dev;
          return dl;
}

static int dlin_tty_open(struct tty_struct *tty)
{
        struct dlin *dl;
        int err;
        dl=dlin_tty_alloc(tty_devnum(tty));
}

static int __init dlin_init_module(void)
{
        int status;
        struct dlin *dl;
        int i;
        int result=-ENOMEM;

        dlin_devs = kzalloc(sizeof(struct net_device *)*maxdev,GFP_KERNEL);/* allocation of array of devices in KERNEL memory */
        if(!dlin_devs)
        {
            printk(KERN_DEBUG "dlin : fail memory allocation\n");
            return -ENOMEM;
        }
    return 1;
}

static void __exit dlin_exit_module(void)
{
  return;
}

module_init(dlin_init_module);
module_exit(dlin_exit_module);
