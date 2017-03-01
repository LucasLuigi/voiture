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

MODULE_AUTHOR("Egreteau Corentin, Faro Mathias, Gissot Lucas, Pleuron Jonathan");
MODULE_LICENSE("Dual BSD/GPL");

static int maxdev =10;
module_param(maxdev,int,0); /* max number of dlin channels */


/* main struct of the driver */
struct dlin {
       struct net_device   *dev; /* our interface */
       struct sk_buff      *tx_skb; /* socket buffer */

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
        dev->netdev_ops = &dlin_netdev_ops;
        dev->flags          = IFF_NOARP;/* non ARP protocol (if.h) */
        dev->tx_queue_len   = 10;

        dev->mtu            = sizeof(struct can_frame);
        dev->type           = ARPHRD_CAN; /* identifier CAN for non ARP protocol (if_arp.h) */
        dev->features       = NETIF_F_HW_CSUM; /* checksum all pkt at hw level (netdevice.h) */
}

static int __init dlin_init_module(void)
{
        int status;
        struct dlin *dl;
        int i;
        char name[IFNAMSIZ]; /* IFNAMSIZ = 16 (if.h) */

        dlin_devs = kzalloc(sizeof(struct net_device *)*maxdev,GFP_KERNEL);/* allocation of array of devices in KERNEL memory */
        if(!dlin_devs)
        {
            printk(KERN_DEBUG "dlin : fail memory allocation\n");
            return -ENOMEM;
        }

        for(i=0 ;i<maxdev ;i++)
        {
          sprintf(name,"dlin%d",i);
          dlin_devs[i]= alloc_netdev(sizeof(*dl),name,NET_NAME_UNKNOWN,dlin_setup); /* allocation of each netdevice */
        }

    return 1;
}

static void __exit dlin_exit_module(void)
{
  return;
}

module_init(dlin_init_module);
module_exit(dlin_exit_module);
