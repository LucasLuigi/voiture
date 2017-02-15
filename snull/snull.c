#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/etherdevice.h>
#include "snull.h"

static int timeout =SNULL_TIMEOUT; // cf snull.h

struct net_device *snull_devs[2]; // the device

struct snull_packet {
	struct snull_packet *next;
	struct netdevice *dev;
	int		 datalen;
	u8		 data[ETH_DATA_LEN];
};

struct snull_priv { // struct privée de chaque interface pour faire le transit de paquets
	struct net_device_stats stats;
	int status;
	struct snull_packet *ppool;
	struct snull_packet *rx_queue;  /* List of incoming packets */
	int rx_int_enabled;
	int tx_packetlen;
	u8 *tx_packetdata;
	struct sk_buff *skb;
	spinlock_t lock;
};

int snull_open(struct net_device *dev) // se lance avec le ifconfig, start interface transmit queue
{
	memcpy(dev->dev_addr,"\0SNUL0",ETH_ALEN);// copie dans le champ dev_addr une addresse MAC
	if (dev == snull_devs[1])
	{
		dev->dev_addr[ETH_ALEN-1]++; // derniere carac de l'adresse MAC +1
	}
	netif_start_queue(dev);
	return 0;
}
int snull_release(struct net_device *dev) // opération inverse au open
{
	netif_stop_queue(dev);
	return 0;
}
int snull_config(struct net_device *dev,struct ifmap *map);
int snull_tx(struct sk_buff *skb,struct net_device *dev)
{
	return 0;
}
int snull_ioctl(struct net_device *dev,struct ifreq *rq, int cmd);
struct net_device_stats *snull_stats(struct net_device *dev);
//int  snull_rebuild_header(struct sk_buff *skb);
int snull_header(struct sk_buff *skb, struct net_device *dev,
                unsigned short type, const void *daddr, const void *saddr,
                unsigned len)
{
	return 0;
}
void snull_tx_timeout(struct net_device *dev);
int snull_change_mtu(struct net_device *dev,int new_mtu);

static const struct header_ops snull_header_ops =
{
	.create 						= snull_header,
	//.rebuild					= snull_rebuild_header
};

static const struct net_device_ops snull_netdev_ops =
{
.ndo_open             = snull_open,
.ndo_stop             = snull_release,
.ndo_set_config       = snull_config,
.ndo_start_xmit  			= snull_tx,
.ndo_do_ioctl         = snull_ioctl,
.ndo_get_stats        = snull_stats,
.ndo_tx_timeout       = snull_tx_timeout,
.ndo_change_mtu				= snull_change_mtu
};

static void snull_rx_ints(struct net_device *dev, int enable) // enable interruption
{
	struct snull_priv *priv = netdev_priv(dev);
	priv->rx_int_enabled = enable;
}

void snull_init(struct net_device *dev) // utile pour le alloc_netdev
{
	struct snull_priv *priv;
	ether_setup(dev); // assigne des champs automatiquement
	dev->watchdog_timeo = timeout;
	dev->netdev_ops			= &snull_netdev_ops;
	dev->header_ops			= &snull_header_ops;
	priv = netdev_priv(dev);
	memset(priv,0,sizeof(struct snull_priv));
	spin_lock_init(&priv->lock);
	snull_rx_ints(dev,1);
}

void snull_teardown_pool(struct net_device *dev) // clean le buffer des paquets
{
	struct snull_priv *priv = netdev_priv(dev);
	struct snull_packet *pkt;

	while ((pkt = priv->ppool))
	{
		priv->ppool = pkt->next;
		kfree (pkt);
	}
}

void snull_cleanup(void) // libère la mémoire
{
	int i;
  	for (i = 0; i < 2;  i++) {
		if (snull_devs[i])
		{
			unregister_netdev(snull_devs[i]);
			snull_teardown_pool(snull_devs[i]);
			free_netdev(snull_devs[i]);
		}
	}
	return;
}

int snull_init_module(void) // init du module
{
  int i, result;
  int ret=-ENOMEM; // erreur d'allocation kernel (mem trop petite dans le kernel)
  snull_devs[0] = alloc_netdev(sizeof(struct snull_priv),
  "sn%d",NET_NAME_UNKNOWN,snull_init); // alloc des interfaces(taille,nom,fonction d'init) NET_NAME_UNKNOWN pour la version 4.0.62 du noyau
  snull_devs[1] = alloc_netdev(sizeof(struct snull_priv),
  "sn%d",NET_NAME_UNKNOWN,snull_init);

  if(snull_devs[0]==NULL || snull_devs[1]==NULL)
    goto out;

    ret=-ENODEV; // erreur d'enregistrement (device non présent)
    for (i=0;i<2;i++)
      if((result=register_netdev(snull_devs[i]))) //si result =1 on a une erreur
        printk("snull: error %i registering device \"%s\"\n",
         result, snull_devs[i]->name);
      else
        ret =0;
        out :
        if (ret) // si on a eu une erreur on cleanup
        snull_cleanup();
        return ret;
}

module_init(snull_init_module);
module_exit(snull_cleanup);
