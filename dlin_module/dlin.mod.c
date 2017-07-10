#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xdce81671, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x15692c87, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0xa120d33c, __VMLINUX_SYMBOL_STR(tty_unregister_ldisc) },
	{ 0xcc5005fe, __VMLINUX_SYMBOL_STR(msleep_interruptible) },
	{ 0x26366f2d, __VMLINUX_SYMBOL_STR(tty_hangup) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xf698b0f, __VMLINUX_SYMBOL_STR(tty_register_ldisc) },
	{ 0x12da5bb2, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x86bb4d07, __VMLINUX_SYMBOL_STR(__netif_schedule) },
	{ 0xe80fbd5e, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0x2e16496e, __VMLINUX_SYMBOL_STR(prepare_to_wait) },
	{ 0x1000e51, __VMLINUX_SYMBOL_STR(schedule) },
	{ 0x12a38747, __VMLINUX_SYMBOL_STR(usleep_range) },
	{ 0x9b4783e7, __VMLINUX_SYMBOL_STR(hrtimer_cancel) },
	{ 0x4c45fa70, __VMLINUX_SYMBOL_STR(hrtimer_start) },
	{ 0xc87c1f84, __VMLINUX_SYMBOL_STR(ktime_get) },
	{ 0xb3f7646e, __VMLINUX_SYMBOL_STR(kthread_should_stop) },
	{ 0xc8b57c27, __VMLINUX_SYMBOL_STR(autoremove_wake_function) },
	{ 0xee1c39fe, __VMLINUX_SYMBOL_STR(sched_setscheduler) },
	{ 0xe707d823, __VMLINUX_SYMBOL_STR(__aeabi_uidiv) },
	{ 0xf087137d, __VMLINUX_SYMBOL_STR(__dynamic_pr_debug) },
	{ 0xb1ce04bb, __VMLINUX_SYMBOL_STR(register_netdevice) },
	{ 0x938b3bd1, __VMLINUX_SYMBOL_STR(wake_up_process) },
	{ 0x7bbea4cd, __VMLINUX_SYMBOL_STR(kthread_create_on_node) },
	{ 0xc4d3a2ee, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x71e51920, __VMLINUX_SYMBOL_STR(hrtimer_init) },
	{ 0x84348325, __VMLINUX_SYMBOL_STR(__rt_spin_lock_init) },
	{ 0xc2b8b57d, __VMLINUX_SYMBOL_STR(__rt_mutex_init) },
	{ 0x968cabb1, __VMLINUX_SYMBOL_STR(alloc_netdev_mqs) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x6e720ff2, __VMLINUX_SYMBOL_STR(rtnl_unlock) },
	{ 0x6ce0c39b, __VMLINUX_SYMBOL_STR(tty_devnum) },
	{ 0x290a3a36, __VMLINUX_SYMBOL_STR(dev_close) },
	{ 0xc7a4fbed, __VMLINUX_SYMBOL_STR(rtnl_lock) },
	{ 0xc6cbbc89, __VMLINUX_SYMBOL_STR(capable) },
	{ 0x2196324, __VMLINUX_SYMBOL_STR(__aeabi_idiv) },
	{ 0xa056344, __VMLINUX_SYMBOL_STR(netif_rx) },
	{ 0x365d75fa, __VMLINUX_SYMBOL_STR(skb_put) },
	{ 0x6df3c58e, __VMLINUX_SYMBOL_STR(__netdev_alloc_skb) },
	{ 0x799aca4, __VMLINUX_SYMBOL_STR(local_bh_enable) },
	{ 0x3ff62317, __VMLINUX_SYMBOL_STR(local_bh_disable) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x16305289, __VMLINUX_SYMBOL_STR(warn_slowpath_null) },
	{ 0x282849c2, __VMLINUX_SYMBOL_STR(netdev_warn) },
	{ 0x684db923, __VMLINUX_SYMBOL_STR(rt_spin_unlock) },
	{ 0xfbc6347a, __VMLINUX_SYMBOL_STR(rt_spin_lock) },
	{ 0x3c5ec69e, __VMLINUX_SYMBOL_STR(kfree_skb) },
	{ 0x280f0c8c, __VMLINUX_SYMBOL_STR(free_netdev) },
	{ 0xcbba3d5f, __VMLINUX_SYMBOL_STR(_mutex_unlock) },
	{ 0x59990aa8, __VMLINUX_SYMBOL_STR(tty_encode_baud_rate) },
	{ 0x86b98ffc, __VMLINUX_SYMBOL_STR(_mutex_lock) },
	{ 0x9d669763, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0x396e4a17, __VMLINUX_SYMBOL_STR(unregister_netdev) },
	{ 0x8699ff73, __VMLINUX_SYMBOL_STR(kthread_stop) },
	{ 0x67c2fa54, __VMLINUX_SYMBOL_STR(__copy_to_user) },
	{ 0x97255bdf, __VMLINUX_SYMBOL_STR(strlen) },
	{ 0xda23fff7, __VMLINUX_SYMBOL_STR(tty_mode_ioctl) },
	{ 0x3c28fe0b, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0x99c6d8fe, __VMLINUX_SYMBOL_STR(__dynamic_netdev_dbg) },
	{ 0xefd6cf06, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr0) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "F55A9944C6DD8E0FF4513E8");
