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
	{ 0x93d1de9d, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x282bc694, __VMLINUX_SYMBOL_STR(i2c_smbus_read_byte_data) },
	{ 0x43a53735, __VMLINUX_SYMBOL_STR(__alloc_workqueue_key) },
	{ 0x19616a90, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0xb46644d0, __VMLINUX_SYMBOL_STR(i2c_smbus_read_i2c_block_data) },
	{ 0x46608fa0, __VMLINUX_SYMBOL_STR(getnstimeofday) },
	{ 0xfe1cd47e, __VMLINUX_SYMBOL_STR(i2c_smbus_write_byte_data) },
	{ 0xb0f1a90c, __VMLINUX_SYMBOL_STR(i2c_put_adapter) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x8c03d20c, __VMLINUX_SYMBOL_STR(destroy_workqueue) },
	{ 0xd6b8e852, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xa2d44866, __VMLINUX_SYMBOL_STR(i2c_unregister_device) },
	{ 0x42160169, __VMLINUX_SYMBOL_STR(flush_workqueue) },
	{ 0xe07e84c7, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xefd6cf06, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr0) },
	{ 0x3688feb1, __VMLINUX_SYMBOL_STR(i2c_get_adapter) },
	{ 0x6bf2d2b9, __VMLINUX_SYMBOL_STR(i2c_new_device) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("i2c:L3GD20");
