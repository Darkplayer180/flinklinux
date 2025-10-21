/*******************************************************************
 *   _________     _____      _____    ____  _____    ___  ____    *
 *  |_   ___  |  |_   _|     |_   _|  |_   \|_   _|  |_  ||_  _|   *
 *    | |_  \_|    | |         | |      |   \ | |      | |_/ /     *
 *    |  _|        | |   _     | |      | |\ \| |      |  __'.     *
 *   _| |_        _| |__/ |   _| |_    _| |_\   |_    _| |  \ \_   *
 *  |_____|      |________|  |_____|  |_____|\____|  |____||____|  *
 *                                                                 *
 *******************************************************************
 *                                                                 *
 *  AXI (Advanced eXtensible Interface) communication module for   *
 *  Zync7000 SoC                                                   *
 *                                                                 *
 *******************************************************************/

/** @file flink_axi.c
 *  @brief AXI (Advanced eXtensible Interface) communication module.
 *
 *  Implements read and write functions over AXI Bus.
 *
 *  @author Patrick Good
 */


#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include "../flink.h"

/* Config option (device tree or hard coded) If you want to configure 
 * this module with hard coded values, just uncomment the define below 
 * and set your values to the defines. Otherwise it will look for a 
 * compatible node in the device tree.
**/
//#define CONFIG_SETTINGS_HARD_CODED
#ifdef CONFIG_SETTINGS_HARD_CODED
	#define AXI_BASE_ADDR 		0x7aa00000 // if you adust it here also adjust the number in define NODE_NAME
	#define AXI_Range_LENGHT 	0x9000
	#define IRQ_OFFSET 			55
	#define SIGNAL_OFFSET 		34
	#define NOF_IRQs			30
	#define NODE_NAME           "flink_axi@7aa00000"
#else
	#define COMPATIBLE_NODE "ost,flink-axi-1.0"
#endif

// ############ Module infos ############
MODULE_AUTHOR("Patrick Good");
MODULE_DESCRIPTION("Flink AXI communication Module");
MODULE_LICENSE("Dual BSD/GPL");

// ############ Module Defines ############
#define MOD_VERSION "0.1.0"
#define MODULE_NAME THIS_MODULE->name

#define PLATFORM_DEV_NAME "flink_axi_driver"
#define PLATFORM_DEV_ID -1

// ############ Function Prototypes ############
static u8 flink_axi_read8(struct flink_device* fdev, u32 addr);
static u16 flink_axi_read16(struct flink_device* fdev, u32 addr);
static u32 flink_axi_read32(struct flink_device* fdev, u32 addr);
static int flink_axi_write8(struct flink_device* fdev, u32 addr, u8 val);
static int flink_axi_write16(struct flink_device* fdev, u32 addr, u16 val);
static int flink_axi_write32(struct flink_device* fdev, u32 addr, u32 val);
static u32 flink_axi_address_space_size(struct flink_device* fdev);

static int flink_axi_probe(struct platform_device *pdev);
static int flink_axi_remove(struct platform_device *pdev);

// ############ Module Data Structs ############
#ifdef CONFIG_SETTINGS_HARD_CODED
	static struct platform_driver flink_axi_driver = {
	 	.driver = {
			.name = PLATFORM_DEV_NAME,
			.owner = THIS_MODULE,
	    },
	    .probe = flink_axi_probe,
	    .remove = flink_axi_remove,
	};

	static struct platform_device flink_axi_device = {
    	.name = PLATFORM_DEV_NAME,
    	.id = PLATFORM_DEV_ID,
	};
#else
	static const struct of_device_id my_driver_of_match[] = {
	    { .compatible = COMPATIBLE_NODE},
	    {},
	};
	MODULE_DEVICE_TABLE(of, my_driver_of_match);

	static struct platform_driver flink_axi_driver = {
	 	.driver = {
			.name = PLATFORM_DEV_NAME,
			.owner = THIS_MODULE,
			.of_match_table = my_driver_of_match,
	    },
	    .probe = flink_axi_probe,
	    .remove = flink_axi_remove,
	};
#endif

struct flink_axi_bus_data
{
	void __iomem *base;
	resource_size_t hardwareAddressBase;
	resource_size_t size;
};

struct flink_bus_ops flink_axi_bus_ops =
{
	.read8              = flink_axi_read8,
	.read16             = flink_axi_read16,
	.read32             = flink_axi_read32,
	.write8             = flink_axi_write8,
	.write16            = flink_axi_write16,
	.write32            = flink_axi_write32,
	.address_space_size = flink_axi_address_space_size
};

// ############ Module Bus Operations ############
static inline u8 flink_check_offset(struct flink_axi_bus_data* d, u32 offset) {
	return d->size > offset;
}

static u8 flink_axi_read8(struct flink_device* fdev, u32 addr) {
    struct flink_axi_bus_data* d = (struct flink_axi_bus_data*)fdev->bus_data;
	if (likely(d != NULL && flink_check_offset(d,addr))) {
		return ioread8(d->base + addr);
	} else {
		printk(KERN_ERR "[%s] Failed to perform the ioread8 operation\n", MODULE_NAME);
	}
	return 0;
}

static u16 flink_axi_read16(struct flink_device* fdev, u32 addr) {
    struct flink_axi_bus_data* d = (struct flink_axi_bus_data*)fdev->bus_data;
	if (likely(d != NULL && flink_check_offset(d,addr))) {
		return ioread16(d->base + addr);
	} else {
		printk(KERN_ERR "[%s] Failed to perform the ioread16 operation\n", MODULE_NAME);
	}
	return 0;
}

static u32 flink_axi_read32(struct flink_device* fdev, u32 addr) {
    struct flink_axi_bus_data* d = (struct flink_axi_bus_data*)fdev->bus_data;
	if (likely(d != NULL && flink_check_offset(d,addr))) {
		return ioread32(d->base + addr);
	} else {
		printk(KERN_ERR "[%s] Failed to perform the ioread32 operation\n", MODULE_NAME);
	}
	return 0;
}

static int flink_axi_write8(struct flink_device* fdev, u32 addr, u8 val) {
    struct flink_axi_bus_data* d = (struct flink_axi_bus_data*)fdev->bus_data;
	if (likely(d != NULL && flink_check_offset(d,addr))) {
		iowrite8(val, d->base + addr);
	} else {
		printk(KERN_ERR "[%s] Failed to perform the iowrite8 operation\n", MODULE_NAME);
	}
	return 0;
}

static int flink_axi_write16(struct flink_device* fdev, u32 addr, u16 val) {
    struct flink_axi_bus_data* d = (struct flink_axi_bus_data*)fdev->bus_data;
	if (likely(d != NULL && flink_check_offset(d,addr))) {
		iowrite16(val, d->base + addr);
	} else {
		printk(KERN_ERR "[%s] Failed to perform the iowrite16 operation\n", MODULE_NAME);
	}
	return 0;
}

static int flink_axi_write32(struct flink_device* fdev, u32 addr, u32 val) {
    struct flink_axi_bus_data* d = (struct flink_axi_bus_data*)fdev->bus_data;
	if (likely(d != NULL && flink_check_offset(d,addr))) {
		iowrite32(val, d->base + addr);
	} else {
		printk(KERN_ERR "[%s] Failed to perform the iowrite32 operation\n", MODULE_NAME);
	}
	return 0;
}

static u32 flink_axi_address_space_size(struct flink_device* fdev) {
    struct flink_axi_bus_data* d = (struct flink_axi_bus_data*)fdev->bus_data;
	return (u32)(d->size);
}

// ############ Platform Driver Probe And Remove ############
static int flink_axi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
    u32 irq_offset = 0;
	u32 signal_offset = 0;
	u32 reg[2];
	u32 nof_irq = 0;
    int ret = 0;
//    void __iomem *base;
    int err = 0;
    struct flink_device *fdev;
	struct flink_axi_bus_data *bus_data;

	bus_data = kzalloc(sizeof(struct flink_axi_bus_data), GFP_KERNEL);
	if (unlikely(!bus_data)) {
		printk(KERN_ERR "[%s] Failed to allocate mem for bus data\n", MODULE_NAME);
		ret = -ENOMEM;
		goto bus_data_alloc_failure;
	}

	#ifdef CONFIG_SETTINGS_HARD_CODED
		bus_data->hardwareAddressBase = AXI_BASE_ADDR;
		bus_data->size = AXI_Range_LENGHT;
		irq_offset = IRQ_OFFSET;
		signal_offset = SIGNAL_OFFSET;
		nof_irq = NOF_IRQs;

		if (unlikely(!request_mem_region(bus_data->hardwareAddressBase, bus_data->size, NODE_NAME))) {
			printk(KERN_ERR "[%s] Failed to request AXI memory region", MODULE_NAME);
			ret = -ENOMEM;
			goto mem_request_failure;
		}

    	bus_data->base = ioremap(bus_data->hardwareAddressBase, bus_data->size);
		if (unlikely(!(bus_data->base))) {
        	printk(KERN_ERR "[%s] Failed to map AXI memory\n", MODULE_NAME);
        	ret = -ENOMEM;
			goto mem_iomap_failure;
		}
	#else
		err = of_property_read_u32(np, "ost,flink-nof-irq", &nof_irq);
    	if (unlikely(err < 0)) {
        	printk(KERN_ERR "[%s] Failed to read 'ost,flink-nof-irq' property. Error Nr: %d\n", MODULE_NAME, err);
			ret = -ENOMEM;
        	goto read_poperties_failure;
    	}
		err = of_property_read_u32(np, "ost,flink-signal-offset", &signal_offset);
    	if (unlikely(err < 0)) {
        	printk(KERN_ERR "[%s] Failed to read 'ost,flink-signal-offset' property. Error Nr: %d\n", MODULE_NAME, err);
			ret = -ENOMEM;
        	goto read_poperties_failure;
    	}
		err = of_property_read_u32_array(np, "reg", reg, 2);
    	if (unlikely(err < 0)) {
        	printk(KERN_ERR "[%s] Failed to read hardware address register (reg) property. Error Nr: %d\n", MODULE_NAME, err);
			ret = -ENOMEM;
        	goto read_poperties_failure;
    	}
		bus_data->hardwareAddressBase = reg[0];
		bus_data->size = reg[1];
		irq_offset = of_irq_get(np, 0);
		if (unlikely(irq_offset < 0)) {
			printk(KERN_ERR "[%s] Failed to get the base irq number from IRQ controller. Error Nr: %d\n", MODULE_NAME, err);
			ret = -ENOMEM;
        	goto read_poperties_failure;
		}
		if (unlikely(!request_mem_region(bus_data->hardwareAddressBase, bus_data->size, MODULE_NAME))) {
			printk(KERN_ERR "[%s] Failed to request AXI memory region", MODULE_NAME);
			ret = -ENOMEM;
			goto mem_request_failure;
		}
		bus_data->base = of_iomap(np, 0);
    	if (unlikely(!(bus_data->base))) {
			printk(KERN_ERR "[%s] Failed to map AXI memory\n", MODULE_NAME);
        	ret = -ENOMEM;
			goto mem_iomap_failure;
		}
	#endif

	#ifdef DBG
		#ifdef CONFIG_SETTINGS_HARD_CODED
			printk(KERN_DEBUG "[%s] Hard coded values are:", MODULE_NAME);
		#else
			printk(KERN_DEBUG "[%s] Values from Device tree are:", MODULE_NAME);
		#endif
		printk(KERN_DEBUG "  --> HW address:     %#lx\n", bus_data->hardwareAddressBase);
		printk(KERN_DEBUG "  --> HW vir address: %#lx\n", bus_data->base);
		printk(KERN_DEBUG "  --> HW size:        %#lx\n", bus_data->size);
		printk(KERN_DEBUG "  --> IRQ offset:     %lu\n", irq_offset);
		printk(KERN_DEBUG "  --> Siagnal offset: %lu\n", signal_offset);
		printk(KERN_DEBUG "  --> nof IRQ's:      %lu\n", nof_irq);
	#endif
	
	fdev = flink_device_alloc();
	if (unlikely(!fdev)) {
		printk(KERN_ERR "[%s] Failed to allocate flink device\n", MODULE_NAME);
		ret = -ENOMEM;
		goto fdev_alloc_failure;
	}

    // setup flink device
	flink_device_init_irq(fdev, &flink_axi_bus_ops, THIS_MODULE, nof_irq, irq_offset, signal_offset);
	fdev->bus_data = bus_data;
	#ifdef DBG
		printk(KERN_DEBUG "[%s] Create flink device...", MODULE_NAME);
	#endif
	if(unlikely(flink_device_add(fdev) < 0)){
		printk(KERN_ERR "[%s] Faild to add flink device", MODULE_NAME);
		ret = -ENOMEM;
		goto flink_add_failure;
	}
	printk(KERN_INFO "[%s] Flink device created", MODULE_NAME);
    return 0;


	flink_add_failure:
		flink_device_delete(fdev);
	fdev_alloc_failure:
		iounmap(bus_data->base);
	mem_iomap_failure:
		release_mem_region(bus_data->hardwareAddressBase, bus_data->size);
	mem_request_failure:
	read_poperties_failure:
		kfree(bus_data);
	bus_data_alloc_failure:
		// nothing to clean up
		printk(KERN_ERR "[%s] Failed to initialise flink-AXI driver\n", MODULE_NAME);
	return ret;
}

static int flink_axi_remove(struct platform_device *pdev)
{
    struct flink_device *fdev;
	struct flink_device *fdev_next;
	struct flink_axi_bus_data *bus_data;

	#ifdef DBG
		printk(KERN_DEBUG "[%s] AXI platform device removing", MODULE_NAME);
	#endif
	list_for_each_entry_safe(fdev, fdev_next, flink_get_device_list(), list) {
		if(fdev->appropriated_module == THIS_MODULE) {
			bus_data = (struct flink_axi_bus_data *)(fdev->bus_data);
			flink_device_remove(fdev);
			flink_device_delete(fdev);
			iounmap(bus_data->base);
			release_mem_region(bus_data->hardwareAddressBase, bus_data->size);
			kfree(bus_data);
		}
	}
	return 0;
}

// ############ Module Initialization and cleanup ############
static int __init axi_init(void)
{
	int err = 0;

	#ifdef CONFIG_SETTINGS_HARD_CODED	
    	err = platform_device_register(&flink_axi_device);
		if (err < 0) {
			printk(KERN_ERR "[%s] cannot register device: %d\n", MODULE_NAME, err);
			goto exit;
		}

		err = platform_driver_register(&flink_axi_driver);
		if (err < 0) {
			printk(KERN_ERR "[%s] unable to register driver: %d\n", MODULE_NAME, err);
			goto exit_unregister_device;
		}
	#else
		err = platform_driver_register(&flink_axi_driver);
		if (err < 0) {
			printk(KERN_ERR "[%s] unable to register driver: %d\n", MODULE_NAME, err);
			goto exit;
		}
	#endif

	#ifdef DBG
		printk(KERN_DEBUG "[%s] Module successfully loaded\n", MODULE_NAME);
	#endif
	return 0;

	#ifdef CONFIG_SETTINGS_HARD_CODED
		exit_unregister_device:
			platform_device_unregister(&flink_axi_device);
	#endif
	exit:
		printk(KERN_ERR "[%s] Init failed\n", MODULE_NAME);
	return err;
}

static void __exit axi_exit(void)
{
    platform_driver_unregister(&flink_axi_driver);
	#ifdef CONFIG_SETTINGS_HARD_CODED	
    	platform_device_unregister(&flink_axi_device);
	#endif
	#ifdef DBG
		printk(KERN_DEBUG "[%s] Module successfully unloaded\n", MODULE_NAME);
	#endif
}

module_init(axi_init);
module_exit(axi_exit);
