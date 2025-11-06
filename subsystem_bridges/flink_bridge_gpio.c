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
 *  Core Module                                                    *
 *                                                                 *
 *******************************************************************/

/** @file flink_brige_gpio.c
 *  @brief Subsystem-Bridge Module for gpio. 
 * 
 *  It connects the GPIO subdevices of a Flink device with 
 *  the GPIO subsystem of the Linux kernel. This bridge makes it 
 *  possible to use GPIO with the default libgpiod.
 *
 *  @author Patrick Good
 */

#include <linux/of.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/bitmap.h>
#include <linux/mfd/core.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>
#include "../flink.h"

MODULE_ALIAS("platform:" BRIDGE_GPIO_NAME);
MODULE_DESCRIPTION("Flink GPIO Subsystem Bridge");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Patrick Good");

/* ============== Defines Module ================= */
#define MODULE_NAME THIS_MODULE->name
#define MAX_LABEL_SIZE 32 /* Due to: Max Display length of userland tools (gpioinfo) */

/* ============== Register offsets ================= */
/**
 * TODO: 
 * Offsets descriptions export to a new file in flinkinterface.
 * Because it is now double defined (Hardcoded) in 
 * flinklinux and flinklib
 */
#define REGSIZE_BIT         32U
#define REGSIZE_BYTE        0x04U
#define REG_FUNCTION        0x00U /* offset in Bytes */
#define REG_MEM_SIZE        0x04U /* offset in Bytes */
#define REG_NOF_CHANNELS    0x08U /* offset in Bytes */
#define REG_UNIQUE_ID       0x0CU /* offset in Bytes */
#define REG_STATUS          0x10U /* offset in Bytes */
#define REG_CONFIG          0x14U /* offset in Bytes */
#define REG_GPIO_BASE_CLK   0x20U /* offset in Bytes */
#define REG_GPIO_DIR_0      0x24U /* offset in Bytes */
#define BANK_SHIFT          __builtin_ctz(REGSIZE_BIT) /* 5 bei 32 */
#define REG_STRIDE_SHIFT    __builtin_ctz(REGSIZE_BYTE)/* 2 bei 4 */

#define FLINK_DIRECTION_IN  0
#define FLINK_DIRECTION_OUT 1
#define FLINK_VALUE_LOW     0
#define FLINK_VALUE_HIGH    1

/* Calculates the direction register address */
static inline u32 REG_GPIO_DIR(u32 channel){return REG_GPIO_DIR_0+((channel>>BANK_SHIFT)<<REG_STRIDE_SHIFT);}
/* Calculates the first register address of the value registers */
static inline u32 REG_GPIO_VALUE_0(u32 nof_gpio) {return REG_GPIO_DIR_0 +((((nof_gpio-1)>>BANK_SHIFT)+1)<<REG_STRIDE_SHIFT);}
/* Calcualte the value register address*/
static inline u32 REG_GPIO_VALUE(u32 channel, u32 reg_gpio_val_0){return reg_gpio_val_0+((channel>>BANK_SHIFT)<<REG_STRIDE_SHIFT);}
/* Calculate the offset inside the register*/
static inline u32 OFFSET_IN_REG(u32 channel){return channel&(REGSIZE_BIT-1);}

/* ============== Bridge private data ================= */
struct flink_bridge_gpio {
    struct device          *dev;
    struct flink_subdevice *subdev;
    struct regmap          *map;
    struct gpio_chip       *gc;
    u32                     reg_gpio_val_0; /* first val reg offset */
    bool                    can_sleep;
    bool                    fast_io;
    char                    label[MAX_LABEL_SIZE];
};

/* ============== Regmap ================= */
struct fb_gpio_regctx {
    struct flink_subdevice *subdev;
    struct flink_bus_ops   *bus_ops;
    struct device          *dev;
};

static int fgpio_reg_read(void *context, unsigned int reg, unsigned int *val) {
    struct fb_gpio_regctx *ctx = context;
    u32 addr = ctx->subdev->base_addr + reg;
    *val = ctx->bus_ops->read32(ctx->subdev->parent, addr);
    #ifdef DBG
        printk(KERN_DEBUG "  -> reg_read\n");
        printk(KERN_DEBUG "    -> Device offset address: 0x%X\n", addr);
        printk(KERN_DEBUG "    -> Device gpio value:     0x%X\n", *val);
    #endif
    return 0;
}

static int fgpio_reg_write(void *context, unsigned int reg, unsigned int val) {
    struct fb_gpio_regctx *ctx = context;
    u32 addr = ctx->subdev->base_addr + reg;
    #ifdef DBG
        printk(KERN_DEBUG "  -> reg_write");
        printk(KERN_DEBUG "    -> Device offset address: 0x%x\n", addr);
        printk(KERN_DEBUG "    -> Device gpio value:     0x%x\n", val);
    #endif
    return ctx->bus_ops->write32(ctx->subdev->parent, addr, val);
}

/* ============== GPIO Lib Callbacks ================= */
/**
 * Definition of (return) values
 * Direction: 0=out, 1=in or GPIO_LINE_DIRECTION_OUT / GPIO_LINE_DIRECTION_IN
 * Value: 0=low, 1=high
 * Or negative if error
 */

static int fgpio_get_direction(struct gpio_chip *gc, unsigned int offset) {
    struct flink_bridge_gpio *fg = gpiochip_get_data(gc);
    #ifdef DBG
        printk(KERN_DEBUG "[%s] Get Direction device #%u GPIO's ...\n", MODULE_NAME, fg->subdev->parent->id);
        printk(KERN_DEBUG "  -> Device gpio offset:     0x%X\n", offset);
    #endif
    unsigned int reg = REG_GPIO_DIR(offset);
    unsigned int dir;
    int ret = regmap_read(fg->map, reg, &dir);
    if(unlikely(ret)) {
        return ret;
    }
    if((dir & BIT(OFFSET_IN_REG(offset))) == FLINK_DIRECTION_IN) {
        return GPIO_LINE_DIRECTION_IN;
    } else {
        return GPIO_LINE_DIRECTION_OUT;
    }
}

static int fgpio_set_direction_input(struct gpio_chip *gc, unsigned int offset) {
    struct flink_bridge_gpio *fg = gpiochip_get_data(gc);
    #ifdef DBG
        printk(KERN_DEBUG "[%s] Set Direction in device #%u GPIO's ...\n", MODULE_NAME, fg->subdev->parent->id);
        printk(KERN_DEBUG "  -> Device gpio offset:     0x%X\n", offset);
    #endif
    unsigned int reg = REG_GPIO_DIR(offset);
    unsigned int new_off = OFFSET_IN_REG(offset);
    unsigned int mask = BIT(new_off);
    unsigned int values = (FLINK_DIRECTION_IN) ? mask : 0;
    return regmap_update_bits(fg->map, reg, mask, values);
}

static int fgpio_set_direction_output(struct gpio_chip *gc,
                                      unsigned int offset,
                                      int value) {
    struct flink_bridge_gpio *fg = gpiochip_get_data(gc);
    #ifdef DBG
        printk(KERN_DEBUG "[%s] Set Direction out device #%u GPIO's ...\n", MODULE_NAME, fg->subdev->parent->id);
        printk(KERN_DEBUG "  -> Device gpio offset:     0x%X\n", offset);
        printk(KERN_DEBUG "  -> Device gpio value:     0x%X\n", value);
    #endif
    unsigned int new_off = OFFSET_IN_REG(offset);
    unsigned int mask = BIT(new_off);
    unsigned int reg, values;
    int ret;

    /* Set value of pin first to avoid glitches on the pin */
    /* BUG: This is the correct approach. However, the FPGA 
     * subdevice does not support this implementation. This is 
     * because, when the output is set to input, the value bit 
     * is read only. It must be set to 'Output' first in order 
     * to set the output value. Due to this bug, the direction 
     * is now set first, followed by the value. */
    
    /* Set direction of the pin */
    reg = REG_GPIO_DIR(offset);
    values = (FLINK_DIRECTION_OUT) ? mask : 0;
    ret = regmap_update_bits(fg->map, reg, mask, values);
    if(unlikely(ret)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to Update Direction of reg: %u pin: %u\n", MODULE_NAME, fg->subdev->parent->id, reg, new_off);
        return ret;
    }

    /* Set value of the pin */
    reg = REG_GPIO_VALUE(offset, fg->reg_gpio_val_0);
    values = (value) ? mask : 0;
    ret = regmap_update_bits(fg->map, reg, mask, values);
    if(unlikely(ret)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to Update Value of reg: %u pin: %u\n", MODULE_NAME, fg->subdev->parent->id, reg, new_off);
        return ret;
    }
    return ret;
}

static int fgpio_get_value(struct gpio_chip *gc, unsigned int offset) {
    struct flink_bridge_gpio *fg = gpiochip_get_data(gc);
    #ifdef DBG
        printk(KERN_DEBUG "[%s] Get Value device #%u GPIO's ...\n", MODULE_NAME, fg->subdev->parent->id);
        printk(KERN_DEBUG "  -> Device gpio offset:     0x%X\n", offset);
    #endif
    unsigned int reg = REG_GPIO_VALUE(offset, fg->reg_gpio_val_0);
    unsigned int new_off = OFFSET_IN_REG(offset);
    unsigned int val;
    int ret = regmap_read(fg->map, reg, &val);
    if(unlikely(ret)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to get value err: %i\n", MODULE_NAME, fg->subdev->parent->id, ret);
        printk(KERN_ERR "[%s]:Flink[%u]   -> Reg 0x%X \n", MODULE_NAME, fg->subdev->parent->id, reg);
        printk(KERN_ERR "[%s]:Flink[%u]   -> Values 0x%X \n", MODULE_NAME, fg->subdev->parent->id, val);     
        return ret;
    }
    return !!(val & BIT(new_off));
}

static void fgpio_set_value(struct gpio_chip *gc,
                             unsigned int offset,
                             int value) {
    struct flink_bridge_gpio *fg = gpiochip_get_data(gc);
    #ifdef DBG
        printk(KERN_DEBUG "[%s] Set Value device #%u GPIO's ...\n", MODULE_NAME, fg->subdev->parent->id);
        printk(KERN_DEBUG "  -> Device gpio offset:     0x%X\n", offset);
        printk(KERN_DEBUG "  -> Device gpio value:     0x%X\n", value);
    #endif
    unsigned int reg = REG_GPIO_VALUE(offset, fg->reg_gpio_val_0);
    unsigned int new_off = OFFSET_IN_REG(offset);
    unsigned int mask = BIT(new_off);
    unsigned int values = (value) ? mask : 0;
    if(unlikely(regmap_update_bits(fg->map, reg, mask, values)>0)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to set value\n", MODULE_NAME, fg->subdev->parent->id);
        printk(KERN_ERR "[%s]:Flink[%u]   -> Mask 0x%X \n", MODULE_NAME, fg->subdev->parent->id, mask);
        printk(KERN_ERR "[%s]:Flink[%u]   -> Values 0x%X \n", MODULE_NAME, fg->subdev->parent->id, values);     
    }
}

static int fgpio_get_multiple_values(struct gpio_chip *gc,
                                     unsigned long *mask,
                                     unsigned long *bits) {
    struct flink_bridge_gpio *fg = gpiochip_get_data(gc);
    u32 nbanks = DIV_ROUND_UP(gc->ngpio, REGSIZE_BIT);
    u32 mask32[nbanks], values32[nbanks];
    bitmap_to_arr32(mask32, mask, gc->ngpio);
    int ret;
    #ifdef DBG
        printk(KERN_DEBUG "[%s] Get multiple Value regs on device #%u GPIO's ...\n", MODULE_NAME, fg->subdev->parent->id);
        printk(KERN_DEBUG "  -> Device mask:  0x%*pb\n", gc->ngpio, mask);
    #endif
    for(int i=0; i<nbanks; i++) {
        if(mask32[i]>0) {
            ret = regmap_read(fg->map, fg->reg_gpio_val_0+i*REGSIZE_BYTE, &(values32[i]));
            if(unlikely(ret<0)) {
                printk(KERN_ERR "[%s]:Flink[%u] Failed to read value reg: %i err: %i\n", MODULE_NAME, fg->subdev->parent->id, i, ret);
                printk(KERN_ERR "[%s]:Flink[%u]   -> Mask 0x%X \n", MODULE_NAME, fg->subdev->parent->id, mask32[i]);
                printk(KERN_ERR "[%s]:Flink[%u]   -> Values 0x%X \n", MODULE_NAME, fg->subdev->parent->id, values32[i]);
                return ret;
            }
            values32[i] &= mask32[i];
        }
    }
    bitmap_from_arr32(bits, values32, gc->ngpio);
    return 0;
}

static void fgpio_set_multiple_values(struct gpio_chip *gc,
                                      unsigned long *mask,
                                      unsigned long *bits) {
    struct flink_bridge_gpio *fg = gpiochip_get_data(gc);
    u32 nbanks = DIV_ROUND_UP(gc->ngpio, REGSIZE_BIT);
    u32 mask32[nbanks], values32[nbanks];
    bitmap_to_arr32(mask32, mask, gc->ngpio);
    bitmap_to_arr32(values32, bits, gc->ngpio);
    int ret;
    #ifdef DBG
        printk(KERN_DEBUG "[%s] Set multiple Value regs on device #%u GPIO's ...\n", MODULE_NAME, fg->subdev->parent->id);
        printk(KERN_DEBUG "  -> Device mask:  0x%*pb\n", gc->ngpio, mask);
        printk(KERN_DEBUG "  -> Device value: 0x%*pb\n", gc->ngpio, bits);
    #endif
    for(int i=0; i<nbanks; i++) {
        if(mask32[i]>0) {
            ret = regmap_update_bits(fg->map, fg->reg_gpio_val_0+i*REGSIZE_BYTE, mask32[i], values32[i]);
            if(unlikely(ret<0)) {
                printk(KERN_ERR "[%s]:Flink[%u] Failed to Update value reg: %i err: %i\n", MODULE_NAME, fg->subdev->parent->id, i, ret);
                printk(KERN_ERR "[%s]:Flink[%u]   -> Mask 0x%X \n", MODULE_NAME, fg->subdev->parent->id, mask32[i]);
                printk(KERN_ERR "[%s]:Flink[%u]   -> Values 0x%X \n", MODULE_NAME, fg->subdev->parent->id, values32[i]);
                return;
            }
        }
    }
    return;
}

/* ============== Init/remove Functions ================= */
static int flink_bridge_gpio_probe(struct platform_device *pdev) {
    
    int ret = 0;
    struct device *dev = &pdev->dev;
    struct fb_gpio_regctx *rmctx; /* regmap context */
    struct regmap_config *rmcfg; /* regmap config */
    struct regmap_bus *rmbo; /* regmap bus ops */
    struct flink_bridge_gpio *fg = NULL;
    if(!dev) {
        printk(KERN_WARNING "[%s]: No Device by init flink_gpio_brige\n", MODULE_NAME);
        ret = -EINVAL;
        goto NO_DEV;
    }

    struct flink_subdevice *subdev = dev_get_platdata(dev);
    if(!subdev) {
        printk(KERN_WARNING "[%s]: No subdevice in platform_data available\n", MODULE_NAME);
        ret = -EINVAL;
        goto NO_DEV;
    }

    fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
    if(!fg) {
        printk(KERN_WARNING "[%s]: Faild to alloc private data\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_PRIV;
    }

    fg->gc = devm_kzalloc(dev, sizeof(*fg->gc), GFP_KERNEL);
    if(!(fg->gc)) {
        printk(KERN_WARNING "[%s]: Faild to alloc gpio_chip data\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_GPIO;
    }
    
    rmctx = devm_kzalloc(dev, sizeof(*rmctx), GFP_KERNEL);
    if(!(rmctx)) {
        printk(KERN_WARNING "[%s]: Faild to alloc rctx\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_RCTX;
    }

    rmcfg = devm_kzalloc(dev, sizeof(*rmcfg) ,GFP_KERNEL);
    if(!(rmcfg)) {
        printk(KERN_WARNING "[%s]: Faild to alloc rmcfg\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_RMCFG;
    }

    rmbo = devm_kzalloc(dev, sizeof(*rmbo), GFP_KERNEL);
    if(!(rmbo)) {
        printk(KERN_WARNING "[%s]: Faild to alloc rmbo\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_RMBO;
    }

    fg->dev = dev;
    fg->subdev = subdev;
    fg->reg_gpio_val_0 = REG_GPIO_VALUE_0(subdev->nof_channels);
    scnprintf(fg->label, sizeof(fg->label), 
              "flink%u-bridge-gpio:0x%08X", subdev->parent->id, subdev->unique_id);
    /**
     * TODO:
     * can_sleep and fast_io should be kept variable. 
     * Since wen bus ops go to spi or i2c, it must be true; 
     * for MMIO access, it could be false. For fast_io, it 
     * should be true if platformbius is used. Then a 
     * spinlock is used as a lock instead of a mutex.
     */
    fg->can_sleep = true;
    fg->fast_io   = false;

    fg->gc->label              = fg->label;
    fg->gc->parent             = dev;
    fg->gc->owner              = THIS_MODULE;
    fg->gc->base               = -1;
    fg->gc->ngpio              = fg->subdev->nof_channels;
    fg->gc->get_direction      = fgpio_get_direction;
    fg->gc->direction_input    = fgpio_set_direction_input;
    fg->gc->direction_output   = fgpio_set_direction_output;
    fg->gc->get                = fgpio_get_value;
    fg->gc->set                = fgpio_set_value;
    fg->gc->can_sleep          = fg->can_sleep;
    fg->gc->get_multiple       = fgpio_get_multiple_values;
    fg->gc->set_multiple       = fgpio_set_multiple_values;

    rmctx->subdev      = subdev;
    rmctx->dev         = dev;
    rmctx->bus_ops     = subdev->parent->bus_ops;
    
    rmcfg->name         = fg->label;
    rmcfg->reg_bits     = REGSIZE_BIT;
    rmcfg->reg_stride   = REGSIZE_BYTE;
    rmcfg->pad_bits     = 0;
    rmcfg->val_bits     = REGSIZE_BIT;
    rmcfg->fast_io      = fg->fast_io;
    rmcfg->max_register = REG_GPIO_VALUE(subdev->nof_channels, fg->reg_gpio_val_0);
    rmcfg->wr_table;
    rmcfg->rd_table;
    rmcfg->volatile_table;
    rmcfg->precious_table;
    rmcfg->wr_noinc_table;
    rmcfg->rd_noinc_table;
    rmcfg->use_single_read  = true;
    rmcfg->use_single_write = true;
    rmcfg->can_multi_write  = false;
    rmcfg->cache_type       = REGCACHE_NONE;

    rmbo->fast_io                   = fg->fast_io;
    rmbo->reg_read                  = fgpio_reg_read;
    rmbo->reg_write                 = fgpio_reg_write;
    rmbo->val_format_endian_default = REGMAP_ENDIAN_NATIVE;
    rmbo->reg_format_endian_default = REGMAP_ENDIAN_NATIVE;

    fg->map = devm_regmap_init(dev, rmbo, rmctx, rmcfg);
    if (!fg->map) {
        printk(KERN_WARNING "[%s]: Faild to init regmap\n", MODULE_NAME);
        ret = -EPERM;
        goto INIT_REGMAP;
    }

    platform_set_drvdata(pdev, fg);
    ret = devm_gpiochip_add_data(dev, fg->gc, fg);
    if (ret) {
        printk(KERN_WARNING "[%s]: Faild to init regmap\n", MODULE_NAME);
        ret = -EPERM;
        goto INIT_GPIO_CHIP;
    }

    printk(KERN_INFO "[%s] Flink%u %s:0x%X connected to Subsystem \n", MODULE_NAME, subdev->parent->id, fmit_lkm_lut[subdev->function_id], subdev->unique_id);

    return ret;

    INIT_GPIO_CHIP:
    INIT_REGMAP:
    ALLOC_RMBO:
    ALLOC_RMCFG:
    ALLOC_RCTX:
    ALLOC_SPINLOCK:
    ALLOC_GPIO:
    ALLOC_PRIV:
    printk(KERN_INFO "[%s] %s:0x%X faild to connect to Subsystem \n", MODULE_NAME, fmit_lkm_lut[subdev->function_id], subdev->unique_id);
    NO_DEV:
    printk(KERN_INFO "[%s] failed to connect to Subsystem \n", MODULE_NAME);

    return ret;
}

static int flink_bridge_gpio_remove(struct platform_device *pdev) {
    dev_info(&pdev->dev, "flink_bridge_gpio removed.\n");
    return 0;
}

static struct platform_driver flink_bridge_gpio_driver = {
    .probe  = flink_bridge_gpio_probe,
    .remove = flink_bridge_gpio_remove,
    .driver = {
        .name = BRIDGE_GPIO_NAME,
    },
};
module_platform_driver(flink_bridge_gpio_driver);
