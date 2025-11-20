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
 *                                                                 *
 *                                                                 *
 *******************************************************************/

/** @file flink_brige_wd.c
 *  @brief Subsystem-Bridge Module for watchdog. 
 * 
 *  It connects the Watchdog subdevices of a Flink device with 
 *  the Watchdog subsystem of the Linux kernel. This bridge makes it 
 *  possible to use watchdog with the default watchdog lib of linux.
 *
 *  @author Patrick Good
 */

#include <linux/of.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include "../flink.h"

MODULE_AUTHOR("Patrick Good");
MODULE_DESCRIPTION("Flink Watchdog Subsystem Bridge");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:" BRIDGE_WD_NAME);


/* ============== Defines Module ================= */
#define MODULE_NAME THIS_MODULE->name
#define NS_PER_SEC (u64)1e9 
#define DEFAULT_TIMEOUT 5 /* sec */
#define MIN_TIMEOUT 1 /* sec */
#define FIRMWARE_VERSION 1

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
#define REG_BASE_CLK        0x20U /* offset in Bytes */
#define REG_COUNTER         0x24U /* offset in Bytes */
#define BIT_ARMED           BIT(0)


static inline u32 FLINK_READ(struct flink_subdevice *sdev, u32 reg){return sdev->parent->bus_ops->read32(sdev->parent, sdev->base_addr+reg);}
static inline int FLINK_WRITE(struct flink_subdevice *sdev, u32 reg, u32 val){return sdev->parent->bus_ops->write32(sdev->parent, sdev->base_addr+reg, val);}

/* ============== Bridge private data ================= */
struct flink_bridge_wd {
    struct device          *dev;
    struct flink_subdevice *subdev;
    struct watchdog_ops    *wd_ops;
    struct watchdog_device *wd_dev;
    u32                    clk_hz;
};

static inline u32 SEC_TO_STEPS(struct flink_bridge_wd *fw, unsigned int sec){return sec*fw->clk_hz;}
static inline unsigned int STEPS_TO_SEC(struct flink_bridge_wd *fw, u32 steps){return steps/fw->clk_hz;}

/* ============== Watchdog Lib Callbacks ================= */
static int fwd_start(struct watchdog_device *wd) {
    struct flink_bridge_wd *fw = watchdog_get_drvdata(wd);
    int ret = 0;
    #ifdef DBG
        printk(KERN_DEBUG "Flink%u:[%s] start WD\n", fw->subdev->parent->id, MODULE_NAME);
        printk(KERN_DEBUG "  -> sec:  %u\n", wd->timeout);
    #endif
    /* Set timeout */
    u32 steps = SEC_TO_STEPS(fw, wd->timeout);
    ret = FLINK_WRITE(fw->subdev, REG_COUNTER, steps);
    if(unlikely(ret<0)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to set counter watchdog:0x%X err: \n", MODULE_NAME, fw->subdev->parent->id, fw->subdev->unique_id, ret);
        return ret;
    }

    /* Arming */
    /* No RMW Operation needed, Because ARM Bit is the only acive bit in this reg */
    ret = FLINK_WRITE(fw->subdev, REG_CONFIG, BIT_ARMED);
    if(unlikely(ret<0)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to arm watchdog:0x%X err: \n", MODULE_NAME, fw->subdev->parent->id, fw->subdev->unique_id, ret);
        return ret;
    }
    return ret;
}

static int fwd_stop(struct watchdog_device *wd) {
    struct flink_bridge_wd *fw = watchdog_get_drvdata(wd);
    int ret = 0;
    #ifdef DBG
        printk(KERN_DEBUG "Flink%u:[%s] Stopping\n", fw->subdev->parent->id, MODULE_NAME);
    #endif
    /* dearm */
    /* No RMW Operation needed, Because ARM Bit is the only acive bit in this reg */
    ret = FLINK_WRITE(fw->subdev, REG_CONFIG, (u32)0);
    if(unlikely(ret<0)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to arm watchdog:0x%X err: \n", MODULE_NAME, fw->subdev->parent->id, fw->subdev->unique_id, ret);
        return ret;
    }
    
    /* Set timeout */
    ret = FLINK_WRITE(fw->subdev, REG_COUNTER, (u32)0);
    if(unlikely(ret<0)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to set counter watchdog:0x%X err: \n", MODULE_NAME, fw->subdev->parent->id, fw->subdev->unique_id, ret);
        return ret;
    }
    
    return ret;
}

static int fwd_ping(struct watchdog_device *wd) {
    struct flink_bridge_wd *fw = watchdog_get_drvdata(wd);
    int ret = 0;
    #ifdef DBG
        printk(KERN_DEBUG "Flink%u:[%s] send Ping to WD dev\n", fw->subdev->parent->id, MODULE_NAME);
        printk(KERN_DEBUG "  -> sec:  %u\n", wd->timeout);
    #endif
    /* Set timeout */
    u32 steps = SEC_TO_STEPS(fw, wd->timeout);
    ret = FLINK_WRITE(fw->subdev, REG_COUNTER, steps);
    if(unlikely(ret<0)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to set counter watchdog:0x%X err: \n", MODULE_NAME, fw->subdev->parent->id, fw->subdev->unique_id, ret);
        return ret;
    }
    
    return ret;
}

static int fwd_set_timeout(struct watchdog_device *wd, unsigned int timeout_sec) {
    struct flink_bridge_wd *fw = watchdog_get_drvdata(wd);
    int ret = 0;

    #ifdef DBG
        printk(KERN_DEBUG "Flink%u:[%s] set timeout\n", fw->subdev->parent->id, MODULE_NAME);
        printk(KERN_DEBUG "  -> sec:  %u\n", timeout_sec);
    #endif

    wd->timeout = timeout_sec;

    u32 steps = SEC_TO_STEPS(fw, timeout_sec);
    ret = FLINK_WRITE(fw->subdev, REG_COUNTER, steps);
    if(unlikely(ret<0)) {
        printk(KERN_ERR "[%s]:Flink[%u] Failed to set counter watchdog:0x%X err: \n", MODULE_NAME, fw->subdev->parent->id, fw->subdev->unique_id, ret);
        return ret;
    }
    return ret;
}

static unsigned int fwd_get_timeleft(struct watchdog_device *wd) {
    struct flink_bridge_wd *fw = watchdog_get_drvdata(wd);
    u32 timeleft_ticks = FLINK_READ(fw->subdev, REG_COUNTER);
    #ifdef DBG
        printk(KERN_DEBUG "Flink%u:[%s] get timeleft\n", fw->subdev->parent->id, MODULE_NAME);
        printk(KERN_DEBUG "  -> sec:  %u\n", STEPS_TO_SEC(fw, timeleft_ticks));
    #endif
    return STEPS_TO_SEC(fw, timeleft_ticks);
}

/* ============== Init/remove Functions ================= */
static int flink_bridge_wd_probe(struct platform_device *pdev) {
    int ret = 0;
    struct device *dev = &pdev->dev;
    struct flink_bridge_wd *fw = NULL;
    struct watchdog_info *wd_i;
    if(!dev) {
        printk(KERN_WARNING "[%s]: No Device by init flink_watchdog_brige\n", MODULE_NAME);
        ret = -EINVAL;
        goto NO_DEV;
    }

    struct flink_subdevice *subdev = dev_get_platdata(dev);
    if(!subdev) {
        printk(KERN_WARNING "[%s]: No subdevice in platform_data available\n", MODULE_NAME);
        ret = -EINVAL;
        goto NO_DEV;
    }

    fw = devm_kzalloc(dev, sizeof(*fw), GFP_KERNEL);
    if(!fw) {
        printk(KERN_WARNING "[%s]: Faild to alloc private data\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_PRIV;
    }

    fw->wd_ops = devm_kzalloc(dev, sizeof(*(fw->wd_ops)), GFP_KERNEL);
    if(!(fw->wd_ops)) {
        printk(KERN_WARNING "[%s]: Faild to alloc bus data\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_WDBUS;
    }

    fw->wd_dev = devm_kzalloc(dev, sizeof(*(fw->wd_dev)), GFP_KERNEL);
    if(!(fw->wd_dev)) {
        printk(KERN_WARNING "[%s]: Faild to alloc wachdog device data\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_WD_DEV;
    }

    wd_i = devm_kzalloc(dev, sizeof(*wd_i), GFP_KERNEL);
    if(!wd_i) {
        printk(KERN_WARNING "[%s]: Faild to alloc watchdog info data\n", MODULE_NAME);
        ret = -ENOMEM;
        goto ALLOC_WD_INFO;
    }

    scnprintf(wd_i->identity, sizeof(wd_i->identity), 
        "flink%u-bridge-wd:0x%08X", subdev->parent->id, subdev->unique_id);
    wd_i->firmware_version = FIRMWARE_VERSION;
    wd_i->options = WDIOF_CARDRESET | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING;

    fw->subdev = subdev;
    fw->dev = dev;
    fw->clk_hz = FLINK_READ(subdev , REG_BASE_CLK);

    fw->wd_ops->owner = THIS_MODULE;
    fw->wd_ops->start = fwd_start;
    fw->wd_ops->stop = fwd_stop;
    fw->wd_ops->ping = fwd_ping;
    fw->wd_ops->set_timeout = fwd_set_timeout;
    fw->wd_ops->get_timeleft = fwd_get_timeleft;

    fw->wd_dev->parent = dev;
    fw->wd_dev->info = wd_i;
    fw->wd_dev->ops = fw->wd_ops;
    fw->wd_dev->min_timeout = MIN_TIMEOUT;
    fw->wd_dev->max_timeout = U32_MAX/fw->clk_hz;

    watchdog_stop_on_reboot(fw->wd_dev);
    watchdog_stop_on_unregister(fw->wd_dev);
    watchdog_set_drvdata(fw->wd_dev, fw);

    ret = watchdog_init_timeout(fw->wd_dev, DEFAULT_TIMEOUT, dev);
    if(unlikely(ret<0)) {
        printk(KERN_WARNING "[%s]: Faild to init timeout watchdog \n", MODULE_NAME);
        goto ALLOC_WD_INFO;
    }

    ret = devm_watchdog_register_device(dev, fw->wd_dev);
    if(unlikely(ret<0)) {
        printk(KERN_WARNING "[%s]: Faild to register watchdog \n", MODULE_NAME);
        goto ALLOC_WD_INFO;
    }

    printk(KERN_INFO "[%s] Flink%u %s:0x%X connected to Subsystem \n", MODULE_NAME, subdev->parent->id, fmit_lkm_lut[subdev->function_id], subdev->unique_id);

    return ret;

    ALLOC_WD_INFO:
    ALLOC_WD_DEV:
    ALLOC_WDBUS:
    ALLOC_PRIV:
    printk(KERN_ERR "[%s] %s:0x%X faild to connect to Subsystem \n", MODULE_NAME, fmit_lkm_lut[subdev->function_id], subdev->unique_id);
    NO_DEV:
    printk(KERN_ERR "[%s] failed to connect to Subsystem \n", MODULE_NAME);

    return ret;
}

static int flink_bridge_wd_remove(struct platform_device *pdev) {
    struct flink_subdevice *subdev = dev_get_platdata(&pdev->dev);
    printk(KERN_INFO "Flink%u %s:0x%X faild to connect to Subsystem \n", subdev->parent->id, fmit_lkm_lut[subdev->function_id], subdev->unique_id);
    return 0;
}

static struct platform_driver flink_bridge_wd_driver = {
    .probe  = flink_bridge_wd_probe,
    .remove = flink_bridge_wd_remove,
    .driver = {
        .name = BRIDGE_WD_NAME,
    },
};
module_platform_driver(flink_bridge_wd_driver);