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

/** @file flink_core.c
 *  @brief Core module for flink. 
 * 
 *  Contains functions to initialize, add and remove flink devices 
 *  and subdevices 
 *
 *  @author Martin Züger
 *  @author Urs Graf
 *  @author Patrick Good
 * 
 *  Changelog
 *  Date      Who   What
 *  28.10.23  Good  Added interrupt capability
 * 					-> Added ioctl #50 #51 #52
 * 					-> Added IRQ service routine
 * 					-> Added flink_device_init_irq(...)
 * 					-> Adjusted flink_device_init(...) & flink_device_delete(...)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/signal.h>
#include <linux/sched/signal.h>
#include <linux/mfd/core.h>

#include "flink.h"

#define MODULE_NAME THIS_MODULE->name
#define SYSFS_CLASS_NAME "flink"
#define MAX_DEV_NAME_LENGTH 15

MODULE_AUTHOR("Martin Zueger <martin@zueger.eu>");
MODULE_DESCRIPTION("fLink core module");
MODULE_LICENSE("Dual BSD/GPL");

static LIST_HEAD(device_list);
static LIST_HEAD(loaded_if_modules);
static struct class* sysfs_class;

// ###### Internal Function Prototypes ######
// do NOT call this directly!!! this function is called over an irq number
static irqreturn_t flink_threaded_irq_handler(int irq, void *dev_id);

// ############ File operations ############

int flink_open(struct inode* i, struct file* f) {
	struct flink_device* fdev = flink_get_device_by_cdev(i->i_cdev);
	struct flink_private_data* p_data = kmalloc(sizeof(struct flink_private_data), GFP_KERNEL);
	memset(p_data, 0, sizeof(*p_data));
	p_data->fdev = fdev;
	f->private_data = p_data;
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] Device node opened.", MODULE_NAME);
	#endif
	return 0;
}

int flink_relase(struct inode* i, struct file* f) {
	kfree(f->private_data);
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] Device node closed.", MODULE_NAME);
	#endif
	return 0;
}

ssize_t flink_read(struct file* f, char __user* data, size_t size, loff_t* offset) {
	struct flink_private_data* pdata = (struct flink_private_data*)(f->private_data);
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] Reading from device...", MODULE_NAME);
	#endif
	if(pdata != NULL && pdata->current_subdevice != NULL) {
		struct flink_subdevice* subdev = pdata->current_subdevice;
		struct flink_device* fdev = subdev->parent;
		unsigned long rsize = 0;
		u32 roffset = (u32)*offset;;
		#if defined(DBG)
			printk(KERN_DEBUG "  -> Device: %u/%u", fdev->id, subdev->id);
			printk(KERN_DEBUG "  -> Size:   0x%x (%u bytes)", (unsigned int)size, (unsigned int)size);
			printk(KERN_DEBUG "  -> Offset: 0x%x", (u32)*offset);
		#endif
		if(roffset > subdev->mem_size) {
			return 0;
		}
		switch(size) {
			case 1: {
				u8 rdata = 0;
				rdata = fdev->bus_ops->read8(fdev, subdev->base_addr + roffset);
				rsize = copy_to_user(data, &rdata, sizeof(rdata));
				if(rsize > 0) {
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Copying to user space failed: %lu bytes not copied!", rsize);
					#endif
					return 0;
				}
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Value:  0x%x", rdata);
				#endif
				return sizeof(rdata);
			}
			case 2: {
				u16 rdata = 0;
				rdata = fdev->bus_ops->read16(fdev, subdev->base_addr + roffset);
				rsize = copy_to_user(data, &rdata, sizeof(rdata));
				if(rsize > 0) {
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Copying to user space failed: %lu bytes not copied!", rsize);
					#endif
					return 0;
				}
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Value:  0x%x", rdata);
				#endif
				return sizeof(rdata);
			}
			case 4: {
				u32 rdata = 0;
				rdata = fdev->bus_ops->read32(fdev, subdev->base_addr + roffset);
				rsize = copy_to_user(data, &rdata, sizeof(rdata));
				if(rsize > 0) {
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Copying to user space failed: %lu bytes not copied!", rsize);
					#endif
					return 0;
				}
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Value:  0x%x", rdata);
				#endif
				return sizeof(rdata);
			}
			default:
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Size of transfer not supported: %lu bytes!", (long unsigned int)size);
				#endif
				return 0;
		}
	}
	return 0;
}

ssize_t flink_write(struct file* f, const char __user* data, size_t size, loff_t* offset) {
	struct flink_private_data* pdata = (struct flink_private_data*)(f->private_data);
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] Writing to device...", MODULE_NAME);
	#endif
	if(pdata != NULL && pdata->current_subdevice != NULL) {
		struct flink_subdevice* subdev = pdata->current_subdevice;
		struct flink_device* fdev = subdev->parent;
		u32 woffset = (u32)*offset;
		unsigned long wsize = 0;
		#if defined(DBG)
			printk(KERN_DEBUG "  -> Device: %u/%u", fdev->id, subdev->id);
			printk(KERN_DEBUG "  -> Size:   0x%x (%u bytes)", (unsigned int)size, (unsigned int)size);
			printk(KERN_DEBUG "  -> Offset: 0x%x", (u32)*offset);
		#endif
		if(woffset > subdev->mem_size) {
			return 0;
		}
		switch(size) {
			case 1: {
			  	u8 wdata = 0;
				wsize = copy_from_user(&wdata, data, sizeof(wdata));
				if(wsize > 0) {
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Copying from user space failed: %lu bytes not copied!", wsize);
					#endif
					return 0;
				}
				fdev->bus_ops->write8(fdev, subdev->base_addr + woffset, wdata);
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Value:  0x%x", wdata);
				#endif
				return sizeof(wdata);
			}
			case 2: {
			  	u16 wdata = 0;
				wsize = copy_from_user(&wdata, data, sizeof(wdata));
				if(wsize > 0) {
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Copying from user space failed: %lu bytes not copied!", wsize);
					#endif
					return 0;
				}
				fdev->bus_ops->write16(fdev, subdev->base_addr + woffset, wdata);
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Value:  0x%x", wdata);
				#endif
				return sizeof(wdata);
			}
			case 4: {
			  	u32 wdata = 0;
				wsize = copy_from_user(&wdata, data, sizeof(wdata));
				if(wsize > 0) {
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Copying from user space failed: %lu bytes not copied!", wsize);
					#endif
					return 0;
				}
				fdev->bus_ops->write32(fdev, subdev->base_addr + woffset, wdata);
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Value:  0x%x", wdata);
				#endif
				return sizeof(wdata);
			}
			default:
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Size of transfer not supported: %lu bytes!", (long unsigned int)size);
				#endif
				return 0;
		}
	}
	return 0;
}

long flink_ioctl(struct file* f, unsigned int cmd, unsigned long arg) {
	int error = 0;
	struct flink_private_data* pdata = (struct flink_private_data*)(f->private_data);
	struct flink_subdevice* src;
	u8 id;
	struct ioctl_bit_container_t rwbit_container;
	struct ioctl_container_t rw_container;
	unsigned long rsize = 0;
	unsigned long wsize = 0;
	u32 temp;

	// for irq register and unregister
	struct flink_process_data* fsignal;
	struct task_struct* user_task = get_current();
	bool found_entry = false;
	u32 requested_irq_nr = 0;
	struct flink_irq_data* hwirq;
	
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] I/O control call...", MODULE_NAME);
	#endif
	switch(cmd) {
		case SELECT_SUBDEVICE:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> SELECT_SUBDEVICE (0x%x)", SELECT_SUBDEVICE);
			#endif
			error = copy_from_user(&id, (void __user *)arg, sizeof(id));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			return flink_select_subdevice(f, id, 0);
		case SELECT_SUBDEVICE_EXCL:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> SELECT_SUBDEVICE_EXCL (0x%x)", SELECT_SUBDEVICE_EXCL);
			#endif
			error = copy_from_user(&id, (void __user *)arg, sizeof(id));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			return flink_select_subdevice(f, id, 1);
		case READ_NOF_SUBDEVICES:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> READ_NOF_SUBDEVICES (0x%x) -> %u", READ_NOF_SUBDEVICES, pdata->fdev->nof_subdevices);
			#endif
			error = copy_to_user((void __user *)arg, &(pdata->fdev->nof_subdevices), sizeof(pdata->fdev->nof_subdevices));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying to userspace: %i", error);
				#endif
				return -EINVAL;
			}
			break;
		case READ_SUBDEVICE_INFO:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> READ_SUBDEVICE_INFO (0x%x)", READ_SUBDEVICE_INFO);
			#endif
			error = copy_from_user(&id, (void __user *)arg, sizeof(id));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			if(id >= pdata->fdev->nof_subdevices) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Illegal subdevice id");
				#endif
				return -EINVAL;
			}
			src = flink_get_subdevice_by_id(pdata->fdev, id);
			if(src == NULL) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Getting kernel subdevice structure failed.");
				#endif
				return -EINVAL;
			}
			error = copy_to_user((void __user *)arg, &(src->id), FLINKLIB_SUBDEVICE_SIZE);
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying to userspace: %i", error);
				#endif
				return -EINVAL;
			}
			break;
		case READ_SINGLE_BIT:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> READ_SINGLE_BIT (0x%x)", READ_SINGLE_BIT);
			#endif
			error = copy_from_user(&rwbit_container, (void __user *)arg, sizeof(rwbit_container));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			temp = pdata->fdev->bus_ops->read32(pdata->fdev, pdata->current_subdevice->base_addr + rwbit_container.offset);
			#if defined(DBG)
				printk(KERN_DEBUG "  -> Read from device: 0x%x", temp);
			#endif
			rwbit_container.value = ((temp & (1 << rwbit_container.bit)) != 0);
			#if defined(DBG)
				printk(KERN_DEBUG "  -> Bit value: 0x%x", rwbit_container.value);
			#endif
			error = copy_to_user((void __user *)arg, &rwbit_container, sizeof(rwbit_container));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying to userspace: %i", error);
				#endif
				return -EINVAL;
			}
			break;
		case WRITE_SINGLE_BIT:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> WRITE_SINGLE_BIT (0x%x)", WRITE_SINGLE_BIT);
			#endif
			error = copy_from_user(&rwbit_container, (void __user *)arg, sizeof(rwbit_container));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			else {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Copied from user space: offset = 0x%x, bit = %u, value = %u", rwbit_container.offset, rwbit_container.bit, rwbit_container.value);
				#endif
			}
			temp = pdata->fdev->bus_ops->read32(pdata->fdev, pdata->current_subdevice->base_addr + rwbit_container.offset);
			#if defined(DBG)
				printk(KERN_DEBUG "  -> Read from device: 0x%x", temp);
			#endif
			if(rwbit_container.value != 0) { // set bit
				temp |= (1 << rwbit_container.bit);
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Setting bit by writing 0x%x to device", temp);
				#endif
			}
			else { // clear bit
				temp &= ~(1 << rwbit_container.bit);
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Clearing bit by writing 0x%x to device", temp);
				#endif
			}
			pdata->fdev->bus_ops->write32(pdata->fdev, pdata->current_subdevice->base_addr + rwbit_container.offset, temp);
			break;
		case SELECT_AND_READ_BIT:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> SELECT_AND_READ_BIT (0x%x)", SELECT_AND_READ_BIT);
			#endif
			error = copy_from_user(&rwbit_container, (void __user *)arg, sizeof(rwbit_container));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			src = flink_get_subdevice_by_id(pdata->fdev, rwbit_container.subdevice);
			if(src == NULL) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Getting kernel subdevice structure failed.");
				#endif
				return -EINVAL;
			}
			temp = pdata->fdev->bus_ops->read32(pdata->fdev, src->base_addr + rwbit_container.offset);
			#if defined(DBG)
				printk(KERN_DEBUG "  -> Read from device: 0x%x", temp);
			#endif
			rwbit_container.value = ((temp & (1 << rwbit_container.bit)) != 0);
			#if defined(DBG)
				printk(KERN_DEBUG "  -> Bit value: 0x%x", rwbit_container.value);
			#endif
			error = copy_to_user((void __user *)arg, &rwbit_container, sizeof(rwbit_container));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying to userspace: %i", error);
				#endif
				return -EINVAL;
			}
			break;
		case SELECT_AND_WRITE_BIT:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> SELECT_AND_WRITE_BIT (0x%x)", SELECT_AND_WRITE_BIT);
			#endif
			error = copy_from_user(&rwbit_container, (void __user *)arg, sizeof(rwbit_container));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			else {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Copied from user space: offset = 0x%x, bit = %u, value = %u", rwbit_container.offset, rwbit_container.bit, rwbit_container.value);
				#endif
			}
			src = flink_get_subdevice_by_id(pdata->fdev, rwbit_container.subdevice);
			if(src == NULL) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Getting kernel subdevice structure failed.");
				#endif
				return -EINVAL;
			}
			temp = pdata->fdev->bus_ops->read32(pdata->fdev, src->base_addr + rwbit_container.offset);
			#if defined(DBG)
				printk(KERN_DEBUG "  -> Read from device: 0x%x", temp);
			#endif
			if(rwbit_container.value != 0) { // set bit
				temp |= (1 << rwbit_container.bit);
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Setting bit by writing 0x%x to device", temp);
				#endif
			}
			else { // clear bit
				temp &= ~(1 << rwbit_container.bit);
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Clearing bit by writing 0x%x to device", temp);
				#endif
			}
			pdata->fdev->bus_ops->write32(pdata->fdev, src->base_addr + rwbit_container.offset, temp);
			break;
		case SELECT_AND_READ:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> SELECT_AND_READ (0x%x)", SELECT_AND_READ);
			#endif
			error = copy_from_user(&rw_container, (void __user *)arg, sizeof(rw_container));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			if (rw_container.data == NULL) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> NULL pointer");
				#endif
				return -EINVAL;
			}
			src = flink_get_subdevice_by_id(pdata->fdev, rw_container.subdevice);
			if(src == NULL) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Getting kernel subdevice structure failed.");
				#endif
				return -EINVAL;
			}
			if (rw_container.offset > src->mem_size) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> offset > mem_size");
				#endif
				return -EINVAL;
			}
			switch(rw_container.size) {
				case 1: {
					u8 rdata = 0;
					rdata = pdata->fdev->bus_ops->read8(pdata->fdev, src->base_addr + rw_container.offset);
					rsize = copy_to_user((void __user *)rw_container.data, &rdata, sizeof(rdata));
					if(rsize > 0) {
						#if defined(DBG)
							printk(KERN_DEBUG "  -> Copying to user space failed: %lu bytes not copied!", rsize);
						#endif
						return 0;
					}
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Value:  0x%x", rdata);
					#endif
					return sizeof(rdata);
				}
				case 2: {
					u16 rdata = 0;
					rdata = pdata->fdev->bus_ops->read16(pdata->fdev, src->base_addr + rw_container.offset);
					rsize = copy_to_user((void __user *)rw_container.data, &rdata, sizeof(rdata));
					if(rsize > 0) {
						#if defined(DBG)
							printk(KERN_DEBUG "  -> Copying to user space failed: %lu bytes not copied!", rsize);
						#endif
						return 0;
					}
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Value:  0x%x", rdata);
					#endif
					return sizeof(rdata);
				}
				case 4: {
					u32 rdata = 0;
					rdata = pdata->fdev->bus_ops->read32(pdata->fdev, src->base_addr + rw_container.offset);
					rsize = copy_to_user((void __user *)rw_container.data, &rdata, sizeof(rdata));
					if(rsize > 0) {
						#if defined(DBG)
							printk(KERN_DEBUG "  -> Copying to user space failed: %lu bytes not copied!", rsize);
						#endif
						return 0;
					}
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Value:  0x%x", rdata);
					#endif
					return sizeof(rdata);
				}
				default:
					return -EINVAL;
			}
			break;
		case SELECT_AND_WRITE:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> SELECT_AND_WRITE (0x%x)", SELECT_AND_WRITE);
			#endif
			error = copy_from_user(&rw_container, (void __user *)arg, sizeof(rw_container));
			if(error != 0) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Error while copying from userspace: %i", error);
				#endif
				return -EINVAL;
			}
			if (rw_container.data == NULL) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> NULL pointer");
				#endif
				return -EINVAL;
			}
			src = flink_get_subdevice_by_id(pdata->fdev, rw_container.subdevice);
			if(src == NULL) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Getting kernel subdevice structure failed.");
				#endif
				return -EINVAL;
			}
			if (rw_container.offset > src->mem_size) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> offset > mem_size");
				#endif
				return -EINVAL;
			}
			switch(rw_container.size) {
				case 1: {
					u8 wdata = 0;
					wsize = copy_from_user(&wdata, (void __user *)rw_container.data, sizeof(wdata));
					if(wsize > 0) {
						#if defined(DBG)
							printk(KERN_DEBUG "  -> Copying from user space failed: %lu bytes not copied!", wsize);
						#endif
						return -EINVAL;
					}
					pdata->fdev->bus_ops->write8(pdata->fdev, src->base_addr + rw_container.offset, wdata);
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Value:  0x%x", wdata);
					#endif
					return sizeof(wdata);
				}
				case 2: {
					u16 wdata = 0;
					wsize = copy_from_user(&wdata, (void __user *)rw_container.data, sizeof(wdata));
					if(wsize > 0) {
						#if defined(DBG)
							printk(KERN_DEBUG "  -> Copying from user space failed: %lu bytes not copied!", wsize);
						#endif
						return -EINVAL;
					}
					pdata->fdev->bus_ops->write16(pdata->fdev, src->base_addr + rw_container.offset, wdata);
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Value:  0x%x", wdata);
					#endif
					return sizeof(wdata);
				}
				case 4: {
					u32 wdata = 0;
					wsize = copy_from_user(&wdata, (void __user *)rw_container.data, sizeof(wdata));
					if(wsize > 0) {
						#if defined(DBG)
							printk(KERN_DEBUG "  -> Copying from user space failed: %lu bytes not copied!", wsize);
						#endif
						return -EINVAL;
					}
					pdata->fdev->bus_ops->write32(pdata->fdev, src->base_addr + rw_container.offset, wdata);
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Value:  0x%x", wdata);
					#endif
					return sizeof(wdata);
				}
				default:
					return -EINVAL;
			}
			break;
		case REGISTER_IRQ: 
			#if defined(DBG)
				printk(KERN_DEBUG "[%s] Register IRQ (0x%x)", MODULE_NAME, REGISTER_IRQ);
			#endif
			if(unlikely(pdata->fdev->nof_irqs == 0)) {
				printk(KERN_WARNING "[%s] Irq functionality not available", MODULE_NAME);
				return -EPERM;
			}
			error = copy_from_user(&rw_container, (void __user *)arg, sizeof(rw_container));
			if(unlikely(error != 0)) {
				printk(KERN_WARNING "[%s] Error while copying from userspace: %i", MODULE_NAME, error);
				return -EINVAL;
			}
			if(unlikely(rw_container.size != 4)) {
				printk(KERN_WARNING "[%s] size must have lenght of 4 bytes aka uint32_t", MODULE_NAME);
				return -EINVAL;
			}
			wsize = copy_from_user(&requested_irq_nr, (void __user *)rw_container.data, sizeof(requested_irq_nr));
			if(unlikely(wsize > 0)) {
				printk(KERN_WARNING "[%s] Copying from user space failed: %lu bytes not copied!", MODULE_NAME, wsize);
				return -EINVAL;
			}
			if(unlikely(requested_irq_nr >= pdata->fdev->nof_irqs)) {
				printk(KERN_WARNING "[%s] IRQ number %lu is too high. Number must be between 0 and %lu", MODULE_NAME, (long unsigned int)requested_irq_nr, (long unsigned int)pdata->fdev->nof_irqs-1);
				return -EINVAL;
			}
			// generate IRQ structure and link to the correct entry in the list, which is sorted by PID
			list_for_each_entry(hwirq, &(pdata->fdev->hw_irq_data), list) {
    			if (hwirq->irq_nr == requested_irq_nr) {
					mutex_lock(&(hwirq->lock_for_ioctl)); // It's not allowed for two processes to read and write the list at the same time.
        			list_for_each_entry(fsignal, &(hwirq->flink_process_data), list) {
						if(unlikely(fsignal->user_task->pid == user_task->pid)) {
							printk(KERN_WARNING "[%s] IRQ %lu is already registered oh the pid", MODULE_NAME, (long unsigned int)hwirq->irq_nr);
							mutex_unlock(&(hwirq->lock_for_ioctl));
							return -EINVAL;
						}
					}
					fsignal = kzalloc(sizeof(struct flink_process_data), GFP_KERNEL);
					if(unlikely(!fsignal)) {
						printk(KERN_ERR "[%s] Failed to allocate memory for signal witch depends on irq %lu", MODULE_NAME, (long unsigned int)hwirq->irq_nr);
						mutex_unlock(&(hwirq->lock_for_ioctl));
						return -ENOMEM;
					}
					INIT_LIST_HEAD(&(fsignal->list));
					fsignal->user_task = user_task;
					hwirq->signal_nr_with_offset = pdata->fdev->signal_offset + hwirq->irq_nr;
					
					spin_lock_bh(&(hwirq->irq_lock));
					list_add(&(fsignal->list), &(hwirq->flink_process_data)); // This is very critical when an IRQ is fired (IRQ reads this list and here it has been modified).
					spin_unlock_bh(&(hwirq->irq_lock));
					mutex_unlock(&(hwirq->lock_for_ioctl));

					hwirq->signal_count++;
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Signal %lu for process %lu registerd", hwirq->signal_nr_with_offset, user_task->pid);
					#endif
        			return hwirq->signal_nr_with_offset;
				}
			}
			break;
		case UNREGISTER_IRQ:
			#if defined(DBG)
				printk(KERN_DEBUG "[%s] Unregister IRQ (0x%x)", MODULE_NAME, UNREGISTER_IRQ);
			#endif
			if(unlikely(pdata->fdev->nof_irqs == 0)) {
				printk(KERN_WARNING "[%s] Irq functionality not available", MODULE_NAME);
				return -EPERM;
			}
			error = copy_from_user(&rw_container, (void __user *)arg, sizeof(rw_container));
			if(unlikely(error != 0)) {
				printk(KERN_WARNING "[%s] Error while copying from userspace: %i", MODULE_NAME, error);
				return -EINVAL;
			}
			if(unlikely(rw_container.size != 4)) {
				printk(KERN_WARNING "[%s] size must have lenght of 4 bytes aka uint32_t", MODULE_NAME);
				return -EINVAL;
			}
			wsize = copy_from_user(&requested_irq_nr, (void __user *)rw_container.data, sizeof(requested_irq_nr));
			if(unlikely(wsize > 0)) {
				printk(KERN_WARNING "[%s] Copying from user space failed: %lu bytes not copied!", MODULE_NAME, wsize);
				return -EINVAL;
			}
			if(unlikely(requested_irq_nr >= pdata->fdev->nof_irqs)) {
				printk(KERN_WARNING "[%s] IRQ number %lu is too high. Number must be between 0 and %lu", MODULE_NAME, (long unsigned int)requested_irq_nr, (long unsigned int)pdata->fdev->nof_irqs-1);
				return -EINVAL;
			}
			list_for_each_entry(hwirq, &(pdata->fdev->hw_irq_data), list) {
    			if(hwirq->irq_nr == requested_irq_nr) {
					if(unlikely(hwirq->signal_count == 0)){
						printk(KERN_WARNING "[%s] No signal registered on the requested IRQ: %lu", MODULE_NAME, (long unsigned int)hwirq->irq_nr);
						return -EINVAL;
					}
					mutex_lock(&(hwirq->lock_for_ioctl)); // It's not allowed for two processes to read and write the list at the same time.
					found_entry = false;
        			list_for_each_entry(fsignal, &(hwirq->flink_process_data), list) {
						if(fsignal->user_task->pid == user_task->pid) {
							#if defined(DBG)
								printk(KERN_DEBUG "  -> Found list entry to remove");
							#endif
							found_entry = true;
							break;
						}
					}
					if(likely(found_entry)) {
						spin_lock_bh(&(hwirq->irq_lock));
						list_del(&(fsignal->list)); // This is very critical when an IRQ is fired (IRQ reads this list and here it has been modified).
						spin_unlock_bh(&(hwirq->irq_lock));
						mutex_unlock(&(hwirq->lock_for_ioctl));

						if(fsignal) {
							kfree(fsignal);
						}
						#if defined(DBG)
							printk(KERN_DEBUG "  -> Signal %lu for process %lu unregisterd", hwirq->signal_nr_with_offset, user_task->pid);
						#endif
					} else {
						mutex_unlock(&(hwirq->lock_for_ioctl));
						#if defined(DBG)
							printk(KERN_DEBUG "  -> No list entry found to remove");
						#endif
						return -EINVAL;
					}
        			break;
				}
			}
			break;
		case GET_SIGNAL_OFFSET:
			#if defined(DBG)
				printk(KERN_DEBUG "[%s] Get signal offset (0x%x)", MODULE_NAME, GET_SIGNAL_OFFSET);
			#endif
			if(unlikely(pdata->fdev->nof_irqs == 0)) {
				printk(KERN_WARNING "[%s] Irq functionality not available", MODULE_NAME);
				return -EPERM;
			}
			error = copy_from_user(&rw_container, (void __user *)arg, sizeof(rw_container));
			if(unlikely(error != 0)) {
				printk(KERN_WARNING "[%s] Error while copying from userspace: %i", MODULE_NAME, error);
				return -EINVAL;
			}
			if(unlikely(rw_container.size != 4)) {
				printk(KERN_WARNING "[%s] Size must have lenght of 4 bytes aka uint32_t", MODULE_NAME);
				return -EINVAL;
			}
			rsize = copy_to_user((void __user *)rw_container.data, &(pdata->fdev->signal_offset), sizeof(pdata->fdev->signal_offset));
			if(unlikely(rsize > 0)) {
				printk(KERN_WARNING "[%s] Copying to user space failed: %lu bytes not copied!", MODULE_NAME, rsize);
				return 0;
			}
			#if defined(DBG)
				printk(KERN_DEBUG "  -> Signal offset:  0x%x", pdata->fdev->signal_offset);
			#endif
			return sizeof(pdata->fdev->signal_offset);
		default:
			#if defined(DBG)
				printk(KERN_DEBUG "  -> Error: illegal ioctl command: 0x%x!", cmd);
			#endif
			return -EINVAL;
    }
	return 0;
}

loff_t flink_llseek(struct file* f, loff_t off, int whence) {
	struct flink_private_data* pdata = (struct flink_private_data*)(f->private_data);
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] llseek call...", MODULE_NAME);
	#endif
	if(pdata != NULL && pdata->current_subdevice != NULL) {
		loff_t newpos;
		switch(whence) {
			case 0: /* SEEK_SET */
				newpos = off;
				break;
			case 1: /* SEEK_CUR */
				newpos = f->f_pos + off;
				break;
			case 2: /* SEEK_END */
				newpos = pdata->current_subdevice->mem_size + off;
				break;
			default: /* can't happen */
				return -EINVAL;
		}
		if(newpos < 0) return -EINVAL;
		f->f_pos = newpos;
		#if defined(DBG)
			printk(KERN_DEBUG "  -> new position: 0x%x", (u32)newpos);
		#endif
		return newpos;
	}
	return -EINVAL;
}

struct file_operations flink_fops = {
	.owner          = THIS_MODULE,
	.open           = flink_open,
	.release        = flink_relase,
	.read           = flink_read,
	.write          = flink_write,
	.unlocked_ioctl = flink_ioctl,
	.llseek         = flink_llseek
};

// ############ Initialization ############
static int __init flink_init(void) {
	int error = 0;
	
	// Create sysfs class
	sysfs_class = CLASS_CREATE(SYSFS_CLASS_NAME);
	
	// ---- All done ----
	printk(KERN_INFO "[%s] Module sucessfully loaded\n", MODULE_NAME);

	return 0;

	// ---- ERROR HANDLING ----
	
	return error;
}
module_init(flink_init);

// ############ Cleanup ############
static void __exit flink_exit(void) {
	// Destroy sysfs class
	class_destroy(sysfs_class);
	
	// ---- All done ----
	printk(KERN_INFO "[%s] Module sucessfully unloaded\n", MODULE_NAME);
}
module_exit(flink_exit);

// ############ Device and module handling functions ############

/*******************************************************************
 *                                                                 *
 *  Internal (private) methods                                     *
 *                                                                 *
 *******************************************************************/

/**
 * create_device_node() - creates a device node for a flink device
 * @fdev: the flink device to create a device node for
 */
static int create_device_node(struct flink_device* fdev) {
	static unsigned int dev_counter = 0;
	int error = 0;
	dev_t dev;
	
	// Allocate, register and initialize char device
	error = alloc_chrdev_region(&dev, dev_counter, 1, MODULE_NAME);
	if(error) {
		printk(KERN_ERR "[%s] Allocation of char dev region failed!", MODULE_NAME);
		goto alloc_chardev_region_failed;
	}
	fdev->char_device = cdev_alloc();
	if(fdev->char_device == NULL) {
		printk(KERN_ERR "[%s] Allocation of char dev failed!", MODULE_NAME);
		goto cdev_alloc_failed;
	}
	cdev_init(fdev->char_device, &flink_fops);
	fdev->char_device->owner = THIS_MODULE;
	error = cdev_add(fdev->char_device, dev, 1);
	if(error) {
		printk(KERN_ERR "[%s] Adding the char dev to the system failed!", MODULE_NAME);
		goto cdev_add_failed;
	}
	
	// create device node
	fdev->sysfs_device = device_create(sysfs_class, NULL, dev, NULL, "flink%u", dev_counter);
	if(IS_ERR(fdev->sysfs_device)) {
		printk(KERN_ERR "[%s] Creation of sysfs device failed!", MODULE_NAME);
		goto device_create_failed;
	}
	
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] Device node created: flink%u", MODULE_NAME, dev_counter);
	#endif
	
	dev_counter++;
	
	return 0;
	
	// Cleanup on error
device_create_failed:
	cdev_del(fdev->char_device);
		
cdev_add_failed:
	fdev->char_device = NULL;

cdev_alloc_failed:
	unregister_chrdev_region(dev, 1);
		
alloc_chardev_region_failed:
	// nothing to do
		
	return error;
}

/**
 * scan_for_subdevices() - scan flink device for subdevices
 * @fdev: the flink device to scan
 *
 * Scans the device for available subdevices and adds them to
 * the device structure. The number of added subdevices is returned.
 */
static unsigned int scan_for_subdevices(struct flink_device* fdev) {
	unsigned int subdevice_counter = 0;
	u32 current_address = 0;
	u32 last_address = current_address + fdev->bus_ops->address_space_size(fdev) - 1;
	u32 current_function = 0;
	u32 current_mem_size = 0;
	u32 total_mem_size = 0;
	struct flink_subdevice* new_subdev;
	
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] Scanning device #%u for subdevices...", MODULE_NAME, fdev->id);
		printk(KERN_DEBUG "  -> Start address:      0x%x", current_address);
		printk(KERN_DEBUG "  -> Last valid address: 0x%x", last_address);
	#endif
	while(current_address < last_address && subdevice_counter < MAX_NOF_SUBDEVICES) {
		current_function = (fdev->bus_ops->read32(fdev, current_address + SUBDEV_FUNCTION_OFFSET));
		current_mem_size = fdev->bus_ops->read32(fdev, current_address + SUBDEV_SIZE_OFFSET);

		#if defined(DBG)
			printk(KERN_DEBUG "[%s] subdevice size: 0x%x (current address: 0x%x)\n", MODULE_NAME, current_mem_size, current_address);
		#endif

		if(current_mem_size > MAIN_HEADER_SIZE + SUB_HEADER_SIZE) {
			// Create and initialize new subdevice
			new_subdev = flink_subdevice_alloc();
			flink_subdevice_init(new_subdev);
			new_subdev->function_id = (u16)(current_function >> 16);
			new_subdev->sub_function_id = (u8)((current_function >> 8) & 0xFF);
			new_subdev->function_version = (u8)(current_function & 0xFF);
			new_subdev->base_addr = current_address;
			new_subdev->mem_size = current_mem_size;
			new_subdev->nof_channels = fdev->bus_ops->read32(fdev, current_address + SUBDEV_NOFCHANNELS_OFFSET);
			new_subdev->unique_id = fdev->bus_ops->read32(fdev, current_address + SUBDEV_UNIQUE_ID_OFFSET);
			
			// Add subdevice to flink device
			flink_subdevice_add(fdev, new_subdev);
			subdevice_counter++;
			
			// if subdevice is info subdevice -> read memory length
			if(new_subdev->function_id == INFO_FUNCTION_ID) {
				total_mem_size = fdev->bus_ops->read32(fdev, current_address + MAIN_HEADER_SIZE + SUB_HEADER_SIZE);
				last_address = total_mem_size - 1;
				#if defined(DBG)
					printk(KERN_DEBUG "[%s] Info subdevice found: total memory length=0x%x", MODULE_NAME, total_mem_size);
				#endif
			}

			// Increment address counter and reset temp variables
			current_address += current_mem_size;
			current_function = 0;
			current_mem_size = 0;
		}
		else {
			#if defined(DBG)
				printk(KERN_ALERT "[%s] aborting\n", MODULE_NAME);
			#endif
			break;
		}
	}
	return subdevice_counter;
}

// irq handler do not call this function directly. Only register it with request_irq()
static irqreturn_t flink_threaded_irq_handler(int irq, void *dev_id) {
    struct siginfo info;
	struct flink_irq_data* irq_data = (struct flink_irq_data*)(dev_id);
	struct flink_process_data* signal_data;

	#if defined(DBG_IRQ)
		printk(KERN_DEBUG "[%s] IRQ nr: %lu rised", MODULE_NAME, irq);
	#endif

	if (unlikely(irq != irq_data->irq_nr_with_offset)) {
		#if defined(DBG_IRQ)
			printk(KERN_DEBUG "  -> IRQ nr: %lu calld the wrong handler (Handler irq nr: %lu)", irq, irq_data->irq_nr_with_offset);
		#endif
		return IRQ_NONE;
	}

	// prepare siginfo to save time
	memset(&info, 0, sizeof(info));
	info.si_code = SI_QUEUE;
	info.si_signo = irq_data->signal_nr_with_offset;
	
	// critical section
	{
		spin_lock_bh(&(irq_data->irq_lock));
		list_for_each_entry(signal_data, &(irq_data->flink_process_data), list) {
			if(signal_data->user_task != NULL) {
				/* Send the signal */
				#if defined(DBG_IRQ) 
					int err = send_sig_info(irq_data->signal_nr_with_offset, (struct kernel_siginfo *) &info, signal_data->user_task);
					if(err < 0) {
						printk(KERN_WARNING "  -> Error while sending signal: %lu to userspace pid: %lu. Error nr: %lu", MODULE_NAME, irq_data->signal_nr_with_offset, signal_data->user_task->pid, err);
					} else {
						printk(KERN_DEBUG "  -> Successfully send signal: %lu to userspace pid: %lu", MODULE_NAME, irq_data->signal_nr_with_offset, signal_data->user_task->pid);
					}
				#else
					send_sig_info(irq_data->signal_nr_with_offset, (struct kernel_siginfo *) &info, signal_data->user_task);
				#endif
			}
		}
		spin_unlock_bh(&(irq_data->irq_lock));
	}
	return IRQ_HANDLED;
}

/* load a child driver to connect flink with linux subsystems*/
static int flinkcore_add_gpio_child(struct device *parent,
                                    resource_size_t base, 
																		resource_size_t size)
{
	struct resource res[] = {
		{
			.start = base,
			.end   = base + size - 1,
			.flags = IORESOURCE_MEM,
		},
	};

//	/* optionale Properties fürs Child (der Hello-World-Treiber ignoriert sie): */
//	static const struct property_entry props[] = {
//		{ PROPERTY_ENTRY_U32("ngpios", 32)},
//		{},
//	};

	struct mfd_cell cell = {
		.name          = "flink-bridge-gpio",     /* matcht .driver.name */
		.of_compatible = "ost,flink-bridge-gpio", /* matcht OF-Tabelle */
		.resources     = res,
		.num_resources = ARRAY_SIZE(res),
//		.properties    = props,
	};

    /* Parent-IRQ als 6. Parameter nur setzen, wenn sinnvoll; hier 0 */
	return mfd_add_devices(parent, PLATFORM_DEVID_AUTO,
	                       &cell, 1, NULL, 0, NULL);
}

/*******************************************************************
 *                                                                 *
 *  Public methods                                                 *
 *                                                                 *
 *******************************************************************/

/**
 * @brief Allocate a flink_device structure.
 * @return flink_device*: Pointer to the new flink_device structure, or NULL on failure.
 */
struct flink_device* flink_device_alloc(void) {
	struct flink_device* fdev = kmalloc(sizeof(struct flink_device), GFP_KERNEL);
	if(fdev) {
		INIT_LIST_HEAD(&(fdev->list));
	}
	return fdev;
}

/**
 * @brief Initialize a flink_device structure
 * @param fdev: The structure to initialize
 * @param bus_ops: The flink_bus_ops for this device, remember them when adding them to
 * system with flink_device_add().
 * @param mod: The kernel module this flink uses for hardware access.
 * @deprecated This function is outdated and should no longer be used.
 *             Use flink_device_init_irq() instead.
 *             Set nof_irq = irq_offset = signal_offset = 0
 */
void flink_device_init(struct flink_device* fdev, struct flink_bus_ops* bus_ops, struct module* mod) {
	flink_device_init_irq(fdev, bus_ops, mod, 0, 0, 0);
}

/**
 * @brief Initialize a flink_device structure
 * @param fdev: The structure to initialize
 * @param bus_ops: The flink_bus_ops for this device, remember them when adding them to
 * system with flink_device_add().
 * @param irq_ops: The flink_irq_ops for this device. This is needed when an irq gets (un)registert from userspace. Set NULL if irq not required
 * @param mod: The kernel module this flink uses for hardware access.
 * @param nof_irq: Number of irqs that are provided. Set 0 if irq not required
 * @param irq_offset: The offset of the first irq number. Set 0 if irq not required
 * @param signal_offset: The offset of the first signal that is sent to userspace. Set 0 if irq not required
 */
void flink_device_init_irq(struct flink_device* fdev, 
						   struct flink_bus_ops* bus_ops, 
						   struct module* mod, 
						   u32 nof_irq, 
						   u32 irq_offset, 
						   u32 signal_offset) {
	struct flink_irq_data* irq_data;
	int err = 0;
	
	memset(fdev, 0, sizeof(*fdev));
	INIT_LIST_HEAD(&(fdev->list));
	INIT_LIST_HEAD(&(fdev->subdevices));
	fdev->bus_ops = bus_ops;
	fdev->appropriated_module = mod;
	
	fdev->irq_offset = irq_offset;
	fdev->signal_offset = signal_offset;
	fdev->nof_irqs = nof_irq;
	INIT_LIST_HEAD(&(fdev->hw_irq_data)); // First dimension static: Second dimension dynamic

	// Create the first dimension of the two-dimensional data structure "flink_irq_sorted_by_irq"
	if(nof_irq > 0){
		for(int i = 0; i < nof_irq; i++){
			irq_data = kzalloc(sizeof(struct flink_irq_data), GFP_KERNEL);
			if(unlikely(!irq_data)) {
				printk(KERN_ERR "[%s] Failed to allocate memory for hw irq: %lu", MODULE_NAME, (long unsigned int)irq_offset + i);
				printk(KERN_ERR "  -> Disabled IRQ functionality!!!");
				nof_irq = 0;
				return;
			}
			INIT_LIST_HEAD(&(irq_data->list));
			INIT_LIST_HEAD(&(irq_data->flink_process_data));
			irq_data->irq_nr = i;
			irq_data->signal_count = 0;
			irq_data->irq_nr_with_offset = irq_offset + i;
			spin_lock_init(&(irq_data->irq_lock));
			mutex_init(&(irq_data->lock_for_ioctl));
			list_add(&(irq_data->list), &(fdev->hw_irq_data));

			// register a threaded irq handler otherwise is occours a problem with the using spinlock
			err = request_threaded_irq(irq_data->irq_nr_with_offset, NULL, flink_threaded_irq_handler, IRQF_ONESHOT, "flink IRQ Handler", (void*)(irq_data));
			if (unlikely(err < 0)) {
				printk(KERN_ERR "[%s] Unabel to register IRQ %lu. Error nr: %d", MODULE_NAME, (long unsigned int)irq_data->irq_nr_with_offset, err);
				printk(KERN_ERR "  -> Disabled IRQ functionality!!!");
				nof_irq = 0;
				return;
			}
		}
	} else {
		#ifdef DBG
			printk(KERN_DEBUG "[%s] Disabled IRQ functionality!!!", MODULE_NAME);
		#endif
	}
}

/**
 * @brief Add a flink device to the system, making it live immediately.
 * @param fdev: The flink device to add. 
 * @return int: A negative error code is returned on failure.
 */
int flink_device_add(struct flink_device* fdev) {
	static unsigned int dev_counter = 0;
	unsigned int nof_subdevices = 0;
	if(fdev != NULL) {
		// Add device to list
		fdev->id = dev_counter++;
		list_add(&(fdev->list), &device_list);
		#if defined(DBG)
			printk(KERN_DEBUG "[%s] Device with id '%u' added to device list.", MODULE_NAME, fdev->id);
		#endif
		
		// Scan for subdevices
		nof_subdevices = scan_for_subdevices(fdev);
		#if defined(DBG)
			printk(KERN_DEBUG "[%s] %u subdevice(s) added to device #%u", MODULE_NAME, nof_subdevices, fdev->id);
		#endif
		
		// Create device node
		create_device_node(fdev);

		// connect subdevices to subsystems fomr kernel
		resource_size_t gpio_phys_base = 0x7aa00000 + 0x1000;
		resource_size_t gpio_size      = 0x100;
		flinkcore_add_gpio_child(fdev->sysfs_device, gpio_phys_base, gpio_size);
		
		return fdev->id;
	}
	return UNKOWN_ERROR;
}

/**
 * @brief Remove a flink device from the system.
 * @param fdev: The flink device to remove. 
 * @return int: A negative error code is returned on failure.
 */
int flink_device_remove(struct flink_device* fdev) {
	if(fdev != NULL) {
		
		// Remove device from list
		list_del(&(fdev->list));
		#if defined(DBG)
			printk(KERN_DEBUG "[%s] Device with id '%u' removed frome device list.", MODULE_NAME, fdev->id);
		#endif
		
		// Destroy device node and free char dev region
		device_destroy(sysfs_class, fdev->char_device->dev);
		cdev_del(fdev->char_device);
		unregister_chrdev_region(fdev->char_device->dev, 1);

		// Remove child brigdes to subsystems
		mfd_remove_devices(fdev->sysfs_device);
		
		return 0;
	}
	return UNKOWN_ERROR;
}

/**
 * @brief Deletes a flink device and frees the allocated memory. All subdevices will be deleted,
 * using flink_subdevice_remove() and flink_subdevice_delete() as well as the whole irq structure.
 * @param fdev: The flink_device structure to delete. 
 * @return int: A negative error code is returned on failure.
 */
int flink_device_delete(struct flink_device* fdev) {
	if(fdev != NULL) {
		struct flink_subdevice* sdev;
		struct flink_subdevice* sdev_next;
		struct flink_irq_data* irq_data, *irq_data_next;
		struct flink_process_data* signal_data, *signal_data_next;
		
		// Remove and delete all subdevices
		list_for_each_entry_safe(sdev, sdev_next, &(fdev->subdevices), list) {
			#if defined(DBG)
				printk(KERN_DEBUG "[%s] Removing and deleting subdevice #%u (from device #%u)", MODULE_NAME, sdev->id, fdev->id);
			#endif
			flink_subdevice_remove(sdev);
			flink_subdevice_delete(sdev);
		}

		// unregister irq and delete the irq related data
		if(fdev->nof_irqs > 0) {
			// first, unregister all IRQs to avoid using the spinlock and to avoid a null pointer error if an IRQ is fired.
			list_for_each_entry_safe(irq_data, irq_data_next, &(fdev->hw_irq_data), list) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Removing and deleting irq structure #%u (from device #%u)", irq_data->irq_nr, fdev->id);
				#endif
				free_irq(irq_data->irq_nr_with_offset, (void*)(irq_data));
			}

			// remove and delete irq structure with the nested signal structure
			list_for_each_entry_safe(irq_data, irq_data_next, &(fdev->hw_irq_data), list) {
				#if defined(DBG)
					printk(KERN_DEBUG "  -> Removing and deleting irq structure #%u (from device #%u)", irq_data->irq_nr, fdev->id);
				#endif
				list_for_each_entry_safe(signal_data, signal_data_next, &(irq_data->flink_process_data), list) {
					#if defined(DBG)
						printk(KERN_DEBUG "  -> Removing and deleting signal structure #%u (from device #%u)", irq_data->signal_nr_with_offset, fdev->id);
					#endif
					list_del(&(signal_data->list));
					if(signal_data) kfree(signal_data);
				}
				list_del(&(irq_data->list));
				if(irq_data) kfree(irq_data);
			}
		}

		// Free memory
		kfree(fdev);
		
		return 0;
	}
	return UNKOWN_ERROR;
}

/**
 * @brief Get a flink device by its id.
 * @param id: The id of the flink device. 
 * @return flink_device*: Returns the flink device structure with the given id. 
 * NULL is returned if no device is found with the given id.
 */
struct flink_device* flink_get_device_by_id(u8 id) {
	struct flink_device* fdev;
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] Looking for device with id '%u'...", MODULE_NAME, id);
	#endif
	list_for_each_entry(fdev, &device_list, list) {
		if(fdev->id == id) {
			#if defined(DBG)
				printk(KERN_DEBUG "[%s] Device with id '%u' found!", MODULE_NAME, id);
			#endif
			return fdev;
		}
	}
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] No device with id '%u' found!", MODULE_NAME, id);
	#endif
	return NULL;
}

/**
 * @brief Get a flink device by cdev.
 * @param char_device: Pointer to the cdev struct of a flink device. 
 * @return flink_device*: Returns the flink device structure associated with the given cdev. 
 * NULL is returned if no suitable device is found.
 */
struct flink_device* flink_get_device_by_cdev(struct cdev* char_device) {
	struct flink_device* fdev;
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] Looking for device with cdev '%p'...", MODULE_NAME, char_device);
	#endif
	list_for_each_entry(fdev, &device_list, list) {
		if(fdev->char_device == char_device) {
			#if defined(DBG)
				printk(KERN_DEBUG "[%s] Device with cdev '%p' found!", MODULE_NAME, char_device);
			#endif
			return fdev;
		}
	}
	#if defined(DBG)
		printk(KERN_DEBUG "[%s] No device with cdev '%p' found!", MODULE_NAME, char_device);
	#endif
	return NULL;
}

/**
 * @brief Get a list with all devices.
 * @return list_head*: Returns a pointer to a list containing
 * all flink devices. The list could be empty.
 */
struct list_head* flink_get_device_list() {
	return &device_list;
}

/**
 * @brief Allocate a flink_subdevice structure.
 * @return flink_subdevice*: Pointer to the new flink_subdevice structure, or NULL on failure.
 */
struct flink_subdevice* flink_subdevice_alloc(void) {
	struct flink_subdevice* fsubdev = kmalloc(sizeof(struct flink_subdevice), GFP_KERNEL);
	if(fsubdev) {
		INIT_LIST_HEAD(&(fsubdev->list));
	}
	return fsubdev;
}

/**
 * @brief Initialize a flink_subdevice structure, making it ready to add to a flink_device
 * with flink_subdevice_add()
 * @param fsubdev: The structure to initialize
 */
void flink_subdevice_init(struct flink_subdevice* fsubdev) {
	memset(fsubdev, 0, sizeof(*fsubdev));
	INIT_LIST_HEAD(&(fsubdev->list));
}

/**
 * @brief Add a flink subdevice to flink device.
 * @param fdev: The flink device to which the subdevice is added. 
 * @param @fsubdev: The flink subdevice to add. 
 * @return int: A negative error code is returned on failure.
 */
int flink_subdevice_add(struct flink_device* fdev, struct flink_subdevice* fsubdev) {
	if(fdev != NULL && fsubdev != NULL) {
		// Define subdevice id
		fsubdev->id = fdev->nof_subdevices++;
		
		// Set parent pointer
		fsubdev->parent = fdev;
		
		// Add subdevice to device
		list_add(&(fsubdev->list), &(fdev->subdevices));
		#if defined(DBG)
			printk(KERN_DEBUG "[%s] Subdevice with id '%u' added to device with id '%u'.", MODULE_NAME, fsubdev->id, fdev->id);
			printk(KERN_DEBUG "  -> Function:         0x%x/0x%x/0x%x", fsubdev->function_id, fsubdev->sub_function_id, fsubdev->function_version);
			printk(KERN_DEBUG "  -> Base address: 0x%x", fsubdev->base_addr);
			printk(KERN_DEBUG "  -> Size:         0x%x (%u bytes)", fsubdev->mem_size, fsubdev->mem_size);
			printk(KERN_DEBUG "  -> Nof Channels: %u", fsubdev->nof_channels);
			printk(KERN_DEBUG "  -> Unique id: 0x%x", fsubdev->unique_id);
		#endif
		
		// Register subdevice at interface module
		//if(fsubdev->register_hook != NULL) fsubdev->register_hook();
			
		return fsubdev->id;
	}
	return UNKOWN_ERROR;
}

/**
 * @brief Remove a flink subdevice from the parent device.
 * @param fsubdev: The flink subdevice to remove. 
 * @return int: A negative error code is returned on failure.
 */
int flink_subdevice_remove(struct flink_subdevice* fsubdev) {
	if(fsubdev != NULL) {
		
		// Remove device from list
		list_del(&(fsubdev->list));
		#if defined(DBG)
			printk(KERN_DEBUG "[%s] Subdevice with id '%u' removed from list.", MODULE_NAME, fsubdev->id);
		#endif
		
		// Delete parent pointer
		fsubdev->parent = NULL;
			
		return 0;
	}
	return UNKOWN_ERROR;
}

/**
 * @brief Deletes a flink subdevice and frees the allocated memory.
 * @param fsubdev: The flink_subdevice structure to delete. 
 * @return int: A negative error code is returned on failure.
 */
int flink_subdevice_delete(struct flink_subdevice* fsubdev) {
	if(fsubdev != NULL) {
		
		// Free memory
		kfree(fsubdev);
		
		return 0;
	}
	return UNKOWN_ERROR;
}

/**
 * @brief Get a flink subdevice by its id.
 * @param fdev: The flink device containing the desired flink_subdevice. 
 * @param id: The id of the flink device. 
 * @return flink_subdevice*: Returns the flink_subdevice structure with the given id. 
 * NULL is returned if no subdevice is found with the given id.
 */
struct flink_subdevice* flink_get_subdevice_by_id(struct flink_device* fdev, u8 id) {
	if(fdev != NULL) {
		struct flink_subdevice* subdev;
		#if defined(DBG)
			printk(KERN_DEBUG "[%s] Looking for subdevice with id '%u' in device %u...", MODULE_NAME, id, fdev->id);
		#endif
		list_for_each_entry(subdev, &(fdev->subdevices), list) {
			if(subdev->id == id) {
				#if defined(DBG)
					printk(KERN_DEBUG "[%s] Subdevice with id '%u' found!", MODULE_NAME, id);
				#endif
				return subdev;
			}
		}
		#if defined(DBG)
			printk(KERN_DEBUG "[%s] No subdevice with id '%u' found!", MODULE_NAME, id);
		#endif
	}
	return NULL;
}

/**
 * @brief Get a flink sysfs class.
 * @return class*: Pointer to the flink sysfs class structure.
 */
struct class* flink_get_sysfs_class(void) {
	return sysfs_class;
}

/**
 * @brief Select a subdevice for exclusive access.
 * @param f:  
 * @param subdevice:  
 * @param excl:  
 * @return int: A negative error code is returned on failure.
 */
int flink_select_subdevice(struct file* f, u8 subdevice, bool excl) {
	struct flink_private_data* pdata = (struct flink_private_data*)(f->private_data);
	if(pdata != NULL && pdata->fdev != NULL) {
		struct flink_device* fdev = pdata->fdev;
		pdata->current_subdevice = flink_get_subdevice_by_id(fdev, subdevice);
		// TODO exclusive access
		#if defined(DBG)
			printk(KERN_DEBUG "[%s] Selecting subdevice %u", MODULE_NAME, subdevice);
			if(excl) printk(KERN_DEBUG "  -> exclusive");
			else printk(KERN_DEBUG "  -> not exclusive");
		#endif
		return 0;
	}
	return UNKOWN_ERROR;
}

// ############ Let other modules do flink stuff ############
EXPORT_SYMBOL(flink_device_alloc);
EXPORT_SYMBOL(flink_device_init);
EXPORT_SYMBOL(flink_device_init_irq);
EXPORT_SYMBOL(flink_device_add);
EXPORT_SYMBOL(flink_device_remove);
EXPORT_SYMBOL(flink_device_delete);
EXPORT_SYMBOL(flink_get_device_by_id);
EXPORT_SYMBOL(flink_get_device_list);
EXPORT_SYMBOL(flink_subdevice_alloc);
EXPORT_SYMBOL(flink_subdevice_init);
EXPORT_SYMBOL(flink_subdevice_add);
EXPORT_SYMBOL(flink_subdevice_remove);
EXPORT_SYMBOL(flink_subdevice_delete);
EXPORT_SYMBOL(flink_get_subdevice_by_id);
EXPORT_SYMBOL(flink_select_subdevice);
EXPORT_SYMBOL(flink_get_sysfs_class);
