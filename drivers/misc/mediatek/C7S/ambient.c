#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/fb.h>

#define DEV_AMBIENT "ambientScreen"

static char *buffer = 0;
int ambient_mode = 0;

int DAL_Clean(void);
int DAL_Clock(char *layerVA);

static ssize_t ambient_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int enable = 0;

	sscanf(buf, "%d", &enable);
	printk("%s: %d\n", __func__, enable);

	if (enable) {
		ambient_mode = 1;
	} else {
		ambient_mode = 0;
	}

	return count;
}

static ssize_t ambient_enable_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "ambient_mode = %d\n", ambient_mode);
}

static ssize_t ambient_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int command = 0;

	sscanf(buf, "%d", &command);
	printk("%s: %d\n", __func__, command);

	switch (command) {
		case 0:
			DAL_Clean();
			break;

		case 6:
			DAL_Clock(buffer);
			break;

		default:
			break;
	}

	return count;
}

static ssize_t ambient_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return snprintf(buf, PAGE_SIZE, "not supported\n");
}

static DEVICE_ATTR(AM_Enable, 0644, ambient_enable_show, ambient_enable_store);
static DEVICE_ATTR(AM_Ctrl, 0644, ambient_ctrl_show, ambient_ctrl_store);

static struct attribute *ambient_attributes[] = {
	&dev_attr_AM_Enable.attr,
	&dev_attr_AM_Ctrl.attr,
	NULL,
};

static struct attribute_group ambient_attribute_group = {
	.attrs = ambient_attributes
};

static int ambient_mmap(struct file *filp, struct vm_area_struct *vma) {
	int page = virt_to_phys(buffer) >> PAGE_SHIFT;

	if (remap_pfn_range(vma, vma->vm_start, page, vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -1;
	}

	return 0;
}

static struct file_operations ambient_fops = {
	.owner = THIS_MODULE,
	.mmap = ambient_mmap,
};

static struct miscdevice ambient_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEV_AMBIENT,
	.fops = &ambient_fops,
};

static int __init ambient_init(void) {
	buffer = kmalloc(0x65000, GFP_KERNEL);

	if (misc_register(&ambient_device)) {
		printk("%s: misc_register failed!\n", __func__);
		return -1;
	}

	if (sysfs_create_group(&ambient_device.this_device->kobj, &ambient_attribute_group)) {
		printk("%s: sysfs_create_group failed!\n", __func__);
		return -1;
	}

	return 0;
}

static void __exit ambient_exit(void) {
 	sysfs_remove_group(&ambient_device.this_device->kobj, &ambient_attribute_group);
	misc_deregister(&ambient_device);
}

module_init(ambient_init);
module_exit(ambient_exit);
