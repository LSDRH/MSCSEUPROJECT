#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i8253.h>
#include <linux/kernel.h>

MODULE_LICENSE("GPL");

#define  DEVICE_NAME "spkr"
#define ENTRY_NAME "intspkr"
#define CLASS_NAME "speaker"
#define PIT_CHANNEL_2_GATE_PORT 0x61  //To control the input signal gets or not to channel 2 of the pit.
#define PIT_CHANNEL_2_DATA_PORT 0x42  //To access timer 2.
#define PIT_OPERATION_MODE_PORT 0x43  //To specify the operation: reads are ignored.

DEFINE_RAW_SPINLOCK(i8253_lock);

static dev_t midispo;
static struct cdev mydevice;
static struct class* spkrClass = NULL;
static struct device* spkrDevice = NULL;
static int mj;

static int ejemplo_open(struct inode *inode, struct file *filp) {
	printk(KERN_INFO "ejemplo_open\n");
	return 0;
}

static int ejemplo_release(struct inode *inode, struct file *filp) {
	printk(KERN_INFO "ejemplo_release\n");
	return 0;
}
static ssize_t ejemplo_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	printk(KERN_INFO "ejemplo_write\n");
	size_t ret = count;
	return ret;
}

static struct file_operations ejemplo_fops = {
        .owner =    THIS_MODULE,
        .open =     ejemplo_open,
        .release =  ejemplo_release,
        .write =    ejemplo_write
};

/*Manipulate ports 0x42 and 0x43 to set the desired frequency that will be fed to the speaker*/
void spkr_set_frequency(unsigned int frequency) {
	uint32_t div;
	unsigned long flags;

	div = PIT_TICK_RATE / frequency;

	outb(0xb6, 0x43); 						//select timer 2 in mode 3

	raw_spin_lock_irqsave(&i8253_lock, flags);
	outb((uint8_t) (div), 0x42); 			//write the first 0-7 bits of the frequency divider.
	outb((uint8_t) (div >> 8), 0x42); 		//write the last bits 8-15 of the frequency divider.
	raw_spin_unlock_irqrestore(&i8253_lock, flags);

	printk(KERN_INFO "spkr set frequency: %d\n", frequency);
}

/*Connect the system timer to the gate of timer 2 and allow the wave reach the speaker, using port 0x61*/
void spkr_on(void) {
	uint8_t tmp;

	tmp = inb(0x61);

  	if (tmp != (tmp | 3)) {
		uint8_t var = (tmp | 3);
 		outb(var, 0x61);
 	}

	printk(KERN_INFO "spkr ON\n");
}

/*Deactivate the speaker manipulating port 0x61*/
void spkr_off(void) {
	uint8_t tmp1 = inb(0x61);

	uint8_t tmp2 = (tmp1 & 0xFC);

 	outb(tmp2, 0x61);

	printk(KERN_INFO "spkr OFF\n");
}

static int reserve_mjAndmn(void) {
	//reserve major and minor numbers
	int registerMajor = alloc_chrdev_region(&midispo, 0, 1, DEVICE_NAME);

	if(registerMajor < 0) {
		printk(KERN_ALERT "spkr failed to register a major number\n");
    	return registerMajor;
	}
	mj = MAJOR(midispo);
	printk(KERN_INFO "major number registered. Assigned major number is: %d\n", mj);
	return 0;
}

static int register_and_load_class(void) {
	//alta dispositivo en sysfs
	spkrClass = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(spkrClass)){
      unregister_chrdev_region(midispo, 1);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(spkrClass);
   	}
   	printk(KERN_INFO "spkr: device class registered correctly\n");

	spkrDevice = device_create(spkrClass, NULL, midispo, NULL, ENTRY_NAME);
	if (IS_ERR(spkrDevice)){             
      class_destroy(spkrClass);      
      unregister_chrdev_region(midispo, 1);
      printk(KERN_ALERT "Failed to create the device\n");
      return PTR_ERR(spkrDevice);
   	}
   	printk(KERN_INFO "spkr: device class created correctly\n");

	return 0;
}

static int register_device_in_kernel(void) {

	//iniciación y alta del dispositivo
	cdev_init(&mydevice, &ejemplo_fops);
	int addDevice = cdev_add(&mydevice, midispo, 1);
	if(addDevice < 0) {
		printk(KERN_INFO "spkr failed in adding the device\n");
		return(addDevice);
	}

	return 0;
}

int spkr_init(void) {
	printk(KERN_INFO "spkr init\n");

	//reserve major and minor numbers
	reserve_mjAndmn();

	//iniciación y alta del dispositivo
	register_device_in_kernel();

	//alta dispositivo en sysfs
	register_and_load_class();

	//play sound
	spkr_set_frequency(5000);
	spkr_on();

	return 0;
}

void spkr_exit(void) {
	printk(KERN_INFO "spkr exit\n");

	device_destroy(spkrClass, midispo);
	class_destroy(spkrClass);
	cdev_del(&mydevice);
	unregister_chrdev_region(midispo, 1);

	//bye sound
	spkr_off();
}

module_init(spkr_init);
module_exit(spkr_exit);