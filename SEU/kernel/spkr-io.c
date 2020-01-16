#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/i8253.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/timer.h>

MODULE_LICENSE("GPL");

#define  DEVICE_NAME "spkr"
#define ENTRY_NAME "intspkr"
#define CLASS_NAME "speaker"
#define PIT_CHANNEL_2_GATE_PORT 0x61  //To control the input signal gets or not to channel 2 of the pit.
#define PIT_CHANNEL_2_DATA_PORT 0x42  //To access timer 2.
#define PIT_OPERATION_MODE_PORT 0x43  //To specify the operation: reads are ignored.
#define PAGE_SIZE 4096

DEFINE_RAW_SPINLOCK(i8253_lock);

//parameters to be received by user
static uint8_t buffer_size = PAGE_SIZE;
static uint8_t buffer_threshold = PAGE_SIZE;

//parameters needed for the write function
static size_t data_size;					//initially is equal to count. It indicates the remaining data in the user buffer.
static size_t bytes_to_write;				//indicates the number of bytes to write to the system kfifo.
static int device_is_active = 0; 


//parameters used to set up module
static dev_t midispo;
static struct cdev mydevice;
static struct class* spkrClass = NULL;
static struct device* spkrDevice = NULL;
static int mj;
atomic_t write_device_open = ATOMIC_INIT(0); /* Is the device open?  Used to prevent multiple access to the device */


//info_mydev
struct info_mydev {
	struct cdev mydev_cdev;
};

//KFIFO to copy buffer from user
struct kfifo fifo;

//KFIFO to handle complete and incomplete sounds
struct kfifo sound_fifo;

//TIMER
struct timer_list timer;


//SOUND: 1st 2 bytes -> frequency. 2nd 2 bytes -> sound duration.
struct my_sound {
	uint16_t frequency;
	uint16_t length;
} sound;


//receive parameters buffer_size and buffer_threshold from user.
module_param(buffer_size, int, S_IRUGO);
module_param(buffer_threshold, int, S_IRUGO);


/*PRODUCE_SOUND
-Checks if sound is silent -> if it is, sets device off and timer on.
-If not, sets device on, and also set frequency and timer.
*/
static void produce_sound() {
	if(sound.frequency == 0) {
		device_is_active = 0;

		//set timer
	}
	else {
		device_is_active = 1;
		spkr_set_frequency(sound.frequency);

		//set timer
	}
}

/* HANDLE_SOUND
tasks:
-Check if there is a complete sound in fifo -> it has 4 bytes at least.
-If fifo has a complete sound, call produce_sound().
-If fifo doesn't have a complete sound, fill sound_kfifo 's remaining space with the available bytes in fifo and then check if sound_fifo has a complete sound. If it does, call produce_sound(). If not, turn the device off.
-If there is a sound to be completed (sound_fifo is not empty) -> we have to complete and play that sound first, even if there are >= 4 bytes in fifo.
*/
static void handle_sound() {

	if((kfifo_len(&fifo) >= 4) & (kfifo_empty(&sound_fifo))) { //fifo contains at least a complete sound (4 bytes) and there isn't a sound to be completed in sound_fifo.
		kfifo_out(&fifo, &sound.frequency, 2);
		kfifo_out(&fifo, &sound.length, 2);

		produce_sound();
	}
	else if ((kfifo_len(&fifo) < 4) | (!kfifo_empty(&sound_fifo))) { //fifo doesn't contain a full sound, or there is a sound to be completed stored in sound_fifo.

		int num_elems_to_copy = min(kfifo_avail(&sound_fifo), kfifo_len(&fifo));
		
		kfifo_out(&fifo, &sound_fifo, num_elems_to_copy);

		if(kfifo_len(&sound_fifo) == 4) {	//we have a complete sound
			kfifo_out(&sound_fifo, &sound.frequency, 2);
			kfifo_out(&sound_fifo, &sound.length, 2);

			produce_sound();
		}
		else {
			device_is_active = 0;
		}

	}

}
static int device_open(struct inode *inode, struct file *filp) {

	printk(KERN_INFO "device_open\n");

	//if we want to open the file in writing mode and it has already been opened, return -EBUSY
	if(filp->f_mode & FMODE_WRITE) {
		printk(KERN_INFO "device_open in writing mode\n");
		if (atomic_read(&write_device_open) > 0) return -EBUSY;
		else atomic_inc(&write_device_open);
	}
	else {
		printk(KERN_INFO "device_open in read mode\n");
	}

	struct info_mydev *info_dev = container_of(inode->i_cdev, struct info_mydev, mydev_cdev);

	filp->private_data = info_dev;

	return 0;
}

static int device_release(struct inode *inode, struct file *filp) {
	printk(KERN_INFO "device_release\n");

	//again, if file was open in writing mode, reduce counter
	if(filp->f_mode & FMODE_WRITE) atomic_dec(&write_device_open);
	return 0;
}
static ssize_t device_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	printk(KERN_INFO "device_write\n");
	data_size = count;
	int ret;
	size_t copied_bytes;
	char dir;
	copy_from_user(&dir, buf, count);

	while (data_size > 0) {

		//IF buffer esta lleno, bloquear proceso

		bytes_to_write = min(data_size, kfifo_avail(&fifo));
		ret = kfifo_from_user(&fifo, buf, data_to_write, &copied_bytes);
		if (ret != 0) {
			printk(KERN_INFO "error in kfifo_from_user\n");
			return -EFAULT;
		}

		data_size -= bytes_to_write;
	
		buf += data_size;

		if (!device_is_active) {
			sound.device_is_active = 1;
			//llamada a handle_sound
		}

	}
	size_t ret = count;
	return ret;
}
static ssize_t device_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
	printk(KERN_INFO "device_read\n");
	size_t ret = count;
	return ret;
}

static struct file_operations ejemplo_fops = {
        .owner =    THIS_MODULE,
        .open =     device_open,
        .release =  device_release,
        .write =    device_write,
		.read = 	device_read
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





	//handle buffer and buffer threshold
	unsigned int x = 2;
    
	if (buffer_size > PAGE_SIZE){
		buffer_size = PAGE_SIZE;
	}
	else if(buffer_size & (buffer_size - 1)){ //buffer_size is not a power of 2
		
		for (x=2;x<16384;x=x*2){

			if (x>=buffer_size){
				buffer_size = x;
				break;
			}

		}
	}

	if (buffer_threshold > buffer_size){
		buffer_threshold = buffer_size;
	}

	//allocate fifo
	int alloc_kfifo = kfifo_alloc(&fifo, buffer_size, GFP_ATOMIC);
	if (alloc_kfifo != 0) {
		printk(KERN_INFO "error allocating KFIFO for fifo\n");
		return -ENOMEM;
	}

	//allocate fifo to handle complete and incomplete sounds
	alloc_kfifo = kfifo_alloc(&sound_fifo, 4, GFP_ATOMIC);
	if (alloc_kfifo != 0) {
		printk(KERN_INFO "error allocating KFIFO for sound fifo\n");
		return -ENOMEM;
	}

	//allocate timer






	/*play sound
	spkr_set_frequency(5000);
	spkr_on();*/

	return 0;
}

void spkr_exit(void) {
	printk(KERN_INFO "spkr exit\n");

	device_destroy(spkrClass, midispo);
	class_destroy(spkrClass);
	cdev_del(&mydevice);
	unregister_chrdev_region(midispo, 1);

	/*bye sound
	spkr_off();*/
}

module_init(spkr_init);
module_exit(spkr_exit);