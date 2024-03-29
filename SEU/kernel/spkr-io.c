#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>

#include <linux/kfifo.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>

#include <linux/spinlock.h>

#include <asm/uaccess.h>
#include <linux/uaccess.h>

#include <linux/mutex.h>

 
#if (LINUX_VERSION_CODE & 0xFFFF00) == KERNEL_VERSION(3,0,0)
#pragma message("LINUX VERSION 3.0.X")
extern raw_spinlock_t i8253_lock;
#else
#include <linux/i8253.h>
#endif  	

MODULE_LICENSE("GPL");

#define  DEVICE_NAME "spkr"
#define ENTRY_NAME "intspkr"
#define CLASS_NAME "speaker"
#define PIT_CHANNEL_2_GATE_PORT 0x61  //To control the input signal gets or not to channel 2 of the pit.
#define PIT_CHANNEL_2_DATA_PORT 0x42  //To access timer 2.
#define PIT_OPERATION_MODE_PORT 0x43  //To specify the operation: reads are ignored.

DEFINE_RAW_SPINLOCK(i8253_lock);

//IOCTL operation
#define MAGIC_NUM '9'
#define SPKR_SET_MUTE_STATE _IOR(MAGIC_NUM, 1, int *)
#define SPKR_GET_MUTE_STATE _IOR(MAGIC_NUM, 2, int *)
#define SPKR_RESET _IO(MAGIC_NUM, 3)

//parameters to be received by user
static int buffer_size = PAGE_SIZE;
static int buffer_threshold = PAGE_SIZE;

//parameters needed for the write function
static size_t data_size;					//initially is equal to count. It indicates the remaining data in the user buffer.
static size_t bytes_to_write;				//indicates the number of bytes to write to the system kfifo.
static int device_is_active = 0; 

//parameters needed by ioctl function
atomic_t reset = ATOMIC_INIT(0); //initially we don't want to reset
int mute[2];

//parameters used to set up module
static dev_t midispo;
static struct cdev mydevice;
static struct class* spkrClass = NULL;
static struct device* spkrDevice = NULL;
static int mj;
static int mn = 0;
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
	unsigned long frequency;
	unsigned long ms;
};

//BLOCKING QUEUE
wait_queue_head_t list_block;

//BLOCKING FSYNC QUEUE
wait_queue_head_t fsync_block;

//MUTEX TO SYNCHRONIZE ACCESS TO FSYNC BLOCKING QUEUE
struct mutex fsync_lock;

//SPINLOCKS
spinlock_t fifo_lock;

//receive minor parameter
module_param(mn, int, S_IRUGO);

//receive parameters buffer_size and buffer_threshold from user.
module_param(buffer_size, int, S_IRUGO);
module_param(buffer_threshold, int, S_IRUGO);

//functions
static int device_open(struct inode *inode, struct file *filp);
static int device_release(struct inode *inode, struct file *filp);
static ssize_t device_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static ssize_t device_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
#if (LINUX_VERSION_CODE & 0xFFFF00) == KERNEL_VERSION(3,0,0)
#pragma message("LINUX VERSION 3.0.X")
static int device_fsync(struct file *filp, int datasync);
#else
#pragma message("LINUX VERSION > 3.0.X")
static int device_fsync(struct file *filp, loff_t start, loff_t end, int datasync);
#endif

static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);



static struct file_operations ejemplo_fops = {
        .owner =    THIS_MODULE,
        .open =     device_open,
        .release =  device_release,
        .write =    device_write,
		.read = 	device_read,
		.fsync = 	device_fsync,
		.unlocked_ioctl =  device_ioctl
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

/*PRODUCE_SOUND
-Check if sound is silent -> if it is, sets device off and timer on.
-If not, sets device on, and also set frequency and timer.
-If ioctl operation SPKR_SET_MUTE says so, we must mute the speaker (but don't stop consuming sounds!)
*/
static void produce_sound(struct my_sound sound) {
	if(sound.frequency == 0) {
		printk(KERN_INFO "SILENCIO\n");
		spkr_off();

		//set timer
		timer.expires = jiffies + msecs_to_jiffies(sound.ms);
		add_timer(&timer);
	}
	else {
		spkr_set_frequency(sound.frequency);

		if(mute[0] == 0) { //if speaker is not muted, turn it on. Else, leave it silent.
			spkr_on();
		}

		//set timer
		timer.expires = jiffies + msecs_to_jiffies(sound.ms);
		add_timer(&timer);
	}
}

/* HANDLE_SOUND
tasks:
-Check if there is a complete sound in fifo -> it has 4 bytes at least.
-If fifo has a complete sound, call produce_sound().
-If fifo doesn't have a complete sound, fill sound_kfifo 's remaining space with the available bytes in fifo and then check if sound_fifo has a complete sound. If it does, call produce_sound(). If not, turn the device off.
-If there is a sound to be completed (sound_fifo is not empty) -> we have to complete and play that sound first, even if there are >= 4 bytes in fifo.
*/
static void handle_sound(void) {
	spin_lock_bh(&fifo_lock);
	unsigned char complete_sound[4];
	struct my_sound sound;
	
	if((kfifo_len(&fifo) >= 4) & (kfifo_is_empty(&sound_fifo))) { //fifo contains at least a complete sound (4 bytes) and there isn't a sound to be completed in sound_fifo.
		
		//printk(KERN_INFO "[**COMPLETE SOUND IN FIFO**]\n");

		kfifo_out(&fifo, &complete_sound, 4);
		sound.frequency = (complete_sound[1]<<8)+complete_sound[0];
		sound.ms = (complete_sound[3]<<8)+complete_sound[2];

		printk(KERN_INFO "frequency = %d\nms = %d\n", sound.frequency, sound.ms);
		device_is_active = 1;
		produce_sound(sound);
	}
	else if ((kfifo_len(&fifo) < 4) | (!kfifo_is_empty(&sound_fifo))) { //fifo doesn't contain a full sound, or there is a sound to be completed stored in sound_fifo.
		unsigned char aux_sound[4];

		int num_elems_to_copy = min(kfifo_avail(&sound_fifo), kfifo_len(&fifo));

		//printk(KERN_INFO "[**INCOMPLETE SOUND IN FIFO**]\n copying %d bytes to sound fifo...\n", num_elems_to_copy);
		
		kfifo_out(&fifo, &aux_sound, num_elems_to_copy);
		kfifo_in(&sound_fifo, &aux_sound, num_elems_to_copy);

		if(kfifo_len(&sound_fifo) == 4) {	//we have a complete sound in sound_fifo
			//printk(KERN_INFO "[**COMPLETE SOUND IN SOUND FIFO**]\n");
			kfifo_out(&sound_fifo, &complete_sound, 4);
			sound.frequency = (complete_sound[1]<<8)+complete_sound[0];
			sound.ms = (complete_sound[3]<<8)+complete_sound[2];

			printk(KERN_INFO "frequency = %d\nms = %d\n", sound.frequency, sound.ms);
			device_is_active = 1;
			produce_sound(sound);
		}
		else {
			device_is_active = 0;
		}

	}
	
	spin_unlock_bh(&fifo_lock);
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
void timer_callback(struct timer_list  *timer)
#else
void timer_callback(const unsigned long d)
#endif
{

	if(kfifo_is_empty(&fifo)) {
		printk(KERN_INFO "[timer_callback] no more sounds in fifo.\n");
		device_is_active = 0;
		wake_up_interruptible(&fsync_block);
		spkr_off();
	}
	else {
		printk(KERN_INFO "[timer_callback] Still some sounds in fifo -> handle_sound().");

		//it reset == 1 -> erase KFIFO.
		if(atomic_read(&reset) == 1) {

			//play current sound (if there is any)
			handle_sound();
			printk(KERN_INFO "[timer_callback] reseting kfifo...\n");
			//erase KFIFO
			kfifo_reset_out(&fifo);
			//set reset = 0, so that we don't keep on reseting kfifo.
			atomic_set(&reset, 0);
		}
		else {
			handle_sound();
		}
		
	}

	printk(KERN_INFO "[timer_callback] kfifo_avail = %d\n", kfifo_avail(&fifo));
	printk(KERN_INFO "[timer_callback] min(data_size, buffer_threshold) = %d\n", min(data_size, buffer_threshold));

	if(kfifo_avail(&fifo) >= min(data_size, buffer_threshold) | kfifo_is_empty(&fifo)) {
		printk(KERN_INFO "[timer_callback] Space available -> waking up process.\n");
		wake_up_interruptible(&list_block);
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
	int ret, retq;
	int copied_bytes;
	
	const char *test_buf;

	if(get_user(test_buf, buf) != 0) {
		printk(KERN_ALERT "Error in user buffer.\n");
		return -EFAULT;
	}

	mutex_lock(&fsync_lock);

	spin_lock_bh(&fifo_lock);
	while (data_size > 0) {
		printk(KERN_INFO "[device_write] *INFO*: device_is_active = %d\n", device_is_active);
		printk(KERN_INFO "fifo avail = %d\n", (kfifo_avail(&fifo)));

		if(kfifo_is_full(&fifo)) {
			printk(KERN_INFO "[device_write] no space available -> blocking process.\n");

			spin_unlock_bh(&fifo_lock);
			retq = wait_event_interruptible(list_block, (!kfifo_is_full(&fifo)));
			spin_lock_bh(&fifo_lock);

			printk(KERN_INFO, "[device_write] space available -> waking up process.\n");
			
			if (retq != 0)
			{
				printk(KERN_INFO "error in wait_event_interruptible\n");
				return -ERESTARTSYS;
			}
		}

		//Copy all possible bytes into fifo
		bytes_to_write = min(data_size, kfifo_avail(&fifo));
		ret = kfifo_from_user(&fifo, buf, bytes_to_write, &copied_bytes);
		if (ret != 0) {
			printk(KERN_INFO "error in kfifo_from_user\n");
			return -EFAULT;
		}

		data_size -= copied_bytes;
		printk(KERN_INFO "[device_write] %d bytes copied to fifo.\n", copied_bytes, data_size);
	
		buf += copied_bytes;

		if (!device_is_active) {

			device_is_active = 1;
			spin_unlock_bh(&fifo_lock);

			printk(KERN_INFO "[device_write]-->handle_sound.\n");

			handle_sound();
			spin_lock_bh(&fifo_lock);
		}

	}

	spin_unlock_bh(&fifo_lock);
	mutex_unlock(&fsync_lock);

	return count;
}

#if (LINUX_VERSION_CODE & 0xFFFF00) == KERNEL_VERSION(3,0,0)
#pragma message("LINUX VERSION 3.0.X")
static int device_fsync(struct file *filp, int datasync)
#else
#pragma message("LINUX VERSION > 3.0.X")
static int device_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
#endif 
{
	
	printk(KERN_INFO "device_fsync\n");
	mutex_lock(&fsync_lock);
	if(wait_event_interruptible(fsync_block, kfifo_len(&fifo) < 4)){
		return -ERESTARTSYS;
	}
	mutex_unlock(&fsync_lock);

	printk(KERN_INFO "exit device_fsync\n");
	return 0;

}

static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)  {
	printk(KERN_INFO "device_ioctl\n");
	unsigned long aux;
	switch(cmd) {
		case SPKR_SET_MUTE_STATE:
		if (copy_from_user(mute, (int *) arg, sizeof(int *)) != 0) {
			printk(KERN_INFO "error in SPKR_SET_MUTE_STATE copy from user\n");  
			return -EFAULT;
		}

		printk(KERN_INFO "mute = %d\n", mute[0]);

			if(mute[0] != 0) { //mute speaker immediately
				printk(KERN_INFO "mute on\n");
				spkr_off();
			}
			else { //unmute speaker
				printk(KERN_INFO "mute off\n");
			}
			printk(KERN_INFO "SPKR_SET_MUTE_STATE: %d\n", mute[0]);
			break;
		case SPKR_GET_MUTE_STATE:
			printk("SPKR_GET_MUTE_STATE: %d\n", mute[0]);

			if(copy_to_user((int *)arg, mute, sizeof(int *)) != 0) {
				printk(KERN_INFO "error in SPKR_GET_MUTE_STATE copy to user\n");
				return -EFAULT;
			}
			break;
		case SPKR_RESET:
			printk(KERN_INFO "SPKR_RESET\n");
			atomic_set(&reset, 1);
	}
}

static ssize_t device_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos) {
	printk(KERN_INFO "device_read\n");
	return count;
}

static int reserve_mjAndmn(void) {
	//reserve major and minor numbers
	int registerMajor = alloc_chrdev_region(&midispo, mn, 1, DEVICE_NAME);

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

	//register device
	register_device_in_kernel();

	//register device in sysfs
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

	printk(KERN_INFO "[device_init] buffer_size = %d", buffer_size);

	if (buffer_threshold > buffer_size){
		buffer_threshold = buffer_size;
	}

	printk(KERN_INFO "[device_init] buffer_threshold = %d", buffer_threshold);

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
	#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    init_timer(&timer);
    timer.data = (unsigned long) &timer;
    timer.function = timer_callback;
	#else
    timer_setup(&timer, timer_callback, 0);
	#endif

	//init process queue
	init_waitqueue_head(&list_block);

	//init fsync queue
	init_waitqueue_head(&fsync_block);

	//init spinlock
	spin_lock_init(&fifo_lock);

	//init fsync mutex
	mutex_init(&fsync_lock);

	return 0;
}

void spkr_exit(void) {
	printk(KERN_INFO "spkr exit\n");
	del_timer_sync(&timer);
	spkr_off();

	device_destroy(spkrClass, midispo);
	class_destroy(spkrClass);
	cdev_del(&mydevice);
	unregister_chrdev_region(midispo, 1);
}

module_init(spkr_init);
module_exit(spkr_exit);