#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/i8253.h>

//MODULE_LICENSE("Dual BSD/GPL");

#define PIT_CHANNEL_2_GATE_PORT 0x61  //To control the input signal gets or not to channel 2 of the pit.
#define PIT_CHANNEL_2_DATA_PORT 0x42  //To access timer 2.
#define PIT_OPERATION_MODE_PORT 0x43  //To specify the operation: reads are ignored.

DEFINE_RAW_SPINLOCK(i8253_lock);

/*Manipulate ports 0x42 and 0x43 to set the desired frequency that will be fed to the speaker*/
void spkr_set_frequency(unsigned int frequency) {
	uint32_t div;
	unsigned long flags;

	div = PIT_TICK_RATE / frequency;

	outb(0x43, 0xbe); 						//select timer 2 in mode 3

	raw_spin_lock_irqsave(&i8253_lock, flags);
	outb(0x42, (uint8_t) (div) ); 			//write the first 0-7 bits of the frequency divider.
	outb(0x42, (uint8_t) (div >> 8)); 		//write the last bits 8-15 of the frequency divider.
	raw_spin_unlock_irqrestore(&i8253_lock, flags);

	printk(KERN_INFO "spkr set frequency: %d\n", frequency);
}

/*Connect the system timer to the gate of timer 2 and allow the wave reach the speaker, using port 0x61*/
void spkr_on(void) {
	uint8_t tmp;

	tmp = inb(0x61);

  	if (tmp != (tmp | 3)) {
 		outb(0x61, tmp | 3);
 	}

	printk(KERN_INFO "spkr ON\n");
}

/*Deactivate the speaker manipulating port 0x61*/
void spkr_off(void) {
	uint8_t tmp = inb(0x61) & 0xFC;
 
 	outb(0x61, tmp);

	printk(KERN_INFO "spkr OFF\n");
}

void spkr_init(void) {
	printk(KERN_INFO "spkr init\n");
	spkr_set_frequency(1000);
	spkr_on();
 	//timer_wait(10);
}

void spkr_exit(void) {
	printk(KERN_INFO "spkr exit\n");
	spkr_off();
}

module_init(spkr_init);
module_exit(spkr_exit);


