#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
MODULE_LICENSE("Dual BSD/GPL");

void spkr_set_frequency(unsigned int frequency) {
	printk(KERN_INFO "spkr set frequency: %d\n", frequency);
}

void spkr_on(void) {
	printk(KERN_INFO "spkr ON\n");
}
void spkr_off(void) {
	printk(KERN_INFO "spkr OFF\n");
}

int spkr_init(void) {
	printk(KERN_INFO "spkr init\n");
	spkr_set_frequency(1000);
	spkr_on();
	return 0;
}

void spkr_exit(void) {
	printk(KERN_INFO "spkr exit\n");
	spkr_off();
}

module_init(spkr_init);
module_exit(spkr_exit);
