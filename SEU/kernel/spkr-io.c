#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>

void spkr_init(void) {
	printk(KERN_INFO "spkr init\n");
}

void spkr_exit(void) {
	printk(KERN_INFO "spkr exit\n");
}

void spkr_set_frequency(unsigned int frequency) {
	printk(KERN_INFO "spkr set frequency: %d\n", frequency);
}

void spkr_on(void) {
	printk(KERN_INFO "spkr ON\n");
}
void spkr_off(void) {
	printk(KERN_INFO "spkr OFF\n");
}
