#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/gfp.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#define DEBUG


#define KEY_MAJOR	100

typedef struct keydevice {
	dev_t devno;
	struct cdev cdev;
} keydev_t;

keydev_t *keydev = NULL;
struct input_dev *inputkey = NULL;
struct class *keydev_class = NULL;

void __iomem *keybase = NULL;

#define KEY_COUNT	6
#define SWKEY_0		0x3e /* 0011 1110 */
#define SWKEY_1		0x3d /* 0011 1101 */
#define SWKEY_2		0x3b /* 0011 1011 */
#define SWKEY_3		0x37 /* 0011 0111 */
#define SWKEY_4		0x2f /* 0010 1111 */
#define SWKEY_5		0x1f /* 0001 1111 */
#define SWKEY_NONE	0x3f /* 0011 1111 */
#define SWKEY_ERROR	0x00 /* 0000 0000 */

static int keycodes[KEY_COUNT] = {
	KEY_Q,
	KEY_W,
	KEY_A,
	KEY_S,
	KEY_D,
	KEY_E,
};

static struct timer_list key_timer;
static unsigned char g_last_key_status;
static unsigned char g_current_key_status;
static unsigned char g_key_index;
static int g_time_flag = 0;

static int key_open(struct inode *inodep, struct file *filep)
{
	return 0;
}

static int key_release(struct inode *inodep, struct file *filep)
{
	return 0;
}

static ssize_t key_read(struct file *filep, char __user *buff, size_t count, loff_t *offp)
{
	return 0;
}

static ssize_t key_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp)
{
	return 0;
}

static long key_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static struct file_operations key_ops = {
	.owner = THIS_MODULE,
	.open = key_open,
	.release = key_release,
	.read = key_read,
	.write = key_write,
	.unlocked_ioctl = key_ioctl,
};

static unsigned char read_key_status(void)
{
	return readb(keybase);
}

static int __is_only_key(unsigned char data)
{
	int i = 0, flag = 0;
	unsigned char reg = data;
	
	for(i = 0; i < KEY_COUNT; i++){
		if(!(reg & 0x01)){
			flag++;
			g_key_index = i;
		}
		reg = reg >> 1;
	}
	#ifdef DEBUG
	//printk("##flag : %d, index : %d\n", flag, g_key_index);
	#endif
	if(flag == 1)
		return 0;
	else
		return -1;
}

static void shake_timer_function(unsigned long data)
{
	g_current_key_status = read_key_status();
		
	if(g_current_key_status == g_last_key_status){
		mod_timer(&key_timer, jiffies + HZ/100);
	}
	else {
		printk("key value: %02x\n", g_last_key_status);
		switch (g_last_key_status) {
			case SWKEY_0:
				input_report_key(inputkey, keycodes[0], 1);
				input_sync(inputkey);
				input_report_key(inputkey, keycodes[0], 0);
				input_sync(inputkey);
				break;
			
			case SWKEY_1:
				input_report_key(inputkey, keycodes[1], 1);
				input_sync(inputkey);
				input_report_key(inputkey, keycodes[1], 0);
				input_sync(inputkey);
				break;
			
			case SWKEY_2:
				input_report_key(inputkey, keycodes[2], 1);
				input_sync(inputkey);
				input_report_key(inputkey, keycodes[2], 0);
				input_sync(inputkey);
				break;
			
			case SWKEY_3:
				input_report_key(inputkey, keycodes[3], 1);
				input_sync(inputkey);
				input_report_key(inputkey, keycodes[3], 0);
				input_sync(inputkey);
				break;
			
			case SWKEY_4:
				input_report_key(inputkey, keycodes[4], 1);
				input_sync(inputkey);
				input_report_key(inputkey, keycodes[4], 0);
				input_sync(inputkey);
				break;
			
			case SWKEY_5:
				input_report_key(inputkey, keycodes[5], 1);
				input_sync(inputkey);
				input_report_key(inputkey, keycodes[5], 0);
				input_sync(inputkey);
				break;
			
			default:
				break;
		}
		g_time_flag = 0;

		g_current_key_status = SWKEY_NONE;
		g_last_key_status = SWKEY_NONE;
		del_timer(&key_timer);
	}

	
	//del_timer(&key_shake_timer[g_key_index]);
	g_key_index = 0;
}

static irqreturn_t keyirq_handler(int irqno, void *dev_id)
{
	int ret = 0;

	g_last_key_status = read_key_status();

	#ifdef DEBUG
	//printk("current reg value : 0x%02x\n", g_last_key_status);
	#endif

	//判断按下的瞬间是不是只有一个按键按下
	ret = __is_only_key(g_last_key_status);
	if(ret != 0)
		goto exit;

	//开启定时器去抖动，然后在定时器的函数里面再读寄存器状态值，再根据相应值报告事件
	if(g_time_flag == 0) {
		init_timer(&key_timer);
		key_timer.function = shake_timer_function;
		key_timer.expires = jiffies + HZ/100;
		add_timer(&key_timer);
		g_time_flag = 1;
	}
	else {
		mod_timer(&key_timer, jiffies + HZ/100);
	}

	return IRQ_HANDLED;
exit:
	g_last_key_status = SWKEY_NONE;
	g_key_index = 0;
	return IRQ_HANDLED;
}

static int __devinit key_probe(struct platform_device *pdev)
{
	int ret, i = 0;
	struct resource *mem = NULL;
	int irqno, iosize;
	
	/* 按键字符设备 */
	keydev = (keydev_t *)kmalloc(sizeof(keydev_t), GFP_KERNEL);
	if(!keydev){
		printk("kmalloc keydev error!\n");
		return -ENODEV;
	}
	
	keydev->devno = MKDEV(KEY_MAJOR, 0);
	ret = register_chrdev_region(keydev->devno, 1, "KBD");
	if(ret){
		printk("register dev number error!\n");
		goto error_exit;
	}
	
	cdev_init(&keydev->cdev, &key_ops);
	keydev->cdev.owner = THIS_MODULE;
	keydev->cdev.ops = &key_ops;

	ret = cdev_add(&keydev->cdev, keydev->devno, 1);
	if(ret < 0){
		printk("keydev char device add error!\n");
		return -ENODEV;
	}
	
	keydev_class = class_create(THIS_MODULE, "keydev_class");
	device_create(keydev_class, NULL, keydev->devno, "key", "key%d", 0);
	
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!mem){
		printk("get platform resource error!\n");
		return -ENXIO;
	}
	iosize = resource_size(mem);
	irqno = platform_get_irq(pdev, 0);
	if(irqno < 0){
		printk("get platform irq error!\n");
		return -EIO;
	}
	
	/* ioremap */
	keybase = ioremap(mem->start, iosize);
	if(!keybase){
		printk("ioremap error!\n");
		return -EIO;
	}
	printk("mem start:0x%08x, iosize:%x, irqno:%d, HZ:%d\n", mem->start, iosize, irqno, HZ);
	
	/* request irq */
	ret = request_irq(irqno, keyirq_handler, IRQ_TYPE_EDGE_FALLING, "key", keydev);
	if(ret < 0 ){
		printk("request irq error!\n");
		return -EIO;
	}
	irq_set_irq_type(irqno, IRQ_TYPE_EDGE_FALLING);

	/* 按键输入设备 */
	inputkey = input_allocate_device();
	inputkey->keycode = &keycodes;
	inputkey->keycodemax = KEY_COUNT;
	inputkey->keycodesize = sizeof(unsigned char);
	inputkey->name = "keyboard";
	inputkey->id.bustype = BUS_HOST;
	
	set_bit(EV_KEY, inputkey->evbit);
	for(i = 0; i < KEY_COUNT; i++){
		set_bit(keycodes[i], inputkey->keybit);
	}
	ret = input_register_device(inputkey);
	if(ret != 0 ){
		printk("input device register error!\n");
		return -ENODEV;
	}

	printk("SANWAY KEYBOARD init ok...");
	
	return 0;

error_exit:
	return -1;
}

static int __devexit key_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver key_driver = {
	.probe = key_probe,
	.remove = key_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sw_key",
	},
};

static int __init key_init(void)
{
	return platform_driver_register(&key_driver);
}

static void __exit key_exit(void)
{
	platform_driver_unregister(&key_driver);
}

module_init(key_init);
module_exit(key_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KEY Driver For PSQ3200 Device");
MODULE_AUTHOR("fuwang@szsanway.com");


