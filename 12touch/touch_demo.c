#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/of_gpio.h>


#define uchar unsigned char
#define mydebug() printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__)

static struct i2c_client *touch_client;
static struct input_dev *touch_dev;
static int irq;

static void touch_read(unsigned char sAddr, unsigned char *buf, unsigned int len)
{
    struct i2c_msg msg[2];
	int i, ret;
	unsigned char address;
	for (i = 0; i < len; i++)
	{
		/* 先写入要读取的寄存器地址 */
		address = sAddr + i;
		msg[0].addr  = touch_client->addr;    /* 目的 */
		msg[0].buf   = &address;              /* 源 */
		msg[0].len   = 1;                     /* 地址=1 byte */
		msg[0].flags = 0;                     /* 表示写 */

		/* 然后启动读操作 */
		msg[1].addr  = touch_client->addr;    /* 源 */
		msg[1].buf   = &buf[i];               /* 目的 */
		msg[1].len   = 1;                     /* 数据=1 byte */
		msg[1].flags = I2C_M_RD;              /* 表示读 */
		ret = i2c_transfer(touch_client->adapter, msg, 2);
		if (ret < 0)
		{
			printk("i2c_transfer eror\n");
		}
		mdelay(10);
	}
	
}

static irqreturn_t touch_isr(int irq, void *dev_id)
{
	u8 touches;
	int i, error;
	unsigned char buf;
	disable_irq_nosync(irq);
	touch_read(0x02, &buf, 1);
	mdelay(1000);
	printk("point num %d\n", buf);
/*
	error = ft6236_read(ft6236->client, 0, sizeof(buf), &buf);
	if (error) {
		dev_err(dev, "read touchdata failed %d\n", error);
		return IRQ_HANDLED;
	}

	touches = buf.touches & 0xf;
	if (touches > FT6236_MAX_TOUCH_POINTS) {
		dev_dbg(dev,
			"%d touch points reported, only %d are supported\n",
			touches, FT6236_MAX_TOUCH_POINTS);
		touches = FT6236_MAX_TOUCH_POINTS;
	}

	for (i = 0; i < touches; i++) {
		struct ft6236_touchpoint *point = &buf.points[i];
		u16 x = ((point->xhi & 0xf) << 8) | buf.points[i].xlo;
		u16 y = ((point->yhi & 0xf) << 8) | buf.points[i].ylo;
		u8 event = point->event >> 6;
		u8 id = point->id >> 4;
		bool act = (event == FT6236_EVENT_PRESS_DOWN ||
			    event == FT6236_EVENT_CONTACT);

		input_mt_slot(input, id);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, act);
		if (!act)
			continue;

		if (ft6236->invert_x)
			x = ft6236->max_x - x;

		if (ft6236->invert_y)
			y = ft6236->max_y - y;

		if (ft6236->swap_xy) {
			input_report_abs(input, ABS_MT_POSITION_X, y);
			input_report_abs(input, ABS_MT_POSITION_Y, x);
		} else {
			input_report_abs(input, ABS_MT_POSITION_X, x);
			input_report_abs(input, ABS_MT_POSITION_Y, y);
		}
	}

	input_mt_sync_frame(input);
	input_sync(input);
*/
	enable_irq(irq);
    return IRQ_HANDLED;
}

static int touch_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
	unsigned char buf;
	int ret;
	
    touch_client = client;
    printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);

	touch_read(0xa3, &buf, 1);
	printk("Chip vendor ID  %x\n", buf);

	touch_read(0xa6, &buf, 1);
	printk("Firmware ID %x\n", buf);

	touch_read(0xa8, &buf, 1);
	printk("CTPM Vendor ID %x\n", buf);

	touch_read(0x00, &buf, 1);
	printk("DEVIDE_MODE %x\n", buf);

	touch_read(0x80, &buf, 1);
	printk("ID_G_THGROUP. %x\n", buf);

	touch_read(0x88, &buf, 1);
	printk("ID_G_PERIODACTIVE. %x\n", buf);

	touch_dev = input_allocate_device();
	if (touch_dev == NULL)
    {
        printk("%s, allocate input device, error\n", __func__);
        return -1;
    }

    //告诉input能够支持哪些事件
    touch_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    touch_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    input_set_abs_params(touch_dev, ABS_MT_POSITION_X, 0, 800, 0, 0);
    input_set_abs_params(touch_dev, ABS_MT_POSITION_Y, 0, 480, 0, 0);
	ret = input_mt_init_slots(touch_dev, 2,INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
    if (ret)
    {
        printk("%s, input_mt_init_slots error\n", __func__);
        return ret;
    }
    touch_dev->name = "touch";
    touch_dev->id.bustype = BUS_I2C;
    touch_dev->dev.parent = &(touch_client)->dev;
    ret = input_register_device(touch_dev);
    if (ret)
    {
        printk("%s, register input device, error\n", __func__);
        return ret;
    }
	 
	printk("irq is %d\n", irq); 
 	
    ret = devm_request_threaded_irq(&touch_client->dev, irq, touch_isr, NULL, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "touch1", NULL);

    if (ret < 0)
    {
        printk("failed to request_irq %d\n", ret);
    }
	
    return 0;
}

static int touch_remove(struct i2c_client *client)
{

    return 0;
}

static const struct i2c_device_id touch_id_table[] =
{
    { "touch", 0 },
    {}
};


/* 1. 分配/设置i2c_driver */
static struct i2c_driver touch_driver =
{
    .driver	= {
        .name	= "touch",
        .owner	= THIS_MODULE,
    },
    .probe		= touch_probe,
    .remove		= touch_remove,
    .id_table	= touch_id_table,
};


static int int_demo_remove(struct platform_device *pdev) {

    printk("%s enter.\n", __func__);
	
    return 0;
}

static int int_demo_probe(struct platform_device *pdev) {
	
	struct device *dev = &pdev->dev;	
/*
    int irq_gpio = of_get_named_gpio(dev->of_node, "tiny4412,touch_int", 0);//通过名字获取gpio
	printk("%s %s %d\n", __FILE__, __FUNCTION__, __LINE__);
	if (irq_gpio < 0) {
        printk("Looking up property in node failed\n");
    }

    irq = gpio_to_irq(irq_gpio);	//将gpio转换成对应的中断号
	printk("int_demo_probe %d\n", irq);
	if (irq < 0) {
		printk("gpio_to_irq failed\n");
    }
*/

	irq = platform_get_irq(pdev, 0);
	printk("int_demo_probe %d\n", irq);
    return 0;
}

static const struct of_device_id touch_demo_dt_ids[] = {
    { .compatible = "tiny4412,touch_demo", },
    {},
};
MODULE_DEVICE_TABLE(of, touch_demo_dt_ids);
static struct platform_driver touch_demo_driver = {
    .driver        = {
        .name      = "touch_demo",
        .of_match_table    = of_match_ptr(touch_demo_dt_ids),
    },
    .probe         = int_demo_probe,
    .remove        = int_demo_remove,
};


static int touch_drv_init(void)
{
	int ret;
	/* 1.注册平台设备驱动 */
	ret = platform_driver_register(&touch_demo_driver);
    if (ret)
        printk(KERN_ERR "int demo: probe failed: %d\n", ret);
    /* 2. 注册i2c_driver */
    i2c_add_driver(&touch_driver);
	
    return 0;
}

static void touch_drv_exit(void)
{
    i2c_del_driver(&touch_driver);
	platform_driver_unregister(&touch_demo_driver);
}


module_init(touch_drv_init);
module_exit(touch_drv_exit);
MODULE_LICENSE("GPL");

