/*
 * Copyright (C) 2014 Mike Franklin
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/mod_devicetable.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include "L3GD20.h"
#include "frz.h"

#define NUM_DEV 10

//#define __DEBUG_RATE

static struct class *L3GD20_class;
static dev_t L3GD20_dev;

#ifdef __DEBUG_RATE
static ktime_t t_start;
static int             count = 0;
#endif

#define SAMPLE_COUNT (PAGE_SIZE / sizeof(struct L3GD20_td)) 
struct L3GD20_device {
  struct i2c_client   *client;
  int                  irq;
  struct device       *dev;
  struct input_dev    *input;
  char                 phys[32];
};

int     L3GD20_open     (struct inode *inode, struct file *filp);

/* 
 * Begin i2c device definitions 
 */
static int frz_L3GD20_probe (struct i2c_client *client, const struct i2c_device_id *id);
static int frz_L3GD20_remove(struct i2c_client *client);
static int frz_L3GD20_detect(struct i2c_client *client, struct i2c_board_info *bi);
void dumpConfig(struct i2c_client *client);

static int  frz_input_open (struct input_dev *input);
static void frz_input_close(struct input_dev *input);

static struct i2c_device_id frz_L3GD20_idtable[] = {
  { .name = "L3GD20", .driver_data = 0x0},
  { }
};

struct L3GD20_d m_data;

static const unsigned short normal_i2c[] = { L3GD20_ADDRESS, I2C_CLIENT_END};

struct i2c_driver frz_L3GD20_driver = {
  .driver = {
    .name = "L3GD20",
//    .owner = THIS_MODULE,
  },
  .id_table = frz_L3GD20_idtable,

  .probe        = frz_L3GD20_probe,
  .remove       = frz_L3GD20_remove,
  .class        = I2C_CLASS_HWMON | I2C_CLASS_DDC, 
  .detect       = frz_L3GD20_detect, 
  .address_list = normal_i2c,
};

MODULE_DEVICE_TABLE(i2c, frz_L3GD20_idtable);

static struct attribute *L3GD20_attributes[] = {
//  &dev_attr_disable.attr,
//  &dev_attr_calibrate.attr,
//  &dev_attr_rate.attr,
//  &dev_attr_autosleep.attr,
//  &dev_attr_position.attr,
//#ifdef ADXL_DEBUG
//  &dev_attr_write.attr,
//#endif
  NULL
};

static const struct attribute_group L3GD20_attr_group = {
  .attrs = L3GD20_attributes,
};

/* 
 * Begin utility functions defs
 */
static int frz_L3GD20_read_byte (struct i2c_client *client, u8 reg);
static int frz_L3GD20_write_byte(struct i2c_client *client, u8 reg, u8 value);

/* 
 * Threaded interupt
 * @dev_id - should be m_activedev
 */
static irqreturn_t frz_handle_interupt(int irq, void* dev_id) {
  struct L3GD20_d      data;

  struct L3GD20_device *dev;
  struct i2c_client    *client;

#ifdef __DEBUG_RATE
  ktime_t current_time;
  ktime_t diff;
#endif

#ifdef __DEBUG_RATE
  if (count % 100 == 0) {
    current_time = ktime_get();
    diff = ktime_sub(current_time, t_start);
    printk("100 iterations in %lld \n", ktime_to_ns(diff));
    t_start = current_time;
  }
  count++;
#endif

  memset(&data, 0, sizeof(data));

  dev = dev_id;
  client = dev->client;

  if (client == NULL) {
    printk("client null\n");
    return IRQ_NONE;
  }
  if (IS_ERR(client)) {
    printk("client err\n");
    return IRQ_NONE;
  }

  if (i2c_smbus_read_i2c_block_data(
        client, 
        L3GD20_REG_OUT_X_L | L3GD20_REG_AUTO_INC, 
	      sizeof(data),
        (u8*) &data) 
      != sizeof(struct L3GD20_d)) return IRQ_NONE;

  input_report_abs(dev->input, ABS_X, data.X);
  input_report_abs(dev->input, ABS_Y, data.Y);
  input_report_abs(dev->input, ABS_Z, data.Z);
  input_sync(dev->input);

  return (IRQ_HANDLED);
}

/*
 * Fast interupt - gets the time of interupt
 * @dev_id - should be m_activedev
 */
static irqreturn_t frz_quick_check(int irq, void* dev_id) {
  struct L3GD20_device *dev;

  dev = dev_id;

  // Get index in buffer, then get time
  return (IRQ_WAKE_THREAD);
}

/* 
 * 
 */
static int frz_L3GD20_probe(struct i2c_client *client,
                     const struct i2c_device_id *id) {
  int sig;
  int ret;
  int err;

  int irq = gpio_to_irq(23);

  struct device *i2c_dev = &client->dev;

  struct L3GD20_device *dev_info = kzalloc(sizeof(*dev_info), GFP_KERNEL);
  struct input_dev *input_dev = input_allocate_device();
  
  ret = -EINVAL;

  if (!dev_info || !input_dev) {
    ret = -ENOMEM;
    goto err_free_mem;
  }

  dev_info->input  = input_dev;
  dev_info->dev    = i2c_dev;
  dev_info->irq    = irq;
  dev_info->client = client;
  
  input_dev->name = "L3GD20 Gyro";
  
  sig = frz_L3GD20_read_byte(client, L3GD20_REG_SIG);
  if (sig < 0) {
    printk("Failed to read: %d\n", sig);
    goto err_free_mem;
  }

  if (sig != L3GD20_SIG) {
    printk("Wrong sig returned, got: %X, expected: %X\n", sig, L3GD20_SIG);
    goto err_free_mem;
  }

  snprintf(dev_info->phys, sizeof(dev_info->phys), "%s/input0", dev_name(i2c_dev));

  input_dev->phys = dev_info->phys;
  input_dev->dev.parent = i2c_dev;
  input_dev->id.product = sig;
  input_dev->id.bustype = BUS_I2C;
  input_dev->open = frz_input_open;
  input_dev->close = frz_input_close;
  
  input_set_drvdata(input_dev, dev_info);

  __set_bit(EV_ABS, input_dev->evbit);
  __set_bit(ABS_X,  input_dev->absbit);
  __set_bit(ABS_Y,  input_dev->absbit);
  __set_bit(ABS_Z,  input_dev->absbit);

  input_set_abs_params(input_dev, ABS_X, -4096, 4096, 0, 0);
  input_set_abs_params(input_dev, ABS_Y, -4096, 4096, 0, 0);
  input_set_abs_params(input_dev, ABS_Z, -4096, 4096, 0, 0);

  __set_bit(EV_KEY, input_dev->evbit);
   
  
  // TODO - make this configurable
  /*
    dr  = 00 - 190 hrz
    bw  = 00
    PD  = 1
    Zen = 1
    Xen = 1
    Yen = 1
  */
  //if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG2,     0b00101001) < 0)    goto err_free_mem;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG3,     0b00001000) < 0)    goto err_free_mem;
  //if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG4,     0b00000000) < 0)    goto err_free_mem;
  //if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG5,     0b01000000) < 0)    goto err_free_mem;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_FIFO_CTRL_REG, 0b0100001) < 0) goto err_free_mem;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG1,     0b00001111) < 0)    goto err_free_mem;

  err = sysfs_create_group(&i2c_dev->kobj, &L3GD20_attr_group);
  if (err)
    goto err_free_mem;

  err = input_register_device(input_dev);
  if (err)
    goto err_remove_attr;

  i2c_set_clientdata(client, dev_info);

  // TODO - not make this hard coded
  printk("IRQ %d\n", dev_info->irq);
  if ((ret = request_threaded_irq(dev_info->irq,
                           frz_quick_check, 
                           frz_handle_interupt, 
                           IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH  , 
                           "L3GD20_rising_high", 
                           dev_info)) < 0) {
    goto err_free_mem;
  }

  dumpConfig(client);

  printk("Success\n");
  return 0;

//err_free_irq:
//  free_irq(dev_info->irq, dev_info);
err_remove_attr:
  sysfs_remove_group(&i2c_dev->kobj, &L3GD20_attr_group);
err_free_mem:
  input_free_device(input_dev);
  kfree(dev_info);
  return ret;
}

void dumpConfig(struct i2c_client *client) {
  unsigned char data[8];
  int idx;

  if (i2c_smbus_read_i2c_block_data(
        client,
        0x20 | L3GD20_REG_AUTO_INC,
        sizeof(data),
        (u8*) data)
      != sizeof(data)) {
    printk("Failed to read\n"); 
    return;
  }

  for (idx = 0; idx < sizeof(data); idx++) {
    printk("%X: %2.2X\n", 0x20 + idx, data[idx]);
  }
}

static int frz_input_open(struct input_dev *input) {
  return 0; 
}

static void frz_input_close(struct input_dev *input) {

}

static int frz_L3GD20_remove(struct i2c_client *client) {
  struct L3GD20_device *dev;

  printk("Removing \n");

  dev = i2c_get_clientdata(client);
  if (!dev) {
    printk("WTF?? %s\n", __func__);
    return -ENODEV;
  }

  sysfs_remove_group(&dev->dev->kobj, &L3GD20_attr_group);
  free_irq(dev->irq, dev); dev->client = NULL;
  input_unregister_device(dev->input);
  kfree(dev);

  return 0;
}

static int frz_L3GD20_detect(struct i2c_client *client, struct i2c_board_info *bi) {
  int sig;

  sig = i2c_smbus_read_byte_data(client, L3GD20_REG_SIG);
  if (sig < 0) {
    printk("Failed to read: %d\n", sig);
    return -EIO;
  }

  if (sig == L3GD20_SIG) {
    strcpy(bi->type, "L3GD20");
    return 0;
  }

  return -ENODEV;
}

static int __init L3GD20_init(void) {
  int ret;
  if ((ret = alloc_chrdev_region(&L3GD20_dev, 0, NUM_DEV, "L3GD20")) < 0) {
    goto out; 
  }


  ret = i2c_add_driver(&frz_L3GD20_driver);
  if (ret < 0) {
    printk("i2c_add_driver return %d\n", ret);
    goto out_release_region;
  }

  return ret;

out_release_region:
  unregister_chrdev_region(L3GD20_dev, NUM_DEV);
out:
  return ret;
}

static void __exit L3GD20_cleanup(void) {
  class_destroy(L3GD20_class);
  L3GD20_class = NULL;

  i2c_del_driver(&frz_L3GD20_driver);
  unregister_chrdev_region(L3GD20_dev, NUM_DEV);
}
/* 
 * Begin utility functions 
 */
static int frz_L3GD20_read_byte(struct i2c_client *client, u8 reg) {
  return i2c_smbus_read_byte_data(client, reg);
}


static int frz_L3GD20_write_byte(struct i2c_client *client, u8 reg, u8 value) {
  return i2c_smbus_write_byte_data(client, reg, value);
}

MODULE_LICENSE("GPL");

module_init(L3GD20_init);
module_exit(L3GD20_cleanup);
