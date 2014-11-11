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
#include <asm/uaccess.h>

#include "L3GD20.h"
#include "frz.h"

#define NUM_DEV 10


static struct class *L3GD20_class;
static dev_t L3GD20_dev;

#ifdef __DEBUG_RATE
static struct timespec t_start;
static int             count = 0;
#endif

#define SAMPLE_COUNT (PAGE_SIZE / sizeof(struct L3GD20_td)) 
struct L3GD20_device {
  struct i2c_client *client;
  int                irq;

  struct device      dev;
};

int     L3GD20_open     (struct inode *inode, struct file *filp);

/* 
 * Begin i2c device definitions 
 */
static int frz_L3GD20_probe (struct i2c_client *client, const struct i2c_device_id *id);
static int frz_L3GD20_remove(struct i2c_client *client);
static int frz_L3GD20_detect(struct i2c_client *client, struct i2c_board_info *bi);

static struct i2c_device_id frz_L3GD20_idtable[] = {
  { .name = "L3GD20", .driver_data = 0x0},
  { }
};

static const unsigned short normal_i2c[] = { L3GD20_ADDRESS, I2C_CLIENT_END};

struct i2c_driver frz_L3GD20_driver = {
  .driver = {
    .name = "L3GD20",
//    .owner = THIS_MODULE,
  },
  .id_table = frz_L3GD20_idtable,

  .probe = frz_L3GD20_probe,
  .remove = frz_L3GD20_remove,
  .class = I2C_CLASS_HWMON | I2C_CLASS_DDC, 
  .detect = frz_L3GD20_detect, 
  .address_list = normal_i2c,
};

MODULE_DEVICE_TABLE(i2c, frz_L3GD20_idtable);

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

  struct L3GD20_device *drv;
  struct i2c_client    *client;
#ifdef __DEBUG_RATE
  struct timespec t;
  long elapsed_nsec; 
#endif

#ifdef __DEBUG_RATE
  if (count % 100 == 0) {
    getnstimeofday(&t);
    elapsed_nsec = (t.tv_sec - t_start.tv_sec) * 1000000000;
    elapsed_nsec += (t.tv_nsec - t_start.tv_nsec);
    printk("100 iterations in %ld nsecs\n", elapsed_nsec);
    t_start.tv_sec = t.tv_sec;
    t_start.tv_nsec = t.tv_nsec;
  }
  count++;
#endif

  drv = dev_id;
  client = drv->client;

  if (IS_ERR(client)) return IRQ_NONE;

  if (i2c_smbus_read_i2c_block_data(
        client, 
        L3GD20_REG_OUT_X_L | L3GD20_REG_AUTO_INC, 
        sizeof(struct L3GD20_d),  
        (u8*) &data) 
      == sizeof(struct L3GD20_d)) {
    // Parse the event
    printk("%d %d %d\n", data.X, data.Y, data.Z);
  }
  else {
    printk("??\n");
  }

  return (IRQ_HANDLED);
}

/*
 * Fast interupt - gets the time of interupt
 * @dev_id - should be m_activedev
 */
static irqreturn_t frz_quick_check(int irq, void* dev_id) {
  struct L3GD20_device *drv;

  drv = dev_id;

  // Get index in buffer, then get time
  //getnstimeofday(&drv->data[drv->cur_index % SAMPLE_COUNT].t);
  return (IRQ_WAKE_THREAD);
}

/* 
 * 
 */
static int frz_L3GD20_probe(struct i2c_client *client,
                     const struct i2c_device_id *id) {
  int sig;
  int ret;

  struct L3GD20_device *d = kzalloc(sizeof(*d), GFP_KERNEL);
  struct input_dev *input_dev = input_allocate_device();

  if (!d || !input_dev) {
    ret = -ENOMEM;
    goto err_free_mem;
  }

  ret = -ENODEV;

  d->client = client;
  sig = frz_L3GD20_read_byte(client, L3GD20_REG_SIG);
  if (sig < 0) {
    printk("Failed to read: %d\n", sig);
    goto err_free_dev;
  }

  if (sig != L3GD20_SIG) {
    printk("Wrong sig returned, got: %X, expected: %X\n", sig, L3GD20_SIG);
    goto err_free_dev;
  }

  // TODO - make this configurable
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG1, 0b01001111) < 0)    goto err_free_dev;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG2, 0b00101001) < 0)    goto err_free_dev;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG3, 0b00001000) < 0)    goto err_free_dev;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG4, 0b10000000) < 0)    goto err_free_dev;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG5, 0b01000000) < 0)    goto err_free_dev;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_FIFO_CTRL_REG, 0b0100001) < 0) goto err_free_dev;

  // TODO - not make this hard coded
  d->irq = gpio_to_irq(18);
  printk("IRQ %d\n", d->irq);
  if ((ret = request_threaded_irq(d->irq,
                           frz_quick_check, 
                           frz_handle_interupt, 
                           IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH  , 
                           "L3GD20_rising_high", 
                           d)) < 0) {
    goto err_free_dev;
  }

  device_initialize(&d->dev);
  d->dev.class = L3GD20_class;
  d->dev.parent = &client->dev;
  d->dev.groups = NULL;
  d->dev.release = NULL;
  ret = kobject_set_name(&d->dev.kobj, "L3GD20-%d", MINOR(d->dev.devt));
  if (ret < 0) {
    printk("Failed to set name %d\n", ret);
    goto err_free_irq;
  }

  ret = device_add(&d->dev);
  if (ret < 0) {
    printk("Failed to add device %d\n", ret);
    goto err_free_irq;
  }
  i2c_set_clientdata(client, d);

  printk("Success\n");
  return 0;

//err_rem_device:
//  device_del(&d->dev);
err_free_irq:
  free_irq(d->irq, d);
err_free_dev:
  d->client = NULL;
err_free_mem:
  kfree(d);
  return ret;
}

static int frz_L3GD20_remove(struct i2c_client *client) {
  struct L3GD20_device *dv;

  printk("Removing \n");

  dv = i2c_get_clientdata(client);
  if (!dv) {
    printk("WTF?? %s\n", __func__);
    return -ENODEV;
  }

  free_irq(dv->irq, dv); dv->client = NULL;

  device_del(&dv->dev);

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
