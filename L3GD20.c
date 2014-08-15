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
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/device.h>
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

  struct cdev        char_dev;
  struct device      dev;

  struct L3GD20_td  *data;
  int                cur_index; // This is the next to be written to...
};

static inline struct L3GD20_device* L3GD20_DEV(struct inode* inode) {
  return container_of(inode->i_cdev, struct L3GD20_device, char_dev);
}

struct L3GD20_file_info {
  int cur_index;
  struct L3GD20_device *L3GD20;
};

static spinlock_t active_devs_lock;
static struct L3GD20_device active_devs[NUM_DEV];

//static int cur_index = 0;
//static struct L3GD20_td data[SAMPLE_COUNT];

/* 
 * Char driver interface
 */
static void* L3GD20_seq_start(struct seq_file *s, loff_t *pos);
static void* L3GD20_seq_next (struct seq_file *s, void *v, loff_t *pos);
static void  L3GD20_seq_stop (struct seq_file *s, void *v);
static int   L3GD20_seq_show (struct seq_file *s, void *v);

static struct seq_operations L3GD20_seq_ops = {
  .start = L3GD20_seq_start,
  .next =  L3GD20_seq_next,
  .stop =  L3GD20_seq_stop,
  .show =  L3GD20_seq_show, 
};


int     L3GD20_open     (struct inode *inode, struct file *filp);
static struct file_operations L3GD20_file_ops = {
  .owner   = THIS_MODULE,
  .open    = L3GD20_open,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = seq_release,
};

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
static int frz_L3GD20_read_measure(struct i2c_client *client, struct L3GD20_d *data);


/* 
 * Threaded interupt
 * @dev_id - should be m_activedev
 */
static irqreturn_t frz_handle_interupt(int irq, void* dev_id) {
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

  if ((dev_id <(void*) active_devs) || (dev_id >(void*) (active_devs + NUM_DEV))) {
    printk("WTF %s\n", __func__);
    return IRQ_NONE;
  }

  spin_lock(&active_devs_lock);
  client = drv->client;
  spin_unlock(&active_devs_lock);

  if (IS_ERR(client)) return IRQ_NONE;

  frz_L3GD20_read_measure(drv->client, &drv->data[drv->cur_index % SAMPLE_COUNT].d);
  drv->cur_index++;

  return (IRQ_HANDLED);
}

/*
 * Fast interupt - gets the time of interupt
 * @dev_id - should be m_activedev
 */
static irqreturn_t frz_quick_check(int irq, void* dev_id) {
  struct L3GD20_device *drv;

  drv = dev_id;

  if ((dev_id <(void*) active_devs) || (dev_id >(void*) (active_devs + NUM_DEV))) {
    return IRQ_NONE;
  }

  // Get index in buffer, then get time
  getnstimeofday(&drv->data[drv->cur_index % SAMPLE_COUNT].t);
  return (IRQ_WAKE_THREAD);
}

/* 
 * 
 */
static int frz_L3GD20_probe(struct i2c_client *client,
                     const struct i2c_device_id *id) {
  int sig;
  int ret;
  int i;

  struct L3GD20_device *d = NULL;

  printk("Probing\n");
  ret = -ENODEV;

  spin_lock(&active_devs_lock);
  for (i = 0; i < NUM_DEV && !d; i++) {
    if (active_devs[i].client == NULL) {
      d = active_devs + i;
    }
  }
  if (d) {
    d->client = client;
  }
  spin_unlock(&active_devs_lock);
  if (!d) {
    goto err;
  }
  

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

  // Allocate the buffer
  d->data = (struct L3GD20_td*) __get_free_page(GFP_KERNEL);

  if (d->data == NULL) {
    ret = -ENOMEM;
    goto err_free_dev;
  }
  memset(d->data, 0, PAGE_SIZE);

  d->cur_index = 0;
  printk("Buffer size: %ld   L3GD20_td size: %u Page size: %ld\n", SAMPLE_COUNT, sizeof(struct L3GD20_td), PAGE_SIZE);


  // TODO - not make this hard coded
  d->irq = gpio_to_irq(17);
  printk("IRQ %d\n", d->irq);
  if ((ret = request_threaded_irq(d->irq,
                           frz_quick_check, 
                           frz_handle_interupt, 
                           IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_HIGH  , 
                           "L3GD20_rising_high", 
                           d)) < 0) {
    goto err_free_mem;
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

  printk("Success\n");
  return 0;

//err_rem_device:
//  device_del(&d->dev);
err_free_irq:
  free_irq(d->irq, d);
err_free_mem:
  free_page((unsigned long) d->data); 
err_free_dev:
  d->client = NULL;
err:
  printk("Err: %s\n", __func__);
  return ret;
}

static int frz_L3GD20_remove(struct i2c_client *client) {
  int i;
  struct L3GD20_device *dv;
  printk("Removing \n");

  for (i = 0; i < NUM_DEV; i++) {
    if (active_devs[i].client == client) {
      dv = &active_devs[i];
      break;
    }
  }

  if (!dv) {
    printk("WTF?? %s\n", __func__);
    return -ENODEV;
  }

  free_irq(dv->irq, dv); dv->client = NULL;

  device_del(&dv->dev);

  if (dv->data)  free_page((unsigned long) dv->data);
  dv->data = NULL;

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

/*
 * Begin File interface
 */

static int __init L3GD20_init(void) {
  int ret;
  int i;

  spin_lock_init(&active_devs_lock);

  if ((ret = alloc_chrdev_region(&L3GD20_dev, 0, NUM_DEV, "L3GD20")) < 0) {
    goto out; 
  }

  for (i = 0; i < NUM_DEV; i++) {
    active_devs[i].dev.devt = MKDEV(MAJOR(L3GD20_dev), i);
    cdev_init(&active_devs[i].char_dev, &L3GD20_file_ops);
    active_devs[i].char_dev.owner = THIS_MODULE;
    if(cdev_add(&active_devs[i].char_dev, active_devs[i].dev.devt, 1) < 0) {
      printk("cdev_add failed\n");
    }
  }

  ret = i2c_add_driver(&frz_L3GD20_driver);
  if (ret < 0) {
    printk("i2c_add_driver return %d\n", ret);
    goto out_release_region;
  }

  L3GD20_class = class_create(THIS_MODULE, "L3GD20");
  if (IS_ERR(L3GD20_class)) {
    ret = PTR_ERR(L3GD20_class);
    goto out_remove_driver;
  }
  
  return ret;

out_remove_driver:
  i2c_del_driver(&frz_L3GD20_driver); 
out_release_region:
  unregister_chrdev_region(L3GD20_dev, NUM_DEV);
out:
  return ret;
}

static void __exit L3GD20_cleanup(void) {
  int i;

  for (i = 0; i < NUM_DEV; i++) {
    cdev_del(&active_devs[i].char_dev);
    device_destroy(L3GD20_class, active_devs[i].dev.devt);
  }

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

static int frz_L3GD20_read_measure(struct i2c_client *client, struct L3GD20_d *data) {
  if (data == NULL) return -EINVAL;

  if (i2c_smbus_read_i2c_block_data(client, 
                                    L3GD20_REG_OUT_X_L | L3GD20_REG_AUTO_INC, 
                                    sizeof(struct L3GD20_d),  
                                    (void*) data) != sizeof(struct L3GD20_d)) {
    return -EIO;
  }
  return sizeof(struct L3GD20_d);
}

/* 
 * Begin char driver
 */
int L3GD20_open(struct inode *inode, struct file *filp) {
  int ret;

  ret = seq_open(filp, &L3GD20_seq_ops);

  if (ret >= 0) {
    struct L3GD20_file_info *f_info = kzalloc(sizeof(struct L3GD20_file_info), GFP_KERNEL);
    struct seq_file *p = filp->private_data;

    f_info->L3GD20 = L3GD20_DEV(inode);
    f_info->cur_index =0;// f_info->L3GD20->cur_index;
    printk("%s: %p\n", __func__, p);

    p->private = f_info;
  }

  return ret;
}

static void* L3GD20_seq_start(struct seq_file *s, loff_t *pos) {
  struct L3GD20_file_info *f_info = s->private;
  if (f_info == NULL) {
    printk("%s: %p %p\n", __func__, f_info, s);
  }
  else {
    printk("%s: %d %p\n", __func__, f_info->cur_index, f_info);
  }
  return f_info;
}

static void* L3GD20_seq_next (struct seq_file *s, void *v, loff_t *pos) {
  printk("%s: %p\n", __func__, v);
  return NULL;
}

static void  L3GD20_seq_stop (struct seq_file *s, void *v) {
  if (s->private) {
    kfree(s->private);
  }
}

static int   L3GD20_seq_show (struct seq_file *s, void *v) {
  return seq_printf(s, "%s: %p\n", __func__, v);
}

MODULE_LICENSE("GPL");

module_init(L3GD20_init);
module_exit(L3GD20_cleanup);
