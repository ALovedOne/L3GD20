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

#include "L3GD20.h"
#include "frz.h"

#define NUM_DEV 1

static dev_t L3GD20_dev;

struct L3GD20_d {
  u16 X;
  u16 Y;
  u16 Z;
};

struct L3GD20_td {
  struct timespec t;
  struct L3GD20_d d;
};

struct L3GD20_device {
  struct i2c_client *client;
  int irq;
  struct L3GD20_td  *data;
  int                data_size;
  int                cur_index;
};

static struct L3GD20_device *m_activedev = NULL;

/* 

 */

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
  struct L3GD20_d d;

  if (dev_id != m_activedev) {
    printk("WTF %s\n", __func__);
    return (IRQ_HANDLED);
  }

  frz_L3GD20_read_measure(m_activedev->client, &d);

  return (IRQ_HANDLED);
}

/*
 * Fast interupt - gets the time of interupt
 * @dev_id - should be m_activedev
 */
static irqreturn_t frz_quick_check(int irq, void* dev_id) {
  if (dev_id != m_activedev) {
    return (IRQ_NONE);
  } 
  
  // Get index in buffer, then get time
//  getnstimeofday(&m_last_sec);
  return (IRQ_WAKE_THREAD);
}


/* 
 * 
 */
static int frz_L3GD20_probe(struct i2c_client *client,
                     const struct i2c_device_id *id) {
  int sig;
  int ret;

  m_activedev = kzalloc(sizeof(struct L3GD20_device), GFP_KERNEL);
  m_activedev->client = client;

  if (IS_ERR(m_activedev)) {
    ret = -ENOMEM;
    goto err;
  }
  

  ret = -ENODEV;

  sig = frz_L3GD20_read_byte(client, L3GD20_REG_SIG);
  if (sig < 0) {
    printk("Failed to read: %d\n", sig);
    goto err_free_mem;
  }

  if (sig != L3GD20_SIG) {
    printk("Wrong sig returned, got: %X, expected: %X\n", sig, L3GD20_SIG);
    goto err_free_mem;
  }

  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG1, 0b01001111) < 0)    goto err_free_mem;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG2, 0b00101001) < 0)    goto err_free_mem;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG3, 0b00001000) < 0)    goto err_free_mem;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG4, 0b10000000) < 0)    goto err_free_mem;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG5, 0b01000000) < 0)    goto err_free_mem;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_FIFO_CTRL_REG, 0b0100001) < 0) goto err_free_mem;

  m_activedev->irq = gpio_to_irq(17);
  if ((ret = request_threaded_irq(m_activedev->irq,
                           frz_quick_check, 
                           frz_handle_interupt, 
                           IRQF_ONESHOT | IRQF_TRIGGER_HIGH | IRQF_TRIGGER_RISING, 
                           "L3GD20_high", 
                           m_activedev)) < 0) {
    goto err_free_mem;
  }

  // TODO - is this required?
  //frz_L3GD20_read_measure(client, &d);

  return 0;

err_free_irq:
  free_irq(m_activedev->irq, m_activedev);

err_free_mem:
  kfree(m_activedev);
err:
  m_activedev = NULL;
  printk("Err: %s\n", __func__);
  return ret;
}

static int frz_L3GD20_remove(struct i2c_client *client) {
  printk("Removing \n");

  if (m_activedev->client != client) {
    printk("WTF?? %s\n", __func__);
  }

  free_irq(m_activedev->irq, m_activedev);
  kfree(m_activedev);
  m_activedev = NULL;
  
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
  struct i2c_adapter *i2c_ad;

  ret = i2c_add_driver(&frz_L3GD20_driver);
  if (ret < 0) {
    printk("i2c_add_driver return %d\n", ret);
    goto out;
  }

  return ret;
put_addapter:
  i2c_put_adapter(i2c_ad);
remove_driver:
  i2c_del_driver(&frz_L3GD20_driver);
out:
  return ret;
}

static void __exit L3GD20_cleanup(void) {
  i2c_del_driver(&frz_L3GD20_driver);
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


MODULE_LICENSE("GPL");

module_init(L3GD20_init);
module_exit(L3GD20_cleanup);
