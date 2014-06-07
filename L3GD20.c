#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/mod_devicetable.h>

#include "L3GD20.h"
#include "frz.h"

struct L3GD20_d {
  u16 X;
  u16 Y;
  u16 Z;
};


static int frz_L3GD20_probe (struct i2c_client *client, const struct i2c_device_id *id);
static int frz_L3GD20_remove(struct i2c_client *client);

static int frz_L3GD20_read_byte (struct i2c_client *client, u8 reg);
static int frz_L3GD20_write_byte(struct i2c_client *client, u8 reg, u8 value);
static int frz_L3GD20_read_measure(struct i2c_client *client, struct L3GD20_d *data);

static int m_irq = -1;
static struct i2c_client *m_client = NULL;

static struct i2c_device_id frz_L3GD20_idtable[] = {
  { .name = "L3GD20", .driver_data = 0x0},
  { }
};

MODULE_DEVICE_TABLE(i2c, frz_L3GD20_idtable);

struct i2c_driver frz_L3GD20_driver = {
  .driver = {
    .name = "L3GD20",
  },
  .id_table = frz_L3GD20_idtable,
  .probe = frz_L3GD20_probe,
  .remove = frz_L3GD20_remove,

};

static irqreturn_t frz_handle_interupt(int irq, void* dev_id) {
  struct L3GD20_d d;

  if (dev_id != m_client) {
    printk("WTF %s\n", __func__);
    return (IRQ_HANDLED);
  }

  frz_L3GD20_read_measure(m_client, &d);

  return (IRQ_HANDLED);
}

static irqreturn_t frz_quick_check(int irq, void* dev_id) {
  if (dev_id != m_client) {
    return (IRQ_NONE);
  } 
  return (IRQ_WAKE_THREAD);
}

static int frz_L3GD20_probe(struct i2c_client *client,
                     const struct i2c_device_id *id) {
  int sig;
  struct L3GD20_d d;
  printk("Probing: %s\n", id->name);

  sig = frz_L3GD20_read_byte(client, L3GD20_REG_SIG);
  if (sig < 0) {
    printk("Failed to read: %d\n", sig);
    goto err;
  }

  if (sig != L3GD20_SIG) {
    printk("Wrong sig returned, got: %X, expected: %X\n", sig, L3GD20_SIG);
    goto err;
  }

  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG1, 0b11001111) < 0) goto err;
  //if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG1, 0b11001111) < 0) goto err;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG2, 0b00101001) < 0) goto err;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG3, 0b00001000) < 0) goto err;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG4, 0b10000000) < 0) goto err;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG5, 0b01000000) < 0) goto err;

  if(frz_L3GD20_write_byte(client, L3GD20_REG_FIFO_CTRL_REG, 0b0100001) < 0) goto err;

  printk("%d\n", frz_L3GD20_read_measure(client, &d));

  m_client = client;

  m_irq = gpio_to_irq(17);
  if (request_threaded_irq(m_irq, 
                           frz_quick_check, 
                           frz_handle_interupt, 
                           IRQF_ONESHOT | IRQF_TRIGGER_HIGH, 
                           "gpio_rising", 
                           client)) {
    printk("GPIO_RISING: trouble getting IRQ: %d\n", m_irq);
    m_irq = 0;
    return (-EIO);
  } else {
    printk("Got IRQ: %d\n", m_irq);
  }

  // Set FIFO

  return 0;

err:
  free_irq(m_irq, client);
  printk("Err: %s\n", __func__);
  return -ENODEV;
}
static int frz_L3GD20_remove(struct i2c_client *client) {
  printk("Removing \n");

  if (m_client != client) {
    printk("WTF?? %s\n", __func__);
  }

  m_client = NULL;

  free_irq(m_irq, client);
  
  return 0;
}

static int frz_L3GD20_read_byte(struct i2c_client *client, u8 reg) {
  // i2c_smbus_read_block_data
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
  return 0;
}
