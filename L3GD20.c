#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/mod_devicetable.h>
#include <linux/time.h>
#include <linux/debugfs.h>
#include <linux/mm.h>

#include "L3GD20.h"
#include "frz.h"

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
  struct dentry     *d_L3GD20;
  struct i2c_client *client;
  int irq;
  struct L3GD20_td  *data;
  int                data_size;
  int                cur_index;
};

static struct L3GD20_device *m_activedev = NULL;

/* 

 */

static int  L3GD20_mmap(struct file *filp, struct vm_area_struct *vma);
static void L3GD20_vm_open(struct vm_area_struct *vma);
static void L3GD20_close(struct vm_area_struct *vma);
static int  L3GD20_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

static const struct file_operations L3GD20_fops = {
  .mmap =    L3GD20_mmap,
};

static struct vm_operations_struct mmap_vm_ops = {
  .open =  L3GD20_vm_open,
  .close = L3GD20_close,
  .fault = L3GD20_fault,
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
  struct timespec t;

  if (dev_id != m_client) {
    printk("WTF %s\n", __func__);
    return (IRQ_HANDLED);
  }

  getnstimeofday(&t);

  frz_L3GD20_read_measure(m_client, &d);

  getnstimeofday(&t);
  if ((( t.tv_sec - m_last_sec.tv_sec) > 1) || 
      (((t.tv_sec - m_last_sec.tv_sec) == 1) &&
       (t.tv_nsec > m_last_sec.tv_nsec))) {
    printk("last count: %d\n", m_count);
    m_count = 0;
    m_last_sec.tv_sec  = t.tv_sec;
    m_last_sec.tv_nsec = t.tv_nsec;
  }
  m_count++;

  /*
  if (t.tv_sec > m_last_sec.tv_sec) {
    t.tv_nsec += (t.tv_sec - m_last_sec.tv_sec)* 1000000000;
  }
  printk("It delay of %ld between fast and slow\n", t.tv_nsec - m_last_sec.tv_nsec);
  */

  return (IRQ_HANDLED);
}

static irqreturn_t frz_quick_check(int irq, void* dev_id) {
  if (dev_id != m_client) {
    return (IRQ_NONE);
  } 

//  getnstimeofday(&m_last_sec);
  return (IRQ_WAKE_THREAD);
}

static int frz_L3GD20_probe(struct i2c_client *client,
                     const struct i2c_device_id *id) {
  int sig;
  int ret;
  //struct L3GD20_d d;
  printk("Probing: %s\n", id->name);

  ret = -ENODEV;

  sig = frz_L3GD20_read_byte(client, L3GD20_REG_SIG);
  if (sig < 0) {
    printk("Failed to read: %d\n", sig);
    goto err;
  }

  if (sig != L3GD20_SIG) {
    printk("Wrong sig returned, got: %X, expected: %X\n", sig, L3GD20_SIG);
    goto err;
  }

  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG1, 0b01001111) < 0) goto err;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG2, 0b00101001) < 0) goto err;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG3, 0b00001000) < 0) goto err;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG4, 0b10000000) < 0) goto err;
  if(frz_L3GD20_write_byte(client, L3GD20_REG_CTRL_REG5, 0b01000000) < 0) goto err;

  if(frz_L3GD20_write_byte(client, L3GD20_REG_FIFO_CTRL_REG, 0b0100001) < 0) goto err;

  //frz_L3GD20_read_measure(client, &d);

  m_client = client;

  m_irq = gpio_to_irq(17);
  if (request_threaded_irq(m_irq, 
                           frz_quick_check, 
                           frz_handle_interupt, 
                           IRQF_ONESHOT | IRQF_TRIGGER_HIGH | IRQF_TRIGGER_RISING, 
                           "L3GD20_high", 
                           client)) {
    ret = -EIO;
    goto err;
  }
  printk("Got IRQ: %d\n", m_irq);

  d_L3GD20 = debugfs_create_file("L3GD20", 0444, NULL, NULL, L3GD20_fops);
  if (IS_ERR(d_L3GD20)) {
    goto err_free_irq;
  }

  // Set FIFO

  return 0;

err_free_irq:
  free_irq(m_irq, client);
err:
  m_irq = -1;
  m_client = NULL;
  printk("Err: %s\n", __func__);
  return ret;
}

static int frz_L3GD20_remove(struct i2c_client *client) {
  printk("Removing \n");

  if (m_client != client) {
    printk("WTF?? %s\n", __func__);
  }

  m_client = NULL;

  if (m_irq > 0) {
    free_irq(m_irq, client);
  }
  m_irq = -1;

  if (d_L3GD20) {
    debugfs_remove(d_L3GD20);
  }
  d_L3GD20 = NULL;
  
  return 0;
}

/*
 * Begin File interface
 */
static int L3GD20_mmap(struct file *filp, struct vm_area_struct *vma) {
  vma->vm_ops = &L3GD20_vm_ops;
  vma->vm_flags |= VM_RESERVED;
  mmap_open(vma);
  return 0;
}


/* 
 * Begin utility functions 
 */
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
