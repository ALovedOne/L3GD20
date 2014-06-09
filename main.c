#include <linux/module.h>
#include <linux/i2c.h>

#include "frz.h"

//struct workqueue_struct *m_work_queue;

static struct i2c_client *i2c_L3GD20;
static struct i2c_board_info bi_L3GD20 = {
  I2C_BOARD_INFO("L3GD20", L3GD20_ADDRESS),
};

static int __init frz_init(void) {
  int ret;
  struct i2c_adapter *i2c_ad;

  // Build L3GD20 client
  ret = i2c_add_driver(&frz_L3GD20_driver);
  if (ret < 0) {
    printk("i2c_add_driver return %d\n", ret);
    goto out;
  }

  i2c_ad = i2c_get_adapter(1);
  if (i2c_ad == NULL) {
    ret = -ENODEV;
    printk("no i2c adapter 1\n");
    goto remove_driver;
  }

  i2c_L3GD20 = i2c_new_device(i2c_ad, &bi_L3GD20);
  if (i2c_L3GD20 == NULL) {
    printk("Filed to create device\n");
    goto put_addapter;
  }

  i2c_put_adapter(i2c_ad);

  // Build workqueue
  //m_work_queue = create_singlethread_workqueue("frz_queue");

  return ret;

put_addapter:
  i2c_put_adapter(i2c_ad);
remove_driver:
  i2c_del_driver(&frz_L3GD20_driver);   
out:
  return ret;
}

static void __exit frz_cleanup(void) {
  if (i2c_L3GD20) {
    i2c_unregister_device(i2c_L3GD20);
  }
  i2c_del_driver(&frz_L3GD20_driver);

  //flush_workqueue(m_work_queue);
  //destroy_workqueue(m_work_queue);
}

MODULE_LICENSE("GPL");

module_init(frz_init);
module_exit(frz_cleanup);
