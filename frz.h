

/*
 *  L3GD20 Accelerometer
 */
#define L3GD20_ADDRESS 0x6b
#define L3GD20_SIG 0b11010100
#define L3GD20_REG_SIG  0xf

#define L3GD20_I_PIN     17

extern struct i2c_driver frz_L3GD20_driver;
