
#define L3GD20_ADDR  0x6b
#define L3GD20_SIG 0b11010100

#define L3GD20_REG_SIG        0xf
#define L3GD20_REG_AUTO_INC   0x80

#define L3GD20_REG_CTRL_REG1      0x20
#define L3GD20_REG_CTRL_REG2      0x21
#define L3GD20_REG_CTRL_REG3      0x22
#define L3GD20_REG_CTRL_REG4      0x23
#define L3GD20_REG_CTRL_REG5      0x24
#define L3GD20_REG_OUT_TEMP       0x26
#define L3GD20_REG_STATUS         0x27
#define L3GD20_REG_OUT_X_L        0x28
#define L3GD20_REG_OUT_X_H        0x29
#define L3GD20_REG_OUT_Y_L        0x2A
#define L3GD20_REG_OUT_Y_H        0x2B
#define L3GD20_REG_OUT_Z_L        0x2C
#define L3GD20_REG_OUT_Z_H        0x2D
#define L3GD20_REG_FIFO_CTRL_REG  0x2E
#define L3GD20_REG_FIFO_SRC_REG   0x2F
#define L3GD20_REG_INT1_CFG       0x30


// REG 1
#define L3GD20_BIT_REG1_ODR_95  (0 << 6)
#define L3GD20_BIT_REG1_ODR_190 (1 << 6)
#define L3GD20_BIT_REG1_ODR_380 (2 << 6)
#define L3GD20_BIT_REG1_ODR_760 (3 << 6)

#define L3GD20_BIT_REG1_CUT_H   (3 << 4)

#define L3GD20_BIT_REG1_PU      (1 << 3)
#define L3GD20_BIT_REG1_ZEN     (1 << 2)
#define L3GD20_BIT_REG1_YEN     (1 << 1)
#define L3GD20_BIT_REG1_XEN     (1 << 0)

// REG 2
#define L3GD20_BIT_REG2_HF_NORM_R (0 << 4)
#define L3GD20_BIT_REG2_HF_REF    (1 << 4)
#define L3GD20_BIT_REG2_HF_NORM   (2 << 4)
#define L3GD20_BIT_REG2_HF_AUTO_R (3 << 4)

#define L3GD20_BIT_REG2_HF_1       0b1001

// REG 3
#define L3GD20_BIT_REG3_INT1_EN (1 << 7)
#define L3GD20_BIT_REG3_BOOT_ST (1 << 6)
#define L3GD20_BIT_REG3_INT1_HL (1 << 5)
#define L3GD20_BIT_REG3_OD      (1 << 4) 
