#ifndef LSM9DS1_REGISTERS_H
#define LSM9DS1_REGISTERS_H

static constexpr auto LSM9DS1_REGISTER_ACT_THS           = 0x04;
static constexpr auto LSM9DS1_REGISTER_ACT_DUR           = 0x05;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_CFG_XL    = 0x06;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_X_XL  = 0x07;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_Y_XL  = 0x08;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_Z_XL  = 0x09;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_DUR_XL    = 0x0A;
static constexpr auto LSM9DS1_REGISTER_REFERENCE_G       = 0x0B;
static constexpr auto LSM9DS1_REGISTER_INT1_CTRL         = 0x0C;
static constexpr auto LSM9DS1_REGISTER_INT2_CTRL         = 0x0D;
static constexpr auto LSM9DS1_REGISTER_WHO_AM_I          = 0x0F;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG1_G       = 0x10;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG2_G       = 0x11;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG3_G       = 0x12;
static constexpr auto LSM9DS1_REGISTER_ORIENT_CFG_G      = 0x13;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_SRC_G     = 0x14;
static constexpr auto LSM9DS1_REGISTER_OUT_TEMP_L        = 0x15;
static constexpr auto LSM9DS1_REGISTER_OUT_TEMP_H        = 0x16;
static constexpr auto LSM9DS1_REGISTER_OUT_X_G_L         = 0x18;
static constexpr auto LSM9DS1_REGISTER_OUT_X_G_H         = 0x19;
static constexpr auto LSM9DS1_REGISTER_OUT_Y_G_L         = 0x1A;
static constexpr auto LSM9DS1_REGISTER_OUT_Y_G_H         = 0x1B;
static constexpr auto LSM9DS1_REGISTER_OUT_Z_G_L         = 0x1C;
static constexpr auto LSM9DS1_REGISTER_OUT_Z_G_H         = 0x1D;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG4         = 0x1E;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG5_XL      = 0x1F;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG6_XL      = 0x20;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG7_XL      = 0x21;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG8         = 0x22;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG9         = 0x23;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG10        = 0x24;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_SRC_XL    = 0x26;
static constexpr auto LSM9DS1_REGISTER_STATUS_REG        = 0x27;
static constexpr auto LSM9DS1_REGISTER_OUT_X_XL_L        = 0x28;
static constexpr auto LSM9DS1_REGISTER_OUT_X_XL_H        = 0x29;
static constexpr auto LSM9DS1_REGISTER_OUT_Y_XL_L        = 0x2A;
static constexpr auto LSM9DS1_REGISTER_OUT_Y_XL_H        = 0x2B;
static constexpr auto LSM9DS1_REGISTER_OUT_Z_XL_L        = 0x2C;
static constexpr auto LSM9DS1_REGISTER_OUT_Z_XL_H        = 0x2D;
static constexpr auto LSM9DS1_REGISTER_FIFO_CTRL         = 0x2E;
static constexpr auto LSM9DS1_REGISTER_FIFO_SRC          = 0x2F;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_CFG_G     = 0x30;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_X_G_L = 0x31;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_X_G_H = 0x32;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_Y_G_L = 0x33;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_Y_G_H = 0x34;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_Z_G_L = 0x35;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_THS_Z_G_H = 0x36;
static constexpr auto LSM9DS1_REGISTER_INT_GEN_DUR_G     = 0x37;

static constexpr auto LSM9DS1_REGISTER_OFFSET_X_REG_L_M = 0x05;
static constexpr auto LSM9DS1_REGISTER_OFFSET_X_REG_H_M = 0x06;
static constexpr auto LSM9DS1_REGISTER_OFFSET_Y_REG_L_M = 0x07;
static constexpr auto LSM9DS1_REGISTER_OFFSET_Y_REG_H_M = 0x08;
static constexpr auto LSM9DS1_REGISTER_OFFSET_Z_REG_L_M = 0x09;
static constexpr auto LSM9DS1_REGISTER_OFFSET_Z_REG_H_M = 0x0A;
static constexpr auto LSM9DS1_REGISTER_WHO_AM_I_M       = 0x0F;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG1_M      = 0x20;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG2_M      = 0x21;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG3_M      = 0x22;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG4_M      = 0x23;
static constexpr auto LSM9DS1_REGISTER_CTRL_REG5_M      = 0x24;
static constexpr auto LSM9DS1_REGISTER_STATUS_REG_M     = 0x27;
static constexpr auto LSM9DS1_REGISTER_OUT_X_L_M        = 0x28;
static constexpr auto LSM9DS1_REGISTER_OUT_X_H_M        = 0x29;
static constexpr auto LSM9DS1_REGISTER_OUT_Y_L_M        = 0x2A;
static constexpr auto LSM9DS1_REGISTER_OUT_Y_H_M        = 0x2B;
static constexpr auto LSM9DS1_REGISTER_OUT_Z_L_M        = 0x2C;
static constexpr auto LSM9DS1_REGISTER_OUT_Z_H_M        = 0x2D;
static constexpr auto LSM9DS1_REGISTER_INT_CFG_M        = 0x30;
static constexpr auto LSM9DS1_REGISTER_INT_SRC_M        = 0x31;
static constexpr auto LSM9DS1_REGISTER_INT_THS_L        = 0x32;
static constexpr auto LSM9DS1_REGISTER_INT_THS_H        = 0x33;

#endif // #ifndef LSM9DS1_REGISTERS_H
