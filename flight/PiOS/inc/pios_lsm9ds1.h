/**
*****************************************************************************
* @addtogroup PIOS PIOS Core hardware abstraction layer
* @{
* @addtogroup PIOS_LSM9DS1 LSM9DS1 Functions
* @brief Hardware functions to deal with the ST 9-axis inertial module.
* @{
*
* @file pios_lsm9ds1.h
* @author Giuseppe Barba <giuseppe.barba@gmail.com>
* @brief LSM9DS1 iNEMO inertial module
* @see The GNU Public License (GPL) Version 3
*
****************************************************************************/
#ifndef PIOS_LSM9DS1_H
#define PIOS_LSM9DS1_H
/**
* Registers address definition
*/

/** Accelerometer and gyroscope registers */
#define LSM9DS1_ACT_THS					0x04
#define LSM9DS1_ACT_DUR					0x05
#define LSM9DS1_INT_GEN_CFG_XL				0x06
#define LSM9DS1_INT_GEN_THS_X_XL			0x07
#define LSM9DS1_INT_GEN_THS_Y_XL			0x08
#define LSM9DS1_INT_GEN_THS_Z_XL			0x09
#define LSM9DS1_INT_GEN_DUR_XL				0x0A
#define LSM9DS1_REFERENCE_G				0x0B
#define LSM9DS1_INT1_CTRL				0x0C
#define LSM9DS1_INT2_CTRL				0x0D
#define LSM9DS1_WHO_AM_I				0x0F
#define LSM9DS1_WHO_AM_I_VAL				0x68

#define LSM9DS1_CTRL_REG1_G				0x10
#define LSM9DS1_CTRL_REG2_G				0x11
#define LSM9DS1_CTRL_REG3_G				0x12
#define LSM9DS1_ORIENT_CFG_G				0x13
#define LSM9DS1_INT_GEN_SRC_G				0x14
#define LSM9DS1_OUT_TEMP_L				0x15
#define LSM9DS1_OUT_TEMP_H				0x16
#define LSM9DS1_STATUS_REG				0x17
#define LSM9DS1_OUT_X_L_G				0x18
#define LSM9DS1_OUT_X_H_G				0x19
#define LSM9DS1_OUT_Y_L_G				0x1A
#define LSM9DS1_OUT_Y_H_G				0x1B
#define LSM9DS1_OUT_Z_L_G				0x1C
#define LSM9DS1_OUT_Z_H_G				0x1D
#define LSM9DS1_CTRL_REG4				0x1E
#define LSM9DS1_CTRL_REG5_XL				0x1F
#define LSM9DS1_CTRL_REG6_XL				0x20
#define LSM9DS1_CTRL_REG7_XL				0x21
#define LSM9DS1_CTRL_REG8				0x22
#define LSM9DS1_CTRL_REG9				0x23
#define LSM9DS1_CTRL_REG10				0x24
#define LSM9DS1_INT_GEN_SRC_XL				0x25
#define LSM9DS1_STATUS_REG2				0x27
#define LSM9DS1_OUT_X_L_XL				0x28
#define LSM9DS1_OUT_X_H_XL				0x29
#define LSM9DS1_OUT_Y_L_XL				0x2A
#define LSM9DS1_OUT_Y_H_XL				0x2B
#define LSM9DS1_OUT_Z_L_XL				0x2C
#define LSM9DS1_OUT_Z_H_XL				0x2D
#define LSM9DS1_FIFO_CTRL				0x2E
#define LSM9DS1_FIFO_SRC				0x2F
#define LSM9DS1_INT_GEN_CFG_G				0x30
#define LSM9DS1_INT_GEN_THS_XH_G			0x31
#define LSM9DS1_INT_GEN_THS_XL_G			0x32
#define LSM9DS1_INT_GEN_THS_YH_G			0x33
#define LSM9DS1_INT_GEN_THS_YL_G			0x34
#define LSM9DS1_INT_GEN_THS_ZH_G			0x35
#define LSM9DS1_INT_GEN_THS_ZL_G			0x36
#define LSM9DS1_INT_GEN_DUR_G				0x37

/** Magnetometer registers */
#define LSM9DS1_OFFSET_X_REG_L_M			0x05
#define LSM9DS1_OFFSET_X_REG_H_M			0x06
#define LSM9DS1_OFFSET_Y_REG_L_M			0x07
#define LSM9DS1_OFFSET_Y_REG_H_M			0x08
#define LSM9DS1_OFFSET_Z_REG_L_M			0x09
#define LSM9DS1_OFFSET_Z_REG_H_M			0x0A
#define LSM9DS1_WHO_AM_I_M				0x0F
#define LSM9DS1_CTRL_REG1_M				0x20
#define LSM9DS1_CTRL_REG2_M				0x21
#define LSM9DS1_CTRL_REG3_M				0x22
#define LSM9DS1_CTRL_REG4_M				0x23
#define LSM9DS1_CTRL_REG5_M				0x24
#define LSM9DS1_STATUS_REG_M				0x27
#define LSM9DS1_OUT_X_L_M				0x28
#define LSM9DS1_OUT_X_H_M				0x29
#define LSM9DS1_OUT_Y_L_M				0x2A
#define LSM9DS1_OUT_Y_H_M				0x2B
#define LSM9DS1_OUT_Z_L_M				0x2C
#define LSM9DS1_OUT_Z_H_M				0x2D
#define LSM9DS1_INT_CFG_M				0x30
#define LSM9DS1_INT_SRC_M				0x31
#define LSM9DS1_INT_THS_L_M				0x32
#define LSM9DS1_INT_THS_H_M				0x33

enum pios_lsm9ds1_odr_g {
	LSM9DS1_G_ODR_OFF = 0x00 LSM9DS1_G_ODR_14_9_HZ = 0x20,
	LSM9DS1_G_ODR_59_5_HZ = 0x40,
	LSM9DS1_G_ODR_119_HZ = 0x60,
	LSM9DS1_G_ODR_238_HZ = 0x80,
	LSM9DS1_G_ODR_476_HZ = 0xA0,
	LSM9DS1_G_ODR_952_HZ = 0xC0
};

enum pios_lsm9ds1_fs_g {
	LSM9DS1_G_FS_245_DPS = 0x00,
	LSM9DS1_G_FS_500_DPS = 0x08,
	LSM9DS1_G_FS_2000_DPS = 0x18
};

enum pios_lsm9ds1_odr_xl {
	LSM9DS1_XL_ODR_OFF = 0x00,
	LSM9DS1_XL_ODR_10_HZ = 0x20,
	LSM9DS1_XL_ODR_50_HZ = 0x40,
	LSM9DS1_XL_ODR_119_HZ = 0x60,
	LSM9DS1_XL_ODR_238_HZ = 0x80,
	LSM9DS1_XL_ODR_476_HZ = 0xA0,
	LSM9DS1_XL_ODR_952_HZ = 0xC0
};

enum pios_lsm9ds1_fs_xl {
	LSM9DS1_XL_FS_2_G = 0x00,
	LSM9DS1_XL_FS_4_G = 0x08,
	LSM9DS1_XL_FS_8_G = 0x18
};

enum pios_lsm9ds1_bw_xl {
	LSM9DS1_XL_BW_408_HZ = 0x00,
	LSM9DS1_XL_BW_211_HZ = 0x01,
	LSM9DS1_XL_BW_105_HZ = 0x02,
	LSM9DS1_XL_BW_50_HZ = 0x03,
};

enum pios_lsm9ds1_odr_m {
	LSM9DS1_M_ODR_0_625_HZ = 0x00,
	LSM9DS1_M_ODR_1_25_HZ = 0x04,
	LSM9DS1_M_ODR_2_5_HZ = 0x08,
	LSM9DS1_M_ODR_5_HZ = 0x0C,
	LSM9DS1_M_ODR_10_HZ = 0x10,
	LSM9DS1_M_ODR_20_HZ = 0x14,
	LSM9DS1_M_ODR_40_HZ = 0x18,
	LSM9DS1_M_ODR_80_HZ = 0x1C
};

enum pios_lsm9ds1_op_mode_m {
	LSM9DS1_M_OM_LP = 0x00,
	LSM9DS1_M_OM_MP = 0x20,
	LSM9DS1_M_OM_HP = 0x40,
	LSM9DS1_M_OM_UHP = 0x60
};

enum pios_lsm9ds1_fs_m {
	LSM9DS1_M_FS_4_G = 0x00,
	LSM9DS1_M_FS_8_G = 0x20,
	LSM9DS1_M_FS_12_G = 0x40,
	LSM9DS1_M_FS_16_G = 0x60
};

enum pios_lsm9ds1_ctrl_reg1 {
	LSM9DS1_CTRL_REG4_ZEN_G = 0x20,
	LSM9DS1_CTRL_REG4_YEN_G = 0x10,
	LSM9DS1_CTRL_REG4_XEN_G = 0x08,
	LSM9DS1_CTRL_REG4_LIR_XL1 = 0x02
};

enum pios_lsm9ds1_int1 {
	LSM9DS1_INT1_IG_G = 0x80,
	LSM9DS1_INT1_IG_XL = 0x40,
	LSM9DS1_INT1_DRDY_G = 0x02,
	LSM9DS1_INT1_DRDY_XL = 0x01
};

enum pios_lsm9ds1_int2 {
	LSM9DS1_INT2_DRDY_TEMP = 0x04,
	LSM9DS1_INT2_DRDY_G = 0x02,
	LSM9DS1_INT2_DRDY_XL = 0x01
};

enum pios_lsm9ds1_ctrl_reg5 {
	LSM9DS1_CTRL_REG5_ZEN_XL = 0x20,
	LSM9DS1_CTRL_REG5_YEN_XL = 0x10,
	LSM9DS1_CTRL_REG5_XEN_XL = 0x08
};

enum pios_lsm9ds1_ctrl_reg8 {
	LSM9DS1_CTRL_REG8_BOOT = 0x80,
	LSM9DS1_CTRL_REG8_BDU = 0x40,
	LSM9DS1_CTRL_REG8_H_LACTIVE = 0x20,
	LSM9DS1_CTRL_REG8_PP_OD = 0x10,
	LSM9DS1_CTRL_REG8_SIM = 0x08,
	LSM9DS1_CTRL_REG8_IF_ADD_INC = 0x04,
	LSM9DS1_CTRL_REG8_BLE = 0x02,
	LSM9DS1_CTRL_REG8_SW_RESET = 0x01
};

enum pios_lsm9ds1_int_cfg_m {
	LSM9DS1_INT_CFG_M_IEA = 0x04,
	LSM9DS1_INT_CFG_M_IEL = 0x02,
	LSM9DS1_INT_CFG_M_IEN = 0x01
};

#endif
