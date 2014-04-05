/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_LIS3DSH LIS3DSH Functions
 * @brief Deals with the hardware interface to the 3-axis gyro
 * @{
 *
 * @file       PIOS_LIS3DSH.h
 * @author     Giuseppe Barba <giuseppe.barba@gmail.com>
 * @brief      LIS3DSH 3-axis gyor function headers
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef PIOS_LIS3DSH_H
#define PIOS_LIS3DSH_H

#include "pios.h"

/* LIS3DSH Addresses */
#define PIOS_LIS3DSH_INFO1			0X0D
#define PIOS_LIS3DSH_INFO2			0X0E
#define PIOS_LIS3DSH_WHOAMI			0x0F
#define PIOS_LIS3DSH_WHOAMI_VAL			0x3F
#define PIOS_LIS3DSH_CTRL_REG3			0X23
#define PIOS_LIS3DSH_CTRL_REG4			0X20
#define PIOS_LIS3DSH_CTRL_REG5			0X24
#define PIOS_LIS3DSH_CTRL_REG6			0X25
#define PIOS_LIS3DSH_STATUS			0x27
#define PIOS_LIS3DSH_OUT_T			0x0C
#define PIOS_LIS3DSH_OFF_X			0x10
#define PIOS_LIS3DSH_OFF_Y			0x11
#define PIOS_LIS3DSH_OFF_Z			0x12
#define PIOS_LIS3DSH_CS_X			0x13
#define PIOS_LIS3DSH_CS_Y			0x14
#define PIOS_LIS3DSH_CS_Z			0x15
#define PIOS_LIS3DSH_LC_L			0x16
#define PIOS_LIS3DSH_LC_H			0x17
#define PIOS_LIS3DSH_STAT			0x18
#define PIOS_LIS3DSH_VFC_1			0x1B
#define PIOS_LIS3DSH_VFC_2			0x1C
#define PIOS_LIS3DSH_VFC_3			0x1D
#define PIOS_LIS3DSH_VFC_4			0x1E
#define PIOS_LIS3DSH_THRS3			0x1F
#define PIOS_LIS3DSH_OUT_X_L			0x28
#define PIOS_LIS3DSH_OUT_X_H			0x29
#define PIOS_LIS3DSH_OUT_Y_L			0x2A
#define PIOS_LIS3DSH_OUT_Y_H			0x2B
#define PIOS_LIS3DSH_OUT_Z_L			0x2C
#define PIOS_LIS3DSH_OUT_Z_H			0x2D
#define PIOS_LIS3DSH_OUT_X_L			0x28
#define PIOS_LIS3DSH_FIFO_CTRL			0x2E
#define PIOS_LIS3DSH_FIFO_SRC			0x2F
#define PIOS_LIS3DSH_CTRL_REG1			0x21
#define PIOS_LIS3DSH_ST1_X			0x40
#define PIOS_LIS3DSH_TIM4_1			0x50
#define PIOS_LIS3DSH_TIM3_1			0x51
#define PIOS_LIS3DSH_TIM2_1			0x52
#define PIOS_LIS3DSH_TIM1_1			0x54
#define PIOS_LIS3DSH_THRS2_1			0x56
#define PIOS_LIS3DSH_THRS1_1			0x57
#define PIOS_LIS3DSH_MASK1_B			0x59
#define PIOS_LIS3DSH_MASK1_A			0x5A
#define PIOS_LIS3DSH_SETT1			0x5B
#define PIOS_LIS3DSH_PR1			0x5C
#define PIOS_LIS3DSH_TC1			0x5D
#define PIOS_LIS3DSH_OUTS1			0x5F
#define PIOS_LIS3DSH_PEAK1			0x19
#define PIOS_LIS3DSH_CTRL_REG2			0x22
#define PIOS_LIS3DSH_ST2_X			0x60
#define PIOS_LIS3DSH_TIM4_2			0x70
#define PIOS_LIS3DSH_TIM3_2			0x71
#define PIOS_LIS3DSH_TIM2_2			0x72
#define PIOS_LIS3DSH_TIM1_2			0x74
#define PIOS_LIS3DSH_THRS2_2			0x76
#define PIOS_LIS3DSH_THRS1_2			0x77
#define PIOS_LIS3DSH_MASK2_B			0x79
#define PIOS_LIS3DSH_MASK2_A			0x7A
#define PIOS_LIS3DSH_SETT2			0x7B
#define PIOS_LIS3DSH_PR2			0x7C
#define PIOS_LIS3DSH_TC2			0x7D
#define PIOS_LIS3DSH_OUTS2			0x7F
#define PIOS_LIS3DSH_PEAK2			0x1A
#define PIOS_LIS3DSH_DES2			0x78

/* Ctrl3 flags */
#define PIOS_LIS3DSH_CTRL3_DR_EN		0x80
#define PIOS_LIS3DSH_CTRL3_IEA			0x40
#define PIOS_LIS3DSH_CTRL3_IEL			0x20
#define PIOS_LIS3DSH_CTRL3_INT2_EN		0x10
#define PIOS_LIS3DSH_CTRL3_INT1_EN		0x08

/* Ctrl4 flags */
#define PIOS_LIS3DSH_CTRL4_BDU			0x08
#define PIOS_LIS3DSH_CTRL4_ZEN			0x04
#define PIOS_LIS3DSH_CTRL4_YEN			0x02
#define PIOS_LIS3DSH_CTRL4_XEN			0x01
#define PIOS_LIS3DSH_CTRL4_PDOWN		0x00
#define PIOS_LIS3DSH_CTRL4_3HZ			0x10
#define PIOS_LIS3DSH_CTRL4_6HZ			0x20
#define PIOS_LIS3DSH_CTRL4_12HZ			0x30
#define PIOS_LIS3DSH_CTRL4_25HZ			0x40
#define PIOS_LIS3DSH_CTRL4_50HZ			0x50
#define PIOS_LIS3DSH_CTRL4_100HZ		0x60
#define PIOS_LIS3DSH_CTRL4_400HZ		0x70
#define PIOS_LIS3DSH_CTRL4_800HZ		0x80
#define PIOS_LIS3DSH_CTRL4_1600HZ		0x90

/* Ctrl5 flags */
#define PIOS_LIS3DSH_CTRL5_BW_50HZ		0xC0
#define PIOS_LIS3DSH_CTRL5_BW_200HZ		0x80
#define PIOS_LIS3DSH_CTRL5_BW_400HZ		0x40
#define PIOS_LIS3DSH_CTRL5_BW_800HZ		0x00
#define PIOS_LIS3DSH_CTRL5_SPI_4WIRE	0x00
#define PIOS_LIS3DSH_CTRL5_SPI_3WIRE	0x01

/* Ctrl6 flags */
#define PIOS_LIS3DSH_CTRL6_BOOT			0x80
#define PIOS_LIS3DSH_CTRL6_FIFO_EN		0x40
#define PIOS_LIS3DSH_CTRL6_ADD_INC		0x10



enum pios_lis3dsh_range {
	PIOS_LIS3DSH_ACCEL_2G = 0x00,
	PIOS_LIS3DSH_ACCEL_4G = 0x08,
	PIOS_LIS3DSH_ACCEL_6G = 0x10,
	PIOS_LIS3DSH_ACCEL_8G = 0x18,
	PIOS_LIS3DSH_ACCEL_16G = 0x20,
};


struct pios_lis3dsh_cfg {
	const struct pios_exti_cfg * exti_cfg;
	enum pios_lis3dsh_range range;
};

/* Public Functions */
extern int8_t PIOS_LIS3DSH_Init(uint32_t spi_id, uint32_t slave_num,
					const struct pios_lis3dsh_cfg * cfg);
extern int8_t PIOS_LIS3DSH_SetRange(enum pios_lis3dsh_range range);
extern int8_t PIOS_LIS3DSH_ReadID();
extern int8_t PIOS_LIS3DSH_Test();
extern bool PIOS_LIS3DSH_IRQHandler();

#endif /* PIOS_LIS3DSH_H */

/**
  * @}
  * @}
  */
