/**
 *****************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_LPS25H LPS25H Functions
 * @brief Hardware functions to deal with the altitude pressure sensor
 * @{
 *
 * @file       pios_lps25h.h
 * @author     Giuseppe Barba <giuseppe.barba@gmail.com>
 * @brief      LPS25H Pressure Sensor Routines
 * @see        The GNU Public License (GPL) Version 3
 *
 ****************************************************************************/
#ifndef PIOS_LPS25H_H
#define PIOS_LPS25H_H
/**
 * Registers address definition
 */

/**
 * Reference pressure addresses
 * The LPS25H_REF_P_XL register contains the lowest part of the reference
 * pressure value that is sum to the sensor output pressure. The full reference
 * pressure value is composed by LPS25H_REF_P_XL, LPS25H_REF_P_H &
 * LPS25H_REF_P_L and is represented as 2’s complement.
 *
 * Reset Value = 0x0
 */
LPS25H_REF_P_XL			(0x08)
LPS25H_REF_P_L			(0x09)
LPS25H_REF_P_H			(0x0A)

LPS25H_WHO_AM_I			(0x0F)

/**
 * Pressure and Temperature internal average configuration.
 *
 * Reset Value = 0x05
 */
LPS25H_RES_CONF			(0x10)
LPS25H_RES_CONF_RESET		(0x05)
LPS25H_RES_CONF_TEMP_MASK	(0x03)
LPS25H_RES_CONF_TEMP_8_SAMP	(0x00)
LPS25H_RES_CONF_TEMP_16_SAMP	(0x01)
LPS25H_RES_CONF_TEMP_32_SAMP	(0x02)
LPS25H_RES_CONF_TEMP_64_SAMP	(0x03)
LPS25H_RES_CONF_PRES_MASK	(0x0C)
LPS25H_RES_CONF_PRES_8_SAMP	(0x00)
LPS25H_RES_CONF_PRES_32_SAMP	(0x04)
LPS25H_RES_CONF_PRES_128_SAMP	(0x08)
LPS25H_RES_CONF_PRES_512_SAMP	(0x0C)

LPS25H_CTRL_REG1		(0x20)
LPS25H_CTRL_REG1_PD_MASK	(0x80)
LPS25H_CTRL_REG1_ODR_MASK	(0x70)
LPS25H_CTRL_REG1_ODR_1HZ	(0x10)
LPS25H_CTRL_REG1_ODR_7HZ	(0x20)
LPS25H_CTRL_REG1_ODR_12_5HZ	(0x30)
LPS25H_CTRL_REG1_ODR_25HZ	(0x40)
LPS25H_CTRL_REG1_DIFF_EN_MASK	(0x08)
LPS25H_CTRL_REG1_BDU_MASK	(0x04)
LPS25H_CTRL_REG1_RESET_AZ_MASK	(0x02)
LPS25H_CTRL_REG1_SIM_MASK	(0x01)

LPS25H_CTRL_REG2		(0x21)
LPS25H_CTRL_REG2_BOOT_MASK	(0x80)
LPS25H_CTRL_REG2_FIFO_EN_MASK	(0x40)
LPS25H_CTRL_REG2_WTM_EN_MASK	(0x20)
LPS25H_CTRL_REG2_FIFO_MEAN_DEC_MASK	(0x10)
LPS25H_CTRL_REG2_SWRESET_MASK	(0x04)
LPS25H_CTRL_REG2_AUTO_ZERO_MASK	(0x02)
LPS25H_CTRL_REG2_ONE_SHOT_MASK	(0x01)

LPS25H_CTRL_REG3		(0x22)
LPS25H_CTRL_REG3_INT_H_L_MASK	(0x80)
LPS25H_CTRL_REG3_PP_OD_MASK	(0x40)
LPS25H_CTRL_REG3_INT1_S2_MASK	(0x02)
LPS25H_CTRL_REG3_INT1_S1_MASK	(0x01)

LPS25H_CTRL_REG4		(0x23)
LPS25H_CTRL_REG4_P1_EMPTY_MASK	(0x08)
LPS25H_CTRL_REG4_P1_WTM_MASK	(0x04)
LPS25H_CTRL_REG4_P1_OVRN_MASK	(0x02)
LPS25H_CTRL_REG4_P1_DRDY_MASK	(0x01)

LPS25H_INT_CFG			(0x24)
LPS25H_INT_CFG_LIR_MASK		(0x04)
LPS25H_INT_CFG_PL_E_MASK	(0x02)
LPS25H_INT_CFG_PH_E_MASK	(0x01)

LPS25H_INT_SOURCE		(0x25)
LPS25H_INT_SOURCE_IA_MASK	(0x04)
LPS25H_INT_SOURCE_PL_MASK	(0x02)
LPS25H_INT_SOURCE_PH_MASK	(0x01)

LPS25H_STATUS			(0x27)
LPS25H_STATUS_P_OR_MASK		(0x10)
LPS25H_STATUS_T_OR_MASK		(0x08)
LPS25H_STATUS_P_DA_MASK		(0x02)
LPS25H_STATUS_T_DA_MASK		(0x01)

/**
 * The LPS25H_PRESS_POUT_XL register contains the lowest part of the pressure
 * output value,that is the difference between the measured pressure and the
 * reference pressure (REF_P registers). The full reference pressure value is
 * composed by LPS25H_OUT_H/_L/_XL and is represented as 2’s complement.
 * Pressure Values exceeding the operating pressure Range are clipped.
 * Pressure output data: Pout(hPa) = LPS25H_OUT / 4096
 * Example: P_OUT = 0x3ED000 LSB = 4116480 LSB = 4116480/4096 hPa= 1005 hPa
 * Default Value is 0x2F800 = 760 hPa
 */
LPS25H_PRESS_POUT_XL		(0x28)
LPS25H_OUT_L			(0x29)
LPS25H_OUT_H			(0x2A)

/**
 * The LPS25H_TEMP_OUT_L register contains the low part of the temperature output
 * value.Temperature data are expressed as LPS25H_TEMP_OUT_H & LPS25H_TEMP_OUT_L
 * as 2’s complement numbers. Temperature output data:
 * T = 42.5 + (LPS25H_TEMP_OUT / 480)
 */
LPS25H_TEMP_OUT_L		(0x2B)
LPS25H_TEMP_OUT_H		(0x2C)

/**
 * The LPS25H_FIFO_CTRL registers allows to control the FIFO functionality.
 * FIFO_MEAN_MODE: The FIFO can be used for implementing a HW moving average
 * on the pressure measurements. The number of samples of the moving average
 * can be 2, 4, 8, 16 or 32 samples, by selecting the watermark levels.
 */
LPS25H_FIFO_CTRL		(0x2E)
LPS25H_FIFO_CTRL_MEAN_MODE	(0xC0)
LPS25H_FIFO_CTRL_2_SAMP_MASK	(0x01)
LPS25H_FIFO_CTRL_4_SAMP_MASK	(0x03)
LPS25H_FIFO_CTRL_8_SAMP_MASK	(0x07)
LPS25H_FIFO_CTRL_16_SAMP_MASK	(0x0F)
LPS25H_FIFO_CTRL_32_SAMP_MASK	(0x1F)

LPS25H_FIFO_STATUS		(0x2F)
LPS25H_FIFO_STATUS_WTM_MASK	(0x80)
LPS25H_FIFO_STATUS_FULL_MASK	(0x40)
LPS25H_FIFO_STATUS_EMPTY_MASK	(0x20)

/**
 * LPS25H_THS_P_L register contains the low part of threshold value for
 * pressure interrupt generation. The complete threshold value is given by
 * THS_P_H & THS_P_L and is expressed as unsigned number.
 * P_ths (hPa) = (THS_P)/16.
 */
LPS25H_THS_P_L			(0x30)
LPS25H_THS_P_H			(0x31)

/**
 * LPS25H_RPDS_L register contains the low part of the pressure offset value
 * after soldering,for differential pressure computing. The complete value is
 * given by LPS25H_RPDS_L & LPS25H_RPDS_H and is expressed as signed 2
 * complement value.
 */
LPS25H_RPDS_L			(0x39)
LPS25H_RPDS_H			(0x3A)

#define TEMPERATURE_OFFSET	42.5

/**
 * Configuration structure for the LPS25H driver
 */
struct pios_lps25h_cfg {
	uint32_t odr;
	uint32_t temp_average;
	uint32_t press_average;
};

/**
 * Public Functions
 */
int32_t PIOS_LPS25H_Init(uint32_t spi_id, uint32_t slave_num,
					const struct pios_lps25h_cfg *cfg);
bool PIOS_LIS3DSH_IRQHandler(void);

#endif /* PIOS_LPS25H_H */

/**
 * @}
 * @}
 */
