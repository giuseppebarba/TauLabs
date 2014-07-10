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
/* Project Includes */
#include "pios.h"
#include "physical_constants.h"

#if defined(PIOS_INCLUDE_LSM9DS1)

#include "pios_lsm9ds1.h"

/* Private constants */
#define LSM9DS1_TASK_PRIORITY		(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)
#define LSM9DS1_TASK_STACK		(512 / 4)
#define PIOS_LSM9DS1_MAX_DOWNSAMPLE	1

/* Global Variables */
enum pios_lsm9ds1_dev_magic {
	PIOS_LSM9DS1_DEV_MAGIC = 0xf132d4a62,
};

struct lsm9ds1_dev {
#if defined(LSM9DS1_USE_SPI)
	uint32_t spi_id;
	uint32_t AxG_slave_num;
	uint32_t Mag_slave_num;
#else
	/** Use I2C serial interface */
	uint32_t i2c_id;
	uint8_t i2c_addr_ax_g;
	uint8_t i2c_addr_mag;
#endif
	enum pios_lsm9ds1_fs_xl accel_fs;
	enum pios_lsm9ds1_fs_g gyro_fs;
	enum pios_lsm9ds1_fs_m mag_fs;

	xQueueHandle gyro_queue;
	xQueueHandle accel_queue;
	xQueueHandle mag_queue;

	xTaskHandle TaskHandle;
	xSemaphoreHandle data_ready_sema;
	const struct pios_lsm9ds1_cfg *cfg;
	enum pios_lsm9ds1_dev_magic magic;

	float accel_odr_hz;
	float gyro_odr_hz;
	float mag_odr_hz;
};

//! Global structure for this device device
static struct lsm9ds1_dev *dev;

static int32_t PIOS_LSM9DS1_Validate(struct lsm9ds1_dev *dev);

#if defined(LSM9DS1_USE_SPI)
/**
 * @brief Claim the SPI bus for the communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_LSM9DS1_ClaimBus(uint32_t slave_num)
{
	if (PIOS_LSM9DS1_Validate(dev) != 0)
		return -1;

	if (PIOS_SPI_ClaimBus(dev->spi_id) != 0)
		return -2;

	PIOS_SPI_RC_PinSet(dev->spi_id, slave_num, 0);

	return 0;
}

/**
 * @brief Release the SPI bus for the communications and end the transaction
 * \param[in] must be true when bus was claimed in lowspeed mode
 * @return 0 if successful
 */
static int32_t PIOS_LSM9DS1_ReleaseBus(uint32_t slave_num)
{
	if (PIOS_LSM9DS1_Validate(dev) != 0)
		return -1;

	PIOS_SPI_RC_PinSet(dev->spi_id, slave_num, 1);

	return PIOS_SPI_ReleaseBus(dev->spi_id);
}

/**
 * @brief Read a register from LSM9DS1
 * @returns The register value
 * @param reg Register address to be read
 * @param *buffer Buffer to store data
 * @param len Number of bytes to be read
 * @return 0 if success
 */
static uint8_t PIOS_LSM9DS1_Read_SPI(uint32_t slave_num, uint8_t reg,
												uint8_t *buffer, uint8_t len)
{
	uint8_t ret;

	if (PIOS_LSM9DS1_ClaimBus(slave_num) != 0)
		return -1;

	if (PIOS_SPI_TransferByte(dev->spi_id, 0x80 | reg) < 0) {
		ret = -2;
		goto out;
	}
	if (PIOS_SPI_TransferBlock(dev->spi_id, NULL, buffer, len, NULL) < 0) {
		ret = -3;
		goto out;
	}

	ret = 0;
out:
	PIOS_LSM9DS1_ReleaseBus(slave_num);

	return ret;
}

/**
 * @brief Writes one byte to the LSM9DS1 register
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * @returns 0 when success
 */
static int32_t PIOS_LSM9DS1_Write_SPI(uint32_t slave_num, uint8_t reg, uint8_t data)
{
	uint8_t ret;

	if (PIOS_LSM9DS1_ClaimBus(slave_num) != 0)
		return -1;

	if (PIOS_SPI_TransferByte(dev->spi_id, 0x7F | reg) < 0) {
		ret = -2;
		goto out;
	}
	PIOS_SPI_TransferByte(dev->spi_id, data);

	PIOS_LSM9DS1_ReleaseBus(slave_num);

	ret = 0;
out:
	return ret;
}

#else

/**
 * @brief Writes one or more bytes to Acc and Gyr of the LSM9DS1
 * \param[in] address Register address
 * \param[in] buffer source buffer
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LSM9DS1_Write_i2c(uint8_t i2c_addr, uint8_t address,
				      uint8_t buffer)
{
	uint8_t data[] = {
		address,
		buffer,
	};

	const struct pios_i2c_txn txn_list[] = {
		{
		 .info = __func__,
		 .addr = i2c_addr,
		 .rw = PIOS_I2C_TXN_WRITE,
		 .len = sizeof(data),
		 .buf = data,
		 },
	};

	return PIOS_I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
 * @brief Reads one or more bytes into a buffer
 * \param[in] address LSM9DS1 register address (depends on size)
 * \param[out] buffer destination buffer
 * \param[in] len number of bytes which should be read
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LSM9DS1_Read_I2C(uint8_t i2c_addr, uint8_t address,
				     uint8_t * buffer, uint8_t len)
{
	uint8_t addr_buffer[] = {
		address,
	};

	const struct pios_i2c_txn txn_list[] = {
		{
		 .info = __func__,
		 .addr = i2c_addr,
		 .rw = PIOS_I2C_TXN_WRITE,
		 .len = sizeof(addr_buffer),
		 .buf = addr_buffer,
		 }
		,
		{
		 .info = __func__,
		 .addr = i2c_addr,
		 .rw = PIOS_I2C_TXN_READ,
		 .len = len,
		 .buf = buffer,
		 }
	};

	return PIOS_I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}
#endif

/**
 * @brief Writes one byte to Acc and Gyro
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LSM9DS1_Write_AxG(uint8_t reg, uint8_t data)
{
#if defined(LSM9DS1_USE_SPI)
	return PIOS_LSM9DS1_Write_SPI(dev->AxG_slave_num, reg, data);
#else
	return PIOS_LSM9DS1_Write_i2c(dev->i2c_addr_ax_g, reg, data);
#endif
}

/**
 * @brief Writes one byte to Mag
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LSM9DS1_Write_Mag(uint8_t reg, uint8_t data)
{
#if defined(LSM9DS1_USE_SPI)
	return PIOS_LSM9DS1_Write_SPI(dev->Mag_slave_num, reg, data);
#else
	return PIOS_LSM9DS1_Write_i2c(dev->i2c_addr_mag, reg, data);
#endif
}

/**
 * @brief Read data from Accelerometer or Gyroscope registers
 * @returns 0 if succesfull read the data or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t PIOS_LSM9DS1_Read_AxG(uint8_t reg, uint8_t * buffer, uint8_t len)
{
#if defined(LSM9DS1_USE_SPI)
	return PIOS_LSM9DS1_Read_SPI(dev->AxG_slave_num, reg, buffer, len);
#else
	return PIOS_LSM9DS1_Read_I2C(dev->i2c_addr_ax_g, reg, buffer, len);
#endif
}

/**
 * @brief Read data from Magnetometer
 * @returns 0 if succesfull read the data or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t PIOS_LSM9DS1_Read_Mag(uint8_t reg, uint8_t * buffer, uint8_t len)
{
#if defined(LSM9DS1_USE_SPI)
	return PIOS_LSM9DS1_Read_SPI(dev->Mag_slave_num, reg, buffer, len);
#else
	return PIOS_LSM9DS1_Read_I2C(dev->i2c_addr_mag, reg, buffer, len);
#endif
}

/**
 * Set the gyro range and store it locally for scaling
 */
int32_t PIOS_LSM9DS1_SetGyroRange(enum pios_lsm9ds1_fs_g gyro_fs)
{
	int32_t ret = -1;
	uint8_t reg = 0x0;

	ret = PIOS_LSM9DS1_Read_AxG(LSM9DS1_CTRL_REG1_G, &reg, 1);
	if (ret < 0)
			return ret;

	reg = (reg & ~LSM9DS1_CTRL_REG1_G_FS_MASK) | (gyro_fs & LSM9DS1_CTRL_REG1_G_FS_MASK);
	ret = PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG1_G, reg);
	if (ret < 0)
		return ret;

	switch (gyro_fs) {
	case LSM9DS1_G_FS_245_DPS:
		PIOS_SENSORS_SetMaxGyro(245);
		break;
	case LSM9DS1_G_FS_500_DPS:
		PIOS_SENSORS_SetMaxGyro(500);
		break;
	case LSM9DS1_G_FS_2000_DPS:
		PIOS_SENSORS_SetMaxGyro(2000);
		break;
	}

	dev->gyro_fs = gyro_fs;
	return 0;
}

/**
 * Set the acc range and store it locally for scaling
 */
int32_t PIOS_LSM9DS1_SetAccRange(enum pios_lsm9ds1_fs_xl accel_fs)
{
	int32_t ret = -1;
	uint8_t reg = 0x0;

	ret = PIOS_LSM9DS1_Read_AxG(LSM9DS1_CTRL_REG6_XL, &reg, 1);
	if (ret < 0)
			return ret;

	reg = (reg & ~LSM9DS1_CTRL_REG6_XL_FS_MASK) | (accel_fs & LSM9DS1_CTRL_REG6_XL_FS_MASK);
	ret = PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG6_XL, reg);
	if (ret < 0)
		return ret;

	dev->accel_fs = accel_fs;
	return 0;
}

/**
 * Set the mag range and store it locally for scaling
 */
int32_t PIOS_LSM9DS1_SetMagRange(enum pios_lsm9ds1_fs_m mag_fs)
{
	int32_t ret = -1;
	uint8_t reg = 0x0;

	ret = PIOS_LSM9DS1_Read_Mag(LSM9DS1_CTRL_REG2_M, &reg, 1);
	if (ret < 0)
			return ret;

	reg = (reg & ~LSM9DS1_CTRL_REG2_M_FS_MASK) | (mag_fs & LSM9DS1_CTRL_REG2_M_FS_MASK);

	ret = PIOS_LSM9DS1_Write_Mag(LSM9DS1_CTRL_REG2_M, reg);
	if (ret < 0)
		return ret;

	dev->mag_fs = mag_fs;
	return 0;
}

/**
 * Set gyro odr
 */
int32_t PIOS_LSM9DS1_SetGyroODR(enum pios_lsm9ds1_odr_g gyro_odr)
{
	int32_t ret = -1;
	uint8_t reg = 0x0;

	ret = PIOS_LSM9DS1_Read_AxG(LSM9DS1_CTRL_REG1_G, &reg, 1);
	if (ret < 0)
		return ret;

	reg = (reg & ~LSM9DS1_CTRL_REG1_G_ODR_MASK) | (gyro_odr & LSM9DS1_CTRL_REG1_G_ODR_MASK);

	ret = PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG1_G, reg);
	if (ret < 0)
		return ret;

	switch(gyro_odr) {
		case LSM9DS1_G_ODR_14_9_HZ:
			dev->gyro_odr_hz = 14.9f;
			break;
		case LSM9DS1_G_ODR_59_5_HZ:
			dev->gyro_odr_hz = 59.5f;
			break;
		case LSM9DS1_G_ODR_119_HZ:
			dev->gyro_odr_hz = 119.0f;
			break;
		case LSM9DS1_G_ODR_238_HZ:
			dev->gyro_odr_hz = 238.0f;
			break;
		case LSM9DS1_G_ODR_476_HZ:
			dev->gyro_odr_hz = 476.0f;
			break;
		case LSM9DS1_G_ODR_952_HZ:
			dev->gyro_odr_hz = 952.0f;
			break;
		default:
		case LSM9DS1_G_ODR_OFF:
			dev->gyro_odr_hz = 0.0f;
			break;
	}
	return 0;
}

/**
 * Set Acc odr
 */
int32_t PIOS_LSM9DS1_SetAccODR(enum pios_lsm9ds1_odr_xl accel_odr)
{
	int32_t ret = -1;
	uint8_t reg = 0x0;

	ret = PIOS_LSM9DS1_Read_AxG(LSM9DS1_CTRL_REG6_XL, &reg, 1);
	if (ret < 0)
		return ret;

	reg = (reg & ~LSM9DS1_CTRL_REG6_XL_ODR_MASK) | (accel_odr & LSM9DS1_CTRL_REG6_XL_ODR_MASK);

	ret = PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG6_XL, reg);
	if (ret < 0)
		return ret;

	switch(accel_odr) {
		case LSM9DS1_XL_ODR_10_HZ:
			dev->gyro_odr_hz = 10.0f;
			break;
		case LSM9DS1_XL_ODR_50_HZ:
			dev->gyro_odr_hz = 50.0f;
			break;
		case LSM9DS1_XL_ODR_119_HZ:
			dev->gyro_odr_hz = 119.0f;
			break;
		case LSM9DS1_XL_ODR_238_HZ:
			dev->gyro_odr_hz = 238.0f;
			break;
		case LSM9DS1_XL_ODR_476_HZ:
			dev->gyro_odr_hz = 476.0f;
			break;
		case LSM9DS1_XL_ODR_952_HZ:
			dev->gyro_odr_hz = 952.0f;
			break;
		default:
		case LSM9DS1_XL_ODR_OFF:
			dev->gyro_odr_hz = 0.0f;
			break;
	}
	return 0;
}

/**
 * Set Mag odr
 */
int32_t PIOS_LSM9DS1_SetMagODR(enum pios_lsm9ds1_odr_m mag_odr)
{
	int32_t ret = -1;
	uint8_t reg = 0x0;

	ret = PIOS_LSM9DS1_Read_Mag(LSM9DS1_CTRL_REG1_M, &reg, 1);
	if (ret < 0)
		return ret;

	reg = (reg & ~LSM9DS1_CTRL_REG1_M_ODR_MASK) | (mag_odr & LSM9DS1_CTRL_REG1_M_ODR_MASK);

	ret = PIOS_LSM9DS1_Write_Mag(LSM9DS1_CTRL_REG1_M, reg);
	if (ret < 0)
		return ret;

	switch(mag_odr) {
		case LSM9DS1_M_ODR_0_625_HZ:
			dev->mag_odr_hz = 0.625f;
			break;
		case LSM9DS1_M_ODR_1_25_HZ:
			dev->mag_odr_hz = 1.25f;
			break;
		case LSM9DS1_M_ODR_2_5_HZ:
			dev->mag_odr_hz = 2.5f;
			break;
		case LSM9DS1_M_ODR_5_HZ:
			dev->mag_odr_hz = 5.0f;
			break;
		case LSM9DS1_M_ODR_10_HZ:
			dev->mag_odr_hz = 10.0f;
			break;
		case LSM9DS1_M_ODR_20_HZ:
			dev->mag_odr_hz = 20.0f;
			break;
		case LSM9DS1_M_ODR_40_HZ:
			dev->mag_odr_hz = 40.0f;
			break;
		case LSM9DS1_M_ODR_80_HZ:
			dev->mag_odr_hz = 80.0f;
			break;
		default:
			dev->mag_odr_hz = 0.0f;
			break;
	}
	return 0;
}

/**
 * @brief Allocate a new device
 */
static struct lsm9ds1_dev *PIOS_LSM9DS1_alloc(void)
{
	struct lsm9ds1_dev *lsm9ds1_dev;

	lsm9ds1_dev = (struct lsm9ds1_dev *)PIOS_malloc(sizeof(*lsm9ds1_dev));
	if (!lsm9ds1_dev)
		return (NULL);

	lsm9ds1_dev->magic = PIOS_LSM9DS1_DEV_MAGIC;

	lsm9ds1_dev->accel_queue = xQueueCreate(PIOS_LSM9DS1_MAX_DOWNSAMPLE,
						sizeof(struct
						       pios_sensor_gyro_data));
	if (lsm9ds1_dev->accel_queue == NULL) {
		vPortFree(lsm9ds1_dev);
		return NULL;
	}

	lsm9ds1_dev->gyro_queue = xQueueCreate(PIOS_LSM9DS1_MAX_DOWNSAMPLE,
					       sizeof(struct
						      pios_sensor_gyro_data));
	if (lsm9ds1_dev->gyro_queue == NULL) {
		vPortFree(lsm9ds1_dev);
		return NULL;
	}

	lsm9ds1_dev->mag_queue = xQueueCreate(PIOS_LSM9DS1_MAX_DOWNSAMPLE,
					      sizeof(struct
						     pios_sensor_mag_data));
	if (lsm9ds1_dev->mag_queue == NULL) {
		vPortFree(lsm9ds1_dev);
		return NULL;
	}

	lsm9ds1_dev->data_ready_sema = xSemaphoreCreateMutex();
	if (lsm9ds1_dev->data_ready_sema == NULL) {
		vPortFree(lsm9ds1_dev);
		return NULL;
	}

	return (lsm9ds1_dev);
}

/**
 * @brief Validate the handle to the device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t PIOS_LSM9DS1_Validate(struct lsm9ds1_dev *dev)
{
	if (dev == NULL)
		return -1;
	if (dev->magic != PIOS_LSM9DS1_DEV_MAGIC)
		return -2;
#if defined(LSM9DS1_USE_SPI)
	if (dev->spi_id == 0)
		return -3;
#else
	if (dev->i2c_id == 0)
		return -3;
#endif
	return 0;
}

/**
 * @brief Configure the interrupt source
 * \return 0 for valid transaction, negative value for i2c error
*
*/
static int32_t PIOS_LSM9DS1_Config_Interrupt()
{
	/**
	 * Use the higher speed sensor as interrupt source
	 */
	if (dev->gyro_odr_hz >= dev->accel_odr_hz)
		return PIOS_LSM9DS1_Write_AxG(LSM9DS1_INT1_CTRL,
				LSM9DS1_INT1_IG_G | LSM9DS1_INT1_DRDY_G);
	else
		return PIOS_LSM9DS1_Write_AxG(LSM9DS1_INT1_CTRL,
				LSM9DS1_INT1_IG_XL | LSM9DS1_INT1_DRDY_XL);
}

/**
 * @brief Initialize the LSM9DS1 sensor
 * \return none
 * \param[in] PIOS_LSM9DS1_ConfigTypeDef struct to be used to configure sensor.
*
*/
static int32_t PIOS_LSM9DS1_Config(struct pios_lsm9ds1_cfg const *cfg)
{
	PIOS_LSM9DS1_SetGyroODR(cfg->gyro_odr);
	PIOS_LSM9DS1_SetAccODR(cfg->accel_odr);
	PIOS_LSM9DS1_SetMagODR(cfg->mag_odr);

	PIOS_LSM9DS1_SetGyroRange(cfg->gyro_fs);
	PIOS_LSM9DS1_SetAccRange(cfg->accel_fs);
	PIOS_LSM9DS1_SetMagRange(cfg->mag_fs);

	/**
	 * Configure hardware filters chain
	 */
	PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG2_G,
				LSM9DS1_CTRL_REG2_G_OUT_SEL_LPF2 |
				LSM9DS1_CTRL_REG2_G_INT_SEL_LPF2);
	PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG3_G,
				LSM9DS1_CTRL_REG3_G_HP_EN |
				LSM9DS1_CTRL_REG3_G_HPCF_4);
	PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG7_XL,
				LSM9DS1_CTRL_REG7_XL_HR |
				LSM9DS1_CTRL_REG7_XL_DCF1 |
				LSM9DS1_CTRL_REG7_XL_FDS |
				LSM9DS1_CTRL_REG7_XL_HPIS1);

	PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG8,
				LSM9DS1_CTRL_REG8_BDU |
				LSM9DS1_CTRL_REG8_IF_ADD_INC);

	PIOS_LSM9DS1_Write_Mag(LSM9DS1_CTRL_REG1_M,
				LSM9DS1_CTRL_REG1_M_UHP |
				LSM9DS1_CTRL_REG1_M_TEMP_COMP);

	PIOS_LSM9DS1_Write_Mag(LSM9DS1_CTRL_REG5_M,
				LSM9DS1_CTRL_REG5_M_BDU);

	/**
	 * Interrupt configuration
	 */
	/**
	 * Gyroscope's axis output enable
	 */
	PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG4,
				LSM9DS1_CTRL_REG4_ZEN_G |
				LSM9DS1_CTRL_REG4_YEN_G |
				LSM9DS1_CTRL_REG4_XEN_G);

	/**
	 * Accelerometer's axis output enable
	 */
	PIOS_LSM9DS1_Write_AxG(LSM9DS1_CTRL_REG5_XL,
				LSM9DS1_CTRL_REG5_ZEN_XL |
				LSM9DS1_CTRL_REG5_YEN_XL |
				LSM9DS1_CTRL_REG5_XEN_XL);

	PIOS_LSM9DS1_Write_Mag(LSM9DS1_CTRL_REG3_M,
				LSM9DS1_CTRL_REG3_M_CONT);

	PIOS_LSM9DS1_Config_Interrupt();

	return 0;
}

/**
 * @brief Get Gyroscope sensitivity related to the selected full scale
 * @return dps/LSB sensitivity factor, 0 on failure
 */
static float PIOS_LSM9DS1_GetGyroScale()
{
	switch (dev->gyro_fs) {
	case LSM9DS1_G_FS_245_DPS:
		return 8.75f / 1000.0f;
	case LSM9DS1_G_FS_500_DPS:
		return 17.5f / 1000.0f;
	case LSM9DS1_G_FS_2000_DPS:
		return 70.0f / 1000.0f;
	}
	return 0;
}

/**
 * @brief Get Accelerometer sensitivity related to the selected full scale
 * @return mg/LSB sensitivity factor, 0 on failure
 */
static float PIOS_LSM9DS1_GetAccScale()
{
	switch (dev->accel_fs) {
	case LSM9DS1_XL_FS_2_G:
		return (0.061 * 9.81 / 1000.0);
	case LSM9DS1_XL_FS_4_G:
		return (0.122 * 9.81 / 1000.0);
	case LSM9DS1_XL_FS_8_G:
		return (0.244 * 9.81 / 1000.0);
	}
	return 0;
}
#if 1
/**
 * @brief Get Magnetometer sensitivity related to the selected full scale
 * @return mgauss/LSB sensitivity factor, 0 on failure
 */
static float PIOS_LSM9DS1_GetMagScale()
{
	switch (dev->mag_fs) {
	case LSM9DS1_M_FS_4_G:
		return 0.14f;
	case LSM9DS1_M_FS_8_G:
		return 0.29f;
	case LSM9DS1_M_FS_12_G:
		return 0.43f;
	case LSM9DS1_M_FS_16_G:
		return 0.58f;
	}
	return 0;
}
#endif
/**
 * Check if an LSM9DS1 is detected at the requested address
 * @return 1 if detected, -1 if successfully probed but wrong id
 *  -2 no device at address
 */
#if defined(LSM9DS1_USE_SPI)
uint8_t PIOS_LSM9DS1_Probe()
{
	//if (PIOS_LSM9DS1_Read_AxG(addr, read, 1) < 0)
	//	return -1;
	return 1;
#else
uint8_t PIOS_LSM9DS1_Probe(uint32_t i2c_id, uint8_t i2c_addr)
{
	uint8_t read = 0;
	uint8_t addr = LSM9DS1_WHO_AM_I;

	const struct pios_i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = i2c_addr,
			.rw = PIOS_I2C_TXN_WRITE,
			.len = 1,
			.buf = &addr,
		},
		{
			.info = __func__,
			.addr = i2c_addr,
			.rw = PIOS_I2C_TXN_READ,
			.len = 1,
			.buf = &read,
		}
	};

	if (PIOS_I2C_Transfer(i2c_id, txn_list, NELEMENTS(txn_list)) < 0)
		return -1;

	return (read != LSM9DS1_WHO_AM_I_VAL);
#endif
}

/**
* @brief IRQ Handler.  Read all the data from onboard buffer
*/
bool PIOS_LSM9DS1_IRQHandler(void)
{
	if (PIOS_LSM9DS1_Validate(dev) != 0)
		return false;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(dev->data_ready_sema, &xHigherPriorityTaskWoken);
	return xHigherPriorityTaskWoken == pdTRUE;
}

static void PIOS_LSM9DS1_Task(void *parameters)
{
	uint8_t rec_buf[6];
	struct pios_sensor_accel_data accel_data;
	struct pios_sensor_gyro_data gyro_data;
	struct pios_sensor_mag_data mag_data;
	float sensitivity = 0;
	float temperature = 0;

	while (1) {
		//Wait for data ready interrupt
		if (xSemaphoreTake(dev->data_ready_sema, portMAX_DELAY) !=
		    pdTRUE)
			continue;

		if (PIOS_LSM9DS1_Read_AxG(LSM9DS1_OUT_TEMP_L, rec_buf, 2) < 0) {
			continue;
		}

		temperature = ((int16_t) ((rec_buf[1] << 8) | rec_buf[0])) / 16.0f + 25.0f;


		if (PIOS_LSM9DS1_Read_AxG(LSM9DS1_OUT_X_L_G, rec_buf, sizeof(rec_buf)) < 0) {
			continue;
		}

		sensitivity = PIOS_LSM9DS1_GetGyroScale();
		gyro_data.x =
		    ((int16_t) ((rec_buf[1] << 8) | rec_buf[0])) * sensitivity;
		gyro_data.y =
		    ((int16_t) ((rec_buf[3] << 8) | rec_buf[2])) * sensitivity;
		gyro_data.z =
		    ((int16_t) ((rec_buf[5] << 8) | rec_buf[4])) * sensitivity;
		gyro_data.temperature = temperature;

		if (PIOS_LSM9DS1_Read_AxG(LSM9DS1_OUT_X_L_XL, rec_buf, sizeof(rec_buf)) < 0) {
			continue;
		}

		sensitivity = PIOS_LSM9DS1_GetAccScale();
		accel_data.x =
		    ((int16_t) ((rec_buf[1] << 8) | rec_buf[0])) * sensitivity;
		accel_data.y =
		    ((int16_t) ((rec_buf[3] << 8) | rec_buf[2])) * sensitivity;
		accel_data.z =
		    ((int16_t) ((rec_buf[5] << 8) | rec_buf[4])) * sensitivity;
		accel_data.temperature = temperature;
		xQueueSendToBack(dev->accel_queue, (void *)&accel_data, 0);
		xQueueSendToBack(dev->gyro_queue, (void *)&gyro_data, 0);

		if (PIOS_LSM9DS1_Read_Mag(LSM9DS1_STATUS_REG_M, rec_buf, 1) < 0) {
			continue;
		}

		if ((rec_buf[0] & (LSM9DS1_STATUS_REG_M_XDA | LSM9DS1_STATUS_REG_M_YDA | LSM9DS1_STATUS_REG_M_ZDA)) != 0) {

			if (PIOS_LSM9DS1_Read_Mag(LSM9DS1_OUT_X_L_M, rec_buf, sizeof(rec_buf)) < 0) {
				continue;
			}
			sensitivity = PIOS_LSM9DS1_GetMagScale();

			mag_data.x =
			    ((int16_t) ((rec_buf[1] << 8) | rec_buf[0])) *
			    sensitivity;
			mag_data.y =
			    ((int16_t) ((rec_buf[3] << 8) | rec_buf[2])) *
			    sensitivity;
			mag_data.z =
			    ((int16_t) ((rec_buf[5] << 8) | rec_buf[4])) *
			    sensitivity;
			xQueueSendToBack(dev->mag_queue, (void *)&mag_data, 0);
		}
	}
}

/**
 * @brief Initialize the LSM9DS1 sensor over I2C interface.
 * @return 0 for success, -1 for failure to allocate, -2 for failure to get irq
 */
#if defined(LSM9DS1_USE_SPI)
int32_t PIOS_LSM9DS1_Init(uint32_t spi_id, uint32_t AxG_slave_num,
					uint32_t Mag_slave_num, const struct pios_lsm9ds1_cfg * cfg)
#else
int32_t PIOS_LSM9DS1_Init(uint32_t i2c_id, const struct pios_lsm9ds1_cfg * cfg)
#endif
{
	dev = PIOS_LSM9DS1_alloc();
	if (dev == NULL)
		return -1;

#if defined(LSM9DS1_USE_SPI)
	dev->spi_id = spi_id;
	dev->AxG_slave_num = AxG_slave_num;
	dev->Mag_slave_num = Mag_slave_num;
#else
	dev->i2c_id = i2c_id;
	dev->i2c_addr_ax_g = cfg->i2c_addr_ax_g;
	dev->i2c_addr_mag = cfg->i2c_addr_mag;
#endif
	dev->cfg = cfg;
	dev->gyro_odr_hz = 0.0f;
	dev->accel_odr_hz = 0.0f;
	dev->mag_odr_hz = 0.0f;

	/* Configure Sensors */
	if (PIOS_LSM9DS1_Config(cfg) != 0)
		return -2;

	int result = xTaskCreate(PIOS_LSM9DS1_Task,
				 (const signed char *)"PIOS_LSM9DS1_Task",
				 LSM9DS1_TASK_STACK, NULL,
				 LSM9DS1_TASK_PRIORITY,
				 &dev->TaskHandle);

	PIOS_Assert(result == pdPASS);

	/* Set up EXTI line */
		PIOS_EXTI_Init(cfg->exti_cfg);

	PIOS_SENSORS_Register(PIOS_SENSOR_ACCEL, dev->accel_queue);
	PIOS_SENSORS_Register(PIOS_SENSOR_GYRO, dev->gyro_queue);
	PIOS_SENSORS_Register(PIOS_SENSOR_MAG, dev->mag_queue);

	return 0;
}
#endif
