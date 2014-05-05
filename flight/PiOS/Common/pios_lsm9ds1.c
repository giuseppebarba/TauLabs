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
	uint32_t slave_num;
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
};

//! Global structure for this device device
static struct lsm9ds1_dev *dev;

static struct lsm9ds1_dev *PIOS_LSM9DS1_alloc(void);
static int32_t PIOS_LSM9DS1_Write_AxG(uint8_t address, uint8_t buffer);

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
		 }
		,
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

/**
 * @brief Writes one byte to Acc and Gyro
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LSM9DS1_SetReg_AxG(uint8_t reg, uint8_t data)
{
#if !defined(LSM9DS1_USE_SPI)
	return PIOS_LSM9DS1_Write(dev->i2c_addr_ax_g, reg, data);
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
static int32_t PIOS_LSM9DS1_SetReg_Mag(uint8_t reg, uint8_t data)
{
#if !defined(LSM9DS1_USE_SPI)
	return PIOS_LSM9DS1_Write(dev->i2c_addr_mag, reg, data);
#endif
}

/**
 * @brief Read a register from LSM9DS1
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t PIOS_MPU9150_GetReg_AxG(uint8_t reg)
{
	uint8_t data;
	int32_t retval;

#if !defined(LSM9DS1_USE_SPI)
	retval =
	    PIOS_LSM9DS1_Read_I2C(dev->i2c_addr_ax_g, reg, &data, sizeof(data));
#endif
	if (retval != 0)
		return retval;
	else
		return data;
}

/**
 * Set the gyro range and store it locally for scaling
 */
int32_t PIOS_LSM9DS1_SetGyroRange(enum pios_lsm9ds1_fs_g gyro_fs)
{
	PIOS_LSM9DS1_SetReg_AxG(LSM9DS1_CTRL_REG1_G, gyro_fs);

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
	PIOS_LSM9DS1_SetReg_AxG(LSM9DS1_CTRL_REG6_XL, accel_fs);
	dev->accel_fs = accel_fs;
	return 0;
}

/**
 * Set the mag range and store it locally for scaling
 */
int32_t PIOS_LSM9DS1_SetMagRange(enum pios_lsm9ds1_fs_m mag_fs)
{
	PIOS_LSM9DS1_SetReg_Mag(LSM9DS1_CTRL_REG2_M, mag_fs);
	dev->mag_fs = mag_fs;
	return 0;
}

/**
 * Set gyro odr
 */
int32_t PIOS_LSM9DS1_SetGyroODR(enum pios_lsm9ds1_odr_g odr)
{
	return PIOS_LSM9DS1_SetReg_AxG(LSM9DS1_CTRL_REG1_G, odr);
}

/**
 * Set Acc odr
 */
int32_t PIOS_LSM9DS1_SetAccODR(enum pios_lsm9ds1_odr_xl odr)
{
	return PIOS_LSM9DS1_SetReg_AxG(LSM9DS1_CTRL_REG6_XL, odr);
}

/**
 * Set Mag odr
 */
int32_t PIOS_LSM9DS1_SetMagODR(enum pios_lsm9ds1_odr_m odr)
{
	return PIOS_LSM9DS1_SetReg_Mag(LSM9DS1_CTRL_REG1_M, odr);
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
	 * Interrupt configuration
	 * Use Gyroscope as interrupt source
	 *
	 */
	PIOS_LSM9DS1_SetReg_AxG(cfg, LSM9DS1_INT1_CTRL,
				LSM9DS1_INT1_IG_G | LSM9DS1_INT1_DRDY_G);

	return 0;
}

#if !defined(LSM9DS1_USE_SPI)
/**
 * @brief Initialize the LSM9DS1 sensor over I2C interface.
 * @return 0 for success, -1 for failure to allocate, -2 for failure to get irq
 */
int32_t PIOS_LSM9DS1_Init(uint32_t i2c_id, const struct pios_lsm9ds1_cfg * cfg)
{
	dev = PIOS_MPU9150_alloc();
	if (dev == NULL)
		return -1;

	dev->i2c_id = i2c_id;
	dev->i2c_addr_ax_g = cfg->i2c_addr_ax_g;
	dev->i2c_addr_mag = cfg->i2c_add_mag;
	dev->cfg = cfg;

	/* Configure Sensors */
	if (PIOS_LSM9DS1_Config(cfg) != 0)
		return -2;

	/* Set up EXTI line */
	PIOS_EXTI_Init(cfg->exti_cfg);

	// Wait 5 ms for data ready interrupt and make sure it happens
	// twice
	if ((xSemaphoreTake(dev->data_ready_sema, 5) != pdTRUE) ||
	    (xSemaphoreTake(dev->data_ready_sema, 5) != pdTRUE)) {
		return -10;
	}

	int result = xTaskCreate(PIOS_LSM9DS1_Task,
				 (const signed char *)"PIOS_LSM9DS1_Task",
				 LSM9DS1_TASK_STACK, NULL,
				 LSM9DS1_TASK_PRIORITY,
				 &dev->TaskHandle);

	PIOS_Assert(result == pdPASS);

	PIOS_SENSORS_Register(PIOS_SENSOR_ACCEL, dev->accel_queue);
	PIOS_SENSORS_Register(PIOS_SENSOR_GYRO, dev->gyro_queue);
	PIOS_SENSORS_Register(PIOS_SENSOR_MAG, dev->mag_queue);

	return 0;
}
#endif

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
		return 61.0f;
	case LSM9DS1_XL_FS_4_G:
		return 122.0f;
	case LSM9DS1_XL_FS_8_G:
		return 244.0f;
	}
	return 0;
}

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

#if !defined(LSM9DS1_USE_SPI)
		if (PIOS_LSM9DS1_Read_I2C
		    (dev->i2c_addr_ax_g, LSM9DS1_OUT_TEMP_L, rec_buf, 2) < 0) {
			continue;
		}
#endif
		temperature =
		    ((int16_t) ((rec_buf[1] << 8) | rec_buf[0])) / 16.0f +
		    25.0f;

#if !defined(LSM9DS1_USE_SPI)
		if (PIOS_LSM9DS1_Read_I2C(dev->i2c_addr_ax_g, LSM9DS1_OUT_X_L_G,
					  rec_buf, sizeof(rec_buf)) < 0) {
			continue;
		}
#endif
		sensitivity = PIOS_LSM9DS1_GetGyroScale();
		gyro_data.x =
		    ((int16_t) ((rec_buf[1] << 8) | rec_buf[0])) * sensitivity;
		gyro_data.y =
		    ((int16_t) ((rec_buf[3] << 8) | rec_buf[2])) * sensitivity;
		gyro_data.z =
		    ((int16_t) ((rec_buf[5] << 8) | rec_buf[4])) * sensitivity;
		gyro_data.temperature = temperature
#if !defined(LSM9DS1_USE_SPI)
		    if (PIOS_LSM9DS1_Read_I2C
			(dev->i2c_addr_ax_g, LSM9DS1_OUT_X_L_XL, rec_buf,
			 sizeof(rec_buf)) < 0) {
			continue;
		}
#endif
		sensitivity = PIOS_LSM9DS1_GetAccScale();
		accel_data.x =
		    ((int16_t) ((rec_buf[1] << 8) | rec_buf[0])) * sensitivity;
		accel_data.y =
		    ((int16_t) ((rec_buf[3] << 8) | rec_buf[2])) * sensitivity;
		accel_data.z =
		    ((int16_t) ((rec_buf[5] << 8) | rec_buf[4])) * sensitivity;
		accel_data.temperature =
		    temperature xQueueSendToBack(dev->accel_queue,
						 (void *)&accel_data, 0);
		xQueueSendToBack(dev->gyro_queue, (void *)&gyro_data, 0);

#if !defined(LSM9DS1_USE_SPI)
		if (PIOS_LSM9DS1_Read_I2C
		    (dev->i2c_addr_mag, LSM9DS1_STATUS_REG_M, rec_buf, 1) < 0) {
			continue;
		}
#endif
		if ((rec_buf[0] &
		     (LSM9DS1_STATUS_REG_M_XDA | LSM9DS1_STATUS_REG_M_YDA |
		      LSM9DS1_STATUS_REG_M_ZDA)) != 0) {
#if !defined(LSM9DS1_USE_SPI)
			if (PIOS_LSM9DS1_Read_I2C
			    (dev->i2c_addr_mag, LSM9DS1_STATUS_REG_M, rec_buf,
			     sizeof(rec_buf)) < 0) {
				continue;
			}
#endif
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
#endif
