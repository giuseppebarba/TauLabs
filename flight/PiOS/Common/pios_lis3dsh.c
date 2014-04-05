/**
 *****************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_LIS3DSH LIS3DSH Functions
 * @brief Deals with the hardware interface to the 3-axis gyro
 * @{
 *
 * @file       pios_LIS3DSH.c
 * @author     Giuseppe Barba <giuseppe.barba@gmail.com>
 * @brief      LIS3DSH 3-axis gyro chip
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************
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

/* Project Includes */
#include "pios.h"
#include "physical_constants.h"

#if defined(PIOS_INCLUDE_LIS3DSH)

#include "fifo_buffer.h"

/* Global Variables */
enum pios_lis3dsh_dev_magic {
    PIOS_LIS3DSH_DEV_MAGIC = 0x9d39bced,
};

#define PIOS_LIS3DSH_QUEUESIZE 2

//! Local types
struct lis3dsh_dev {
	uint32_t spi_id;
	uint32_t slave_num;
	xQueueHandle queue;
	const struct pios_lis3dsh_cfg *cfg;
	enum pios_lis3dsh_range range;
	enum pios_lis3dsh_dev_magic magic;
	volatile bool configured;
};

struct pios_lis3dsh_data {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
};

//! Global structure for this device device
static struct lis3dsh_dev *pios_lis3dsh_dev;

//! Private functions
static struct lis3dsh_dev *PIOS_LIS3DSH_alloc(void);
static int32_t PIOS_LIS3DSH_Validate(struct lis3dsh_dev *dev);
static void PIOS_LIS3DSH_Config(const struct pios_lis3dsh_cfg *cfg);
static int8_t PIOS_LIS3DSH_SetReg(uint8_t address, uint8_t buffer);
static uint8_t PIOS_LIS3DSH_GetReg(uint8_t address);
static int32_t PIOS_LIS3DSH_GetRegIsr(uint8_t reg, bool *woken);
static int32_t PIOS_LIS3DSH_ClaimBus();
static int32_t PIOS_LIS3DSH_ClaimBusIsr();
static int32_t PIOS_LIS3DSH_ReleaseBus();
static int32_t PIOS_LIS3DSH_ReleaseBusIsr();

/* Local Variables */

/**
 * @brief Allocate a new device
 */
static struct lis3dsh_dev *PIOS_LIS3DSH_alloc(void)
{
	struct lis3dsh_dev *lis3dsh_dev;

	lis3dsh_dev = (struct lis3dsh_dev *)PIOS_malloc(sizeof(*lis3dsh_dev));

	if (!lis3dsh_dev)
		return (NULL);

	lis3dsh_dev->magic = PIOS_LIS3DSH_DEV_MAGIC;

	lis3dsh_dev->configured = false;

	lis3dsh_dev->queue = xQueueCreate(PIOS_LIS3DSH_QUEUESIZE,
					  sizeof(struct pios_sensor_accel_data));

	if (lis3dsh_dev->queue == NULL) {
		vPortFree(lis3dsh_dev);
		return NULL;
	}

	return lis3dsh_dev;
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t PIOS_LIS3DSH_Validate(struct lis3dsh_dev *dev)
{
	if (dev == NULL)
		return -1;

	if (dev->magic != PIOS_LIS3DSH_DEV_MAGIC)
		return -2;

	if (dev->spi_id == 0)
		return -3;

	return 0;
}

/**
 * @brief Initialize the LIS3DSH 3-axis gyro sensor.
 * @return none
 */
int8_t PIOS_LIS3DSH_Init(uint32_t spi_id, uint32_t slave_num,
					const struct pios_lis3dsh_cfg *cfg)
{
	pios_lis3dsh_dev = PIOS_LIS3DSH_alloc();

	if (pios_lis3dsh_dev == NULL)
		return -1;

	pios_lis3dsh_dev->spi_id = spi_id;
	pios_lis3dsh_dev->slave_num = slave_num;
	pios_lis3dsh_dev->cfg = cfg;

	/* Configure the LIS3DSH Sensor */
	PIOS_LIS3DSH_Config(cfg);

	/* Set up EXTI */
	PIOS_EXTI_Init(cfg->exti_cfg);

	// An initial read is needed to get it running
	//struct pios_lis3dsh_data data;
	//PIOS_LIS3DSH_ReadAccel(&data);

	PIOS_SENSORS_Register(PIOS_SENSOR_ACCEL, pios_lis3dsh_dev->queue);

	return 0;
}

/**
 * @brief Initialize the LIS3DSH 3-axis accel sensor
 * \return none
 * \param[in] PIOS_LIS3DSH_ConfigTypeDef struct to be used to configure sensor.
*
*/
static void PIOS_LIS3DSH_Config(const struct pios_lis3dsh_cfg *cfg)
{
	while (PIOS_LIS3DSH_SetReg(PIOS_LIS3DSH_CTRL_REG6,
		PIOS_LIS3DSH_CTRL6_ADD_INC) != 0);

	// This register enables the channels and sets the bandwidth
	while (PIOS_LIS3DSH_SetReg(PIOS_LIS3DSH_CTRL_REG4,
		PIOS_LIS3DSH_CTRL4_100HZ |
		PIOS_LIS3DSH_CTRL4_BDU |
		PIOS_LIS3DSH_CTRL4_ZEN |
		PIOS_LIS3DSH_CTRL4_YEN |
		PIOS_LIS3DSH_CTRL4_XEN) != 0);

	// Set int2 to go high on data ready
	while (PIOS_LIS3DSH_SetReg(PIOS_LIS3DSH_CTRL_REG3,
		PIOS_LIS3DSH_CTRL3_DR_EN |
		PIOS_LIS3DSH_CTRL3_IEA |
		PIOS_LIS3DSH_CTRL3_INT1_EN) != 0);

	while (PIOS_LIS3DSH_SetReg(PIOS_LIS3DSH_CTRL_REG5,
		PIOS_LIS3DSH_CTRL5_BW_50HZ) != 0);

	while (PIOS_LIS3DSH_SetRange(cfg->range) != 0);

	pios_lis3dsh_dev->configured = true;
}

/**
 * @brief Sets the maximum range of the LIS3DSH
 * @returns 0 for success, -1 for invalid device, -2 if unable to set register
 */
int8_t PIOS_LIS3DSH_SetRange(enum pios_lis3dsh_range range)
{
	if (PIOS_LIS3DSH_Validate(pios_lis3dsh_dev) != 0)
		return -1;

	pios_lis3dsh_dev->range = range;

	if (PIOS_LIS3DSH_SetReg(PIOS_LIS3DSH_CTRL_REG5,
						pios_lis3dsh_dev->range) != 0)
		return -2;

	return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_LIS3DSH_ClaimBus()
{
	if (PIOS_LIS3DSH_Validate(pios_lis3dsh_dev) != 0)
		return -1;

	if (PIOS_SPI_ClaimBus(pios_lis3dsh_dev->spi_id) != 0)
		return -2;

	PIOS_SPI_RC_PinSet(pios_lis3dsh_dev->spi_id,
						pios_lis3dsh_dev->slave_num, 0);

	return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_LIS3DSH_ClaimBusIsr(bool *woken)
{
	if (PIOS_LIS3DSH_Validate(pios_lis3dsh_dev) != 0)
		return -1;

	if (PIOS_SPI_ClaimBusISR(pios_lis3dsh_dev->spi_id, woken) < 0)
		return -2;

	PIOS_SPI_RC_PinSet(pios_lis3dsh_dev->spi_id,
						pios_lis3dsh_dev->slave_num, 0);

	return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the
 * transaction
 * @return 0 if successful, -1 for invalid device
 */
static int32_t PIOS_LIS3DSH_ReleaseBus()
{
	if (PIOS_LIS3DSH_Validate(pios_lis3dsh_dev) != 0)
		return -1;

	PIOS_SPI_RC_PinSet(pios_lis3dsh_dev->spi_id,
						pios_lis3dsh_dev->slave_num, 1);

	return PIOS_SPI_ReleaseBus(pios_lis3dsh_dev->spi_id);
}

/**
 * @brief Release the SPI bus for the accel communications and end the
 * transaction
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful, -1 for invalid device
 */
static int32_t PIOS_LIS3DSH_ReleaseBusIsr(bool *woken)
{
	if (PIOS_LIS3DSH_Validate(pios_lis3dsh_dev) != 0)
		return -1;

	PIOS_SPI_RC_PinSet(pios_lis3dsh_dev->spi_id,
						pios_lis3dsh_dev->slave_num, 1);

	return PIOS_SPI_ReleaseBusISR(pios_lis3dsh_dev->spi_id, woken);
}

/**
 * @brief Read a register from LIS3DSH
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static uint8_t PIOS_LIS3DSH_GetReg(uint8_t reg)
{
	uint8_t data;

	if (PIOS_LIS3DSH_ClaimBus() != 0)
		return -1;

	PIOS_SPI_TransferByte(pios_lis3dsh_dev->spi_id, (0x80 | reg));
	data = PIOS_SPI_TransferByte(pios_lis3dsh_dev->spi_id, 0);

	PIOS_LIS3DSH_ReleaseBus();

	return data;
}

/**
 * @brief Writes one byte to the LIS3DSH
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int8_t PIOS_LIS3DSH_SetReg(uint8_t reg, uint8_t data)
{
	if (PIOS_LIS3DSH_ClaimBus() != 0)
		return -1;

	PIOS_SPI_TransferByte(pios_lis3dsh_dev->spi_id, 0x7f & reg);
	PIOS_SPI_TransferByte(pios_lis3dsh_dev->spi_id, data);

	PIOS_LIS3DSH_ReleaseBus();

	return 0;
}

/**
 * @brief Read a register from LIS3DSH from ISR context
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 * \param[in] task woken
 */
static int32_t PIOS_LIS3DSH_GetRegIsr(uint8_t reg, bool *woken)
{
	uint8_t data;

	if (PIOS_LIS3DSH_ClaimBusIsr(woken) != 0)
		return -1;

	PIOS_SPI_TransferByte(pios_lis3dsh_dev->spi_id, (0x80 | reg));
	data = PIOS_SPI_TransferByte(pios_lis3dsh_dev->spi_id, 0);

	PIOS_LIS3DSH_ReleaseBusIsr(woken);

	return data;
}

/**
 * @brief Read the identification bytes from the LIS3DSH sensor
 * \return ID read from LIS3DSH or -1 if failure
*/
int8_t PIOS_LIS3DSH_ReadID()
{
	int32_t id = PIOS_LIS3DSH_GetReg(PIOS_LIS3DSH_WHOAMI);

	if (id < 0)
		return -1;

	return id;
}

static float PIOS_LIS3DSH_GetScale()
{
	if (PIOS_LIS3DSH_Validate(pios_lis3dsh_dev) != 0)
		return -1;

	switch (pios_lis3dsh_dev->range) {
	case PIOS_LIS3DSH_ACCEL_2G:
		return GRAVITY * 0.06 / (16 * 1000.0f);
	case PIOS_LIS3DSH_ACCEL_4G:
		return GRAVITY * 0.12 / (16 * 1000.0f);
	case PIOS_LIS3DSH_ACCEL_6G:
		return GRAVITY * 0.18 / (16 * 1000.0f);
	case PIOS_LIS3DSH_ACCEL_8G:
		return GRAVITY * 0.24 / (16 * 1000.0f);
	case PIOS_LIS3DSH_ACCEL_16G:
		return GRAVITY * 0.73 / (16 * 1000.0f);
	}

	return 0;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
int8_t PIOS_LIS3DSH_Test(void)
{
	int8_t id;

	if ((id = PIOS_LIS3DSH_ReadID()) < 0)
		return -1;

	if (id == PIOS_LIS3DSH_WHOAMI_VAL)
		return 0;

	return -2;
}

/**
* @brief IRQ Handler.  Read all the data from onboard buffer
*/
bool PIOS_LIS3DSH_IRQHandler(void)
{
	struct pios_lis3dsh_data data;
	bool woken = false;
	float scale;
	struct pios_sensor_accel_data normalized_data;

	uint8_t buf[7] = { PIOS_LIS3DSH_OUT_X_L | 0x80 | 0x40, 0, 0, 0, 0, 0, 0 };
	uint8_t rec[7];

	if (PIOS_LIS3DSH_ClaimBusIsr(&woken) != 0)
		return woken;

	if (PIOS_SPI_TransferBlock(pios_lis3dsh_dev->spi_id, &buf[0], &rec[0],
						sizeof(buf), NULL) < 0) {
		PIOS_LIS3DSH_ReleaseBus();
		data.accel_x = 0;
		data.accel_y = 0;
		data.accel_z = 0;
		return woken;
	}

	PIOS_LIS3DSH_ReleaseBusIsr(&woken);

	memcpy((uint8_t *)&data.accel_x, &rec[1], 6);

	scale = PIOS_LIS3DSH_GetScale();
	normalized_data.x = data.accel_x * scale;
	normalized_data.y = data.accel_y * scale;
	normalized_data.z = data.accel_z * scale;
	normalized_data.temperature = PIOS_LIS3DSH_GetRegIsr(PIOS_LIS3DSH_OUT_T,
									&woken);

	portBASE_TYPE xHigherPriorityTaskWoken;
	xQueueSendToBackFromISR(pios_lis3dsh_dev->queue,
			(void *)&normalized_data, &xHigherPriorityTaskWoken);

	return woken || (xHigherPriorityTaskWoken == pdTRUE);
}

#endif /* PIOS_INCLUDE_LIS3DSH */

/**
 * @}
 * @}
 */
