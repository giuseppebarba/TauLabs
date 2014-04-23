/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_LPS25H LPS25H Functions
 * @brief Hardware functions to deal with the altitude pressure sensor
 * @{
 *
 * @file       pios_lps25h.c
 * @author     Giuseppe Barba <giuseppe.barba@gmail.com>
 * @brief      LPS25H Pressure Sensor Routines
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************/
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

#if defined(PIOS_INCLUDE_LPS25H)

#include "pios_lps25h.h"

/* Private constants */
#define LPS25H_TASK_PRIORITY	(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)
#define LPS25H_TASK_STACK	(512 / 4)
#define LPS25H_AUTO_INCREMENT	0x80
#define	LPS25H_RES_MAX		(LPS25H_RES_CONF_PRES_512_SAMP | LPS25H_RES_CONF_TEMP_64_SAMP)
#define LPS25H_P0		101.3250f
#define PIOS_LPS25H_QUEUESIZE	1

/* Private methods */
static int32_t PIOS_LPS25H_Read(uint8_t address, uint8_t * buffer, uint8_t len);
static int32_t PIOS_LPS25H_WriteCommand(uint8_t command);
static void PIOS_LPS25H_Task(void *parameters);

/* Private types */

/* Local Types */

enum pios_lps25h_dev_magic {
	PIOS_LPS25H_DEV_MAGIC = 0xefba8e10,
};

struct lps25h_dev {
	const struct pios_lps25h_cfg *cfg;
	uint32_t spi_id;
	uint32_t slave_num;
	xQueueHandle queue;

	enum pios_lps25h_dev_magic magic;
};


static struct lps25h_dev *pios_lps25h_dev;

/**
 * @brief Writes one byte to the LPS25H
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int8_t PIOS_LPS25H_SetReg(uint8_t reg, uint8_t data)
{
	if (PIOS_LPS25H_ClaimBus() != 0)
		return -1;

	PIOS_SPI_TransferByte(pios_lps25h_dev->spi_id, 0x7f & reg);
	PIOS_SPI_TransferByte(pios_lps25h_dev->spi_id, data);

	PIOS_LPS25H_ReleaseBus();

	return 0;
}

/**
 * @brief Read a register from LPS25H from ISR context
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 * \param[in] task woken
 */
static int32_t PIOS_LPS25H_GetRegIsr(uint8_t reg, bool *woken)
{
	uint8_t data;

	if (PIOS_LPS25H_ClaimBusIsr(woken) != 0)
		return -1;

	PIOS_SPI_TransferByte(pios_lps25h_dev->spi_id, (0x80 | reg)); // request byte
	data = PIOS_SPI_TransferByte(pios_lps25h_dev->spi_id, 0);     // receive response

	PIOS_LPS25H_ReleaseBusIsr(woken);

	return data;
}

/**
 * @brief Read a register from LPS25H
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static uint8_t PIOS_LPS25H_GetReg(uint8_t reg)
{
	uint8_t data;

	if (PIOS_LPS25H_ClaimBus() != 0)
		return -1;

	PIOS_SPI_TransferByte(pios_lps25h_dev->spi_id, (0x80 | reg)); // request byte
	data = PIOS_SPI_TransferByte(pios_lps25h_dev->spi_id, 0);     // receive response

	PIOS_LPS25H_ReleaseBus();

	return data;
}

/**
 * @brief Allocate a new device
 */
static struct lps25h_dev * PIOS_LPS25H_alloc(void)
{
	struct lps25h_dev *lps25h_dev;

	lps25h_dev = (struct lps25h_dev *)pvPortMalloc(sizeof(*lps25h_dev));
	if (!lps25h_dev)
		return (NULL);

	memset(lps25h_dev, 0, sizeof(*lps25h_dev));

	lps25h_dev->queue = xQueueCreate(PIOS_LPS25H_QUEUESIZE,
					 sizeof(struct pios_sensor_baro_data));
	if (lps25h_dev->queue == NULL) {
		vPortFree(lps25h_dev);
		return NULL;
	}

	lps25h_dev->magic = PIOS_LPS25H_DEV_MAGIC;

	return lps25h_dev;
}

/**
 * Initialise the LPS25H sensor
 */
int32_t PIOS_LPS25H_Init(uint32_t spi_id, uint32_t slave_num,
					const struct pios_lps25h_cfg *cfg)
{
	pios_lps25h_dev = (struct lps25h_dev *)PIOS_LPS25H_alloc();
	if (pios_lps25h_dev == NULL)
		return -1;

	pios_lps25h_dev->spi_id = spi_id;
	pios_lps25h_dev->slave_num = slave_num;

	pios_lps25h_dev->cfg = cfg;

	/** Reset LPS25H sensor */
	if (PIOS_LPS25H_SetReg(LPS25H_CTRL_REG2, LPS25H_CTRL_REG2_BOOT_MASK) != 0)
		return -1;

	/** Give chip some time to initialize */
	PIOS_DELAY_WaitmS(50);
	PIOS_WDG_Clear();

	if (PIOS_LPS25H_SetReg(LPS25H_CTRL_REG1,
			LPS25H_CTRL_REG1_PD_MASK |
			(cfg->odr & LPS25H_CTRL_REG1_ODR_MASK) |
			LPS25H_CTRL_REG1_DIFF_EN_MASK |
			LPS25H_CTRL_REG1_BDU_MASK) != 0)
		return -1;

	/**
	 * Use suggested configuration for pressure and temperature average
	 */
	if (PIOS_LPS25H_SetReg(LPS25H_RES_CONF, LPS25H_RES_CONF_RESET) != 0)
		return -1;

	if (PIOS_LPS25H_SetReg(LPS25H_FIFO_CTRL,
			LPS25H_FIFO_CTRL_MEAN_MODE) != 0)
		return -1;

	if (PIOS_LPS25H_SetReg(LPS25H_CTRL_REG2,
			LPS25H_CTRL_REG2_FIFO_EN_MASK) != 0)
		return -1;

	/**
	 * Enable Data Ready signal on INT1 pin
	 */
	if (PIOS_LPS25H_SetReg(LPS25H_CTRL_REG4,
			LPS25H_CTRL_REG4_P1_DRDY_MASK) != 0)
		return -1;

	return 0;
}

/**
 * @brief Claim the SPI bus for the baro communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_LPS25H_ClaimBus(void)
{
	if (PIOS_SPI_ClaimBus(pios_lps25h_dev->spi_id) != 0)
		return -2;

	PIOS_SPI_RC_PinSet(pios_lps25h_dev->spi_id, pios_lps25h_dev->slave_num, 0);

	return 0;
}

/**
 * @brief Claim the SPI bus for the baro communications and select this chip
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_LPS25H_ClaimBusIsr(bool *woken)
{
	if (PIOS_SPI_ClaimBusISR(pios_lis3dsh_dev->spi_id, woken) < 0)
		return -2;

	PIOS_SPI_RC_PinSet(pios_lis3dsh_dev->spi_id, pios_lis3dsh_dev->slave_num, 0);

	return 0;
}

/**
 * @brief Release the SPI bus for the baro communications and end the transaction
 * @return 0 if successful
 */
static int32_t PIOS_LPS25H_ReleaseBus(void)
{
	PIOS_SPI_RC_PinSet(pios_lps25h_dev->spi_id, pios_lps25h_dev->slave_num, 1);

	return PIOS_SPI_ReleaseBus(pios_lps25h_dev->spi_id);
}

/**
* Reads one or more bytes into a buffer
* \param[in] the command indicating the address to read
* \param[out] buffer destination buffer
* \param[in] len number of bytes which should be read
* \return 0 if operation was successful
* \return -1 if failed to claim SPI bus
* \return -2 if error during SPI transfer
*/
static int32_t PIOS_LPS25H_Read(uint8_t address, uint8_t *buffer, uint8_t len)
{
	if (PIOS_LPS25H_ClaimBus() != 0)
		return -1;

	if (PIOS_SPI_TransferByte(pios_lps25h_dev->spi_id, address) < 0) {
		PIOS_LPS25H_ReleaseBus();
		return -2;
	}

	if (PIOS_SPI_TransferBlock(pios_lps25h_dev->spi_id, NULL, buffer, len, NULL) < 0) {
		PIOS_LPS25H_ReleaseBus();
		return -2;
	}
	PIOS_LPS25H_ReleaseBus();

	return 0;
}

/**
* Read raw temperature and pressure data
* \return 0 if successfully read the ADC, -1 if failed
*/
static int32_t PIOS_LPS25H_ReadData(int32_t *temp, int32_t *press)
{
	uint8_t data[5];

	if (PIOS_LPS25H_Read(LPS25H_PRESS_POUT_XL | LPS25H_AUTO_INCREMENT,
			data, 5) != 0)
		return -1;

	(*press) = (int32_t)((((int8_t) data[2]) << 16) | (data[1] <<  8) |
					(data[0]));
	(*temp) = (int32_t)((((int8_t) data[4]) << 8) | (data[3]));

	return 0;
}

/**
* @brief IRQ Handler.
*/
bool PIOS_LPS25H_IRQHandler(void)
{
	bool woken = false;
	int32_t raw_temp, raw_press;
	struct pios_sensor_baro_data data;
	portBASE_TYPE xHigherPriorityTaskWoken;

	if (PIOS_LPS25H_ClaimBusIsr(&woken) != 0)
		return woken;

	if (PIOS_LPS25H_ReadData(&raw_temp, &raw_press) != 0) {
		return -1;
	}

	PIOS_LIS3DSH_ReleaseBusIsr(&woken);

	data.temperature = TEMPERATURE_OFFSET ((float) raw_temp) / 480.0f
	data.pressure = ((float) raw_press) / 4096.0f;
	data.altitude = 44330.8f * (1.0f -
			powf(data.pressure / LPS25H_P0, (1.0f / 5.255f)));

	xQueueSendToBackFromISR(pios_lps25h_dev->queue, (void *)&data,
						&xHigherPriorityTaskWoken);

	return (woken || (xHigherPriorityTaskWoken == pdTRUE));
}


#endif

/**
 * @}
 * @}
 */
