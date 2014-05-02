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
#define LSM9DS1_TASK_PRIORITY	(tskIDLE_PRIORITY + configMAX_PRIORITIES - 1)	// max priority
#define LSM9DS1_TASK_STACK		(512 / 4)

/* Global Variables */
enum pios_lsm9ds1_dev_magic {
	PIOS_LSM9DS1_DEV_MAGIC = 0xf132d4a62,
};

struct lsm9ds1_dev {
	uint32_t spi_id;
	uint32_t slave_num;
	enum pios_mpu60x0_accel_range accel_range;
	enum pios_mpu60x0_range gyro_range;
	xQueueHandle gyro_queue;
	xQueueHandle accel_queue;
	xQueueHandle mag_queue;
	xTaskHandle TaskHandle;
	xSemaphoreHandle data_ready_sema;
	const struct pios_lsm9ds1_cfg * cfg;
	enum pios_lsm9ds1_dev_magic magic;
};

//! Global structure for this device device
static struct mpu9150_dev * dev;

#endif