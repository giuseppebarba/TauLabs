/**
 ******************************************************************************
 * @addtogroup TauLabsTargets Tau Labs Targets
 * @{
 * @addtogroup FlyingF3 FlyingF3 support files
 * @{
 *
 * @file       pios_board.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @brief      The board specific initialization routines
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
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

/* Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */

#include "board_hw_defs.c"

#include <pios.h>
#include <openpilot.h>
#include <uavobjectsinit.h>
#include "hwdiscoveryf4.h"
#include "manualcontrolsettings.h"
#include "modulesettings.h"

/* One slot per selectable receiver group.
 *  eg. PWM, PPM, GCS, DSMMAINPORT, DSMFLEXIPORT, SBUS
 * NOTE: No slot in this map for NONE.
 */

#if defined(PIOS_INCLUDE_HMC5883)
#include "pios_hmc5883_priv.h"
static const struct pios_hmc5883_cfg pios_hmc5883_external_cfg = {
	.M_ODR = PIOS_HMC5883_ODR_75,
	.Meas_Conf = PIOS_HMC5883_MEASCONF_NORMAL,
	.Gain = PIOS_HMC5883_GAIN_1_9,
	.Mode = PIOS_HMC5883_MODE_SINGLE,
	.Default_Orientation = PIOS_HMC5883_TOP_0DEG,
};
#endif /* PIOS_INCLUDE_HMC5883 */

/**
 * Configuration for the MS5611 chip
 */
#if defined(PIOS_INCLUDE_MS5611)
#include "pios_ms5611_priv.h"
static const struct pios_ms5611_cfg pios_ms5611_cfg = {
	.oversampling = MS5611_OSR_1024,
	.temperature_interleaving = 1,
};
#endif /* PIOS_INCLUDE_MS5611 */

/**
 * Configuration for the LSM9DS1 chip
 */
#if defined(PIOS_INCLUDE_LSM9DS1)
#include "pios_lsm9ds1.h"
static const struct pios_exti_cfg pios_exti_lsm9ds1_cfg __exti_config = {
	.vector = PIOS_LSM9DS1_IRQHandler,
	.line = EXTI_Line11,
	.pin = {
		.gpio = GPIOD,
		.init = {
			.GPIO_Pin = GPIO_Pin_11,
			.GPIO_Speed = GPIO_Speed_100MHz,
			.GPIO_Mode = GPIO_Mode_IN,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd = GPIO_PuPd_NOPULL,
		},
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel = EXTI15_10_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
			.NVIC_IRQChannelSubPriority = 0,
			.NVIC_IRQChannelCmd = ENABLE,
		},
	},
	.exti = {
		.init = {
			.EXTI_Line = EXTI_Line11,
			.EXTI_Mode = EXTI_Mode_Interrupt,
			.EXTI_Trigger = EXTI_Trigger_Rising,
			.EXTI_LineCmd = ENABLE,
		},
	},
};

static const struct pios_lsm9ds1_cfg lsm9ds1_cfg = {
	.exti_cfg = &pios_exti_lsm9ds1_cfg,
	.i2c_addr_ax_g = LSM9DS1_I2C_AX_G_ADDR_L,
	.i2c_addr_mag = LSM9DS1_I2C_MAG_ADDR_L,
	.accel_fs = LSM9DS1_XL_FS_8_G,
	.gyro_fs = LSM9DS1_G_FS_500_DPS,
	.mag_fs = LSM9DS1_M_FS_8_G,
	.accel_odr = LSM9DS1_XL_ODR_476_HZ,
	.gyro_odr = LSM9DS1_G_ODR_476_HZ,
	.mag_odr = LSM9DS1_M_ODR_80_HZ
};
#endif /* PIOS_INCLUDE_LSM9DS1 */

/* One slot per selectable receiver group.
 *  eg. PWM, PPM, GCS, SPEKTRUM1, SPEKTRUM2, SBUS
 * NOTE: No slot in this map for NONE.
 */
uintptr_t pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE];

#define PIOS_COM_TELEM_RF_RX_BUF_LEN 512
#define PIOS_COM_TELEM_RF_TX_BUF_LEN 512

#define PIOS_COM_GPS_RX_BUF_LEN 32
#define PIOS_COM_GPS_TX_BUF_LEN 16

#define PIOS_COM_TELEM_USB_RX_BUF_LEN 65
#define PIOS_COM_TELEM_USB_TX_BUF_LEN 65

#define PIOS_COM_BRIDGE_RX_BUF_LEN 65
#define PIOS_COM_BRIDGE_TX_BUF_LEN 12

#define PIOS_COM_MAVLINK_TX_BUF_LEN 128

#define PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN 128

#define PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN 19

#define PIOS_COM_PICOC_RX_BUF_LEN 128
#define PIOS_COM_PICOC_TX_BUF_LEN 128

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
#define PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN 40
uintptr_t pios_com_debug_id;
#endif /* PIOS_INCLUDE_DEBUG_CONSOLE */

uintptr_t pios_com_gps_id;
uintptr_t pios_com_telem_usb_id;
uintptr_t pios_com_telem_rf_id;
uintptr_t pios_com_vcp_id;
uintptr_t pios_com_bridge_id;
uintptr_t pios_com_mavlink_id;
uintptr_t pios_com_frsky_sensor_hub_id;
uintptr_t pios_com_lighttelemetry_id;
uintptr_t pios_com_picoc_id;
uintptr_t pios_com_overo_id;

uintptr_t pios_uavo_settings_fs_id;
uintptr_t pios_waypoints_settings_fs_id;

/*
 * Setup a com port based on the passed cfg, driver and buffer sizes. rx or tx size of 0 disables rx or tx
 */
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
static void PIOS_Board_configure_com (const struct pios_usart_cfg *usart_port_cfg, size_t rx_buf_len, size_t tx_buf_len,
		const struct pios_com_driver *com_driver, uintptr_t *pios_com_id)
{
	uintptr_t pios_usart_id;
	if (PIOS_USART_Init(&pios_usart_id, usart_port_cfg)) {
		PIOS_Assert(0);
	}

	uint8_t * rx_buffer;
	if (rx_buf_len > 0) {
		rx_buffer = (uint8_t *) pvPortMalloc(rx_buf_len);
		PIOS_Assert(rx_buffer);
	} else {
		rx_buffer = NULL;
	}

	uint8_t * tx_buffer;
	if (tx_buf_len > 0) {
		tx_buffer = (uint8_t *) pvPortMalloc(tx_buf_len);
		PIOS_Assert(tx_buffer);
	} else {
		tx_buffer = NULL;
	}

	if (PIOS_COM_Init(pios_com_id, com_driver, pios_usart_id,
				rx_buffer, rx_buf_len,
				tx_buffer, tx_buf_len)) {
		PIOS_Assert(0);
	}
}
#endif	/* PIOS_INCLUDE_USART && PIOS_INCLUDE_COM */

#ifdef PIOS_INCLUDE_DSM
static void PIOS_Board_configure_dsm(const struct pios_usart_cfg *pios_usart_dsm_cfg, const struct pios_dsm_cfg *pios_dsm_cfg,
		const struct pios_com_driver *pios_usart_com_driver,enum pios_dsm_proto *proto,
		ManualControlSettingsChannelGroupsOptions channelgroup,uint8_t *bind)
{
	uintptr_t pios_usart_dsm_id;
	if (PIOS_USART_Init(&pios_usart_dsm_id, pios_usart_dsm_cfg)) {
		PIOS_Assert(0);
	}

	uintptr_t pios_dsm_id;
	if (PIOS_DSM_Init(&pios_dsm_id, pios_dsm_cfg, pios_usart_com_driver,
			pios_usart_dsm_id, *proto, *bind)) {
		PIOS_Assert(0);
	}

	uintptr_t pios_dsm_rcvr_id;
	if (PIOS_RCVR_Init(&pios_dsm_rcvr_id, &pios_dsm_rcvr_driver, pios_dsm_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[channelgroup] = pios_dsm_rcvr_id;
}
#endif

#ifdef PIOS_INCLUDE_HSUM
static void PIOS_Board_configure_hsum(const struct pios_usart_cfg *pios_usart_hsum_cfg,
		const struct pios_com_driver *pios_usart_com_driver,enum pios_hsum_proto *proto,
		ManualControlSettingsChannelGroupsOptions channelgroup)
{
	uintptr_t pios_usart_hsum_id;
	if (PIOS_USART_Init(&pios_usart_hsum_id, pios_usart_hsum_cfg)) {
		PIOS_Assert(0);
	}
	
	uintptr_t pios_hsum_id;
	if (PIOS_HSUM_Init(&pios_hsum_id, pios_usart_com_driver, pios_usart_hsum_id, *proto)) {
		PIOS_Assert(0);
	}
	
	uintptr_t pios_hsum_rcvr_id;
	if (PIOS_RCVR_Init(&pios_hsum_rcvr_id, &pios_hsum_rcvr_driver, pios_hsum_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[channelgroup] = pios_hsum_rcvr_id;
}
#endif

/**
 * Indicate a target-specific error code when a component fails to initialize
 * 1 pulse - flash chip
 */
static void panic(int32_t code) {
	while(1){
		for (int32_t i = 0; i < code; i++) {
			PIOS_WDG_Clear();
			PIOS_LED_Toggle(PIOS_LED_ALARM);
			PIOS_DELAY_WaitmS(200);
			PIOS_WDG_Clear();
			PIOS_LED_Toggle(PIOS_LED_ALARM);
			PIOS_DELAY_WaitmS(200);
		}
		PIOS_DELAY_WaitmS(200);
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(200);
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(100);
		PIOS_WDG_Clear();
	}
}

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */

#include <pios_board_info.h>

void PIOS_Board_Init(void) {

	/* Delay system */
	PIOS_DELAY_Init();

	const struct pios_board_info * bdinfo = &pios_board_info_blob;

#if defined(PIOS_INCLUDE_LED)
	const struct pios_led_cfg * led_cfg = PIOS_BOARD_HW_DEFS_GetLedCfg(bdinfo->board_rev);
	PIOS_Assert(led_cfg);
	PIOS_LED_Init(led_cfg);
#endif	/* PIOS_INCLUDE_LED */


#if defined(PIOS_INCLUDE_FLASH)
	/* Inititialize all flash drivers */
	if (PIOS_Flash_Internal_Init(&pios_internal_flash_id, &flash_internal_cfg) != 0)
		panic(1);

	/* Register the partition table */
	const struct pios_flash_partition * flash_partition_table;
	uint32_t num_partitions;
	flash_partition_table = PIOS_BOARD_HW_DEFS_GetPartitionTable(bdinfo->board_rev, &num_partitions);
	PIOS_FLASH_register_partition_table(flash_partition_table, num_partitions);

	/* Mount all filesystems */
	if (PIOS_FLASHFS_Logfs_Init(&pios_uavo_settings_fs_id, &flashfs_settings_cfg, FLASH_PARTITION_LABEL_SETTINGS) != 0)
		panic(1);

#if defined(PIOS_INCLUDE_EXTERNAL_FLASH_WAYPOINTS)
	/* Use external flash chip to store waypoints */
	if (PIOS_FLASHFS_Logfs_Init(&pios_waypoints_settings_fs_id, &flashfs_external_waypoints_cfg, FLASH_PARTITION_LABEL_WAYPOINTS) != 0)
		panic(1);
#endif /* PIOS_INCLUDE_EXTERNAL_FLASH_WAYPOINTS */

#endif	/* PIOS_INCLUDE_FLASH */

	/* Initialize UAVObject libraries */
	EventDispatcherInitialize();
	UAVObjInitialize();

	HwDiscoveryF4Initialize();
	ModuleSettingsInitialize();

#if defined(PIOS_INCLUDE_RTC)
	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);
#endif

#ifndef ERASE_FLASH
	/* Initialize watchdog as early as possible to catch faults during init
	 * but do it only if there is no debugger connected
	 */
	if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == 0) {
		PIOS_WDG_Init();
	}
#endif

	/* Initialize the alarms library */
	AlarmsInitialize();

	/* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* Set up pulse timers */
	//inputs
	PIOS_TIM_InitClock(&tim_2_cfg);
	PIOS_TIM_InitClock(&tim_4_cfg);
	PIOS_TIM_InitClock(&tim_8_cfg);
	PIOS_TIM_InitClock(&tim_9_cfg);
	//outputs
	PIOS_TIM_InitClock(&tim_1_cfg);
	PIOS_TIM_InitClock(&tim_3_cfg);

	/* IAP System Setup */
	PIOS_IAP_Init();
	uint16_t boot_count = PIOS_IAP_ReadBootCount();
	if (boot_count < 3) {
		PIOS_IAP_WriteBootCount(++boot_count);
		AlarmsClear(SYSTEMALARMS_ALARM_BOOTFAULT);
	} else {
		/* Too many failed boot attempts, force hw config to defaults */
		HwDiscoveryF4SetDefaults(HwDiscoveryF4Handle(), 0);
		ModuleSettingsSetDefaults(ModuleSettingsHandle(),0);
		AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
	}

#if defined(PIOS_INCLUDE_USB)
	/* Initialize board specific USB data */
	PIOS_USB_BOARD_DATA_Init();

	/* Flags to determine if various USB interfaces are advertised */
	bool usb_hid_present = false;
	bool usb_cdc_present = false;

#if defined(PIOS_INCLUDE_USB_CDC)
	if (PIOS_USB_DESC_HID_CDC_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
	usb_cdc_present = true;
#else
	if (PIOS_USB_DESC_HID_ONLY_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
#endif

	uintptr_t pios_usb_id;
	PIOS_USB_Init(&pios_usb_id, PIOS_BOARD_HW_DEFS_GetUsbCfg(bdinfo->board_rev));

#if defined(PIOS_INCLUDE_USB_CDC)

	uint8_t hw_usb_vcpport;
	/* Configure the USB VCP port */
	HwDiscoveryF4USB_VCPPortGet(&hw_usb_vcpport);

	if (!usb_cdc_present) {
		/* Force VCP port function to disabled if we haven't advertised VCP in our USB descriptor */
		hw_usb_vcpport = HWDISCOVERYF4_USB_VCPPORT_DISABLED;
	}

	uintptr_t pios_usb_cdc_id;
	if (PIOS_USB_CDC_Init(&pios_usb_cdc_id, &pios_usb_cdc_cfg, pios_usb_id)) {
		PIOS_Assert(0);
	}

	switch (hw_usb_vcpport) {
	case HWDISCOVERYF4_USB_VCPPORT_DISABLED:
		break;
	case HWDISCOVERYF4_USB_VCPPORT_USBTELEMETRY:
#if defined(PIOS_INCLUDE_COM)
		{
			uint8_t * rx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_TELEM_USB_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_TELEM_USB_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						rx_buffer, PIOS_COM_TELEM_USB_RX_BUF_LEN,
						tx_buffer, PIOS_COM_TELEM_USB_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_COM */
		break;
	case HWDISCOVERYF4_USB_VCPPORT_COMBRIDGE:
#if defined(PIOS_INCLUDE_COM)
		{
			uint8_t * rx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_BRIDGE_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_BRIDGE_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_vcp_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						rx_buffer, PIOS_COM_BRIDGE_RX_BUF_LEN,
						tx_buffer, PIOS_COM_BRIDGE_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_COM */
		break;
	case HWDISCOVERYF4_USB_VCPPORT_DEBUGCONSOLE:
#if defined(PIOS_INCLUDE_COM)
#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
		{
			uint8_t * tx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_debug_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						NULL, 0,
						tx_buffer, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
#endif	/* PIOS_INCLUDE_COM */
		break;
	case HWDISCOVERYF4_USB_VCPPORT_PICOC:
#if defined(PIOS_INCLUDE_COM)
		{
			uint8_t * rx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_PICOC_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_PICOC_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_picoc_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						rx_buffer, PIOS_COM_PICOC_RX_BUF_LEN,
						tx_buffer, PIOS_COM_PICOC_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_COM */
		break;
	}
#endif	/* PIOS_INCLUDE_USB_CDC */

#if defined(PIOS_INCLUDE_USB_HID)
	/* Configure the usb HID port */
	uint8_t hw_usb_hidport;
	HwDiscoveryF4USB_HIDPortGet(&hw_usb_hidport);

	if (!usb_hid_present) {
		/* Force HID port function to disabled if we haven't advertised HID in our USB descriptor */
		hw_usb_hidport = HWDISCOVERYF4_USB_HIDPORT_DISABLED;
	}

	uintptr_t pios_usb_hid_id;
	if (PIOS_USB_HID_Init(&pios_usb_hid_id, &pios_usb_hid_cfg, pios_usb_id)) {
		PIOS_Assert(0);
	}

	switch (hw_usb_hidport) {
	case HWDISCOVERYF4_USB_HIDPORT_DISABLED:
		break;
	case HWDISCOVERYF4_USB_HIDPORT_USBTELEMETRY:
#if defined(PIOS_INCLUDE_COM)
		{
			uint8_t * rx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_TELEM_USB_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) pvPortMalloc(PIOS_COM_TELEM_USB_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_hid_com_driver, pios_usb_hid_id,
						rx_buffer, PIOS_COM_TELEM_USB_RX_BUF_LEN,
						tx_buffer, PIOS_COM_TELEM_USB_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
#endif	/* PIOS_INCLUDE_COM */
		break;
	}

#endif	/* PIOS_INCLUDE_USB_HID */

	if (usb_hid_present || usb_cdc_present) {
		PIOS_USBHOOK_Activate();
	}
#endif	/* PIOS_INCLUDE_USB */

	/* Configure the IO ports */
	uint8_t hw_DSMxBind;
	HwDiscoveryF4DSMxBindGet(&hw_DSMxBind);

	/* init sensor queue registration */
	PIOS_SENSORS_Init();

	/* UART1 Port */
	uint8_t hw_uart1;
	HwDiscoveryF4Uart1Get(&hw_uart1);
	switch (hw_uart1) {
	case HWDISCOVERYF4_UART1_DISABLED:
		break;
	case HWDISCOVERYF4_UART1_GPS:
#if defined(PIOS_INCLUDE_GPS) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart1_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_GPS_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
#endif
		break;
	case HWDISCOVERYF4_UART1_SBUS:
		//hardware signal inverter required
#if defined(PIOS_INCLUDE_SBUS) && defined(PIOS_INCLUDE_USART)
		{
			uintptr_t pios_usart_sbus_id;
			if (PIOS_USART_Init(&pios_usart_sbus_id, &pios_usart1_sbus_cfg)) {
				PIOS_Assert(0);
			}
			uintptr_t pios_sbus_id;
			if (PIOS_SBus_Init(&pios_sbus_id, &pios_usart1_sbus_aux_cfg, &pios_usart_com_driver, pios_usart_sbus_id)) {
				PIOS_Assert(0);
			}
			uintptr_t pios_sbus_rcvr_id;
			if (PIOS_RCVR_Init(&pios_sbus_rcvr_id, &pios_sbus_rcvr_driver, pios_sbus_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_SBUS] = pios_sbus_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_SBUS */
		break;
	case HWDISCOVERYF4_UART1_DSM2:
	case HWDISCOVERYF4_UART1_DSMX10BIT:
	case HWDISCOVERYF4_UART1_DSMX11BIT:
#if defined(PIOS_INCLUDE_DSM)
		{
			enum pios_dsm_proto proto;
			switch (hw_uart1) {
			case HWDISCOVERYF4_UART1_DSM2:
				proto = PIOS_DSM_PROTO_DSM2;
				break;
			case HWDISCOVERYF4_UART1_DSMX10BIT:
				proto = PIOS_DSM_PROTO_DSMX10BIT;
				break;
			case HWDISCOVERYF4_UART1_DSMX11BIT:
				proto = PIOS_DSM_PROTO_DSMX11BIT;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_dsm(&pios_usart1_dsm_hsum_cfg, &pios_usart1_dsm_aux_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_DSMMAINPORT, &hw_DSMxBind);
		}
#endif	/* PIOS_INCLUDE_DSM */
		break;
	case HWDISCOVERYF4_UART1_HOTTSUMD:
	case HWDISCOVERYF4_UART1_HOTTSUMH:
#if defined(PIOS_INCLUDE_HSUM)
		{
			enum pios_hsum_proto proto;
			switch (hw_uart1) {
			case HWDISCOVERYF4_UART1_HOTTSUMD:
				proto = PIOS_HSUM_PROTO_SUMD;
				break;
			case HWDISCOVERYF4_UART1_HOTTSUMH:
				proto = PIOS_HSUM_PROTO_SUMH;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_hsum(&pios_usart1_dsm_hsum_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_HOTTSUM);
		}
#endif	/* PIOS_INCLUDE_HSUM */
		break;
	case HWDISCOVERYF4_UART1_PICOC:
#if defined(PIOS_INCLUDE_PICOC) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart1_cfg, PIOS_COM_PICOC_RX_BUF_LEN, PIOS_COM_PICOC_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_picoc_id);
#endif /* PIOS_INCLUDE_PICOC */
		break;
	}


	/* UART2 Port */
	uint8_t hw_uart2;
	HwDiscoveryF4Uart2Get(&hw_uart2);
	switch (hw_uart2) {
	case HWDISCOVERYF4_UART2_DISABLED:
		break;
	case HWDISCOVERYF4_UART2_TELEMETRY:
#if defined(PIOS_INCLUDE_TELEMETRY_RF) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart2_cfg, PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_telem_rf_id);
#endif /* PIOS_INCLUDE_TELEMETRY_RF */
		break;
	case HWDISCOVERYF4_UART2_GPS:
#if defined(PIOS_INCLUDE_GPS) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart2_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_GPS_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
#endif
		break;
	case HWDISCOVERYF4_UART2_DSM2:
	case HWDISCOVERYF4_UART2_DSMX10BIT:
	case HWDISCOVERYF4_UART2_DSMX11BIT:
#if defined(PIOS_INCLUDE_DSM)
		{
			enum pios_dsm_proto proto;
			switch (hw_uart2) {
			case HWDISCOVERYF4_UART2_DSM2:
				proto = PIOS_DSM_PROTO_DSM2;
				break;
			case HWDISCOVERYF4_UART2_DSMX10BIT:
				proto = PIOS_DSM_PROTO_DSMX10BIT;
				break;
			case HWDISCOVERYF4_UART2_DSMX11BIT:
				proto = PIOS_DSM_PROTO_DSMX11BIT;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_dsm(&pios_usart2_dsm_hsum_cfg, &pios_usart2_dsm_aux_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_DSMMAINPORT, &hw_DSMxBind);
		}
#endif	/* PIOS_INCLUDE_DSM */
		break;
	case HWDISCOVERYF4_UART2_HOTTSUMD:
	case HWDISCOVERYF4_UART2_HOTTSUMH:
#if defined(PIOS_INCLUDE_HSUM)
		{
			enum pios_hsum_proto proto;
			switch (hw_uart2) {
			case HWDISCOVERYF4_UART2_HOTTSUMD:
				proto = PIOS_HSUM_PROTO_SUMD;
				break;
			case HWDISCOVERYF4_UART2_HOTTSUMH:
				proto = PIOS_HSUM_PROTO_SUMH;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_hsum(&pios_usart2_dsm_hsum_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_HOTTSUM);
		}
#endif	/* PIOS_INCLUDE_HSUM */
		break;
	case HWDISCOVERYF4_UART2_DEBUGCONSOLE:
#if defined(PIOS_INCLUDE_DEBUG_CONSOLE) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart2_cfg, 0, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_debug_id);
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
		break;
	case HWDISCOVERYF4_UART2_COMBRIDGE:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart2_cfg, PIOS_COM_BRIDGE_RX_BUF_LEN, PIOS_COM_BRIDGE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_bridge_id);
#endif
		break;
	case HWDISCOVERYF4_UART2_MAVLINKTX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_usart2_cfg, 0, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_mavlink_id);
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDISCOVERYF4_UART2_MAVLINKTX_GPS_RX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK) && defined(PIOS_INCLUDE_GPS)
		PIOS_Board_configure_com(&pios_usart2_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
		pios_com_mavlink_id = pios_com_gps_id;
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDISCOVERYF4_UART2_FRSKYSENSORHUB:
#if defined(PIOS_INCLUDE_FRSKY_SENSOR_HUB) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart2_cfg, 0, PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sensor_hub_id);
#endif /* PIOS_INCLUDE_FRSKY_SENSOR_HUB */
		break;
	case HWDISCOVERYF4_UART2_LIGHTTELEMETRYTX:
#if defined(PIOS_INCLUDE_LIGHTTELEMETRY)
		PIOS_Board_configure_com(&pios_usart2_cfg, 0, PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_lighttelemetry_id);
#endif  
		break;
	case HWDISCOVERYF4_UART2_PICOC:
#if defined(PIOS_INCLUDE_PICOC) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart2_cfg, PIOS_COM_PICOC_RX_BUF_LEN, PIOS_COM_PICOC_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_picoc_id);
#endif /* PIOS_INCLUDE_PICOC */
		break;
	}


	/* UART3 Port */
	uint8_t hw_uart3;
	HwDiscoveryF4Uart3Get(&hw_uart3);
	switch (hw_uart3) {
	case HWDISCOVERYF4_UART3_DISABLED:
		break;
	case HWDISCOVERYF4_UART3_TELEMETRY:
#if defined(PIOS_INCLUDE_TELEMETRY_RF) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart3_cfg, PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_telem_rf_id);
#endif /* PIOS_INCLUDE_TELEMETRY_RF */
		break;
	case HWDISCOVERYF4_UART3_GPS:
#if defined(PIOS_INCLUDE_GPS) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart3_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_GPS_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
#endif
		break;
	case HWDISCOVERYF4_UART3_DSM2:
	case HWDISCOVERYF4_UART3_DSMX10BIT:
	case HWDISCOVERYF4_UART3_DSMX11BIT:
#if defined(PIOS_INCLUDE_DSM)
		{
			enum pios_dsm_proto proto;
			switch (hw_uart3) {
			case HWDISCOVERYF4_UART3_DSM2:
				proto = PIOS_DSM_PROTO_DSM2;
				break;
			case HWDISCOVERYF4_UART3_DSMX10BIT:
				proto = PIOS_DSM_PROTO_DSMX10BIT;
				break;
			case HWDISCOVERYF4_UART3_DSMX11BIT:
				proto = PIOS_DSM_PROTO_DSMX11BIT;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_dsm(&pios_usart3_dsm_hsum_cfg, &pios_usart3_dsm_aux_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_DSMMAINPORT, &hw_DSMxBind);
		}
#endif	/* PIOS_INCLUDE_DSM */
	case HWDISCOVERYF4_UART3_HOTTSUMD:
	case HWDISCOVERYF4_UART3_HOTTSUMH:
#if defined(PIOS_INCLUDE_HSUM)
		{
			enum pios_hsum_proto proto;
			switch (hw_uart3) {
			case HWDISCOVERYF4_UART3_HOTTSUMD:
				proto = PIOS_HSUM_PROTO_SUMD;
				break;
			case HWDISCOVERYF4_UART3_HOTTSUMH:
				proto = PIOS_HSUM_PROTO_SUMH;
				break;
			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_hsum(&pios_usart3_dsm_hsum_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_HOTTSUM);
		}
#endif	/* PIOS_INCLUDE_HSUM */
		break;
	case HWDISCOVERYF4_UART3_DEBUGCONSOLE:
#if defined(PIOS_INCLUDE_DEBUG_CONSOLE) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart3_cfg, 0, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_debug_id);
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
		break;
	case HWDISCOVERYF4_UART3_COMBRIDGE:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart3_cfg, PIOS_COM_BRIDGE_RX_BUF_LEN, PIOS_COM_BRIDGE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_bridge_id);
#endif
		break;
	case HWDISCOVERYF4_UART3_MAVLINKTX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_usart3_cfg, 0, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_mavlink_id);
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDISCOVERYF4_UART3_MAVLINKTX_GPS_RX:
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM) && defined(PIOS_INCLUDE_MAVLINK) && defined(PIOS_INCLUDE_GPS)
		PIOS_Board_configure_com(&pios_usart3_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
		pios_com_mavlink_id = pios_com_gps_id;
#endif	/* PIOS_INCLUDE_MAVLINK */
		break;
	case HWDISCOVERYF4_UART3_FRSKYSENSORHUB:
#if defined(PIOS_INCLUDE_FRSKY_SENSOR_HUB) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart3_cfg, 0, PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sensor_hub_id);
#endif /* PIOS_INCLUDE_FRSKY_SENSOR_HUB */
		break;
	case HWDISCOVERYF4_UART3_LIGHTTELEMETRYTX:
#if defined(PIOS_INCLUDE_LIGHTTELEMETRY)
		PIOS_Board_configure_com(&pios_usart3_cfg, 0, PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_lighttelemetry_id);
#endif  
		break;
	case HWDISCOVERYF4_UART3_PICOC:
#if defined(PIOS_INCLUDE_PICOC) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_usart3_cfg, PIOS_COM_PICOC_RX_BUF_LEN, PIOS_COM_PICOC_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_picoc_id);
#endif /* PIOS_INCLUDE_PICOC */
		break;
	}


	/* Configure the rcvr port */
	uint8_t hw_rcvrport;
	HwDiscoveryF4RcvrPortGet(&hw_rcvrport);

	switch (hw_rcvrport) {
	case HWDISCOVERYF4_RCVRPORT_DISABLED:
		break;
	case HWDISCOVERYF4_RCVRPORT_PWM:
#if defined(PIOS_INCLUDE_PWM)
		{
			uintptr_t pios_pwm_id;
			PIOS_PWM_Init(&pios_pwm_id, &pios_pwm_cfg);

			uintptr_t pios_pwm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_pwm_rcvr_id, &pios_pwm_rcvr_driver, pios_pwm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PWM] = pios_pwm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PWM */
		break;
	case HWDISCOVERYF4_RCVRPORT_PPM:
	case HWDISCOVERYF4_RCVRPORT_PPMOUTPUTS:
#if defined(PIOS_INCLUDE_PPM)
		{
			uintptr_t pios_ppm_id;
			PIOS_PPM_Init(&pios_ppm_id, &pios_ppm_cfg);

			uintptr_t pios_ppm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_ppm_rcvr_driver, pios_ppm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM] = pios_ppm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PPM */
		break;
	case HWDISCOVERYF4_RCVRPORT_PPMPWM:
		/* This is a combination of PPM and PWM inputs */
#if defined(PIOS_INCLUDE_PPM)
		{
			uintptr_t pios_ppm_id;
			PIOS_PPM_Init(&pios_ppm_id, &pios_ppm_cfg);

			uintptr_t pios_ppm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_ppm_rcvr_driver, pios_ppm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM] = pios_ppm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PPM */
#if defined(PIOS_INCLUDE_PWM)
		{
			uintptr_t pios_pwm_id;
			PIOS_PWM_Init(&pios_pwm_id, &pios_pwm_with_ppm_cfg);

			uintptr_t pios_pwm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_pwm_rcvr_id, &pios_pwm_rcvr_driver, pios_pwm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PWM] = pios_pwm_rcvr_id;
		}
#endif	/* PIOS_INCLUDE_PWM */
		break;
	}


#if defined(PIOS_INCLUDE_GCSRCVR)
	GCSReceiverInitialize();
	uintptr_t pios_gcsrcvr_id;
	PIOS_GCSRCVR_Init(&pios_gcsrcvr_id);
	uintptr_t pios_gcsrcvr_rcvr_id;
	if (PIOS_RCVR_Init(&pios_gcsrcvr_rcvr_id, &pios_gcsrcvr_rcvr_driver, pios_gcsrcvr_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_GCS] = pios_gcsrcvr_rcvr_id;
#endif	/* PIOS_INCLUDE_GCSRCVR */

#ifndef PIOS_DEBUG_ENABLE_DEBUG_PINS
	switch (hw_rcvrport) {
		case HWDISCOVERYF4_RCVRPORT_DISABLED:
		case HWDISCOVERYF4_RCVRPORT_PWM:
		case HWDISCOVERYF4_RCVRPORT_PPM:
			/* Set up the servo outputs */
#ifdef PIOS_INCLUDE_SERVO
			PIOS_Servo_Init(&pios_servo_cfg);
#endif
			break;
		case HWDISCOVERYF4_RCVRPORT_PPMOUTPUTS:
		case HWDISCOVERYF4_RCVRPORT_OUTPUTS:
#ifdef PIOS_INCLUDE_SERVO
			PIOS_Servo_Init(&pios_servo_rcvr_cfg);
#endif
			break;
	}
#else
	PIOS_DEBUG_Init(&pios_tim_servo_all_channels, NELEMENTS(pios_tim_servo_all_channels));
#endif

	PIOS_WDG_Clear();
	PIOS_DELAY_WaitmS(200);
	PIOS_WDG_Clear();

#if defined(PIOS_INCLUDE_I2C)
	if (PIOS_I2C_Init(&pios_i2c_external_adapter_id, &pios_i2c_external_adapter_cfg)) {
		PIOS_DEBUG_Assert(0);
	}

	if (PIOS_I2C_CheckClear(pios_i2c_external_adapter_id) != 0)
		panic(5);

#if defined(PIOS_INCLUDE_LSM9DS1)
	if (PIOS_LSM9DS1_Probe(pios_i2c_external_adapter_id, lsm9ds1_cfg.i2c_addr_ax_g) == LSM9DS1_WHO_AM_I_VAL)
	{
		int retval;
		retval = PIOS_LSM9DS1_Init(pios_i2c_external_adapter_id, &lsm9ds1_cfg);
		if (retval == -10)
			panic(1); // indicate missing IRQ separately
		if (retval != 0)
			panic(2);

		// To be safe map from UAVO enum to driver enum
		uint8_t hw_gyro_range;
		HwDiscoveryF4GyroRangeGet(&hw_gyro_range);
		switch(hw_gyro_range) {
			case HWDISCOVERYF4_GYRORANGE_245:
				PIOS_LSM9DS1_SetGyroRange(LSM9DS1_G_FS_245_DPS);
				break;
			case HWDISCOVERYF4_GYRORANGE_500:
				PIOS_LSM9DS1_SetGyroRange(LSM9DS1_G_FS_500_DPS);
				break;
			case HWDISCOVERYF4_GYRORANGE_2000:
				PIOS_LSM9DS1_SetGyroRange(LSM9DS1_G_FS_2000_DPS);
				break;
		}

		uint8_t hw_accel_range;
		HwDiscoveryF4AccelRangeGet(&hw_accel_range);
		switch(hw_accel_range) {
			case HWDISCOVERYF4_ACCELRANGE_2G:
				PIOS_LSM9DS1_SetAccRange(LSM9DS1_XL_FS_2_G);
				break;
			case HWDISCOVERYF4_ACCELRANGE_4G:
				PIOS_LSM9DS1_SetAccRange(LSM9DS1_XL_FS_4_G);
				break;
			case HWDISCOVERYF4_ACCELRANGE_8G:
				PIOS_LSM9DS1_SetAccRange(LSM9DS1_XL_FS_8_G);
				break;
		}
	} else
		panic(3);
#endif

	//I2C is slow, sensor init as well, reset watchdog to prevent reset here
	PIOS_WDG_Clear();

#if defined(PIOS_INCLUDE_MS5611)
	if (PIOS_MS5611_Init(&pios_ms5611_cfg, pios_i2c_10dof_adapter_id) != 0)
		panic(4);
	if (PIOS_MS5611_Test() != 0)
		panic(4);
#endif

	//I2C is slow, sensor init as well, reset watchdog to prevent reset here
	PIOS_WDG_Clear();

#endif	/* PIOS_INCLUDE_I2C */


#if defined(PIOS_INCLUDE_GPIO)
	PIOS_GPIO_Init();
#endif

#if defined(PIOS_INCLUDE_ADC)
	PIOS_ADC_Init(&pios_adc_cfg);
#endif

	/* Make sure we have at least one telemetry link configured or else fail initialization */
	PIOS_Assert(pios_com_telem_rf_id || pios_com_telem_usb_id);
}

/**
 * @}
 */
