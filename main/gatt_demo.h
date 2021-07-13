/*
 * gatt_demo.h
 *
 *  Created on: 2021Äê7ÔÂ13ÈÕ
 *      Author: dell
 */

#ifndef MAIN_GATT_DEMO_H_
#define MAIN_GATT_DEMO_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"
//#include "MAX30102.h"

#include "driver/i2c.h"


#include "gpio_config.h"
#include "max30102.h"

/*
#include "freertos/FreeRTOS.h"

original is

#ifndef configUSE_TRACE_FACILITY
	#define configUSE_TRACE_FACILITY 0
#endif

edit to

#ifndef configUSE_TRACE_FACILITY
	#define configUSE_TRACE_FACILITY 0
#endif

*/

#endif /* MAIN_GATT_DEMO_H_ */
