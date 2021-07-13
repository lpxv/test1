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

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

extern uint8_t notifyed_a_en;
extern struct gatts_profile_inst gl_profile_tab[PROFILE_NUM];

#endif /* MAIN_GATT_DEMO_H_ */
