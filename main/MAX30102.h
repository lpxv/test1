/*
 * MAX30102.h
 *
 *  Created on: 2021Äê7ÔÂ13ÈÕ
 *      Author: dell
 */

#ifndef MAIN_MAX30102_H_
#define MAIN_MAX30102_H_


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


extern void max30102_init(void);
extern esp_err_t max30102_Bus_Write(uint16_t WriteAddr, uint8_t data_wr);
extern esp_err_t max30102_Bus_Read( uint16_t ReadAddr, uint8_t* data_rd);
extern esp_err_t i2c_master_init(void);
extern void MAX30102_Read_FIFO_Data(uint8_t *data);
extern void MAX30102_Read_FIFO_Data_All (uint8_t *data);
extern uint16_t num_avaliable_samples;

#endif /* MAIN_MAX30102_H_ */
