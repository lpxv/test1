/*
 * gpio_config.h
 *
 *  Created on: 2021Äê7ÔÂ13ÈÕ
 *      Author: dell
 */

#ifndef MAIN_GPIO_CONFIG_H_
#define MAIN_GPIO_CONFIG_H_


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "max30102.h"
#include "gatt_demo.h"

extern uint8_t *ptr;
extern uint8_t spo2_fifo_burst[32][6];

extern void gpio_config_init(void);

#endif /* MAIN_GPIO_CONFIG_H_ */
