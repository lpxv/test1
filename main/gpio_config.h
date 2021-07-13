/*
 * gpio_config.h
 *
 *  Created on: 2021��7��13��
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


extern void gpio_config_init(void);

#endif /* MAIN_GPIO_CONFIG_H_ */
