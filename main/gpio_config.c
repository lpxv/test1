/*
 * gpio_config.c
 *
 *  Created on: 2021Äê7ÔÂ13ÈÕ
 *      Author: dell
 */
#include "gpio_config.h"


#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {

    	printf("gpio_task_example() running_0! \n" );
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    	printf("gpio_task_example() running_1! \n" );
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    	printf("gpio_task_example() running_2! \n" );

        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
        	printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));

        	if (io_num == 4)
        		printf("gpio4 falling edge detected! \n" );
        }

        printf("gpio_task_example() running_3! \n" );
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void gpio_config_init(void)
{
    gpio_config_t io_conf;

    //interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;

    //enable pull-up mode
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;

    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_NEGEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task prio=3
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
}

