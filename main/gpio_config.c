/*
 * gpio_config.c
 *
 *  Created on: 2021年7月13日
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

unsigned char temp_read=0;
uint8_t *ptr;
uint8_t spo2_fifo_burst[32][6];
uint32_t spo2_data_red;
uint32_t spo2_data_ir;
uint8_t count_i;
TickType_t xTickCount;
//uint8_t notify_data_test[15]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    uint32_t io_level;
    esp_err_t ret;

    for(;;)
    {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
        	io_level = gpio_get_level(io_num);
        	printf("GPIO[%d] intr, val: %d\n", io_num, io_level);

        	xTickCount = xTaskGetTickCount();
			printf("Tick = %d\n", xTickCount);//1 tick = 10ms?
			vTaskDelay(1 / portTICK_PERIOD_MS);

        	if ( (io_num==4) && (io_level==0) )
			{
				printf("gpio4 falling edge detected! \n" );
				//no interrupter
				temp_read = 0x00;
				max30102_Bus_Read(0x00, &temp_read);//read status 1 reg
				printf(" Interrupt Status = %x\n", temp_read);
/*
				if (temp_read & 0x80)
				{
					//printf(" INT A_FULL \n");
					//read fifo burst
					ptr = (uint8_t *)spo2_fifo_burst;
					MAX30102_Read_FIFO_Data_All(ptr);

					for (count_i=0; count_i<num_avaliable_samples; count_i++)//这里怎么考虑队列的长度？cmd究竟可以放多少的队列？
					{
						spo2_data_red = ( ( (*ptr<<16) + (*(ptr+1)<<8) + (*(ptr+2)) ) & 0x03ffff );
						ptr += 3;
						spo2_data_ir = ( ( (*ptr<<16) + (*(ptr+1)<<8) + (*(ptr+2)) ) & 0x03ffff );
						ptr += 3;
						printf("%d    %d\n", spo2_data_red, spo2_data_ir);// red--- ir
					}

					if (notifyed_a_en == 1)
					{
					  printf("ready to send notify\n");
					  //the size of notify_data[] need less than MTU size
					  ret = esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_A_APP_ID].gatts_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
							  num_avaliable_samples*6, ptr, false);
					  if (ret == ESP_OK)
						  printf("send notify success \n");
					  else
						  printf("send notify failed, maybe the connection not created, pl recheck\n");

					}
				}
				if(temp_read & 0x40)
				{
					printf(" INT PPG_RDY \n");
					//MAX30102_Read_FIFO_Data(spo2_fifo);
					//spo2_data[0] = ((spo2_fifo[0]<<16 | spo2_fifo[1]<<8 | spo2_fifo[2]) & 0x03ffff);
					//spo2_data[1] = ((spo2_fifo[3]<<16 | spo2_fifo[4]<<8 | spo2_fifo[5]) & 0x03ffff);
					//printf("%d    %d\n", spo2_data[0], spo2_data[1]);
					//xTickCount = xTaskGetTickCount();
					//ESP_LOGE(GATTS_TAG, "Tick = %d\n", xTickCount);//1 tick resp 10ms?
				}
				if (temp_read & 0x20)
				{
					printf(" INT ALC_OVF \n");
				}
				if (temp_read & 0x10)
				{
					printf(" INT PROX_ INT \n");
				}
				*/
			}
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
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

