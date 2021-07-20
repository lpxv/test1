/*
 * MAX30102.c
 *
 *  Created on: 2021年7月13日
 *      Author: dell
 */


#include "max30102.h"

#ifndef GATTS_TAG
#define GATTS_TAG "GATTS_DEMO"
#endif

uint8_t fifo_wr_ptr;
uint8_t ovf_counter;
uint8_t fifo_rd_ptr;
uint16_t num_avaliable_samples;


//--------------------------------------------------------------------------------------------------------------------//
// lpx add for i2c drive
//--------------------------------------------------------------------------------------------------------------------//
//#define CONFIG_I2C_MASTER_PORT_NUM  0
//#define _I2C_NUMBER(num) I2C_NUM_##num
//#define I2C_NUMBER(num) _I2C_NUMBER(num)
//#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_NUM I2C_NUM_0

#define CONFIG_I2C_MASTER_SCL 13
#define CONFIG_I2C_MASTER_SDA 15
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */

#define CONFIG_I2C_MASTER_FREQUENCY 100000						//100KHz
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define MAX30102_DeviceAddr 0xAE								//SLAVE I2C Address max30102
//#define MAX30102_DeviceAddr 0xA0								//SLAVE I2C Address 24c02


#define WRITE_BIT 0x00                      /*!< I2C master write */
#define READ_BIT 0x01                       /*!< I2C master read  */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
#define INTERRUPT_REG  					0X00

#define INTERRUPT_STATUS1 0x00
#define INTERRUPT_STATUS2 0x01
#define INTERRUPT_ENABLE1 0x02
#define INTERRUPT_ENABLE2 0x03
#define FIFO_WR_PTR 0x04
#define OVERFLOW_COUNTER 0x05
#define FIFO_RD_PTR 0x06
#define FIFO_DATA 0x07
#define FIFO_CONFIG 0x08
#define MODE_CONFIG 0x09
#define SPO2_CONFIG 0x0A
#define LED1_PULSE_AMP 0x0C
#define LED2_PULSE_AMP 0x0D
#define PILOT_PA 0x10
#define DIE_TEMP_CONFIG 0x21
#define PROX_INT_THRESH 0x30


esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
    	ESP_LOGE(GATTS_TAG, "i2c_param_config() = %d \n", err);
    	return err;
    }
    err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_LOGE(GATTS_TAG, "i2c_driver_install() = %d \n", err);
	return err;
}

/* AT24C02写入一个字节函数，第一个参数为要写入的值，第二个参数为要写入的地址*/
esp_err_t max30102_Bus_Write(uint16_t WriteAddr, uint8_t data_wr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, WriteAddr % 256, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_wr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    //ESP_LOGE(GATTS_TAG, "max30102_Bus_Write() = %d \n", ret);

    return ret;
}


/* AT24C02读取一个字节函数，第一个参数为要读出值的存放指针，第二个参数为要读出的地址*/
esp_err_t max30102_Bus_Read( uint16_t ReadAddr, uint8_t* data_rd)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ReadAddr % 256, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_DeviceAddr | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_rd, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    //ESP_LOGE(GATTS_TAG, "max30102_Bus_Read() = %d \n", ret);

    return ret;
}

void MAX30102_Read_FIFO_Data(uint8_t *data)
{
	/*
    max30102_Bus_Read(FIFO_DATA, &data[0]);
    max30102_Bus_Read(FIFO_DATA, &data[1]);
    max30102_Bus_Read(FIFO_DATA, &data[2]);
    max30102_Bus_Read(FIFO_DATA, &data[3]);
    max30102_Bus_Read(FIFO_DATA, &data[4]);
    max30102_Bus_Read(FIFO_DATA, &data[5]);
*/
    //esp_err_t ret = max30102_Bus_Read(FIFO_DATA, &data[5]);
   // ESP_LOGE(GATTS_TAG, "max30102_Bus_Read() err = %d \n", ret);
   // ESP_LOGE(GATTS_TAG, "max30102_Bus_Read() data = %d \n", data[0]);

	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, FIFO_DATA, ACK_CHECK_EN);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | READ_BIT, ACK_CHECK_EN);

	i2c_master_read_byte(cmd, &data[0], ACK_VAL);
	i2c_master_read_byte(cmd, &data[1], ACK_VAL);
	i2c_master_read_byte(cmd, &data[2], ACK_VAL);
	i2c_master_read_byte(cmd, &data[3], ACK_VAL);
	i2c_master_read_byte(cmd, &data[4], ACK_VAL);
	i2c_master_read_byte(cmd, &data[5], NACK_VAL);

	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);



}



void MAX30102_Read_FIFO_Data_All_backup (uint8_t *data)
{

	i2c_cmd_handle_t cmd;
	uint8_t i,j;
	esp_err_t ret;

	//First transaction: Get the FIFO status
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, FIFO_WR_PTR, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &fifo_wr_ptr, ACK_VAL);
	i2c_master_read_byte(cmd, &ovf_counter, ACK_VAL);
	i2c_master_read_byte(cmd, &fifo_rd_ptr, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if (ovf_counter != 0)
	{
		ESP_LOGE(GATTS_TAG, "fifo overflow count = %d \n", ovf_counter);
	}
	num_avaliable_samples = ( (fifo_wr_ptr + 32)  - fifo_rd_ptr) % 32;

	//Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, FIFO_DATA, ACK_CHECK_EN);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | READ_BIT, ACK_CHECK_EN);

	j = num_avaliable_samples*6 - 1;
	for (i=0; i<j; i++)//这里怎么考虑队列的长度？cmd究竟可以放多少的队列？
	{
		i2c_master_read_byte(cmd, data++, ACK_VAL);
	}
	i2c_master_read_byte(cmd, data, NACK_VAL);//NACK is needed

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	if (ret != ESP_OK)
	{
		ESP_LOGE(GATTS_TAG, "i2c_master_cmd_begin() B err = %d \n", ret);
	}
	i2c_cmd_link_delete(cmd);

	/*
	 fifo_rd_ptr = fifo_wr_ptr;
	//Third transaction: write FIFO_RD_PTR need???
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, FIFO_RD_PTR, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, fifo_rd_ptr, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	if (ret != ESP_OK)
	{
		ESP_LOGE(GATTS_TAG, "i2c_master_cmd_begin() err = %d \n", ret);
	}
	i2c_cmd_link_delete(cmd);
	 */
}

void MAX30102_Read_FIFO_Data_All (uint8_t *data)
{

	i2c_cmd_handle_t cmd;
	uint8_t i,j;
	esp_err_t ret;

	//First transaction: Get the FIFO status
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, FIFO_WR_PTR, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &fifo_wr_ptr, ACK_VAL);
	i2c_master_read_byte(cmd, &ovf_counter, ACK_VAL);
	i2c_master_read_byte(cmd, &fifo_rd_ptr, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if (ovf_counter != 0)
	{
		ESP_LOGE(GATTS_TAG, "fifo overflow count = %d \n", ovf_counter);
	}
	num_avaliable_samples = ( (fifo_wr_ptr + 32)  - fifo_rd_ptr) % 32;

	//Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, FIFO_DATA, ACK_CHECK_EN);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | READ_BIT, ACK_CHECK_EN);

	j = num_avaliable_samples*6 - 1;
	for (i=0; i<j; i++)//这里怎么考虑队列的长度？cmd究竟可以放多少的队列？
	{
		i2c_master_read_byte(cmd, data++, ACK_VAL);
	}
	i2c_master_read_byte(cmd, data, NACK_VAL);//NACK is needed

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	if (ret != ESP_OK)
	{
		ESP_LOGE(GATTS_TAG, "i2c_master_cmd_begin() B err = %d \n", ret);
	}
	i2c_cmd_link_delete(cmd);



	//Third transaction: write FIFO_RD_PTR need???
	fifo_rd_ptr = fifo_wr_ptr;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MAX30102_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, FIFO_RD_PTR, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, fifo_rd_ptr, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	if (ret != ESP_OK)
	{
		ESP_LOGE(GATTS_TAG, "i2c_master_cmd_begin() err = %d \n", ret);
	}
	i2c_cmd_link_delete(cmd);

}

unsigned char temp_a=0;
void max30102_init(void)
{
	unsigned char temp_num=0;
	/*

	max30102_Bus_Write(0x09, 0x0b);  //mode configuration : temp_en[3]      MODE[2:0]=010 HR only enabled    011 SP02 enabled
	max30102_Bus_Write(0x01, 0xf0); //open all of interrupt
	max30102_Bus_Write(0X00, 0x00); //all interrupt clear
	max30102_Bus_Write(0x03, 0x02); //DIE_TEMP_RDY_EN
	max30102_Bus_Write(0x21, 0x01); //SET   TEMP_EN
	max30102_Bus_Write(0x0a, 0x47); //SPO2_SR[4:2]=001  100 per second    LED_PW[1:0]=11  16BITS
	max30102_Bus_Write(0x0c, 0x47);
	max30102_Bus_Write(0x0d, 0x47);

//macro
	max30102_Bus_Write(MODE_CONFIG, 0x0b);  //mode configuration : temp_en[3]      MODE[2:0]=010 HR only enabled    011 SP02 enabled
	max30102_Bus_Write(INTERRUPT_ENABLE1, 0xf0); //open all of interrupt
	max30102_Bus_Write(INTERRUPT_ENABLE2, 0x00); //all interrupt clear
	max30102_Bus_Write(INTERRUPT_ENABLE2, 0x02); //DIE_TEMP_RDY_EN
	max30102_Bus_Write(DIE_TEMP_CONFIG, 0x01); //SET   TEMP_EN
	max30102_Bus_Write(SPO2_CONFIG, 0x47); //SPO2_SR[4:2]=001  100 per second    LED_PW[1:0]=11  16BITS
	max30102_Bus_Write(LED1_PULSE_AMP, 0x47);
	max30102_Bus_Write(LED2_PULSE_AMP, 0x47);
	*/
	esp_err_t ret;
	int i=0;
	//read device ID
	ret = max30102_Bus_Read(0xfe, &temp_num);
	ESP_LOGE(GATTS_TAG, "Revision ID =  0x%x\n", temp_num);
	ret = max30102_Bus_Read(0xff, &temp_num);
	ESP_LOGE(GATTS_TAG, "Part ID =  0x%x\n", temp_num);

	// 主要寄存器配置参数
	//reset first
	max30102_Bus_Write(MODE_CONFIG, 0X40);         //RESET FIRST
	temp_num = 0x12;// give a value before read.
	while(1)
	{
		max30102_Bus_Read(MODE_CONFIG, &temp_num);
		if ( temp_num == 0x00 )
		{
			ESP_LOGE(GATTS_TAG, "reset device OK,");
			break;//reset OK
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
		i++;
		if (i>100)
		{
			ESP_LOGE(GATTS_TAG, "reset device timeout\n");
			return;
		}
	}
	//vTaskDelay(20 / portTICK_PERIOD_MS);

	ESP_LOGE(GATTS_TAG, "Continue init...\r\n");


	//max30102_Bus_Write(INTERRUPT_ENABLE1, 0xC0);   // A_FULL_EN, PPG_RDY_EN set to 1.
	max30102_Bus_Write(INTERRUPT_ENABLE1, 0xB0);   // A_FULL_EN + ALC_OVF_EN + PROX_INT_EN
	max30102_Bus_Write(INTERRUPT_ENABLE2, 0x02);   //TEMP RDY EN 0x02
	max30102_Bus_Write(FIFO_WR_PTR, 0x00);         //recommend to clear first
	max30102_Bus_Write(OVERFLOW_COUNTER, 0x00);    //recommend to clear first
	max30102_Bus_Write(FIFO_RD_PTR, 0x00);         //recommend to clear first

	//max30102_Bus_Write(FIFO_CONFIG, 0x0f);         //sample avg = 1, fifo rollover=false, fifo almost full = 17
	//max30102_Bus_Write(FIFO_CONFIG, 0xef);         //sample avg = 32, fifo rollover=false, fifo almost full = 17
	//max30102_Bus_Write(FIFO_CONFIG, 0x4A);         //sample avg = 4, fifo rollover=false, fifo almost full = 22
	max30102_Bus_Write(FIFO_CONFIG, 0x5A);         //sample avg = 4, fifo rollover=true, fifo almost full = 22


	max30102_Bus_Write(MODE_CONFIG, 0x03);         //SpO2 mode. RED and IR
	//max30102_Bus_Write(SPO2_CONFIG, 0x2B);         // SPO2_ADC range = 4096nA, 200Hz, LED pulseWidth (411uS) ,18bit
	max30102_Bus_Write(SPO2_CONFIG, 0x0B);         // SPO2_ADC range = 2048nA, 200Hz, LED pulseWidth (411uS) ,18bit

	max30102_Bus_Write(DIE_TEMP_CONFIG, 0x01);     //TEMP_EN set 1.

	//max30102_Bus_Write(LED1_PULSE_AMP,  0X40);     //Choose value for ~ 13mA for LED1(red)
	//max30102_Bus_Write(LED2_PULSE_AMP,  0X40);     //Choose value for ~ 13mA for LED2(ir)
	//max30102_Bus_Write(LED1_PULSE_AMP,  0X30);     //Choose value for ~ 6.5mA for LED1(red)
	//max30102_Bus_Write(LED2_PULSE_AMP,  0X20);     //Choose value for ~ 6.5mA for LED2(ir)
	max30102_Bus_Write(LED1_PULSE_AMP,  0X15);     //Choose value for ~ 6.5mA for LED1(red)
	max30102_Bus_Write(LED2_PULSE_AMP,  0X10);     //Choose value for ~ 6.5mA for LED2(ir)


	max30102_Bus_Write(PILOT_PA,  0X01);     		//PILOT_PA set ir led current
	max30102_Bus_Write(PROX_INT_THRESH,  0X01);     //PROX_INT_THRESH set ir therohold

	max30102_Bus_Read(INTERRUPT_STATUS1, &temp_a);          //clear int flag first.不然可能无法进中断
	max30102_Bus_Read(INTERRUPT_STATUS2, &temp_a);
}
