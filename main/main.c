


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define  I2C_FREQ       100000
#define  I2C_GPIO_SDA   GPIO_NUM_17
#define  I2C_GPIO_SCL   GPIO_NUM_18
#define  I2C_PORT_NUM   I2C_NUM_1
#define  ACK_CHECK_EN    0x1
#define  ACK_CHECK_DES   0x0
#define  ACK_VAL         0x0
#define  NACK_VAL        0x1
#define ADS1110_ADDR 0x48
#define I2C_MASTER_TX_BUF_DISABLE  0         // غیرفعال کردن بافر ارسال
#define I2C_MASTER_RX_BUF_DISABLE  0         // غیرفعال کردن بافر دریافت

static const char *TAG = "Ads1110";
int ret_ads1110;
uint8_t Upper_data,Lower_data,configRegister;
int16_t Result ;

float ads1110(){

  
  uint8_t data[2];
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ADS1110_ADDR << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  int16_t raw_value = (data[0] << 8) | data[1];

  float voltage = (raw_value * 4.096) / 32768.0;  

  return  voltage ;

}


void Write_ads1110(){

      i2c_cmd_handle_t cmd = i2c_cmd_link_create();

      ret_ads1110 = i2c_master_start(cmd);


      ret_ads1110 = i2c_master_write_byte(cmd, (0x48 << 1) | I2C_MASTER_WRITE , ACK_CHECK_EN );                 //address   1001000 | I2C_MASTER_WRITE
      if(ret_ads1110 == ESP_OK){ESP_LOGE(TAG, "i2c_ok_address : %d",ret_ads1110);}


      ret_ads1110 = i2c_master_write_byte(cmd, 0x8F , ACK_CHECK_EN );               //CONFIGURATION REGISTER 
      if(ret_ads1110 == ESP_OK){ESP_LOGE(TAG, "i2c_ok_CONFIGURATION : %d",ret_ads1110);}


      ret_ads1110 = i2c_master_stop(cmd);
     
      ret_ads1110 = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);

      if(ret_ads1110 == ESP_OK){ESP_LOGE(TAG, "i2c_ok_write : %d",ret_ads1110);}

      i2c_cmd_link_delete(cmd);

      usleep(1000);


}


void Read_ads1110(uint8_t *Lower_data ,uint8_t *Upper_data,uint8_t *configRegister)
{

     i2c_cmd_handle_t cmd = i2c_cmd_link_create();

     ret_ads1110 = i2c_master_start(cmd);
    
     ret_ads1110 = i2c_master_write_byte(cmd, (0x48 << 1) | I2C_MASTER_READ , ACK_CHECK_EN  );      // address      

     ret_ads1110 = i2c_master_read_byte(cmd, Upper_data  , 1);

     ret_ads1110 = i2c_master_read_byte(cmd, Lower_data  , 0);

     ret_ads1110 = i2c_master_read_byte(cmd, configRegister  , 0);

     ret_ads1110 = i2c_master_stop(cmd);

     ret_ads1110 = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);

     if(ret_ads1110 == ESP_OK){ESP_LOGE(TAG, "i2c_okread : %d",ret_ads1110); }
     
     i2c_cmd_link_delete(cmd);

     usleep(1000);

}



void app_main()
{

i2c_config_t  i2c_conf ={


        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_GPIO_SDA,
        .scl_io_num = I2C_GPIO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
		.clk_flags = 0,

        
    };
//___________________________________________________________________
i2c_param_config(I2C_PORT_NUM ,  &i2c_conf);
//___________________________________________________________________
//i2c_driver_install(I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0);
i2c_driver_install(I2C_PORT_NUM, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
//___________________________________________________________________

Write_ads1110();

while (1)
{
    
       Read_ads1110(&Upper_data,&Lower_data,&configRegister);
    
       Result = ( Lower_data << 8) | Upper_data  ;

       float voltage = (Result * 4.096) / 32768.0;


   


       printf("  Voltage   : %f\n",voltage);  // ads1110()

       ESP_LOGE(TAG, "____________________: ");

       vTaskDelay(1000 / portTICK_PERIOD_MS);

    }




}





















