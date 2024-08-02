#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

#include "ssd1306.h"
#include "font8x8_basic.h"

#define THRESH_LOW -1.5
#define THRESH_HIGH 0

#define TAG "SSD1306"
SSD1306_t dev;



int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;

static esp_timer_handle_t fire_timer;
static bool fire_on = false;
static bool below_neg_15 = false;
bool t_detect_flag = true;


bool trough_detected(float curr_val, float prev_val)
{
	if(curr_val > prev_val)
		return true;
	return false;
}

void fire_off_callback(void* arg) {
    printf("-----Fire OFF (Timer)----\n");
    ssd1306_clear_screen(&dev, false);
    fire_on = false;
    below_neg_15 = false;
    t_detect_flag = true;
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void gait_detection(void *params) {
    float angle_roll = 0;
    float prev_value = 0.0;
	float curr_value = 0.0;
    float fire_angle = 0;
    while (1) {
        mpu6050_read_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        angle_roll = atan2(accel_z, sqrt(accel_x * accel_x + accel_y * accel_y)) * 180.0 / M_PI;
        prev_value = curr_value;
		curr_value = angle_roll;
        if(curr_value < THRESH_LOW)
		{
			if(trough_detected(curr_value, prev_value) && t_detect_flag && !fire_on && angle_roll<-20)
			{
				//stim on
                below_neg_15 = true; 
				t_detect_flag = false;
                fire_angle = prev_value+5.0;
                // printf("Previous Value: %.2f   Fire angle%.2f\n",prev_value,fire_angle);

			}
		}


        if (below_neg_15 && angle_roll > fire_angle && !fire_on) {
            ssd1306_display_text_x3(&dev, 0, "Fire", 5, false);         
            printf("-----Fire ON----\n");  
            
            fire_on = true;
            below_neg_15 = false; // Reset monitoring flag after firing starts
            esp_timer_start_once(fire_timer, 750000); // 1 second in microseconds
        }

        if (angle_roll > (fire_angle+50.0) && fire_on) {
            printf("-----Fire OFF----\n");
            ssd1306_clear_screen(&dev, false);
            fire_on = false;
            t_detect_flag = true;
            esp_timer_stop(fire_timer);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100 milliseconds
    }
}

void app_main(void) {
#if CONFIG_I2C_INTERFACE
    ESP_LOGI(TAG, "INTERFACE is i2c");
    ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
    ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
    ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_FLIP
    dev._flip = true;
    ESP_LOGW(TAG, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
    ESP_LOGI(TAG, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);
    mpu6050_init();
#endif // CONFIG_SSD1306_128x64

#if CONFIG_SSD1306_128x32
    ESP_LOGI(TAG, "Panel is 128x32");
    ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

    ssd1306_contrast(&dev, 0xff);
    ssd1306_clear_screen(&dev, false);

    const esp_timer_create_args_t fire_timer_args = {
        .callback = &fire_off_callback,
        .name = "fire_timer"
    };

    if (esp_timer_create(&fire_timer_args, &fire_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create fire timer");
        return;
    }

    xTaskCreate(gait_detection, "Gait", 8000, NULL, 1, NULL);
}
