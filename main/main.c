#include <stdio.h>
#include "driver/ledc.h"

static void led_init(void);
static void led_set_brightness(uint8_t brightness);

static void led_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_USE_RC_FAST_CLK  // RC_FAST_CLK läuft im Light Sleep weiter!
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = GPIO_NUM_13,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    //ESP_LOGI(TAG, "PWM initialisiert mit RC_FAST_CLK (Light-Sleep kompatibel)");
}

static void led_set_brightness(uint8_t brightness) {
    // Set duty cycle (0-255 for 8-bit resolution)
    uint32_t duty = (brightness * 255) / 255; // Scale brightness to duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void app_main(void)
{
    led_init();

    led_set_brightness(50); // 50% Helligkeit
}