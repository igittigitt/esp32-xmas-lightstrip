#ifndef LED_CONTROLLER
#define LED_CONTROLLER

#include <stdint.h>    // uint32_t, uint8_t, int32_t
#include <stdbool.h>   // bool
#include "driver/gpio.h"     // gpio_num_t
#include "freertos/FreeRTOS.h" // TaskHandle_t
#include "freertos/semphr.h" // SemaphoreHandle_t
#include "freertos/task.h" // used for vTaskDelay

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Define available LED modes
 */
typedef enum {
    LED_MODE_OFF,       // LED turns complety off
    LED_MODE_ON,        // Switch LED on at full brightness
    LED_MODE_BLINK,     // Blink LED between two brightnesses with the same on/off period
    LED_MODE_FLASH,     // Blink LED between two brightnesses with individual on/off periods
    LED_MODE_FADE,      // Fade LED from one brightness to another one in a given time
    LED_MODE_PULSE      // Like blink but sweep between both brightnesses in a given period
} LedMode_t;

// LED Konfiguration
typedef struct {
    LedMode_t mode;
    uint8_t   duty1;
    uint8_t   duty2;
    uint32_t  period1_ms;
    uint32_t  period2_ms;
    int32_t   count;          // -1 für endlos, sonst Anzahl Wiederholungen
} LedConfig_t;

extern QueueHandle_t ledQueue;

// API functions
esp_err_t led_controller(gpio_num_t pin);

static inline esp_err_t led_set_mode(const LedConfig_t *cfg) {
    if (!ledQueue || !cfg) return ESP_ERR_INVALID_STATE;
    xQueueOverwrite(ledQueue, cfg);
    return ESP_OK;
}

/* Komfort-Wrapper */
static inline esp_err_t led_off(void)
{
    return led_set_mode(&(LedConfig_t){
        .mode = LED_MODE_OFF
    });
}

static inline esp_err_t led_on(uint8_t duty)
{
    return led_set_mode(&(LedConfig_t){
        .mode = LED_MODE_ON,
        .duty1 = duty
    });
}

static inline esp_err_t led_blink(uint8_t duty1, uint8_t duty2, uint32_t period_ms)
{
    return led_set_mode(&(LedConfig_t){
        .mode = LED_MODE_BLINK,
        .duty1 = duty1,
        .duty2 = duty2,
        .period1_ms = period_ms
    });
}

static inline esp_err_t led_fade(uint8_t duty1, uint8_t duty2, uint32_t period_ms)
{
    return led_set_mode(&(LedConfig_t){
        .mode = LED_MODE_FADE,
        .duty1 = duty1,
        .duty2 = duty2,
        .period1_ms = period_ms,
        .count = -1
    });
}

static inline esp_err_t led_pulse(uint8_t duty1, uint8_t duty2, uint32_t period_ms)
{
    return led_set_mode(&(LedConfig_t){
        .mode = LED_MODE_PULSE,
        .duty1 = duty1,
        .duty2 = duty2,
        .period1_ms = period_ms,
        .count = -1
    });
}

#ifdef __cplusplus
}
#endif

#endif