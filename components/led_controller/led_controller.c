#include "led_controller.h"
#include <stdint.h>    // uint32_t, uint8_t, int32_t
//#include <stdbool.h>   // bool
#include "esp_log.h" // logging
#include "freertos/FreeRTOS.h" // TaskHandle_t
#include "freertos/semphr.h" // SemaphoreHandle_t
#include "freertos/task.h" // used for vTaskDelay
//#include "freertos/timers.h" // used for esp_timer
#include "freertos/queue.h" // QueueHandle_t
//#include "driver/gpio.h"     // gpio_num_t
#include "driver/ledc.h" // LEDC driver
#include <math.h>

//-------------------------------------------------------------------------------//
// TYPES
//-------------------------------------------------------------------------------//

// LED Controller Struktur
/*
typedef struct {
    gpio_num_t pin;
    LedConfig_t config;
    LedConfig_t newConfig;
    bool configChanged;
    SemaphoreHandle_t mutex;
    TaskHandle_t taskHandle;
    int32_t remainingCount;
} LedController_t;
*/

//-------------------------------------------------------------------------------//
// prototypes for internal only functions
//-------------------------------------------------------------------------------//

static void ledInit(gpio_num_t led_gpio);
static void LedSetDuty(uint8_t brightness);

/*
static void ledOff();
static void ledOn();
static void ledBlink(uint32_t period_ms, uint8_t duty, int32_t *remaining);
static void ledFade(uint32_t period_ms, uint8_t start_duty, uint8_t end_duty, int32_t *remaining);
static void ledPulse(uint32_t period_ms, uint8_t min_duty, uint8_t max_duty, int32_t *remaining);
static void led_set_config(LedMode_t mode, uint32_t period_ms, uint8_t duty_percent, uint8_t fade_start, uint8_t fade_end, uint8_t pulse_min, uint8_t pulse_max, int32_t count);
*/
static void led_controller_task(void *pvParameters);

//-------------------------------------------------------------------------------//

static const char* TAG = "LED_CTRL";

//static LedController_t ledCtrl;

QueueHandle_t ledQueue = NULL;

/* ---------------- Gamma-LUT ---------------- */
static uint8_t gamma8[256];
void gamma_init(void) {
    const float gamma = 2.2f;
    for (int i = 0; i < 256; i++) {
        float normalized = (float)i / 255.0f;
        float corrected = powf(normalized, gamma);
        gamma8[i] = (uint8_t)(corrected * 255.0f + 0.5f);
    }
}

//-------------------------------------------------------------------------------//
// EXTERNAL FUNCTIONS (API)
//-------------------------------------------------------------------------------//

esp_err_t led_controller(gpio_num_t led_gpio)
{
    gamma_init();
    
    // init PWM
    ledInit(led_gpio);

    // Init led-controller
    /*
    ledCtrl.pin = led_gpio;
    ledCtrl.config.mode = LED_MODE_OFF;
    ledCtrl.config.period_ms = 1000;
    ledCtrl.config.duty = 100;
    ledCtrl.config.count = -1;
    ledCtrl.configChanged = false;
    ledCtrl.remainingCount = -1;

    ledCtrl.mutex = xSemaphoreCreateMutex();
    if (ledCtrl.mutex == NULL) {
        ESP_LOGE(TAG, "Mutex could not be created");
        return ESP_ERR_NO_MEM;
    }
    */

    // create queue
    ledQueue = xQueueCreate(1, sizeof(LedConfig_t));
    configASSERT(ledQueue);

    // start LED controller task (daemon)
    BaseType_t result = xTaskCreate(led_controller_task, "led controller", 3072, NULL, 1, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Could not create led_controller task");
        //vSemaphoreDelete(ledCtrl.mutex);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

//-------------------------------------------------------------------------------//
/*
bool led_pattern_finished(void)
{
    return (ledCtrl.remainingCount == 0);
}
*/

/*
esp_err_t led_blink(uint8_t brightness1, uint8_t brightness2, uint32_t period_ms)
{
    LedConfig_t mode = {
        .duty = brightness2,
        .count = -1,
        .mode = LED_MODE_BLINK,
        .period_ms = period_ms
    };
    xQueueOverwrite(ledQueue, &mode);
    return ESP_OK;
}
*/

//-------------------------------------------------------------------------------//
// INTERNAL FUNCTIONS (static)
//-------------------------------------------------------------------------------//

/**
 * Configure LED PWM Controller (LEDC)
 * 
 * @param led_gpio GPIO number of LED connected to
 */
static void ledInit(gpio_num_t led_gpio)
{
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
        .gpio_num = led_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    
    ESP_LOGI(TAG, "LED Controller initialized (uses RC_FAST_CLK to be Light-Sleep compatible)");
}

/**
 * Set duty cycle (0-255 for 8-bit resolution)
 */
static void LedSetDuty(uint8_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t) duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

/*
// LED aus
static void ledOff()
{
    led_set_brightness(0);
}

// LED an
static void ledOn()
{
    led_set_brightness(255);
}

// Blink-Muster ausführen
static void ledBlink(uint32_t period_ms, uint8_t duty, int32_t *remaining)
{
    static bool state = false;
    
    if (*remaining == 0) {
        ledOff();
        return;
    }
    
    state = !state;
    if (state) {
        led_set_brightness(duty);
    } else {
        ledOff();
        if (*remaining > 0) (*remaining)--;
    }
    
    vTaskDelay(pdMS_TO_TICKS(period_ms / 2));
}

// Fade-Muster ausführen (von Start- zu End-Duty, 0-255)
static void ledFade(uint32_t period_ms, uint8_t start_duty, uint8_t end_duty, int32_t *remaining)
{
    if (*remaining == 0) {
        led_set_brightness(end_duty);  // Bei Ende auf Endwert bleiben
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }
    
    // Berechne optimale Anzahl Steps basierend auf Periode
    uint32_t steps = 100;
    uint32_t step_delay = period_ms / steps;
    
    // Mindestens 10ms pro Step für flüssige Animation
    if (step_delay < 10) {
        steps = period_ms / 10;
        if (steps < 10) steps = 10;  // Mindestens 10 Steps
        step_delay = period_ms / steps;
    }
    
    // Von start_duty zu end_duty überblenden
    for (uint32_t i = 0; i <= steps; i++) {
        if (ledCtrl.configChanged) return;
        
        // Lineare Interpolation zwischen start und end (0-255)
        int32_t duty = start_duty + (((int32_t)end_duty - (int32_t)start_duty) * (int32_t)i) / (int32_t)steps;
        
        // Stelle sicher dass Duty im gültigen Bereich ist
        if (duty < 0) duty = 0;
        if (duty > 255) duty = 255;
        
        led_set_brightness((uint8_t)duty);
        vTaskDelay(pdMS_TO_TICKS(step_delay));
    }
    
    if (*remaining > 0) (*remaining)--;
}

// Pulse-Muster ausführen (Herzschlag-Effekt mit min/max Duty, 0-255)
static void ledPulse(uint32_t period_ms, uint8_t min_duty, uint8_t max_duty, int32_t *remaining)
{
    if (*remaining == 0) {
        led_set_brightness(min_duty);  // Bei Ende auf Minimalwert
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }
    
    // Optimierte Steps für flüssige Animation
    uint32_t steps = 40;
    uint32_t step_delay = period_ms / (4 * steps);
    
    // Mindestens 5ms pro Step
    if (step_delay < 5) {
        steps = period_ms / 20;  // 20ms = 4 * 5ms
        if (steps < 10) steps = 10;
        step_delay = period_ms / (4 * steps);
    }
    
    // Erster Puls: min -> max -> min
    for (uint32_t i = 0; i <= steps; i++) {
        if (ledCtrl.configChanged) return;
        int32_t duty = min_duty + (((int32_t)max_duty - (int32_t)min_duty) * (int32_t)i) / (int32_t)steps;
        if (duty < 0) duty = 0;
        if (duty > 255) duty = 255;
        led_set_brightness((uint8_t)duty);
        vTaskDelay(pdMS_TO_TICKS(step_delay));
    }
    for (uint32_t i = steps; i > 0; i--) {
        if (ledCtrl.configChanged) return;
        int32_t duty = min_duty + (((int32_t)max_duty - (int32_t)min_duty) * (int32_t)i) / (int32_t)steps;
        if (duty < 0) duty = 0;
        if (duty > 255) duty = 255;
        led_set_brightness((uint8_t)duty);
        vTaskDelay(pdMS_TO_TICKS(step_delay));
    }
    
    // Kurze Pause zwischen den Pulsen
    vTaskDelay(pdMS_TO_TICKS(period_ms / 8));
    if (ledCtrl.configChanged) return;
    
    // Zweiter Puls: min -> max -> min
    for (uint32_t i = 0; i <= steps; i++) {
        if (ledCtrl.configChanged) return;
        int32_t duty = min_duty + (((int32_t)max_duty - (int32_t)min_duty) * (int32_t)i) / (int32_t)steps;
        if (duty < 0) duty = 0;
        if (duty > 255) duty = 255;
        led_set_brightness((uint8_t)duty);
        vTaskDelay(pdMS_TO_TICKS(step_delay));
    }
    for (uint32_t i = steps; i > 0; i--) {
        if (ledCtrl.configChanged) return;
        int32_t duty = min_duty + (((int32_t)max_duty - (int32_t)min_duty) * (int32_t)i) / (int32_t)steps;
        if (duty < 0) duty = 0;
        if (duty > 255) duty = 255;
        led_set_brightness((uint8_t)duty);
        vTaskDelay(pdMS_TO_TICKS(step_delay));
    }
    
    // Längere Pause nach dem Doppel-Puls
    vTaskDelay(pdMS_TO_TICKS(period_ms / 4));
    
    if (*remaining > 0) (*remaining)--;
}
*/

/**
 * 
 */
static void led_controller_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Controller task started");

    LedConfig_t ledConfig = { .mode = LED_MODE_OFF };
    LedConfig_t newLedConfig;

    TickType_t nextChange = xTaskGetTickCount(); // nächste LED-Aktualisierung
    bool ledState = false;                        // AN/OFF für Blink/Flash
    uint8_t fadeDuty = 0;                         // aktueller Duty für Fade/Pulse

    while(1)
    {
        TickType_t now = xTaskGetTickCount();
        TickType_t waitTicks = 1; // Default minimal

        switch (ledConfig.mode) {
            case LED_MODE_OFF:
                LedSetDuty(0);
                waitTicks = portMAX_DELAY;
                break;

            case LED_MODE_ON:
                if (!ledConfig.duty1) ledConfig.duty1 = 255;
                LedSetDuty(gamma8[ledConfig.duty1]);
                waitTicks = portMAX_DELAY;
                break;

            case LED_MODE_BLINK:
                {
                    uint32_t on_time  = ledConfig.period1_ms;
                    uint32_t off_time = ledConfig.period2_ms ? ledConfig.period2_ms : ledConfig.period1_ms;
                    if (on_time + off_time == 0) {
                        LedSetDuty(0);
                        waitTicks = portMAX_DELAY;
                        break;
                    }

                    // Catch-up Loop: erledigte Toggles nachholen
                    while ((int32_t)(now - nextChange) >= 0) {
                        if (ledState) {
                            LedSetDuty(gamma8[ledConfig.duty2]);
                            nextChange += pdMS_TO_TICKS(off_time);
                            ledState = false;
                        } else {
                            LedSetDuty(gamma8[ledConfig.duty1]);
                            nextChange += pdMS_TO_TICKS(on_time);
                            ledState = true;
                        }
                    }
                    waitTicks = nextChange - now;
                }
                break;

            case LED_MODE_FLASH:
                {
                    uint32_t on_time  = ledConfig.period1_ms;
                    uint32_t off_time = ledConfig.period2_ms ? ledConfig.period2_ms : ledConfig.period1_ms;
                    if (on_time + off_time == 0) {
                        LedSetDuty(0);
                        waitTicks = portMAX_DELAY;
                        break;
                    }

                    while ((int32_t)(now - nextChange) >= 0) {
                        if (ledState) {
                            LedSetDuty(gamma8[ledConfig.duty2]);
                            nextChange += pdMS_TO_TICKS(off_time);
                            ledState = false;
                        } else {
                            LedSetDuty(gamma8[ledConfig.duty1]);
                            nextChange += pdMS_TO_TICKS(on_time);
                            ledState = true;
                        }
                    }
                    waitTicks = nextChange - now;
                }
                break;

            case LED_MODE_FADE:
            case LED_MODE_PULSE:
                {
                    // Lineare Fade / Pulse Implementation
                    uint8_t start = ledConfig.duty1;
                    uint8_t end   = ledConfig.duty2 ? ledConfig.duty2 : 255;
                    uint8_t step  = 5;
                    int total_steps = (abs(end - start) + step - 1) / step;
                    int step_ms = total_steps ? ledConfig.period1_ms / total_steps : ledConfig.period1_ms;

                    while ((int32_t)(now - nextChange) >= 0) {
                        LedSetDuty(gamma8[fadeDuty]);
                        if (!ledState) { // hochzählen
                            fadeDuty += step;
                            if (fadeDuty >= end) {
                                fadeDuty = end;
                                ledState = true; // Start runterzählen
                            }
                        } else { // runterzählen
                            fadeDuty -= step;
                            if (fadeDuty <= start) {
                                fadeDuty = start;
                                ledState = false; // Start hochzählen
                            }
                        }
                        nextChange += pdMS_TO_TICKS(step_ms);
                    }

                    waitTicks = nextChange - now;
                }
                break;
        }

        // Warte bis nächster Effekt oder neue Nachricht
        if (xQueueReceive(ledQueue, &newLedConfig, waitTicks) == pdPASS) {
            ledConfig = newLedConfig;
            nextChange = xTaskGetTickCount();
            ledState = false;
            fadeDuty = ledConfig.duty1;
        }
    }
}
