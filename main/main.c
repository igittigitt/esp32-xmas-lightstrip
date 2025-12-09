#include <stdio.h>
#include "driver/ledc.h" // LEDC driver
#include "freertos/FreeRTOS.h" // used for pdMS_TO_TICKS
#include "freertos/task.h" // used for vTaskDelay
#include "esp_log.h" // logging
#include "nvs_flash.h"
#include "nvs.h" // NVS
#include "iot_button.h" // button library

// defines
#define LONG_PRESS_TIME_MS 800
#define SHORT_PRESS_TIME_MS 200
#define BUTTON_GPIO 9

// global variables
uint8_t led_brightness = 0; // Helligkeit von 0 bis 255
static const char* TAG = "LED_STRIP_CTRL";

// declare functions
static void led_init(void);
static void led_set_brightness(uint8_t brightness);
static void led_blink(uint8_t times, uint32_t period_ms);
static void led_fade_off(uint32_t duration_ms);
static void led_fade_on(uint32_t duration_ms, uint8_t target_brightness);
static void led_pulse(uint8_t times, uint32_t pulse_duration_ms, uint8_t brightness);
static button_handle_t init_button(void);
static void on_single_click(void *arg, void *usr_data);
static void on_button_press_repeat(void *arg, void *usr_data);
static void on_press_repeat_done(void *arg, void *usr_data);
static void on_double_click(void *arg, void *usr_data);
static void on_multiple_click(void *arg, void *usr_data);
static void on_long_press_start(void *arg, void *usr_data);
static void on_long_press_hold(void *arg, void *usr_data);
static void on_long_press_up(void *arg, void *usr_data);
static void on_button_press_down(void *arg, void *usr_data);
static void on_button_press_up(void *arg, void *usr_data);

//-------------------------------------------------------------------------------//
// LED functions
//-------------------------------------------------------------------------------//

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
    led_brightness = brightness;
}

/**
 * Let the LED blink a number of times with a given period.
 * 
 * @param times Number of blinks
 * @param period_ms Period in milliseconds
 */
static void led_blink(uint8_t times, uint32_t period_ms) {
    uint8_t last_brightness = led_brightness;
    uint8_t brightness;
    if (last_brightness == 0) {
        brightness = 100;
    } else {
        brightness = 0;
    }
    for (uint8_t i = 1; i <= times; i += 1) {
        led_set_brightness(brightness);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
        led_set_brightness(last_brightness);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

static void led_fade_off(uint32_t duration_ms) {
    uint8_t start_brightness = led_brightness;
    for (int i = start_brightness; i >= 0; i--) {
        led_set_brightness(i);
        vTaskDelay(pdMS_TO_TICKS(duration_ms / start_brightness));
    }
}

static void led_fade_on(uint32_t duration_ms, uint8_t target_brightness) {
    for (int i = 0; i <= target_brightness; i++) {
        led_set_brightness(i);
        vTaskDelay(pdMS_TO_TICKS(duration_ms / target_brightness));
    }
}

static void led_pulse(uint8_t times, uint32_t pulse_duration_ms, uint8_t brightness) {
    for (uint8_t i = 0; i < times; i++) {
        led_set_brightness(brightness);
        vTaskDelay(pdMS_TO_TICKS(pulse_duration_ms));
        led_set_brightness(0);
        vTaskDelay(pdMS_TO_TICKS(pulse_duration_ms));
    }
}


//-------------------------------------------------------------------------------//
// BUTTON functions
//-------------------------------------------------------------------------------//

/**
 * @brief Initialisiert den Button mit allen Event-Handlern
 * @return Button-Handle oder NULL bei Fehler
 */
static button_handle_t init_button(void)
{
    // Button-Konfiguration
    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = LONG_PRESS_TIME_MS,
        .short_press_time = SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = BUTTON_GPIO,
            .active_level = 0,  // Active LOW (Button verbindet GPIO mit GND)
        },
    };
    
    // Button erstellen
    button_handle_t btn = iot_button_create(&btn_cfg);
    if (btn == NULL) {
        ESP_LOGE(TAG, "Fehler beim Erstellen des Buttons!");
        return NULL;
    }
    ESP_LOGI(TAG, "Button auf GPIO %d erstellt", BUTTON_GPIO);
    
    // Event-Handler registrieren
    esp_err_t ret;
    
    // Einfach-Klick
    ret = iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, on_single_click, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von SINGLE_CLICK: %s", esp_err_to_name(ret));
    }
    
/*
    // WHILE repeated clicks
    ret = iot_button_register_cb(btn, BUTTON_PRESS_REPEAT,  on_button_press_repeat, (void*)btn);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von PRESS_REPEAT: %s", esp_err_to_name(ret));
    }
*/

    // AFTER repeated clicks
    ret = iot_button_register_cb(btn, BUTTON_PRESS_REPEAT_DONE,  on_press_repeat_done, (void*)btn);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von PRESS_REPEAT_DONE: %s", esp_err_to_name(ret));
    }

/*
    // Doppel-Klick
    ret = iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, on_double_click, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von DOUBLE_CLICK: %s", esp_err_to_name(ret));
    }
*/

/*
    // Multiple clicks (2 or more)
    ret = iot_button_register_cb(btn, BUTTON_MULTIPLE_CLICK, on_multiple_click, (void*)btn);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von MULTIPLE_CLICK: %s", esp_err_to_name(ret));
    }
*/

    // Langer Druck - Start
    ret = iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, on_long_press_start, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von LONG_PRESS_START: %s", esp_err_to_name(ret));
    }
    
    // Langer Druck - Halten
    ret = iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, on_long_press_hold, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von LONG_PRESS_HOLD: %s", esp_err_to_name(ret));
    }
    
    // Langer Druck - Ende
    ret = iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, on_long_press_up, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von LONG_PRESS_UP: %s", esp_err_to_name(ret));
    }
    
    // Optional: Press Down/Up Events für Debugging
    iot_button_register_cb(btn, BUTTON_PRESS_DOWN, on_button_press_down, NULL);
    iot_button_register_cb(btn, BUTTON_PRESS_UP, on_button_press_up, NULL);
    
    ESP_LOGI(TAG, "Alle Event-Handler registriert");
    
    return btn;
}

// ============================================================================
// CALLBACK-FUNKTIONEN
// ============================================================================

/**
 * @brief Callback für Einfach-Klick
 */
static void on_single_click(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "╔════════════════════════════╗");
    ESP_LOGI(TAG, "║   EINFACH-KLICK erkannt!   ║");
    ESP_LOGI(TAG, "╚════════════════════════════╝");
    
    // Hier deine Aktion für Einfach-Klick
    // Beispiel: LED umschalten, Modus ändern, etc.
}

/**
 * @brief Universeller Click-Handler mit BUTTON_PRESS_REPEAT
 * Dieser Event wird bei jedem Button-Release getriggert und enthält
 * die Anzahl der Klicks in button_event_data_t
 */
static void on_button_press_repeat(void *arg, void *usr_data)
{
    button_handle_t btn = (button_handle_t)arg;
    
    // WICHTIG: Bei PRESS_REPEAT bekommen wir die Klick-Anzahl über get_repeat()
    int click_count = iot_button_get_repeat(btn);
    if (click_count < 2) {
        return;
    }
    ESP_LOGI(TAG, "%d KLICKS erkannt!", click_count);
    switch (click_count) {
        case 1:
            ESP_LOGI(TAG, "EINFACH-KLICK erkannt!");
            break;
            
        case 2:
            ESP_LOGI(TAG, "DOPPEL-KLICK erkannt!");
            break;
            
        case 3:
            ESP_LOGI(TAG, "DREIFACH-KLICK erkannt!");
            break;
            
        default:
            ESP_LOGI(TAG, "%d-FACH-KLICK erkannt!", click_count);
            break;
    }
}

/**
 * @brief Handler für BUTTON_PRESS_REPEAT_DONE
 * 
 * WICHTIG: Dieser Event wird nur EINMAL ausgelöst, nachdem die
 * gesamte Klick-Serie abgeschlossen ist!
 */
static void on_press_repeat_done(void *arg, void *usr_data)
{
    button_handle_t btn = (button_handle_t)arg;
    
    // Anzahl der Klicks in der Serie abrufen
    int click_count = iot_button_get_repeat(btn);
    
    ESP_LOGI(TAG, "PRESS_REPEAT_DONE: Finale Klick-Serie mit %d Klick(s)", click_count);
    
    // Jetzt die verschiedenen Klick-Typen unterscheiden
    switch (click_count) {
        case 1:
            ESP_LOGI(TAG, "╔════════════════════════════╗");
            ESP_LOGI(TAG, "║   EINFACH-KLICK erkannt!   ║");
            ESP_LOGI(TAG, "╚════════════════════════════╝");
            // Hier deine Aktion für Einfach-Klick
            break;
            
        case 2:
            ESP_LOGI(TAG, "╔════════════════════════════╗");
            ESP_LOGI(TAG, "║   DOPPEL-KLICK erkannt!    ║");
            ESP_LOGI(TAG, "╚════════════════════════════╝");
            // Hier deine Aktion für Doppel-Klick
            break;
            
        case 3:
            ESP_LOGI(TAG, "╔════════════════════════════╗");
            ESP_LOGI(TAG, "║  DREIFACH-KLICK erkannt!   ║");
            ESP_LOGI(TAG, "╚════════════════════════════╝");
            // Hier deine Aktion für Dreifach-Klick
            break;
            
        case 4:
            ESP_LOGI(TAG, "╔════════════════════════════╗");
            ESP_LOGI(TAG, "║  VIERFACH-KLICK erkannt!   ║");
            ESP_LOGI(TAG, "╚════════════════════════════╝");
            // Hier deine Aktion für Vierfach-Klick
            break;
            
        default:
            // Für 5+ Klicks
            ESP_LOGI(TAG, "╔════════════════════════════╗");
            ESP_LOGI(TAG, "║   %d-FACH-KLICK erkannt!    ║", click_count);
            ESP_LOGI(TAG, "╚════════════════════════════╝");
            // Hier deine Aktion für mehr Klicks
            break;
    }
}

/**
 * @brief Callback für Doppel-Klick
 */
static void on_double_click(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "╔════════════════════════════╗");
    ESP_LOGI(TAG, "║   DOPPEL-KLICK erkannt!    ║");
    ESP_LOGI(TAG, "╚════════════════════════════╝");
}

/**
 * @brief Callback für 5-fach Klick oder mehr
 */
static void on_multiple_click(void *arg, void *usr_data)
{
    button_handle_t btn = (button_handle_t)arg;
    
    // Anzahl der Klicks ermitteln
    int click_count = iot_button_get_repeat(btn);
    
    ESP_LOGI(TAG, "╔════════════════════════════╗");
    ESP_LOGI(TAG, "║   %d-FACH-KLICK erkannt!    ║", click_count);
    ESP_LOGI(TAG, "╚════════════════════════════╝");
}

/**
 * @brief Callback wenn langer Druck beginnt
 */
static void on_long_press_start(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "╔════════════════════════════╗");
    ESP_LOGI(TAG, "║  LANGER DRUCK gestartet!   ║");
    ESP_LOGI(TAG, "╚════════════════════════════╝");
}

/**
 * @brief Callback während Button gehalten wird (wiederholt sich)
 */
static void on_long_press_hold(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "⏳ Langer Druck wird gehalten...");
    
    // Wird alle 500ms aufgerufen während Button gedrückt bleibt
    // Nützlich für kontinuierliche Aktionen
}

/**
 * @brief Callback wenn langer Druck endet
 */
static void on_long_press_up(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "✓ Langer Druck beendet");
}

/**
 * @brief Callback für Button gedrückt (optional)
 */
static void on_button_press_down(void *arg, void *usr_data)
{
    ESP_LOGD(TAG, "↓ Button gedrückt");
}

/**
 * @brief Callback für Button losgelassen (optional)
 */
static void on_button_press_up(void *arg, void *usr_data)
{
    ESP_LOGD(TAG, "↑ Button losgelassen");
}

//-------------------------------------------------------------------------------//
// MAIN
//-------------------------------------------------------------------------------//

void app_main(void)
{
    // set loglevel
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "ESP32 Xmas Lightstrip Controller started");

    // do inits
    nvs_flash_init();
    led_init();
    init_button();

    // open NVS storage
    nvs_handle_t handle;
    nvs_open("storage", NVS_READWRITE, &handle);

    // restore led brightness from flash
    nvs_get_u8(handle, "led_brightness", &led_brightness);
    led_set_brightness(led_brightness);
    ESP_LOGI(TAG, "Restored LED brightness from Flash: %u", led_brightness);

    vTaskDelay(pdMS_TO_TICKS(2000));

    led_set_brightness(20); // 20% Helligkeit
    led_blink(10, 500); // 10 mal blinken mit 500ms Intervall
    led_set_brightness(100); // 100% Helligkeit
    led_blink(10, 250); // 10 mal blinken mit 500ms Intervall

    // store final brightness to flash
    nvs_set_u8(handle, "led_brightness", 50);
    nvs_commit(handle);
    ESP_LOGI(TAG, "Stored LED brightness into Flash: %u", 50);
}