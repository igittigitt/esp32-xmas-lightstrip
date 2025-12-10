#include <stdio.h>
#include "driver/ledc.h" // LEDC driver
#include "freertos/FreeRTOS.h" // used for pdMS_TO_TICKS
#include "freertos/task.h" // used for vTaskDelay
#include "esp_log.h" // logging
#include "nvs_flash.h"
#include "nvs.h" // NVS
#include "iot_button.h" // button library

//-------------------------------------------------------------------------------//
// defines
//-------------------------------------------------------------------------------//

#define LONG_PRESS_TIME_MS 800
#define SHORT_PRESS_TIME_MS 200
#define BUTTON_GPIO 9
#define LED_DIM_DIRECTION_UP true
#define LED_DIM_DIRECTION_DOWN false

//-------------------------------------------------------------------------------//
// declare types
//-------------------------------------------------------------------------------//

// State machine states
typedef enum {
    STATE_OFF = 0,
    STATE_TIMER_RUN,
    STATE_TIMER_PROGRAM_ON,
    STATE_TIMER_PROGRAM_OFF,
    STATE_TIMER_ERASE,
    STATE_LED_DIM_UP,
    STATE_LED_DIM_DOWN,
    STATE_LED_FADE_UP,
    STATE_LED_FADE_DOWN,
    STATE_STORE_BRIGHTNESS,
    STATE_MAX
} state_machine_state_t;

// State machine events
typedef enum {
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK,
    BTN_EVENT_LONG_CLICK,
    BTN_EVENT_HOLD,
    BTN_EVENT_BUTTON_RELEASED
} state_machine_button_event_t;

//-------------------------------------------------------------------------------//
// global variables
//-------------------------------------------------------------------------------//

static const char* TAG = "LED_STRIP_CTRL";

uint8_t led_brightness = 0; // Helligkeit von 0 bis 255
bool led_dim_direction = LED_DIM_DIRECTION_UP; // true = up, false = down

static state_machine_state_t current_state = STATE_OFF;
static const char* state_names[] = {
    "OFF",
    "TIMER_RUN",
    "TIMER_PROGRAM_ON",
    "TIMER_PROGRAM_OFF",
    "LED_DIM_UP",
    "LED_DIM_DOWN",
    "LED_FADE_UP",
    "LED_FADE_DOWN",
    "STORE_BRIGHTNESS"
};

//-------------------------------------------------------------------------------//
// declare functions
//-------------------------------------------------------------------------------//

// LED
static void led_init(void);
static void led_set_brightness(uint8_t brightness);
static void led_blink(uint8_t times, uint32_t period_ms);
static void led_fade_off(uint32_t duration_ms);
static void led_fade_on(uint32_t duration_ms, uint8_t target_brightness);
static void led_pulse(uint8_t times, uint32_t pulse_duration_ms, uint8_t brightness);

// Button
static button_handle_t init_button(void);
static void on_press_repeat_done(void *arg, void *usr_data);
static void on_long_press_start(void *arg, void *usr_data);
static void on_long_press_hold(void *arg, void *usr_data);
static void on_long_press_up(void *arg, void *usr_data);
static void on_button_press_down(void *arg, void *usr_data);
static void on_button_press_up(void *arg, void *usr_data);

// State Machine
void state_machine_init(void);
void state_machine_transition(state_machine_state_t new_state);
state_machine_state_t state_machine_get_current(void);
void state_machine_handle_button_event(state_machine_button_event_t event);
static void on_state_enter_off(state_machine_state_t from_state);
static void on_state_enter_timer_run(state_machine_state_t from_state);
static void on_state_enter_timer_program_on(state_machine_state_t from_state);
static void on_state_enter_timer_program_off(state_machine_state_t from_state);
static void on_state_enter_timer_erase(state_machine_state_t from_state);
static void on_state_enter_dim_up(state_machine_state_t from_state);

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
    
    esp_err_t ret;

    // Repeated clicks (1..n)
    ret = iot_button_register_cb(btn, BUTTON_PRESS_REPEAT_DONE,  on_press_repeat_done, (void*)btn);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von PRESS_REPEAT_DONE: %s", esp_err_to_name(ret));
    }

    // Long press - Start
    ret = iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, on_long_press_start, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von LONG_PRESS_START: %s", esp_err_to_name(ret));
    }
    
    // Long press - Hold
    ret = iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, on_long_press_hold, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von LONG_PRESS_HOLD: %s", esp_err_to_name(ret));
    }
    
    // Long press - Release
    ret = iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, on_long_press_up, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von LONG_PRESS_UP: %s", esp_err_to_name(ret));
    }
    
    // Button was pressed down
    //iot_button_register_cb(btn, BUTTON_PRESS_DOWN, on_button_press_down, NULL);

    // Button was released up
    //iot_button_register_cb(btn, BUTTON_PRESS_UP, on_button_press_up, NULL);
    
    return btn;
}

// BUTTON callback functions

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
    
    //ESP_LOGI(TAG, "PRESS_REPEAT_DONE: Finale Klick-Serie mit %d Klick(s)", click_count);
    
    // Jetzt die verschiedenen Klick-Typen unterscheiden
    switch (click_count) {
        case 1:
            ESP_LOGI(TAG, "EINFACH-KLICK erkannt!");
            state_machine_handle_button_event(BTN_EVENT_SINGLE_CLICK);
            break;
            
        case 2:
            ESP_LOGI(TAG, "DOPPEL-KLICK erkannt!");
            state_machine_handle_button_event(BTN_EVENT_DOUBLE_CLICK);
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
 * @brief Callback wenn langer Druck beginnt
 */
static void on_long_press_start(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "LANGER DRUCK gestartet!");

    state_machine_handle_button_event(BTN_EVENT_LONG_CLICK);
}

/**
 * @brief Callback während Button gehalten wird (wiederholt sich)
 */
static void on_long_press_hold(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "⏳ LANGER DRUCK wird gehalten...");
    
    // Wird alle 500ms aufgerufen während Button gedrückt bleibt
    // Nützlich für kontinuierliche Aktionen
}

/**
 * @brief Callback wenn langer Druck endet
 */
static void on_long_press_up(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "LANGER DRUCK beendet");
}

/**
 * @brief Callback für Button gedrückt (optional)
 */
static void on_button_press_down(void *arg, void *usr_data)
{
    ESP_LOGD(TAG, "↓ Button wurde gedrückt");
}

/**
 * @brief Callback für Button losgelassen (optional)
 */
static void on_button_press_up(void *arg, void *usr_data)
{
    ESP_LOGD(TAG, "↑ Button wurde losgelassen");
}

//-------------------------------------------------------------------------------//
// STATE MACHINE functions
//-------------------------------------------------------------------------------//

/**
 * @brief Initialisiert die Zustandsmaschine
 */
void state_machine_init(void)
{
    // TODO: load initial state from NVS if needed
    current_state = STATE_OFF;
    ESP_LOGI(TAG, "State machine initialized to: %s", state_names[current_state]);
}

void state_machine_transition(state_machine_state_t new_state)
{
    if (new_state >= STATE_MAX) {
        ESP_LOGE(TAG, "Invalid state: %d", new_state);
        return;
    }
    
    state_machine_state_t old_state = current_state;
    current_state = new_state;
    ESP_LOGI(TAG, "State transition: %s -> %s", state_names[old_state], state_names[new_state]);
    
    switch(new_state) {
        case STATE_OFF:
            on_state_enter_off(old_state);
            break;
        case STATE_TIMER_RUN:
            on_state_enter_timer_run(old_state);
            break;
        case STATE_TIMER_PROGRAM_ON:
            on_state_enter_timer_program_on(old_state);
            break;
        case STATE_TIMER_PROGRAM_OFF:
            on_state_enter_timer_program_off(old_state);
            break;
        case STATE_TIMER_ERASE:
            on_state_enter_timer_erase(old_state);
            break;
        case STATE_LED_DIM_UP:
            on_state_enter_dim_up(old_state);
            break;

        // ... weitere Cases

        default:
            ESP_LOGE(TAG, "Unhandled state transistion: %d -> %d", old_state, new_state);
            return;
    }
}

state_machine_state_t state_machine_get_current(void) {
    return current_state;
}

void state_machine_handle_button_event(state_machine_button_event_t event)
{
    switch(current_state)
    {
        case STATE_OFF:
            if (event == BTN_EVENT_SINGLE_CLICK) {
                state_machine_transition(STATE_TIMER_RUN);
            }
            else if (event == BTN_EVENT_LONG_CLICK) {
                state_machine_transition(STATE_TIMER_ERASE);
            }
            break;
            
        case STATE_TIMER_RUN:
            if (event == BTN_EVENT_SINGLE_CLICK) {
                state_machine_transition(STATE_OFF);
            }
            else if (event == BTN_EVENT_DOUBLE_CLICK) {
                if (led_brightness == 0) {
                    state_machine_transition(STATE_TIMER_PROGRAM_ON);
                }
            }
            else if (event == BTN_EVENT_HOLD) {
                if (led_dim_direction == LED_DIM_DIRECTION_UP) {
                    state_machine_transition(STATE_LED_DIM_UP);
                } else {
                    state_machine_transition(STATE_LED_DIM_DOWN);
                }
            }
            break;

        case STATE_TIMER_PROGRAM_ON:
            if (event == BTN_EVENT_SINGLE_CLICK) {
                state_machine_transition(STATE_OFF);
            }
            else if (event == BTN_EVENT_DOUBLE_CLICK) {
                if (led_brightness > 0) {
                    state_machine_transition(STATE_TIMER_RUN);
                }
            }
            break;
        /*
        case STATE_TIMER_PROGRAM_OFF:
            if (event == BTN_EVENT_SINGLE_CLICK) {
                state_machine_transition(STATE_OFF);
            }
            break;
        */
        case STATE_LED_DIM_UP:
            if (event == BTN_EVENT_SINGLE_CLICK) {
                state_machine_transition(STATE_STORE_BRIGHTNESS);
            }
            // Release führt zurück zum vorherigen Zustand
            break;
            
        // ... weitere Zustandslogik
        
        default:
            break;
    }
}

// State-Enter Callbacks

static void on_state_enter_off(state_machine_state_t from_state) {
    if (from_state == STATE_TIMER_PROGRAM_ON) {
        ESP_LOGI(TAG, "Timer programming ABORTED!");
        led_blink(5, 100);
        led_set_brightness(0);
    }
    else {
        ESP_LOGI(TAG, "Entering OFF state - turning off LED");
        led_set_brightness(0);
    }
}

static void on_state_enter_timer_run(state_machine_state_t from_state) {
    if (from_state == STATE_TIMER_PROGRAM_ON) {
        ESP_LOGI(TAG, "Timer programming COMPLETED!");
        led_blink(3, 100);
    }
    else {
        ESP_LOGI(TAG, "Entering TIMER_RUN state");
    }
}

static void on_state_enter_timer_program_on(state_machine_state_t from_state) {
    ESP_LOGI(TAG, "Program TIMER START TIME");
    led_fade_on(2000, 100);
    led_blink(3, 250);
}

static void on_state_enter_timer_program_off(state_machine_state_t from_state) {
    ESP_LOGI(TAG, "Program TIMER STOP TIME");
    led_fade_off(2000);
}

static void on_state_enter_timer_erase(state_machine_state_t from_state) {
    ESP_LOGI(TAG, "All timers erased!");
    led_blink(5, 250);
}

static void on_state_enter_dim_up(state_machine_state_t from_state) {
    ESP_LOGI(TAG, "Entering LED_DIM_UP state");
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
    state_machine_init();

    // open NVS storage
    nvs_handle_t handle;
    nvs_open("storage", NVS_READWRITE, &handle);

    // restore led brightness from flash
    nvs_get_u8(handle, "led_brightness", &led_brightness);
    //led_set_brightness(led_brightness);
    ESP_LOGI(TAG, "Restored LED brightness from Flash: %u", led_brightness);

    led_set_brightness(0);
    while(1) {
       vTaskDelay(pdMS_TO_TICKS(1000));
    }
 
/*
    led_set_brightness(20); // 20% Helligkeit
    led_blink(10, 500); // 10 mal blinken mit 500ms Intervall
    led_set_brightness(100); // 100% Helligkeit
    led_blink(10, 250); // 10 mal blinken mit 500ms Intervall

    // store final brightness to flash
    nvs_set_u8(handle, "led_brightness", 50);
    nvs_commit(handle);
    ESP_LOGI(TAG, "Stored LED brightness into Flash: %u", 50);
*/
}