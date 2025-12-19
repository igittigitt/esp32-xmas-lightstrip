#include <stdio.h>
#include <time.h> // (exports "struct tm"
#include "driver/ledc.h" // LEDC driver
#include "freertos/FreeRTOS.h" // used for pdMS_TO_TICKS
#include "freertos/task.h" // used for vTaskDelay
#include "freertos/timers.h" // used for esp_timer
#include "esp_log.h" // logging
//#include "nvs_flash.h"
//#include "nvs.h" // NVS
#include "iot_button.h" // button library
#include "esp_sleep.h" // sleep modes
#include "driver/uart.h"

// I2C libraries
#include "i2cdev.h" // I2C (old I2C API, currently needed by ds3231)
//#include "driver/i2c_master.h" // I2C Master driver (new API)
#include "ds3231.h" // DS3231 RTC (I2C realtime clock) <- OLD API
#include "at24c.h" // AT24C EEPROM (I2C EEPROM) <- OLD API

//-------------------------------------------------------------------------------//
// defines
//-------------------------------------------------------------------------------//

#define BUTTON_GPIO 9
#define LONG_PRESS_TIME_MS 1000 // button must be pressed at least this time to detect "long press"
#define SHORT_PRESS_TIME_MS 300 // debounce time for short presses

#define LED_DIM_DIRECTION_UP true
#define LED_DIM_DIRECTION_DOWN false
#define LED_DIM_SPEED_MS 3000 // Zeit für komplettes Dimmen (auf oder ab) in ms

#define I2C_MASTER_FREQ_HZ   100000  // 100kHz needed to drive DS3231 (fast) and AT24C (slow)
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_11
#define I2C_MASTER_SDA_IO GPIO_NUM_12

#define DS3231_ADDR 0x68 // DS3231 I2C Adresse
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_CONTROL_A1IE (1 << 0)
#define DS3231_CONTROL_A2IE (1 << 1)
#define DS3231_CONTROL_INTCN (1 << 2)
#define DS3231_CONTROL_BBSQW (1 << 6)
// GPIO für DS3231 VCC (Power-Control)
#define DS3231_VCC_PIN GPIO_NUM_10
// Interrupt-Pin vom DS3231 SQW/INT
#define DS3231_ALARM_INT_PIN GPIO_NUM_14

#define AT24C_ADDR 0x57 // AT24C32 I2C Adresse (A0,A1,A2 auf HIGH)
#define AT24C_PAGE_SIZE  32
#define AT24C_CAPACITY   4096
#define AT24C_ADDR_WIDTH 2
//
#define AT24C_TIMER_DATA_START_ADDR 0x0004 // where timer data starts in EEPROM

#define MAX_TIMERS 8

//-------------------------------------------------------------------------------//
// declare types
//-------------------------------------------------------------------------------//

// State machine states
typedef enum {
    STATE_OFF = 0,
    STATE_RESET,
    STATE_TIMER,
    STATE_TIMER_DIM,
    STATE_PROGRAM,
    STATE_PROGRAM_DIM,
    STATE_MAX
} state_machine_state_t;

// State machine events
typedef enum {
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK,
    BTN_EVENT_TRIPLE_CLICK,
    BTN_EVENT_LONG_CLICK_START,
    BTN_EVENT_LONG_CLICK_HOLD,
    BTN_EVENT_LONG_CLICK_END
} state_machine_button_event_t;

// Timer structure (uses packed attribute to avoid padding for better EEPROM storage)
typedef struct __attribute__((packed)) {
  uint8_t hour;        // 0-23
  uint8_t minute;      // 0-59
  uint8_t brightness;  // LED Helligkeitswert
  bool active;        // Timer aktiv/inaktiv
} led_timer_t;

//-------------------------------------------------------------------------------//
// global variables/handles
//-------------------------------------------------------------------------------//

static const char* TAG = "LED_STRIP_CTRL";

uint8_t led_brightness = 0; // Helligkeit von 0 bis 255
bool led_dim_direction = LED_DIM_DIRECTION_UP; // true = up, false = down

static state_machine_state_t current_state = STATE_OFF;
static const char* state_names[] = {
    "OFF",
    "RESET",
    "TIMER",
    "TIMER_DIM",
    "PROGRAM",
    "PROGRAM_DIM"
};

static TimerHandle_t reset_timer = NULL;
static bool reset_confirmed = false;

static led_timer_t timers[MAX_TIMERS]; // initializes all timer to 0 (inactive)
static led_timer_t new_timer; // store start time of current program

// I2C device handles
static i2c_dev_t ds3231_dev;
static at24c_handle_t at24c_dev;

//-------------------------------------------------------------------------------//
// declare functions (prototypes)
//-------------------------------------------------------------------------------//

// LED
static void led_init(void);
static void led_set_brightness(uint8_t brightness);
static void led_blink(uint8_t times, uint32_t period_ms);
static void led_fade_off(uint32_t duration_ms, uint8_t start_brightness);
static void led_fade_on(uint32_t duration_ms, uint8_t target_brightness);
static void led_pulse(uint8_t times, uint32_t pulse_duration_ms, uint8_t start_end_brightness);
uint8_t led_dim_dirchange_pause = 0;

// Button
static button_handle_t button_init(void);
static void on_press_repeat_done(void *arg, void *usr_data);
static void on_long_press_start(void *arg, void *usr_data);
static void on_long_press_hold(void *arg, void *usr_data);
static void on_long_press_up(void *arg, void *usr_data);
static void on_button_press_down(void *arg, void *usr_data);
static void on_button_press_up(void *arg, void *usr_data);

// State Machine
static void state_machine_init(void);
static void state_machine_transition(state_machine_state_t new_state, state_machine_button_event_t event);
static void state_machine_handle_button_event(state_machine_button_event_t event);
static void reset_timer_callback(TimerHandle_t xTimer);
//
static void on_state_enter_off(state_machine_button_event_t event);
static void on_state_exit_off(state_machine_state_t from_state, state_machine_button_event_t event);
static void on_state_enter_reset(state_machine_button_event_t event);
static void on_state_exit_reset(state_machine_state_t from_state, state_machine_button_event_t event);
static void on_state_enter_timer(state_machine_button_event_t event);
static void on_state_exit_timer(state_machine_state_t from_state, state_machine_button_event_t event);
// static void on_state_enter_timer_dim(state_machine_button_event_t event);
// static void on_state_exit_timer_dim(state_machine_state_t from_state, state_machine_button_event_t event);
static void on_state_enter_program(state_machine_button_event_t event);
static void on_state_exit_program(state_machine_state_t from_state, state_machine_button_event_t event);
static void on_state_enter_program_dim(state_machine_button_event_t event);
static void on_state_exit_program_dim(state_machine_state_t from_state, state_machine_button_event_t event);

// I2C
esp_err_t i2c_bus_init(void);

// RTC
esp_err_t ds3231_i2c_init(void);
void test_ds3231(void);
static void ds3231_init_power_gpio(void);
static void ds3231_interrupt_gpio_init(void);
static void ds3231_power_on(void);
static void ds3231_power_off(void);
static void ds3231_print_time(const struct tm *now, const char *prefix);
//static void ds3231_init_interrupt_gpio(void);
//static void ds3231_configure_alarms(void);
//static void ds3231_clear_alarms(void);
static void ds3231_configure_wakeup(void);

// AT24C EEPROM
esp_err_t at24c_i2c_init(void);

// Timer/Scheduler
uint8_t count_timers(void);
esp_err_t add_timer(led_timer_t timer);
esp_err_t get_next_timer(const struct tm *now, led_timer_t *timer);
esp_err_t get_previous_timer(const struct tm *now, led_timer_t *timer);
esp_err_t set_ds3231_alarm(uint8_t hour, uint8_t minute);
esp_err_t clear_ds3231_alarms(void);
esp_err_t delete_all_timers(void);
void print_timers(void);
esp_err_t load_timers_from_eeprom(void);
esp_err_t save_timers_to_eeprom(void);
static int time_to_minutes(uint8_t hour, uint8_t minute);
static int minutes_until(int now_minutes, int target_minutes);
static int minutes_since(int now_minutes, int target_minutes);
static bool is_time_before(const led_timer_t *t1, const led_timer_t *t2);
static void sort_timers(void);

// MISC 
static void delay_ms(uint32_t wait_ms);

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
        brightness = 255;
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

/**
 * @brief Fades the LED off from current brightness to 0 over duration_ms
 * @param duration_ms Duration of the fade in milliseconds
 * @param start_brightness Starting brightness (0-100)
 */
static void led_fade_off(uint32_t duration_ms, uint8_t start_brightness)
{
    if (start_brightness > 0)
    {
        int wait_ms = duration_ms / start_brightness;
        for (int i = start_brightness; i > 0; i--) {
            led_set_brightness(i);
            delay_ms(wait_ms);
        }
        led_set_brightness(0);
    }
}

/**
 * @brief Fades the LED on from 0 to target_brightness over duration_ms
 * 
 * @param duration_ms Duration of the fade in milliseconds
 * @param target_brightness Target brightness (0-255)
 */
static void led_fade_on(uint32_t duration_ms, uint8_t target_brightness)
{
    if (target_brightness > led_brightness)
    {
        int wait_ms = duration_ms / (target_brightness - led_brightness);
        for (int i = 0; i <= target_brightness; i++) {
            led_set_brightness(i);
            delay_ms(wait_ms);
        }
    }
}

/**
 * @brief Pulses the LED a number of times with given period and brightness
 * @param times Number of pulses
 * @param pulse_period_ms Period of each pulse in milliseconds
 */
static void led_pulse(uint8_t times, uint32_t pulse_period_ms, uint8_t start_end_brightness)
{
    for (uint8_t i = 0; i < times; i++) {
        led_fade_off(pulse_period_ms, start_end_brightness);
        led_fade_on(pulse_period_ms, start_end_brightness);
    }
}


//-------------------------------------------------------------------------------//
// BUTTON functions
//-------------------------------------------------------------------------------//

/**
 * @brief Initialisiert den Button mit allen Event-Handlern
 * @return Button-Handle oder NULL bei Fehler
 */
static button_handle_t button_init(void)
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
            state_machine_handle_button_event(BTN_EVENT_TRIPLE_CLICK);
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
    state_machine_handle_button_event(BTN_EVENT_LONG_CLICK_START);
}

/**
 * @brief Callback called every ???ms as long as button keeps pressed
 */
static void on_long_press_hold(void *arg, void *usr_data)
{
    //ESP_LOGI(TAG, "⏳ LANGER DRUCK wird gehalten...");
    state_machine_handle_button_event(BTN_EVENT_LONG_CLICK_HOLD);
}

/**
 * @brief Callback wenn langer Druck endet
 */
static void on_long_press_up(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "LANGER DRUCK beendet");
    state_machine_handle_button_event(BTN_EVENT_LONG_CLICK_END);
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
static void state_machine_init(void)
{
    // One-shot Timer, 5000ms
    reset_timer = xTimerCreate(
        "reset_timer",           // Name
        pdMS_TO_TICKS(5000),    // Period: 5 Sekunden
        pdFALSE,                // Auto-reload: nein (one-shot)
        NULL,                   // Timer ID
        reset_timer_callback    // Callback
    );

    // TODO: load initial state from NVS if needed
    current_state = STATE_OFF;
    ESP_LOGI(TAG, "State machine initialized to: %s", state_names[current_state]);
}

/**
 * @brief Führt einen Zustandsübergang durch
 * 
 * @param new_state Neuer Zustand
 * @param event Ereignis, das den Übergang ausgelöst hat
 */
static void state_machine_transition(state_machine_state_t new_state, state_machine_button_event_t event)
{
    if (new_state >= STATE_MAX) {
        ESP_LOGE(TAG, "Invalid state: %d", new_state);
        return;
    }

        // call old state exit-handlers
    ESP_LOGI(TAG, "Exiting %s state", state_names[current_state]);
    switch(current_state) {
        case STATE_OFF:
            on_state_exit_off(new_state, event);
            break;
        case STATE_RESET:
            on_state_exit_reset(new_state, event);
            break;
        case STATE_TIMER:
            on_state_exit_timer(new_state, event);
            break;
        /*
        case STATE_TIMER_DIM:
            on_state_exit_timer_dim(new_state, event);
            break;
        */
        case STATE_PROGRAM:
            on_state_exit_program(new_state, event);
            break;
        /*
        case STATE_PROGRAM_DIM:
            on_state_exit_timer_program_dim(new_state, event);
            break;
        */
        default:
            ESP_LOGE(TAG, "Unhandled exit-handler for state: %d", current_state);
            return;
    }

    // call new state entry-handlers
    ESP_LOGI(TAG, "Entering %s state", state_names[new_state]);
    switch(new_state) {
        case STATE_OFF:
            on_state_enter_off(event);
            break;
        case STATE_RESET:
            on_state_enter_reset(event);
            break;
        case STATE_TIMER:
            on_state_enter_timer(event);
            break;
        /*
        case STATE_TIMER_DIM:
            on_state_enter_timer_dim(event);
            break;
        */
        case STATE_PROGRAM:
            on_state_enter_program(event);
            break;
        /*
        case STATE_PROGRAM_DIM:
            on_state_enter_program_dim(event);
            break;
        */
        default:
            ESP_LOGE(TAG, "Unhandled entry-hanlder for state: %d", new_state);
            return;
    }

    ESP_LOGI(TAG, "Transitioning from %s to %s", state_names[current_state], state_names[new_state]);
    current_state = new_state;
}

/**
 * @brief State Machine business logic
 * 
 * - A "single click" switches between OFF and TIMER mode (LED fades on or off accordingly) If no timer was programmed yet, LED blinks quickly 5 times
 * - A "long click" held for 5s in OFF mode deletes all stored timers
 * - A "double click" in TIMER mode enters PROGRAM mode and stores current time as potential start time of a new timer
 * - Another "double click" in PROGRAM mode creates a new timer using the start time, the current time as stop time and the current brightness. After storing the LED fades down to 0%
 * - A "triple click" in PROGRAM mode aborts programming and returns to TIMER mode
 * - A "long click" in PROGRAM mode enters PRGRAM_DIM mode which cycles LED brightness up and down to the limits until button is released
 * 
 * @param event Button event triggering the SM
 */
static void state_machine_handle_button_event(state_machine_button_event_t event)
{
    switch(current_state)
    {
        case STATE_OFF:
            if (event == BTN_EVENT_SINGLE_CLICK) {
                state_machine_transition(STATE_TIMER, event);
            }
            else if (event == BTN_EVENT_LONG_CLICK_START) {
                state_machine_transition(STATE_RESET, event);
            }
            break;

        case STATE_RESET:
            if (event == BTN_EVENT_LONG_CLICK_HOLD) {
                if (!reset_confirmed) {
                    led_blink(2, 100); // Feedback während des Wartens
                }
            }
            else if (event == BTN_EVENT_LONG_CLICK_END) {
                state_machine_transition(STATE_OFF, event);
            }
            break;

        case STATE_TIMER:
            if (event == BTN_EVENT_SINGLE_CLICK) {
                state_machine_transition(STATE_OFF, event);
            }
            else if (event == BTN_EVENT_DOUBLE_CLICK) {
                state_machine_transition(STATE_PROGRAM, event);
            }
            break;
/*
        case STATE_TIMER_DIM:
            if (event == BTN_EVENT_LONG_CLICK_HOLD) {
                // adjust brightness
                if (led_dim_direction == LED_DIM_DIRECTION_UP) {
                    if (led_brightness < 255) {
                        led_set_brightness(led_brightness + 5);
                    } else {
                        led_dim_direction = LED_DIM_DIRECTION_DOWN;
                    }
                } else {
                    if (led_brightness > 10) {
                        led_set_brightness(led_brightness - 5);
                    } else {
                        led_dim_direction = LED_DIM_DIRECTION_UP;
                    }
                }
            }
            else if (event == BTN_EVENT_LONG_CLICK_END) {
                state_machine_transition(STATE_PROGRAM, event);
            }
            break;
*/

        case STATE_PROGRAM:
            if (event == BTN_EVENT_DOUBLE_CLICK) {
                state_machine_transition(STATE_TIMER, event);
            }
            else if (event == BTN_EVENT_TRIPLE_CLICK) {
                state_machine_transition(STATE_TIMER, event);
            }
            else if (event == BTN_EVENT_LONG_CLICK_START) {
                ESP_LOGI(TAG, "Start dimming ...");
            }
            else if (event == BTN_EVENT_LONG_CLICK_HOLD) {
                // re-adjust brightness (up/down)
                // NOTE: event will be called repeatedly every 500ms as long as button is held
                if (led_dim_direction == LED_DIM_DIRECTION_UP) {
                    if (led_brightness + 3 >= 255) {
                        // clamp to max
                        if (led_dim_dirchange_pause++ < 100) {
                            return; // skip this step to slow down direction change
                        }
                        led_brightness = 255;
                        led_dim_direction = LED_DIM_DIRECTION_DOWN;
                        led_dim_dirchange_pause = 0;
                    } else {
                        led_brightness += 3;
                        led_set_brightness(led_brightness);
                    }
                } else {
                    if (led_brightness - 3 <= 10) {
                        // clamp to min
                        if (led_dim_dirchange_pause++ < 100) {
                            return; // skip this step to slow down direction change
                        }
                        led_brightness = 10;
                        led_dim_direction = LED_DIM_DIRECTION_UP;
                        led_dim_dirchange_pause = 0;
                    } else {
                        led_brightness -= 3;
                        led_set_brightness(led_brightness);
                    }
                }
            }
            else if (event == BTN_EVENT_LONG_CLICK_END) {
            }
            break;

    /*
        case STATE_PROGRAM_DIM:
            if (event == BTN_EVENT_LONG_CLICK_HOLD) {
                // adjust brightness
                if (led_dim_direction == LED_DIM_DIRECTION_UP) {
                    if (led_brightness < 255) {
                        led_set_brightness(led_brightness + 5);
                    } else {
                        led_dim_direction = LED_DIM_DIRECTION_DOWN;
                    }
                } else {
                    if (led_brightness > 10) {
                        led_set_brightness(led_brightness - 5);
                    } else {
                        led_dim_direction = LED_DIM_DIRECTION_UP;
                    }
                }
            }
            else if (event == BTN_EVENT_LONG_CLICK_END) {
                state_machine_transition(STATE_PROGRAM, event);
            }
            break;
    */
        default:
            break;
    }
}

// State Callbacks

static void reset_timer_callback(TimerHandle_t xTimer)
{
    if (current_state == STATE_RESET) {
        ESP_LOGI(TAG, "All timers erased!");
        ds3231_power_on();
        delete_all_timers();
        ds3231_power_off();
        led_blink(5, 250);
        reset_confirmed = true;
    }
}

/**
 * @brief Handler für Entry in OFF state
 * 
 * Drive LED off from current brightness, or from 255 if already off.
 * @param event Event that triggered the state change
 */
static void on_state_enter_off(state_machine_button_event_t event) {
    led_fade_off(2000, 255);
}

/**
 * @brief Handler für Exit from OFF state
 * 
 * @param from_state Previous state
 * @param event Event that triggered the state change
 */
static void on_state_exit_off(state_machine_state_t from_state, state_machine_button_event_t event) {
}


/**
 * @brief Handler für Entry in RESET state
 * 
 * @param event Event that triggered the state change
 */
static void on_state_enter_reset(state_machine_button_event_t event){
    reset_confirmed = false;
    led_blink(1, 100);
    
    // Timer starten: einmalig nach 5 Sekunden
    xTimerStart(reset_timer, 0);
}

/**
 * @brief Handler für Exit from RESET state
 * 
 * @param from_state Previous state
 * @param event Event that triggered the state change
 */
static void on_state_exit_reset(state_machine_state_t from_state, state_machine_button_event_t event) {
    // Stop timer if button was released
    xTimerStop(reset_timer, 0);
    reset_confirmed = false;
}


/**
 * @brief Handler on entry into TIMER state
 * 
 * @param event Event that triggered the state change
 */
static void on_state_enter_timer(state_machine_button_event_t event)
{
    led_fade_on(2000, 255);
    led_set_brightness(0);

    if (count_timers() == 0) {
        ESP_LOGI(TAG, "No timers programmed");
        led_blink(5, 100);
    } else {
        ESP_LOGI(TAG, "Entering TIMER state (scheduler)");
        // get next timer to execute
        struct tm now;
        ds3231_power_on();
        ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &now));
        ds3231_power_off();
        led_timer_t next_timer;
        ESP_ERROR_CHECK(get_next_timer(&now, &next_timer));
        ESP_LOGI(TAG, "Next timer: %02d:%02d, brightness: %d",
                    next_timer.hour, next_timer.minute, next_timer.brightness);
    }
}

static void on_state_exit_timer(state_machine_state_t from_state, state_machine_button_event_t event) {
}


static void on_state_enter_timer_dim(state_machine_button_event_t event) {
}

static void on_state_exit_timer_dim(state_machine_state_t from_state, state_machine_button_event_t event) {
}


static void on_state_enter_program(state_machine_button_event_t event)
{
    ESP_LOGI(TAG, "Programming new TIMER");
    led_set_brightness(255);

    // store current time for potential ON timer
    struct tm now;
    ds3231_power_on();
    ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &now));
    ds3231_power_off();
    new_timer.hour = now.tm_hour;
    new_timer.minute = now.tm_min;
    new_timer.brightness = 255;
    new_timer.active = false;
}

static void on_state_exit_program(state_machine_state_t from_state, state_machine_button_event_t event)
{
    if (event == BTN_EVENT_TRIPLE_CLICK) {
        ESP_LOGI(TAG, "Programming ABORTED");
        led_blink(5, 100);
        led_set_brightness(0);
        return;
    }

    ESP_LOGI(TAG, "Add new TIMER");

    // store current time for OFF timer
    struct tm now;
    ds3231_power_on();
    ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &now));
    ds3231_power_off();
    led_timer_t off_timer;
    off_timer.hour = now.tm_hour;
    off_timer.minute = now.tm_min;
    off_timer.brightness = 0;
    off_timer.active = false;

    // add 1 minute to off-timer if same time as on-timer
    if (new_timer.hour == off_timer.hour && new_timer.minute == off_timer.minute) {
        off_timer.minute += 1;
        if (off_timer.minute > 59) {
            off_timer.hour += 1;
            off_timer.minute = 0;
            if (off_timer.hour > 23) {
                off_timer.hour = 0;
            }
        }
    }

    // add both timers to list
    add_timer(new_timer);
    add_timer(off_timer);

    // store timers to EEPROM
    ds3231_power_on();
    save_timers_to_eeprom();
    ds3231_power_off();

    led_set_brightness(0);
    print_timers();
}


static void on_state_enter_program_dim(state_machine_button_event_t event) {
}

static void on_state_exit_program_dim(state_machine_state_t from_state, state_machine_button_event_t event) {
}

//-------------------------------------------------------------------------------//
// I2C functions
//-------------------------------------------------------------------------------//

/**
 * @brief Initialisiert I2C Bus und alle I2C Geräte (DS3231, AT24C)
 * 
 * @return ESP_OK bei Erfolg, sonst Fehlercode
 */
esp_err_t i2c_bus_init(void)
{
    esp_err_t ret;

    ds3231_power_on(); // TODO: really needed? Hm, pullup array is GND so SDA/SCL does not pullup...

    // Initialize I2C bus
    ESP_LOGI(TAG, "Initializing I2C bus...");
    ret = i2cdev_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return ret;
    }
    
    ds3231_power_off(); // TODO

    ESP_LOGI(TAG, "I2C bus initialized successfully");
    return ESP_OK;
}

//-------------------------------------------------------------------------------//
// EEPROM AT24C functions
//-------------------------------------------------------------------------------//

// Initialize AT24C EEPROM
esp_err_t at24c_i2c_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing AT24C EEPROM...");
    ret = at24c_init(&at24c_dev, I2C_MASTER_NUM, AT24C_ADDR,
                     I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO,
                     AT24C_PAGE_SIZE, AT24C_CAPACITY, AT24C_ADDR_WIDTH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AT24C: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "AT24C EEPROM initialized successfully");

    return ESP_OK;
}

//-------------------------------------------------------------------------------//
// RTC DS3231 functions
//-------------------------------------------------------------------------------//

// Initialize DS3231 RTC
esp_err_t ds3231_i2c_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing DS3231 RTC...");
    memset(&ds3231_dev, 0, sizeof(i2c_dev_t));
    ret = i2c_dev_create_mutex(&ds3231_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DS3231 I2C mutex");
        return ret;
    }
    
    ds3231_dev.port = I2C_MASTER_NUM;
    ds3231_dev.addr = DS3231_ADDR;
    ds3231_dev.cfg.sda_io_num = I2C_MASTER_SDA_IO;
    ds3231_dev.cfg.scl_io_num = I2C_MASTER_SCL_IO;
#if HELPER_TARGET_IS_ESP32
    ds3231_dev.cfg.master.clk_speed = 100000;
#endif
    
    ESP_LOGI(TAG, "DS3231 RTC initialized successfully");
    return ESP_OK;
}

void test_ds3231(void)
{
    ESP_LOGI(TAG, "=== Testing DS3231 RTC ===");
    
    esp_err_t ret;

    // Zeit lesen
    struct tm time;
    ret = ds3231_get_time(&ds3231_dev, &time);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get time: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Current time: %04d-%02d-%02d %02d:%02d:%02d",
             time.tm_year + 1900, time.tm_mon + 1, time.tm_mday,
             time.tm_hour, time.tm_min, time.tm_sec);

    // Temperatur lesen (DS3231 hat einen eingebauten Temperatursensor)
    float temp;
    ret = ds3231_get_temp_float(&ds3231_dev, &temp);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Temperature: %.2f°C", temp);
    }
}

/**
 * @brief Init GPIO pin for Power-Control of DS3231 RTC
 */
static void ds3231_init_power_gpio(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DS3231_VCC_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(DS3231_VCC_PIN, 0); // defaults to POWER OFF
}

// Timer Interrupt GPIO14 als Eingang konfigurieren (pull-up)
static void ds3231_interrupt_gpio_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << DS3231_ALARM_INT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);
}

/**
 * @brief Power ON the DS3231 RTC (VCC HIGH) and wait 100 ms for stabilization
 */
static void ds3231_power_on(void)
{
    u_int32_t wait_ms = 100;
    gpio_set_level(DS3231_VCC_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    //ESP_LOGI(TAG, "DS3231 power ON with %d ms wait (run on VCC)", wait_ms);
}

/**
 * @brief Power OFF the DS3231 RTC (VCC LOW, runs on battery)
 */
static void ds3231_power_off(void)
{
    gpio_set_level(DS3231_VCC_PIN, 0);
    //ESP_LOGI(TAG, "DS3231 power OFF (run on VBAT)");
}

/**
 * @brief Configure ESP wakeup from Sleep via DS3231 Alarm interrupt
 */
static void ds3231_configure_wakeup(void)
{
    // ESP32-H2 durch externen Interrupt aufwecken (DS3231 Alarm)
    // RTC GPIO für Deep Sleep konfigurieren

    // TODO: DUMMY
    //esp_sleep_enable_ext0_wakeup(DS3231_ALARM_INT_PIN, 0); // LOW = Alarm ausgelöst
    
    ESP_LOGI(TAG, "Wakeup auf GPIO %d (LOW) konfiguriert", DS3231_ALARM_INT_PIN);
}

static void ds3231_print_time(const struct tm *now, const char *prefix) {
        ESP_LOGI(TAG, "RTC %s: %02d:%02d:%02d",
            prefix,
            now->tm_hour,
            now->tm_min,
            now->tm_sec
        );
        //now->tm_mday, 
        //now->tm_mon + 1,      // tm_mon ist 0-11
        //now->tm_year + 1900,  // tm_year ist Jahre seit 1900
}

// use ds3231_set_time() and ds3231_get_time() from ds3231.h


//-------------------------------------------------------------------------------//
// TIMER functions
//-------------------------------------------------------------------------------//


/**
 * @brief Initialize timer list in RAM and EEPROM
 */
esp_err_t delete_all_timers(void)
{
    // clear timer array in RAM
    memset(timers, 0, sizeof(timers));

    // clear timers stored in EEPROM
    esp_err_t ret = save_timers_to_eeprom();
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

/**
 * @brief Lädt alle Timer aus dem EEPROM
 */
esp_err_t load_timers_from_eeprom(void)
{
    esp_err_t ret;

    // check EEPROM signature first
    ret = at24c_check_signature(&at24c_dev);
    if (ret == ESP_ERR_NOT_FOUND)
    {
        ESP_LOGW(TAG, "No valid signature found in EEPROM, initializing ...");
        memset(timers, 0, sizeof(timers));
        ret = save_timers_to_eeprom();
        if (ret != ESP_OK) return ret;
    }
    else if (ret != ESP_OK) {
        return ret;
    }

    // load data into timers array
    uint16_t addr = AT24C_TIMER_DATA_START_ADDR;  
    for (int i = 0; i < MAX_TIMERS; i++)
    {
        ret = at24c_read(&at24c_dev, addr, (uint8_t*)&timers[i], sizeof(led_timer_t));
        if (ret != ESP_OK) return ret;
        addr += sizeof(led_timer_t);       
    }

    ESP_LOGI(TAG, "All timers loaded from EEPROM");

    return ESP_OK;
}

/**
 * @brief Speichert alle Timer ins EEPROM
 */
esp_err_t save_timers_to_eeprom(void)
{
    esp_err_t ret;

    // check EEPROM signature first
    ret = at24c_check_signature(&at24c_dev);
    if (ret == ESP_ERR_NOT_FOUND)
    {
        ESP_LOGW(TAG, "No valid signature found in EEPROM, initializing ...");
        memset(timers, 0, sizeof(timers));
        ret = save_timers_to_eeprom();
        if (ret != ESP_OK) return ret;
    }
    else if (ret != ESP_OK) {
        return ret;
    }

    uint16_t addr = AT24C_TIMER_DATA_START_ADDR;
    for (int i = 0; i < MAX_TIMERS; i++)
    {
        ret = at24c_write(&at24c_dev, addr, (uint8_t*)&timers[i], sizeof(led_timer_t));
        if (ret != ESP_OK) return ret;
        addr += sizeof(led_timer_t);       
    }

    ESP_LOGI(TAG, "All timers saved to EEPROM");
    return ESP_OK;
}


/**
 * @brief Fügt einen neuen Timer hinzu
 * 
 * @param timer (copy by-value)
 * @return esp_err_t
 */
esp_err_t add_timer(led_timer_t timer)
{
    if (timer.hour > 23 || timer.minute > 59) {
        ESP_LOGE(TAG, "Invalid time %d:%d given!", timer.hour, timer.minute);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Finde ersten freien Platz
    int free_slot = -1;
    for (int i = 0; i < MAX_TIMERS; i++) {
        if (!timers[i].active) {
            free_slot = i;
            break;
        }
    }
    
    if (free_slot == -1) {
        ESP_LOGE(TAG, "Timer-Liste voll!");
        return ESP_ERR_NO_MEM;
    }
    
    // Timer kopieren
    timer.active = 1; // Sicherstellen dass er aktiv ist
    timers[free_slot] = timer;

    // Nach Uhrzeit sortieren
    //sort_timers();
        
    ESP_LOGI(TAG, "Timer hinzugefügt: %02d:%02d, Helligkeit: %d", 
             timer.hour, timer.minute, timer.brightness);
    
    return ESP_OK; //ret;
}

/**
 * @brief Return number of (active) timers in list
 * 
 * @return uint8_t Number of active timers
 */
uint8_t count_timers(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < MAX_TIMERS; i++) {
        if (timers[i].active) {
            cnt++;
        }
    }
    return cnt;
}

/**
 * @brief Holt den nächsten Timer aus der Liste
 * 
 * @param now Zeiger auf aktuelle Zeitstruktur
 * @param timer Zeiger auf Timer-Struktur zum Füllen
 * @return esp_err_t
 */
esp_err_t get_next_timer(const struct tm *now, led_timer_t *timer)
{
    if (now == NULL || timer == NULL) {
        ESP_LOGE(TAG, "NULL-Pointer übergeben");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t count;
    if (count_timers() == 0) {
        ESP_LOGD(TAG, "Kein aktiver Timer vorhanden");
        return ESP_ERR_NOT_FOUND;
    }

    int now_minutes = time_to_minutes(now->tm_hour, now->tm_min);
    int min_distance = 24 * 60; // Maximum mögliche Distanz
    int next_index = -1;
    
    // Finde Timer mit kleinster Distanz vorwärts
    for (int i = 0; i < MAX_TIMERS; i++)
    {
        if (!timers[i].active) continue;
        
        int timer_minutes = time_to_minutes(timers[i].hour, timers[i].minute);
        int distance = minutes_until(now_minutes, timer_minutes);
        
        if (distance < min_distance) {
            min_distance = distance;
            next_index = i;
        }
    }
    
    if (next_index == -1) {
        ESP_LOGW(TAG, "Kein aktiver Timer gefunden");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Timer kopieren
    *timer = timers[next_index];
    
    ESP_LOGI(TAG, "Nächster Timer: %02d:%02d (in %d Minuten), Helligkeit: %d",
             timer->hour, timer->minute, min_distance, timer->brightness);

    return ESP_OK;
}

esp_err_t get_previous_timer(const struct tm *now, led_timer_t *timer)
{
    if (now == NULL || timer == NULL) {
        ESP_LOGE(TAG, "NULL-Pointer übergeben");
        return ESP_ERR_INVALID_ARG;
    }

    if (count_timers() == 0) {
        ESP_LOGD(TAG, "Kein aktiver Timer vorhanden");
        return ESP_ERR_NOT_FOUND;
    }

    int now_minutes = time_to_minutes(now->tm_hour, now->tm_min);
    int min_distance = 24 * 60; // Maximum mögliche Distanz
    int prev_index = -1;
    
    // Finde Timer mit kleinster Distanz rückwärts
    for (int i = 0; i < MAX_TIMERS; i++) {
        if (!timers[i].active) continue;
        
        int timer_minutes = time_to_minutes(timers[i].hour, timers[i].minute);
        int distance = minutes_since(now_minutes, timer_minutes);
        
        if (distance < min_distance) {
            min_distance = distance;
            prev_index = i;
        }
    }
    
    if (prev_index == -1) {
        ESP_LOGW(TAG, "Kein aktiver Timer gefunden");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Timer kopieren
    *timer = timers[prev_index];
    
    ESP_LOGI(TAG, "Vorheriger Timer: %02d:%02d (vor %d Minuten), Helligkeit: %d",
             timer->hour, timer->minute, min_distance, timer->brightness);

    return ESP_OK;
}

// DS3231 Alarm setzen
esp_err_t set_ds3231_alarm(uint8_t hour, uint8_t minute)
{
    /*
    uint8_t alarm_data[5] = {
        0x07,                    // Startadresse: Alarm 1 Register
        dec_to_bcd(minute),      // Minuten
        dec_to_bcd(hour),        // Stunden
        0x80,                    // Tag ignorieren (Bit 7 = 1)
        0x80                     // Datum ignorieren (Bit 7 = 1)
    };
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, alarm_data, sizeof(alarm_data), true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Setzen des DS3231 Alarms");
        return ret;
    }
    
    // Alarm aktivieren (Control Register)
    uint8_t control_data[2] = {0x0E, 0x05}; // A1IE = 1, INTCN = 1
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, control_data, sizeof(control_data), true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Alarm-Flag löschen (Status Register)
    uint8_t status_data[2] = {0x0F, 0x00};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, status_data, sizeof(status_data), true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    ESP_LOGI(TAG, "DS3231 Alarm gesetzt: %02d:%02d", hour, minute);
    return ret;
    */
   return ESP_OK;
}

// Timer auflisten
void print_timers(void)
{
    printf("=== Timers (%d/%d) ===\n", count_timers(), MAX_TIMERS);
    for (int i = 0; i < MAX_TIMERS; i++) {
        if (!timers[i].active) continue;
        printf("%d: %02d:%02d -> Helligkeit: %3d\n",
            i + 1,
            timers[i].hour,
            timers[i].minute,
            timers[i].brightness);
    }
}

// Zeitvergleich in Minuten seit Mitternacht
static int time_to_minutes(uint8_t hour, uint8_t minute) {
    return hour * 60 + minute;
}

// Berechnet Zeitdifferenz vorwärts (mit Überlauf bei Mitternacht)
static int minutes_until(int now_minutes, int target_minutes) {
    if (target_minutes >= now_minutes) {
        return target_minutes - now_minutes;
    } else {
        // Über Mitternacht
        return (24 * 60 - now_minutes) + target_minutes;
    }
}

// Berechnet Zeitdifferenz rückwärts (mit Überlauf bei Mitternacht)
static int minutes_since(int now_minutes, int target_minutes) {
    if (target_minutes <= now_minutes) {
        return now_minutes - target_minutes;
    } else {
        // Über Mitternacht zurück
        return now_minutes + (24 * 60 - target_minutes);
    }
}

// Zeitvergleich (gibt true zurück wenn t1 < t2)
static bool is_time_before(const led_timer_t *t1, const led_timer_t *t2) {
    if (t1->hour < t2->hour) return true;
    if (t1->hour > t2->hour) return false;
    return t1->minute < t2->minute;
}

// Timer nach Uhrzeit sortieren
static void sort_timers(void) {
    // Zähle aktive Timer
    int count = 0;
    for (int i = 0; i < MAX_TIMERS && timers[i].active; i++) {
        count++;
    }
    
    // Sortiere nur aktive Timer
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (is_time_before(&timers[j + 1], &timers[j])) {
                led_timer_t temp = timers[j];
                timers[j] = timers[j + 1];
                timers[j + 1] = temp;
            }
        }
    }
}

//-------------------------------------------------------------------------------//
// MISC functions
//-------------------------------------------------------------------------------//

static void delay_ms(uint32_t wait_ms)
{
    if (wait_ms >= 10) {
        vTaskDelay(pdMS_TO_TICKS(wait_ms)); // vTaskDelay only works >= 10 ms
    } else {
        esp_rom_delay_us(wait_ms * 1000);
    }
}

void print_hex(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (i % 16 == 0) {
            printf("\n%04X: ", (unsigned)i);
        }
        printf("%02X ", data[i]);
    }
    printf("\n");
}


//-------------------------------------------------------------------------------//
// MAIN
//-------------------------------------------------------------------------------//

void app_main(void)
{
    esp_err_t ret;

    // set loglevel
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "ESP32 Xmas Lightstrip Controller started");

    /* LIGHT SLEEP TEST
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT); // blue LED
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT); // boot button
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);
       while(1) {
        for(int i = 0; i < 6; i++) {
            gpio_set_level(GPIO_NUM_13, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(GPIO_NUM_13, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        esp_sleep_enable_gpio_wakeup();
        gpio_wakeup_enable(BUTTON_GPIO, GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_timer_wakeup(15 * 1000000);
        esp_light_sleep_start();
    }
    */

    // open NVS storage
    //nvs_flash_init();
    //nvs_handle_t handle;
    //nvs_open("storage", NVS_READWRITE, &handle);
    //nvs_get_u8(handle, "led_brightness", &led_brightness);

    // Initialize hardware
    led_init(); // initialize LED PWM
    button_init(); // initialize button with callbacks
    ds3231_init_power_gpio(); // DS3231 is OFF at startup
    ds3231_interrupt_gpio_init(); // Setup GPIO connected to DS3231 SQW
    i2c_bus_init(); // initialize I2C bus and devices
    ds3231_i2c_init();
    at24c_i2c_init();
    state_machine_init(); // initialize state machine

    // TODO: Prüfe ob Oszillator gestoppt war (z.B. durch leere Batterie) und Lösche alle Timer um ohne weitere Eingriffe wieder in Funktion zu kommen
    /*
    bool stopped = false;
    if (ds3231_get_oscillator_stop_flag(&ds3231_dev, &stopped) == ESP_OK) {
        if (stopped) {
            ESP_LOGW(TAG, "Oszillator war gestoppt - Zeit muss neu gesetzt werden!");
            ds3231_clear_oscillator_stop_flag(&ds3231_dev);
            clear_all_timers();
        }
    }
    */

    ds3231_power_on();

    at24c_dump(&at24c_dev, 0x0000, 64);

    // load timers from EEPROM
    ret = load_timers_from_eeprom();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load timers from EEPROM (%s). Try to initialize EEPROM", esp_err_to_name(ret));
        ret = at24c_write_signature(&at24c_dev);
        ret = delete_all_timers();
    } else {
        ESP_LOGI(TAG, "Timers loaded from EEPROM successfully");
        print_timers();
    }

    struct tm now;
    ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &now));
    ds3231_print_time(&now, "Current time");

    // get next timer to execute
    led_timer_t next_timer;
    ret = get_next_timer(&now, &next_timer);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Next timer set to: %02d:%02d having brightness: %d%", next_timer.hour, next_timer.minute, next_timer.brightness);

        // Set DS3231 Alarm 2 for next timer
        struct tm alarm_time = {
            .tm_hour = next_timer.hour,
            .tm_min = next_timer.minute
        };
        ESP_ERROR_CHECK(ds3231_clear_alarm_flags(&ds3231_dev, DS3231_ALARM_2));
        ESP_ERROR_CHECK(ds3231_set_alarm(&ds3231_dev, DS3231_ALARM_2, NULL, 0, &alarm_time, DS3231_ALARM2_MATCH_MINHOUR));

        // Activate interrupt for DS3231 Alarm
        ESP_ERROR_CHECK(ds3231_enable_alarm_ints(&ds3231_dev, DS3231_ALARM_2));

        // Set BB_SQW flag to '1' (needed to preserve SQW pin function in VBAT mode)
        uint8_t control_reg;
        i2c_dev_read_reg(&ds3231_dev, DS3231_ADDR_CONTROL, &control_reg, 1);
        control_reg |= (DS3231_CONTROL_INTCN | DS3231_CONTROL_A1IE | DS3231_CONTROL_BBSQW);
        i2c_dev_write_reg(&ds3231_dev, DS3231_ADDR_CONTROL, &control_reg, 1);
    }
    else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "No timer found to process");
    }
    else {
        ESP_LOGI(TAG, "Failed to get next timer: %s", esp_err_to_name(ret));
    }

    // Power OFF DS3231 to run on battery
    ds3231_power_off();

    // Configure wakeup from DS3231 Alarm interrupt
    //ds3231_configure_wakeup();

/*
    // Go to Light-Sleep if no button pressed
    ESP_LOGI(TAG, "I'm sooo tired... :-O");
    vTaskDelay(pdMS_TO_TICKS(1000));

    //uart_set_wakeup_threshold(UART_NUM_0, 3);
    //esp_sleep_enable_uart_wakeup(UART_NUM_0);

    led_fade_off(1000, 100);
    //vTaskDelay(pdMS_TO_TICKS(1000));

    esp_sleep_enable_timer_wakeup(5* 1000000);

    iot_button_stop();
    gpio_wakeup_enable(BUTTON_GPIO, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    esp_light_sleep_start();

    // ESP_LOGI(TAG, "Uaaaaah! I'M ALIVE !!! >:-D");
    led_fade_on(1000, 100);
    led_pulse(3, 1000, 100);
    led_set_brightness(100);
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_GPIO:
            ESP_LOGI(TAG, "Erweckt durch LOW auf GPIO");
            led_blink(1, 500);
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Erweckt durch Timout Timer");
            led_blink(2, 500);
            break;
        default:
            ESP_LOGI(TAG, "Erweckt durch unbekannte Ursache: %d", wakeup_reason);
            led_blink(5, 100);
            break;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    led_set_brightness(100);

    iot_button_resume();
*/

    while(1)
    {
        /*
        ds3231_power_on();
        ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &now));
        ds3231_power_off();
        ds3231_print_time(&now, "Current time is");
        */

        /*
        bool a1_flag, a2_flag;
        ds3231_alarm_t alarms;
        ds3231_get_alarm_flags(&ds3231_dev, &alarms);
        a1_flag = (alarms & DS3231_ALARM_1) != 0;
        a2_flag = (alarms & DS3231_ALARM_2) != 0;

        ds3231_power_off();

        int level = gpio_get_level(DS3231_ALARM_INT_PIN);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "⏰ Zeit: %02d:%02d:%02d | GPIO14: %s | Alarm-Flag: %s",
                rtctime.tm_hour, rtctime.tm_min, rtctime.tm_sec,
                level ? "HIGH" : "LOW",
                a1_flag ? "GESETZT" : "Gelöscht");
            ESP_LOGI(TAG, "Zeit: %02d.%02d.%04d %02d:%02d:%02d",
                rtctime.tm_mday, 
                rtctime.tm_mon + 1,      // tm_mon ist 0-11
                rtctime.tm_year + 1900,  // tm_year ist Jahre seit 1900
                rtctime.tm_hour,
                rtctime.tm_min,
                rtctime.tm_sec
            );
        } else {
            ESP_LOGE(TAG, "Fehler beim Lesen: %s", esp_err_to_name(err));
        }
        */
        //ESP_LOGI(TAG, "DS3231 Alarm INT Pin Level: %s", level ? "zzz" : "🔔 Alarm ausgelöst!");

        //print_timers();

        //printf(".");
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