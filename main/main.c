#include <stdio.h>
#include "driver/ledc.h" // LEDC driver
#include "freertos/FreeRTOS.h" // used for pdMS_TO_TICKS
#include "freertos/task.h" // used for vTaskDelay
#include "freertos/timers.h" // used for esp_timer
#include "esp_log.h" // logging
#include "nvs_flash.h"
#include "nvs.h" // NVS
#include "iot_button.h" // button library
#include "i2cdev.h" // I2C
#include "ds3231.h" // DS3231 RTC (I2C realtime clock)

//-------------------------------------------------------------------------------//
// defines
//-------------------------------------------------------------------------------//

#define BUTTON_GPIO 9
#define LONG_PRESS_TIME_MS 800
#define SHORT_PRESS_TIME_MS 200

#define LED_DIM_DIRECTION_UP true
#define LED_DIM_DIRECTION_DOWN false
#define LED_DIM_SPEED_MS 3000 // Zeit für komplettes Dimmen (auf oder ab) in ms

#define I2C_MASTER_SCL_IO GPIO_NUM_11
#define I2C_MASTER_SDA_IO GPIO_NUM_12
#define I2C_MASTER_NUM I2C_NUM_0
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_CONTROL_A1IE (1 << 0)
#define DS3231_CONTROL_A2IE (1 << 1)
#define DS3231_CONTROL_INTCN (1 << 2)
#define DS3231_CONTROL_BBSQW (1 << 6)
// GPIO für DS3231 VCC (Power-Control)
#define DS3231_VCC_PIN GPIO_NUM_10
// Interrupt-Pin vom DS3231 SQW/INT
#define DS3231_ALARM_INT_PIN GPIO_NUM_14


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

//-------------------------------------------------------------------------------//
// global variables
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
static void state_machine_init(void);
static void state_machine_transition(state_machine_state_t new_state, state_machine_button_event_t event);
static void state_machine_handle_button_event(state_machine_button_event_t event);
static void reset_timer_callback(TimerHandle_t xTimer);
//
static void on_state_enter_off(state_machine_button_event_t event);
static void on_state_enter_reset(state_machine_button_event_t event);
static void on_state_enter_timer(state_machine_button_event_t event);
static void on_state_enter_timer_dim(state_machine_button_event_t event);
static void on_state_enter_program(state_machine_button_event_t event);
static void on_state_enter_program_dim(state_machine_button_event_t event);
//
static void on_state_exit_off(state_machine_state_t from_state, state_machine_button_event_t event);
static void on_state_exit_reset(state_machine_state_t from_state, state_machine_button_event_t event);

// RTC
static void ds3231_init_power_gpio(void);
static void ds3231_power_on(void);
static void ds3231_power_off(void);

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
 * @brief Callback called every 500ms as long as button keeps pressed
 */
static void on_long_press_hold(void *arg, void *usr_data)
{
    ESP_LOGI(TAG, "⏳ LANGER DRUCK wird gehalten...");
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
        case STATE_TIMER:
            //on_state_exit_TIMER(new_state, event);
            break;
        case STATE_TIMER_DIM:
            //on_state_exit_TIMER(new_state, event);
            break;
        case STATE_PROGRAM:
            //on_state_exit_timer_program_on(new_state, event);
            break;
        case STATE_PROGRAM_DIM:
            //on_state_exit_timer_program_off(new_state, event);
            break;

        // ... weitere Cases

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
        case STATE_TIMER_DIM:
            on_state_enter_timer_dim(event);
            break;
        case STATE_PROGRAM:
            on_state_enter_program(event);
            break;
        case STATE_PROGRAM_DIM:
            on_state_enter_program_dim(event);
            break;
        default:
            ESP_LOGE(TAG, "Unhandled entry-hanlder for state: %d", new_state);
            return;
    }

    ESP_LOGI(TAG, "State transition: %s -> %s", state_names[current_state], state_names[new_state]);
    current_state = new_state;
}

static void reset_timer_callback(TimerHandle_t xTimer)
{
    if (current_state == STATE_RESET) {
        ESP_LOGI(TAG, "All timers erased!");
        //erase_all_timers();
        led_blink(5, 250);
        reset_confirmed = true;
    }
}

/**
 * @brief State Machine business logic
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
                    led_blink(2, 100);  // Feedback während des Wartens
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
            else if (event == BTN_EVENT_LONG_CLICK_START) {
                state_machine_transition(STATE_TIMER_DIM, event);
            }
            break;

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

        case STATE_PROGRAM:
            if (event == BTN_EVENT_TRIPLE_CLICK) { // abort programming
                state_machine_transition(STATE_TIMER, event);
            }
            else if (event == BTN_EVENT_DOUBLE_CLICK) { // finish programming
                if (led_brightness > 0) {
                    state_machine_transition(STATE_TIMER, event);
                }
            }
            else if (event == BTN_EVENT_LONG_CLICK_START) {  // re-adjust brightness of new timer
                state_machine_transition(STATE_PROGRAM_DIM, event);
            }
            break;

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
        default:
            break;
    }
}

// State Callbacks

static void on_state_enter_off(state_machine_button_event_t event) {
    if (led_brightness == 0) {
        led_set_brightness(100);
    }
    led_fade_off(2000);
}

static void on_state_exit_off(state_machine_state_t from_state, state_machine_button_event_t event) {
    // nothing to do
}

static void on_state_enter_reset(state_machine_button_event_t event){
    reset_confirmed = false;
    led_blink(1, 100);
    
    // Timer starten: einmalig nach 5 Sekunden
    xTimerStart(reset_timer, 0);
}

static void on_state_exit_reset(state_machine_state_t from_state, state_machine_button_event_t event) {
    // Stop timer if button was released
    xTimerStop(reset_timer, 0);
    reset_confirmed = false;
}

static void on_state_enter_timer(state_machine_button_event_t event) {
    if (current_state == STATE_PROGRAM) {
        ESP_LOGI(TAG, "Timer programming COMPLETED!");
        led_blink(3, 250);
        led_fade_off(2000);
    }
    else {
        //if (no_timers_programmed()) {
            ESP_LOGI(TAG, "No timers programmed! Please program a timer first.");
            led_blink(10, 100);
        //} else {
        //    ESP_LOGI(TAG, "Entering TIMER state");
        //    led_blink(2, 500);
            //start_next_timer();
        //}
    }
}

static void on_state_enter_timer_dim(state_machine_button_event_t event) {
    ESP_LOGI(TAG, "Entering DIM LED brightness state");
}

static void on_state_enter_program(state_machine_button_event_t event) {
    ESP_LOGI(TAG, "Programming TIMER started");
    led_blink(2, 250);
    led_fade_on(2000, 100);
}

static void on_state_enter_program_dim(state_machine_button_event_t event) {
    ESP_LOGI(TAG, "Entering DIM LED brightness state");
}

//-------------------------------------------------------------------------------//
// RTC DS3231 functions
//-------------------------------------------------------------------------------//

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

/**
 * @brief Power ON the DS3231 RTC (VCC HIGH) and wait 100 ms for stabilization
 */
static void ds3231_power_on(void)
{
    gpio_set_level(DS3231_VCC_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "DS3231 power ON (run on VCC)");
}

/**
 * @brief Power OFF the DS3231 RTC (VCC LOW, runs on battery)
 */
static void ds3231_power_off(void)
{
    gpio_set_level(DS3231_VCC_PIN, 0);
    ESP_LOGI(TAG, "DS3231 power OFF (run on VBAT)");
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
    ESP_ERROR_CHECK(i2cdev_init());
    ds3231_init_power_gpio();

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(ds3231_init_desc(&dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    // open NVS storage
    nvs_handle_t handle;
    nvs_open("storage", NVS_READWRITE, &handle);
    
    // restore led brightness from flash
    //nvs_get_u8(handle, "led_brightness", &led_brightness);
    led_set_brightness(0);
    //led_set_brightness(led_brightness);
    ESP_LOGI(TAG, "Restored LED brightness from Flash: %u", led_brightness);

    // RTC TESTS

    // Prüfe ob Oszillator gestoppt war (z.B. durch leere Batterie)
    /*
    bool stopped = false;
    if (ds3231_get_oscillator_stop_flag(&dev, &stopped) == ESP_OK) {
        if (stopped) {
            ESP_LOGW(TAG, "Oszillator war gestoppt - Zeit muss neu gesetzt werden!");
            ds3231_clear_oscillator_stop_flag(&dev);
        }
    }
    */

    // Alarm-TEST: GPIO14 als Eingang konfigurieren (pull-up)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << DS3231_ALARM_INT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    struct tm rtctime;

    // Alarm konfigurieren
    struct tm alarm_time;
    /*
    alarm_time = {
        .tm_hour = 15,
        .tm_min = 30
    };
    */
    ds3231_power_on();
    ds3231_get_time(&dev, &alarm_time); // get current time
    alarm_time.tm_sec = (alarm_time.tm_sec + 15) % 60; // set alarm time to next minute in the future
    ESP_LOGI(TAG, "Alarmzeit: %02d.%02d.%04d %02d:%02d:%02d",
        alarm_time.tm_mday, 
        alarm_time.tm_mon + 1,      // tm_mon ist 0-11
        alarm_time.tm_year + 1900,  // tm_year ist Jahre seit 1900
        alarm_time.tm_hour,
        alarm_time.tm_min,
        alarm_time.tm_sec
    );

    // Setze Alarm 1 auf die eingestellte Zeit (Alarm bei Sekunden-Match)
    ds3231_clear_alarm_flags(&dev, DS3231_ALARM_1);
    ESP_ERROR_CHECK(ds3231_set_alarm(&dev, DS3231_ALARM_1, &alarm_time, DS3231_ALARM1_MATCH_SECMINHOUR, NULL, 0));
    // Interrupts für Alarm 1 aktivieren
    ESP_ERROR_CHECK(ds3231_enable_alarm_ints(&dev, DS3231_ALARM_1));
    // BBSQW für Interrupt-Auslösung im Battery-Backup-Modus auf 1 setzen
    uint8_t control_reg;
    i2c_dev_read_reg(&dev, DS3231_ADDR_CONTROL, &control_reg, 1);
    control_reg |= (DS3231_CONTROL_INTCN | DS3231_CONTROL_A1IE | DS3231_CONTROL_BBSQW);
    i2c_dev_write_reg(&dev, DS3231_ADDR_CONTROL, &control_reg, 1);
    ds3231_power_off();

    while(1)
    {
        ds3231_power_on();
        esp_err_t err = ds3231_get_time(&dev, &rtctime);
        
        bool a1_flag, a2_flag;
        ds3231_alarm_t alarms;
        ds3231_get_alarm_flags(&dev, &alarms);
        a1_flag = (alarms & DS3231_ALARM_1) != 0;
        a2_flag = (alarms & DS3231_ALARM_2) != 0;

        ds3231_power_off();

        int level = gpio_get_level(DS3231_ALARM_INT_PIN);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "⏰ Zeit: %02d:%02d:%02d | GPIO14: %s | Alarm-Flag: %s",
                rtctime.tm_hour, rtctime.tm_min, rtctime.tm_sec,
                level ? "HIGH" : "LOW",
                a1_flag ? "GESETZT" : "Gelöscht");

            /*
            ESP_LOGI(TAG, "Zeit: %02d.%02d.%04d %02d:%02d:%02d",
                rtctime.tm_mday, 
                rtctime.tm_mon + 1,      // tm_mon ist 0-11
                rtctime.tm_year + 1900,  // tm_year ist Jahre seit 1900
                rtctime.tm_hour,
                rtctime.tm_min,
                rtctime.tm_sec
            );
            */
        } else {
            ESP_LOGE(TAG, "Fehler beim Lesen: %s", esp_err_to_name(err));
        }

        //ESP_LOGI(TAG, "DS3231 Alarm INT Pin Level: %s", level ? "zzz" : "🔔 Alarm ausgelöst!");

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