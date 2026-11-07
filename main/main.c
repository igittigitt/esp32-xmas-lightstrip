//-------------------------------------------------------------------------------//
// INCLUDES
#include <stdio.h>
#include <time.h> // (exports "struct tm")
//#include "driver/ledc.h" // LEDC driver

#include "freertos/FreeRTOS.h" // used for pdMS_TO_TICKS
#include "freertos/task.h" // used for vTaskDelay
#include "freertos/timers.h" // used for esp_timer
#include "freertos/queue.h"

#include "esp_timer.h"       // for esp_timer_get_time()
#include "esp_log.h" // logging
//#include "nvs_flash.h"
//#include "nvs.h" // NVS
#include "iot_button.h" // button library
#include "esp_sleep.h" // sleep modes

// Sleep and Debugging stuff
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#include "esp_vfs_dev.h"
//#include "esp_private/esp_clk.h"

// I2C libraries
#include "i2cdev.h" // I2C (old I2C API, currently needed by ds3231)
//#include "driver/i2c_master.h" // I2C Master driver (new API)

// DS3231 Module (RTC+EEPROM)
#include "ds3231.h" // DS3231 RTC (I2C realtime clock) <- OLD API
#include "at24c.h" // AT24C EEPROM (I2C EEPROM) <- OLD API
#include "esp_idf_lib_helpers.h"

// Console (UART)
#include "esp_console.h" // used for keyboard commands via interactive console (idf_monitor.py or PuTTY)
#include "driver/uart.h"

#include "led_controller.h"

//-------------------------------------------------------------------------------//
// APP defines
#define DEBUG                   1
#define LOG_LEVEL               ESP_LOG_DEBUG // ESP_LOG_INFO
#define SLEEP_INDICATOR_GPIO    GPIO_NUM_4 //
#define MAX_TIMERS              8
#define INACTIVITY_TIMEOUT_MS   (15 * 1000LL) // inactivity timer timeouts

//-------------------------------------------------------------------------------//
// DS3231 MODULE defines
#define DS3231_ADDR                 0x68 // DS3231 I2C Adresse
#define DS3231_ADDR_CONTROL         0x0e
#define DS3231_CONTROL_A1IE         (1 << 0)
#define DS3231_CONTROL_A2IE         (1 << 1)
#define DS3231_CONTROL_INTCN        (1 << 2)
#define DS3231_CONTROL_BBSQW        (1 << 6)
#define DS3231_EEPROM_I2C_ADDR      0x57 // AT24C32 I2C Adresse (A0,A1,A2 auf HIGH)
#define DS3231_EEPROM_PAGE_SIZE     32
#define DS3231_EEPROM_CAPACITY      4096
#define DS3231_EEPROM_ADDR_WIDTH    2
#define DS3231_EEPROM_DATA_OFFSET   0x0004 // where timer data starts in EEPROM

#define DS3231_MODULE_POWERSAVE     1 // if set to '1' (true) module will be switched on only on access
#define DS3231_VCC_PIN              GPIO_NUM_10 // GPIO für DS3231 VCC (Power-Control)
#define DS3231_ALARM_INT_PIN        GPIO_NUM_14 // Interrupt-Pin vom DS3231 SQW/INT

//-------------------------------------------------------------------------------//
// BUTTON defines
#define BUTTON_GPIO             GPIO_NUM_5 // GPIO_NUM_9 = BOOT-Button
#define BUTTON_ACTIVE_LEVEL     1 // 0 = if pullup and push to GND
#define LONG_PRESS_TIME_MS      1000 // button must be pressed at least this time to detect "long press"
#define SHORT_PRESS_TIME_MS     300 // debounce time for short presses

//-------------------------------------------------------------------------------//
// LED defines
#define LED_GPIO                GPIO_NUM_13
#define LED_DIM_DIRECTION_UP    true
#define LED_DIM_DIRECTION_DOWN  false
#define LED_DIM_SPEED_MS        3000 // Zeit für komplettes Dimmen (auf oder ab) in ms

//-------------------------------------------------------------------------------//
// I2C defines
#define I2C_MASTER_FREQ_HZ      100000  // 100kHz needed to drive DS3231 (fast) and AT24C (slow)
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SCL_IO       GPIO_NUM_11
#define I2C_MASTER_SDA_IO       GPIO_NUM_12


//-------------------------------------------------------------------------------//
// TYPES

// State machine states
typedef enum {
    STATE_OFF = 0,
    STATE_RESET,
    STATE_TIMER,
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
// GLOBAL VARIABLES

// APP
static bool console_mode = false; // true = console mode enabled on boot
static const char* TAG = "XMAS_LED_CTRL";
static bool reset_confirmed = false;
static TimerHandle_t reset_timer = NULL;

// SLEEP
static volatile bool sleep_enabled = true;
static QueueHandle_t gpio_evt_queue = NULL;
static SemaphoreHandle_t sleep_semaphore = NULL;

// LED
uint8_t led_brightness = 0; // Helligkeit von 0 bis 255
bool led_dim_direction = LED_DIM_DIRECTION_UP; // true = up, false = down

// STATE MACHINE
static state_machine_state_t current_state = STATE_OFF;
static const char* state_names[] = {
    "OFF",
    "RESET",
    "TIMER",
    "PROGRAM",
    "PROGRAM_DIM"
};

// I2C
static i2c_dev_t ds3231_dev;
static at24c_handle_t at24c_dev;

// BUTTON
button_handle_t btn;

// TIMER
TimerHandle_t inactivity_timer;
// Timer structure (uses packed attribute to avoid padding for better EEPROM storage)
typedef struct __attribute__((packed)) {
  uint8_t hour;        // 0-23
  uint8_t minute;      // 0-59
  uint8_t brightness;  // LED Helligkeitswert
  bool active;        // Timer aktiv/inaktiv
} led_timer_t;
static led_timer_t timers[MAX_TIMERS]; // initializes all timer to 0 (inactive)
static led_timer_t new_timer; // store start time of current program


//-------------------------------------------------------------------------------//
// PROTOTYPES
//-------------------------------------------------------------------------------//

//-------------------------------------------------------------------------------//
// DS3231 MODULE
void test_ds3231(void);
esp_err_t ds3231_rtc_i2c_init(void);
esp_err_t ds3231_eeprom_i2c_init(void);
static void ds3231_setup_power_gpio(void);
static void ds3231_power_on(void);
static void ds3231_power_off(void);
//static void ds3231_print_time(const struct tm *now, const char *prefix);
static void ds3231_setup_intr_gpio(void);
//static void ds3231_configure_alarms(void);
//static void ds3231_clear_alarms(void);
//static void ds3231_configure_wakeup(void);

//-------------------------------------------------------------------------------//
// LED
/*
static TaskHandle_t blink_task_handle = NULL;
static void led_init(gpio_num_t led_gpio);
static void led_set_brightness(uint8_t brightness);
static void led_blink(uint8_t times, uint32_t period_ms);
static void led_blink_task(void *pvParameters);
void led_blink_task_start(uint32_t period_ms);
void led_blink_task_stop(void);

static void led_fade_off(uint32_t duration_ms, uint8_t start_brightness);
static void led_fade_on(uint32_t duration_ms, uint8_t target_brightness);
//static void led_pulse(uint8_t times, uint32_t pulse_duration_ms, uint8_t start_end_brightness);
*/
uint8_t led_dim_dirchange_pause = 0;

//-------------------------------------------------------------------------------//
// Button
static button_handle_t button_init(void);
static void on_press_repeat_done(void *arg, void *usr_data);
static void on_long_press_start(void *arg, void *usr_data);
static void on_long_press_hold(void *arg, void *usr_data);
static void on_long_press_up(void *arg, void *usr_data);
static void on_button_press_down(void *arg, void *usr_data);
static void on_button_press_up(void *arg, void *usr_data);

//-------------------------------------------------------------------------------//
// State Machine
static void state_machine_init(void);
static void state_machine_transition(state_machine_state_t new_state, state_machine_button_event_t event);
static void state_machine_handle_button_event(state_machine_button_event_t event);
static void reset_timer_callback(TimerHandle_t xTimer);
static void on_state_enter_off(state_machine_button_event_t event);
static void on_state_exit_off(state_machine_state_t from_state, state_machine_button_event_t event);
static void on_state_enter_reset(state_machine_button_event_t event);
static void on_state_exit_reset(state_machine_state_t from_state, state_machine_button_event_t event);
static void on_state_enter_timer(state_machine_button_event_t event);
// static void on_state_exit_timer(state_machine_state_t from_state, state_machine_button_event_t event);
static void on_state_enter_program(state_machine_button_event_t event);
static void on_state_exit_program(state_machine_state_t from_state, state_machine_button_event_t event);
//static void on_state_enter_program_dim(state_machine_button_event_t event);
//static void on_state_exit_program_dim(state_machine_state_t from_state, state_machine_button_event_t event);

//-------------------------------------------------------------------------------//
// I2C
esp_err_t i2c_bus_init(void);

//-------------------------------------------------------------------------------//
// APP
static void debug_task(void *pvParameters);
void check_console_mode_on_boot(gpio_num_t button_gpio, uint8_t button_active_level);
static void timer_terminal_menu(void);

//-------------------------------------------------------------------------------//
// Sleep
void reinit_usb_cdc();
static void sleep_task(void *arg);
static void inactivity_timer_callback(TimerHandle_t xTimer);
static void sleep_indicator_init(void);
static void sleep_indicator_sleeping(void);
static void sleep_indicator_awake(void);
static void gpio_isr_handler(void *arg);
static void gpio_event_task(void *arg);

//-------------------------------------------------------------------------------//
// TIMER
uint8_t count_timers(void);
esp_err_t add_timer(led_timer_t timer);
esp_err_t get_next_timer(const struct tm *now, led_timer_t *timer);
esp_err_t get_previous_timer(const struct tm *now, led_timer_t *timer);
esp_err_t set_ds3231_alarm(led_timer_t timer);
esp_err_t clear_ds3231_alarms(void);
esp_err_t delete_all_timers(void);
void print_timers(void);
esp_err_t load_timers_from_eeprom(void);
esp_err_t save_timers_to_eeprom(void);
static int time_to_minutes(uint8_t hour, uint8_t minute);
static int minutes_until(int now_minutes, int target_minutes);
static int minutes_since(int now_minutes, int target_minutes);
//static bool is_time_before(const led_timer_t *t1, const led_timer_t *t2);
//static void sort_timers(void);


//-------------------------------------------------------------------------------//
// SLEEP functions
//-------------------------------------------------------------------------------//
// NOTE: Sleep only works with activated power-management CONFIG_PM_ENABLE=y

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
    
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
//-------------------------------------------------------------------------------//
/*
void update_activity() {
    last_activity_time = esp_timer_get_time();
}
//-------------------------------------------------------------------------------//
void button_single_click_cb(void *arg, void *data) {
    printf("Button clicked!\n");
    update_activity();
}
*/
//-------------------------------------------------------------------------------//
void reinit_usb_cdc() {
    // USB-CDC neu initialisieren nach Sleep
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    
    // Kurz warten für USB Enumeration
    vTaskDelay(pdMS_TO_TICKS(100));
    
    usb_serial_jtag_driver_install(&usb_serial_config);

    vTaskDelay(pdMS_TO_TICKS(500)); // Zeit für Verbindungsaufbau
}
//-------------------------------------------------------------------------------//
static void inactivity_timer_callback(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Inactivity timeout - triggering sleep");
    xSemaphoreGive(sleep_semaphore);  // Signal an Task
}
//-------------------------------------------------------------------------------//
static void sleep_indicator_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << SLEEP_INDICATOR_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);
    sleep_indicator_awake();
}
//-------------------------------------------------------------------------------//
static void sleep_indicator_sleeping(void) {
    gpio_set_level(SLEEP_INDICATOR_GPIO, 1);
}
//-------------------------------------------------------------------------------//
static void sleep_indicator_awake(void) {
    gpio_set_level(SLEEP_INDICATOR_GPIO, 0);
}


//-------------------------------------------------------------------------------//
// RTOS Tasks
//-------------------------------------------------------------------------------//
static void gpio_event_task(void *arg)
{
    uint32_t io_num;
    
    while(1)
    {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "GPIO[%ld] interrupt! Pin level: %d", io_num, gpio_get_level(io_num));
        }

        vTaskDelay(pdTICKS_TO_MS(100));
    }
}
//-------------------------------------------------------------------------------//
static void sleep_task(void *arg)
{
    while(1)
    {
        if(xSemaphoreTake(sleep_semaphore, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Entering light-sleep now...");
            // TX Buffer leeren vor dem Sleep
            //vTaskDelay(pdMS_TO_TICKS(100));
            //uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(1000));
            uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

            iot_button_stop();

            ds3231_power_off();

            // SLEEP
            #if DEBUG
            //esp_sleep_enable_timer_wakeup(10 * 1000000); // wakeup after 10 seconds
            sleep_indicator_sleeping(); // external sleep indication (DEBUG only)
            int64_t t_before_us = esp_timer_get_time();
            #endif

            //********** LIGHT SLEEP ***********/
            esp_err_t err = esp_light_sleep_start();
            //********** LIGHT SLEEP ***********/
            
            #if DEBUG
            int64_t t_after_us = esp_timer_get_time();
            sleep_indicator_awake();
            ESP_LOGI(TAG, "sleep return code was %s", esp_err_to_name(err));
            #endif

            ds3231_power_on();

            // Aufwachgrund feststellen
            //uint32_t wakeup_reasons = esp_sleep_get_wakeup_causes();
            #if DEBUG
            const char* wakeup_reason;
            switch (esp_sleep_get_wakeup_cause()) {
                case ESP_SLEEP_WAKEUP_TIMER:
                    wakeup_reason = "TIMER";
                    //TODO:led_blink(3, 500);
                    break;
                case ESP_SLEEP_WAKEUP_GPIO:
                    wakeup_reason = "GPIO";
                    //TODO:led_blink(2, 500);
                    break;
                default:
                    wakeup_reason = "OTHER";
                    //TODO:led_blink(10, 100);
                    break;
            }
            ESP_LOGI(TAG, "Wakeup from light sleep, reason: %s, t=%lld ms, slept for %lld ms\n", wakeup_reason, t_after_us / 1000, (t_after_us - t_before_us) / 1000);
            #endif
            //xTaskResumeAll();

            iot_button_resume();

            //vTaskDelay(pdMS_TO_TICKS(100));
            //xTimerStart(inactivity_timer, pdMS_TO_TICKS(INACTIVITY_TIMEOUT_MS));
        }

        xSemaphoreGive(sleep_semaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


//-------------------------------------------------------------------------------//
// DS3231 I2C RTC-Module with EEPROM
//-------------------------------------------------------------------------------//

// Initialize AT24C EEPROM
esp_err_t ds3231_eeprom_i2c_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing AT24C EEPROM...");
    ret = at24c_init(&at24c_dev, I2C_MASTER_NUM, DS3231_EEPROM_I2C_ADDR,
                     I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO,
                     DS3231_EEPROM_PAGE_SIZE, DS3231_EEPROM_CAPACITY, DS3231_EEPROM_ADDR_WIDTH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AT24C: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "AT24C EEPROM initialized successfully");

    return ESP_OK;
}
//-------------------------------------------------------------------------------//
// Initialize DS3231 RTC
esp_err_t ds3231_rtc_i2c_init(void)
{
    ESP_LOGI(TAG, "Initializing DS3231 MODULE RTC");
    memset(&ds3231_dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(i2c_dev_create_mutex(&ds3231_dev));
    
    ds3231_dev.port = I2C_MASTER_NUM;
    ds3231_dev.addr = DS3231_ADDR;
    ds3231_dev.cfg.sda_io_num = I2C_MASTER_SDA_IO;
    ds3231_dev.cfg.scl_io_num = I2C_MASTER_SCL_IO;
    ds3231_dev.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    ESP_ERROR_CHECK(ds3231_clear_alarm_flags(&ds3231_dev, DS3231_ALARM_BOTH));

    return ESP_OK;
}
//-------------------------------------------------------------------------------//
void test_ds3231(void)
{
    ESP_LOGI(TAG, "=== Testing DS3231 RTC ===");
    
    // Zeit lesen
    struct tm time;
    ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &time));

    ESP_LOGI(TAG, "Current time: %04d-%02d-%02d %02d:%02d:%02d",
             time.tm_year + 1900, time.tm_mon + 1, time.tm_mday,
             time.tm_hour, time.tm_min, time.tm_sec);

    // Temperatur lesen (DS3231 hat einen eingebauten Temperatursensor)
    float temp;
    ESP_ERROR_CHECK(ds3231_get_temp_float(&ds3231_dev, &temp));
    ESP_LOGI(TAG, "✓ Temperature: %.2f°C", temp);
}
//-------------------------------------------------------------------------------//
/**
 * @brief Init GPIO pin for Power-Control of DS3231 RTC
 */
static void ds3231_setup_power_gpio(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DS3231_VCC_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);
    ds3231_power_off();
}
//-------------------------------------------------------------------------------//
/**
 * @brief Power ON the DS3231 RTC (VCC HIGH) and wait 100 ms for stabilization
 */
static void ds3231_power_on(void)
{
    gpio_set_level(DS3231_VCC_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    #if DEBUG
    ESP_LOGI(TAG, "DS3231 power ON (run on VCC)");
    #endif
}
//-------------------------------------------------------------------------------//
/**
 * @brief Power OFF the DS3231 RTC (VCC LOW, runs on battery)
 */
static void ds3231_power_off(void)
{
    gpio_set_level(DS3231_VCC_PIN, 0);
    #if DEBUG
    ESP_LOGI(TAG, "DS3231 power OFF (run on VBAT)");
    #endif
}
//-------------------------------------------------------------------------------//
// Timer Interrupt GPIO als Eingang konfigurieren (pull-up)
static void ds3231_setup_intr_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DS3231_ALARM_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE // GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // ISR Service installieren
    gpio_install_isr_service(0);
    gpio_isr_handler_add(DS3231_ALARM_INT_PIN, gpio_isr_handler, (void*) DS3231_ALARM_INT_PIN);
}
//-------------------------------------------------------------------------------//
/**
 * @brief Configure ESP wakeup from Sleep via DS3231 Alarm interrupt
 */
/*
static void ds3231_configure_wakeup(void)
{
    // ESP32-H2 durch externen Interrupt aufwecken (DS3231 Alarm)
    // RTC GPIO für Deep Sleep konfigurieren

    // TODO: DUMMY
    //esp_sleep_enable_ext0_wakeup(DS3231_ALARM_INT_PIN, 0); // LOW = Alarm ausgelöst
    
    ESP_LOGI(TAG, "Wakeup auf GPIO %d (LOW) konfiguriert", DS3231_ALARM_INT_PIN);
}
*/
//-------------------------------------------------------------------------------//
// DS3231 Alarm setzen
esp_err_t set_ds3231_alarm(led_timer_t timer)
{
    struct tm alarm_time = {0};
    alarm_time.tm_hour = timer.hour;
    alarm_time.tm_min = timer.minute;

    
    ESP_ERROR_CHECK(ds3231_clear_alarm_flags(&ds3231_dev, DS3231_ALARM_2));
    
    ESP_ERROR_CHECK(ds3231_set_alarm(&ds3231_dev, DS3231_ALARM_2, NULL, 0, &alarm_time, DS3231_ALARM2_MATCH_MINHOUR));

    ESP_ERROR_CHECK(ds3231_enable_alarm_ints(&ds3231_dev, DS3231_ALARM_2));
    // Set BB_SQW flag to '1' (needed to preserve SQW pin function in VBAT mode)
    uint8_t control_reg;
    i2c_dev_read_reg(&ds3231_dev, DS3231_ADDR_CONTROL, &control_reg, 1);
    control_reg |= (DS3231_CONTROL_INTCN | DS3231_CONTROL_A2IE | DS3231_CONTROL_BBSQW);
    i2c_dev_write_reg(&ds3231_dev, DS3231_ADDR_CONTROL, &control_reg, 1);
    
    ESP_LOGI(TAG, "Alarm set to: %02d:%02d (%03d%)", alarm_time.tm_hour, alarm_time.tm_min, timer.brightness);
  
    return ESP_OK;
}
//-------------------------------------------------------------------------------//
/*
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
*/
//-------------------------------------------------------------------------------//
// use ds3231_set_time() and ds3231_get_time() from ds3231.h


//-------------------------------------------------------------------------------//
// LED functions
//-------------------------------------------------------------------------------//

/**
 * Configure LED PWM Controller (LEDC)
 * 
 * @param led_gpio GPIO number of LED connected to
 */
/*
static void led_init(gpio_num_t led_gpio)
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

//-------------------------------------------------------------------------------//

// Set duty cycle (0-255 for 8-bit resolution)
static void led_set_brightness(uint8_t brightness)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t) brightness));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    led_brightness = brightness;
}
*/
//-------------------------------------------------------------------------------//

/**
 * Let the LED blink a number of times with a given period.
 * 
 * @param times Number of blinks
 * @param period_ms Period in milliseconds
 */
/*
static void led_blink(uint8_t times, uint32_t period_ms) {
    uint8_t last_brightness = led_brightness;
    uint8_t brightness;
    if (last_brightness == 0) {
        brightness = 255;
    } else {
        brightness = 0;
    }
    for (int i = 1; i <= times; i++) {
        led_set_brightness(brightness);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
        led_set_brightness(last_brightness);
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
}

//-------------------------------------------------------------------------------//

// Task starten
void led_blink_task_start(uint32_t period_ms)
{
    // check task is already running
    if (blink_task_handle != NULL) {
        return;
    }

    // create new task calling "led_blink_task()"
    xTaskCreate(
        led_blink_task,
        "led_blink_task",
        2048,
        (void*)period_ms,
        5,  // Priorität
        &blink_task_handle
    );
}

//-------------------------------------------------------------------------------//

static void led_blink_task(void *pvParameters) {
    uint8_t period_ms = (uint32_t)pvParameters;
    printf("Blink period %d\n", (int)period_ms);
    while (1) {
        led_blink(1, period_ms);
    }
}

//-------------------------------------------------------------------------------//

// Task stoppen
void led_blink_task_stop(void) {
    if (blink_task_handle != NULL) {
        vTaskDelete(blink_task_handle);
        blink_task_handle = NULL;
    }
}
*/

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
            .active_level = BUTTON_ACTIVE_LEVEL,  // Active LOW (Button verbindet GPIO mit GND)
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
    ret = iot_button_register_cb(btn, BUTTON_PRESS_DOWN, on_button_press_down, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von BUTTON_PRESS_DOWN: %s", esp_err_to_name(ret));
    }

    // Button was released up
    ret = iot_button_register_cb(btn, BUTTON_PRESS_UP, on_button_press_up, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Registrieren von BUTTON_PRESS_UP: %s", esp_err_to_name(ret));
    }
    
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

    xTimerReset(inactivity_timer, 0);

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
    xTimerReset(inactivity_timer, 0);

    ESP_LOGI(TAG, "LANGER DRUCK gestartet!");
    state_machine_handle_button_event(BTN_EVENT_LONG_CLICK_START);
}

/**
 * @brief Callback called every ???ms as long as button keeps pressed
 */
static void on_long_press_hold(void *arg, void *usr_data)
{
    xTimerReset(inactivity_timer, 0);

    //ESP_LOGI(TAG, "⏳ LANGER DRUCK wird gehalten...");
    state_machine_handle_button_event(BTN_EVENT_LONG_CLICK_HOLD);
}

/**
 * @brief Callback wenn langer Druck endet
 */
static void on_long_press_up(void *arg, void *usr_data)
{
    xTimerReset(inactivity_timer, 0);

    ESP_LOGI(TAG, "LANGER DRUCK beendet");
    state_machine_handle_button_event(BTN_EVENT_LONG_CLICK_END);
}

/**
 * @brief Callback für Button gedrückt (optional)
 */
static void on_button_press_down(void *arg, void *usr_data)
{
    xTimerReset(inactivity_timer, 0);

    ESP_LOGD(TAG, "↓ Button wurde gedrückt");
}

/**
 * @brief Callback für Button losgelassen (optional)
 */
static void on_button_press_up(void *arg, void *usr_data)
{
    xTimerReset(inactivity_timer, 0);

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
            //on_state_exit_timer(new_state, event);
            break;
        case STATE_PROGRAM:
            on_state_exit_program(new_state, event);
            break;
        case STATE_PROGRAM_DIM:
            //on_state_exit_timer_program_dim(new_state, event);
            break;
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
        case STATE_PROGRAM:
            on_state_enter_program(event);
            break;
        case STATE_PROGRAM_DIM:
            //on_state_enter_program_dim(event);
            break;
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
                    //TODO:led_blink(2, 100); // Feedback während des Wartens
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
                        //TODO:led_set_brightness(led_brightness);
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
                        //TODO:led_set_brightness(led_brightness);
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
        delete_all_timers();
        //TODO:led_blink(5, 250);
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
    //TODO:led_blink(2, 500);
    //TODO:led_fade_off(1000, 0);
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
    //TODO:led_blink(1, 100);
    
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

// ********* TIMER RUN MODE

/**
 * @brief Handler on entry into TIMER state
 * 
 * @param event Event that triggered the state change
 */
static void on_state_enter_timer(state_machine_button_event_t event)
{
    esp_err_t ret;

    //TODO:led_blink(2, 500);
    //TODO:led_fade_on(1000, 255);

    if (count_timers() == 0) {
        ESP_LOGI(TAG, "No timers programmed");
       //TODO: led_blink(5, 100);
        return;
    }
    ESP_LOGI(TAG, "Entering TIMER state");

    // get current time
    struct tm now;
    ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &now));

    // set LED brightness from previous (active) timer
    led_timer_t previous_timer;
    ret = get_previous_timer(&now, &previous_timer);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to get previous timer: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Previous timer: %02d:%02d, brightness: %d", previous_timer.hour, previous_timer.minute, previous_timer.brightness);
    //TODO:led_set_brightness(previous_timer.brightness);

    // set RTC alarm to next timer in future
    led_timer_t next_timer;
    ret = get_next_timer(&now, &next_timer);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Failed to get next timer: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Next timer: %02d:%02d, brightness: %d", next_timer.hour, next_timer.minute, next_timer.brightness);

    ESP_ERROR_CHECK(set_ds3231_alarm(next_timer));
}

/*
static void on_state_exit_timer(state_machine_state_t from_state, state_machine_button_event_t event) {
}
*/

// TIMER PROGRAM MODE
static void on_state_enter_program(state_machine_button_event_t event)
{
    ESP_LOGI(TAG, "Programming new TIMER");
    //TODO:led_blink(2, 500);
    //TODO:led_set_brightness(255);

    // store current time for potential ON timer
    struct tm now;
    ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &now));
    new_timer.hour = now.tm_hour;
    new_timer.minute = now.tm_min;
    new_timer.brightness = 255;
    new_timer.active = false;
}

static void on_state_exit_program(state_machine_state_t from_state, state_machine_button_event_t event)
{
    if (event == BTN_EVENT_TRIPLE_CLICK) {
        ESP_LOGI(TAG, "Programming ABORTED");
        //TODO: led_blink(5, 100);
        //TODO: led_set_brightness(0);
        return;
    }

    ESP_LOGI(TAG, "Add new TIMER");

    // store current time for OFF timer
    struct tm now;
    ESP_ERROR_CHECK(ds3231_get_time(&ds3231_dev, &now));
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
    save_timers_to_eeprom();

    //TODO: led_set_brightness(0);
    print_timers();
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

    // Initialize I2C bus
    ESP_LOGI(TAG, "Initializing I2C bus...");
    ret = i2cdev_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C bus initialized successfully");
    return ESP_OK;
}


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
//-------------------------------------------------------------------------------//
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
        ESP_ERROR_CHECK(save_timers_to_eeprom());
    }
    else if (ret != ESP_OK) {
        return ret;
    }

    // load data into timers array
    uint16_t addr = DS3231_EEPROM_DATA_OFFSET;  
    for (int i = 0; i < MAX_TIMERS; i++)
    {
        ESP_ERROR_CHECK(at24c_read(&at24c_dev, addr, (uint8_t*)&timers[i], sizeof(led_timer_t)));
        addr += sizeof(led_timer_t);       
    }

    ESP_LOGI(TAG, "All timers loaded from EEPROM");
    
    return ESP_OK;
}
//-------------------------------------------------------------------------------//
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
        ESP_ERROR_CHECK(save_timers_to_eeprom());
    }
    else if (ret != ESP_OK) {
        return ret;
    }

    uint16_t addr = DS3231_EEPROM_DATA_OFFSET;
    for (int i = 0; i < MAX_TIMERS; i++)
    {
        ESP_ERROR_CHECK(at24c_write(&at24c_dev, addr, (uint8_t*)&timers[i], sizeof(led_timer_t)));
        addr += sizeof(led_timer_t);       
    }

    ESP_LOGI(TAG, "All timers saved to EEPROM");

    return ESP_OK;
}
//-------------------------------------------------------------------------------//
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
//-------------------------------------------------------------------------------//
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
//-------------------------------------------------------------------------------//
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
//-------------------------------------------------------------------------------//
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
//-------------------------------------------------------------------------------//
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
//-------------------------------------------------------------------------------//
// Zeitvergleich in Minuten seit Mitternacht
static int time_to_minutes(uint8_t hour, uint8_t minute) {
    return hour * 60 + minute;
}
//-------------------------------------------------------------------------------//
// Berechnet Zeitdifferenz vorwärts (mit Überlauf bei Mitternacht)
static int minutes_until(int now_minutes, int target_minutes) {
    if (target_minutes >= now_minutes) {
        return target_minutes - now_minutes;
    } else {
        // Über Mitternacht
        return (24 * 60 - now_minutes) + target_minutes;
    }
}
//-------------------------------------------------------------------------------//
// Berechnet Zeitdifferenz rückwärts (mit Überlauf bei Mitternacht)
static int minutes_since(int now_minutes, int target_minutes) {
    if (target_minutes <= now_minutes) {
        return now_minutes - target_minutes;
    } else {
        // Über Mitternacht zurück
        return now_minutes + (24 * 60 - target_minutes);
    }
}
//-------------------------------------------------------------------------------//
// Zeitvergleich (gibt true zurück wenn t1 < t2)
/*
static bool is_time_before(const led_timer_t *t1, const led_timer_t *t2) {
    if (t1->hour < t2->hour) return true;
    if (t1->hour > t2->hour) return false;
    return t1->minute < t2->minute;
}
*/
//-------------------------------------------------------------------------------//
// Timer nach Uhrzeit sortieren
/*
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
*/

//-------------------------------------------------------------------------------//
// MISC functions
//-------------------------------------------------------------------------------//

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
// CONSOLE MODE functions
//-------------------------------------------------------------------------------//

void check_console_mode_on_boot(gpio_num_t button_gpio, uint8_t button_active_level)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << button_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(50));
  
    if (gpio_get_level(button_gpio) == button_active_level) {
        console_mode = true;
        ESP_LOGI(TAG, "=== CONSOLE MODE ACTIVATED ===");
    }
}

// Hilfsfunktion: String zu Integer konvertieren
static bool str_to_int(const char *str, int *val, int min, int max) {
    char *endptr;
    long num = strtol(str, &endptr, 10);
    
    if (endptr == str || *endptr != '\0' || num < min || num > max) {
        return false;
    }
    
    *val = (int)num;
    return true;
}

static bool str_to_uint8(const char *str, uint8_t *val, uint8_t min, uint8_t max) {
    char *endptr;
    long num = strtol(str, &endptr, 10);
    
    if (endptr == str || *endptr != '\0' || num < min || num > max) {
        return false;
    }
    
    *val = (uint8_t)num;
    return true;
}

#define MAX_LINE_LEN 128

// Watchdog-sichere read_line Funktion mit Echo
static void read_line(char *buf, size_t max_len) {
    size_t idx = 0;
    memset(buf, 0, max_len);

    while (1) {
        int c = getchar();  // line-buffered Input (idf_monitor liefert nach Enter)
        if (c == EOF) {
            // Kein Zeichen verfügbar, Yield für andere Tasks
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (c == '\n' || c == '\r') {
            buf[idx] = '\0';
            putchar('\n');  // Zeilenumbruch ausgeben
            fflush(stdout);
            return;         // Zeile fertig
        } else if (c == 127 || c == 8) {  // Backspace
            if (idx > 0) {
                idx--;
                printf("\b \b");  // letztes Zeichen löschen
                fflush(stdout);
            }
        } else if (c >= 32 && c <= 126) {  // druckbare Zeichen
            if (idx < max_len - 1) {
                buf[idx++] = (char)c;
                putchar(c);  // Echo
                fflush(stdout);
            }
        }

        // Watchdog füttern / Task anderen Tasks CPU geben
        taskYIELD();
    }
}

/*
static void print_datetime(struct tm time)
{
    printf("%02d:%02d:%02d (%02d.%02d.%04d)",
        time.tm_hour, time.tm_min, time.tm_sec,
        time.tm_mday, time.tm_mon + 1, time.tm_year + 1900
    );
}
*/

static void print_led_timer(led_timer_t timer)
{
    printf("%02d:%02d (%3d%%) %s\n",
        timer.hour,
        timer.minute,
        timer.brightness,
        timer.active ? "ACTIVE" : "inactive"
    );
}

// Timer anzeigen
static void show_timers(void) {
    uint8_t count = count_timers();
    
    printf("\n=== Timer-Liste ===\n");
    printf("Anzahl: %d/%d\n\n", count, MAX_TIMERS);
    
    if (count == 0) {
        printf("Keine Timer konfiguriert.\n\n");
        return;
    }
    
    printf("Nr.  Zeit    Helligkeit  Status\n");
    printf("---  ------  ----------  ------\n");
    
    for (int i = 0; i < count; i++) {
        printf("%d. ", i + 1);
        print_led_timer(timers[i]);
        printf("\n");
    }
    printf("\n");
}

// Timer hinzufügen
static void add_timer_interactive(void) {
    char buf[32];
    led_timer_t new_timer = {0};
    
    printf("\n=== Neuer Timer ===\n");
    
    // Stunde eingeben
    printf("Stunde (0-23):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_uint8(buf, &new_timer.hour, 0, 23)) {
        printf("Ungueltige Eingabe!\n\n");
        return;
    }
    
    // Minute eingeben
    printf("Minute (0-59):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_uint8(buf, &new_timer.minute, 0, 59)) {
        printf("Ungueltige Eingabe!\n\n");
        return;
    }
    
    // Helligkeit eingeben
    printf("Helligkeit (0-255):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_uint8(buf, &new_timer.brightness, 0, 255)) {
        printf("Ungueltige Eingabe!\n\n");
        return;
    }
    
    new_timer.active = true;
    
    // Timer hinzufügen
    esp_err_t err = add_timer(new_timer);
    if (err == ESP_OK) {
        printf("Timer erfolgreich hinzugefuegt!\n");
        
        // Automatisch speichern
        err = save_timers_to_eeprom();
        if (err == ESP_OK) {
            printf("Timer im EEPROM gespeichert.\n\n");
        } else {
            printf("FEHLER beim Speichern: %s\n\n", esp_err_to_name(err));
        }
    } else {
        printf("FEHLER: Timer konnte nicht hinzugefuegt werden (%s)\n\n", 
               esp_err_to_name(err));
    }
}

// Timer bearbeiten
static void edit_timer_interactive(void) {
    char buf[32];
    uint8_t idx;
    uint8_t count = count_timers();
    
    if (count == 0) {
        printf("\nKeine Timer zum Bearbeiten vorhanden.\n\n");
        return;
    }
    
    show_timers();
    
    printf("Timer-Nummer (1-%d, 0=Abbruch): ", count);
    read_line(buf, sizeof(buf));
    
    if (!str_to_uint8(buf, &idx, 0, count) || idx == 0) {
        printf("Abbruch.\n\n");
        return;
    }
    
    idx--; // Array-Index
    
    printf("\n=== Timer %d bearbeiten ===\n", idx + 1);
    printf("Aktuelle Werte: %02d:%02d, %d%%, %s\n\n",
           timers[idx].hour, timers[idx].minute, timers[idx].brightness,
           timers[idx].active ? "Aktiv" : "Inaktiv");
    
    // Stunde
    printf("Neue Stunde (0-23, Enter=behalten): ");
    read_line(buf, sizeof(buf));
    if (strlen(buf) > 0) {
        uint8_t val;
        if (str_to_uint8(buf, &val, 0, 23)) {
            timers[idx].hour = val;
        } else {
            printf("Ungueltige Eingabe, Wert behalten.\n");
        }
    }
    
    // Minute
    printf("Neue Minute (0-59, Enter=behalten): ");
    read_line(buf, sizeof(buf));
    if (strlen(buf) > 0) {
        uint8_t val;
        if (str_to_uint8(buf, &val, 0, 59)) {
            timers[idx].minute = val;
        } else {
            printf("Ungueltige Eingabe, Wert behalten.\n");
        }
    }
    
    // Helligkeit
    printf("Neue Helligkeit (0-100, Enter=behalten): ");
    read_line(buf, sizeof(buf));
    if (strlen(buf) > 0) {
        uint8_t val;
        if (str_to_uint8(buf, &val, 0, 100)) {
            timers[idx].brightness = val;
        } else {
            printf("Ungueltige Eingabe, Wert behalten.\n");
        }
    }
    
    // Aktiv/Inaktiv
    printf("Status aendern? (j=aktiv, n=inaktiv, Enter=behalten): ");
    read_line(buf, sizeof(buf));
    if (strlen(buf) > 0) {
        if (buf[0] == 'j' || buf[0] == 'J') {
            timers[idx].active = true;
        } else if (buf[0] == 'n' || buf[0] == 'N') {
            timers[idx].active = false;
        }
    }
    
    printf("\nTimer aktualisiert!\n");
    
    // Speichern
    esp_err_t err = save_timers_to_eeprom();
    if (err == ESP_OK) {
        printf("Aenderungen im EEPROM gespeichert.\n\n");
    } else {
        printf("FEHLER beim Speichern: %s\n\n", esp_err_to_name(err));
    }
}

// Timer löschen
static void delete_timer_interactive(void) {
    char buf[32];
    uint8_t idx;
    uint8_t count = count_timers();
    
    if (count == 0) {
        printf("\nKeine Timer zum Loeschen vorhanden.\n\n");
        return;
    }
    
    show_timers();
    
    printf("Timer-Nummer zum Loeschen (1-%d, 0=Abbruch):\n", count);
    read_line(buf, sizeof(buf));
    
    if (!str_to_uint8(buf, &idx, 0, count) || idx == 0) {
        printf("Abbruch.\n\n");
        return;
    }
    
    idx--; // Array-Index
    
    // Bestätigung
    printf("Timer %d (%02d:%02d) wirklich loeschen? (j/n):\n",
           idx + 1, timers[idx].hour, timers[idx].minute);
    read_line(buf, sizeof(buf));
    
    if (buf[0] != 'j' && buf[0] != 'J') {
        printf("Abbruch.\n\n");
        return;
    }
    
    // Timer löschen (nach vorne schieben)
    for (int i = idx; i < count - 1; i++) {
        timers[i] = timers[i + 1];
    }
    
    // Letzten Timer deaktivieren
    memset(&timers[count - 1], 0, sizeof(led_timer_t));
    
    printf("Timer geloescht!\n");
    
    // Speichern
    esp_err_t err = save_timers_to_eeprom();
    if (err == ESP_OK) {
        printf("Aenderungen im EEPROM gespeichert.\n\n");
    } else {
        printf("FEHLER beim Speichern: %s\n\n", esp_err_to_name(err));
    }
}

static void program_setclock(void)
{
    char buf[32];
    struct tm datetime;
    
    printf("\n=== Set DS3231 time and date ===\n");
    
    printf("Time/hour (0-23):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_int(buf, &datetime.tm_hour, 0, 23)) {
        printf("Invalid input!\n\n");
        return;
    }
    printf("Time/minutes (0-59):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_int(buf, &datetime.tm_min, 0, 59)) {
        printf("Invalid input!\n\n");
        return;
    }
    printf("Time/seconds (0-59):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_int(buf, &datetime.tm_sec, 0, 59)) {
        printf("Invalid input!\n\n");
        return;
    }

    printf("Date/day (1-31):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_int(buf, &datetime.tm_mday, 1, 31)) {
        printf("Invalid input!\n\n");
        return;
    }
    printf("Date/month (1-12):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_int(buf, &datetime.tm_mon, 1, 12)) {
        printf("Invalid input!\n\n");
        return;
    }
    datetime.tm_mon -= 1;
    printf("Date/year (1900-2999):\n");
    read_line(buf, sizeof(buf));
    if (!str_to_int(buf, &datetime.tm_year, 1900, 2999)) {
        printf("Invalid input!\n\n");
        return;
    }
    datetime.tm_year -= 1900;

    esp_err_t err = ds3231_set_time(&ds3231_dev, &datetime);
    if (err == ESP_OK) {
        printf("Clock set successfully!\n");
    } else {
        printf("ERROR setting clock: %s\n\n", esp_err_to_name(err));
    }
}

// Hauptmenü
void timer_terminal_menu(void)
{
    char buf[32];
    
    // Timer aus EEPROM laden
    esp_err_t err = load_timers_from_eeprom();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Fehler beim Laden der Timer: %s", esp_err_to_name(err));
    }
    
    while (1) {
        printf("\n=== PROGRAM MODE ===\n");
        printf("1. Timer anzeigen\n");
        printf("2. Timer hinzufuegen\n");
        printf("3. Timer bearbeiten\n");
        printf("4. Timer loeschen\n");
        printf("5. Timer neu laden (EEPROM)\n");
        printf("6. Alle Timer loeschen\n");
        printf("7. Set current time and date\n");
        printf("8. Reset DS3231 ALARM flags\n");
        printf("0. Beenden (REBOOT)\n");
        printf("\nOption wählen und ENTER drücken: ");
        
        read_line(buf, sizeof(buf));
        
        switch (buf[0]) {
            case '1':
                show_timers();
                break;
            
            case '2':
                add_timer_interactive();
                break;
            
            case '3':
                edit_timer_interactive();
                break;
            
            case '4':
                delete_timer_interactive();
                break;
            
            case '5':
                printf("\nTimer werden neu geladen...\n");
                err = load_timers_from_eeprom();
                if (err == ESP_OK) {
                    printf("Timer erfolgreich geladen.\n\n");
                } else {
                    printf("FEHLER: %s\n\n", esp_err_to_name(err));
                }
                break;
            
            case '6':
                printf("\nAlle Timer wirklich loeschen? (j/n):\n");
                read_line(buf, sizeof(buf));
                if (buf[0] == 'j' || buf[0] == 'J') {
                    memset(timers, 0, sizeof(timers));
                    err = save_timers_to_eeprom();
                    if (err == ESP_OK) {
                        printf("Alle Timer geloescht!\n\n");
                    } else {
                        printf("FEHLER beim Speichern: %s\n\n", esp_err_to_name(err));
                    }
                } else {
                    printf("Abbruch.\n\n");
                }
                break;

            case '7':
                program_setclock();
                break;

            case '8':
                bool a1_flag, a2_flag;
                ds3231_alarm_t alarms;
                ds3231_get_alarm_flags(&ds3231_dev, &alarms);
                a1_flag = (alarms & DS3231_ALARM_1) != 0;
                a2_flag = (alarms & DS3231_ALARM_2) != 0;
                int level = gpio_get_level(DS3231_ALARM_INT_PIN);
                ESP_LOGI(TAG, "SQW: %s | Alarm-Flag 1: %s | Alarm-Flag 2: %s",
                    level ? "---" : "ALARM",
                    a1_flag ? "ALARM" : "---",
                    a2_flag ? "ALARM" : "---"
                );
                printf("\nClear all ALARM flags? (y/n):\n");
                read_line(buf, sizeof(buf));
                if (buf[0] == 'y' || buf[0] == 'y') {
                    err = ds3231_clear_alarm_flags(&ds3231_dev, DS3231_ALARM_BOTH);
                    if (err == ESP_OK) {
                        printf("All ALARM flags cleared!\n\n");
                    } else {
                        printf("ERROR clearing ALARM flags: %s\n\n", esp_err_to_name(err));
                    }
                    ds3231_get_alarm_flags(&ds3231_dev, &alarms);
                    a1_flag = (alarms & DS3231_ALARM_1) != 0;
                    a2_flag = (alarms & DS3231_ALARM_2) != 0;
                    int level = gpio_get_level(DS3231_ALARM_INT_PIN);
                    ESP_LOGI(TAG, "SQW: %s | Alarm-Flag 1: %s | Alarm-Flag 2: %s",
                        level ? "---" : "ALARM",
                        a1_flag ? "ALARM" : "---",
                        a2_flag ? "ALARM" : "---"
                    );
                } else {
                    printf("Operation canceled.\n\n");
                }
                break;

            case '0':
                return;
            
            default:
                printf("\nUngueltige Auswahl!\n");
                break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void debug_task(void *pvParameters)
{
    while(1)
    {
        struct tm now;
        ds3231_get_time(&ds3231_dev, &now);

        bool a1_flag, a2_flag;
        ds3231_alarm_t alarms;
        ds3231_get_alarm_flags(&ds3231_dev, &alarms);
        a1_flag = (alarms & DS3231_ALARM_1) != 0;
        a2_flag = (alarms & DS3231_ALARM_2) != 0;

        int level = gpio_get_level(DS3231_ALARM_INT_PIN);
        ESP_LOGI(TAG, "Uhrzeit: %02d:%02d:%02d (%02d.%02d.%04d) | SQW: %s | Alarm-Flag 1: %s | Alarm-Flag 2: %s",
            now.tm_hour, now.tm_min, now.tm_sec,
            now.tm_mday, now.tm_mon + 1, now.tm_year + 1900,
            level ? "---" : "ALARM",
            a1_flag ? "ALARM" : "---",
            a2_flag ? "ALARM" : "---"
        );
        //print_timers();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


//-------------------------------------------------------------------------------//
// MAIN
//-------------------------------------------------------------------------------//

void app_main(void)
{
    esp_err_t err;

    // set loglevel
    esp_log_level_set(TAG, LOG_LEVEL);
    ESP_LOGI(TAG, "ESP32 Xmas Lightstrip Controller started");

    // Semaphore erstellen
    sleep_semaphore = xSemaphoreCreateBinary();
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    #if DEBUG
    sleep_indicator_init();
    #endif

    // Wakeup-Sources einstellen (BUTTON + RTC-Alarm)
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL); // first, disable ALL wakeup sources

    // Initialize hardware
    //led_init(LED_GPIO); // initialize LED PWM

    // create new LED Controller Task
    led_controller(LED_GPIO);
    while (1)
    {
        led_on(150);
        vTaskDelay(pdMS_TO_TICKS(3000));

        led_blink(255, 0, 500);
        vTaskDelay(pdMS_TO_TICKS(5000));

        led_fade(0, 255, 3000);
        vTaskDelay(pdMS_TO_TICKS(10000));

/*
        ESP_LOGI(TAG, "START BLINKING ...");
        //vTaskDelay(pdMS_TO_TICKS(2000));
        LedConfig_t cfg = { .mode = LED_MODE_BLINK, .duty1 = 255, .period1_ms = 500 };
        led_set_mode(&cfg);
        //led_set_mode(&(LedConfig_t){ .mode = LED_MODE_BLINK, .duty1 = 255, .period1_ms = 500 });

        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "... STOP BLINKING");
        //ESP_LOGI(TAG, "Warte bis LED mode set 2");
        led_set_mode(&(LedConfig_t){ .mode = LED_MODE_OFF });
        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG,"I'm alive...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_set_config(LED_MODE_ON, 0, 255, 0,0,0,0,0);
        ESP_LOGI(TAG,"LED blink");
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_set_config(LED_MODE_BLINK, 500, 255, 0, 0, 0, 0, 10);
        while (!led_pattern_finished()) { vTaskDelay(pdMS_TO_TICKS(100)); }

        ESP_LOGI(TAG,"LED fade on");
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_set_config(LED_MODE_FADE, 1000, 0, 0, 255, 0, 0, 1);
        while (!led_pattern_finished()) { vTaskDelay(pdMS_TO_TICKS(100)); }

        ESP_LOGI(TAG,"LED fade off");
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_set_config(LED_MODE_FADE, 1000, 0, 255, 0, 0, 0, 1);
        while (!led_pattern_finished()) { vTaskDelay(pdMS_TO_TICKS(100)); }
*/

/*
        ESP_LOGI(TAG,"LED pulse");
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_set_config(LED_MODE_PULSE, 2500, 0, 0, 0, 35, 200, -1);
        while (!led_pattern_finished()) { vTaskDelay(pdMS_TO_TICKS(100)); }
*/
/*
        led_set_config(LED_MODE_OFF, 0, 0, 0,0,0,0,0);
        vTaskDelay(pdMS_TO_TICKS(3000));
*/

/*
        ESP_LOGI(TAG,"LED fade on");
        led_set_config(LED_MODE_FADE, 1500, 0, 20, 255, 0, 0, 1);
        while (!led_pattern_finished()) { vTaskDelay(pdMS_TO_TICKS(100)); }

        ESP_LOGI(TAG,"LED fade off");
        led_set_config(LED_MODE_FADE, 1500, 0, 255, 20, 0, 0, 1);
        while (!led_pattern_finished()) { vTaskDelay(pdMS_TO_TICKS(100)); }
        */
        vTaskDelay(pdMS_TO_TICKS(100));
    }




    ds3231_setup_power_gpio(); // DS3231 is OFF at startup
    ds3231_power_on();
    i2c_bus_init(); // initialize I2C bus and devices
    ds3231_rtc_i2c_init();
    ds3231_eeprom_i2c_init();
    ds3231_setup_intr_gpio(); // Setup GPIO connected to DS3231 SQW

    // enable BUTTON wakeup (HIGH) and DS3231 Alarm (LOW)
    gpio_wakeup_enable(BUTTON_GPIO, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(DS3231_ALARM_INT_PIN, GPIO_INTR_LOW_LEVEL); // NOTE: SQW pin get LOW on ALARM
    esp_sleep_enable_gpio_wakeup();

    // enter console mode if button was pressed on boot
    check_console_mode_on_boot(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL);
    if (console_mode) {
        ESP_LOGI(TAG, "Console mode active - sleep disabled");
        //TODO:led_blink_task_start(1000);
        timer_terminal_menu();
        //TODO:led_blink_task_stop();
        printf("\nRestarting system ...\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }

    btn = button_init(); // initialize button with callbacks
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

    // Tasks starten
    xTaskCreate(gpio_event_task, "gpio_event", 2048, NULL, 1, NULL);
    xTaskCreate(sleep_task, "sleep", 2048, NULL, 1, NULL);

    // load timers from EEPROM
    err = load_timers_from_eeprom();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load timers from EEPROM (%s). Try to initialize EEPROM", esp_err_to_name(err));
        //TODO:led_blink(10, 100);
        err = at24c_write_signature(&at24c_dev);
        err = delete_all_timers();
    }
    else {
        #if DEBUG
        at24c_dump(&at24c_dev, 0x0000, 64);
        #endif
        ESP_LOGI(TAG, "Timers loaded from EEPROM successfully");
        print_timers();
    }

    // start sleep-timer (puts module into light-sleep after inactivity)
    inactivity_timer = xTimerCreate(
        "inactivity",
        pdMS_TO_TICKS(INACTIVITY_TIMEOUT_MS),
        pdFALSE,
        NULL,
        inactivity_timer_callback
    );
    xTimerStart(inactivity_timer, 0);
    ESP_LOGI(TAG, "System goes into Light-Sleep after %d s of inactivity\n", (int)(INACTIVITY_TIMEOUT_MS / 1000));

    #if DEBUG
    ESP_LOGI(TAG, "Create debug-task in background");
    xTaskCreate(debug_task, "debug_task", 2048, NULL, 5, NULL);
    #endif

    ESP_LOGI(TAG, "*** SYSTEM READY ***");

    while(1) { vTaskDelay(pdTICKS_TO_MS(100)); }
}