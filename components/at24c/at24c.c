// components/at24c/at24c.c
#include "at24c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "AT24C";

#define AT24C_WRITE_DELAY_MS 10 // Write cycle time in milliseconds
#define AT24C_SIGNATURE "XMAS"

//-------------------------------------------------------------------------------//

esp_err_t at24c_init(at24c_handle_t *handle,
                     i2c_port_t port,
                     uint8_t dev_addr,
                     gpio_num_t sda_gpio,
                     gpio_num_t scl_gpio,
                     uint16_t page_size,
                     uint32_t capacity,
                     uint8_t addr_width)
{
    if (!handle) {
        ESP_LOGE(TAG, "Handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    handle->page_size = page_size;
    handle->capacity = capacity;
    handle->addr_width = addr_width;

    memset(&handle->i2c_dev, 0, sizeof(i2c_dev_t));
    esp_err_t ret = i2c_dev_create_mutex(&handle->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create mutex: %s", esp_err_to_name(ret));
        return ret;
    }

    handle->i2c_dev.port = port;
    handle->i2c_dev.addr = dev_addr;
    handle->i2c_dev.cfg.sda_io_num = sda_gpio;
    handle->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    handle->i2c_dev.cfg.master.clk_speed = 100000;
#endif

    ESP_LOGI(TAG, "AT24C initialized - addr:0x%02X, page:%d, capacity:%lu, addr_width:%d",
             dev_addr, page_size, capacity, addr_width);
    return ESP_OK;
}

//-------------------------------------------------------------------------------//

esp_err_t at24c_read(at24c_handle_t *handle, uint16_t addr, uint8_t *data, size_t len)
{
    if (!handle || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (addr + len > handle->capacity) {
        ESP_LOGE(TAG, "Read beyond capacity");
        return ESP_ERR_INVALID_ARG;
    }

    I2C_DEV_TAKE_MUTEX(&handle->i2c_dev);

    if (handle->addr_width == 2) {
        uint8_t addr_bytes[2];
        addr_bytes[0] = (addr >> 8) & 0xFF;
        addr_bytes[1] = addr & 0xFF;

        I2C_DEV_CHECK(&handle->i2c_dev, i2c_dev_write(&handle->i2c_dev, NULL, 0, addr_bytes, 2));
        I2C_DEV_CHECK(&handle->i2c_dev, i2c_dev_read(&handle->i2c_dev, NULL, 0, data, len));
    } else {
        I2C_DEV_CHECK(&handle->i2c_dev, i2c_dev_read_reg(&handle->i2c_dev, addr & 0xFF, data, len));
    }

    I2C_DEV_GIVE_MUTEX(&handle->i2c_dev);

    return ESP_OK;
}

//-------------------------------------------------------------------------------//

esp_err_t at24c_write(at24c_handle_t *handle, uint16_t addr, const uint8_t *data, size_t len)
{
    if (!handle || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (addr + len > handle->capacity) {
        ESP_LOGE(TAG, "Write beyond capacity");
        return ESP_ERR_INVALID_ARG;
    }

    size_t remaining = len;
    size_t offset = 0;

    while (remaining > 0) {
        uint16_t current_addr = addr + offset;
        size_t page_offset = current_addr % handle->page_size;
        size_t bytes_to_write = handle->page_size - page_offset;
        if (bytes_to_write > remaining) {
            bytes_to_write = remaining;
        }

        I2C_DEV_TAKE_MUTEX(&handle->i2c_dev);

        if (handle->addr_width == 2) {
            uint8_t write_buf[2 + bytes_to_write];
            write_buf[0] = (current_addr >> 8) & 0xFF;
            write_buf[1] = current_addr & 0xFF;
            memcpy(&write_buf[2], &data[offset], bytes_to_write);

            I2C_DEV_CHECK(&handle->i2c_dev, 
                         i2c_dev_write(&handle->i2c_dev, NULL, 0, write_buf, 2 + bytes_to_write));
        } else {
            I2C_DEV_CHECK(&handle->i2c_dev, 
                         i2c_dev_write_reg(&handle->i2c_dev, current_addr & 0xFF, 
                                          &data[offset], bytes_to_write));
        }

        I2C_DEV_GIVE_MUTEX(&handle->i2c_dev);

        vTaskDelay(pdMS_TO_TICKS(AT24C_WRITE_DELAY_MS));

        remaining -= bytes_to_write;
        offset += bytes_to_write;
    }

    return ESP_OK;
}

//-------------------------------------------------------------------------------//

esp_err_t at24c_check_signature(at24c_handle_t *handle)
{
    esp_err_t ret;

    size_t signature_len = strlen(AT24C_SIGNATURE);
    uint8_t signature[signature_len];

    ret = at24c_read(handle, 0x00, signature, signature_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read signature from EEPROM: %s", esp_err_to_name(ret));
        return ret;
    }

    if (memcmp(signature, AT24C_SIGNATURE, signature_len) != 0) {
        ESP_LOGW(TAG, "Invalid EEPROM signature");
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

//-------------------------------------------------------------------------------//

esp_err_t at24c_write_signature(at24c_handle_t *handle)
{
    esp_err_t ret;

    size_t signature_len = strlen(AT24C_SIGNATURE);

    ret = at24c_write(handle, 0x00, (uint8_t *)AT24C_SIGNATURE, signature_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write signature to EEPROM: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

//-------------------------------------------------------------------------------//

esp_err_t at24c_deinit(at24c_handle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_dev_delete_mutex(&handle->i2c_dev);
}

//-------------------------------------------------------------------------------//

void at24c_dump(at24c_handle_t *handle, int address, size_t len)
{
    uint8_t eeprom_buf[len];
    at24c_read(handle, address, eeprom_buf, len);
    _at24c_print_hex(eeprom_buf, len);
}

//-------------------------------------------------------------------------------//

void _at24c_print_hex(const uint8_t *data, size_t len)
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

esp_err_t test_at24c(at24c_handle_t *handle)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== Testing AT24C EEPROM ==========");
    
    const char *test_string = "Hello from ESP32!";
    uint16_t test_addr = 0x0100;
    
    ESP_LOGI(TAG, "Writing: '%s'", test_string);
    
    esp_err_t ret = at24c_write(handle, test_addr, (uint8_t *)test_string, strlen(test_string) + 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✓ Written");

    char read_buffer[64] = {0};
    ret = at24c_read(handle, test_addr, (uint8_t *)read_buffer, strlen(test_string) + 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "✓ Read: '%s'", read_buffer);
    
    if (strcmp(test_string, read_buffer) == 0) {
        ESP_LOGI(TAG, "✓ SUCCESS: Data matches!");
    } else {
        ESP_LOGE(TAG, "✗ ERROR: Data mismatch!");
    }
    
    ESP_LOGI(TAG, "==========================================");
    return ESP_OK;
}

//-------------------------------------------------------------------------------//
