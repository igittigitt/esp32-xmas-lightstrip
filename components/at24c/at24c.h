// components/at24c/at24c.h
#ifndef AT24C_H
#define AT24C_H

#include "i2cdev.h"
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AT24C EEPROM handle structure
 */
typedef struct {
    i2c_dev_t i2c_dev;      // i2c_dev_t (nicht i2cdev_t)
    uint16_t page_size;     // Page size in bytes (z.B. 32, 64, 128)
    uint32_t capacity;      // Total capacity in bytes
    uint8_t addr_width;     // Address width: 1 = 8-bit, 2 = 16-bit
} at24c_handle_t;

/**
 * @brief Initialize AT24C EEPROM
 * 
 * @param handle Pointer to AT24C handle
 * @param port I2C port number
 * @param dev_addr I2C device address (usually 0x50-0x57)
 * @param sda_gpio GPIO number for SDA
 * @param scl_gpio GPIO number for SCL
 * @param page_size Page size in bytes
 * @param capacity Total capacity in bytes
 * @param addr_width Address width (1 or 2 bytes)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t at24c_init(at24c_handle_t *handle,
                     i2c_port_t port,
                     uint8_t dev_addr,
                     gpio_num_t sda_gpio,
                     gpio_num_t scl_gpio,
                     uint16_t page_size,
                     uint32_t capacity,
                     uint8_t addr_width);

/**
 * @brief Read data from EEPROM
 * 
 * @param handle AT24C handle
 * @param addr Start address
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return esp_err_t ESP_OK on success
 */
esp_err_t at24c_read(at24c_handle_t *handle, uint16_t addr, uint8_t *data, size_t len);

/**
 * @brief Write data to EEPROM
 * 
 * @param handle AT24C handle
 * @param addr Start address
 * @param data Data to write
 * @param len Number of bytes to write
 * @return esp_err_t ESP_OK on success
 */
esp_err_t at24c_write(at24c_handle_t *handle, uint16_t addr, const uint8_t *data, size_t len);

/**
 * @brief Deinitialize AT24C EEPROM
 * 
 * @param handle AT24C handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t at24c_deinit(at24c_handle_t *handle);

esp_err_t at24c_check_signature(at24c_handle_t *handle);

esp_err_t at24c_write_signature(at24c_handle_t *handle);

void at24c_dump(at24c_handle_t *handle, int address, size_t len);

void _at24c_print_hex(const uint8_t *data, size_t len);

/**
 * @brief Test AT24C EEPROM functionality
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t test_at24c(at24c_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // AT24C_H