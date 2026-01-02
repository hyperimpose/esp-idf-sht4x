/*
 * Copyright (c) 2018, Sensirion AG
 * Copyright (c) 2026, hyperimpose.org
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_rom_sys.h"

#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"


/*
 * ESP-IDF Implementation Notes
 * ============================
 *
 * The HAL API as defined in sensirion_i2c_hal.h includes the following:
 * - sensirion_i2c_hal_select_bus/1
 * - sensirion_i2c_hal_init/0
 * - sensirion_i2c_hal_free/0
 * Those functions are not used internally by the driver and they seem to only
 * exist as an external API for the users of the library.
 *
 * Bus lifecycle and device registration are handled by the user application to
 * allow for shared bus configurations. To do this, we introduce two functions:
 * - sensirion_i2c_hal_device_init/1
 * - sensirion_i2c_hal_device_free/0
 *
 * Devices handles
 * ---------------
 * The device used by the driver is set in the pointer `*sht4x_dev_handle'.
 *
 * Use `sensirion_i2c_hal_device_init/1' to set the pointer to the active
 * I2C device handle. Use `sensirion_i2c_hal_device_free/0' to unset it.
 *
 * You MUST free the pointer before setting it to a new device.
 *
 * Thread safety
 * -------------
 * The pointer `*sht4x_dev_handle' is protected by a mutex.
 *
 * Calling `sensirion_i2c_hal_device_init/1' sets the mutex and blocks any
 * further calls until `sensirion_i2c_hal_device_free/0' is called.
 *
 * ---
 *
 * The read/write functions are thread safe according to the ESP-IDF docs.
 * This means that you can query the same device from different threads.
 *
 * See: https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c6
 *                                /api-reference/peripherals
 *                                /i2c.html#thread-safety
 *
 * Ignored arguments
 * -----------------
 * The `address' parameter in read/write functions is ignored because the
 * ESP-IDF handle already contains the target address and bus context.
 */

static i2c_master_dev_handle_t *sht4x_dev_handle = NULL;
static SemaphoreHandle_t sht4x_dev_mutex = NULL;


void sensirion_i2c_hal_device_init(i2c_master_dev_handle_t* handle) {
    if (sht4x_dev_mutex)
        if (xSemaphoreTake(sht4x_dev_mutex, portMAX_DELAY) == pdTRUE)
            sht4x_dev_handle = handle;
}

void sensirion_i2c_hal_device_free(void) {
    sht4x_dev_handle = NULL;
    if (sht4x_dev_mutex)
        xSemaphoreGive(sht4x_dev_mutex);
}


/*
 * INSTRUCTIONS
 * ============
 *
 * Implement all functions where they are marked as IMPLEMENT.
 * Follow the function specification in the comments.
 */

/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) {
    return NOT_IMPLEMENTED_ERROR;
}

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_hal_init(void) {
    if (sht4x_dev_mutex == NULL) {
        sht4x_dev_mutex = xSemaphoreCreateMutex();
    }
}

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */
void sensirion_i2c_hal_free(void) {
    if (sht4x_dev_mutex != NULL) {
        vSemaphoreDelete(sht4x_dev_mutex);
        sht4x_dev_mutex = NULL;
    }
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    if (sht4x_dev_handle == NULL) {
        return -1;
    }
    esp_err_t ret = i2c_master_receive(*sht4x_dev_handle, data, count, -1);
    return (ret == ESP_OK) ? 0 : -1;
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data,
                               uint8_t count) {
    if (sht4x_dev_handle == NULL) {
        return -1;
    }
    esp_err_t ret = i2c_master_transmit(*sht4x_dev_handle, data, count, -1);
    return (ret == ESP_OK) ? 0 : -1;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    /* NOTE: esp_rom_delay_us is part of the esp-idf internal API.
     *
     * Using vTaskDelay alone also works fine.
     */
    if (useconds < 10000) {
        esp_rom_delay_us(useconds);
    } else {
        uint32_t msec = (useconds + 999) / 1000;
        vTaskDelay(pdMS_TO_TICKS(msec));
    }
}
