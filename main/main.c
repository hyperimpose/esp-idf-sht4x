/*
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

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensirion_i2c_hal.h"
#include "sht4x.h"
#include "sensirion_common.h"

/* The following is adapted from the usage example in Sensirion's repo. */


#define I2C_PORT    0
#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 6


void app_main(void) {

    vTaskDelay(pdMS_TO_TICKS(3000));  // Small delay until serial connection

    /* Setup I2C ================================================== */

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_master_dev_handle_t dev_handle = NULL;  // SHT45 device handle
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT45_I2C_ADDR_44,  // SHT45 I2C Address
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    /* Initialize the driver ====================================== */

    sensirion_i2c_hal_init();  // Init the HAL. Only called once.
    sensirion_i2c_hal_device_init(&dev_handle);  // Set the device.

    /* Use the sensor ============================================= */

    int16_t error = NO_ERROR;

    /* Reset the Sensor */
    sht4x_soft_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Get the Serial Number */
    uint32_t serial_number = 0;
    error = sht4x_serial_number(&serial_number);
    if (error != NO_ERROR) {
        printf("Error executing serial_number(): %i\n", error);
    } else {
        printf("SHT4x Serial Number: %u\n", (unsigned int) serial_number);
    }

    /* Get Temperature & Humidity readings */
    while (1) {
        int32_t temperature_milli_degC = 0;
        int32_t humidity_milli_RH = 0;

        vTaskDelay(pdMS_TO_TICKS(2000));
        error = sht4x_measure_lowest_precision(&temperature_milli_degC,
					       &humidity_milli_RH);

        if (error != NO_ERROR) {
	    printf("Error executing measure_lowest_precision_ticks(): %i\n",
                   error);
            continue;
	}

        // printf("Temp: %i milli °C: | Hum: %i milli percent RH\n",
        //        temperature_milli_degC, humidity_milli_RH);

        // Convert milli-units to float
	float temp = temperature_milli_degC / 1000.0f;
	float hum = humidity_milli_RH / 1000.0f;
	printf("Temp: %.2f °C | Hum: %.2f %%RH\n", temp, hum);
    }

    /* Misc HAL notes ============================================= */

    /* At this point, if we had multiple sensors set, we could unset the
     * current device, set a new one and take new measurements.
     */

    // sensirion_i2c_hal_device_free(&dev_handle);
    // sensirion_i2c_hal_device_init(&dev_handle2);
    // Take measuremensts here ...
    // sensirion_i2c_hal_device_free(&dev_handle2);

    /* When we are done using the library we can also free the HAL completely */
    sensirion_i2c_hal_free();
}
