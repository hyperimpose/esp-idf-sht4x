#include "driver/i2c_types.h"

#include "../embedded-i2c-sht4x/sht4x_i2c.h"
#include "../embedded-i2c-sht4x/sensirion_i2c_hal.h"

extern void sensirion_i2c_hal_device_init(i2c_master_dev_handle_t* hdl);
extern void sensirion_i2c_hal_device_free(i2c_master_dev_handle_t* hdl);
