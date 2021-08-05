#pragma once

#include <driver/i2c.h>
#include <libi2c.h>

typedef struct sensirion_cmd_def {
    uint16_t code;
    uint32_t delay_ms;
} sensirion_cmd_def_t;

typedef struct sensirion_dev sensirion_dev_t;
typedef sensirion_dev_t* sensirion_dev_handle_t;

// Register the device on the given I2C bus.
void sensirion_init(i2c_port_t port, uint8_t addr,
                    sensirion_dev_handle_t* out_dev);

// Release the given handle.
void sensirion_destroy(sensirion_dev_handle_t dev);

// Perform a command over I2C. Use of these functions is thread-safe.
__result_use_check esp_err_t
sensirion_cmd_perform(sensirion_dev_handle_t dev,
                      const sensirion_cmd_def_t* cmd, const uint16_t* out_data,
                      size_t out_count, uint16_t* in_data, size_t in_count);
