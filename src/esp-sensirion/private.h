#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <libi2c.h>

#include "device/sensirion.h"

struct sensirion_dev {
    i2c_7bit_handle_t i2c;

    SemaphoreHandle_t lock;
};
