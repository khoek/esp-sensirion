#include <driver/i2c.h>
#include <esp_log.h>
#include <libcrc.h>
#include <libesp/marshall.h>
#include <libi2c.h>

#include "private.h"

static const char* TAG = "sensirion";

void sensirion_init(i2c_port_t port, uint8_t addr,
                    sensirion_dev_handle_t* out_dev) {
    sensirion_dev_t* dev = malloc(sizeof(sensirion_dev_t));
    i2c_7bit_init(port, addr, &dev->i2c);
    dev->lock = xSemaphoreCreateMutex();

    *out_dev = dev;
}

void sensirion_destroy(sensirion_dev_handle_t dev) {
    i2c_7bit_destroy(dev->i2c);
    vSemaphoreDelete(dev->lock);
    free(dev);
}

static inline esp_err_t cmd_transact_write(sensirion_dev_handle_t dev,
                                           uint16_t code,
                                           const uint16_t* out_data,
                                           size_t out_count) {
    esp_err_t ret;

    uint8_t reg[2];
    marshall_1u16_to_2u8_be_args(&reg[0], &reg[1], code);

    ESP_LOGD(TAG, "%s: ptr=0x%04X", __func__, code);

    // First decompose the 16-bit words to send as pairs of bytes plus a CRC.
    uint8_t bytes[3 * out_count];
    for (size_t i = 0; i < out_count; i++) {
        marshall_1u16_to_2u8_be_args(&bytes[(3 * i) + 0], &bytes[(3 * i) + 1],
                                     out_data[i]);
        uint8_t crc8 = crc8_calc_sensirion(bytes + (3 * i), 2);
        bytes[(3 * i) + 2] = crc8;

        ESP_LOGD(TAG, "%s: write(%u/%u)=0x%04X, crc=0x%02X", __func__, i,
                 out_count, out_data[i], crc8);
    }

    // Then write 3 byte chunks representing 16 bits (MSB, LSB, CRC) over I2C.
    ret = i2c_7bit_reg_write(dev->i2c, reg, 2, bytes, 3 * out_count);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

static inline esp_err_t cmd_transact_read(sensirion_dev_handle_t dev,
                                          uint16_t* in_data, size_t in_count) {
    esp_err_t ret;

    // First read 3 byte chunks representing 16 bits (MSB, LSB, CRC) over I2C.
    uint8_t bytes[3 * in_count];
    ret = i2c_7bit_reg_read(dev->i2c, NULL, 0, bytes, 3 * in_count);
    if (ret != ESP_OK) {
        return ret;
    }

    // Then assemble the response bytes into 16-bit words and check their CRCs.
    for (size_t i = 0; i < in_count; i++) {
        marshall_2u8_to_1u16_be(in_data + i, bytes + (3 * i));
        uint8_t crc8 = crc8_calc_sensirion(bytes + (3 * i), 2);

        if (bytes[(3 * i) + 2] != crc8) {
            ESP_LOGE(TAG, "crc8 error: read=0x%04X, crc=0x%02X vs 0x%02X",
                     in_data[i], bytes[(3 * i) + 2], crc8);
            return ESP_FAIL;
        }

        ESP_LOGD(TAG, "%s: read(%u/%u)=0x%04X, crc=0x%02X", __func__, i,
                 in_count, in_data[i], crc8);
    }

    return ESP_OK;
}

esp_err_t sensirion_cmd_perform(sensirion_dev_handle_t dev,
                                const sensirion_cmd_def_t* def,
                                const uint16_t* out_data, size_t out_count,
                                uint16_t* in_data, size_t in_count) {
    esp_err_t ret;

    while (xSemaphoreTake(dev->lock, portMAX_DELAY) != pdTRUE)
        ;

    // First transmit the command code/address, and any payload data which is
    // supposed to be sent after it.
    ret = cmd_transact_write(dev, def->code, out_data, out_count);
    if (ret != ESP_OK) {
        goto cmd_transact_out;
    }

    // Wait the datasheet-specified duration.
    vTaskDelay(1 + (def->delay_ms / portTICK_PERIOD_MS));

    // If there is no data to recieve, we are now done.
    if (!in_count) {
        goto cmd_transact_out;
    }

    // Otherwise, recieve the data which we are supposed to get back,
    // noting that we have now waited the neccesary `delay_ms`.
    ret = cmd_transact_read(dev, in_data, in_count);
    if (ret != ESP_OK) {
        goto cmd_transact_out;
    }

cmd_transact_out:
    xSemaphoreGive(dev->lock);
    return ret;
}
