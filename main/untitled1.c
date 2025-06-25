#include <stdio.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


#define ONBOARD_GREEN_LED_GPIO GPIO_NUM_0

// SPI GPIO config
#define PIN_MOSI 21
#define PIN_CLK 18
#define PIN_CS 5
#define PIN_DC 17
#define PIN_RST 10
#define PIN_BUSY 9

#define TAG "EPD TEST"

spi_device_handle_t epd_spi;

void spi_init(void) {
    const spi_bus_config_t bus_config = {
        .miso_io_num = -1, // no master in slave out pin configured on the board.
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4069,
    };

    const spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 2 * 1000 * 1000, // todo: find proper speed
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_config, &epd_spi));
}

void gpio_init(void) {
    gpio_reset_pin(ONBOARD_GREEN_LED_GPIO);
    gpio_set_direction(ONBOARD_GREEN_LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_direction(PIN_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_BUSY, GPIO_MODE_INPUT);
}

void epd_send_command(uint8_t command) {
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &command,
    };
    ESP_ERROR_CHECK(spi_device_transmit(epd_spi, &t));
}

void epd_send_data(uint8_t data) {
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(epd_spi, &t));
}

void epd_reset(void) {
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
}

void blink_task(void *pvParameters) {

    for (;;) {
        gpio_set_level(ONBOARD_GREEN_LED_GPIO, 1);
        ESP_LOGI(TAG, "i led on");

        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(ONBOARD_GREEN_LED_GPIO, 0);
        ESP_LOGI(TAG, "i led off");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    // xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
    gpio_init();
    spi_init();
    epd_reset();

    ESP_LOGI(TAG, "Sending dummy commands...");

    epd_send_command(0x01);

    epd_send_data(0x03);
    epd_send_data(0x00);
    // ok lets go
    epd_send_data(0x2b);
    epd_send_data(0x2b);

    epd_send_command(0x04);

    while (gpio_get_level(PIN_BUSY) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Display responded!");
}

