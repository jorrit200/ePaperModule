#include <stdio.h>
#include <sys/signal.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_now.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define ONBOARD_GREEN_LED_GPIO GPIO_NUM_0
#define WIFI_CHANNEL 6

// SPI GPIO config
#define PIN_MOSI 21
#define PIN_CLK 18
#define PIN_CS 5
#define PIN_DC 17
#define PIN_RST 10
#define PIN_BUSY 9
#define PIN_BUTTON 8

#define TAG "EPD TEST"

spi_device_handle_t epd_spi;

typedef void (*task_fn_t)(void *);

typedef struct {
    task_fn_t fn;
    void *arg;
} queued_task_t;

static QueueHandle_t send_esp_now_queue;

static uint16_t row_count = 0;

esp_err_t save_host(const esp_now_peer_info_t* host) {
    nvs_handle_t h;
    esp_err_t err = nvs_open("host", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, "1", host, sizeof(*host));
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);
    return err;
}

esp_err_t load_host(esp_now_peer_info_t* host) {
    nvs_handle_t h;
    esp_err_t err = nvs_open("host", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    size_t l = sizeof(*host);
    err = nvs_get_blob(h, "1", host, &l);

    nvs_close(h);
    return err;
}

typedef enum  {
    NONE,
    RECEIVING_BITMAP
} program_state_t;

esp_err_t save_state(const program_state_t* state) {
    nvs_handle_t h;
    esp_err_t err = nvs_open("program_state", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    const size_t l = sizeof(*state);
    err = nvs_set_blob(h, "1", state, l);
    if (err == ESP_OK) err = nvs_commit(h);

    nvs_close(h);
    return err;
}

esp_err_t load_state(program_state_t* state) {
    nvs_handle_t h;
    esp_err_t err = nvs_open("program_state", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    size_t l = sizeof(*state);
    err = nvs_get_blob(h, "1", state, &l);

    nvs_close(h);
    return err;
}

bool submit_send_esp_now_task(task_fn_t fn, void *arg) {
    const queued_task_t task = {
        .fn = fn,
        .arg = arg,
    };
    return xQueueSendToBack(send_esp_now_queue, &task, pdMS_TO_TICKS(10)) == pdTRUE;
}

uint8_t *build_pong(const uint16_t screen_width, const uint16_t screen_height, size_t *out_len) {
    const char *response = "CLIENT_PONG.";
    *out_len = strlen(response);
    return (uint8_t *) response;
    const size_t len = strlen(response);
    *out_len = len + 2 + 2;
    uint8_t *out = malloc(*out_len);
    if (!out) return NULL;

    size_t offset = 0;

    memcpy(out, response, len);
    offset += len;

    out[offset++] = screen_width & 0xFF;
    out[offset++] = (screen_width >> 8) & 0xFF;

    out[offset++] = screen_height & 0xFF;
    out[offset] = (screen_height >> 8) & 0xFF;

    return out;
}

struct pong_data {
    const uint8_t *pong_buffer;
    const size_t pong_len;
    const uint8_t *peer_address;
};

typedef struct pong_data pong_data_t;


void epd_send_command(uint8_t command) {
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &command,
    };
    ESP_ERROR_CHECK(spi_device_transmit(epd_spi, &t));
}

void wait_until_idle() {
    ESP_LOGI(TAG, "Waiting for idle");
    do {
        vTaskDelay(pdMS_TO_TICKS(50));
    } while (gpio_get_level(PIN_BUSY) == 0);
    ESP_LOGI(TAG, "IDLE");
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
    ESP_LOGI(TAG, "reset complete");
    wait_until_idle();
}

void epd_init_uc8179(void) {
    epd_send_command(0x06); // BOOSTER SOFT START
    epd_send_data(0xE7);
    // phase A: 11, 010, 111. voltage ramp time, drive strength, minimum off time. 40 ms, "moderate", 6.58 µs
    epd_send_data(0xE7);
    // phase B: 11, 010, 111. voltage ramp time, drive strength, minimum off time. 40 ms, "moderate", 6.58 µs
    epd_send_data(0xE7); // phase C1: --, 010, 111. -, drive strength, minimum off time. -, "moderate", 6.58 µs
    epd_send_data(0xE7);
    // phase CE, -, C2 strength, C2 minimum off time. 0, -, 010, 111, C2-disable, "moderate", 6.58 µs

    epd_send_command(0x01); // POWER SETTING
    epd_send_data(0x07); // ---, BD_EN, -, VSR_EN, VS_EN, VG_En. ---0-111.
    epd_send_data(0x17); // VPP_EN, --, VCOM_SLEW, -, VG_LEVEL[2:0]. Disabled, "fast", 20 V, 0--1-111
    epd_send_data(0x3A); // --, VDH_LVL[5:0], --111010. High polarity voltage = 14 V
    epd_send_data(0x3A); // --, VDL_LVL[5:0], --111010. Low polarity voltage = -14 V
    epd_send_data(0x03); // --, VDHR_LVL[5:0], --00011. Red pixel voltage = 3 V

    epd_send_command(0x04);
    wait_until_idle();

    epd_send_command(0x00); // PANEL SETTINGS
    epd_send_data(0x1F); // --, REG, KW/R, UD, SHL, SHD_N, RST_N. --011111

    // PLL
    epd_send_command(0x30);
    epd_send_data(0x3A);


    // Resolution
    epd_send_command(0x61); // Set resolution
    epd_send_data(0x03);
    epd_send_data(0x20); // width: 800px
    epd_send_data(0x01);
    epd_send_data(0xE0); // height: 480px

    ESP_LOGI(TAG, "UC8179 init complete");
}

void epd_clear_white(void) {
    ESP_LOGI(TAG, "clearing to white");

    epd_send_command(0x13);
    for (int row = 0; row < 480; row++) {
        for (int col = 0; col < 800 / 8; col++) {
            const uint8_t d = 0x00;
            epd_send_data(d);
        }
        vTaskDelay(1); // let the watchdog breathe
    }

    epd_send_command(0x12); // DISPLAY REFRESH
    ESP_LOGI(TAG, "updating the screen");

    wait_until_idle();
}

void epd_deep_sleep(void) {
    epd_send_command(0x07);
    epd_send_data(0xA5);
}


void send_pong(void *arg) {
    const pong_data_t *data = arg;
    ESP_LOGI(TAG, "Sending pong: ");
    for (size_t i = 0; i < data->pong_len; i++) {
        putchar(data->pong_buffer[i]);
    }
    putchar('\n');
    ESP_LOGI(TAG, "To peer address: ");
    for (size_t i = 0; i < ESP_NOW_ETH_ALEN; i++) {
        printf("%02X", (unsigned int) data->peer_address[i]);
    }

    if (!esp_now_is_peer_exist(data->peer_address)) {
        esp_now_peer_info_t host_peer = {
            .channel = WIFI_CHANNEL,
            .ifidx = WIFI_IF_STA,
            .encrypt = false
        };
        memcpy(host_peer.peer_addr, data->peer_address, ESP_NOW_ETH_ALEN);
        esp_now_add_peer(&host_peer);
        save_host(&host_peer);
    }

    const esp_err_t err = esp_now_send(data->pong_buffer, data->peer_address, data->pong_len);
    if (err != ESP_ERR_ESPNOW_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to send pong: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Pong sent");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
}

void listen_for_host_broadcast(const esp_now_recv_info_t *recv_info, const uint8_t *data, const int len) {
    printf("Received packet (%d bytes) from %02X:%02X:%02X:%02X:%02X:%02X: ",
           len,
           recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
           recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    for (int i = 0; i < len; i++) putchar(data[i]);
    putchar('\n');

    if (strncmp((const char *) data, "HOST_DISCOVERY_PING", len) == 0) {
        ESP_LOGI(TAG, "Added host");


        size_t pong_size;
        const uint8_t *pong = build_pong(800, 480, &pong_size);

        pong_data_t pong_data = {
            .pong_buffer = pong,
            .pong_len = pong_size,
            .peer_address = recv_info->src_addr
        };
        // submit_send_esp_now_task(send_pong, &pong_data);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_sleep_enable_timer_wakeup(1 * 1000000);
        ESP_LOGI(TAG, "Going to sleep for 1 second");
        esp_deep_sleep_start();
    }

    if (strncmp((const char *) data, "TRANSMITTING BITMAP", len) == 0) {
        const program_state_t state = RECEIVING_BITMAP;
        save_state(&state);
        esp_sleep_enable_timer_wakeup(5000);
        esp_deep_sleep_start();
    }

}

void listen_for_host_bitmap(const esp_now_recv_info_t *recv_info, const uint8_t *data, const int len) {
    ESP_LOGI(TAG, "listening for bitmap parts");
    for (int i = 0; i < len; i++) {
        epd_send_data(data[i]);
    }
    if (++row_count >= 480) {
        epd_send_command(0x12); // DISPLAY REFRESH
        wait_until_idle();
        save_state(NONE);
        esp_sleep_enable_timer_wakeup(5000);
        esp_deep_sleep_start();
    }
}


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

    gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
}

void init_task_queue() {
    send_esp_now_queue = xQueueCreate(10, sizeof(queued_task_t));
    if (send_esp_now_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
    }
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

void task_runner(void *pvParamaters) {
    queued_task_t task;
    for (;;) {
        if (xQueueReceive(send_esp_now_queue, &task, portMAX_DELAY)) {
            ESP_LOGI(TAG, "send esp now task received");
            if (task.fn != NULL) {
                task.fn(task.arg);
            }
        }
    }
}

void app_main(void) {
    // xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
    gpio_init();
    spi_init();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    const wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_BELOW));

    ESP_ERROR_CHECK(esp_now_init());

    static esp_now_peer_info_t broadcast_peer = {
        .channel = WIFI_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false
    };
    memset(broadcast_peer.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);

    const esp_err_t add_peer_err = esp_now_add_peer(&broadcast_peer);
    if (add_peer_err == ESP_OK) {
        ESP_LOGI(TAG, "added broadcast peer");
    } else {
        ESP_LOGE(TAG, "failed to add broadcast peer: %s", esp_err_to_name(add_peer_err));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    if (gpio_get_level(PIN_BUTTON) == 1) {
        esp_err_t err = nvs_flash_erase();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "nvs erase done");
            err = nvs_flash_init();
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "re-init flash");
            } else {
                ESP_LOGE(TAG, "re-init nvs flash failed: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "nvs erase failed: %s", esp_err_to_name(err));
        }
    }

    init_task_queue();

    program_state_t state;
    load_state(&state);

    if (state == RECEIVING_BITMAP) {
        ESP_LOGI(TAG, "receiving bitmap");
        epd_reset();
        epd_init_uc8179();
        epd_send_command(0x13);
        esp_now_register_recv_cb(listen_for_host_bitmap);
        return;
    }

    esp_now_peer_info_t recovered_host;
    if (load_host(&recovered_host) == ESP_OK) {
        ESP_LOGI(TAG, "recovered host  %02X:%02X:%02X:%02X:%02X:%02X\n",
                 recovered_host.peer_addr[0], recovered_host.peer_addr[1], recovered_host.peer_addr[2],
                 recovered_host.peer_addr[3], recovered_host.peer_addr[4], recovered_host.peer_addr[5]);
        epd_reset();
        epd_init_uc8179();
        epd_clear_white();
    } else {
        ESP_LOGE(TAG, "no host in nvs, waiting for host ping.");
        esp_now_register_recv_cb(listen_for_host_broadcast);
    }
    xTaskCreate(task_runner, "task_runner", 4096, NULL, 5, NULL);
}
