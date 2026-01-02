#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#define LV_HOR_RES_MAX              (240)
#define LV_VER_RES_MAX              (135)

#define DEMO_LCD_BK_LIGHT_OFF_LEVEL 0
#define DEMO_LCD_BK_LIGHT_ON_LEVEL  1
#define DEMO_LCD_CMD_BITS           8
#define DEMO_LCD_PARAM_BITS         8
#define DEMO_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)

#define DEMO_PIN_NUM_BK_LIGHT       GPIO_NUM_45
#define DEMO_PIN_NUM_LCD_CS         GPIO_NUM_42
#define DEMO_PIN_NUM_LCD_DC         GPIO_NUM_40
#define DEMO_PIN_NUM_LCD_RST        GPIO_NUM_41
#define DEMO_PIN_NUM_MISO           -1
#define DEMO_PIN_NUM_MOSI           GPIO_NUM_35
#define DEMO_PIN_NUM_SCLK           GPIO_NUM_36

void update_battery_ui(unsigned voltage, unsigned voltage_deci, unsigned capacity, unsigned capacity_deci);
void update_button_ui(bool b0, bool b1, bool b2);
void demo_ui(lv_display_t *disp);

static uint8_t display_buffer[LV_HOR_RES_MAX * LV_VER_RES_MAX * 2];
static uint8_t test_square[16 * 16 * 2];

static const char* TAG = "Main";

static esp_lcd_panel_handle_t panel_handle = NULL;

// Using SPI2 in the demo
#define LCD_HOST  SPI2_HOST

typedef enum {
    BATTERY_UPDATE,
    BUTTON_UPDATE,
} demo_data_update_type_t;

typedef struct {
    unsigned voltage;
    unsigned voltage_deci;
    unsigned capacity;
    unsigned capacity_deci;
} battery_update_t;

typedef struct {
    bool b0;
    bool b1;
    bool b2;
} button_update_t;

typedef struct {
    demo_data_update_type_t update_type;
    union {
        battery_update_t battery_update;
        button_update_t button_update;
    };
} demo_data_t;

static demo_data_t demo_data;
static QueueHandle_t demo_data_queue = NULL;

bool demo_lock(SemaphoreHandle_t lock, uint32_t timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lock, timeout_ticks) == pdTRUE;
}

void demo_unlock(SemaphoreHandle_t lock)
{
    xSemaphoreGiveRecursive(lock);
}

static void demo_display_task(void *arg)
{
    ESP_LOGI(TAG, "Starting Display Update task");

    while (1) {
        if (xQueueReceive(demo_data_queue, &demo_data, (TickType_t)4) == pdPASS) {
            if (lvgl_port_lock(0)) {
                if (demo_data.update_type == BATTERY_UPDATE) {
                    update_battery_ui(demo_data.battery_update.voltage,
                            demo_data.battery_update.voltage_deci,
                            demo_data.battery_update.capacity,
                            demo_data.battery_update.capacity_deci);
                } else if (demo_data.update_type == BUTTON_UPDATE) {
                    update_button_ui(demo_data.button_update.b0, 
                            demo_data.button_update.b1,
                            demo_data.button_update.b2);
                }
                lvgl_port_unlock();
            }
        }
    }
}

static void demo_battery_task(void* arg)
{
    unsigned current_vcell = 0;
    unsigned current_vcell_deci = 0;
    unsigned current_soc = 0;
    unsigned current_soc_deci = 0;

    ESP_LOGI("Battery", "Starting battery task");

    // Enable the I2C bus
    ESP_LOGI("Battery", "Initializing I2C bus as master");
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_bus_config_t i2c_master_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .intr_priority = 0,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_4,
        .sda_io_num = GPIO_NUM_3,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &i2c_bus_handle));

    ESP_LOGI("Battery", "Adding battery gauge device to I2C bus");
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_master_dev_handle_t battery_gauge_device_handle;
    i2c_device_config_t battery_gauge_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &battery_gauge_device_config, &battery_gauge_device_handle));

    ESP_LOGI("Battery", "Resetting I2C master bus");
    ESP_ERROR_CHECK(i2c_master_bus_reset(i2c_bus_handle));
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t result = i2c_master_probe(i2c_bus_handle, 0x36, 100);
    ESP_LOGI("Battery", "Probing battery gauge device... [0x%X]", result);

    uint32_t task_counter = 0;
    while (1) {
        uint8_t cmd_buffer[16];
        uint8_t buffer[32];
        memset(cmd_buffer, 0, sizeof(cmd_buffer));
        memset(buffer, 0, sizeof(buffer));
        cmd_buffer[0] = 0x02;
        ESP_ERROR_CHECK(i2c_master_transmit_receive(battery_gauge_device_handle, cmd_buffer, 1, buffer, 2, 100));
        unsigned vcell = buffer[0] / 50;
        unsigned vcell_deci = (buffer[0] % 50) / 5;    
        cmd_buffer[0] = 0x04;
        ESP_ERROR_CHECK(i2c_master_transmit_receive(battery_gauge_device_handle, cmd_buffer, 1, buffer, 2, 100));
        unsigned soc = buffer[0];
        unsigned soc_deci = buffer[1] * 10 / 256;
        if (vcell != current_vcell
                || vcell_deci != current_vcell_deci
                || soc != current_soc
                || soc_deci != current_soc_deci) {
            demo_data_t battery_data;
            battery_data.update_type = BATTERY_UPDATE;
            battery_data.battery_update.voltage = current_vcell = vcell;
            battery_data.battery_update.voltage_deci = current_vcell_deci = vcell_deci;
            battery_data.battery_update.capacity = current_soc = soc;
            battery_data.battery_update.capacity_deci = current_soc_deci = soc_deci;
            xQueueSend(demo_data_queue, &battery_data, 2);
            ESP_LOGI(TAG, "[%ld] VCELL: %u.%uV  SOC: %u.%u%%", task_counter, vcell, vcell_deci, soc, soc_deci);
        }

        task_counter++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void demo_button_task(void* arg)
{
    uint8_t current_b0 = 0;
    uint8_t current_b1 = 0;
    uint8_t current_b2 = 0;

    ESP_LOGI("Button", "Starting button task");

    gpio_reset_pin(GPIO_NUM_0);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);

    gpio_reset_pin(GPIO_NUM_1);
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_1, GPIO_PULLDOWN_ENABLE);

    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLDOWN_ENABLE);

    uint32_t task_counter = 0;
    while (1) {
        uint8_t b0 = 1 - gpio_get_level(GPIO_NUM_0);
        uint8_t b1 = gpio_get_level(GPIO_NUM_1);
        uint8_t b2 = gpio_get_level(GPIO_NUM_2);
        if (b0 != current_b0 || b1 != current_b1 || b2 != current_b2) {
            demo_data_t button_data;
            button_data.update_type = BUTTON_UPDATE;
            button_data.button_update.b0 = (current_b0 = b0);
            button_data.button_update.b1 = (current_b1 = b1);
            button_data.button_update.b2 = (current_b2 = b2);
            xQueueSend(demo_data_queue, &button_data, 2);
            ESP_LOGI(TAG, "Button state: B0=%u B1=%u B2=%u", b0, b1, b2);
        }
        task_counter++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void demo_clock_task(void* arg)
{
    ESP_LOGI(TAG, "Starting time task");

    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;

    // Set timezone to America/Denver
    setenv("TZ", "MST7MDT", 1);
    tzset();

    while (1) {
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "The current date/time in Denver is: %s", strftime_buf);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {

    ESP_LOGI(TAG, "Starting main task");
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int x = 0; x < LV_HOR_RES_MAX; x++) {
        for(int y = 0; y < LV_VER_RES_MAX; y++) {
            display_buffer[(y * LV_HOR_RES_MAX + x) * 2] = 0x84;
            display_buffer[(y * LV_HOR_RES_MAX + x) * 2 + 1] = 0x10;
        }
    }
    for (int x = 0; x < 16; x++) {
        for(int y = 0; y < 16; y++) {
            test_square[(y * 16 + x) * 2] = 0x00;
            test_square[(y * 16 + x) * 2 + 1] = 0x1F;
        }
    }

    gpio_reset_pin(GPIO_NUM_7);
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);
    gpio_reset_pin(GPIO_NUM_21);
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_21, 1);

    ESP_LOGI(TAG, "Turn off LCD backlight");
    ESP_ERROR_CHECK(gpio_reset_pin(DEMO_PIN_NUM_BK_LIGHT));
    ESP_ERROR_CHECK(gpio_set_direction(DEMO_PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(DEMO_PIN_NUM_BK_LIGHT, DEMO_LCD_BK_LIGHT_OFF_LEVEL));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = DEMO_PIN_NUM_SCLK,
        .mosi_io_num = DEMO_PIN_NUM_MOSI,
        .miso_io_num = DEMO_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LV_HOR_RES_MAX * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = DEMO_PIN_NUM_LCD_DC,
        .cs_gpio_num = DEMO_PIN_NUM_LCD_CS,
        .pclk_hz = DEMO_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = DEMO_LCD_CMD_BITS,
        .lcd_param_bits = DEMO_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        // .on_color_trans_done = demo_notify_lvgl_flush_ready,
        // .user_ctx = disp,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ST7789 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = DEMO_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 40, 52));
    // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, 1));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(gpio_set_level(DEMO_PIN_NUM_BK_LIGHT, DEMO_LCD_BK_LIGHT_ON_LEVEL));

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LV_HOR_RES_MAX, 80, display_buffer));
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 80, LV_HOR_RES_MAX, 160, display_buffer));
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 160, LV_HOR_RES_MAX, 240, display_buffer));
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 16, 16, 32, 32, test_square));

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LV_HOR_RES_MAX * LV_VER_RES_MAX,
        .double_buffer = false,
        .hres = LV_HOR_RES_MAX,
        .vres = LV_VER_RES_MAX,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = false,
            .direct_mode = false,
            .swap_bytes = true,
        }
    };
    // const lvgl_port_display_rgb_cfg_t rgb_cfg = {
    //     .flags = {
    //         .bb_mode = true,
    //         .avoid_tearing = true,
    //     }
    // };
    lv_display_t* display_handle = lvgl_port_add_disp(&disp_cfg);
    demo_ui(display_handle);

    ESP_LOGI(TAG, "Creating FreeRTOS resources and tasks");
    demo_data_queue = xQueueCreate(10, sizeof(demo_data_t));
    assert(demo_data_queue);
    xTaskCreate(demo_display_task, "Display Update", 3072, NULL, 4, NULL);
    xTaskCreate(demo_battery_task, "Battery Monitor", 3072, NULL, 4, NULL);
    xTaskCreate(demo_button_task, "Button Monitor", 3072, NULL, 4, NULL);
    xTaskCreate(demo_clock_task, "Clock", 3072, NULL, 5, NULL);

    uint32_t ctr = 0;
    while (true) {

        ESP_LOGI(TAG, "Main loop running... [%lu]", ctr);
        ctr++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGI(TAG, "Main task finished");
}
