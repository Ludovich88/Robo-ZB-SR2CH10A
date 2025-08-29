#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
#include <driver/gpio.h"
#include <driver/uart.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_zigbee_core.h>
#include <esp_zigbee_zcl_command.h>
#include <esp_zigbee_zcl_common.h>
#include <esp_zigbee_zcl_identify.h>
#include <esp_zigbee_zcl_on_off.h>
#include <esp_zigbee_zcl_electrical_measurement.h>
#include <esp_zigbee_zcl_metering.h>

static const char *TAG = "SMART_RELAY";

// GPIO Definitions
#define BUTTON_PAIRING_GPIO     GPIO_NUM_0    // IO0 - кнопка для запуска пейринга
#define LED_STATUS_GPIO         GPIO_NUM_1    // IO1 - индикатор состояния связи
#define ZERO_CROSS_GPIO         GPIO_NUM_2    // IO2 - сигнал zero cross
#define UART_RX_GPIO            GPIO_NUM_4    // IO4 - RX от BL0940
#define UART_TX_GPIO            GPIO_NUM_5    // IO5 - TX к BL0940
#define SWITCH_1_GPIO           GPIO_NUM_14   // IO14 - переключатель реле 1
#define SWITCH_2_GPIO           GPIO_NUM_15   // IO15 - переключатель реле 2
#define RELAY_2_GPIO            GPIO_NUM_18   // IO18 - реле номер 2
#define RELAY_1_GPIO            GPIO_NUM_19   // IO19 - реле номер 1

// UART Configuration
#define UART_NUM                UART_NUM_1
#define UART_BAUD_RATE          4800
#define UART_BUF_SIZE           1024

// Zigbee Configuration
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
#define ESP_ZB_ZED_CONFIG() { \
    .esp_zb_role = ESP_ZB_ED_ROLE, \
    .install_code_policy = ESP_ZB_COMMISSIONING_POLICY_INSTALL_CODE, \
    .nwk_cfg.zed_cfg = { \
        .ed_timeout = ED_AGING_TIMEOUT_64MIN, \
        .keep_alive = 3000, \
    }, \
}

// Global variables
static bool relay_1_state = false;
static bool relay_2_state = false;
static bool pairing_mode = false;
static uint32_t energy_consumption = 0;
static uint32_t power_consumption = 0;
static uint32_t voltage_rms = 0;
static uint32_t current_rms = 0;

// Queues and timers
static QueueHandle_t gpio_evt_queue;
static TimerHandle_t status_led_timer;
static TimerHandle_t energy_read_timer;

// Zigbee device endpoint
static esp_zb_ep_list_t *esp_zb_ep_list = NULL;

// Function prototypes
static void gpio_init(void);
static void uart_init(void);
static void zigbee_init(void);
static void gpio_isr_handler(void *arg);
static void status_led_task(void *pvParameter);
static void energy_monitoring_task(void *pvParameter);
static void button_task(void *pvParameter);
static void zero_cross_isr_handler(void *arg);
static void uart_rx_task(void *pvParameter);
static void send_zigbee_update(void);
static void toggle_relay(uint8_t relay_num, bool state);

// GPIO Interrupt Service Routine
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Zero Cross Interrupt Service Routine
static void IRAM_ATTR zero_cross_isr_handler(void *arg)
{
    // Zero cross detected - can be used for phase control
    // This is called when AC voltage crosses zero
}

// Initialize GPIO pins
static void gpio_init(void)
{
    // Configure button input with pull-up
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_PAIRING_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    // Configure status LED output
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_STATUS_GPIO);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Configure zero cross input
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ZERO_CROSS_GPIO);
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf);

    // Configure switch inputs with pull-up
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << SWITCH_1_GPIO) | (1ULL << SWITCH_2_GPIO);
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);

    // Configure relay outputs
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RELAY_1_GPIO) | (1ULL << RELAY_2_GPIO);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Set initial relay states (off)
    gpio_set_level(RELAY_1_GPIO, 0);
    gpio_set_level(RELAY_2_GPIO, 0);

    // Install GPIO ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PAIRING_GPIO, gpio_isr_handler, (void*) BUTTON_PAIRING_GPIO);
    gpio_isr_handler_add(ZERO_CROSS_GPIO, zero_cross_isr_handler, (void*) ZERO_CROSS_GPIO);
    gpio_isr_handler_add(SWITCH_1_GPIO, gpio_isr_handler, (void*) SWITCH_1_GPIO);
    gpio_isr_handler_add(SWITCH_2_GPIO, gpio_isr_handler, (void*) SWITCH_2_GPIO);

    ESP_LOGI(TAG, "GPIO initialized");
}

// Initialize UART for BL0940 communication
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_NONE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "UART initialized for BL0940");
}

// Initialize Zigbee
static void zigbee_init(void)
{
    esp_zb_cfg_t zb_cfg = ESP_ZB_ZED_CONFIG();
    ESP_ERROR_CHECK(esp_zb_init(&zb_cfg));

    // Create endpoint list
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_ep_t *esp_zb_ep = esp_zb_ep_create(HA_ONOFF_SWITCH_ENDPOINT, HA_ONOFF_SWITCH_PROFID, HA_ONOFF_SWITCH_DEVICEID, 0, NULL, NULL);
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_ep);

    // Register endpoint list
    esp_zb_ep_list_register(esp_zb_ep_list);

    ESP_LOGI(TAG, "Zigbee initialized");
}

// Toggle relay function
static void toggle_relay(uint8_t relay_num, bool state)
{
    if (relay_num == 1) {
        relay_1_state = state;
        gpio_set_level(RELAY_1_GPIO, state ? 1 : 0);
        ESP_LOGI(TAG, "Relay 1: %s", state ? "ON" : "OFF");
    } else if (relay_num == 2) {
        relay_2_state = state;
        gpio_set_level(RELAY_2_GPIO, state ? 1 : 0);
        ESP_LOGI(TAG, "Relay 2: %s", state ? "ON" : "OFF");
    }
    
    // Update Zigbee state
    send_zigbee_update();
}

// Send Zigbee update
static void send_zigbee_update(void)
{
    // Update on/off cluster
    esp_zb_zcl_on_off_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr.short_addr = 0x0000;
    cmd_req.zcl_basic_cmd.dst_addr_u.addr.endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    cmd_req.zcl_basic_cmd.cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
    cmd_req.zcl_basic_cmd.cmd_flags = ESP_ZB_ZCL_CMD_FLAG_DEFAULT;
    
    esp_zb_zcl_on_off_cmd_send(&cmd_req);
}

// Status LED task
static void status_led_task(void *pvParameter)
{
    bool led_state = false;
    
    while (1) {
        if (pairing_mode) {
            // Fast blink during pairing
            gpio_set_level(LED_STATUS_GPIO, led_state);
            led_state = !led_state;
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (esp_zb_get_network_joined_status()) {
            // Slow blink when connected
            gpio_set_level(LED_STATUS_GPIO, led_state);
            led_state = !led_state;
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // Off when disconnected
            gpio_set_level(LED_STATUS_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Energy monitoring task
static void energy_monitoring_task(void *pvParameter)
{
    uint8_t data[UART_BUF_SIZE];
    uint8_t len;
    
    while (1) {
        len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Parse BL0940 data
            // BL0940 sends data in specific format
            if (len >= 8 && data[0] == 0x58) { // Start byte
                // Parse voltage, current, power, energy
                voltage_rms = (data[1] << 8) | data[2];
                current_rms = (data[3] << 8) | data[4];
                power_consumption = (data[5] << 8) | data[6];
                energy_consumption += power_consumption / 3600; // Convert to Wh
                
                ESP_LOGI(TAG, "V: %d mV, I: %d mA, P: %d mW, E: %d mWh", 
                         voltage_rms, current_rms, power_consumption, energy_consumption);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Button task
static void button_task(void *pvParameter)
{
    uint32_t gpio_num;
    
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            if (gpio_num == BUTTON_PAIRING_GPIO) {
                // Button pressed - enter pairing mode
                pairing_mode = !pairing_mode;
                if (pairing_mode) {
                    ESP_LOGI(TAG, "Entering pairing mode");
                    esp_zb_start_commissioning();
                } else {
                    ESP_LOGI(TAG, "Exiting pairing mode");
                }
            } else if (gpio_num == SWITCH_1_GPIO) {
                // Switch 1 toggled
                bool switch_state = gpio_get_level(SWITCH_1_GPIO);
                toggle_relay(1, switch_state);
            } else if (gpio_num == SWITCH_2_GPIO) {
                // Switch 2 toggled
                bool switch_state = gpio_get_level(SWITCH_2_GPIO);
                toggle_relay(2, switch_state);
            }
        }
    }
}

// Main application entry point
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Smart Relay Application");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create GPIO event queue
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // Initialize hardware
    gpio_init();
    uart_init();
    zigbee_init();
    
    // Create tasks
    xTaskCreate(status_led_task, "status_led", 2048, NULL, 5, NULL);
    xTaskCreate(energy_monitoring_task, "energy_mon", 4096, NULL, 4, NULL);
    xTaskCreate(button_task, "button", 2048, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "Smart Relay Application started successfully");
}
