#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "esp_adc/adc_oneshot.h"

#define TXD_PIN (GPIO_NUM_5)  // ESP32 TX to UART RX
#define RXD_PIN (GPIO_NUM_4)  // ESP32 RX to UART TX
#define UART_NUM UART_NUM_1
#define TRIGGER_GPIO GPIO_NUM_0
#define FS80NK_GPIO GPIO_NUM_3
#define NTC_ADC ADC1_CHANNEL_2

#define NTC_OHM 10000.0
#define KELVIN 273.15

#define BUF_SIZE (1024)

static const char *TAG = "LoRa_UART";

static int status_flag = 0;
static bool last_state = 1;

static adc_oneshot_unit_handle_t adc_handle;

static int frequency = 10000;
static bool check_up = 0;
static uint8_t active_sensors = 3;

// Initialize UART
void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Send Data
void uart_send(const char* data) {
    uart_write_bytes(UART_NUM, data, strlen(data));
}

void parse_data(const char *input, char **active_out, char **freq_out, char **check) {
    const char *active_ptr = strstr(input, "active:");
    const char *freq_ptr = strstr(input, "freq:");
    const char *check_ptr = strstr(input, "check");

    *active_out = NULL;
    *freq_out = NULL;

    if (strstr(input, "check")) {
        check_up = 1;
    }

    if (active_ptr) {
        active_ptr += 7; //7 is length of "active:"
        int len = strspn(active_ptr, "01");
        *active_out = (char *)malloc(len + 1);
        strncpy(*active_out, active_ptr, len);
        (*active_out)[len] = '\0';
    }

    if (freq_ptr) {
        freq_ptr += 5; //5 is length of "freq:"
        int len = strspn(freq_ptr, "0123456789");
        *freq_out = (char *)malloc(len + 1);
        strncpy(*freq_out, freq_ptr, len);
        (*freq_out)[len] = '\0';
    }

    if (check) {

    }

}

// Receive Data
void uart_receive_task(void *arg) {
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);

        if (len > 0) {
            if (len < BUF_SIZE) {
                data[len] = '\0';
            } else {
                data[BUF_SIZE - 1] = '\0';
            }

            ESP_LOGI(TAG, "Received: %s", data);
            char *active;
            char *freq;
            char *check;
            parse_data((const char *)data, &active, &freq, &check);

            char *endptr;

            if (freq != NULL) {
                int num_freq = strtol(freq, &endptr, 10); // 10 for decimal 
                if (num_freq >= 1000) {
                    frequency = num_freq;
                }
                
            }

            if (active != NULL) {
                active_sensors = 0x00;
                for (int i = 0; active[i] != '\0'; i++) {
                    active_sensors <<= 1;
                    if (active[i] == '1') {
                        active_sensors |= 1;
                    }
                } 
            }

            free(freq);
            free(active);
            printf("\n");
        }
    }
}

void init_trigger_pin() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void fs80nk_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FS80NK_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void ntc_init() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_2, &chan_config);
}

void ir_sensor() {
    if (gpio_get_level(FS80NK_GPIO) == 0) {
        printf("Object detected\n");
        uart_send("Object detected\n");
    } else {
        printf("No object\n");
        uart_send("No object detected\n");
    }
}

void NTC_sensor() {
    int value;
    adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &value);
    float volt = ((float)value / 4095.0) * 3.3;
    if (volt <= 0.0) printf("NTC ERROR");
    float ohm = (3.3 * NTC_OHM / volt) - NTC_OHM;

    const float NTC = 3950;     // adjust to your thermistor
    float temp = 1.0 / (1.0/(KELVIN+25) + (1.0/NTC)*log(ohm/NTC_OHM)) - KELVIN + 6; //+6 to fix offset
    printf("Temperature: %f\n", temp);
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Temperature: %f\n", temp);
    uart_send(buffer);
}

void sensor_trigger_0() {
    bool current_state = gpio_get_level(TRIGGER_GPIO);
    if (current_state == 0) {
        uart_send("Sensor 0 AAN\n");
        printf("0 AAN\n");
    } else if (current_state == 1) {
        uart_send("Sensor 0 UIT\n");
        printf("0 UIT\n");
    }
}

void sensor_trigger_1() {
    bool current_state = gpio_get_level(TRIGGER_GPIO);
    if (current_state == 0) {
        uart_send("Sensor 1 AAN\n");
        printf("1 AAN\n");
    } else if (current_state == 1) {
        uart_send("Sensor 1 UIT\n");
        printf("1 UIT\n");
    }
}

void sensor_trigger_2() {
    bool current_state = gpio_get_level(TRIGGER_GPIO);
    if (current_state == 0) {
        uart_send("Sensor 2 AAN\n");
        printf("2 AAN\n");
    } else if (current_state == 1) {
        uart_send("Sensor 2 UIT\n");
        printf("2 UIT\n");
    }
}

void sensor_trigger_3() {
    bool current_state = gpio_get_level(TRIGGER_GPIO);
    if (current_state == 0) {
        uart_send("Sensor 3 AAN\n");
        printf("3 AAN\n");
    } else if (current_state == 1) {
        uart_send("Sensor 3 UIT\n");
        printf("3 UIT\n");
    }
}

//Activate for testing purposes

// void message_task(void *arg) {
//     int counter = 0;
//     while (1) {
//         bool current_state = gpio_get_level(TRIGGER_GPIO);
//         printf("Current: %d\n", current_state);
//             if (current_state != last_state) {
//                 switch (counter) {
//                     case 0:
//                         printf("case 0\n");
//                         uart_send("active:0001 freq:5000");
//                         counter++;
//                         break;
//                     case 1:
//                         printf("case 1\n");
//                         uart_send("active:0101");
//                         counter++;
//                         break;
//                     case 2:
//                         printf("case 2\n");
//                         uart_send("freq:10000");
//                         counter++;
//                         break;
//                     case 3:
//                         printf("case 3\n");
//                         uart_send("freq:5000 active:1111");
//                         counter = 0;
//                         break;
//                     default:
//                         counter = 0;
//                         break;
//                 }
//             }

//         last_state = current_state;

//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

void message_task(void *arg) {
    void (*func_array[])() = {NTC_sensor, ir_sensor, sensor_trigger_2, sensor_trigger_3};
    int func_count = sizeof(func_array) / sizeof(func_array[0]);

    while (1) {
        bool current_state = gpio_get_level(FS80NK_GPIO);

        if((current_state != last_state && current_state == 0) || check_up == 1) {
            for (int i = 0; i < func_count; i++) {
                if (active_sensors & (1<<i)) {
                    printf("In active if\n");
                    func_array[i]();
                }
            }
            check_up = 0;
        }

        uart_wait_tx_done(UART_NUM, portMAX_DELAY);
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    uart_init();
    init_trigger_pin();
    fs80nk_init();
    ntc_init();
    xTaskCreate(message_task, "message_task", 2048, NULL, 4, NULL);
    xTaskCreate(uart_receive_task, "uart_receive_task", 4096, NULL, 5, NULL);

    uart_send("Klaar:");
}