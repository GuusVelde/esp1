// Auteur : Guus van der Velde
// Student: 1035940

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
#include "config.h"

#define TXD_PIN (GPIO_NUM_5)  //ESP32 TX to UART RX
#define RXD_PIN (GPIO_NUM_4)  //ESP32 RX to UART TX
#define UART_NUM UART_NUM_1
#define TRIGGER_GPIO GPIO_NUM_0
#define FS80NK_GPIO GPIO_NUM_3
#define NTC_ADC ADC1_CHANNEL_2

#define KELVIN 273.15

#define BUF_SIZE (1024)

static const char *TAG = "LoRa_UART";

//prevents repeated sending
static bool last_state = 1;

static adc_oneshot_unit_handle_t adc_handle;



//global control variables
static int frequency = 10; //frequency of checking the sensors in ms
static bool check_up = 0;
static uint8_t active_sensors = 3;

const int min_freq = 10; //minimal required frequency

//initialize UART
void uart_init() {
    const uart_config_t uart_config = {
        .baud_rate = BAUD,
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

//send data to LoRa module
void uart_send(const char* data) {
    uart_write_bytes(UART_NUM, data, strlen(data));
}

//initialize trigger sensor
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

//initialize IR sensor
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

//initialize temperature sensor
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
        active_ptr += C_ACTIVE; //length of "active:"
        int len = strspn(active_ptr, "01"); //active is in binary format so only 1 and 0
        *active_out = (char *)malloc(len + 1);
        strncpy(*active_out, active_ptr, len);
        (*active_out)[len] = '\0';
    }

    if (freq_ptr) {
        freq_ptr += C_FREQ; //length of "freq:"
        int len = strspn(freq_ptr, "0123456789"); //freq is a number so only check for numbers
        *freq_out = (char *)malloc(len + 1);
        strncpy(*freq_out, freq_ptr, len);
        (*freq_out)[len] = '\0';
    }
}

//receive Data
void uart_receive_task(void *arg) {
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);

        if (len > 0) {
            if (len < BUF_SIZE) {
                data[len] = '\0';
            } else {
                data[BUF_SIZE - 1] = '\0'; //cuts of messages that are to long
            }

            ESP_LOGI(TAG, "Received: %s", data);
            char *active;
            char *freq;
            char *check;
            parse_data((const char *)data, &active, &freq, &check);

            char *endptr;

            if (freq != NULL) {
                int num_freq = strtol(freq, &endptr, 10); //10 for decimal 
                if (num_freq >= min_freq) { //eliminate small frequencies that could cause problems
                    frequency = num_freq;
                }
                
            }

            if (active != NULL) {
                active_sensors = 0x00;
                for (int i = 0; active[i] != '\0'; i++) { //bitshifts for length of string and places 1 where needed.
                    active_sensors <<= 1;
                    if (active[i] == '1') {
                        active_sensors |= 1;
                    }
                } 
            }

            free(freq);
            free(active);
        }
    }
}



//ir logic
void ir_sensor() {
    if (gpio_get_level(FS80NK_GPIO) == 0) {
        printf("Object detected\n");
        uart_send("Object detected\n");
    } else {
        printf("No object\n");
        uart_send("No object detected\n");
    }
}

//ntc logic
void ntc_sensor() {
    int value;
    adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &value);
    if (value > 0) {
        float volt = ((float)value / ADC_MAX) * NTC_VOLT; //numbers out of datasheet. Adjust based on sensor
        float ohm = (NTC_VOLT * NTC_OHM / volt) - NTC_OHM;

        float temp = 1.0 / (1.0/(KELVIN+AVG_TMP) + (1.0/NTC)*log(ohm/NTC_OHM)) - KELVIN + NTC_OFFSET; //+6 to fix offset (trial and error) +25 for average temperature. Change based on location
        printf("Temperature: %f\n", temp);
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Temperature: %f\n", temp);
        uart_send(buffer);
    } else {
        uart_send("Temperature: ERROR\n");
    }
}

//temporary sensor logic
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

//send messages and control sensors
void message_task(void *arg) {
    void (*func_array[])() = {ntc_sensor, ir_sensor, sensor_trigger_2, sensor_trigger_3};
    int func_count = sizeof(func_array) / sizeof(func_array[0]);

    while (1) {
        bool current_state = gpio_get_level(FS80NK_GPIO);

        if((current_state != last_state && current_state == 0) || check_up == 1) {
            for (int i = 0; i < func_count; i++) {
                if (active_sensors & (1<<i)) {
                    func_array[i]();
                }
            }
            check_up = 0;
        }

        uart_wait_tx_done(UART_NUM, portMAX_DELAY);
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(frequency));
    }
}

void app_main(void) {
    //initialize everything
    uart_init();
    init_trigger_pin();
    fs80nk_init();
    ntc_init();

    //receive task gets higher priority to prevent the module from missing messages
    xTaskCreate(message_task, "message_task", 2048, NULL, 4, NULL);
    xTaskCreate(uart_receive_task, "uart_receive_task", 4096, NULL, 5, NULL);

    //send message over network to let the other nodes know that this node exists and where it exists
    uart_send("Klaar:");
}

