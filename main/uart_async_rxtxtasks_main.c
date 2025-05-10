/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "base64.h"


#include <time.h>

static const int RX_BUF_SIZE = 4096;

#define TXD_PIN (GPIO_NUM_18)
#define RXD_PIN (GPIO_NUM_17)

#define NO_OF_ITERS 3
#define RX_TASK_PRIO 9
#define TX_GPIO_NUM 1
#define RX_GPIO_NUM 21
#define EXAMPLE_TAG "TWAI Listen Only"
static const char *TX_TASK_TAG = "TX_TASK";

#define TX_BUFF_SIZE 256

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_general_config_t g_config = {.mode = TWAI_MODE_LISTEN_ONLY,
                                               .tx_io = TX_GPIO_NUM,
                                               .rx_io = RX_GPIO_NUM,
                                               .clkout_io = TWAI_IO_UNUSED,
                                               .bus_off_io = TWAI_IO_UNUSED,
                                               .tx_queue_len = 0,
                                               .rx_queue_len = 5,
                                               .alerts_enabled = TWAI_ALERT_NONE,
                                               .clkout_divider = 0};

static SemaphoreHandle_t rx_sem;

void init(void)
{
  const uart_config_t uart_config = {
      .baud_rate = 460800,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  // We won't use a buffer for sending data.
  uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char *logName, const char *data)
{
  const int len = strlen(data);
  const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
  ESP_LOGI(logName, "Wrote %d bytes", txBytes);
  return txBytes;
}

// Base64 encoding function
char *base64_encode(const unsigned char *data, size_t input_length, size_t *output_length) {
  *output_length = 4 * ((input_length + 2) / 3); // Calculate output length
  char *encoded_data = malloc(*output_length + 1); // Allocate memory for encoded data
  if (encoded_data == NULL) {
      return NULL; // Memory allocation failed
  }

  const char *base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  int i, j;
  for (i = 0, j = 0; i < input_length;) {
      unsigned int octet_a = i < input_length ? data[i++] : 0;
      unsigned int octet_b = i < input_length ? data[i++] : 0;
      unsigned int octet_c = i < input_length ? data[i++] : 0;

      unsigned int triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

      encoded_data[j++] = base64_chars[(triple >> 3 * 6) & 0x3F];
      encoded_data[j++] = base64_chars[(triple >> 2 * 6) & 0x3F];
      if (i - 2 < input_length) {
          encoded_data[j++] = base64_chars[(triple >> 1 * 6) & 0x3F];
      } else {
          encoded_data[j++] = '=';
      }
      if (i - 1 < input_length) {
          encoded_data[j++] = base64_chars[(triple >> 0 * 6) & 0x3F];
      } else {
          encoded_data[j++] = '=';
      }
  }

  encoded_data[*output_length] = '\0'; // Null-terminate the string
  return encoded_data;
}

// Function to convert unsigned long long to byte array
void ull_to_byte_array(unsigned long long num, unsigned char *buffer, size_t buffer_size) {
  for (size_t i = 0; i < buffer_size; i++) {
      buffer[i] = (num >> (8 * (buffer_size - 1 - i))) & 0xFF;
  }
}

static void rx_task(void *arg)
{
  static const char *RX_TASK_TAG = "RX_TASK";
  esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
  uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
  while (1)
  {
    const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
    if (rxBytes > 0)
    {
      data[rxBytes] = 0;
      ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
      ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
      char err_str[] = "ERROR";
      if (strstr((char *)data, err_str) != NULL)
      {
        printf("Substring found!\n");
      }
      else
      {
        printf("Substring not found.\n");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  free(data);
}

static void tx_task(void *arg)
{
  xSemaphoreTake(rx_sem, portMAX_DELAY);

  esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

  sendData(TX_TASK_TAG, "AT\r\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT\r\n");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+IPREX?\r\n");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+IPREX=460800\r\n");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+IPREX?\r\n");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT\r\n");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+CMQTTSTART\r\n");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+CMQTTACCQ=0,\"esp32-s3-sim7674g\",0\r\n");
  vTaskDelay(500 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+CMQTTCONNECT=0,\"tcp://mqtt.maua.br\",20,1\r\n");
  vTaskDelay(500 / portTICK_PERIOD_MS);

  while (1)
  {
    twai_message_t rx_msg;
    twai_receive(&rx_msg, portMAX_DELAY);
    unsigned long long data = 0;
    unsigned char ch[1];

    for (int i = 0; i < rx_msg.data_length_code; i++)
    {
      // data = data << 8 | rx_msg.data[i];
      memcpy(ch + i, (char*)&rx_msg.data[i], sizeof(ch));
    }
    ESP_LOGI(EXAMPLE_TAG, "Id: %lu, Msg: %s", rx_msg.identifier, ch);
    unsigned char buffer2[sizeof(ch)];
    size_t output_length2;
    printf("output_length2: %zu\n", output_length2);
    char *encoded2 = base64_encode(ch, sizeof(data), &output_length2);
    printf("Encoded2: %s\n", encoded2);
    if (encoded2 != NULL) {
     int count = 1;
      uint8_t at_topic[256];
      uint8_t at_message[256];

      uint8_t topic[256];
      uint8_t message[256];

      char organization[] = "FSAELive";
      char device_type[] = "Car";
      char measurement[] = "Can";
      char device_id[] = "mauaracingsss";
      char direction[] = "up";
      char etc[] = "4g";

      // topic: OpenDataTelemetry/ORGANIZATION/DEVICE_TYPE/MEASUREMENT/DEVICE_ID/DIRECTION/ETC
      int j = snprintf((char *)(topic), 17 + 1 + sizeof(organization) + 1 + sizeof(device_type) + 1 + sizeof(measurement) + 1 + sizeof(device_id) + 1 + sizeof(direction) + 1 + sizeof(etc), "OpenDataTelemetry/%s/%s/%s/%s/%s/%s", organization, device_type, measurement, device_id, direction, etc);
      j = snprintf((char *)(at_topic), 16 + j, "AT+CMQTTTOPIC=0,%d\r\n", j);

      // data is a lon long unsigned int. Need to convert into char
      // j = snprintf((char *)(message), sizeof(count) + 8, "%d", count);
      // j = snprintf((char *)(at_message), 8 + j + 2, "AT+CMQTTPAYLOAD=0,%d\r\n", j);

      printf("sizeof(organization): %zu\n", sizeof(organization));
      printf("sizeof(count): %zu\n", sizeof(count));
      printf("sizeof(encoded2): %zu\n", strlen(encoded2));
      // j = snprintf((char *)(message), sizeof(organization) + sizeof(organization), "%s, %s", encoded2, encoded2);
      j = snprintf((char *)(message), strlen(encoded2), "%s", encoded2);
      j = snprintf((char *)(at_message), 18 + j, "AT+CMQTTPAYLOAD=0,%d\r\n", j);

      ESP_LOGI(EXAMPLE_TAG, "Id: %lu, Msg: %s", rx_msg.identifier, ch);

      // printf("topic: %s\n", (topic));
      // printf("at_topic: %s\n", (at_topic));
      // printf("message: %s\n", (message));
      // printf("at_message: %s\n", (at_message));

      printf("topic: %s\n", (char *)(topic));
      printf("at_topic: %s\n", (char *)(at_topic));
      printf("message: %s\n", (char *)(message));
      printf("at_message: %s\n", (char *)(at_message));

      sendData(TX_TASK_TAG, (char *) at_topic);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      sendData(TX_TASK_TAG, (char *) topic);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      sendData(TX_TASK_TAG, (char *) at_message);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      sendData(TX_TASK_TAG, (char *) message);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      sendData(TX_TASK_TAG, "AT+CMQTTPUB=0,0,60\r\n");
      vTaskDelay(10 / portTICK_PERIOD_MS);

      memset(&topic, 0, sizeof(topic));
      memset(&at_topic, 0, sizeof(at_topic));
      memset(&message, 0, sizeof(message));
      memset(&at_message, 0, sizeof(at_message));

      vTaskDelay(100);
      free(encoded2);
    } else {
        printf("Memory allocation failed.\n");
    }
  }

  xSemaphoreGive(rx_sem);
  vTaskDelete(NULL);
}

void app_main(void)
{
  rx_sem = xSemaphoreCreateBinary();
  // xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);

  init();
  xTaskCreate(rx_task, "uart_rx_task", 2048 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
  xTaskCreate(tx_task, "uart_tx_task", 2048 * 2, NULL, configMAX_PRIORITIES - 2, NULL);

  // Install and start TWAI driver
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_LOGI(EXAMPLE_TAG, "Driver installed");
  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI(EXAMPLE_TAG, "Driver started");

  xSemaphoreGive(rx_sem); // Start RX task
  vTaskDelay(pdMS_TO_TICKS(100));
  xSemaphoreTake(rx_sem, portMAX_DELAY); // Wait for RX task to complete

  // Stop and uninstall TWAI driver
  ESP_ERROR_CHECK(twai_stop());
  ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
  ESP_ERROR_CHECK(twai_driver_uninstall());
  ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

  // Cleanup
  vSemaphoreDelete(rx_sem);
}
