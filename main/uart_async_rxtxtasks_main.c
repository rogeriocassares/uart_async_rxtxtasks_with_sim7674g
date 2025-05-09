/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 4096;

#define TXD_PIN (GPIO_NUM_18)
#define RXD_PIN (GPIO_NUM_17)

#define TX_BUFF_SIZE 256

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

static void tx_task(void *arg)
{
  static const char *TX_TASK_TAG = "TX_TASK";
  esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

  sendData(TX_TASK_TAG, "AT\r\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT\r\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+IPREX?\r\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+IPREX=460800\r\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+IPREX?\r\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT\r\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+CMQTTSTART\r\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+CMQTTACCQ=0,\"Waveshare-Sim7029\",0\r\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  sendData(TX_TASK_TAG, "AT+CMQTTCONNECT=0,\"tcp://mqtt.maua.br\",20,1\r\n");
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // int data = 86;
  // char data_str;
  // char data_str[] = "waveshare";
  // // sprintf(data_str, "%d", data);
  // char end_str[] = "\r\n";

  // char data_buff[256];
  // Copy src to the end of dest

  while (1)
  {

    for (int count = 1; count <= 50; ++count)
    {
      uint8_t at_topic[256];
      uint8_t at_message[256];

      uint8_t topic[256];
      uint8_t message[256];

      char prganization[] = "FSAELive";
      char device_type[] = "Car";
      char measurement[] = "Can";
      char device_id[] = "mauaracing";
      char direction[] = "up";
      char etc[] = "4g";

      // topic: OpenDataTelemetry/ORGANIZATION/DEVICE_TYPE/MEASUREMENT/DEVICE_ID/DIRECTION/ETC
      int j = snprintf((char *)(topic), 17 + 1 + sizeof(prganization) + 1 + sizeof(device_type) + 1 + sizeof(measurement) + 1 + sizeof(device_id) + 1 + sizeof(direction) + 1 + sizeof(etc), "OpenDataTelemetry/%s/%s/%s/%s/%s/%s", prganization, device_type, measurement, device_id, direction, etc);
      j = snprintf((char *)(at_topic), 16 + j, "AT+CMQTTTOPIC=0,%d\r\n", j);

      j = snprintf((char *)(message), sizeof(prganization) + sizeof(prganization), "%d, %d", count, count);
      j = snprintf((char *)(at_message), 18 + j, "AT+CMQTTPAYLOAD=0,%d\r\n", j);

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
      sendData(TX_TASK_TAG, (char *) message);;
      vTaskDelay(10 / portTICK_PERIOD_MS);
      sendData(TX_TASK_TAG, "AT+CMQTTPUB=0,0,60\r\n");
      vTaskDelay(10 / portTICK_PERIOD_MS);

      memset(&topic, 0, sizeof(topic));
      memset(&at_topic, 0, sizeof(at_topic));
      memset(&message, 0, sizeof(message));
      memset(&at_message, 0, sizeof(at_message));
    }
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
      if (strstr((char *)data, err_str) != NULL) {
        printf("Substring found!\n");

      } else {
          printf("Substring not found.\n");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  free(data);
}

void app_main(void)
{
  init();
  xTaskCreate(rx_task, "uart_rx_task", 2048 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
  xTaskCreate(tx_task, "uart_tx_task", 2048 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}
