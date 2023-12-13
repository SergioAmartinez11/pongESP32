/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"

#define BOARD_LEN 8

static const char *TAG = "MQTT_EXAMPLE";
char *sesionTopic = "v1/playersOnline";
char *gameTopic = "v1/gaming";
int gameBoard[BOARD_LEN][BOARD_LEN];
int raquetaPosY = 1;
int raquetaPosX = 0;
int puntos = 0;
bool gameOnFlag = false;
char *playerName = "playerA";
int playersOnline = 0;

TaskHandle_t vTaskGameboard_handler = NULL;

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
QueueHandle_t uart0_queue;
esp_mqtt_client_handle_t client;

void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(BUF_SIZE);
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            bzero(dtmp, BUF_SIZE);
            // Check for UART data.
            if (event.type == UART_DATA)
            {
                uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
                // Process the received data here.
                ESP_LOGI(TAG, "Read from UART: %s\n", dtmp);
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void configure_uart()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    // Install UART driver.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    // Set UART pins (using default pins for UART0).
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Create a queue to handle UART events.
    uart0_queue = xQueueCreate(10, sizeof(uart_event_t));

    // Enable RX interrupt.
    ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM));

    // Create a task to handle UART events.
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char messageBuffer[100];

    // Guardar el DATA en una variable
    char data_buffer[event->data_len + 1];   // +1 para el carácter nulo '\0'
    char topic_buffer[event->topic_len + 1]; // +1 para el carácter nulo '\0'

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        // subscribe to login topic
        msg_id = esp_mqtt_client_subscribe(client, sesionTopic, 0);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        //  subscribe to gameTopic
        msg_id = esp_mqtt_client_subscribe(client, gameTopic, 0);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        //  login to game
        snprintf(messageBuffer, sizeof(messageBuffer), "%s,%d %d,%d", playerName, raquetaPosX, raquetaPosY, puntos);
        msg_id = esp_mqtt_client_publish(client, sesionTopic, messageBuffer, 0, 0, 0);

        break;
    case MQTT_EVENT_DATA:
        // Guardar el TOPIC en una variable
        snprintf(topic_buffer, sizeof(topic_buffer), "%.*s", event->topic_len, event->topic);
        //printf("TOPIC=%s\r\n", topic_buffer);

        snprintf(data_buffer, sizeof(data_buffer), "%.*s", event->data_len, event->data);
       // printf("DATA=%s\r\n", data_buffer);

        if (strcmp(topic_buffer, gameTopic) == 0)
        {
            ESP_LOGI(TAG, "game topic: %s", data_buffer);
        }

        if (strcmp(topic_buffer, sesionTopic) == 0)
        {
            playersOnline++;
            ESP_LOGI(TAG, "Players online: %d", playersOnline);

            if (playersOnline > 2)
            {
                gameOnFlag = true;
            }
        }

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void initializeGameBoard()
{
    int i = 0;
    int j = 0;
    for (int i = 0; i < BOARD_LEN; i++)
    {
        for (int j = 0; j < BOARD_LEN; j++)
        {
            gameBoard[i][j] = 0;
        }
    }

    // POSICION DE RAQUETA POR DEFAULT EN LA ESQUINA INFERIOR IZQUIERDA
    gameBoard[BOARD_LEN][0] = 1;
    gameBoard[BOARD_LEN - 1][0] = 1;
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://test.mosquitto.org:1883",
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void vTaskGameboard()
{
    int msg_id;
    int messageBuffer[100];
    while (1)
    {
        if (gameOnFlag)
        {

            snprintf(messageBuffer, sizeof(messageBuffer), "%s,%d %d,%d", playerName, raquetaPosX, raquetaPosY, puntos);
            msg_id = esp_mqtt_client_publish(client, gameTopic, messageBuffer,0,1,0);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */

    // Configure UART.
    configure_uart();

    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
    xTaskCreatePinnedToCore(vTaskGameboard, "vTaskGameboard_task", 4096, NULL, 10, &vTaskGameboard_handler, 1);
}
