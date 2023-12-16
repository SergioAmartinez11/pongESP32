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
#include "driver/adc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "myUart.h"

#define BOARD_LEN 8
#define NAME_INDEX 0
#define LOGIN_STATE_INDEX 1
#define RAQUETA_POS_INDEX 2
#define POINTS_INDEX 3
#define PELOTA_POS_X 4
#define PELOTA_POS_Y 5
#define GAME_STATUS_INDEX 6
#define GAME_ON_FLAG 1
#define GAME_OFF_FLAG 0

typedef struct
{
    char name[50];
    char state[10];
    int posRaqueta;
    int puntos;
    int pelotaX;
    int pelotaY;
} player_data_t;

static const char *TAG = "MQTT_EXAMPLE";
char *sesionTopic = "v1/playersOnline";
char *gameTopic = "v1/gaming";
int gameBoard[BOARD_LEN][BOARD_LEN];
bool gameOnFlag = false;
int playersOnline = 0;

player_data_t me = {"playerA", "ON", 1, 0, 0, 0};
player_data_t rival;

TaskHandle_t vTaskGameboard_handler = NULL;

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
QueueHandle_t uart0_queue;
esp_mqtt_client_handle_t client;

#define ADC_CHANNEL ADC1_CHANNEL_0


// void uart_event_task(void *pvParameters)
// {
//     uart_event_t event;
//     size_t buffered_size;
//     uint8_t *dtmp = (uint8_t *)malloc(BUF_SIZE);
//     for (;;)
//     {
//         // Waiting for UART event.
//         if (xQueueReceive(uart0_queue, (void *)&event, (TickType_t)portMAX_DELAY))
//         {
//             bzero(dtmp, BUF_SIZE);
//             // Check for UART data.
//             if (event.type == UART_DATA)
//             {
//                 uart_read_bytes(UART_NUM, dtmp, event.size, portMAX_DELAY);
//                 // Process the received data here.
//                 ESP_LOGI(TAG, "Read from UART: %s\n", dtmp);
//             }
//         }
//     }
//     free(dtmp);
//     dtmp = NULL;
//     vTaskDelete(NULL);
// }

// void configure_uart()
// {
//     uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

//     // Install UART driver.
//     ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
//     ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

//     // Set UART pins (using default pins for UART0).
//     ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

//     // Create a queue to handle UART events.
//     uart0_queue = xQueueCreate(10, sizeof(uart_event_t));

//     // Enable RX interrupt.
//     ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM));

//     // Create a task to handle UART events.
//     xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
// }

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

char **splitString(char cadena[])
{
    // Inicializar un arreglo dinámico para almacenar las palabras
    char **palabras = NULL;
    int numPalabras = 0;

    // Utilizar strtok para dividir la cadena por comas
    char *token = strtok(cadena, ",");

    // Mientras haya tokens
    while (token != NULL)
    {
        // Incrementar el número de palabras
        numPalabras++;

        // Reasignar el tamaño del arreglo dinámico
        palabras = realloc(palabras, sizeof(char *) * numPalabras);

        // Asignar memoria para la nueva palabra y copiar el token
        palabras[numPalabras - 1] = malloc(strlen(token) + 1);
        strcpy(palabras[numPalabras - 1], token);

        // Obtener el siguiente token
        token = strtok(NULL, ",");
    }

    // Agregar un elemento nulo al final del arreglo para indicar el final
    palabras = realloc(palabras, sizeof(char *) * (numPalabras + 1));
    palabras[numPalabras] = NULL;

    return palabras;
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
        snprintf(messageBuffer, sizeof(messageBuffer), "%s,%s,%d,%d,%d,%d, %d", me.name, me.state, me.posRaqueta, me.puntos, me.pelotaX, me.pelotaY, GAME_OFF_FLAG);
        msg_id = esp_mqtt_client_publish(client, sesionTopic, messageBuffer, 0, 1, 0);

        break;
    case MQTT_EVENT_DATA:
        // FILTRA TOPIC Y DATOS ENVIADOS
        snprintf(topic_buffer, sizeof(topic_buffer), "%.*s", event->topic_len, event->topic);
        snprintf(data_buffer, sizeof(data_buffer), "%.*s", event->data_len, event->data);
        char **data_payload = splitString(data_buffer);


        //escucha siempre al rival
        if (!strcmp(topic_buffer, gameTopic) && strcmp(data_payload[NAME_INDEX], me.name))
        {
            if (atoi(data_payload[GAME_STATUS_INDEX]) == GAME_ON_FLAG)
            {
                gameOnFlag = true;
            }

            if(atoi(data_payload[GAME_STATUS_INDEX]) == GAME_OFF_FLAG)
            {
                gameOnFlag = false;
            }

            rival.posRaqueta = atoi(data_payload[RAQUETA_POS_INDEX]);
            rival.puntos = atoi(data_payload[POINTS_INDEX]);
            rival.pelotaX = atoi(data_payload[PELOTA_POS_X]);
            rival.pelotaY = atoi(data_payload[PELOTA_POS_Y]);
            strcpy(rival.state, data_payload[LOGIN_STATE_INDEX]); 

        }

        if (!strcmp(topic_buffer, sesionTopic))
        {
            if (!strcmp(data_payload[LOGIN_STATE_INDEX], "ON") && strcmp(data_payload[NAME_INDEX], me.name))
            {
                ESP_LOGI(TAG, "Rival connected: %s", data_payload[NAME_INDEX]);
                snprintf(messageBuffer, sizeof(messageBuffer), "%s,%s,%d,%d,%d,%d,%d", me.name, me.state, me.posRaqueta, me.puntos, me.pelotaX, me.pelotaY, GAME_ON_FLAG);
                msg_id = esp_mqtt_client_publish(client, gameTopic, messageBuffer, 0, 1, 0);
            }

        }

        // free(data_payload);
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
            ESP_LOGI(TAG, "Player: %s, Sate: %s, Pad position: %d, Points: %d, Pong: <%d> <%d>", rival.name, rival.state, rival.posRaqueta, rival.puntos, rival.pelotaX, rival.pelotaY);
            ESP_LOGI(TAG, "Player: %s, Sate: %s, Pad position: %d, Points: %d, Pong: <%d> <%d>", me.name, me.state, me.posRaqueta, me.puntos, me.pelotaX, me.pelotaY);

            //          uint32_t adc_value = adc1_get_raw(ADC_CHANNEL);
            // Print the ADC value
            //            printf("ADC Value: %u\n", adc_value);

            // You can convert this ADC value to a voltage if you know the reference voltage and the ADC resolution
            // For example, if the reference voltage is 3.3V and the ADC resolution is 12 bits:
            //        float voltage = adc_value * (3.3 / ((1 << 12) - 1));
            //      printf("Voltage: %.2fV\n", voltage);

            snprintf(messageBuffer, sizeof(messageBuffer), "%s,%s,%d,%d,%d,%d", me.name, me.state, me.posRaqueta, me.puntos, me.pelotaX, me.pelotaY, GAME_ON_FLAG);
            msg_id = esp_mqtt_client_publish(client, gameTopic, messageBuffer, 0, 0, 0);
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

    // Configure UART
    uartInit(PC_UART_PORT, 115200, 3, 0, 1, PC_UART_TX_PIN, PC_UART_RX_PIN);
    ESP_ERROR_CHECK(example_connect());

    // configure_adc();
    // configure_gpio();
    char data[40];
    ESP_LOGI(TAG, "Nombre: ");

    uartGets(PC_UART_PORT, data);
    ESP_LOGI(TAG, "Name %s", data);
    strcpy(me.name, data);
    mqtt_app_start();
    xTaskCreatePinnedToCore(vTaskGameboard, "vTaskGameboard_task", 4096, NULL, 10, &vTaskGameboard_handler, 1);
}
