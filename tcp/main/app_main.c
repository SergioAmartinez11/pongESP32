/* JUEGO PONG
TABLERO 8x8
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
#include "esp_adc_cal.h"

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
char *startingTopic = "v1/starting" int gameBoard[BOARD_LEN][BOARD_LEN];
bool gameOnFlag = false;
int playersOnline = 0;
bool isYourTurn = false;
bool singleCall = true;

int posPelotaX = 0;
int posPelotaY = 0;

int directionX = 1;
int directionY = 1;

player_data_t me = {"playerA", "ON", 1, 0, 0, 0};
player_data_t rival;

TaskHandle_t vTaskGameboard_handler = NULL;

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
QueueHandle_t uart0_queue;
esp_mqtt_client_handle_t client;

#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  // Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_6; // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6; // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    // Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

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

        if (!strcmp(topic_buffer, startingTopic) && strcmp(data_payload[0], me.name))
        {
            posPelotaX = data_payload[1];
            posPelotaY = data_payload[2];
            gameOnFlag = true;
        }
        // escucha siempre al rival
        if (!strcmp(topic_buffer, gameTopic) && strcmp(data_payload[NAME_INDEX], me.name))
        {
            if (atoi(data_payload[GAME_STATUS_INDEX]) == GAME_ON_FLAG)
            {
                gameOnFlag = true;
            }

            if (atoi(data_payload[GAME_STATUS_INDEX]) == GAME_OFF_FLAG)
            {
                gameOnFlag = false;
            }

            rival.posRaqueta = atoi(data_payload[RAQUETA_POS_INDEX]);
            rival.puntos = atoi(data_payload[POINTS_INDEX]);
            rival.pelotaX = atoi(data_payload[PELOTA_POS_X]);
            rival.pelotaY = atoi(data_payload[PELOTA_POS_Y]);
            strcpy(rival.state, data_payload[LOGIN_STATE_INDEX]);
            strcpy(rival.name, data_payload[NAME_INDEX]);
        }

        if (!strcmp(topic_buffer, sesionTopic))
        {
            if (!strcmp(data_payload[LOGIN_STATE_INDEX], "ON") && strcmp(data_payload[NAME_INDEX], me.name))
            {
                // soy el jugador 1 entonces manda posicion inicial
                ESP_LOGI(TAG, "Rival connected: %s", data_payload[NAME_INDEX]);
                ESP_LOGI(TAG, "Game starting...");
                snprintf(messageBuffer, sizeof(messageBuffer), "%s,%d,%d", me.name, posPelotaX, posPelotaY);
                msg_id = esp_mqtt_client_publish(client, startingTopic, messageBuffer, 0, 1, 0);
                gameOnFlag = true;
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
            ESP_LOGI(TAG, "Rival: %s, Sate: %s, Pad position: %d, Points: %d, Pong: <%d> <%d>", rival.name, rival.state, rival.posRaqueta, rival.puntos, rival.pelotaX, rival.pelotaY);
            ESP_LOGI(TAG, "Me: %s, Sate: %s, Pad position: %d, Points: %d, Pong: <%d> <%d>", me.name, me.state, me.posRaqueta, me.puntos, me.pelotaX, me.pelotaY);

            int adc_reading = 0;
            // Multisampling
            for (int i = 0; i < NO_OF_SAMPLES; i++)
            {
                if (unit == ADC_UNIT_1)
                {
                    adc_reading += adc1_get_raw((adc1_channel_t)channel);
                }
                else
                {
                    int raw;
                    adc2_get_raw((adc2_channel_t)channel, width, &raw);
                    adc_reading += raw;
                }
            }
            adc_reading /= NO_OF_SAMPLES;
            // Convert adc_reading to voltage in mV
            int voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

            if (voltage > 900)
            {
                me.posRaqueta += 1;
            }
            if (voltage < 900)
            {
                me.posRaqueta -= 1;
            }

            if (me.posRaqueta < 1)
            {
                me.posRaqueta = 1;
            }
            if (me.posRaqueta > 8)
            {
                me.posRaqueta = 8;
            }

            if ((me.posRaqueta == me.pelotaY) && (me.pelotaX == 1))
            {
                // contacto con mi raqueta
                directionX = (-1) * directionX;
                posPelotaX = posPelotaX + directionX;
                posPelotaY = posPelotaY + directionY;
            }

            if (posPelotaY > 7)
            {
                directionY = (-1) * directionY
            }

            if (posPelotaY < 1)
            {
                directionY = (-1) * directionY
            }

            if ((me.posRaqueta != me.pelotaY) && (me.pelotaX < 1))
            {
                // no hubo contacto, el rival gana un punto
            }

            snprintf(messageBuffer, sizeof(messageBuffer), "%s,%s,%d,%d,%d,%d,%d", me.name, me.state, voltage, me.puntos, posPelotaX, posPelotaY, GAME_ON_FLAG);
            msg_id = esp_mqtt_client_publish(client, gameTopic, messageBuffer, 0, 0, 0);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
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

    // config ADC
    check_efuse();

    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    char data[40];
    ESP_LOGI(TAG, "Nombre: ");
    uartGets(PC_UART_PORT, data);
    strcpy(me.name, data);

    ESP_LOGI(TAG, "Posicion de pelota en Y: ");
    uartGets(PC_UART_PORT, data);
    strcpy(me.pelotaY, data);
    // la raqueta tambien tiene la misma posicion inicial que la pelota
    me.pelotaY = atoi(data);
    me.posRaqueta = atoi(data);

    mqtt_app_start();
    xTaskCreatePinnedToCore(vTaskGameboard, "vTaskGameboard_task", 4096, NULL, 10, &vTaskGameboard_handler, 1);
}
