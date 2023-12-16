#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "myUart.h"



void delayMs(uint16_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void uartClrScr(uart_port_t uart_num)
{
    // Uso "const" para sugerir que el contenido del arreglo lo coloque en Flash y no en RAM
    const char caClearScr[] = "\e[2J";
    uart_write_bytes(uart_num, caClearScr, sizeof(caClearScr));
}
void uartGoto11(uart_port_t uart_num)
{
    // Limpie un poco el arreglo de caracteres, los siguientes tres son equivalentes:
     // "\e[1;1H" == "\x1B[1;1H" == {27,'[','1',';','1','H'}
    const char caGoto11[] = "\e[1;1H";
    uart_write_bytes(uart_num, caGoto11, sizeof(caGoto11));
}

bool uartKbhit(uart_port_t uart_num)
{
    uint8_t length;
    uart_get_buffered_data_len(uart_num, (size_t*)&length);
    return (length > 0);
}

void uartPutchar(uart_port_t uart_num, char c)
{
    uart_write_bytes(uart_num, &c, sizeof(c));
}

void uartPuts(uart_port_t uart_num, char* str)
{
    uint32_t i = 0;
    char c;

    while(*(str + i) != '\0')
    {
        c = *(str + i);
        uartPutchar(uart_num, c);
        i++;                      //escribe el final de la cadena
        //printf("c: %c\n",c);
    }
    c = '\0';      
    uartPutchar(uart_num, c);

}

void uartGotoxy(uart_port_t uart_num, uint8_t x, uint8_t y){
    
    char cadX[10] = {0};
    char cadY[10] = {0};
    myItoa(x,cadX,10);
    myItoa(y,cadY,10);
    uartPuts(uart_num,"\e[");
    uartPuts(uart_num,cadX);
    uartPuts(uart_num,";");
    uartPuts(uart_num,cadY);
    uartPuts(uart_num,"H");
}

char uartGetchar(uart_port_t uart_num)
{
    char c;
    // Wait for a received byte
    while(!uartKbhit(uart_num))
    {
        delayMs(10);
    }
    // read byte, no wait
    uart_read_bytes(uart_num, &c, sizeof(c), 0);

    return c;
}

//puede ser char* uartGets(){... return cadena;}
void uartGets(uart_port_t uart_num, char *str)
{
    char c = 0;
    uint32_t i = 0;
    //uint32_t j = 0;

    do{
        c = uartGetchar(uart_num);
        if(c != 13){
            if(c == 8 && (i != 0))
            {   
                uartPutchar(uart_num,c);
                uartPutchar(uart_num,' ');
                uartPutchar(uart_num,c);
                
                i--;
            }else{
                if(!(c == 8 && i == 0)){
                    uartPutchar(uart_num,c);
                    *(str+i) = c;
                    i++; 
                }
            }   
        }
    }while(c != 13);
    *(str+i) = '\0';
    
}

void myItoa(uint32_t number, char* str, uint8_t base){
    int res = 0, c = 0;
    int size = 0, i = 0, j=0;
    char a,b; 

    c = number;
    while (c > 0)
    {
        res = c%base;
        c /= base;
        if(res > 9){
            str[i] = res + 55;
        }
        else{
            str[i] = res + 48;
        }
        i++;
    }
    str[i] = '\0';
    i=0;    
    //obtiene el size del str
    while(str[i] != '\0')
    {
        size++;
        i++;
    }
    
    for(i=0;i<size-1;i++)
    {
        for(j=i+1;j<size;j++)
        {
            a = str[i];
            b = str[j];
            str[i] = b;
            str[j] = a;   
        }
    }
   

}

uint16_t myAtoi(char *str)
{
    int num = 0,i = 0,pos=1,size=0;
    while(str[i] != '\0')
    {
        size++;
        i++;
    }


    for(i=0;i<size-1;i++)
    {
        pos *= 10;
    }
    for(i=0;i<size;i++)
    {
        if(str[i] >='0' && str[i] <= '9')
        {
            num=num+(str[i] - 48)*pos;
            pos /= 10;
        }
        else{
            num=num/(pos*10);
            break;
        }
    }

   
    return num;
}

void uartSetColor(uart_port_t uart_num, uint8_t color){
    char setColor[10];
    myItoa(color,setColor,10);
    uartPuts(uart_num,"\033[0;");
    uartPuts(uart_num,setColor);
    uartPuts(uart_num,"m");

}


void uartInit(uart_port_t uart_num, uint32_t baudrate, uint8_t size, uint8_t parity, uint8_t stop, uint8_t txPin, uint8_t rxPin)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    const uart_config_t uart_config = {
        .baud_rate = baudrate,        /*UARTS_BAUD_RATE,*/
        .data_bits = size,                 //UART_DATA_8_BITS,
        .parity    = parity,                   //UART_PARITY_DISABLE,
        .stop_bits = stop,                   //UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, READ_BUF_SIZE, READ_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, txPin, rxPin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

}