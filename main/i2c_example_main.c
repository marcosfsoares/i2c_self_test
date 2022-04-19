/*
  ******************************************************************************
  * @file           : i2c_example.c.c
  * @brief          : Programa principal
  ******************************************************************************
  * //  Projeto - Regador Intligente
  * //  Disciplina: Sistema Operacional em Tempo Real
  * //  Curso de Pós Graduação em Sistemas Embarcados - SENAI-SP
  * //  Alunos: 
  * //          Marcos Flávio Soares
  * //          Leonardo Pongillo
  ******************************************************************************
*/

// ========== Bibliotecas ==========
#include <stdio.h>
#include <stdint.h>  
#include <string.h>
#include <stddef.h>

/*
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/**
 * WiFi
 */
#include "esp_wifi.h"

/*
 * Wi-Fi Manager
 */
#include "wifi_manager.h"

/*
 * Drivers
 */
#include "driver/gpio.h"
#include "driver/adc.h"

/*
 * logs
 */
#include "esp_log.h"

/*
 * System
 */
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

/**
 * NVS
 */
#include "nvs.h"
#include "nvs_flash.h"

/**
 * Lib MQTT
 */
#include "mqtt_client.h"
#include "esp_tls.h"


/**
 * Lwip
 */
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/api.h"
#include "lwip/netdb.h"
#include "lwip/ip4_addr.h"

// ========== Defines ==========
#define DEBUG 1
//#define TOPICO_UMIDADE "umidade"
#define TOPICO_UMIDADE_WRITE "channels/1705493/publish"
#define TOPICO_CHUVA "chuva"
#define DASHTIME 8000


// Mapa display
#define LCD_RS 19
#define LCD_E  23
#define LCD_P4 18
#define LCD_P5 17
#define LCD_P6 16
#define LCD_P7 15

#define MOT_PIN  22

#define BUTTON_1    5 
#define GPIO_INPUT_PIN_SEL  (1ULL<<BUTTON_1)

// ========== Variáveis Globais ==========
static const char *TAG = "Regador";
QueueHandle_t xMailbox_Sensor; 
QueueHandle_t xMailbox_Motor;   
SemaphoreHandle_t xSemaphore;
static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t mqtt_event_group;
const static int WIFI_CONNECTED_BIT = BIT0;
esp_mqtt_client_handle_t client;


// ========== Protótipos ==========
static void sensor_display_task(void *pvParameter);
static void sensor_umidade_task(void *pvParameter);
static void IRAM_ATTR gpio_isr_handler(void * pvParameter);
static void motor_task(void *pvParameter);
static void dashboard_task(void *pvParameter);

static void init_display();
static void init_botao();
static void comando_display(uint8_t data);
static void arrange_display(uint8_t data);
static void char_display(uint8_t data);
static void char_display(uint8_t data);

static void set_display(uint8_t line, uint8_t column);

static void mqtt_app_start( void );
void wifi_off(void *pvParameter);
void wifi_on(void *pvParameter);
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
void monitoring_task(void *pvParameter);



// ========== Inicialização ==========



void app_main(void)
{
    
    /*
    	Inicialização da memória não volátil para armazenamento de dados (Non-volatile storage (NVS)).
    	**Necessário para realização da calibração do PHY. 
    */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


     /*
       Event Group do FreeRTOS. 
       Só podemos enviar ou ler alguma informação TCP quando a rede WiFi estiver configurada, ou seja, 
       somente após o aceite de conexão e a liberação do IP pelo roteador da rede.
    */
    wifi_event_group = xEventGroupCreate();
    

    /*
      O ESP32 está conectado ao broker MQTT? Para sabermos disso, precisamos dos event_group do FreeRTOS.
    */
    mqtt_event_group = xEventGroupCreate();

    /* inicia o wifi manager */
	wifi_manager_start();

    /* Funções de callback para os eventos de aquisição de IP e desconexão */
	wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &wifi_on);
	wifi_manager_set_callback(WM_EVENT_STA_DISCONNECTED, &wifi_off);

    /**
	*	Aguardando a conexão do wi-fi com o roteador
	*/
    xEventGroupWaitBits( wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY );

    /**
     * Inicializa e configura a rede mqtt;
     */
    mqtt_app_start();

    /**
     * Inicializa e display  o botão;
     */
    init_display();
    init_botao();
    
    xSemaphore = xSemaphoreCreateBinary();
    
    /* comando necessário pois o semáforo começa em zero */
    xSemaphoreGive(xSemaphore);

    if((xMailbox_Sensor = xQueueCreate(1, sizeof(float)))== NULL)
	{
		ESP_LOGI(TAG, "Não é possível criar xQueue_Sensor...");	
        return;	
	}

    if((xMailbox_Motor = xQueueCreate(1, sizeof(uint8_t)))== NULL)
	{
		ESP_LOGI(TAG, "Não é possível criar xQueue_Motor...");	
        return;	
	}

    if((xTaskCreate(sensor_display_task, "display_task", 1024 * 2, (void *)0, 10, NULL)) != pdTRUE)
	{
		ESP_LOGI(TAG, "Não é possível criar sensor_display_task...");
        return;		
    }

    if((xTaskCreate(sensor_umidade_task, "sensor_umidade_task", 1024 * 2, (void *)0, 10, NULL)) != pdTRUE)
	{
		ESP_LOGI(TAG, "Não é possível criar sensor_umidade_task...");
        return;		
	}

    if((xTaskCreate(motor_task, "motor_task", 1024 * 2, (void *)0, 10, NULL)) != pdTRUE)
	{
		ESP_LOGI(TAG, "Não é possível criar motor_task...");
        return;		
	}

    if((xTaskCreate(dashboard_task, "dashboard_task", 1024 * 2, (void *)0, 10, NULL)) != pdTRUE)
	{
		ESP_LOGI(TAG, "Não é possível criar dashboard_task...");
        return;		
	}

    /*  Here we simply create a task on core 2 that monitors free heap memory */
	xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 2048, NULL, 1, NULL, 1);
}


/**
* Task de monitoramento do heap do core1
*/
void monitoring_task(void *pvParameter)
{
	for(;;){
		ESP_LOGI(TAG, "free heap: %d",esp_get_free_heap_size());
		vTaskDelay( pdMS_TO_TICKS(10000) );
	}
}

/**
* Função de callback chamada quando o ESP32 se conectar na rede Wi-Fi
*/
void wifi_on(void *pvParameter)
{
	ip_event_got_ip_t* param = (ip_event_got_ip_t*)pvParameter;

	/* transform IP to human readable string */
	char str_ip[16];
	esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);
	if(DEBUG)
	{
		ESP_LOGI(TAG, "WI-FI is connected under the IP %s!", str_ip);
	}
	xEventGroupSetBits( wifi_event_group, WIFI_CONNECTED_BIT );
}


/**
 * Função de callback chamada quando o ESP32 estiver offline na rede WiFi;
 */
void wifi_off(void *pvParameter)
{
	if( DEBUG )
		ESP_LOGI(TAG, "WI-FI is disconnected.");
	
	xEventGroupClearBits( wifi_event_group, WIFI_CONNECTED_BIT );
}


/**
 * Função de callback do stack MQTT; 
 * Por meio deste callback é possível receber as notificações com os status
 * da conexão e dos tópicos assinados e publicados;
 */
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    client = event->client;
    int msg_id;
    
    switch (event->event_id) 
    {

        case MQTT_EVENT_BEFORE_CONNECT: 
			if( DEBUG )
				ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
        break;

        /**
         * Evento chamado quando o ESP32 se conecta ao broker MQTT, ou seja, 
         * caso a conexão socket TCP, SSL/TSL e autenticação com o broker MQTT
         * tenha ocorrido com sucesso, este evento será chamado informando que 
         * o ESP32 está conectado ao broker;
         */
        case MQTT_EVENT_CONNECTED:

            if( DEBUG )
                ESP_LOGI(TAG, "MQTT_CONECTADO_AO_BROKER");

            /**
             * Se chegou aqui é porque o ESP32 está conectado ao Broker MQTT; 
             * A notificação é feita setando em nível 1 o bit CONNECTED_BIT da 
             * variável mqtt_event_group;
             */
            xEventGroupSetBits( mqtt_event_group, WIFI_CONNECTED_BIT );
            break;
        /**
         * Evento chamado quando o ESP32 for desconectado do broker MQTT;
         */
        case MQTT_EVENT_DISCONNECTED:

            if( DEBUG )
                ESP_LOGI(TAG, "MQTT_DESCONECTADO_DO_BROKER");   
            /**
             * Se chegou aqui é porque o ESP32 foi desconectado do broker MQTT;
             */
            xEventGroupClearBits(mqtt_event_group, WIFI_CONNECTED_BIT);
            break;

        /**
         * O eventos seguintes são utilizados para notificar quando um tópico é
         * assinado pelo ESP32;
         */
        case MQTT_EVENT_SUBSCRIBED:
        
            if( DEBUG )
                ESP_LOGI(TAG, "MQTT_ASSINATURA_EFETUADA_NO_BROKER");  
            break;
        
        /**
         * Quando a assinatura de um tópico é cancelada pelo ESP32, 
         * este evento será chamado;
         */
        case MQTT_EVENT_UNSUBSCRIBED:
            if( DEBUG )
                ESP_LOGI(TAG, "MQTT_ASSINATURA_CANCELADA_NO_BROKER");  
            break;
        
        /**
         * Este evento será chamado quando um tópico for publicado pelo ESP32;
         */
        case MQTT_EVENT_PUBLISHED:
            
            if( DEBUG )
                ESP_LOGI(TAG, "MQTT_MENSAGEM_PUBLICADA, msg_id=%d", event->msg_id);
            break;
        
        /**
         * Este evento será chamado quando uma mensagem chegar em algum tópico 
         * assinado pelo ESP32;
         */
        case MQTT_EVENT_DATA:

            if( DEBUG )
            {
                ESP_LOGI(TAG, "MQTT_CHEGADA_DE_DADOS"); 

                /**
                 * Sabendo o nome do tópico que publicou a mensagem é possível
                 * saber a quem data pertence;
                 */
                ESP_LOGI(TAG, "TOPICO=%.*s\r\n", event->topic_len, event->topic);
                ESP_LOGI(TAG, "DADOS RECEBIDOS=%.*s\r\n", event->data_len, event->data);               
            }       
            break;
        
        /**
         * Evento chamado quando ocorrer algum erro na troca de informação
         * entre o ESP32 e o Broker MQTT;
         */
        case MQTT_EVENT_ERROR:
            if( DEBUG )
                ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
			
		case MQTT_EVENT_ANY:
			if( DEBUG )
				ESP_LOGI(TAG, "MQTT_EVENTOS_ANY");
			break;
        
        case MQTT_EVENT_DELETED:
            if( DEBUG )
                ESP_LOGI(TAG, "MQTT_EVENT_DELETED");
            break;
        default:
            if( DEBUG )
				ESP_LOGI(TAG, "MQTT_EVENTOS_ERROR");
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	if( DEBUG )
		ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

/**
 * Configura o stack MQTT carregando o certificado SSL/TLS;
 */
static void mqtt_app_start( void )
{
   /**
    * Conexão MQTT com certificado SSL/TSL
    */
    const esp_mqtt_client_config_t mqtt_cfg = 
    {
		.uri = "mqtt://mqtt3.thingspeak.com:1883", 
		.username = "KwM6BwQyJRINNSQyCjktNgs",
		.password = "d34VKzuqX08JpgFZX8BqQmuJ",
        .client_id = "KwM6BwQyJRINNSQyCjktNgs"
    };


    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}



static void sensor_display_task(void *pvParameter)
{
    ESP_LOGI(TAG, "sensor_display_task");
    uint8_t on = 0;
    float floatData;
    int centena;
    int dezena;
    int unidade;
    

    while (1) 
    { 
        xQueuePeek(xMailbox_Sensor, &floatData, portMAX_DELAY);
        
        ESP_LOGI(TAG, "floatData: %.2f", floatData);

        
        if(floatData < 70)
        {
            gpio_set_level(MOT_PIN, 0);
            // Envia o valor do sensor para a fila "xQueueMotor"
            on = 1;
            xQueueOverwrite(xMailbox_Motor, &on);
           
        }
        else
        {
            gpio_set_level(MOT_PIN, 1);
            // Envia o valor do sensor para a fila "xQueueMotor"
            on = 0;
            xQueueOverwrite(xMailbox_Motor, &on);
          
        }

    
        centena = (uint8_t)floatData / 100;
        dezena  = (uint8_t)(floatData / 10) % 10; 
        unidade = (uint8_t)floatData % 10;

        set_display(2,9);
        char_display(centena + 48);
        char_display(dezena  + 48);
        char_display(unidade + 48);
        char_display(37);
        
       

        vTaskDelay(2000 / portTICK_RATE_MS); 
    }

    vTaskDelete(NULL);
}

static void sensor_umidade_task(void *pvParameter)
{
    
    float floatData = 0;
    ESP_LOGI(TAG, "sensor_umidade_task");

    // Configurando a resolução do ADC para 10bits        
    adc1_config_width(3);
    
    // Configurando o Channel do ADC para o Channel 0
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);

    while(1)
	{
        // Função que de leitura do valor analogico, passando o ADC1 que é o do GPIO1
        uint8_t sensorValue = adc1_get_raw(ADC1_CHANNEL_0);

        ESP_LOGI(TAG, "valor: %d", sensorValue);
        
        floatData = (100 - (sensorValue * 100) / 255 );
        ESP_LOGI(TAG, "Porcentagem de Umidade: %.2f", floatData);
 

        // Envia o valor do sensor para a fila "xQueueSensor"
        xQueueOverwrite(xMailbox_Sensor, &floatData);
 
        // Descarrega os buffers de saída de dados
        fflush(stdout);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}

static void motor_task(void *pvParameter)
{
    while(1)
    {
        if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {  
            ESP_LOGI(TAG, "BUTTON_1");
            gpio_set_level(MOT_PIN, 0);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
         gpio_set_level(MOT_PIN, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

}


static void dashboard_task(void *pvParameter)
{
    float dashboardData;
    char str[100]; 
    uint32_t on = 0;


    while(1)
    {
        xQueuePeek(xMailbox_Sensor, &dashboardData, portMAX_DELAY);
        xQueuePeek(xMailbox_Motor, &on, portMAX_DELAY);
        
        // Formatação para publicar na plataforma ThingSpeak
        sprintf( str, "field1=%.2f&field2=%d&status=MQTTPUBLISH", dashboardData, on); 
       

        if( esp_mqtt_client_publish( client, TOPICO_UMIDADE_WRITE, str, strlen( str ), 0, 0 ) == 0 )
		{
			if( DEBUG )
                ESP_LOGI( TAG, "Mensagem publicada com sucesso.");
		}
        vTaskDelay(DASHTIME / portTICK_PERIOD_MS);

        
     
    }
    vTaskDelete(NULL);
}


// ==========================================================================================================


static void init_display()
{
    gpio_pad_select_gpio(LCD_RS);  
    gpio_set_direction(LCD_RS,GPIO_MODE_OUTPUT); // Configura como entrada digital
    gpio_pullup_en(LCD_RS);
    gpio_pulldown_dis(LCD_RS);  

    gpio_pad_select_gpio(LCD_E);  
    gpio_set_direction(LCD_E,GPIO_MODE_OUTPUT); // Configura como entrada digital
    gpio_pullup_en(LCD_E);
    gpio_pulldown_dis(LCD_E); 

    gpio_pad_select_gpio(LCD_P4);  
    gpio_set_direction(LCD_P4,GPIO_MODE_OUTPUT); // Configura como entrada digital
    gpio_pullup_en(LCD_P4);
    gpio_pulldown_dis(LCD_P4); 

    gpio_pad_select_gpio(LCD_P5);  
    gpio_set_direction(LCD_P5,GPIO_MODE_OUTPUT); // Configura como entrada digital
    gpio_pullup_en(LCD_P5);
    gpio_pulldown_dis(LCD_P5); 

    gpio_pad_select_gpio(LCD_P6);  
    gpio_set_direction(LCD_P6,GPIO_MODE_OUTPUT); // Configura como entrada digital
    gpio_pullup_en(LCD_P6);
    gpio_pulldown_dis(LCD_P6); 

    gpio_pad_select_gpio(LCD_P7);  
    gpio_set_direction(LCD_P7,GPIO_MODE_OUTPUT); // Configura como entrada digital
    gpio_pullup_en(LCD_P7);
    gpio_pulldown_dis(LCD_P7); 

    comando_display(0x03);
    comando_display(0x03);
    comando_display(0x03);
    comando_display(0x02);  /*send for initialization of LCD */
    comando_display(0x28);  /* 4 bits, 2 linhas, 5x8*/
	comando_display(0x01);  /*clear display screen*/
    comando_display(0x0c);  /*display on cursor off*/
	comando_display(0x06);  /*increment cursor (shift cursor to right)*/
    comando_display(0x80);  /*posição inicial*/

    set_display(1, 5);
    char_display('R');
    char_display('E');
    char_display('G');
    char_display('A');
    char_display('D');
    char_display('O');
    char_display('R');

    set_display(2, 1);
    char_display('U');
    char_display('M');
    char_display('I');
    char_display('D');
    char_display('A');
    char_display('D');
    char_display('E');

}

static void init_botao()
{
    /**
	 * Button (por interrupção externa)
	 */
	gpio_config_t io_conf2 = {
		    .intr_type      = GPIO_INTR_NEGEDGE,
		    .mode           = GPIO_MODE_INPUT,
		    .pin_bit_mask   = GPIO_INPUT_PIN_SEL,
		    .pull_up_en     = GPIO_PULLDOWN_DISABLE,
		    .pull_down_en   = GPIO_PULLDOWN_DISABLE
	};
    gpio_config(&io_conf2); 

    /**
     * Habilita a interrupção externa das GPIOs;
     */
    gpio_install_isr_service(1);
    
    /**
     * Registra o pino que irá gerar acionar a interrupção por borda de descida 
     * e informa qual a função de callback que será chamada. 
     * O número da GPIOs será passado no parametro da função de callback.
     */
    gpio_isr_handler_add(BUTTON_1, gpio_isr_handler, (void*) BUTTON_1 );

    gpio_pad_select_gpio(MOT_PIN);
    gpio_set_direction(MOT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MOT_PIN, 0);
}

static void comando_display(uint8_t data)
{
    uint8_t Ldata = 0x00;
    uint8_t Hdata = 0x00;

    Hdata = (data >> 4);
    //ESP_LOGW(TAG, "Hdata %d", Hdata);
    arrange_display(Hdata);
    vTaskDelay(pdMS_TO_TICKS(15)); //ticks para ms;
    gpio_set_level(LCD_RS, 0);
    gpio_set_level(LCD_E, 1);
    vTaskDelay(pdMS_TO_TICKS(15)); //ticks para ms;
    gpio_set_level(LCD_E, 0);

    Ldata = (data & 0x0F);
    //ESP_LOGW(TAG, "Ldata %d", Ldata);
    arrange_display(Ldata);
    vTaskDelay(pdMS_TO_TICKS(15)); //ticks para ms;
    gpio_set_level(LCD_RS, 0);
    gpio_set_level(LCD_E, 1);
    vTaskDelay(pdMS_TO_TICKS(15)); //ticks para ms;
    gpio_set_level(LCD_E, 0);
}

static void char_display(uint8_t data)
{
    uint8_t Ldata = 0x00;
    uint8_t Hdata = 0x00;

    Hdata = (data >> 4);
    //ESP_LOGW(TAG, "Hdata %d", Hdata);
    arrange_display(Hdata);
    vTaskDelay(pdMS_TO_TICKS(15)); //ticks para ms;
    gpio_set_level(LCD_RS, 1);
    gpio_set_level(LCD_E, 1);
    vTaskDelay(pdMS_TO_TICKS(15)); //ticks para ms;
    gpio_set_level(LCD_E, 0);

    Ldata = (data & 0x0F);
    //ESP_LOGW(TAG, "Ldata %d", Ldata);
    arrange_display(Ldata);
    vTaskDelay(pdMS_TO_TICKS(15)); //ticks para ms;
    gpio_set_level(LCD_RS, 1);
    gpio_set_level(LCD_E, 1);
    vTaskDelay(pdMS_TO_TICKS(15)); //ticks para ms;
    gpio_set_level(LCD_E, 0);
}

static void arrange_display(uint8_t data)
{
    switch (data)
    {
    case 0:
        gpio_set_level(LCD_P7, 0);
        gpio_set_level(LCD_P6, 0);
        gpio_set_level(LCD_P5, 0);
        gpio_set_level(LCD_P4, 0);
        break;

    case 1:
        gpio_set_level(LCD_P7, 0);
        gpio_set_level(LCD_P6, 0);
        gpio_set_level(LCD_P5, 0);
        gpio_set_level(LCD_P4, 1);
        break;

    case 2:
        gpio_set_level(LCD_P7, 0);
        gpio_set_level(LCD_P6, 0);
        gpio_set_level(LCD_P5, 1);
        gpio_set_level(LCD_P4, 0);
        break;

    case 3:
        gpio_set_level(LCD_P7, 0);
        gpio_set_level(LCD_P6, 0);
        gpio_set_level(LCD_P5, 1);
        gpio_set_level(LCD_P4, 1);
        break;

    case 4:
        gpio_set_level(LCD_P7, 0);
        gpio_set_level(LCD_P6, 1);
        gpio_set_level(LCD_P5, 0);
        gpio_set_level(LCD_P4, 0);
        break;

    case 5:
        gpio_set_level(LCD_P7, 0);
        gpio_set_level(LCD_P6, 1);
        gpio_set_level(LCD_P5, 0);
        gpio_set_level(LCD_P4, 1);
        break;

    case 6:
        gpio_set_level(LCD_P7, 0);
        gpio_set_level(LCD_P6, 1);
        gpio_set_level(LCD_P5, 1);
        gpio_set_level(LCD_P4, 0);
        break;

    case 7:
        gpio_set_level(LCD_P7, 0);
        gpio_set_level(LCD_P6, 1);
        gpio_set_level(LCD_P5, 1);
        gpio_set_level(LCD_P4, 1);
        break;

    case 8:
        gpio_set_level(LCD_P7, 1);
        gpio_set_level(LCD_P6, 0);
        gpio_set_level(LCD_P5, 0);
        gpio_set_level(LCD_P4, 0);
        break;

    case 9:
        gpio_set_level(LCD_P7, 1);
        gpio_set_level(LCD_P6, 0);
        gpio_set_level(LCD_P5, 0);
        gpio_set_level(LCD_P4, 1);
        break;

    case 10:
        gpio_set_level(LCD_P7, 1);
        gpio_set_level(LCD_P6, 0);
        gpio_set_level(LCD_P5, 1);
        gpio_set_level(LCD_P4, 0);
        break;

    case 11:
        gpio_set_level(LCD_P7, 1);
        gpio_set_level(LCD_P6, 0);
        gpio_set_level(LCD_P5, 1);
        gpio_set_level(LCD_P4, 1);
        break;

    case 12:
        gpio_set_level(LCD_P7, 1);
        gpio_set_level(LCD_P6, 1);
        gpio_set_level(LCD_P5, 0);
        gpio_set_level(LCD_P4, 0);
        break;

    case 13:
        gpio_set_level(LCD_P7, 1);
        gpio_set_level(LCD_P6, 1);
        gpio_set_level(LCD_P5, 0);
        gpio_set_level(LCD_P4, 1);
        break;

    case 14:
        gpio_set_level(LCD_P7, 1);
        gpio_set_level(LCD_P6, 1);
        gpio_set_level(LCD_P5, 1);
        gpio_set_level(LCD_P4, 0);
        break;

    case 15:
        gpio_set_level(LCD_P7, 1);
        gpio_set_level(LCD_P6, 1);
        gpio_set_level(LCD_P5, 1);
        gpio_set_level(LCD_P4, 1);
        break;
    }    
}

static void set_display(uint8_t line, uint8_t column)
{
    uint8_t data = 0x00;

    if(line == 1)
    {
        data = 0x80 + column - 1;
        comando_display(data);
    }
    else if(line >= 1)
    {
        data = 0xC0 + column - 1;
        comando_display(data);
    }
}

static void IRAM_ATTR gpio_isr_handler(void * pvParameter)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(BUTTON_1 == (uint32_t) pvParameter)
    {
        if( gpio_get_level(BUTTON_1 ) == 0) 
        {
            xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);

            if(xHigherPriorityTaskWoken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }
        }   
    }   
}