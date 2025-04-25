#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <math.h>
#include "sdkconfig.h" 
#include "ssd1366.h"
#include "font8x8_basic.h"
#include <driver/gpio.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <dht11.h>
#include <string.h>
#include "driver/uart.h"


// GPS VARIAVEIS GLOBAIS
//GPS
#define LEN_LAT_LONG_QUEUE 1000
#define R 6371.0
#define GPS_UART UART_NUM_2

#define TXD_PIN 17
#define RXD_PIN 16

#define BUF_SIZE 1024
#define BT_IO2 0

SemaphoreHandle_t mutex_dados;
SemaphoreHandle_t sem_sinc;
SemaphoreHandle_t mutex_horario;
SemaphoreHandle_t mutex_media;


bool corrida_em_andamento = false;

typedef struct {
    int hora;
    int minuto;
    int segundo;
} tempo;

tempo hora_atual;
char start_time[10];
char finish_time[10];
float velocidade_media;
float total_calorias;
float velocidade_instantanea = 0;
bool primeiro_valor = true;
float ultima_altitude = 0;
// inicializa os valores como 0

double total_distance = 0;
double total_altitude = 0;
float delta_tempo_final= 0;
typedef struct {
    float latitude;
    float longitude;
    char time[10];
} coordenada;

//int pos = 0;
//coordenada geral[LEN_LAT_LONG_QUEUE];

float delta_altitude = 0;

QueueHandle_t fila_coordenadas;
// FINAL GPS VARIAVEIS GLOBAIS

//bool delay_dht11 = false;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//comunicação i2c

#define SDA_PIN 21
#define SCL_PIN 22

SemaphoreHandle_t xMutexi2c;

void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//DHT11

SemaphoreHandle_t xMutexdht;
struct dht11_reading;
#define DHT11_PIN 4
uint8_t temp;
uint8_t umi;

void vTaskGet(void *pvparameters)
{   
    struct dht11_reading temp_data;

    ESP_LOGI("GET","Task GET iniciando leitura DHT11...");
    while (1)
    {   
        if (xSemaphoreTake(xMutexdht, portMAX_DELAY)) { 
        temp_data=DHT11_read();
        temp = temp_data.temperature;
        umi = temp_data.humidity;
        ESP_LOGI("GET","Medindo temperatura");
        ESP_LOGI("GET","Temperatura: %d | Umidade: %d",temp,umi);
        xSemaphoreGive(xMutexdht);
        if (temp == 255 && umi==255){
            ESP_LOGI("GET","Estourou");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else
        {   
            vTaskDelay(pdMS_TO_TICKS(45000));

        }
        
        }
    }
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ADXL345

#define STEP_THRESHOLD 282  // Limite para detectar um passo
#define BUFFER_SIZE 10       // Tamanho do buffer para média móvel


#define ADXL345_ADDR 0x53  //endereço i2c
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATAX0 0x32

int step_count = 0;
float accel_buffer[BUFFER_SIZE] = {0};
int buffer_index = 0;
uint16_t passos=0;

void adxl345_init() {
    uint8_t power_ctl_reg[] = {ADXL345_POWER_CTL, 0x08}; // Liga o sensor

    esp_err_t espRc = i2c_master_write_to_device(
        I2C_NUM_0, ADXL345_ADDR, power_ctl_reg, sizeof(power_ctl_reg), 1000 / portTICK_PERIOD_MS
    );

    if (espRc == ESP_OK) {
        ESP_LOGI("ADXL345", "ADXL345 configurado com sucesso");
    } else {
        ESP_LOGE("ADXL345", "Falha ao configurar ADXL345. Código: 0x%.2X", espRc);
    }
}

void adxl345_read(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    uint8_t reg = ADXL345_DATAX0;

    esp_err_t espRc = i2c_master_write_read_device(
        I2C_NUM_0, ADXL345_ADDR, &reg, 1, data, 6, 1000 / portTICK_PERIOD_MS
    );

    if (espRc == ESP_OK) {
        *x = (int16_t)(data[1] << 8 | data[0]);
        *y = (int16_t)(data[3] << 8 | data[2]);
        *z = (int16_t)(data[5] << 8 | data[4]);
    } else {
        ESP_LOGE("ADXL345", "Erro ao ler dados do ADXL345. Código: 0x%.2X", espRc);
    }
}

void task_adxl345(void *pvParameters) {
    int16_t x, y, z;
    float magnitude=0;
    float prev_magnitude = 0;

    while (1) {
        if (xSemaphoreTake(xMutexi2c, portMAX_DELAY)) {
        adxl345_read(&x, &y, &z);
        //ESP_LOGI("ADXL345", "X: %d, Y: %d, Z: %d", x, y, z);
        // Média móvel para suavizar o sinal
        accel_buffer[buffer_index] = magnitude;
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;

        magnitude = sqrt((x*x)+(y*y)+(z*z));
        //printf("magnitude: %f", magnitude);

        float avg_magnitude = 0.0;
        for (int i = 0; i < BUFFER_SIZE; i++) {
            avg_magnitude += accel_buffer[i];
        }
        avg_magnitude /= BUFFER_SIZE;
        //printf("avg magnitude: %f", avg_magnitude);

        // Detecta um passo baseado no threshold
        if (avg_magnitude > STEP_THRESHOLD && prev_magnitude < STEP_THRESHOLD) {
            passos++;
            printf("Passo detectado! Total: %d\n", passos);
        }

        prev_magnitude = avg_magnitude;
        xSemaphoreGive(xMutexi2c);
        vTaskDelay(pdMS_TO_TICKS(100));  // Atualiza a cada 500ms
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Display

#define tag "SSD1306"
uint8_t telas=0;
#define BT_IO 23

SemaphoreHandle_t botao_tela;

void botao_cont (void *pvParameters){

    while (1)
    {
        xSemaphoreTake(botao_tela, portMAX_DELAY);
        if(gpio_get_level(BT_IO)==0){
            ESP_LOGW("botao_tela","Entrou no botao_tela");
            telas++;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(tag, "OLED configured successfully");
	} else {
		ESP_LOGE(tag, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}


void ssd1306_clear_display(void) {
    i2c_cmd_handle_t cmd;

    uint8_t zero[128]={0}; // Vetor de zeros para limpar o display
    for (uint8_t i = 0; i < 8; i++) {
        // Cria um novo comando I2C
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

        // Define o byte de controle para comando único
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 | i, true); // Configura a página

        // Escreve os dados para limpar a linha (128 zeros)
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(cmd, zero, 128, true);

        // Finaliza o comando I2C
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }
}

void ssd1306_display_text(const char *text) {
    uint8_t text_len = strlen(text);
    i2c_cmd_handle_t cmd;
    uint8_t cur_page = 0;

    // Inicia a comunicação com o display
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    // Configura o controle de página e coluna inicial
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0x00, true); // reset coluna
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset página

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Exibe o texto no display
    for (uint8_t i = 0; i < text_len; i++) {
        if (text[i] == '\n') {
            // Nova linha
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
            i2c_master_write_byte(cmd, 0x00, true); // reset coluna
            i2c_master_write_byte(cmd, 0x10, true);
            i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // incrementa página

            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
        } else {
            // Escreve o caractere no display
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
            i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
        }
    }
}


void task_ssd1306_display_text(void *pvparameters) {
    // Chama a função para exibir o texto no display
    
    while (1)
    {
        ssd1306_clear_display();
        if(telas==0){
            if (xSemaphoreTake(xMutexi2c, portMAX_DELAY)) {  
            char str[128]="";
            snprintf(str, sizeof(str), "     Contagem \n    de passos:\n           \n        %d", passos);
            ssd1306_display_text(str);
            vTaskDelay(pdMS_TO_TICKS(1000));
            xSemaphoreGive(xMutexi2c);  
            }
        }
        else if (telas==1)
        {
            if (xSemaphoreTake(xMutexdht, portMAX_DELAY)) { 
            char str2[128]="";
            snprintf(str2, sizeof(str2), "   Temperatura: \n    %d oC \n    Umidade:    \n      %d ", temp, umi);
            ssd1306_display_text(str2);
            xSemaphoreGive(xMutexdht);
            vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        /*
        else if (telas == 2) {
            char str3[256] = "";
            float tempo_de_corrida = 0.0; // Inicialize aqui
            if (corrida_em_andamento && xSemaphoreTake(mutex_dados, portMAX_DELAY)) {

                coordenada coordenada_final;
                if (uxQueueMessagesWaiting(fila_coordenadas) != 0) {
                    if (xQueueReceive(fila_coordenadas, &coordenada_final, portMAX_DELAY) == pdTRUE) {
                        char tempo_final[10];
                        strcpy(tempo_final, coordenada_final.time);
                        int final_hours, final_minutes, final_seconds;
                        sscanf(tempo_final, "%2d%2d%2d", &final_hours, &final_minutes, &final_seconds);
                        int inicial_hours, inicial_minutes, inicial_seconds;
                        sscanf(start_time, "%2d%2d%2d", &inicial_hours, &inicial_minutes, &inicial_seconds);
                        tempo_de_corrida = ((final_hours * 3600 + final_minutes * 60 + final_seconds) - (inicial_hours * 3600 + inicial_minutes * 60 + inicial_seconds));
                        tempo_de_corrida = tempo_de_corrida / 60; // transforma para minutos
                    }
                }
                xSemaphoreGive(mutex_dados);
            }
            snprintf(str3, sizeof(str3), " Vel. Inst.:\n %f km/h \n Distancia: \n %lf km\n Tempo Corr.: \n %f min", velocidade_instantanea, total_distance, tempo_de_corrida);
            ssd1306_display_text(str3);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        else if (telas==3){
            char str4[128] = "";
            if (xSemaphoreTake(mutex_horario, portMAX_DELAY)) {
                ESP_LOGW("TELA","TELA 3");
                snprintf(str4, sizeof(str4), "   Horario:   \n  %02d:%02d:%02d",hora_atual.hora, hora_atual.minuto, hora_atual.segundo );
                ssd1306_display_text(str4);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xSemaphoreGive(mutex_horario);
             }
            
                }
        else if (telas == 4){
            char str5[128] = "";
            if (xSemaphoreTake(mutex_dados, portMAX_DELAY)) {
                ESP_LOGW("TELA","TELA 4");
                snprintf(str5, sizeof(str5), "  Total de Calorias: \n  %f \n  D. Altitude:\n   %f",total_calorias,delta_altitude);
                ssd1306_display_text(str5);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xSemaphoreGive(mutex_dados);
             }
                }
        else if (telas == 5){
            char str6[128] = "";
            if (xSemaphoreTake(mutex_media, portMAX_DELAY)) {
                ESP_LOGW("TELA","TELA 5");
                snprintf(str6, sizeof(str6), "  Resumo:   \n Vel. Med.: \n  %f \n  Tempo Tot.:\n   %f",velocidade_media,delta_tempo_final);
                ssd1306_display_text(str6);
                vTaskDelay(pdMS_TO_TICKS(1000));
                xSemaphoreGive(mutex_media);
             }
                }
        else if (telas == 6){
            ESP_LOGW("TELA","TELA 6");
            char str7[128] = "";
            snprintf(str7, sizeof(str7), "Comecou  \n a \n Corrida");
            ssd1306_display_text(str7);
            vTaskDelay(pdMS_TO_TICKS(1000));
             
                }
        else if (telas == 7){
            ESP_LOGW("TELA","TELA 7");
            char str8[128] = "";
            snprintf(str8, sizeof(str8), "Terminou  \n a \n Corrida");
            ssd1306_display_text(str8);
            vTaskDelay(pdMS_TO_TICKS(1000));
             
                }
                
        */
        else{
            telas=0;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//GPS


static const char *TAG = "GPS";
#define M_PI 3.14159265358979323846

double toRadians(double value){
    double radians = (1*M_PI)/180;
    return radians*value;
}

float valor_MET(){
    float MET;
    // site: https://sites.google.com/site/compendiumofphysicalactivities/Activity-Categories/running
    // velocidades em kmh
    if (velocidade_instantanea >= 22.53076) {
        MET = 23.0; 
        return MET;
    }
    else if (velocidade_instantanea >= 20.92142 && velocidade_instantanea < 22.53076) {
        MET = 19.8;
        return MET; 
    }
    else if (velocidade_instantanea >= 19.31208 && velocidade_instantanea < 20.92142) {
        MET = 19.0;
        return MET; 
    }
    else if (velocidade_instantanea >= 17.70274 && velocidade_instantanea < 19.31208) {
        MET = 16.0;
        return MET; 
    }
    else if (velocidade_instantanea >= 16.0934 && velocidade_instantanea < 17.70274) {
        MET = 14.5;
        return MET; 
    }
    else if (velocidade_instantanea >= 14.48406 && velocidade_instantanea < 16.0934) {
        MET = 12.8;
        return MET;
    }
    else if (velocidade_instantanea >= 13.840323999999999 && velocidade_instantanea < 14.48406) {
        MET = 12.3;
        return MET; 
    }
    else if (velocidade_instantanea >= 12.87472 && velocidade_instantanea < 13.840323999999999) {
        MET = 11.8;
        return MET;
    }
    else if (velocidade_instantanea >= 12.07005 && velocidade_instantanea < 12.87472) {
        MET = 11.8;
        return MET; 
    }
    else if (velocidade_instantanea >= 11.26538 && velocidade_instantanea < 12.07005) {
        MET = 11.0;
        return MET; 
    }
    else if (velocidade_instantanea >= 10.782578 && velocidade_instantanea < 11.26538) {
        MET = 10.5;
        return MET; 
    }
    else if (velocidade_instantanea >= 9.65604 && velocidade_instantanea < 10.782578) {
        MET = 9.8; 
        return MET;
    }
    else if (velocidade_instantanea >= 8.368568 && velocidade_instantanea < 9.65604) {
        MET = 9.0; 
        return MET;
    }
    else if (velocidade_instantanea >= 8.0467 && velocidade_instantanea < 8.368568) {
        MET = 8.3; 
        return MET;
    }
    else if (velocidade_instantanea >= 6.43736 && velocidade_instantanea < 8.0467) {
        MET = 7.15;
        return MET; 
    }
    else if (velocidade_instantanea > 0 && velocidade_instantanea < 6.43736) {
        MET = 6.0; 
        return MET;
}
    else{
    return 0.0;
    }
}

float calculo_gasto_calorico(float delta_tempo) {
    float MET;
    MET = valor_MET();
    int peso = 80;
    // https://nutritotal.com.br/pro/material/met-multiplos-de-equivalentes-metabolicos/
    float calorias = MET *peso * delta_tempo/3600; //kcal, tempo estava em s transformado para h
    return calorias;   

}

void soma_das_altitudes(float altitude){
    ESP_LOGI("CALC_ALTI","Altitude: %f | Última Altitude: %f",altitude,ultima_altitude);
    float delta;
    if (ultima_altitude != 0){
        delta = altitude - ultima_altitude;
    }
    else{
        delta = 0;
    }

    ESP_LOGI("CALC_ALTI","Delta %f",delta);
    if (fabs(delta) > 0.3){
        delta_altitude = delta_altitude + delta;
    }
    ESP_LOGI("CALC_ALTI","Delta Altitude: %f",delta_altitude);
    ultima_altitude = altitude;
}

void measure_distance(coordenada antiga, coordenada atual,float altitude){
    // Convert the latitudes 
    // and longitudes
    // from degree to radians.    

    double lat1 = toRadians(antiga.latitude);
    double long1 = toRadians(antiga.longitude);
    double lat2 = toRadians(atual.latitude);
    double long2 = toRadians(atual.longitude);

    double dlong = long2 - long1;
    double dlat = lat2 - lat1;

    double distance = pow(sin(dlat / 2), 2) + 
                          cos(lat1) * cos(lat2) * 
                          pow(sin(dlong / 2), 2);

    distance = 2 * asin(sqrt(distance));
 
    // Radius of Earth in 
    // Kilometers, R = 6371
    // Use R = 3956 for miles
     
    // Calculate the result
    distance = distance * R;
    int atual_hours, atual_minutes, atual_seconds;
    sscanf(atual.time, "%2d%2d%2d", &atual_hours, &atual_minutes, &atual_seconds);
    int antiga_hours, antiga_minutes, antiga_seconds;
    sscanf(antiga.time, "%2d%2d%2d", &antiga_hours, &antiga_minutes, &antiga_seconds);
    float delta_tempo = ((atual_hours * 3600 + atual_minutes * 60 + atual_seconds)-(antiga_hours * 3600 + antiga_minutes * 60 + antiga_seconds));
    ESP_LOGW("Print_Info","Distancia: %lf | Delta_Tempo: %f",distance,delta_tempo);
    ESP_LOGW("Print_Info","Tenpo Atual: %d:%d:%d | Tempo_Anterior: %d:%d:%d",atual_hours, atual_minutes, atual_seconds,antiga_hours, antiga_minutes, antiga_seconds);


    if (xSemaphoreTake(mutex_dados, portMAX_DELAY)) {

        if (delta_tempo > 0 && distance > 0.003){
            velocidade_instantanea = (distance/delta_tempo)*3600; //km/h
            float calorias = calculo_gasto_calorico(delta_tempo);
            total_calorias = total_calorias + calorias;
        }
        soma_das_altitudes(altitude);
        total_distance = total_distance + distance;
        xSemaphoreGive(mutex_dados);
    }

    ESP_LOGI("Print_Info","Distância Total: %lf | Total de Calorias: %f | Velocidade: %f |Delta Altitude: %f",total_distance,total_calorias,velocidade_instantanea,delta_altitude);
    vTaskDelay(pdMS_TO_TICKS(1000)); // trava a tarefa por 1/2 s para não obter amostras demais
}



// Função para converter coordenadas NMEA para decimal
double convert_nmea_to_decimal(char *nmea, char direction) {
    if (strlen(nmea) < 3) {
        //ESP_LOGW(TAG, "Dado NMEA inválido: %s", nmea);
        return 0.0;
    }
    double degrees, minutes, decimal;
    degrees = (int)(atof(nmea) / 100);
    minutes = atof(nmea) - (degrees * 100);
    decimal = degrees + (minutes / 60.0);
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    //ESP_LOGD(TAG, "Convertido: %s %c -> %.6f", nmea, direction, decimal);
    return decimal;
}


// Função para processar a sentença NMEA
void process_nmea_data(char *data) {
    //ESP_LOGI(TAG, "Recebido: %s", data);
    
    if (strncmp(data, "$GPGGA", 6) == 0) {
        char time[10], lat[15], lat_dir, lon[15], lon_dir;
        int fix_quality, satellites;
        float hdop, altitude;

        int matched = sscanf(data, "$GPGGA,%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%f",
                             time, lat, &lat_dir, lon, &lon_dir, &fix_quality, &satellites, &hdop, &altitude);

        if (matched < 8) {
            //ESP_LOGW(TAG, "Sentença GPGGA incompleta: %s", data);
            return;
        }

        ESP_LOGI(TAG, "Hora UTC: %s", time);
        ESP_LOGI(TAG, "Fix: %d, Satélites: %d, HDOP: %.1f, Altitude: %.1f", fix_quality, satellites, hdop, altitude);
        
        if(xSemaphoreTake(mutex_horario, portMAX_DELAY)){
            sscanf(time, "%2d%2d%2d", &hora_atual.hora, &hora_atual.minuto, &hora_atual.segundo);
            hora_atual.hora = hora_atual.hora-3;
            ESP_LOGW(TAG, "HORARIO -> %02d:%02d:%02d",hora_atual.hora, hora_atual.minuto, hora_atual.segundo);
            xSemaphoreGive(mutex_horario);
        }
        if (fix_quality > 0) {
            if(corrida_em_andamento){
                // so faz o parse se a corrida estiver começado a corrida
                double latitude = convert_nmea_to_decimal(lat, lat_dir);
                double longitude = convert_nmea_to_decimal(lon, lon_dir);
                ESP_LOGI(TAG, "Latitude: %.6f, Longitude: %.6f", latitude, longitude);
                coordenada novaCoordenada;
                novaCoordenada.latitude = latitude;
                novaCoordenada.longitude = longitude;
                strcpy(novaCoordenada.time, time);
                coordenada antiga_coordenada;
                //ESP_LOGW(TAG, "Falha ao receber da fila 1.");
                if (!primeiro_valor) {
                    if (xQueueReceive(fila_coordenadas, &antiga_coordenada, portMAX_DELAY) == pdTRUE) {
                        measure_distance(antiga_coordenada, novaCoordenada, altitude);
                    } else {
                        ESP_LOGW(TAG, "Falha ao receber da fila 2.");
                    }
                }
                //ESP_LOGW(TAG, "Falha ao receber da fila 3.");
                if (xQueueSendToBack(fila_coordenadas, &novaCoordenada, portMAX_DELAY) != pdTRUE) {
                    ESP_LOGE(TAG, "Falha ao enviar para a fila!");
                }
                if(primeiro_valor){
                    primeiro_valor = false;
                    strcpy(start_time, time);
                }

                
            } }
        else {
            ESP_LOGW(TAG, "Sem fix de GPS...");

        }
    }
}

// Tarefa para ler dados do GPS
void gps_task(void *pvParameters) {
    ESP_LOGI(TAG, "Inicializando UART para GPS...");
    uint8_t data[BUF_SIZE];

    while (1) {
        ESP_LOGD(TAG, "Esperando dados do GPS...");
        int len = uart_read_bytes(GPS_UART, data, BUF_SIZE - 1, pdMS_TO_TICKS(1000));

        if (len > 0) {
            data[len] = '\0';  // Garante que a string termine corretamente
            //ESP_LOGI(TAG, "Bytes recebidos: %d", len);
            //ESP_LOGD(TAG, "Dados brutos: %s", data);

            // Processa cada linha da resposta
            char *token = strtok((char *)data, "\r\n");
            while (token != NULL) {
                process_nmea_data(token);
                token = strtok(NULL, "\r\n");
            }
        } else {
            ESP_LOGW(TAG, "Nenhum dado recebido do GPS!");
        }

    }
}

//Botao troca de tela
//IRAM_ATTR coloca para memoria RAM (normalmente é flash) para ter acesso mais rapido achei
static void IRAM_ATTR gpio_handler_2(void *args)
{
    BaseType_t HPTW = pdFALSE;
    xSemaphoreGiveFromISR(botao_tela, &HPTW);

    if (HPTW == pdTRUE)
    {
        portYIELD_FROM_ISR(); // Troca de contexto para rodar a task imediatamente
    }
}

// Botao comeca corrida
//IRAM_ATTR coloca para memoria RAM (normalmente é flash) para ter acesso mais rapido achei
static void IRAM_ATTR gpio_handler(void *args)
{
    BaseType_t HPTW = pdFALSE;
    xSemaphoreGiveFromISR(sem_sinc, &HPTW);

    if (HPTW == pdTRUE)
    {
        portYIELD_FROM_ISR(); // Troca de contexto para rodar a task imediatamente
    }
}


void calculo_velocidade_media(char tempo_final[10]){
    if(xSemaphoreTake(mutex_media, portMAX_DELAY)){
        int final_hours, final_minutes, final_seconds;
        sscanf(tempo_final, "%2d%2d%2d", &final_hours, &final_minutes, &final_seconds);
        int inicial_hours, inicial_minutes, inicial_seconds;
        sscanf(start_time, "%2d%2d%2d", &inicial_hours, &inicial_minutes, &inicial_seconds);
        delta_tempo_final = ((final_hours * 3600 + final_minutes * 60 + final_seconds)-(inicial_hours * 3600 + inicial_minutes * 60 + inicial_seconds));
        velocidade_media = total_distance/delta_tempo_final;
        ESP_LOGW("VELO", "VELOCIDADE MEDIA = %f",velocidade_media);
        ESP_LOGW("VELO", "TEMPO = %f",delta_tempo_final);
        xSemaphoreGive(mutex_media);
    }
    
}

void botao_start_stop_task(void *pvParameters)
{
    while (1)
    {
        ESP_LOGW("BOTAO", "ERRO 1");
        xSemaphoreTake(sem_sinc, portMAX_DELAY); // Espera até o botão ser pressionado
        ESP_LOGW("BOTAO", "ERRO 2");
        if (corrida_em_andamento){
            coordenada coordenada_final;
            if(uxQueueMessagesWaiting(fila_coordenadas) != 0){         
                if (xQueueReceive(fila_coordenadas, &coordenada_final, portMAX_DELAY) == pdTRUE) {
                    calculo_velocidade_media(coordenada_final.time);
            }}
            corrida_em_andamento = false;
            telas = 7;
            ESP_LOGW("BT", "TERMINA A CORRIDA");
        }
        else{
            // reseta os valores para se começar uma nova corrida
            xQueueReset(fila_coordenadas);
            velocidade_instantanea = 0;
            primeiro_valor = true;
            ultima_altitude = 0;
            total_distance = 0;
            total_altitude = 0;
            delta_altitude = 0;
            delta_tempo_final = 0;
            corrida_em_andamento = true;
            telas = 6;
            ESP_LOGW("BT", "COMEÇA A CORRIDA");
            
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//app main

void app_main(void)
{
    
    // Inicializaçoes
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
	i2c_master_init();
	ssd1306_init();
    adxl345_init(); 

    // Configuração do periférico
    //botao 1 -> GPS
    gpio_reset_pin(BT_IO2);
    gpio_set_direction(BT_IO2, GPIO_MODE_INPUT);
    gpio_set_intr_type(BT_IO2, GPIO_INTR_NEGEDGE); // Dispara quando pressionado
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BT_IO2, gpio_handler, NULL);

    //DHT11
    DHT11_init(DHT11_PIN);

    //botao 2 -> TELA
    gpio_reset_pin(BT_IO);
    gpio_set_direction(BT_IO, GPIO_MODE_INPUT);
    gpio_set_intr_type(BT_IO, GPIO_INTR_NEGEDGE); // Dispara quando pressionado
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BT_IO, gpio_handler_2, NULL);

    //uart
    uart_driver_install(GPS_UART, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART, &uart_config);
    uart_set_pin(GPS_UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    


    //semaforos
    xMutexi2c = xSemaphoreCreateMutex();
    xMutexdht = xSemaphoreCreateMutex();
    mutex_dados = xSemaphoreCreateMutex();
    botao_tela = xSemaphoreCreateBinary();
    sem_sinc = xSemaphoreCreateBinary();
    mutex_media = xSemaphoreCreateMutex();
    mutex_horario = xSemaphoreCreateMutex();
    //filas

    fila_coordenadas = xQueueCreate(LEN_LAT_LONG_QUEUE, sizeof(coordenada));
    if (fila_coordenadas == NULL) {
        ESP_LOGE(TAG, "Falha ao criar a fila de coordenadas!");
        return; // Encerra a aplicação se a fila não for criada
    }


    //Tasks
    ESP_LOGI(TAG, "Iniciando o sistema...");
	vTaskDelay(100/portTICK_PERIOD_MS);
	xTaskCreate(&task_ssd1306_display_text, "ssd1306_display_text", 2048, NULL, 1 , NULL);
    xTaskCreate(&task_adxl345, "task_adxl345", 2048, NULL, 1, NULL);
    xTaskCreate(&botao_cont, "contagem_botao", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskGet,"Get_temp_hum",2048,NULL,1,NULL); //dht11
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 1, NULL);
    xTaskCreate(botao_start_stop_task, "botao_start_stop_task", 4096, NULL, 1, NULL);
}