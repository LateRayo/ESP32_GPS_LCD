#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "rom/ets_sys.h"  // ets_delay_us
#include "driver/uart.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "../GPSModule/GPDModule.h"

#define RETRY_MAX 3

#define I2C_RETRY(cmd_func) ({ \
    esp_err_t __err; \
    int __retry = 0; \
    do { \
        __err = (cmd_func); \
        if (__err != ESP_OK) ESP_LOGW(TAG, "I2C retry #%d: %s", __retry + 1, esp_err_to_name(__err)); \
    } while (__err != ESP_OK && ++__retry < RETRY_MAX); \
    __err; \
})


// I2C configuration
#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_SCL_IO     22
#define I2C_MASTER_SDA_IO     21
#define I2C_MASTER_FREQ_HZ    100000
#define I2C_MASTER_TX_BUF_LEN 0
#define I2C_MASTER_RX_BUF_LEN 0

// LCD I2C address and control bits
#define I2C_LCD_ADDR    0x27  // Cambia si tu módulo usa otra dirección
#define LCD_BACKLIGHT   0x08
#define LCD_ENABLE_BIT  0x04
#define LCD_RW_BIT      0x02
#define LCD_RS_BIT      0x01

uint8_t cont = 0;

static const char *TAG = "i2c_lcd";

// Inicializa el bus I2C
static esp_err_t i2c_master_init(void) 
{
    i2c_config_t conf = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);

    if (err != ESP_OK) return err;

    return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                              I2C_MASTER_RX_BUF_LEN,
                              I2C_MASTER_TX_BUF_LEN, 0);
}

// Escribe un byte al expansor del LCD
/*static esp_err_t lcd_write_byte(uint8_t data) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}*/

static esp_err_t lcd_write_byte(uint8_t data) 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = I2C_RETRY(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100)));

    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed after retries: %s", esp_err_to_name(ret));
    }

    return ret;
}
/*
// Temporización del pulso E
static void lcd_pulse_enable(uint8_t data) 
{
    // Enable high
    lcd_write_byte(data | LCD_ENABLE_BIT);
    ets_delay_us(1);
    // Enable low
    lcd_write_byte(data & ~LCD_ENABLE_BIT);
    ets_delay_us(50);
}*/

static esp_err_t lcd_pulse_enable(uint8_t data) 
{
    esp_err_t err;

    err = lcd_write_byte(data | LCD_ENABLE_BIT);
    if (err != ESP_OK) return err;

    ets_delay_us(1);

    err = lcd_write_byte(data & ~LCD_ENABLE_BIT);
    if (err != ESP_OK) return err;

    ets_delay_us(50);

    return ESP_OK;
}


// Envía media instrucción (nibble)
/*static void lcd_send_nibble(uint8_t nibble, uint8_t control) 
{
    uint8_t data = (nibble << 4) | control | LCD_BACKLIGHT;
    lcd_pulse_enable(data);
}*/
static esp_err_t lcd_send_nibble(uint8_t nibble, uint8_t control) 
{
    uint8_t data = (nibble << 4) | control | LCD_BACKLIGHT;
    return lcd_pulse_enable(data);
}
/*
// Envía comando completo (8 bits)
static void lcd_send_command(uint8_t cmd) 
{
    lcd_send_nibble((cmd >> 4) & 0x0F, 0);
    lcd_send_nibble(cmd & 0x0F,    0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

// Envía dato al LCD (carácter)
static void lcd_send_data(uint8_t data) 
{
    lcd_send_nibble((data >> 4) & 0x0F, LCD_RS_BIT);
    lcd_send_nibble(data & 0x0F,       LCD_RS_BIT);
    vTaskDelay(pdMS_TO_TICKS(2));
}*/

static esp_err_t lcd_send_command(uint8_t cmd) 
{
    esp_err_t err;
    err = lcd_send_nibble((cmd >> 4) & 0x0F, 0);
    if (err != ESP_OK) return err;

    err = lcd_send_nibble(cmd & 0x0F, 0);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

static esp_err_t lcd_send_data(uint8_t data) 
{
    esp_err_t err;
    err = lcd_send_nibble((data >> 4) & 0x0F, LCD_RS_BIT);
    if (err != ESP_OK) return err;

    err = lcd_send_nibble(data & 0x0F, LCD_RS_BIT);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

// Inicializa el LCD en 4-bit mode
void lcd_init(void) 
{
    ESP_ERROR_CHECK(i2c_master_init());
    vTaskDelay(pdMS_TO_TICKS(50));  // >40 ms tras power on

    // Secuencia especial para forzar 4-bit
    lcd_send_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));   // >4.1 ms
    lcd_send_nibble(0x03, 0);
    ets_delay_us(150);              // >100 µs
    lcd_send_nibble(0x03, 0);
    ets_delay_us(150);              // >100 µs
    lcd_send_nibble(0x02, 0);       // Modo 4 bits
    vTaskDelay(pdMS_TO_TICKS(2));

    // Configuración estándar
    lcd_send_command(0x28);  // 4-bit, 2 líneas, 5x8 puntos
    lcd_send_command(0x01);  // Clear display
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_command(0x06);  // Entry mode: increment
    lcd_send_command(0x0C);  // Display ON, cursor OFF, blink OFF
}

// Escribe string en la pantalla
void lcd_write_string(const char *str) 
{
    while (*str) 
    {
        lcd_send_data((uint8_t)(*str));
        str++;
    }
}

// Borra toda la pantalla y sitúa el cursor en (0,0)
void lcd_clear(void) 
{
    lcd_send_command(0x01);           // Comando “Clear Display”
    vTaskDelay(pdMS_TO_TICKS(2));     // >1.53 ms según datasheet
}

// Devuelve el cursor a la posición de origen sin borrar el texto
void lcd_home(void) 
{
    lcd_send_command(0x02);           // Comando “Return Home”
    vTaskDelay(pdMS_TO_TICKS(2));     // >1.53 ms
}

// Sitúa el cursor en la columna col (0–15) y fila row (0 o 1)
void lcd_set_cursor(uint8_t col, uint8_t row) 
{
    uint8_t addr = (row == 0 ? 0x80 : 0xC0) + (col & 0x0F);
    lcd_send_command(addr);           // DDRAM address = base + offset
    // No hace falta delay extra, lcd_send_command ya hace ~2 ms
}

// Control de backlight mediante reenvío de un byte “dummy” con el bit de backlight
static uint8_t backlight_state = LCD_BACKLIGHT;

// Enciende la luz de fondo
void lcd_backlight_on(void) {
    backlight_state = LCD_BACKLIGHT;
    lcd_write_byte(backlight_state);
}

// Apaga la luz de fondo
void lcd_backlight_off(void) {
    backlight_state = 0x00;
    lcd_write_byte(backlight_state);
}

// Borra una línea completa (0 ó 1) y deja el cursor al inicio de esa línea
void lcd_clear_line(uint8_t row) {
    lcd_set_cursor(0, row);           // Mueve cursor al inicio de línea
    for (uint8_t i = 0; i < 16; i++) {
        lcd_send_data(' ');           // Escribe 16 espacios
    }
    lcd_set_cursor(0, row);           // Vuelve a poner cursor en el inicio
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------


void double_a_string(double valor, char *buf, size_t buf_len) 
{
    //memset(&data, 0, sizeof(data));
    int len = snprintf(buf, buf_len, "%.7f", valor);
    if (len < 0 || (size_t)len >= buf_len) {
        // error o truncamiento
    }
}

void int_a_string(int valor, char *buf, size_t buf_len) 
{
    //memset(&data, 0, sizeof(data));
    int len = snprintf(buf, buf_len, "%d", valor);
    if (len < 0) {
        // Error al formatear
    } else if ((size_t)len >= buf_len) {
        // Truncamiento: buf no fue lo bastante grande
    }
}

static void gps_task(void *arg)
{
    char sentence[NMEA_MAX_LEN];
    while (1) {
        if (read_nmea_sentence(sentence, sizeof(sentence))) {
            ESP_LOGI(TAG1, "NMEA: %s", sentence);
            if (strncmp(sentence, "$GPGGA", 6) == 0) {
                parse_gpgga(sentence, &data);
            } else if (strncmp(sentence, "$GPRMC", 6) == 0) {
                parse_gprmc(sentence, &data);
            }
        }
        // pequeña espera para ceder CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//----------------------------------------------------------------
//-----------------------------------------------------------------

#include "driver/gpio.h"
#include "esp_log.h"

#define BUTTON_GPIO    GPIO_NUM_4
#define PAGES 5
static const char *TAG2 = "button";

// Llamar desde app_main() o init de tu módulo
void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,   // máscara del pin
        .mode         = GPIO_MODE_INPUT,       // modo entrada
        .pull_up_en   = GPIO_PULLUP_ENABLE,    // pull-up interno
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // sin pull-down
        .intr_type    = GPIO_INTR_DISABLE      // sin interrupciones
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG2, "Botón en GPIO%d configurado como entrada con pull-up", BUTTON_GPIO);
}

// Leer el estado en cualquier parte
bool button_is_pressed(void)
{
    // gpio_get_level devuelve 0 o 1
    // con pull-up: 0 = pulsado, 1 = suelto
    return gpio_get_level(BUTTON_GPIO) == 0;
}

#define DEBOUNCE_DELAY_MS  50  // tiempo de “rebote” típico

void button_task(void *arg)
{
    bool last_stable = gpio_get_level(BUTTON_GPIO);  // nivel inicial
    bool last_read   = last_stable;
    TickType_t last_time = xTaskGetTickCount();

    while (1) {
        bool level = gpio_get_level(BUTTON_GPIO);
        if (level != last_read) {
            // hay cambio, reseteamos temporizador de debounce
            last_time = xTaskGetTickCount();
            last_read = level;
        } else {
            // si lleva estable DEBOUNCE_DELAY_MS, lo tomamos como cambio real
            if ((xTaskGetTickCount() - last_time) > pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)
                && last_read != last_stable)
            {
                last_stable = last_read;
                if (last_stable == 0) {
                    // flanco de alto→bajo: botón PRESIONADO
                    ESP_LOGI(TAG2, "Botón PRESIONADO");
                    cont = (cont + 1) % PAGES;
                } else {
                    // flanco de bajo→alto: botón LIBERADO
                    ESP_LOGI(TAG2, "Botón LIBERADO");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void init(void)
{
    button_init();
    gps_uart_init(9600);// El NEO‑6M viene por defecto a 9600 bps
    lcd_init(); 
}

// Ejemplo de aplicación
void app_main(void) 
{
    init();
    char buf[32];
    memset(&data, 0, sizeof(data));
    uint8_t contAX = -1;

    xTaskCreate(gps_task, "gps_task", 4*1024, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(button_task, "button_task", 2*1024, NULL, tskIDLE_PRIORITY+1, NULL);
    
    

    while (1) 
    {   
        switch (cont)
        {
        case 0: // latitud, longitud
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home(); 
            double_a_string(data.latitude, buf, sizeof(buf));
            lcd_write_string("lat:");
            lcd_write_string(buf);
            lcd_send_command(0xC0); // muevo a la segunda lina
            lcd_write_string("lon:");
            double_a_string(data.longitude, buf, sizeof(buf));
            lcd_write_string(buf);
            break;
        case 1: // fehca
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            int_a_string(data.year,buf,sizeof(buf));
            lcd_write_string(buf);
            lcd_write_string("/");
            int_a_string(data.month,buf,sizeof(buf));
            lcd_write_string(buf);
            lcd_write_string("/");
            int_a_string(data.day,buf,sizeof(buf));
            lcd_write_string(buf);
            break;
        case 2:
            if (contAX != cont || data.second == 0)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("hora: ");
            if (data.hour >= 0 && data.hour <= 3 )
            {
                data.hour += 12;
            }
            
            int_a_string(data.hour-3,buf,sizeof(buf));
            lcd_write_string(buf);
            lcd_write_string(":");
            int_a_string(data.minute,buf,sizeof(buf));
            lcd_write_string(buf);
            lcd_write_string(":");
            int_a_string(data.second,buf,sizeof(buf));
            lcd_write_string(buf);
            //lcd_send_command(0xC0); // muevo a la segunda lina            
            break;

        case 3:
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("altitud:"); //error +-20,30
            int_a_string(data.altitude,buf,sizeof(buf));
            lcd_write_string(buf);
            lcd_write_string("m");
            /*El GPS toma como referencia una superficie matemática ideal llamada 
            elipsoide WGS-84, que es como un globo ligeramente achatado en los polos. 
            Este elipsoide no es el suelo real ni el nivel del mar, sino un modelo idealizado de la forma de la Tierra.*/
            break;
        
        case 4:
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("satelites:");
            int_a_string(data.num_sats,buf,sizeof(buf));
            lcd_write_string(buf);
            break;

        /*case 5:
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("HDOP:");
            double_a_string(data.hdop, buf, sizeof(buf));
            lcd_write_string(buf);
            break;*/

        /*case 6:
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("PDOP:");
            double_a_string(data.pdop, buf, sizeof(buf));
            lcd_write_string(buf);
            break;*/

        /*case 7:
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("VDOP:");
            double_a_string(data.vdop, buf, sizeof(buf));
            lcd_write_string(buf);
            break;*/

        /*case 8:
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("PRN[0-1]:");
            int_a_string(data.sats_prn[0], buf, sizeof(buf));
            lcd_write_string(buf);
            lcd_write_string(" ");
            int_a_string(data.sats_prn[1], buf, sizeof(buf));
            lcd_write_string(buf);
            break;*/

        /*case 9:
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("SNR[0-1]:");
            double_a_string(data.snr[0], buf, sizeof(buf));
            lcd_write_string(buf);
            lcd_write_string(" ");
            double_a_string(data.snr[1], buf, sizeof(buf));
            lcd_write_string(buf);
            break;*/

        /*case 10:
            if (contAX != cont)
            {
                lcd_clear();
                contAX = cont;
            }
            lcd_home();
            lcd_write_string("Fix:");
            if (data.num_sats >= 4)
                lcd_write_string("3D");
            else if (data.num_sats >= 3)
                lcd_write_string("2D");
            else
                lcd_write_string("NoFix");
            break;*/

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}