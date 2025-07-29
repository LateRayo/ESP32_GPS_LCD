// esp32_gps_driver.c
// Driver GPS para ESP32 con parsing de GPGGA, GPRMC, GPGSA y GPGSV

#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// inicializar uart
#define GPS_UART_NUM      UART_NUM_1
#define GPS_UART_TX_PIN   GPIO_NUM_17
#define GPS_UART_RX_PIN   GPIO_NUM_16
#define GPS_BUF_SIZE      (1024)

static const char *TAG1 = "gps_uart";

void gps_uart_init(int baud_rate)
{
    uart_config_t uart_config = {
        .baud_rate  = baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_LOGI(TAG1, "UART%d initialized at %dbps", GPS_UART_NUM, baud_rate);
}

#define NMEA_MAX_LEN 128

// Lee una sentencia NMEA de hasta NMEA_MAX_LEN caracteres
static bool read_nmea_sentence(char *out_buf, size_t max_len)
{
    static char line[NMEA_MAX_LEN];
    static size_t idx = 0;
    uint8_t byte;
    int len;

    while ((len = uart_read_bytes(GPS_UART_NUM, &byte, 1, pdMS_TO_TICKS(100))) > 0) {
        if (byte == '\r' || byte == '\n') {
            if (idx > 0) {
                line[idx] = '\0';
                strncpy(out_buf, line, max_len);
                idx = 0;
                return true;
            }
        } else if (idx < sizeof(line)-1) {
            line[idx++] = byte;
        }
    }
    return false;
}

// Estructura para guardar datos
typedef struct {
    double latitude;
    double longitude;
    double altitude;
    int    num_sats;
    double hdop;
    double pdop;
    double vdop;
    int    day, month, year;
    int    hour, minute, second;
    int    sats_prn[12];
    double snr[12];
} gps_data_t;

static gps_data_t data;

// Convierte de “ddmm.mmmm” + hemisferio a grados decimales
static double parse_deg(const char *dm, char hemi)
{
    double val = atof(dm);
    double deg = floor(val/100.0);
    double min = val - deg*100.0;
    double dd  = deg + min/60.0;
    return (hemi=='S' || hemi=='W') ? -dd : dd;
}

static void parse_gpgga(const char *s, gps_data_t *d)
{
    char *tok, buf[NMEA_MAX_LEN];
    strcpy(buf, s);
    tok = strtok(buf, ",");            // $GPGGA
    tok = strtok(NULL, ",");           // time
    if (tok) {
        int hh, mm; double ss;
        sscanf(tok, "%2d%2d%lf", &hh, &mm, &ss);
        d->hour = hh; d->minute = mm; d->second = (int)ss;
    }
    tok = strtok(NULL, ",");           // lat
    char lat_s[16] = {0}; if (tok) strncpy(lat_s, tok, sizeof(lat_s)-1);
    tok = strtok(NULL, ",");           // N/S
    char ns = tok ? tok[0] : 'N';
    tok = strtok(NULL, ",");           // lon
    char lon_s[16] = {0}; if (tok) strncpy(lon_s, tok, sizeof(lon_s)-1);
    tok = strtok(NULL, ",");           // E/W
    char ew = tok ? tok[0] : 'E';
    d->latitude  = parse_deg(lat_s, ns);
    d->longitude = parse_deg(lon_s, ew);
    tok = strtok(NULL, ",");           // fix
    tok = strtok(NULL, ",");           // num sats
    d->num_sats = tok ? atoi(tok) : 0;
    tok = strtok(NULL, ",");           // HDOP
    d->hdop = tok ? atof(tok) : 0.0;
    tok = strtok(NULL, ",");           // altitude
    d->altitude = tok ? atof(tok) : 0.0;
}

static void parse_gprmc(const char *s, gps_data_t *d)
{
    char *tok, buf[NMEA_MAX_LEN];
    strcpy(buf, s);
    tok = strtok(buf, ",");            // $GPRMC
    tok = strtok(NULL, ",");           // time
    tok = strtok(NULL, ",");           // status A/V
    if (!tok || tok[0] != 'A') return;   // sólo procesar si es válido
    // saltar campos hasta date
    for (int i = 0; i < 6; i++) tok = strtok(NULL, ",");
    // tok ahora es ddmmyy*CC
    if (tok) {
        char date_str[7] = {0};
        // copiar sólo los primeros 6 caracteres (ddmmyy)
        strncpy(date_str, tok, 6);
        int dd = (date_str[0]-'0')*10 + (date_str[1]-'0');
        int mm = (date_str[2]-'0')*10 + (date_str[3]-'0');
        int yy = (date_str[4]-'0')*10 + (date_str[5]-'0');
        d->day   = dd;
        d->month = mm;
        d->year  = 2000 + yy;
    }
}

static void parse_gpgsa(const char *s, gps_data_t *d)
{
    char *tok, buf[NMEA_MAX_LEN];
    strcpy(buf, s);
    tok = strtok(buf, ",");             // $GPGSA
    strtok(NULL, ",");                  // mode1
    strtok(NULL, ",");                  // mode2
    for (int i = 0; i < 12; i++) strtok(NULL, ","); // satélites usados
    tok = strtok(NULL, ",");            // PDOP
    d->pdop = tok ? atof(tok) : 0.0;
    tok = strtok(NULL, ",");            // HDOP
    d->hdop = tok ? atof(tok) : d->hdop;
    tok = strtok(NULL, ",");            // VDOP
    d->vdop = tok ? atof(tok) : 0.0;
}

static void parse_gpgsv(const char *s, gps_data_t *d)
{
    char *tok, buf[NMEA_MAX_LEN];
    strcpy(buf, s);
    strtok(buf, ",");                   // $GPGSV
    int total_msgs = atoi(strtok(NULL, ","));
    int msg_num    = atoi(strtok(NULL, ","));
    strtok(NULL, ",");                  // sats_in_view
    int sat_idx = (msg_num - 1) * 4;
    for (int i = 0; i < 4 && sat_idx + i < 12; i++) {
        tok = strtok(NULL, ",");         // PRN
        if (!tok) break;
        d->sats_prn[sat_idx + i] = atoi(tok);
        strtok(NULL, ",");               // elev
        strtok(NULL, ",");               // azim
        tok = strtok(NULL, ",");         // SNR
        d->snr[sat_idx + i] = tok ? atof(tok) : 0.0;
    }
}

void process_gps(void)
{
    char sentence[NMEA_MAX_LEN];

    if (!read_nmea_sentence(sentence, sizeof(sentence))) {
        return;
    }

    if (strncmp(sentence, "$GPGGA", 6) == 0) {
        parse_gpgga(sentence, &data);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0) {
        parse_gprmc(sentence, &data);
    } else if (strncmp(sentence, "$GPGSA", 6) == 0) {
        parse_gpgsa(sentence, &data);
    } else if (strncmp(sentence, "$GPGSV", 6) == 0) {
        parse_gpgsv(sentence, &data);
    }
}

// Al final, data.day, data.month, data.year estarán bien parseados.
