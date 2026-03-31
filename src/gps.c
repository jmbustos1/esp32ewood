/**
 * @file gps.c
 * @brief Modulo GPS via SIM7600 (AT+CGPS / AT+CGPSINFO).
 *
 * Usa sim7600_send_command() para enviar AT commands por UART2.
 * Mantiene internamente la ultima posicion valida; si AT+CGPSINFO
 * retorna campos vacios (sin fix), devuelve la posicion anterior.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "esp_log.h"

#include "app_config.h"
#include "sim7600.h"
#include "gps.h"

static const char *TAG = "GPS";

/** Ultima posicion conocida (persiste entre llamadas) */
static gps_position_t s_last_pos = {
    .latitude  = 0.0,
    .longitude = 0.0,
    .speed_kmh = 0.0f,
    .altitude  = 0.0f,
    .course    = 0.0f,
    .valid     = false
};

/* ── Helpers ─────────────────────────────────────────────────────── */

/**
 * Convierte formato NMEA ddmm.mmmmmm a grados decimales.
 * @param raw  Cadena "ddmm.mmmmmm" (lat) o "dddmm.mmmmmm" (lon)
 * @param deg_digits  2 para latitud, 3 para longitud
 * @return Grados decimales (siempre positivo; el signo lo pone el indicador N/S/E/W)
 */
static double nmea_to_degrees(const char *raw, int deg_digits)
{
    if (raw == NULL || raw[0] == '\0') return 0.0;

    char deg_str[4] = {0};
    strncpy(deg_str, raw, deg_digits);
    double degrees = atof(deg_str);
    double minutes = atof(raw + deg_digits);

    return degrees + minutes / 60.0;
}

/**
 * Convierte velocidad de nudos a km/h.
 */
static float knots_to_kmh(float knots)
{
    return knots * 1.852f;
}

/* ── API publica ─────────────────────────────────────────────────── */

int gps_init(void)
{
    char buf[256];

    /* Verificar estado actual del GPS */
    sim7600_response_t r = sim7600_send_command("AT+CGPS?\r\n", buf, sizeof(buf), RESPONSE_TIMEOUT_MS);
    if (r == SIM7600_OK && strstr(buf, "+CGPS: 1") != NULL) {
        ESP_LOGI(TAG, "GPS ya esta activo");
        return 0;
    }

    /* Iniciar GPS en modo standalone */
    r = sim7600_send_command("AT+CGPS=1,1\r\n", buf, sizeof(buf), RESPONSE_TIMEOUT_MS);
    if (r != SIM7600_OK) {
        ESP_LOGE(TAG, "Error iniciando GPS (AT+CGPS=1,1)");
        return -1;
    }

    ESP_LOGI(TAG, "GPS iniciado (standalone). Esperando fix...");
    return 0;
}

int gps_stop(void)
{
    char buf[128];
    sim7600_response_t r = sim7600_send_command("AT+CGPS=0\r\n", buf, sizeof(buf), RESPONSE_TIMEOUT_MS);
    if (r != SIM7600_OK) {
        ESP_LOGE(TAG, "Error deteniendo GPS");
        return -1;
    }
    ESP_LOGI(TAG, "GPS detenido");
    return 0;
}

int gps_update(void)
{
    char buf[512];

    sim7600_response_t r = sim7600_send_command("AT+CGPSINFO\r\n", buf, sizeof(buf), RESPONSE_TIMEOUT_MS);
    if (r != SIM7600_OK) {
        ESP_LOGW(TAG, "Error AT+CGPSINFO");
        return -1;
    }

    /*
     * Respuesta esperada:
     * +CGPSINFO:<lat>,<N/S>,<lon>,<E/W>,<date>,<time>,<alt>,<speed>,<course>
     *
     * Sin fix:
     * +CGPSINFO:,,,,,,,,
     */
    char *info = strstr(buf, "+CGPSINFO:");
    if (info == NULL) {
        ESP_LOGW(TAG, "Respuesta inesperada: %s", buf);
        return -1;
    }

    info += strlen("+CGPSINFO:");

    /* Saltar espacios */
    while (*info == ' ') info++;

    /* Verificar si hay fix: si el primer campo esta vacio, no hay fix */
    if (*info == ',' || *info == '\r' || *info == '\n' || *info == '\0') {
        ESP_LOGD(TAG, "Sin fix GPS (campos vacios)");
        return 1; /* Sin fix, posicion anterior se mantiene */
    }

    /* Parsear campos separados por coma:
     * lat,N/S,lon,E/W,date,time,alt,speed,course */
    char lat_str[20] = {0};
    char ns[4] = {0};
    char lon_str[20] = {0};
    char ew[4] = {0};
    char date_str[12] = {0};
    char time_str[12] = {0};
    char alt_str[12] = {0};
    char speed_str[12] = {0};
    char course_str[12] = {0};

    int parsed = sscanf(info, "%19[^,],%3[^,],%19[^,],%3[^,],%11[^,],%11[^,],%11[^,],%11[^,],%11[^,\r\n]",
                        lat_str, ns, lon_str, ew, date_str, time_str, alt_str, speed_str, course_str);

    if (parsed < 4) {
        ESP_LOGW(TAG, "Parse incompleto (%d campos): %s", parsed, info);
        return 1;
    }

    /* Convertir lat/lon de NMEA a grados decimales */
    double lat = nmea_to_degrees(lat_str, 2);  /* ddmm.mmmmmm */
    double lon = nmea_to_degrees(lon_str, 3);  /* dddmm.mmmmmm */

    /* Aplicar signo segun indicador */
    if (ns[0] == 'S') lat = -lat;
    if (ew[0] == 'W') lon = -lon;

    /* Velocidad: el campo viene en nudos */
    float speed_knots = (speed_str[0] != '\0') ? atof(speed_str) : 0.0f;
    float alt = (alt_str[0] != '\0') ? atof(alt_str) : 0.0f;
    float course = (course_str[0] != '\0') ? atof(course_str) : 0.0f;

    /* Validacion basica de coordenadas */
    if (fabs(lat) > 90.0 || fabs(lon) > 180.0) {
        ESP_LOGW(TAG, "Coordenadas fuera de rango: %.6f, %.6f", lat, lon);
        return 1;
    }

    /* Actualizar posicion */
    s_last_pos.latitude  = lat;
    s_last_pos.longitude = lon;
    s_last_pos.speed_kmh = knots_to_kmh(speed_knots);
    s_last_pos.altitude  = alt;
    s_last_pos.course    = course;
    s_last_pos.valid     = true;

    ESP_LOGI(TAG, "Fix: %.6f, %.6f | %.1f km/h | alt %.1f m",
             lat, lon, s_last_pos.speed_kmh, alt);

    return 0;
}

gps_position_t gps_get_position(void)
{
    return s_last_pos;
}
