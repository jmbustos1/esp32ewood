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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"
#include "sim7600.h"
#include "gps.h"

static const char *TAG = "GPS";

/* Watchdog del motor GNSS del SIM7600.
 *
 * Bug real de campo (scooter #1 de Bruno, 2026-08-22 -> 2026-09-04):
 * el motor GNSS del SIM7600 puede apagarse solo (reset interno del bloque
 * GPS, glitch por bajo voltaje transitorio, etc.) sin que el resto del
 * modem se caiga. La FSM de connectivity no se entera (el TCP sigue vivo)
 * y Fix F nunca se dispara. Sin este watchdog el firmware reportaba la
 * ultima coord conocida por 13 dias seguidos sin ningun log de alarma.
 *
 * GPS_NOFIX_RECYCLE_STREAK: tras N updates consecutivos sin fix, forzamos
 * AT+CGPS=0/1 para recuperar el motor GNSS. Con UPDATE_INTERVAL_SEC=5,
 * N=30 => ~2.5 min antes de reciclar (tolera oclusiones cortas).
 *
 * GPS_FIX_STALE_MS: edad maxima del ultimo fix antes de reportar
 * pos.valid=false. Sin esto, gps_get_position devuelve la coord vieja
 * indefinidamente y el backend cree que es actual. */
#define GPS_NOFIX_RECYCLE_STREAK 30
#define GPS_FIX_STALE_MS         (150 * 1000LL)

/** Ultima posicion conocida (persiste entre llamadas) */
static gps_position_t s_last_pos = {
    .latitude  = 0.0,
    .longitude = 0.0,
    .speed_kmh = 0.0f,
    .altitude  = 0.0f,
    .course    = 0.0f,
    .valid     = false
};

static int s_nofix_streak = 0;
static int64_t s_last_fix_ms = 0;
static int s_recycle_count = 0;

static void gps_recycle_engine(void)
{
    char buf[128];
    s_recycle_count++;
    ESP_LOGW(TAG, "Reciclando motor GNSS (SIM7600) tras %d updates sin fix (recycle #%d)",
             s_nofix_streak, s_recycle_count);
    sim7600_send_command("AT+CGPS=0\r\n", buf, sizeof(buf), RESPONSE_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(2000));
    sim7600_response_t r = sim7600_send_command("AT+CGPS=1,1\r\n", buf, sizeof(buf), RESPONSE_TIMEOUT_MS);
    if (r == SIM7600_OK) {
        ESP_LOGI(TAG, "Motor GNSS reciclado; esperando fix nuevo (TTFF puede tardar 30-90s)");
    } else {
        ESP_LOGE(TAG, "Falla reciclando GNSS (AT+CGPS=1,1) — se reintentara en el proximo streak");
    }
    s_nofix_streak = 0;
}

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
        s_nofix_streak++;
        ESP_LOGW(TAG, "Sin fix GPS (streak=%d/%d)", s_nofix_streak, GPS_NOFIX_RECYCLE_STREAK);
        if (s_nofix_streak >= GPS_NOFIX_RECYCLE_STREAK) {
            gps_recycle_engine();
        }
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
    s_nofix_streak       = 0;
    s_last_fix_ms        = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Fix: %.6f, %.6f | %.1f km/h | alt %.1f m",
             lat, lon, s_last_pos.speed_kmh, alt);

    return 0;
}

gps_position_t gps_get_position(void)
{
    /* Invalida el fix si envejecio. Sin esto, el firmware reporta la
     * ultima coord conocida como si fuera actual y desde el backend no
     * se puede distinguir un scooter con GPS activo quieto de uno con
     * GNSS colgado (fue el caso del scooter #1 de Bruno). */
    if (s_last_pos.valid && s_last_fix_ms > 0) {
        int64_t age_ms = (esp_timer_get_time() / 1000) - s_last_fix_ms;
        if (age_ms > GPS_FIX_STALE_MS) {
            ESP_LOGW(TAG, "Ultimo fix envejecido (%lld ms) -> marcando invalido", age_ms);
            s_last_pos.valid = false;
        }
    }
    return s_last_pos;
}
