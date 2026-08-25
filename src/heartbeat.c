/**
 * @file heartbeat.c
 * @brief Implementacion del heartbeat / self-check task (Fix D, 2026-08-17).
 */

#include "heartbeat.h"

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "connectivity.h"

static const char *TAG = "HEARTBEAT";

/* Threshold antes del primer disparo: 90 s.
 * Elegido con margen sobre el timeout del backend (60 s) para dar tiempo
 * a que Bug 2 (+IPCLOSE handler) y el backoff normal de la FSM actuen
 * primero. Solo interviene si esos otros mecanismos fallan. */
#define HB_INITIAL_THRESHOLD_US   (90LL * 1000 * 1000)

/* Backoff entre disparos si sigue sin CIPSEND OK: 5 min.
 * Evita spamear recovery si la reconexion no logra en el primer intento
 * (ej. backend abajo mucho tiempo, modem realmente congelado). */
#define HB_BACKOFF_US             (300LL * 1000 * 1000)

/* Intervalo de chequeo del task. 15 s da granularidad suficiente sin
 * consumir recursos innecesarios. */
#define HB_CHECK_INTERVAL_MS      15000

static volatile int64_t s_last_send_ok_us = 0;
static volatile int64_t s_last_trigger_us = 0;
static bool s_initialized = false;

static void heartbeat_task(void *pv)
{
    ESP_LOGI(TAG, "Task iniciado (threshold=%llds, backoff=%llds, check=%dms)",
             (long long)(HB_INITIAL_THRESHOLD_US / 1000000),
             (long long)(HB_BACKOFF_US / 1000000),
             HB_CHECK_INTERVAL_MS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HB_CHECK_INTERVAL_MS));

        int64_t now = esp_timer_get_time();

        /* Si nunca hubo un CIPSEND OK, no hacemos nada. La FSM esta en
         * su primer setup (o en recovery inicial post-reboot); no queremos
         * disparar recovery adicional durante el arranque normal. */
        if (s_last_send_ok_us == 0) continue;

        int64_t silence_us = now - s_last_send_ok_us;

        /* Todavia dentro del threshold — telemetria fluyendo OK. */
        if (silence_us < HB_INITIAL_THRESHOLD_US) continue;

        /* Ya disparamos recientemente — esperar backoff antes de otro disparo. */
        if (s_last_trigger_us > 0 &&
            (now - s_last_trigger_us) < HB_BACKOFF_US) {
            continue;
        }

        /* Disparar recovery. NET_DOWN es mas profundo que SOCKET_DOWN:
         * re-hace PDP + PAP + NETOPEN + CIPCLOSE + CIPOPEN. Es lo que
         * queremos si el firmware quedo zombi porque un simple re-CIPOPEN
         * no basto. */
        ESP_LOGE(TAG, "🚨 Heartbeat expiro: %llds sin CIPSEND OK. Forzando recovery (NET_DOWN).",
                 (long long)(silence_us / 1000000));
        s_last_trigger_us = now;
        connectivity_notify_send_failed(true);
    }
}

int heartbeat_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "heartbeat_init llamado dos veces; ignorando");
        return 0;
    }

    /* Pinneado a CPU 1 para no compartir CPU 0 con periph_rx_task y la
     * FSM principal. Prioridad baja (3) — no necesita reaccionar rapido,
     * solo garantizar que corre eventualmente. */
    BaseType_t r = xTaskCreatePinnedToCore(
        heartbeat_task, "heartbeat", 3072, NULL, 3, NULL, 1);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Error creando heartbeat task");
        return -1;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "Task heartbeat creado en CPU 1 (prioridad 3)");
    return 0;
}

void heartbeat_notify_send_ok(void)
{
    s_last_send_ok_us = esp_timer_get_time();
    /* Al recibir OK, limpiamos el "ultimo trigger" para que el proximo
     * silencio prolongado dispare inmediato a los 90s sin esperar
     * el backoff completo del ciclo anterior. */
    s_last_trigger_us = 0;
}
