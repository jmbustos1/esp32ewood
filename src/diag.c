/**
 * @file diag.c
 * @brief Diagnóstico del firmware para el backend. Ver docs/DIAGNOSTICO.md.
 *
 * Solo lee valores en memoria: no envía comandos AT ni toma el UART, así no
 * interfiere con la FSM ni con el TCP. Los contadores se tocan desde CPU 0
 * (FSM) y CPU 1 (heartbeat), por eso se usan operaciones atómicas.
 */
#include "diag.h"

#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "app_config.h"
#include "gps.h"
#include "periph_uart.h"

static int s_reset_reason = 0;

/* Estado de la FSM (escrito desde ambos núcleos). */
static volatile int s_fsm_state = CONNECTIVITY_MODULE_OFFLINE;
static volatile int s_fsm_prev = CONNECTIVITY_MODULE_OFFLINE;
static volatile int64_t s_fsm_since_us = 0;
static volatile uint32_t s_fsm_transitions = 0;

static volatile uint32_t s_hb_triggers = 0;

/* Envío: contador de mensajes y firma de lo último enviado. */
static uint32_t s_msg_count = 0;
static bool s_sent_once = false;
static int s_last_sig[7];

void diag_init(void)
{
    s_reset_reason = (int)esp_reset_reason();
    s_fsm_since_us = esp_timer_get_time();
}

void diag_on_fsm_transition(connectivity_state_t from, connectivity_state_t to)
{
    s_fsm_prev = (int)from;
    s_fsm_state = (int)to;
    s_fsm_since_us = esp_timer_get_time();
    __atomic_add_fetch(&s_fsm_transitions, 1, __ATOMIC_RELAXED);
}

void diag_on_heartbeat_trigger(void)
{
    __atomic_add_fetch(&s_hb_triggers, 1, __ATOMIC_RELAXED);
}

int diag_append_json(char *buf, size_t cap)
{
#if !DIAG_TELEMETRY_ENABLED
    (void)buf; (void)cap;
    return 0;
#else
    s_msg_count++;

    gps_position_t pos = gps_get_position();
    bms_power_t bms = periph_get_bms_power();
    int64_t fix_age_ms = gps_get_fix_age_ms();

    int st = s_fsm_state;
    int sig[7] = {
        st,
        (int)__atomic_load_n(&s_fsm_transitions, __ATOMIC_RELAXED),
        (int)__atomic_load_n(&s_hb_triggers, __ATOMIC_RELAXED),
        gps_get_recycle_count(),
        pos.valid ? 1 : 0,
        bms.valid ? 1 : 0,
        s_reset_reason,
    };

    bool changed = !s_sent_once || memcmp(sig, s_last_sig, sizeof(sig)) != 0;
    bool periodic = (s_msg_count % DIAG_EVERY_N_MESSAGES) == 0;
    if (!changed && !periodic) {
        return 0;
    }

    const esp_app_desc_t *app = esp_app_get_description();
    int64_t now_us = esp_timer_get_time();

    int n = snprintf(buf, cap,
                     ",\"d\":{\"fw\":\"%.16s\",\"up\":%lld,\"rr\":%d,"
                     "\"st\":%d,\"ps\":%d,\"ts\":%lld,\"tr\":%d,\"hb\":%d,"
                     "\"gv\":%d,\"fa\":%lld,\"gr\":%d,\"bv\":%d}",
                     app->version,
                     (long long)(now_us / 1000000),
                     s_reset_reason,
                     st, (int)s_fsm_prev,
                     (long long)((now_us - s_fsm_since_us) / 1000000),
                     sig[1], sig[2],
                     sig[4],
                     (long long)(fix_age_ms < 0 ? -1 : fix_age_ms / 1000),
                     sig[3], sig[5]);

    if (n <= 0 || (size_t)n >= cap) {
        return 0; /* No cabe: mejor no enviar diagnóstico que romper el JSON */
    }

    memcpy(s_last_sig, sig, sizeof(sig));
    s_sent_once = true;
    return n;
#endif
}
