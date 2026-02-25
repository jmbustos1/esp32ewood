/**
 * @file connectivity.h
 * @brief Máquina de estados de conectividad (módulo, red, socket) y recuperación con backoff.
 */

#ifndef CONNECTIVITY_H
#define CONNECTIVITY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Estados de la capa de conectividad.
 * MODULE_OFFLINE → MODULE_READY → (setup) → NET_READY o RUNNING.
 * NET_DOWN → NET_READY → (cipopen) → RUNNING.
 * SOCKET_DOWN → (cipopen con backoff) → RUNNING.
 */
typedef enum {
    CONNECTIVITY_MODULE_OFFLINE,   /**< Módulo no responde por UART. Reintentar AT cada T1. */
    CONNECTIVITY_MODULE_READY,     /**< AT OK; falta configurar red/socket. */
    CONNECTIVITY_NET_DOWN,        /**< Red no abierta. Reintentar NETOPEN cada T2. */
    CONNECTIVITY_NET_READY,       /**< Red abierta; falta socket o se cayó. */
    CONNECTIVITY_SOCKET_DOWN,     /**< Socket cerrado. Reintentar solo CIPOPEN con backoff. */
    CONNECTIVITY_RUNNING          /**< Red + socket OK; loop normal. */
} connectivity_state_t;

/**
 * Inicializa el estado a MODULE_OFFLINE y resetea contadores de backoff.
 * Llamar al entrar en sim7600_scooter_update_loop.
 */
void connectivity_init(void);

/**
 * Devuelve el estado actual.
 */
connectivity_state_t connectivity_get_state(void);

/**
 * Fija el estado (p. ej. RUNNING tras setup inicial en sim7600_scooter_update_loop).
 */
void connectivity_set_state(connectivity_state_t state);

/**
 * Ejecuta un paso de recuperación según el estado actual.
 * - MODULE_OFFLINE: intenta AT; si OK pasa a MODULE_READY y ejecuta setup idempotente.
 * - MODULE_READY: ejecuta setup idempotente (CIPMODE, buffer, NETOPEN, DNS, CIPOPEN).
 * - NET_DOWN: intenta NETOPEN; si OK pasa a NET_READY e intenta CIPOPEN.
 * - NET_READY: intenta CIPOPEN.
 * - SOCKET_DOWN: intenta CIPOPEN (respetando backoff).
 * - RUNNING: no hace nada (el loop hace drain/send).
 * @param server_ip IP o dominio del servidor
 * @param server_port Puerto del servidor
 */
void connectivity_step_recovery(const char *server_ip, int server_port);

/**
 * Espera el tiempo de backoff correspondiente al estado actual
 * (T1 para MODULE_OFFLINE, T2 para NET_DOWN, paso actual para SOCKET_DOWN).
 */
void connectivity_wait_backoff(void);

/**
 * Notificar fallo al enviar telemetría.
 * Transiciona a NET_DOWN (error de red) o SOCKET_DOWN (cierre/error TCP).
 * @param is_network_error true si el fallo indica red cerrada (ej. netopen_status); false para socket/backend.
 */
void connectivity_notify_send_failed(bool is_network_error);

/**
 * Opcional: notificar resultado de un AT de health-check.
 * Si se reciben CONNECTIVITY_AT_TIMEOUT_COUNT timeouts seguidos, pasar a MODULE_OFFLINE.
 * (Puede implementarse en una segunda fase.)
 */
void connectivity_notify_at_timeout(void);
void connectivity_notify_at_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* CONNECTIVITY_H */
