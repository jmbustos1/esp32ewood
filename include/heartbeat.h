#ifndef HEARTBEAT_H
#define HEARTBEAT_H

/**
 * @file heartbeat.h
 * @brief Application-level heartbeat / self-check task (Fix D, 2026-08-17).
 *
 * Objetivo: garantizar que si la telemetria deja de enviarse por CUALQUIER
 * razon (conocida o desconocida), el firmware fuerza recovery de la FSM
 * de conectividad. Cubre escenarios que los handlers de URCs no capturan
 * (NAT recycle silencioso, bug futuro que bloquee el loop, mutex tomado
 * indefinidamente, etc.).
 *
 * Comportamiento:
 *   - Task independiente en CPU 1, prioridad baja (3).
 *   - Chequea cada 15s el timestamp del ultimo CIPSEND exitoso.
 *   - Primer disparo: 90s de silencio → connectivity_notify_send_failed(true) → NET_DOWN.
 *   - Backoff: si tras el disparo no vuelve la telemetria, siguientes disparos cada 5min.
 *   - Reset: cualquier CIPSEND OK actualiza timestamp y limpia el backoff.
 *
 * NO reemplaza a los handlers de +IPCLOSE (Bug 2) ni al backoff propio de
 * la FSM — es una red de seguridad de ultimo recurso.
 */

/**
 * @brief Crea el task de heartbeat. Llamar una vez desde el setup del
 *        loop principal, despues de crear las otras tasks.
 * @return 0 si OK, -1 si error creando task. Idempotente si se llama dos veces.
 */
int heartbeat_init(void);

/**
 * @brief Notifica al heartbeat que hubo un CIPSEND exitoso.
 *        Llamar desde el path exitoso de envio de telemetria (despues de
 *        confirmar el "+CIPSEND: <link>,<req>,<cnf>" del modem).
 */
void heartbeat_notify_send_ok(void);

#endif /* HEARTBEAT_H */
