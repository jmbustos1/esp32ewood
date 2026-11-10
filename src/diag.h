/**
 * @file diag.h
 * @brief Diagnóstico del firmware enviado al backend dentro de la telemetría.
 *
 * Agrega un bloque "d":{...} al JSON de telemetría con valores que el
 * firmware ya tiene en memoria (sin comandos AT extra). Documentación de cada
 * campo: docs/DIAGNOSTICO.md. Se apaga con DIAG_TELEMETRY_ENABLED=0.
 */
#ifndef DIAG_H
#define DIAG_H

#include <stdbool.h>
#include <stddef.h>

#include "connectivity.h"

/** Captura el motivo de arranque. Llamar una vez al inicio de app_main. */
void diag_init(void);

/** La FSM de conectividad cambió de estado (puede llamarse desde CPU 0 o 1). */
void diag_on_fsm_transition(connectivity_state_t from, connectivity_state_t to);

/** El heartbeat (Fix D) forzó una recuperación. */
void diag_on_heartbeat_trigger(void);

/**
 * Escribe ,"d":{...} en buf si corresponde enviarlo en este mensaje:
 * cada DIAG_EVERY_N_MESSAGES mensajes o cuando cambió algún campo clave.
 * Retorna los bytes escritos (0 si no corresponde o no cabe).
 */
int diag_append_json(char *buf, size_t cap);

#endif /* DIAG_H */
