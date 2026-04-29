/**
 * @file periph_uart.h
 * @brief Comunicacion UART1 con electronica (BMS, actuadores, IMU).
 *
 * Lee datos via protocolo binario de frames y mantiene ultimo valor conocido.
 * Si no hay respuesta, retorna el ultimo dato valido recibido.
 */

#ifndef PERIPH_UART_H
#define PERIPH_UART_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Datos BMS ───────────────────────────────────────────────────── */

typedef struct {
    float    voltage;        /**< Voltaje del banco [V] */
    float    current;        /**< Corriente instantanea [A] (negativo = descarga) */
    float    capacity_ah;    /**< Capacidad residual [Ah] */
    uint8_t  soc;            /**< State of Charge [%] */
    bool     valid;          /**< true si se recibio al menos una lectura */
} bms_power_t;

typedef struct {
    uint8_t  control_status;     /**< b0=cargando, b1=descargando */
    uint16_t protection_status;  /**< Flags de proteccion (ver tabla doc) */
    uint32_t balance_status;     /**< Bit por celda: 1=balanceando */
    bool     valid;
} bms_status_t;

/* ── Estado de actuadores ────────────────────────────────────────── */

typedef enum {
    LOCK_STATE_UNKNOWN = 0,
    LOCK_STATE_LOCKED,
    LOCK_STATE_UNLOCKED
} lock_state_t;

/* ── API ─────────────────────────────────────────────────────────── */

/**
 * Inicializa UART1 para comunicacion con electronica.
 * Configura pines, baudrate, e inicia tarea de recepcion.
 *
 * @return 0 OK, -1 error
 */
int periph_uart_init(void);

/**
 * Solicita lectura periodica de SoC al BMS (MSGID 0x02).
 * @param period_sec  0=una vez, 1-250=intervalo en segundos
 * @return 0 OK, -1 error
 */
int periph_request_bms_soc(uint8_t period_sec);

/**
 * Solicita lectura periodica del Control/Protection Status (MSGID 0x05).
 * @param period_sec  0=una vez, 1-250=intervalo en segundos
 * @return 0 OK, -1 error
 */
int periph_request_bms_status(uint8_t period_sec);

/**
 * Envia comando lock al modulo de actuadores (MODULE 0x03).
 * @return 0 OK, -1 error
 */
int periph_send_lock(void);

/**
 * Envia comando unlock al modulo de actuadores (MODULE 0x03).
 * @return 0 OK, -1 error
 */
int periph_send_unlock(void);

/**
 * Retorna ultimos datos de potencia/SoC del BMS.
 * Si nunca se recibieron datos, valid==false.
 */
bms_power_t periph_get_bms_power(void);

/**
 * Retorna ultimo estado de control/proteccion del BMS.
 */
bms_status_t periph_get_bms_status(void);

/**
 * Retorna el estado actual del candado (lock/unlock).
 */
lock_state_t periph_get_lock_state(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_UART_H */
