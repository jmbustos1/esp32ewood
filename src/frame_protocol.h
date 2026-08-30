/**
 * @file frame_protocol.h
 * @brief Protocolo binario de comunicacion con electronica (BMS, actuadores, IMU).
 *
 * Estructura de frame:
 *   SOF (2B: 0xA5 0x5A) | LENGTH (2B BE) | TYPE (1B) | PAYLOAD (NB) | CHECKSUM (2B Fletcher-16)
 *
 * LENGTH = cantidad de bytes desde TYPE hasta fin de PAYLOAD (inclusive).
 * CHECKSUM = Fletcher-16 calculado desde LENGTH hasta fin de PAYLOAD.
 */

#ifndef FRAME_PROTOCOL_H
#define FRAME_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── SOF ─────────────────────────────────────────────────────────── */
#define FRAME_SOF_0     0xA5
#define FRAME_SOF_1     0x5A

/* ── TYPE ────────────────────────────────────────────────────────── */
#define FRAME_TYPE_WRITE    0x01
#define FRAME_TYPE_READ     0x02
#define FRAME_TYPE_ANS      0x03
#define FRAME_TYPE_EVENT    0x04
#define FRAME_TYPE_ACK      0x05
#define FRAME_TYPE_ERROR    0x06

/* ── MODULE ID ───────────────────────────────────────────────────── */
#define MODULE_BMS          0x01
#define MODULE_ACCEL        0x02
#define MODULE_ACTUATORS    0x03
#define MODULE_SYSTEM       0xFF

/* ── BMS MSGID ───────────────────────────────────────────────────── */
#define BMS_MSGID_PRODUCT_INFO      0x01
#define BMS_MSGID_VOLTAGE_SOC       0x02
#define BMS_MSGID_TEMPERATURE       0x03
#define BMS_MSGID_CELL_VOLTAGES     0x04
#define BMS_MSGID_CONTROL_STATUS    0x05

/* ── SYSTEM MSGID ────────────────────────────────────────────────── */
#define SYS_MSGID_BOOT          0x01
#define SYS_MSGID_RUN           0x02
#define SYS_MSGID_FAULT         0x03
#define SYS_MSGID_KEEP_ALIVE    0x04
/* MSGID 0x06 SYS STATUS (2026-08-30): frame consolidado del hub de sensores.
 * Reemplaza al MODULE_BMS/MSGID 0x02 (VOLTAGE_SOC) — el hub ya NO responde
 * a ese comando. Layout del payload (9 bytes):
 *   [0-1]  voltaje  (uint16 BE, x10 mV)
 *   [2-3]  corriente (int16 BE, x10 mA)
 *   [4-5]  capacidad residual (uint16 BE, x10 mAh)
 *   [6]    SoC (uint8, %)
 *   [7]    sistema activo (0x0F=inactivo, 0xFF=activo)
 *   [8]    Locker (0x0F=cerrado, 0xFF=abierto)
 * Por default el hub emite este frame cada 10s espontaneamente; se puede
 * cambiar el period enviando un READ con MODULE=0xFF MSGID=0x06 Period=N. */
#define SYS_MSGID_STATUS        0x06

/* Valores del byte Locker en SYS_MSGID_STATUS */
#define SYS_LOCKER_CLOSED       0x0F
#define SYS_LOCKER_OPEN         0xFF

/* Valores del byte Sistema activo en SYS_MSGID_STATUS */
#define SYS_ACTIVE_INACTIVE     0x0F
#define SYS_ACTIVE_ACTIVE       0xFF

/* ── Tamanos ─────────────────────────────────────────────────────── */
#define FRAME_SOF_SIZE      2
#define FRAME_LENGTH_SIZE   2
#define FRAME_CHECKSUM_SIZE 2
#define FRAME_HEADER_SIZE   (FRAME_SOF_SIZE + FRAME_LENGTH_SIZE)  /* 4 bytes antes de TYPE */
#define FRAME_OVERHEAD      (FRAME_HEADER_SIZE + FRAME_CHECKSUM_SIZE)  /* 6 bytes total overhead */
#define FRAME_MAX_PAYLOAD   256
#define FRAME_MAX_SIZE      (FRAME_OVERHEAD + 1 + FRAME_MAX_PAYLOAD)  /* TYPE + PAYLOAD max */

/* ── Frame parseado ──────────────────────────────────────────────── */
typedef struct {
    uint8_t  type;
    uint8_t  module_id;
    uint8_t  msg_id;
    uint8_t  period;
    uint8_t  data[FRAME_MAX_PAYLOAD];
    uint16_t data_len;     /**< Bytes en data[] (PAYLOAD sin module_id, msg_id, period) */
    uint16_t length;       /**< Campo LENGTH original del frame */
} frame_parsed_t;

/* ── Parser incremental (maquina de estados) ─────────────────────── */
typedef enum {
    FPARSE_WAIT_SOF0,
    FPARSE_WAIT_SOF1,
    FPARSE_WAIT_LEN_HI,
    FPARSE_WAIT_LEN_LO,
    FPARSE_WAIT_BODY,      /**< TYPE + PAYLOAD */
    FPARSE_WAIT_CHKS_HI,
    FPARSE_WAIT_CHKS_LO
} frame_parser_state_t;

typedef struct {
    frame_parser_state_t state;
    uint16_t length;        /**< LENGTH leido del frame */
    uint16_t body_idx;      /**< Bytes recibidos de body (TYPE+PAYLOAD) */
    uint8_t  body[1 + FRAME_MAX_PAYLOAD];  /**< TYPE + PAYLOAD */
    uint8_t  len_hi;
    uint16_t chks_received;
} frame_parser_t;

/**
 * Inicializa (o resetea) el parser.
 */
void frame_parser_init(frame_parser_t *p);

/**
 * Alimenta un byte al parser.
 * @return true si se completo un frame valido (leer con frame_parser_get_result)
 */
bool frame_parser_feed(frame_parser_t *p, uint8_t byte);

/**
 * Obtiene el ultimo frame parseado. Solo valido inmediatamente despues de que
 * frame_parser_feed() retorne true.
 */
frame_parsed_t frame_parser_get_result(const frame_parser_t *p);

/* ── Construccion de frames ──────────────────────────────────────── */

/**
 * Construye un frame completo listo para enviar por UART.
 *
 * @param type      FRAME_TYPE_READ, FRAME_TYPE_WRITE, etc.
 * @param payload   Datos del payload (MODULE_ID + MSG)
 * @param payload_len  Tamano del payload
 * @param out_buf   Buffer de salida (debe tener al menos payload_len + FRAME_OVERHEAD + 1 bytes)
 * @param out_size  Tamano del buffer de salida
 * @return Cantidad de bytes escritos en out_buf, o -1 si error
 */
int frame_build(uint8_t type, const uint8_t *payload, uint16_t payload_len,
                uint8_t *out_buf, size_t out_size);

/**
 * Construye un frame READ para solicitar datos de un modulo.
 *
 * @param module_id MODULE_BMS, MODULE_ACCEL, etc.
 * @param msg_id    MSGID del mensaje solicitado
 * @param period    0 = una vez, 1-250 = segundos entre envios
 * @param out_buf   Buffer de salida
 * @param out_size  Tamano del buffer
 * @return Cantidad de bytes escritos, o -1 si error
 */
int frame_build_read(uint8_t module_id, uint8_t msg_id, uint8_t period,
                     uint8_t *out_buf, size_t out_size);

/* ── Checksum ────────────────────────────────────────────────────── */

/**
 * Calcula Fletcher-16 sobre un bloque de datos.
 * Se calcula desde LENGTH (2 bytes) hasta el ultimo byte de PAYLOAD.
 */
uint16_t fletcher16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_PROTOCOL_H */
