/**
 * @file app_config.h
 * @brief Configuración centralizada del firmware (UART, pines, scooter, servidor).
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "driver/uart.h"
#include "driver/gpio.h"

/* Valores sensibles (SCOOTER_ID, APN_*, SCOOTER_TCP_*, DEVICE_SECRET) viven
 * en config_local.h (gitignored). Copiar config_local.example.h a
 * config_local.h y llenar con valores reales antes de compilar. */
#include "config_local.h"

// ---------------------------------------------------------------------------
// UART módulo SIM7600
// ---------------------------------------------------------------------------
#define UART_SIM          UART_NUM_2
#define UART_TX           GPIO_NUM_17
#define UART_RX           GPIO_NUM_16
#define BUF_SIZE          1024
#define RESPONSE_TIMEOUT_MS 5000

// ---------------------------------------------------------------------------
// Modo buffer (AT+CIPRXGET=1)
// ---------------------------------------------------------------------------
#define MAX_LINKS         10

// ---------------------------------------------------------------------------
// Configuración del scooter
// NOTA: SCOOTER_ID vive en config_local.h (sensible, unico por dispositivo).
// ---------------------------------------------------------------------------
#define UPDATE_INTERVAL_SEC 5
#define RX_POLL_MS        150
#define MIN_BATTERY_FOR_TELEMETRY 0  /* SoC minimo para enviar (0 = siempre) */

// ---------------------------------------------------------------------------
// UART electronica (BMS, actuadores) — UART1 remapeada
// ---------------------------------------------------------------------------
#define PERIPH_UART_NUM     UART_NUM_1
#define PERIPH_TX_PIN       GPIO_NUM_4
#define PERIPH_RX_PIN       GPIO_NUM_5
#define PERIPH_BAUD_RATE    115200

// ---------------------------------------------------------------------------
// BMS: intervalo de solicitud periodica
// ---------------------------------------------------------------------------
#define BMS_SOC_POLL_PERIOD_SEC     1   /* Solicitar SoC cada N segundos */
#define BMS_STATUS_POLL_PERIOD_SEC  10  /* Solicitar control status cada N segundos */

// ---------------------------------------------------------------------------
// Servidor TCP + APN + DEVICE_SECRET viven en config_local.h (sensibles).
// Si el backend cambia de IP al reiniciar, usar IP elastica o hostname (DNS).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Conectividad y backoff (máquina de fallas)
// ---------------------------------------------------------------------------
#define CONNECTIVITY_AT_RETRY_MS        30000   /* T1: intervalo entre intentos AT (módulo offline) */
#define CONNECTIVITY_NET_RETRY_MS       30000   /* T2: intervalo entre intentos NETOPEN */
#define CONNECTIVITY_SOCKET_BACKOFF_LEN  5     /* pasos de backoff: 10s, 30s, 1min, 2min, 5min (tope) */
#define CONNECTIVITY_AT_TIMEOUT_COUNT   3       /* N timeouts AT seguidos → MODULE_OFFLINE */
#define CONNECTIVITY_NET_CHECK_CYCLES  10      /* Cada N ciclos en RUNNING, comprobar netopen_status */

// ---------------------------------------------------------------------------
// Comandos del servidor
// ---------------------------------------------------------------------------
#define COMMAND_QUEUE_SIZE      20
#define MAX_PROCESSED_REQUESTS  20
#define PENDING_ACK_QUEUE_SIZE 10

// ---------------------------------------------------------------------------
// Debug parser de frames UART1 (electronica/BMS)
// Apagar (0) en produccion para no inundar el UART debug.
// ---------------------------------------------------------------------------
#define FRAME_DEBUG_BYTES        0  /* log byte a byte cuando entra al parser */
#define FRAME_DEBUG_FIELDS       0  /* log campo a campo al decodificar BMS */
#define FRAME_DEBUG_TX           0  /* hex dump de frames enviados */
#define FRAME_ACCEPT_BAD_CHKS    0  /* procesar frame aunque falle checksum (DEBUG ONLY) */

#endif /* APP_CONFIG_H */
