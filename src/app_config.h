/**
 * @file app_config.h
 * @brief Configuración centralizada del firmware (UART, pines, scooter, servidor).
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "driver/uart.h"
#include "driver/gpio.h"

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
// ---------------------------------------------------------------------------
#define SCOOTER_ID        1
#define SCOOTER_BATTERY    85
#define SCOOTER_SPEED_KMH 15.5f
#define UPDATE_INTERVAL_SEC 5
#define RX_POLL_MS        150

// ---------------------------------------------------------------------------
// Servidor TCP
// Si la instancia (p. ej. EC2) cambia de IP al hacer stop/start, usa una
// IP elástica o un hostname (DNS); si usas hostname, asegura DNS en el módulo.
// ---------------------------------------------------------------------------
#define SCOOTER_TCP_HOST   "98.92.176.224"
#define SCOOTER_TCP_PORT   8201

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
// Ubicaciones (Concepción, Chile)
// ---------------------------------------------------------------------------
typedef struct {
    double latitude;
    double longitude;
} location_t;

#endif /* APP_CONFIG_H */
