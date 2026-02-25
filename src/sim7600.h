/**
 * @file sim7600.h
 * @brief API pública del driver SIM7600 (AT, buffer mode, TCP, comandos servidor).
 */

#ifndef SIM7600_H
#define SIM7600_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Tipos de respuesta AT
// ---------------------------------------------------------------------------
typedef enum {
    SIM7600_OK,
    SIM7600_ERROR,
    SIM7600_TIMEOUT,
    SIM7600_UNKNOWN
} sim7600_response_t;

// ---------------------------------------------------------------------------
// Máquina de estados (modo buffer)
// ---------------------------------------------------------------------------
typedef enum {
    STATE_BOOT,
    STATE_NET_UP,
    STATE_SOCKET_CONNECTING,
    STATE_RUN_IDLE,
    STATE_TX_PREPARE,
    STATE_TX_WAIT_PROMPT,
    STATE_TX_SENDING,
    STATE_TX_WAIT_RESULT,
    STATE_RX_DRAIN,
    STATE_ERROR_RECOVER
} boot_state_t;

// ---------------------------------------------------------------------------
// Comandos AT básicos
// ---------------------------------------------------------------------------
sim7600_response_t sim7600_send_command(const char *cmd, char *response, size_t response_size, int timeout_ms);
sim7600_response_t sim7600_wait_for_response(const char *expected, char *response, size_t response_size, int timeout_ms);
void sim7600_send(const char *cmd);

// ---------------------------------------------------------------------------
// PDP, CIPMODE, red
// ---------------------------------------------------------------------------
sim7600_response_t sim7600_set_pdp_context(int cid, const char *pdp_type, const char *apn,
                                          const char *pdp_addr, int d_comp, int h_comp);
sim7600_response_t sim7600_read_pdp_context(char *response, size_t response_size);
sim7600_response_t sim7600_test_pdp_context(char *response, size_t response_size);
sim7600_response_t sim7600_set_cipmode(int mode);
sim7600_response_t sim7600_read_cipmode(char *response, size_t response_size);
sim7600_response_t sim7600_netopen(void);
sim7600_response_t sim7600_netopen_status(char *response, size_t response_size);
sim7600_response_t sim7600_netclose(void);
sim7600_response_t sim7600_set_dns(const char *primary_dns, const char *secondary_dns);

// ---------------------------------------------------------------------------
// Modo buffer (AT+CIPRXGET=1)
// ---------------------------------------------------------------------------
sim7600_response_t sim7600_set_buffer_mode(int mode);
sim7600_response_t sim7600_drain_rx_buffer(int link_num);
int sim7600_get_rx_buffer_length(int link_num);
sim7600_response_t sim7600_read_rx_buffer(int link_num, int len, char *buffer, size_t buffer_size);

// ---------------------------------------------------------------------------
// TCP: abrir, cerrar, enviar, recibir
// ---------------------------------------------------------------------------
sim7600_response_t sim7600_cipopen_tcp_transparent(int link_num, const char *server_ip, int server_port);
/** Lee estado de conexiones con AT+CIPOPEN? (respuesta en response; ver doc: establecido = +CIPOPEN: n,\"TCP\",... ; no establecido = +CIPOPEN: n) */
sim7600_response_t sim7600_cipopen_read(char *response, size_t response_size);
/** Devuelve 1 si en la respuesta de AT+CIPOPEN? el link_num está establecido (tiene tipo/IP/puerto), 0 si no. */
int sim7600_cipopen_is_link_established(int link_num, const char *cipopen_response);
/** Devuelve 1 si el link está establecido Y conectado al servidor (server_ip, server_port). Usar tras cipopen_read(). */
int sim7600_cipopen_link_matches_server(int link_num, const char *cipopen_response, const char *server_ip, int server_port);
sim7600_response_t sim7600_cipopen_tcp(int link_num, const char *server_ip, int server_port);
sim7600_response_t sim7600_cipclose(int link_num);
sim7600_response_t sim7600_cipsend(int link_num, const uint8_t *data, size_t data_len);
int sim7600_read_data_non_transparent(uint8_t *buffer, size_t buffer_size, int timeout_ms);

// ---------------------------------------------------------------------------
// Comandos del servidor y ACK
// ---------------------------------------------------------------------------
void sim7600_process_server_command(const char *data, size_t len);
void sim7600_async_read_task(void *pvParameters);
sim7600_response_t sim7600_send_ack(const char *command_id, const char *command, const char *timestamp,
                                    const char *client_id, const char *request_id, int message_id);
void sim7600_command_processor_task(void *pvParameters);

// ---------------------------------------------------------------------------
// Modo transparente (opcional)
// ---------------------------------------------------------------------------
int sim7600_send_data_transparent(const uint8_t *data, size_t len);
int sim7600_read_data_transparent(uint8_t *buffer, size_t buffer_size, int timeout_ms);

// ---------------------------------------------------------------------------
// Pruebas y bucle de telemetría
// ---------------------------------------------------------------------------
void sim7600_tcp_ping_test(const char *server_ip, int server_port);
sim7600_response_t sim7600_send_scooter_update(int link_num, int scooter_id,
                                               double latitude, double longitude,
                                               int battery, float speed_kmh);
void sim7600_scooter_update_loop(const char *server_ip, int server_port);

#ifdef __cplusplus
}
#endif

#endif /* SIM7600_H */
