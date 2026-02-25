/**
 * @file connectivity.c
 * @brief Implementación de la máquina de estados de conectividad y backoff.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"
#include "connectivity.h"
#include "sim7600.h"

static const char *TAG = "CONNECTIVITY";

static connectivity_state_t s_state = CONNECTIVITY_MODULE_OFFLINE;
static int s_socket_backoff_index = 0;
static int s_at_timeout_count = 0;

/** Backoff para SOCKET_DOWN: 10s, 30s, 1min, 2min, 5min (ms). */
static const unsigned int s_socket_backoff_ms[CONNECTIVITY_SOCKET_BACKOFF_LEN] = {
    10000, 30000, 60000, 120000, 300000
};

static void run_full_setup(const char *server_ip, int server_port)
{
    sim7600_response_t r;

    ESP_LOGI(TAG, "[Setup] CIPMODE(0)...");
    r = sim7600_set_cipmode(0);
    if (r != SIM7600_OK) {
        ESP_LOGW(TAG, "[Setup] CIPMODE no OK, continuando");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "[Setup] Buffer mode(1)...");
    r = sim7600_set_buffer_mode(1);
    if (r != SIM7600_OK) {
        ESP_LOGW(TAG, "[Setup] Buffer mode no OK, continuando");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "[Setup] NETOPEN...");
    r = sim7600_netopen();
    if (r != SIM7600_OK) {
        ESP_LOGE(TAG, "[Setup] NETOPEN falló → NET_DOWN");
        s_state = CONNECTIVITY_NET_DOWN;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "[Setup] DNS...");
    sim7600_set_dns("8.8.8.8", "8.8.4.4");
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Cerrar link 0 si estaba abierto para dejar estado limpio */
    sim7600_cipclose(0);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "[Setup] CIPOPEN...");
    r = sim7600_cipopen_tcp(0, server_ip, server_port);
    if (r != SIM7600_OK) {
        ESP_LOGE(TAG, "[Setup] CIPOPEN falló → SOCKET_DOWN");
        s_state = CONNECTIVITY_SOCKET_DOWN;
        s_socket_backoff_index = 0;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    s_state = CONNECTIVITY_RUNNING;
    s_socket_backoff_index = 0;
    ESP_LOGI(TAG, "[Setup] OK → RUNNING");
}

void connectivity_init(void)
{
    s_state = CONNECTIVITY_MODULE_OFFLINE;
    s_socket_backoff_index = 0;
    s_at_timeout_count = 0;
    ESP_LOGI(TAG, "Estado inicial: MODULE_OFFLINE (backoff reseteado)");
}

connectivity_state_t connectivity_get_state(void)
{
    return s_state;
}

void connectivity_set_state(connectivity_state_t state)
{
    s_state = state;
    if (state == CONNECTIVITY_RUNNING) {
        s_socket_backoff_index = 0;
    }
}

void connectivity_step_recovery(const char *server_ip, int server_port)
{
    static char at_buf[BUF_SIZE];
    sim7600_response_t r;

    switch (s_state) {
    case CONNECTIVITY_MODULE_OFFLINE: {
        r = sim7600_send_command("AT\r\n", at_buf, sizeof(at_buf), RESPONSE_TIMEOUT_MS);
        if (r == SIM7600_TIMEOUT || r == SIM7600_ERROR) {
            ESP_LOGW(TAG, "AT sin respuesta, seguir en MODULE_OFFLINE");
            return;
        }
        ESP_LOGI(TAG, "AT OK → MODULE_READY, ejecutando setup completo");
        s_state = CONNECTIVITY_MODULE_READY;
        s_at_timeout_count = 0;
        run_full_setup(server_ip, server_port);
        break;
    }
    case CONNECTIVITY_MODULE_READY:
        run_full_setup(server_ip, server_port);
        break;

    case CONNECTIVITY_NET_DOWN:
        ESP_LOGI(TAG, "Reintento NETOPEN...");
        r = sim7600_netopen();
        if (r != SIM7600_OK) {
            ESP_LOGW(TAG, "NETOPEN falló, seguir en NET_DOWN");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
        s_state = CONNECTIVITY_NET_READY;
        ESP_LOGI(TAG, "NETOPEN OK → NET_READY");
        /* Intentar CIPOPEN en el mismo paso */
        sim7600_cipclose(0);
        vTaskDelay(pdMS_TO_TICKS(500));
        r = sim7600_cipopen_tcp(0, server_ip, server_port);
        if (r == SIM7600_OK) {
            s_state = CONNECTIVITY_RUNNING;
            s_socket_backoff_index = 0;
            ESP_LOGI(TAG, "CIPOPEN OK → RUNNING");
        } else {
            s_state = CONNECTIVITY_SOCKET_DOWN;
            s_socket_backoff_index = 0;
            ESP_LOGI(TAG, "CIPOPEN falló → SOCKET_DOWN");
        }
        break;

    case CONNECTIVITY_NET_READY:
        sim7600_cipclose(0);
        vTaskDelay(pdMS_TO_TICKS(500));
        r = sim7600_cipopen_tcp(0, server_ip, server_port);
        if (r == SIM7600_OK) {
            s_state = CONNECTIVITY_RUNNING;
            s_socket_backoff_index = 0;
            ESP_LOGI(TAG, "CIPOPEN OK → RUNNING");
        } else {
            s_state = CONNECTIVITY_SOCKET_DOWN;
            s_socket_backoff_index = 0;
            ESP_LOGI(TAG, "CIPOPEN falló → SOCKET_DOWN");
        }
        break;

    case CONNECTIVITY_SOCKET_DOWN:
        sim7600_cipclose(0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        r = sim7600_cipopen_tcp(0, server_ip, server_port);
        if (r == SIM7600_OK) {
            s_state = CONNECTIVITY_RUNNING;
            s_socket_backoff_index = 0;
            ESP_LOGI(TAG, "CIPOPEN OK → RUNNING");
        } else {
            ESP_LOGW(TAG, "CIPOPEN falló, backoff índice %d", s_socket_backoff_index);
        }
        break;

    case CONNECTIVITY_RUNNING:
        /* Nada; el loop hace drain/send */
        break;
    }
}

void connectivity_wait_backoff(void)
{
    unsigned int ms = 1000; /* por defecto 1 s para estados sin backoff largo */

    switch (s_state) {
    case CONNECTIVITY_MODULE_OFFLINE:
        ms = CONNECTIVITY_AT_RETRY_MS;
        break;
    case CONNECTIVITY_NET_DOWN:
        ms = CONNECTIVITY_NET_RETRY_MS;
        break;
    case CONNECTIVITY_SOCKET_DOWN:
        if (s_socket_backoff_index < CONNECTIVITY_SOCKET_BACKOFF_LEN) {
            ms = s_socket_backoff_ms[s_socket_backoff_index];
            if (s_socket_backoff_index < CONNECTIVITY_SOCKET_BACKOFF_LEN - 1) {
                s_socket_backoff_index++;
            }
        } else {
            ms = s_socket_backoff_ms[CONNECTIVITY_SOCKET_BACKOFF_LEN - 1];
        }
        ESP_LOGI(TAG, "Esperando backoff %u ms (índice %d)", (unsigned)ms, s_socket_backoff_index);
        break;
    default:
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void connectivity_notify_send_failed(bool is_network_error)
{
    if (is_network_error) {
        s_state = CONNECTIVITY_NET_DOWN;
        ESP_LOGI(TAG, "Fallo envío (red) → NET_DOWN");
    } else {
        s_state = CONNECTIVITY_SOCKET_DOWN;
        s_socket_backoff_index = 0;
        ESP_LOGI(TAG, "Fallo envío (socket) → SOCKET_DOWN");
    }
}

void connectivity_notify_at_timeout(void)
{
    s_at_timeout_count++;
    if (s_at_timeout_count >= CONNECTIVITY_AT_TIMEOUT_COUNT) {
        s_state = CONNECTIVITY_MODULE_OFFLINE;
        s_at_timeout_count = 0;
        ESP_LOGI(TAG, "%d timeouts AT → MODULE_OFFLINE", CONNECTIVITY_AT_TIMEOUT_COUNT);
    }
}

void connectivity_notify_at_ok(void)
{
    s_at_timeout_count = 0;
}
