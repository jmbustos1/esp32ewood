#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"
#include "sim7600.h"
#include "gps.h"
#include "periph_uart.h"

static const char *TAG = "MAIN";

// Buffer local para pruebas iniciales (el módulo sim7600 usa el suyo internamente)
static char response_buf[BUF_SIZE];

void app_main(void)
{
    sim7600_response_t result;

    // Configuración UART SIM7600
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_SIM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_SIM, &uart_config);
    uart_set_pin(UART_SIM, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART SIM7600 inicializada, esperando modulo...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // ── Inicializar UART electronica (BMS, actuadores) ──────────────
    ESP_LOGI(TAG, "=== Inicializando UART electronica ===");
    if (periph_uart_init() == 0) {
        ESP_LOGI(TAG, "UART electronica OK");

        // Solicitar SoC periodico al BMS
        periph_request_bms_soc(BMS_SOC_POLL_PERIOD_SEC);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Solicitar estado de control/proteccion periodico
        periph_request_bms_status(BMS_STATUS_POLL_PERIOD_SEC);
    } else {
        ESP_LOGW(TAG, "UART electronica no disponible, usando valores por defecto");
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // ── Prueba basica de comunicacion SIM7600 ───────────────────────
    ESP_LOGI(TAG, "=== Prueba de comunicacion ===");
    result = sim7600_send_command("AT\r\n", response_buf, sizeof(response_buf), RESPONSE_TIMEOUT_MS);
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "SIM7600 responde correctamente");
    } else {
        ESP_LOGE(TAG, "Error comunicandose con SIM7600");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // ── Iniciar GPS lo antes posible (TTFF puede tomar tiempo) ──────
    ESP_LOGI(TAG, "=== Iniciando GPS ===");
    if (gps_init() == 0) {
        ESP_LOGI(TAG, "GPS iniciado, esperando fix en background...");
    } else {
        ESP_LOGW(TAG, "GPS no disponible, se usara ultima posicion conocida");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // ── Configurar contexto PDP (APN M2M con autenticacion) ─────────
    ESP_LOGI(TAG, "=== Configurando contexto PDP ===");
    result = sim7600_set_pdp_context(1, "IP", APN_NAME, NULL, 0, 0);
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "APN configurado: %s", APN_NAME);
    } else {
        ESP_LOGE(TAG, "Error configurando APN");
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // Autenticacion PAP (requerida por SIM M2M Entel)
    result = sim7600_set_auth(1, 1, APN_USER, APN_PASS);
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "Autenticacion PAP configurada (user=%s)", APN_USER);
    } else {
        ESP_LOGW(TAG, "Error configurando autenticacion PAP");
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // Leer configuracion del contexto PDP
    result = sim7600_read_pdp_context(response_buf, sizeof(response_buf));
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "Configuracion PDP: %s", response_buf);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    // ── Iniciar servicio de actualizaciones ──────────────────────────
    ESP_LOGI(TAG, "=== Iniciando servicio de scooter ===");
    sim7600_scooter_update_loop(SCOOTER_TCP_HOST, SCOOTER_TCP_PORT);

    // Nota: sim7600_scooter_update_loop() tiene un loop infinito
    ESP_LOGI(TAG, "=== Servicio detenido ===");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Sistema en espera...");
    }
}
