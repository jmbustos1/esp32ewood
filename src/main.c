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

static const char *TAG = "MAIN";

// Buffer local para pruebas iniciales (el módulo sim7600 usa el suyo internamente)
static char response_buf[BUF_SIZE];

void app_main(void)
{
    sim7600_response_t result;
    
    // Configuración UART
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

    ESP_LOGI(TAG, "UART inicializada, esperando SIM7600...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 1. Prueba básica de comunicación
    ESP_LOGI(TAG, "=== Prueba de comunicación ===");
    result = sim7600_send_command("AT\r\n", response_buf, sizeof(response_buf), RESPONSE_TIMEOUT_MS);
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "✓ SIM7600 responde correctamente");
    } else {
        ESP_LOGE(TAG, "✗ Error comunicándose con SIM7600");
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. Verificar valores soportados para AT+CGDCONT
    ESP_LOGI(TAG, "=== Verificando valores soportados AT+CGDCONT ===");
    result = sim7600_test_pdp_context(response_buf, sizeof(response_buf));
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "Valores soportados recibidos");
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 3. Configurar contexto PDP
    // NOTA: Ajusta el APN según tu operador (ej: "internet", "claro.com", "movistar.es", etc.)
    ESP_LOGI(TAG, "=== Configurando contexto PDP ===");
    result = sim7600_set_pdp_context(1, "IP", "bam.entelpcs.cl", NULL, 0, 0);
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "✓ Contexto PDP configurado correctamente");
    } else {
        ESP_LOGE(TAG, "✗ Error configurando contexto PDP");
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 4. Leer configuración del contexto PDP
    ESP_LOGI(TAG, "=== Leyendo configuración de contexto PDP ===");
    result = sim7600_read_pdp_context(response_buf, sizeof(response_buf));
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "Configuración actual:");
        // La respuesta contiene +CGDCONT: 1,"IP","internet",...
        ESP_LOGI(TAG, "%s", response_buf);
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 5. Iniciar servicio de actualizaciones del scooter
    // Envía actualizaciones periódicas con coordenadas alternadas
    ESP_LOGI(TAG, "=== Iniciando servicio de scooter ===");
    sim7600_scooter_update_loop(SCOOTER_TCP_HOST, SCOOTER_TCP_PORT);
    
    // Nota: sim7600_scooter_update_loop() tiene un loop infinito,
    // por lo que este código nunca se alcanzará
    ESP_LOGI(TAG, "=== Servicio detenido ===");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Sistema en espera...");
    }
}

