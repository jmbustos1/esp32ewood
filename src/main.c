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

static const char *TAG = "SIM7600";

// UART a usar
#define UART_SIM UART_NUM_2

// Pines (ajusta si usas otros)
#define UART_TX GPIO_NUM_17
#define UART_RX GPIO_NUM_16

#define BUF_SIZE 1024
#define RESPONSE_TIMEOUT_MS 5000

// Buffers estáticos compartidos para evitar stack overflow
static char shared_response_buffer[BUF_SIZE];
static uint8_t shared_data_buffer[BUF_SIZE];

// Mutex para proteger acceso al UART durante operaciones críticas
static SemaphoreHandle_t uart_mutex = NULL;

// ============================================================================
// Máquina de Estados para Modo Buffer (AT+CIPRXGET=1)
// ============================================================================

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

// Estado actual del sistema
static boot_state_t current_state = STATE_BOOT;

// Tracking de RX por link (máximo 10 links)
#define MAX_LINKS 10
static bool rx_pending[MAX_LINKS] = {false};
static int rest_len[MAX_LINKS] = {0};

// Flag para modo buffer activo
static bool buffer_mode_active = false;

// Configuración del scooter (hardcodeada)
#define SCOOTER_ID 1
#define SCOOTER_BATTERY 85
#define SCOOTER_SPEED_KMH 15.5f
#define UPDATE_INTERVAL_SEC 5
#define RX_POLL_MS 150  /* Intervalo para revisar RX durante la espera entre actualizaciones */

// Servidor TCP
#define SCOOTER_TCP_HOST ""
#define SCOOTER_TCP_PORT 8201

// Coordenadas alternadas (Concepción, Chile)
typedef struct {
    double latitude;
    double longitude;
} location_t;

static const location_t locations[2] = {
    {-36.8201, -73.0444},  // Ubicación 1: Centro de Concepción
    {-36.8300, -73.0500}   // Ubicación 2: Cerca del centro
};

// Tipos de respuesta
typedef enum {
    SIM7600_OK,
    SIM7600_ERROR,
    SIM7600_TIMEOUT,
    SIM7600_UNKNOWN
} sim7600_response_t;

// Declaraciones forward (prototipos)
sim7600_response_t sim7600_read_cipmode(char *response, size_t response_size);
sim7600_response_t sim7600_netclose(void);
sim7600_response_t sim7600_netopen_status(char *response, size_t response_size);
void sim7600_async_read_task(void *pvParameters);
void sim7600_process_server_command(const char *data, size_t len);
void sim7600_command_processor_task(void *pvParameters);
sim7600_response_t sim7600_send_ack(const char *command_id, const char *command, const char *timestamp, 
                                    const char *client_id, const char *request_id, int message_id);
// Nuevas funciones para modo buffer
sim7600_response_t sim7600_set_buffer_mode(int mode);
sim7600_response_t sim7600_drain_rx_buffer(int link_num);
int sim7600_get_rx_buffer_length(int link_num);
sim7600_response_t sim7600_read_rx_buffer(int link_num, int len, char *buffer, size_t buffer_size);

// Función para enviar comando y leer respuesta
// CRÍTICO: Toda operación AT debe estar protegida con uart_mutex para evitar intercalación
sim7600_response_t sim7600_send_command(const char *cmd, char *response, size_t response_size, int timeout_ms)
{
    // Proteger toda la operación AT con mutex
    if (uart_mutex != NULL) {
        xSemaphoreTake(uart_mutex, portMAX_DELAY);
    }
    
    // NO limpiar buffer aquí - puede borrar URCs importantes
    // En modo buffer, los datos están en buffer interno, no en UART
    // Solo limpiar en casos muy específicos (boot, inicialización)
    
    // Enviar comando
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
    
    if (response == NULL || response_size == 0) {
        if (uart_mutex != NULL) {
            xSemaphoreGive(uart_mutex);
        }
        return SIM7600_UNKNOWN;
    }
    
    memset(response, 0, response_size);
    
    // Leer respuesta con timeout
    int total_len = 0;
    int64_t start_time = esp_timer_get_time() / 1000; // ms
    
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
        int len = uart_read_bytes(UART_SIM, (uint8_t *)response + total_len, 
                                   response_size - total_len - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            total_len += len;
            response[total_len] = '\0';
            
            // Verificar si recibimos OK o ERROR
            // Buscar OK/ERROR en líneas completas (no en medio de otros datos)
            char *ok_pos = strstr(response, "OK");
            char *error_pos = strstr(response, "ERROR");
            
            // Verificar que OK/ERROR estén en líneas separadas (precedidos por \r\n o al inicio)
            if (ok_pos != NULL) {
                // Verificar que OK esté en línea separada
                if (ok_pos == response || ok_pos[-1] == '\n' || ok_pos[-1] == '\r') {
                    ESP_LOGI(TAG, "<< %s", response);
                    if (uart_mutex != NULL) {
                        xSemaphoreGive(uart_mutex);
                    }
                    return SIM7600_OK;
                }
            }
            if (error_pos != NULL) {
                // Verificar que ERROR esté en línea separada
                if (error_pos == response || error_pos[-1] == '\n' || error_pos[-1] == '\r') {
                    ESP_LOGE(TAG, "<< %s", response);
                    if (uart_mutex != NULL) {
                        xSemaphoreGive(uart_mutex);
                    }
                    return SIM7600_ERROR;
                }
            }
        }
        
        // Pequeña pausa para no saturar CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGW(TAG, "<< Timeout - Respuesta parcial: %s", response);
    if (uart_mutex != NULL) {
        xSemaphoreGive(uart_mutex);
    }
    return SIM7600_TIMEOUT;
}

// Función mejorada para leer respuestas especiales (CONNECT, CLOSED, +NETOPEN, etc.)
// CRÍTICO: Debe estar protegida con uart_mutex (normalmente se llama desde funciones que ya tienen el mutex)
sim7600_response_t sim7600_wait_for_response(const char *expected, char *response, size_t response_size, int timeout_ms)
{
    // NOTA: Esta función asume que el mutex ya está tomado por el llamador
    // No tomamos mutex aquí para evitar deadlock si se llama desde funciones que ya lo tienen
    
    memset(response, 0, response_size);
    
    int total_len = 0;
    int64_t start_time = esp_timer_get_time() / 1000; // ms
    
    // Si buscamos el prompt ">", usar búsqueda más robusta
    bool is_prompt_search = (expected && strcmp(expected, ">") == 0);
    
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
        int len = uart_read_bytes(UART_SIM, (uint8_t *)response + total_len, 
                                   response_size - total_len - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            total_len += len;
            response[total_len] = '\0';
            
            // Para el prompt ">", buscar de forma más robusta:
            // El prompt ">" debe venir después de "AT+CIPSEND=X,Y" y antes de cualquier dato del servidor
            if (is_prompt_search) {
                // Buscar posición de "AT+CIPSEND" en el buffer
                char *cipsend_pos = strstr(response, "AT+CIPSEND");
                
                if (cipsend_pos != NULL) {
                    // Buscar el final de la línea AT+CIPSEND (buscar \n o \r)
                    char *cipsend_end = cipsend_pos;
                    while (*cipsend_end && *cipsend_end != '\n' && *cipsend_end != '\r') {
                        cipsend_end++;
                    }
                    
                    // Buscar ">" que esté después de AT+CIPSEND y en una línea separada
                    char *prompt_pos = NULL;
                    for (int i = (cipsend_end - response); i < total_len; i++) {
                        if (response[i] == '>') {
                            // Verificar que esté precedido por \n o \r (línea separada)
                            if (i > 0 && (response[i-1] == '\n' || response[i-1] == '\r')) {
                                // Verificar que NO sea parte de "+IPD" o "RECV FROM"
                                bool is_valid_prompt = true;
                                
                                // Verificar contexto antes del ">"
                                // No debe ser parte de "+IPD" o números
                                if (i >= 4) {
                                    if (strncmp(response + i - 4, "+IPD", 4) == 0) {
                                        is_valid_prompt = false;
                                    }
                                }
                                if (i >= 3 && strncmp(response + i - 3, "IPD", 3) == 0) {
                                    is_valid_prompt = false;
                                }
                                // Verificar que no esté dentro de un número (como "+IPD112>")
                                if (i > 0 && response[i-1] >= '0' && response[i-1] <= '9') {
                                    // Verificar si es parte de +IPD
                                    for (int j = i - 1; j >= 0 && j >= i - 10; j--) {
                                        if (response[j] == 'D' && j >= 2 && 
                                            response[j-1] == 'P' && response[j-2] == 'I' && 
                                            response[j-3] == '+') {
                                            is_valid_prompt = false;
                                            break;
                                        }
                                        if (response[j] == '\n' || response[j] == '\r') {
                                            break; // Llegamos al inicio de línea
                                        }
                                    }
                                }
                                
                                // Verificar que no esté dentro de "RECV FROM:"
                                if (i >= 10) {
                                    char *recv_from_check = response + i - 10;
                                    if (strncmp(recv_from_check, "RECV FROM:", 10) == 0) {
                                        is_valid_prompt = false;
                                    }
                                }
                                
                                // Verificar que NO sea parte de "+CIPRXGET:" (URC del modo buffer)
                                if (i >= 10) {
                                    char *ciprxget_check = response + i - 10;
                                    if (strncmp(ciprxget_check, "+CIPRXGET:", 10) == 0) {
                                        is_valid_prompt = false;
                                    }
                                }
                                
                                // Verificar que NO esté después de "+CIPRXGET:" en la misma línea
                                // Buscar hacia atrás para ver si hay "+CIPRXGET:" antes del \n/\r
                                for (int j = i - 1; j >= 0 && j >= i - 50; j--) {
                                    if (response[j] == '\n' || response[j] == '\r') {
                                        break; // Llegamos al inicio de línea, el '>' es válido
                                    }
                                    if (j >= 9 && strncmp(response + j - 9, "+CIPRXGET:", 10) == 0) {
                                        is_valid_prompt = false;
                                        break;
                                    }
                                }
                                
                                if (is_valid_prompt) {
                                    prompt_pos = response + i;
                                    ESP_LOGI(TAG, "✓ Prompt '>' detectado en posición %d (después de AT+CIPSEND)", i);
                                    break;
                                }
                            }
                        }
                    }
                    
                    if (prompt_pos != NULL) {
                        ESP_LOGI(TAG, "<< %s", response);
                        return SIM7600_OK;
                    }
                } else {
                    // Si no hay AT+CIPSEND en el buffer, buscar cualquier ">" en línea separada
                    // (puede ser que el comando ya se procesó)
                    for (int i = 0; i < total_len; i++) {
                        if (response[i] == '>') {
                            if (i == 0 || response[i-1] == '\n' || response[i-1] == '\r') {
                                // Verificar que NO sea parte de "+IPD" o "+CIPRXGET:"
                                bool is_valid = true;
                                if (i >= 4 && strncmp(response + i - 4, "+IPD", 4) == 0) {
                                    is_valid = false;
                                }
                                if (i >= 10 && strncmp(response + i - 10, "+CIPRXGET:", 10) == 0) {
                                    is_valid = false;
                                }
                                if (is_valid) {
                                    ESP_LOGI(TAG, "<< %s", response);
                                    ESP_LOGI(TAG, "✓ Prompt '>' detectado (sin AT+CIPSEND en buffer)");
                                    return SIM7600_OK;
                                }
                            }
                        }
                    }
                }
                
                // Durante la espera del prompt, también detectar URCs +CIPRXGET: 1,<link>
                // y marcarlos para procesar después, pero NO cambiar de estado
                char *ciprxget_urc = strstr(response, "+CIPRXGET: 1,");
                if (ciprxget_urc != NULL) {
                    int link_num = -1;
                    if (sscanf(ciprxget_urc, "+CIPRXGET: 1,%d", &link_num) == 1 && 
                        link_num >= 0 && link_num < MAX_LINKS) {
                        rx_pending[link_num] = true;
                        ESP_LOGI(TAG, "📥 URC detectado durante espera de prompt: +CIPRXGET: 1,%d (marcado para drenar después)", link_num);
                    }
                }
            } else {
                // Buscar respuesta esperada normal
                if (expected && strstr(response, expected) != NULL) {
                    ESP_LOGI(TAG, "<< %s", response);
                
                // Para +NETOPEN, verificar que el error sea 0 (éxito)
                if (strstr(expected, "+NETOPEN") != NULL) {
                    char *netopen_pos = strstr(response, "+NETOPEN:");
                    if (netopen_pos) {
                        int err_code = -1;
                        sscanf(netopen_pos, "+NETOPEN: %d", &err_code);
                        if (err_code == 0) {
                            return SIM7600_OK;
                        } else {
                            ESP_LOGE(TAG, "Error en NETOPEN: código %d", err_code);
                            return SIM7600_ERROR;
                        }
                    }
                }
                
                // Para CONNECT, verificar que no sea CONNECT FAIL
                if (strstr(expected, "CONNECT") != NULL) {
                    if (strstr(response, "CONNECT FAIL") != NULL) {
                        ESP_LOGE(TAG, "Conexión falló");
                        return SIM7600_ERROR;
                    }
                    if (strstr(response, "CONNECT") != NULL) {
                        return SIM7600_OK;
                    }
                }
                
                    return SIM7600_OK;
                }
            }
            
            // Verificar errores
            if (strstr(response, "ERROR") != NULL || 
                strstr(response, "CONNECT FAIL") != NULL ||
                strstr(response, "+CIPERROR") != NULL) {
                // Si hay +CIPERROR, extraer el código de error
                char *ciperror_pos = strstr(response, "+CIPERROR:");
                if (ciperror_pos) {
                    int error_code = -1;
                    if (sscanf(ciperror_pos, "+CIPERROR: %d", &error_code) == 1) {
                        ESP_LOGE(TAG, "Error TCP/IP: código %d", error_code);
                    }
                }
                ESP_LOGE(TAG, "<< %s", response);
                return SIM7600_ERROR;
            }
            
            // Si no hay expected, buscar OK
            if (!expected && strstr(response, "OK") != NULL) {
                ESP_LOGI(TAG, "<< %s", response);
                return SIM7600_OK;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGW(TAG, "<< Timeout esperando '%s' - Respuesta parcial: %s", 
             expected ? expected : "OK", response);
    return SIM7600_TIMEOUT;
}

// Función simple para enviar comando (sin esperar respuesta detallada)
void sim7600_send(const char *cmd)
{
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
}

// ============================================================================
// AT+CGDCONT - Define PDP Context
// ============================================================================

/**
 * @brief Configura el contexto PDP (Packet Data Protocol)
 * 
 * @param cid Identificador del contexto (1-24, 100-179)
 * @param pdp_type Tipo de protocolo: "IP", "PPP", "IPV6", "IPV4V6"
 * @param apn Nombre del punto de acceso (ej: "internet", "claro.com")
 * @param pdp_addr Dirección IP (generalmente "0.0.0.0" o NULL para omitir)
 * @param d_comp Compresión de datos: 0=off, 1=on, 2=V.42bis
 * @param h_comp Compresión de header: 0=off, 1=on, 2=RFC1144, 3=RFC2507, 4=RFC3095
 * @return sim7600_response_t SIM7600_OK si se configuró correctamente
 */
sim7600_response_t sim7600_set_pdp_context(int cid, const char *pdp_type, const char *apn, 
                                            const char *pdp_addr, int d_comp, int h_comp)
{
    char cmd[256];
    
    // Construir comando AT+CGDCONT
    if (pdp_addr == NULL) {
        // Forma simplificada: AT+CGDCONT=<cid>,"<PDP_type>","<APN>"
        snprintf(cmd, sizeof(cmd), "AT+CGDCONT=%d,\"%s\",\"%s\"\r\n", 
                 cid, pdp_type, apn ? apn : "");
    } else {
        // Forma completa con todos los parámetros
        snprintf(cmd, sizeof(cmd), "AT+CGDCONT=%d,\"%s\",\"%s\",\"%s\",%d,%d\r\n",
                 cid, pdp_type, apn ? apn : "", pdp_addr, d_comp, h_comp);
    }
    
    ESP_LOGI(TAG, "Configurando contexto PDP: CID=%d, Tipo=%s, APN=%s", 
             cid, pdp_type, apn ? apn : "(vacío)");
    
    return sim7600_send_command(cmd, shared_response_buffer, sizeof(shared_response_buffer), RESPONSE_TIMEOUT_MS);
}

/**
 * @brief Lee la configuración del contexto PDP
 * 
 * @param response Buffer para almacenar la respuesta
 * @param response_size Tamaño del buffer
 * @return sim7600_response_t SIM7600_OK si se leyó correctamente
 */
sim7600_response_t sim7600_read_pdp_context(char *response, size_t response_size)
{
    char cmd[] = "AT+CGDCONT?\r\n";
    ESP_LOGI(TAG, "Leyendo configuración de contexto PDP...");
    return sim7600_send_command(cmd, response, response_size, RESPONSE_TIMEOUT_MS);
}

/**
 * @brief Verifica los valores soportados para AT+CGDCONT
 * 
 * @param response Buffer para almacenar la respuesta
 * @param response_size Tamaño del buffer
 * @return sim7600_response_t SIM7600_OK si se obtuvo correctamente
 */
sim7600_response_t sim7600_test_pdp_context(char *response, size_t response_size)
{
    char cmd[] = "AT+CGDCONT=?\r\n";
    ESP_LOGI(TAG, "Verificando valores soportados para AT+CGDCONT...");
    return sim7600_send_command(cmd, response, response_size, RESPONSE_TIMEOUT_MS);
}

// ============================================================================
// AT+CIPMODE - Select TCP/IP application mode
// ============================================================================

/**
 * @brief Configura el modo de aplicación TCP/IP
 * 
 * @param mode 0=No transparente (comando), 1=Transparente (datos directos)
 * @return sim7600_response_t SIM7600_OK si se configuró correctamente
 */
sim7600_response_t sim7600_set_cipmode(int mode)
{
    char cmd[32];
    
    // Primero verificar el modo actual
    ESP_LOGI(TAG, "Verificando modo TCP/IP actual...");
    sim7600_response_t read_result = sim7600_read_cipmode(shared_response_buffer, sizeof(shared_response_buffer));
    
    if (read_result == SIM7600_OK) {
        // Buscar el modo actual en la respuesta (+CIPMODE: <mode>)
        char *mode_pos = strstr(shared_response_buffer, "+CIPMODE:");
        if (mode_pos) {
            int current_mode = -1;
            if (sscanf(mode_pos, "+CIPMODE: %d", &current_mode) == 1) {
                if (current_mode == mode) {
                    ESP_LOGI(TAG, "✓ Modo ya está configurado correctamente (%d)", mode);
                    return SIM7600_OK;
                }
                ESP_LOGI(TAG, "Modo actual: %d, cambiando a: %d", current_mode, mode);
            }
        }
    }
    
    // Cerrar cualquier conexión activa antes de cambiar el modo
    ESP_LOGI(TAG, "Cerrando conexiones activas antes de cambiar modo...");
    for (int i = 0; i < 10; i++) {
        char close_cmd[32];
        snprintf(close_cmd, sizeof(close_cmd), "AT+CIPCLOSE=%d\r\n", i);
        sim7600_send_command(close_cmd, shared_response_buffer, sizeof(shared_response_buffer), 2000);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Cerrar red si está activa
    ESP_LOGI(TAG, "Verificando estado de red...");
    sim7600_netclose();  // Es seguro llamarlo aunque no esté activa
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Ahora cambiar el modo
    snprintf(cmd, sizeof(cmd), "AT+CIPMODE=%d\r\n", mode);
    ESP_LOGI(TAG, "Configurando modo TCP/IP: %s", mode == 1 ? "Transparente" : "No transparente");
    
    return sim7600_send_command(cmd, shared_response_buffer, sizeof(shared_response_buffer), RESPONSE_TIMEOUT_MS);
}

/**
 * @brief Lee el modo TCP/IP actual
 * 
 * @param response Buffer para almacenar la respuesta
 * @param response_size Tamaño del buffer
 * @return sim7600_response_t SIM7600_OK si se leyó correctamente
 */
sim7600_response_t sim7600_read_cipmode(char *response, size_t response_size)
{
    char cmd[] = "AT+CIPMODE?\r\n";
    ESP_LOGI(TAG, "Leyendo modo TCP/IP...");
    return sim7600_send_command(cmd, response, response_size, RESPONSE_TIMEOUT_MS);
}

// ============================================================================
// AT+NETOPEN - Start TCPIP service
// ============================================================================

/**
 * @brief Activa el contexto PDP (inicia servicio TCP/IP)
 * 
 * @return sim7600_response_t SIM7600_OK si se activó correctamente o ya estaba activa
 */
sim7600_response_t sim7600_netopen(void)
{
    // Primero verificar el estado actual
    ESP_LOGI(TAG, "Verificando estado de red...");
    sim7600_response_t status_result = sim7600_netopen_status(shared_response_buffer, sizeof(shared_response_buffer));
    
    if (status_result == SIM7600_OK) {
        // Buscar el estado en la respuesta (+NETOPEN: <net_state>)
        char *netopen_pos = strstr(shared_response_buffer, "+NETOPEN:");
        if (netopen_pos) {
            int net_state = -1;
            if (sscanf(netopen_pos, "+NETOPEN: %d", &net_state) == 1) {
                if (net_state == 1) {
                    ESP_LOGI(TAG, "✓ Red ya está activa (estado: 1)");
                    return SIM7600_OK;
                }
                ESP_LOGI(TAG, "Red está cerrada (estado: %d), activando...", net_state);
            }
        }
    }
    
    // Intentar abrir la red
    char cmd[] = "AT+NETOPEN\r\n";
    ESP_LOGI(TAG, "Activando contexto PDP...");
    uart_flush_input(UART_SIM);
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
    
    // Esperar respuesta +NETOPEN: <err> donde 0 es éxito
    sim7600_response_t result = sim7600_wait_for_response("+NETOPEN:", shared_response_buffer, sizeof(shared_response_buffer), 120000);
    
    // Si recibimos error "Network is already opened", considerarlo éxito
    if (result == SIM7600_ERROR && strstr(shared_response_buffer, "Network is already opened") != NULL) {
        ESP_LOGI(TAG, "✓ Red ya estaba abierta (mensaje del módulo)");
        return SIM7600_OK;
    }
    
    return result;
}

/**
 * @brief Verifica el estado de la red
 * 
 * @param response Buffer para almacenar la respuesta
 * @param response_size Tamaño del buffer
 * @return sim7600_response_t SIM7600_OK si se leyó correctamente
 */
sim7600_response_t sim7600_netopen_status(char *response, size_t response_size)
{
    char cmd[] = "AT+NETOPEN?\r\n";
    ESP_LOGI(TAG, "Verificando estado de red...");
    return sim7600_send_command(cmd, response, response_size, RESPONSE_TIMEOUT_MS);
}

/**
 * @brief Desactiva el contexto PDP (cierra servicio TCP/IP)
 * 
 * @return sim7600_response_t SIM7600_OK si se desactivó correctamente
 */
sim7600_response_t sim7600_netclose(void)
{
    char cmd[] = "AT+NETCLOSE\r\n";
    
    ESP_LOGI(TAG, "Desactivando contexto PDP...");
    return sim7600_send_command(cmd, shared_response_buffer, sizeof(shared_response_buffer), RESPONSE_TIMEOUT_MS);
}

// ============================================================================
// AT+CIPDNSSET - Configure DNS server
// ============================================================================

/**
 * @brief Configura los servidores DNS
 * 
 * @param primary_dns DNS primario (ej: "8.8.8.8")
 * @param secondary_dns DNS secundario (ej: "8.8.4.4", puede ser NULL)
 * @return sim7600_response_t SIM7600_OK si se configuró correctamente
 */
sim7600_response_t sim7600_set_dns(const char *primary_dns, const char *secondary_dns)
{
    char cmd[128];
    
    // Sintaxis: AT+CIPDNSSET=<mode>[,<primary_dns>[,<secondary_dns>]]
    // mode: 0=obtener automático, 1=configurar manual
    if (secondary_dns) {
        snprintf(cmd, sizeof(cmd), "AT+CIPDNSSET=1,\"%s\",\"%s\"\r\n", primary_dns, secondary_dns);
    } else {
        snprintf(cmd, sizeof(cmd), "AT+CIPDNSSET=1,\"%s\"\r\n", primary_dns);
    }
    
    ESP_LOGI(TAG, "Configurando DNS: Primario=%s, Secundario=%s", 
             primary_dns, secondary_dns ? secondary_dns : "N/A");
    
    sim7600_response_t result = sim7600_send_command(cmd, shared_response_buffer, sizeof(shared_response_buffer), RESPONSE_TIMEOUT_MS);
    
    // Dar tiempo para que el DNS se configure
    if (result == SIM7600_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    return result;
}

// ============================================================================
// AT+CIPRXGET - Buffer Access Mode (Modo Buffer)
// ============================================================================

/**
 * @brief Configura el modo de recepción de datos TCP/UDP
 * 
 * @param mode 0=Directo (URC automático), 1=Buffer (manual con AT+CIPRXGET)
 * @return sim7600_response_t SIM7600_OK si se configuró correctamente
 */
sim7600_response_t sim7600_set_buffer_mode(int mode)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=%d\r\n", mode);
    
    ESP_LOGI(TAG, "Configurando modo buffer: %s", mode == 1 ? "Buffer (manual)" : "Directo (automático)");
    
    sim7600_response_t result = sim7600_send_command(cmd, shared_response_buffer, sizeof(shared_response_buffer), RESPONSE_TIMEOUT_MS);
    
    if (result == SIM7600_OK) {
        buffer_mode_active = (mode == 1);
        ESP_LOGI(TAG, "✓ Modo buffer configurado: %s", buffer_mode_active ? "ACTIVO" : "INACTIVO");
    } else {
        ESP_LOGE(TAG, "✗ Error configurando modo buffer");
    }
    
    return result;
}

/**
 * @brief Consulta la longitud de datos pendientes en el buffer de un socket
 * 
 * @param link_num Número de enlace (0-9)
 * @return int Longitud en bytes, -1 si hay error
 */
int sim7600_get_rx_buffer_length(int link_num)
{
    // NOTA: Esta función asume que el mutex ya está tomado por el llamador (sim7600_drain_rx_buffer)
    // NO llamar a sim7600_send_command() porque intentaría tomar el mutex de nuevo (deadlock)
    // Hacer la operación directamente sin mutex (el llamador ya lo tiene)
    
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=4,%d\r\n", link_num);
    
    ESP_LOGI(TAG, "📊 Consultando longitud de buffer RX para link %d...", link_num);
    
    // NO limpiar buffer - puede borrar URCs importantes
    // NO tomar mutex - el llamador ya lo tiene
    
    // Enviar comando directamente (sin mutex, el llamador ya lo tiene)
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
    
    // Leer respuesta directamente (sin mutex)
    memset(shared_response_buffer, 0, sizeof(shared_response_buffer));
    int total_len = 0;
    int64_t start_time = esp_timer_get_time() / 1000; // ms
    
    while ((esp_timer_get_time() / 1000 - start_time) < RESPONSE_TIMEOUT_MS) {
        int len = uart_read_bytes(UART_SIM, (uint8_t *)shared_response_buffer + total_len, 
                                   sizeof(shared_response_buffer) - total_len - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            total_len += len;
            shared_response_buffer[total_len] = '\0';
            
            // Verificar si recibimos OK o ERROR
            char *ok_pos = strstr(shared_response_buffer, "OK");
            char *error_pos = strstr(shared_response_buffer, "ERROR");
            
            if (ok_pos != NULL && (ok_pos == shared_response_buffer || ok_pos[-1] == '\n' || ok_pos[-1] == '\r')) {
                // OK recibido, parsear respuesta
                break;
            }
            if (error_pos != NULL && (error_pos == shared_response_buffer || error_pos[-1] == '\n' || error_pos[-1] == '\r')) {
                // ERROR recibido
                ESP_LOGE(TAG, "<< %s", shared_response_buffer);
                return -1;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Parsear respuesta
    if (strstr(shared_response_buffer, "OK") != NULL) {
        // Parsear respuesta: +CIPRXGET: 4,<link>,<rest_len>
        // El buffer puede contener el echo del comando y otros datos mezclados
        // Buscar la línea que empieza con "+CIPRXGET:" y tiene el formato correcto
        char *ciprxget_line = strstr(shared_response_buffer, "+CIPRXGET:");
        if (ciprxget_line != NULL) {
            // Buscar la línea completa (puede estar en formato "+CIPRXGET: 4,0,112" o "+CIPRXGET:4,0,112")
            int mode, link, rest;
            // Intentar con espacio después de ":"
            if (sscanf(ciprxget_line, "+CIPRXGET: %d,%d,%d", &mode, &link, &rest) == 3) {
                rest_len[link_num] = rest;
                ESP_LOGI(TAG, "✓ Buffer RX link %d: %d bytes pendientes", link_num, rest);
                return rest;
            }
            // Intentar sin espacio después de ":"
            if (sscanf(ciprxget_line, "+CIPRXGET:%d,%d,%d", &mode, &link, &rest) == 3) {
                rest_len[link_num] = rest;
                ESP_LOGI(TAG, "✓ Buffer RX link %d: %d bytes pendientes", link_num, rest);
                return rest;
            }
            ESP_LOGW(TAG, "⚠ No se pudo parsear línea +CIPRXGET: %.*s", 50, ciprxget_line);
            return -1;
        } else {
            // Verificar si es "No data" (puede venir sin +CIPRXGET:)
            if (strstr(shared_response_buffer, "No data") != NULL) {
                rest_len[link_num] = 0;
                ESP_LOGI(TAG, "✓ Buffer RX link %d: vacío (No data)", link_num);
                return 0;
            }
            ESP_LOGW(TAG, "⚠ No se encontró línea +CIPRXGET: en respuesta (primeros 200 chars): %.*s", 200, shared_response_buffer);
            return -1;
        }
    } else {
        // Verificar si es "No data"
        if (strstr(shared_response_buffer, "No data") != NULL) {
            rest_len[link_num] = 0;
            ESP_LOGI(TAG, "✓ Buffer RX link %d: vacío (0 bytes)", link_num);
            return 0;
        }
        ESP_LOGE(TAG, "✗ Error consultando buffer RX: %s", shared_response_buffer);
        return -1;
    }
}

/**
 * @brief Lee datos del buffer de un socket en formato ASCII
 * 
 * @param link_num Número de enlace (0-9)
 * @param len Cantidad de bytes a leer
 * @param buffer Buffer para almacenar los datos
 * @param buffer_size Tamaño del buffer
 * @return sim7600_response_t SIM7600_OK si se leyó correctamente
 */
sim7600_response_t sim7600_read_rx_buffer(int link_num, int len, char *buffer, size_t buffer_size)
{
    // NOTA: Esta función asume que el mutex ya está tomado por el llamador (sim7600_drain_rx_buffer)
    // NO tomar mutex aquí para evitar deadlock
    
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=2,%d,%d\r\n", link_num, len);
    
    ESP_LOGI(TAG, "📖 Leyendo %d bytes del buffer RX link %d...", len, link_num);
    
    // NO tomar mutex aquí - ya está tomado por el llamador (sim7600_drain_rx_buffer)
    // NO limpiar buffer - puede borrar URCs importantes
    // El parser manejará ecos mezclados
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
    
    // Leer respuesta: formato es +CIPRXGET: 2,<link>,<read_len>,<rest_len>\r\n<data>\r\nOK
    int64_t start_time = esp_timer_get_time() / 1000;
    int total_len = 0;
    memset(shared_response_buffer, 0, sizeof(shared_response_buffer));
    
    bool header_received = false;
    int read_len = 0;
    int new_rest_len = 0;
    char *data_start = NULL;
    
    while ((esp_timer_get_time() / 1000 - start_time) < 10000) { // 10s timeout
        int len_read = uart_read_bytes(UART_SIM, (uint8_t *)shared_response_buffer + total_len,
                                       sizeof(shared_response_buffer) - total_len - 1,
                                       pdMS_TO_TICKS(100));
        if (len_read > 0) {
            total_len += len_read;
            shared_response_buffer[total_len] = '\0';
            
            // Buscar header +CIPRXGET: 2,<link>,<read_len>,<rest_len>
            if (!header_received) {
                char *header_pos = strstr(shared_response_buffer, "+CIPRXGET:");
                if (header_pos) {
                    int mode, link;
                    if (sscanf(header_pos, "+CIPRXGET: %d,%d,%d,%d", &mode, &link, &read_len, &new_rest_len) == 4) {
                        header_received = true;
                        rest_len[link_num] = new_rest_len;
                        ESP_LOGI(TAG, "<< +CIPRXGET: 2,%d,%d,%d (restante: %d)", link, read_len, new_rest_len, new_rest_len);
                        
                        // Buscar inicio de datos (después de \r\n después del header)
                        data_start = strstr(header_pos, "\r\n");
                        if (data_start) {
                            data_start += 2; // Saltar \r\n
                        } else {
                            data_start = header_pos + strlen("+CIPRXGET: X,X,X,X");
                        }
                    }
                }
            }
            
            // Si tenemos header, buscar fin de datos (OK)
            if (header_received && data_start) {
                char *ok_pos = strstr(shared_response_buffer, "\r\nOK");
                if (ok_pos) {
                    // Extraer datos entre data_start y ok_pos
                    size_t data_size = ok_pos - data_start;
                        if (data_size > 0 && data_size < buffer_size) {
                            memcpy(buffer, data_start, data_size);
                            buffer[data_size] = '\0';
                            ESP_LOGI(TAG, "✓ Datos leídos: %d bytes", (int)data_size);
                            
                            // NO liberar mutex aquí - el llamador (sim7600_drain_rx_buffer) lo hará
                            return SIM7600_OK;
                    } else if (data_size >= buffer_size) {
                        ESP_LOGE(TAG, "✗ Buffer de salida demasiado pequeño (necesita %d bytes)", (int)data_size);
                        // NO liberar mutex aquí - el llamador lo hará
                        return SIM7600_ERROR;
                    }
                }
            }
        } else {
            // Si no hay más datos y tenemos header, puede que los datos estén completos
            if (header_received && total_len > 0) {
                // Buscar OK al final
                if (strstr(shared_response_buffer, "OK") != NULL && data_start) {
                    char *ok_pos = strstr(shared_response_buffer, "\r\nOK");
                    if (ok_pos) {
                        size_t data_size = ok_pos - data_start;
                        if (data_size > 0 && data_size < buffer_size) {
                            memcpy(buffer, data_start, data_size);
                            buffer[data_size] = '\0';
                            ESP_LOGI(TAG, "✓ Datos leídos: %d bytes", (int)data_size);
                            
                            // NO liberar mutex aquí - el llamador (sim7600_drain_rx_buffer) lo hará
                            return SIM7600_OK;
                        }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(100)); // Esperar un poco más
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // NO liberar mutex aquí - el llamador (sim7600_drain_rx_buffer) lo hará
    
    ESP_LOGE(TAG, "✗ Timeout leyendo buffer RX (parcial: %.*s)", 100, shared_response_buffer);
    return SIM7600_TIMEOUT;
}

/**
 * @brief Drena completamente el buffer RX de un socket
 * 
 * @param link_num Número de enlace (0-9)
 * @return sim7600_response_t SIM7600_OK si se drenó correctamente
 */
sim7600_response_t sim7600_drain_rx_buffer(int link_num)
{
    ESP_LOGI(TAG, "🔄 [RX_DRAIN] Iniciando drenaje de buffer RX para link %d", link_num);
    
    #define RX_CHUNK_SIZE 256
    char rx_chunk[RX_CHUNK_SIZE + 1];
    int total_drained = 0;
    int iterations = 0;
    const int MAX_ITERATIONS = 100; // Protección contra loops infinitos
    
    while (iterations < MAX_ITERATIONS) {
        iterations++;
        
        // 1. Consultar cuánto hay pendiente
        int rest = sim7600_get_rx_buffer_length(link_num);
        if (rest < 0) {
            ESP_LOGE(TAG, "✗ Error consultando buffer RX");
            break;
        }
        
        if (rest == 0) {
            ESP_LOGI(TAG, "✅ [RX_DRAIN] Buffer RX drenado completamente (%d bytes totales, %d iteraciones)", 
                    total_drained, iterations);
            rx_pending[link_num] = false;
            rest_len[link_num] = 0;
            // NO verificar aquí porque puede haber ecos de ACKs en el buffer
            // Si hay nuevos datos, el URC se detectará en la tarea asíncrona y se procesará en el siguiente ciclo
            return SIM7600_OK;
        }
        
        // 2. Leer chunk (máximo RX_CHUNK_SIZE)
        int chunk_size = (rest > RX_CHUNK_SIZE) ? RX_CHUNK_SIZE : rest;
        ESP_LOGI(TAG, "   [RX_DRAIN] Iteración %d: Leyendo chunk de %d bytes (restante: %d bytes)", 
                iterations, chunk_size, rest);
        
        sim7600_response_t result = sim7600_read_rx_buffer(link_num, chunk_size, rx_chunk, sizeof(rx_chunk));
        if (result != SIM7600_OK) {
            ESP_LOGE(TAG, "✗ [RX_DRAIN] Error leyendo chunk del buffer RX");
            break;
        }
        
        int chunk_len = strlen(rx_chunk);
        total_drained += chunk_len;
        
        // 3. Procesar datos recibidos (parsear NDJSON)
        ESP_LOGI(TAG, "   📥 [RX_DRAIN] Datos recibidos (%d bytes): %.*s", chunk_len,
                 chunk_len > 150 ? 150 : chunk_len, rx_chunk);
        
        // Parsear frames NDJSON (separados por \n)
        const char *frame_start = rx_chunk;
        while (frame_start && *frame_start) {
            // Buscar siguiente \n o fin de string
            const char *frame_end = strchr(frame_start, '\n');
            size_t frame_len = frame_end ? (frame_end - frame_start) : strlen(frame_start);
            
            // Limpiar \r si existe al final
            if (frame_len > 0 && frame_start[frame_len - 1] == '\r') {
                frame_len--;
            }
            
            if (frame_len > 0) {
                // Procesar frame
                ESP_LOGI(TAG, "   📨 [RX_DRAIN] Frame NDJSON (%d bytes): %.*s", 
                        (int)frame_len, (int)frame_len, frame_start);
                sim7600_process_server_command(frame_start, frame_len);
            }
            
            if (frame_end) {
                frame_start = frame_end + 1;
            } else {
                break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    
    if (iterations >= MAX_ITERATIONS) {
        ESP_LOGW(TAG, "⚠ [RX_DRAIN] Máximo de iteraciones alcanzado, deteniendo drenaje");
    }
    
    rx_pending[link_num] = false;
    ESP_LOGI(TAG, "✅ [RX_DRAIN] Drenaje completado (%d bytes totales, %d iteraciones)", 
            total_drained, iterations);
    return SIM7600_OK;
}

// ============================================================================
// AT+CIPOPEN - Setup TCP/UDP client socket connection
// ============================================================================

/**
 * @brief Abre una conexión TCP en modo transparente
 * 
 * @param link_num Número de enlace (0-9, debe ser 0 en modo transparente)
 * @param server_ip Dirección IP o dominio del servidor
 * @param server_port Puerto del servidor
 * @return sim7600_response_t SIM7600_OK si se conectó correctamente
 */
sim7600_response_t sim7600_cipopen_tcp_transparent(int link_num, const char *server_ip, int server_port)
{
    char cmd[128];
    
    snprintf(cmd, sizeof(cmd), "AT+CIPOPEN=%d,\"TCP\",\"%s\",%d\r\n", 
             link_num, server_ip, server_port);
    
    ESP_LOGI(TAG, "Abriendo conexión TCP: Link=%d, Servidor=%s:%d", 
             link_num, server_ip, server_port);
    
    uart_flush_input(UART_SIM);
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
    
    // En modo transparente, esperamos "CONNECT" o "CONNECT FAIL"
    return sim7600_wait_for_response("CONNECT", shared_response_buffer, sizeof(shared_response_buffer), 120000);
}

/**
 * @brief Abre una conexión TCP en modo no transparente
 * 
 * @param link_num Número de enlace (0-9)
 * @param server_ip Dirección IP o dominio del servidor
 * @param server_port Puerto del servidor
 * @return sim7600_response_t SIM7600_OK si se conectó correctamente
 */
sim7600_response_t sim7600_cipopen_tcp(int link_num, const char *server_ip, int server_port)
{
    char cmd[128];
    
    snprintf(cmd, sizeof(cmd), "AT+CIPOPEN=%d,\"TCP\",\"%s\",%d\r\n", 
             link_num, server_ip, server_port);
    
    ESP_LOGI(TAG, "Abriendo conexión TCP (no transparente): Link=%d, Servidor=%s:%d", 
             link_num, server_ip, server_port);
    
    uart_flush_input(UART_SIM);
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
    
    // En modo no transparente, esperamos +CIPOPEN: <link_num>,<err> donde 0 es éxito
    sim7600_response_t result = sim7600_wait_for_response("+CIPOPEN:", shared_response_buffer, sizeof(shared_response_buffer), 120000);
    
    if (result == SIM7600_OK) {
        // Verificar que el código de error sea 0
        char *cipopen_pos = strstr(shared_response_buffer, "+CIPOPEN:");
        if (cipopen_pos) {
            int link, err_code;
            if (sscanf(cipopen_pos, "+CIPOPEN: %d,%d", &link, &err_code) == 2) {
                if (err_code == 0) {
                    ESP_LOGI(TAG, "✓ Conexión TCP establecida (link %d)", link);
                    return SIM7600_OK;
                } else {
                    ESP_LOGE(TAG, "✗ Error en conexión: código %d", err_code);
                    return SIM7600_ERROR;
                }
            }
        }
    }
    
    return result;
}

/**
 * @brief Cierra una conexión TCP/UDP
 * 
 * @param link_num Número de enlace a cerrar (0-9)
 * @return sim7600_response_t SIM7600_OK si se cerró correctamente
 */
sim7600_response_t sim7600_cipclose(int link_num)
{
    char cmd[32];
    
    snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d\r\n", link_num);
    
    ESP_LOGI(TAG, "Cerrando conexión: Link=%d", link_num);
    
    uart_flush_input(UART_SIM);
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
    
    // En modo transparente, esperamos "CLOSED" o respuesta +CIPCLOSE
    sim7600_response_t result = sim7600_wait_for_response("CLOSED", shared_response_buffer, sizeof(shared_response_buffer), 10000);
    if (result == SIM7600_TIMEOUT) {
        // Intentar buscar +CIPCLOSE si no encontramos CLOSED
        result = sim7600_wait_for_response("+CIPCLOSE", shared_response_buffer, sizeof(shared_response_buffer), 5000);
    }
    
    return result;
}

// ============================================================================
// AT+CIPSEND - Send TCP/UDP data (modo no transparente)
// ============================================================================

/**
 * @brief Envía datos TCP usando AT+CIPSEND (modo no transparente)
 * 
 * @param link_num Número de enlace (0-9)
 * @param data Datos a enviar
 * @param data_len Longitud de los datos (0 para longitud variable con Ctrl+Z)
 * @return sim7600_response_t SIM7600_OK si se envió correctamente
 */
sim7600_response_t sim7600_cipsend(int link_num, const uint8_t *data, size_t data_len)
{
    char cmd[64];
    
    if (data_len == 0) {
        // Longitud variable - termina con Ctrl+Z
        snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,\r\n", link_num);
    } else {
        // Longitud fija
        snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%d\r\n", link_num, data_len);
    }
    
    ESP_LOGI(TAG, "Enviando datos: Link=%d, Longitud=%d", link_num, data_len);
    
    // Bloquear UART durante operación crítica
    if (uart_mutex != NULL) {
        xSemaphoreTake(uart_mutex, portMAX_DELAY);
    }
    
    // NO limpiar buffer - puede borrar URCs importantes
    // El parser manejará ecos mezclados
    uart_write_bytes(UART_SIM, cmd, strlen(cmd));
    ESP_LOGI(TAG, ">> %s", cmd);
    
    // Esperar el prompt ">"
    sim7600_response_t result = sim7600_wait_for_response(">", shared_response_buffer, sizeof(shared_response_buffer), 5000);
    if (result != SIM7600_OK) {
        // Verificar si hay error TCP/IP
        if (strstr(shared_response_buffer, "+CIPERROR") != NULL) {
            ESP_LOGE(TAG, "✗ Error TCP/IP detectado, conexión puede estar cerrada");
            // La conexión puede estar cerrada, pero no liberamos el mutex aquí
            // porque el llamador debería manejar la reconexión
        } else {
            ESP_LOGE(TAG, "✗ No se recibió el prompt '>' - Respuesta parcial: %s", shared_response_buffer);
        }
        if (uart_mutex != NULL) {
            xSemaphoreGive(uart_mutex);
        }
        return SIM7600_ERROR;
    }
    
    // Después de recibir ">", detectar URC +CIPRXGET: 1,<link> si está mezclado en el buffer
    char *ciprxget_urc = strstr(shared_response_buffer, "+CIPRXGET: 1,");
    if (ciprxget_urc != NULL) {
        int urc_link = -1;
        if (sscanf(ciprxget_urc, "+CIPRXGET: 1,%d", &urc_link) == 1 &&
            urc_link >= 0 && urc_link < MAX_LINKS) {
            rx_pending[urc_link] = true;
            ESP_LOGI(TAG, "📥 URC +CIPRXGET: 1,%d detectado durante espera de prompt (marcado para drenar)", urc_link);
        }
    }
    
    // Enviar los datos
    uart_write_bytes(UART_SIM, data, data_len);
    
    // Si es longitud variable, enviar Ctrl+Z (0x1A) para terminar
    if (data_len == 0) {
        uint8_t ctrl_z = 0x1A;
        uart_write_bytes(UART_SIM, &ctrl_z, 1);
        ESP_LOGI(TAG, ">> [Datos + Ctrl+Z]");
    } else {
        ESP_LOGI(TAG, ">> [Datos: %.*s]", (int)data_len, data);
    }
    
    // Esperar respuesta +CIPSEND: <link_num>,<reqSendLength>,<cnfSendLength>
    result = sim7600_wait_for_response("+CIPSEND:", shared_response_buffer, sizeof(shared_response_buffer), 10000);
    
    if (result == SIM7600_OK) {
        // Verificar que se envió correctamente
        char *cipsend_pos = strstr(shared_response_buffer, "+CIPSEND:");
        if (cipsend_pos) {
            int link, req_len, cnf_len;
            if (sscanf(cipsend_pos, "+CIPSEND: %d,%d,%d", &link, &req_len, &cnf_len) == 3) {
                if (req_len == cnf_len) {
                    ESP_LOGI(TAG, "✓ Datos enviados correctamente (%d bytes)", cnf_len);
                    
                    // Datos del servidor llegan via CIPRXGET; el loop principal los drena
                    if (rx_pending[link_num]) {
                        ESP_LOGI(TAG, "📥 [TX_WAIT_RESULT] RX pendiente (link %d), se drenará en loop principal", link_num);
                    }
                    
                    if (uart_mutex != NULL) {
                        xSemaphoreGive(uart_mutex);
                    }
                    return SIM7600_OK;
                } else {
                    ESP_LOGW(TAG, "⚠ Datos parcialmente enviados: solicitado=%d, confirmado=%d", req_len, cnf_len);
                    if (uart_mutex != NULL) {
                        xSemaphoreGive(uart_mutex);
                    }
                    return (cnf_len > 0) ? SIM7600_OK : SIM7600_ERROR;
                }
            }
        }
    }
    
    // Liberar mutex antes de retornar
    if (uart_mutex != NULL) {
        xSemaphoreGive(uart_mutex);
    }
    
    return result;
}

// ============================================================================
// Lectura de datos en modo no transparente
// ============================================================================

/**
 * @brief Lee datos recibidos en modo no transparente
 * Los datos pueden venir directamente al puerto COM o se pueden leer con AT+CIPRXGET
 * 
 * @param buffer Buffer para almacenar los datos
 * @param buffer_size Tamaño del buffer
 * @param timeout_ms Timeout en milisegundos
 * @return int Número de bytes leídos
 */
int sim7600_read_data_non_transparent(uint8_t *buffer, size_t buffer_size, int timeout_ms)
{
    int64_t start_time = esp_timer_get_time() / 1000;
    int total_len = 0;
    
    memset(buffer, 0, buffer_size);
    
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
        int len = uart_read_bytes(UART_SIM, buffer + total_len, 
                                   buffer_size - total_len - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            total_len += len;
            buffer[total_len] = '\0';
        }
        
        // Si no hay más datos disponibles y ya leímos algo, salir
        if (len == 0 && total_len > 0) {
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (total_len > 0) {
        ESP_LOGI(TAG, "Recibidos %d bytes en modo no transparente", total_len);
    }
    
    return total_len;
}

// ============================================================================
// Lectura asíncrona y procesamiento de comandos del servidor
// ============================================================================

// Buffer para comandos recibidos
static char server_command_buffer[512];
static bool async_read_active = false;

// Contador de mensajes recibidos del servidor
static int server_messages_received = 0;

// Sistema de deduplicación: cache de request_id procesados recientemente
#define MAX_PROCESSED_REQUESTS 20
static char processed_request_ids[MAX_PROCESSED_REQUESTS][64];
static int processed_request_count = 0;
static int processed_request_index = 0;

// Cola de comandos para procesamiento serializado
#define COMMAND_QUEUE_SIZE 20  // Aumentado para manejar múltiples mensajes simultáneos
typedef struct {
    char command[256];
    char timestamp[64];
    char client_id[64];  // ID del cliente que hizo la consulta (opcional)
    char request_id[64]; // ID de la petición (opcional, alternativa a client_id)
    char command_id[64]; // ID del comando original (para ACK con mismo id)
    int message_id;
} command_queue_item_t;

static QueueHandle_t command_queue = NULL;
static bool command_processor_active = false;

// Buffer estático para evitar stack overflow al procesar comandos
static command_queue_item_t static_cmd_item;

// ============================================================================
// OBSOLETO: Cola de ACKs pendientes (ya no se usa en modo buffer)
// ============================================================================
// NOTA: En modo buffer, los ACKs son inmediatos y no se incluyen en actualizaciones.
// Obsoleto: no se usa en modo buffer.
// TODO: Eliminar en futura limpieza si se confirma que no se necesita compatibilidad.
// ============================================================================
#define PENDING_ACK_QUEUE_SIZE 10
typedef struct {
    char command[32];
    char timestamp[64];
    char client_id[64];  // ID del cliente para correlación
    char request_id[64]; // ID de la petición (opcional)
    int message_id;
} pending_ack_item_t;

__attribute__((unused)) static QueueHandle_t pending_ack_queue = NULL;  // OBSOLETO: No se usa en modo buffer

// Buffers estáticos (OBSOLETOS: no se usan en modo buffer)
__attribute__((unused)) static pending_ack_item_t static_acks_buffer[PENDING_ACK_QUEUE_SIZE];
__attribute__((unused)) static char static_ack_obj_buffer[256];

/**
 * @brief Procesa comandos recibidos del servidor
 * 
 * @param data Datos recibidos
 * @param len Longitud de los datos
 */
void sim7600_process_server_command(const char *data, size_t len)
{
    if (len == 0 || data == NULL) {
        return;
    }
    
    // Buscar JSON en los datos
    const char *json_start = strchr(data, '{');
    if (json_start == NULL) {
        // No es JSON, puede ser texto plano
        if (strstr(data, "unlock") != NULL || strstr(data, "UNLOCK") != NULL) {
            server_messages_received++;
            ESP_LOGI(TAG, "🔓 [Mensaje #%d] Comando UNLOCK recibido del servidor (texto plano)", server_messages_received);
        } else if (strstr(data, "lock") != NULL || strstr(data, "LOCK") != NULL) {
            server_messages_received++;
            ESP_LOGI(TAG, "🔒 [Mensaje #%d] Comando LOCK recibido del servidor (texto plano)", server_messages_received);
        }
        return;
    }
    
    // Es JSON, intentar parsear
    // Buscar el final del JSON
    const char *json_end = strchr(json_start, '}');
    if (json_end == NULL) {
        ESP_LOGW(TAG, "JSON incompleto recibido");
        return;
    }
    
    size_t json_len = json_end - json_start + 1;
    if (json_len >= sizeof(server_command_buffer)) {
        ESP_LOGW(TAG, "JSON demasiado grande");
        return;
    }
    
    memcpy(server_command_buffer, json_start, json_len);
    server_command_buffer[json_len] = '\0';
    
    // Extraer request_id PRIMERO para verificar duplicados
    char request_id[64] = "";     // ID de la petición (opcional)
    
    // Macro auxiliar para extraer campo JSON (reutilizada)
    static char extract_buffer[32];
    #define EXTRACT_JSON_FIELD(json, field_name, output, output_size) do { \
        int field_len = strlen(field_name); \
        if (field_len < sizeof(extract_buffer) - 3) { \
            extract_buffer[0] = '"'; \
            strncpy(extract_buffer + 1, field_name, sizeof(extract_buffer) - 3); \
            extract_buffer[field_len + 1] = '"'; \
            extract_buffer[field_len + 2] = '\0'; \
            char *field_pos = strstr(json, extract_buffer); \
            if (field_pos) { \
                char *value_start = strchr(field_pos, ':'); \
                if (value_start) { \
                    value_start = strchr(value_start, '"'); \
                    if (value_start) { \
                        value_start++; \
                        char *value_end = strchr(value_start, '"'); \
                        if (value_end && (value_end - value_start) < output_size) { \
                            strncpy(output, value_start, value_end - value_start); \
                            output[value_end - value_start] = '\0'; \
                        } \
                    } \
                } \
            } \
        } \
    } while(0)
    
    EXTRACT_JSON_FIELD(server_command_buffer, "request_id", request_id, sizeof(request_id));
    
    // Verificar si este request_id ya fue procesado (deduplicación)
    bool already_processed = false;
    if (request_id[0] != '\0') {
        for (int i = 0; i < processed_request_count; i++) {
            if (strcmp(processed_request_ids[i], request_id) == 0) {
                already_processed = true;
                ESP_LOGW(TAG, "⚠ Comando duplicado ignorado (request_id: %s)", request_id);
                break;
            }
        }
    }
    
    // Si ya fue procesado, ignorar
    if (already_processed) {
        return;
    }
    
    // Agregar request_id a la lista de procesados
    if (request_id[0] != '\0') {
        strncpy(processed_request_ids[processed_request_index], request_id, 
                sizeof(processed_request_ids[0]) - 1);
        processed_request_ids[processed_request_index][sizeof(processed_request_ids[0]) - 1] = '\0';
        processed_request_index = (processed_request_index + 1) % MAX_PROCESSED_REQUESTS;
        if (processed_request_count < MAX_PROCESSED_REQUESTS) {
            processed_request_count++;
        }
    }
    
    // Incrementar contador de mensajes recibidos
    server_messages_received++;
    
    ESP_LOGI(TAG, "📥 [Mensaje #%d] Comando JSON recibido: %s", server_messages_received, server_command_buffer);
    
    // Extraer información del comando para el ACK
    char command_type[32] = "";
    char timestamp[64] = "";
    char client_id[64] = "";      // ID del cliente que hizo la consulta
    char command_id[64] = "";    // ID del comando original (para ACK)
    // request_id ya fue extraído arriba para deduplicación
    
    // Buscar tipo de comando
    if (strstr(server_command_buffer, "\"command\"") != NULL) {
        // Extraer valor de "command"
        EXTRACT_JSON_FIELD(server_command_buffer, "command", command_type, sizeof(command_type));
        
        // Extraer timestamp si existe
        EXTRACT_JSON_FIELD(server_command_buffer, "timestamp", timestamp, sizeof(timestamp));
        
        // Extraer client_id si existe (para correlación con cliente original)
        EXTRACT_JSON_FIELD(server_command_buffer, "client_id", client_id, sizeof(client_id));
        
        // Extraer id del comando (para ACK con mismo id)
        EXTRACT_JSON_FIELD(server_command_buffer, "id", command_id, sizeof(command_id));
        
        // request_id ya fue extraído arriba para deduplicación
    }
    
    #undef EXTRACT_JSON_FIELD
    
    // Agregar comando a la cola para procesamiento serializado
    // Usar buffer estático para evitar stack overflow
    if (command_queue != NULL) {
        // Usar buffer estático en lugar de variable local (ahorra ~450 bytes de stack)
        strncpy(static_cmd_item.command, server_command_buffer, sizeof(static_cmd_item.command) - 1);
        static_cmd_item.command[sizeof(static_cmd_item.command) - 1] = '\0';
        strncpy(static_cmd_item.timestamp, timestamp, sizeof(static_cmd_item.timestamp) - 1);
        static_cmd_item.timestamp[sizeof(static_cmd_item.timestamp) - 1] = '\0';
        strncpy(static_cmd_item.client_id, client_id, sizeof(static_cmd_item.client_id) - 1);
        static_cmd_item.client_id[sizeof(static_cmd_item.client_id) - 1] = '\0';
        strncpy(static_cmd_item.request_id, request_id, sizeof(static_cmd_item.request_id) - 1);
        static_cmd_item.request_id[sizeof(static_cmd_item.request_id) - 1] = '\0';
        strncpy(static_cmd_item.command_id, command_id, sizeof(static_cmd_item.command_id) - 1);
        static_cmd_item.command_id[sizeof(static_cmd_item.command_id) - 1] = '\0';
        static_cmd_item.message_id = server_messages_received;
        
        if (xQueueSend(command_queue, &static_cmd_item, 0) == pdTRUE) {
            ESP_LOGI(TAG, "✅ Comando agregado a cola (ID: %d)", static_cmd_item.message_id);
        } else {
            ESP_LOGW(TAG, "⚠ Cola de comandos llena, comando descartado (ID: %d)", static_cmd_item.message_id);
        }
    } else {
        // Si no hay cola, procesar directamente (fallback)
        ESP_LOGW(TAG, "⚠ Cola no inicializada, procesando directamente");
        if (strstr(server_command_buffer, "\"command\"") != NULL) {
            if (strstr(server_command_buffer, "\"unlock\"") != NULL) {
                ESP_LOGI(TAG, "🔓 [Mensaje #%d] Comando UNLOCK recibido (procesamiento directo)", server_messages_received);
                sim7600_send_ack(command_id, command_type, timestamp, client_id, request_id, server_messages_received);
            } else if (strstr(server_command_buffer, "\"lock\"") != NULL) {
                ESP_LOGI(TAG, "🔒 [Mensaje #%d] Comando LOCK recibido (procesamiento directo)", server_messages_received);
                sim7600_send_ack(command_id, command_type, timestamp, client_id, request_id, server_messages_received);
            }
        }
    }
}

/**
 * @brief Tarea asíncrona para leer datos del servidor continuamente
 * 
 * @param pvParameters Parámetros de la tarea (no usado)
 */
void sim7600_async_read_task(void *pvParameters)
{
    uint8_t temp_buffer[256];
    
    ESP_LOGI(TAG, "📡 Tarea de lectura asíncrona iniciada");
    
    while (async_read_active) {
        // En modo buffer: solo detectar URCs, NO leer datos directamente
        // Los datos se leen mediante polling en el loop principal
        // Intentar tomar el mutex con timeout corto (50ms) para no bloquear operaciones críticas
        if (uart_mutex != NULL) {
            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Mutex obtenido, leer solo para detectar URCs
                // Leer con timeout muy corto para no interferir
                int len = uart_read_bytes(UART_SIM, temp_buffer, sizeof(temp_buffer) - 1,
                                          pdMS_TO_TICKS(15));
                
                // Liberar mutex inmediatamente después de leer
                xSemaphoreGive(uart_mutex);
                
                if (len > 0) {
                    temp_buffer[len] = '\0';
                    
                    // Detectar URC +CIPRXGET: 1,<link> (datos en buffer del socket)
                    char *ciprxget_urc = strstr((char*)temp_buffer, "+CIPRXGET: 1,");
                    if (ciprxget_urc != NULL) {
                        int link_num = -1;
                        if (sscanf(ciprxget_urc, "+CIPRXGET: 1,%d", &link_num) == 1 &&
                            link_num >= 0 && link_num < MAX_LINKS) {
                            rx_pending[link_num] = true;
                            ESP_LOGI(TAG, "📥 [URC] +CIPRXGET: 1,%d - datos pendientes (drenar en loop)", link_num);
                            continue;
                        }
                    }
                    char *ciprxget_urc2 = strstr((char*)temp_buffer, "+CIPRXGET:1,");
                    if (ciprxget_urc2 != NULL) {
                        int link_num = -1;
                        if (sscanf(ciprxget_urc2, "+CIPRXGET:1,%d", &link_num) == 1 &&
                            link_num >= 0 && link_num < MAX_LINKS) {
                            rx_pending[link_num] = true;
                            ESP_LOGI(TAG, "📥 [URC] +CIPRXGET:1,%d - datos pendientes (drenar en loop)", link_num);
                            continue;
                        }
                    }
                    
                    // Detectar +IPCLOSE y +CIPERROR; filtrar respuestas AT
                    if (strstr((char*)temp_buffer, "+IPCLOSE:") != NULL) {
                        ESP_LOGW(TAG, "⚠ [URC] Conexión cerrada: %.*s", 100, temp_buffer);
                        continue;
                    }
                    if (strstr((char*)temp_buffer, "+CIPERROR:") != NULL) {
                        ESP_LOGE(TAG, "⚠ [URC] Error TCP/IP: %.*s", 100, temp_buffer);
                        continue;
                    }
                    if (strstr((char*)temp_buffer, "+CIPSEND:") != NULL ||
                        strstr((char*)temp_buffer, "+CIPOPEN:") != NULL ||
                        strstr((char*)temp_buffer, "+NETOPEN:") != NULL ||
                        strstr((char*)temp_buffer, "+NETCLOSE:") != NULL ||
                        strstr((char*)temp_buffer, "+CIPCLOSE:") != NULL ||
                        strstr((char*)temp_buffer, "+CIPRXGET:") != NULL ||
                        strstr((char*)temp_buffer, "OK") != NULL ||
                        strstr((char*)temp_buffer, "ERROR") != NULL ||
                        strstr((char*)temp_buffer, ">") != NULL) {
                        continue;
                    }
                }
            }
        }
        
        // Pequeña pausa para no saturar CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "📡 Tarea de lectura asíncrona finalizada");
    vTaskDelete(NULL);
}

// ============================================================================
// Procesamiento serializado de comandos con cola
// ============================================================================

/**
 * @brief Envía ACK al servidor confirmando recepción de comando
 * 
 * @param command Tipo de comando recibido (ej: "unlock")
 * @param timestamp Timestamp del comando original
 * @param client_id ID del cliente que hizo la consulta (opcional, puede ser "")
 * @param request_id ID de la petición (opcional, puede ser "")
 * @param message_id ID del mensaje recibido
 * @return sim7600_response_t SIM7600_OK si se envió correctamente
 */
/**
 * @brief Envía ACK inmediato al servidor en formato NDJSON
 * 
 * @param command_id ID del comando original (para correlación)
 * @param command Tipo de comando ejecutado (ej: "unlock")
 * @param timestamp Timestamp del comando original
 * @param client_id ID del cliente que hizo la consulta
 * @param request_id ID de la petición
 * @param message_id ID interno del mensaje
 * @return sim7600_response_t SIM7600_OK si se envió correctamente
 */
sim7600_response_t sim7600_send_ack(const char *command_id, const char *command, const char *timestamp, 
                                    const char *client_id, const char *request_id, int message_id)
{
    char ack_json[512];
    int ack_len;
    
    // Construir JSON de ACK en formato NDJSON
    // Formato: {"id":"c-<original_id>","type":"ack","status":"ok","command":"unlock",...}
    // Si no hay command_id, usar message_id como fallback
    if (command_id != NULL && command_id[0] != '\0') {
        ack_len = snprintf(ack_json, sizeof(ack_json),
                          "{\"id\":\"%s\",\"type\":\"ack\",\"status\":\"ok\",\"command\":\"%s\",\"original_timestamp\":\"%s\"",
                          command_id, command, timestamp);
    } else {
        // Fallback: usar message_id como id
        char fallback_id[32];
        snprintf(fallback_id, sizeof(fallback_id), "c-%d", message_id);
        ack_len = snprintf(ack_json, sizeof(ack_json),
                          "{\"id\":\"%s\",\"type\":\"ack\",\"status\":\"ok\",\"command\":\"%s\",\"original_timestamp\":\"%s\"",
                          fallback_id, command, timestamp);
    }
    
    // Agregar client_id si existe
    if (client_id != NULL && client_id[0] != '\0') {
        ack_len += snprintf(ack_json + ack_len, sizeof(ack_json) - ack_len,
                          ",\"client_id\":\"%s\"", client_id);
    }
    
    // Agregar request_id si existe
    if (request_id != NULL && request_id[0] != '\0') {
        ack_len += snprintf(ack_json + ack_len, sizeof(ack_json) - ack_len,
                          ",\"request_id\":\"%s\"", request_id);
    }
    
    // Cerrar JSON y agregar \n (NDJSON)
    ack_len += snprintf(ack_json + ack_len, sizeof(ack_json) - ack_len, "}\n");
    
    if (ack_len >= sizeof(ack_json)) {
        ESP_LOGE(TAG, "✗ Error: Buffer ACK demasiado pequeño");
        return SIM7600_ERROR;
    }
    
    ESP_LOGI(TAG, "📤 [ACK Inmediato] Enviando ACK: %s", ack_json);
    
    // Enviar ACK usando AT+CIPSEND
    // NOTA: En modo buffer, esto no interferirá con RX porque los datos están en buffer
    return sim7600_cipsend(0, (uint8_t *)ack_json, ack_len);
}

/**
 * @brief Tarea que procesa comandos de la cola de forma serializada
 * 
 * @param pvParameters Parámetros de la tarea (no usado)
 */
void sim7600_command_processor_task(void *pvParameters)
{
    command_queue_item_t cmd_item;
    
    ESP_LOGI(TAG, "⚙️ Tarea procesadora de comandos iniciada");
    
    while (command_processor_active) {
        // Esperar comando de la cola (bloqueante)
        if (xQueueReceive(command_queue, &cmd_item, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "🔄 Procesando comando de la cola (ID: %d)", cmd_item.message_id);
            
            // Procesar comando unlock o lock
            if (strstr(cmd_item.command, "\"command\"") != NULL) {
                const char *cmd_name = NULL;
                if (strstr(cmd_item.command, "\"unlock\"") != NULL) {
                    cmd_name = "unlock";
                    ESP_LOGI(TAG, "🔓 [ID: %d] Ejecutando comando UNLOCK", cmd_item.message_id);
                } else if (strstr(cmd_item.command, "\"lock\"") != NULL) {
                    cmd_name = "lock";
                    ESP_LOGI(TAG, "🔒 [ID: %d] Ejecutando comando LOCK", cmd_item.message_id);
                }
                
                if (cmd_name != NULL) {
                    // Aquí puedes agregar la lógica real: GPIO/relé, etc.
                    sim7600_response_t ack_result = sim7600_send_ack(cmd_item.command_id, cmd_name, cmd_item.timestamp, 
                                                                      cmd_item.client_id, cmd_item.request_id, 
                                                                      cmd_item.message_id);
                    if (ack_result == SIM7600_OK) {
                        ESP_LOGI(TAG, "✅ ACK inmediato enviado (ID: %s, message_id: %d)", 
                                cmd_item.command_id[0] != '\0' ? cmd_item.command_id : "N/A", cmd_item.message_id);
                    } else {
                        ESP_LOGW(TAG, "⚠ ACK inmediato falló (ID: %d) - El backend debe hacer retry", cmd_item.message_id);
                    }
                } else {
                    ESP_LOGW(TAG, "⚠ Comando desconocido en cola (ID: %d): %s", cmd_item.message_id, cmd_item.command);
                }
            } else {
                ESP_LOGW(TAG, "⚠ Comando desconocido en cola (ID: %d): %s", cmd_item.message_id, cmd_item.command);
            }
        }
    }
    
    ESP_LOGI(TAG, "⚙️ Tarea procesadora de comandos finalizada");
    vTaskDelete(NULL);
}

// ============================================================================
// Funciones auxiliares para modo transparente
// ============================================================================

/**
 * @brief Envía datos en modo transparente (directamente por UART)
 * 
 * @param data Datos a enviar
 * @param len Longitud de los datos
 * @return int Número de bytes enviados
 */
int sim7600_send_data_transparent(const uint8_t *data, size_t len)
{
    int sent = uart_write_bytes(UART_SIM, data, len);
    ESP_LOGI(TAG, "Enviados %d bytes en modo transparente", sent);
    return sent;
}

/**
 * @brief Lee datos en modo transparente
 * 
 * @param buffer Buffer para almacenar los datos
 * @param buffer_size Tamaño del buffer
 * @param timeout_ms Timeout en milisegundos
 * @return int Número de bytes leídos
 */
int sim7600_read_data_transparent(uint8_t *buffer, size_t buffer_size, int timeout_ms)
{
    int64_t start_time = esp_timer_get_time() / 1000;
    int total_len = 0;
    
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
        int len = uart_read_bytes(UART_SIM, buffer + total_len, 
                                   buffer_size - total_len - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            total_len += len;
            buffer[total_len] = '\0';
        }
        
        // Si no hay más datos disponibles, salir
        if (len == 0 && total_len > 0) {
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (total_len > 0) {
        ESP_LOGI(TAG, "Recibidos %d bytes en modo transparente", total_len);
    }
    
    return total_len;
}

// ============================================================================
// Función principal: Ping TCP 5 veces
// ============================================================================

/**
 * @brief Realiza ping TCP a un servidor público 5 veces usando modo no transparente con AT+CIPSEND
 * 
 * @param server_ip Dirección IP o dominio del servidor
 * @param server_port Puerto del servidor
 */
void sim7600_tcp_ping_test(const char *server_ip, int server_port)
{
    sim7600_response_t result;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "=== Iniciando prueba de ping TCP ===");
    ESP_LOGI(TAG, "Servidor: %s:%d (Modo NO transparente)", server_ip, server_port);
    ESP_LOGI(TAG, "========================================");
    
    // 1. Configurar modo NO transparente
    ESP_LOGI(TAG, "\n[1/7] Configurando modo NO transparente...");
    result = sim7600_set_cipmode(0);
    if (result != SIM7600_OK) {
        ESP_LOGE(TAG, "✗ Error configurando modo no transparente");
        return;
    }
    ESP_LOGI(TAG, "✓ Modo no transparente configurado");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 2. Activar contexto PDP
    ESP_LOGI(TAG, "\n[2/7] Activando contexto PDP...");
    result = sim7600_netopen();
    if (result != SIM7600_OK) {
        ESP_LOGE(TAG, "✗ Error activando contexto PDP");
        return;
    }
    
    // Verificar que se activó correctamente (buscar +NETOPEN: 0)
    vTaskDelay(pdMS_TO_TICKS(3000)); // Dar tiempo para que la red se establezca
    ESP_LOGI(TAG, "✓ Contexto PDP activado");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 2.5. Configurar DNS (opcional pero recomendado)
    ESP_LOGI(TAG, "\n[2.5/7] Configurando DNS...");
    result = sim7600_set_dns("8.8.8.8", "8.8.4.4"); // Google DNS
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "✓ DNS configurado");
    } else {
        ESP_LOGW(TAG, "⚠ Advertencia: No se pudo configurar DNS (puede que ya esté configurado)");
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // Dar tiempo adicional después de configurar DNS
    
    // 3. Abrir conexión TCP
    ESP_LOGI(TAG, "\n[3/7] Abriendo conexión TCP...");
    result = sim7600_cipopen_tcp(0, server_ip, server_port);
    if (result != SIM7600_OK) {
        ESP_LOGE(TAG, "✗ Error abriendo conexión TCP");
        ESP_LOGE(TAG, "  Código de error común: 11 = Error DNS, 12 = Error de conexión");
        ESP_LOGI(TAG, "  Intenta usar una IP directa en lugar de dominio");
        sim7600_netclose();
        return;
    }
    ESP_LOGI(TAG, "✓ Conexión TCP establecida");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 4. Realizar ping 5 veces usando AT+CIPSEND
    ESP_LOGI(TAG, "\n[4/7] Realizando ping 5 veces con AT+CIPSEND...");
    for (int i = 1; i <= 5; i++) {
        char ping_msg[64];
        snprintf(ping_msg, sizeof(ping_msg), "PING %d - ESP32 SIM7600 Test\r\n", i);
        size_t msg_len = strlen(ping_msg);
        
        ESP_LOGI(TAG, ">>> Ping #%d: Enviando '%s'", i, ping_msg);
        
        // Enviar datos usando AT+CIPSEND con longitud fija
        result = sim7600_cipsend(0, (uint8_t *)ping_msg, msg_len);
        if (result != SIM7600_OK) {
            ESP_LOGE(TAG, "✗ Error enviando ping #%d", i);
            continue;
        }
        
        // Esperar respuesta del servidor
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Leer datos recibidos (pueden venir directamente al puerto COM)
        int rx_len = sim7600_read_data_non_transparent(shared_data_buffer, sizeof(shared_data_buffer), 2000);
        if (rx_len > 0) {
            // Filtrar respuestas AT del buffer
            char *data_start = (char *)shared_data_buffer;
            // Buscar datos reales (saltar respuestas AT como OK, +CIPSEND, etc.)
            if (strstr(data_start, "OK") || strstr(data_start, "+CIPSEND")) {
                // Los datos reales pueden venir después de las respuestas AT
                char *real_data = strstr(data_start, ping_msg);
                if (real_data) {
                    ESP_LOGI(TAG, "<<< Ping #%d: Respuesta recibida (%d bytes):", i, rx_len);
                    ESP_LOGI(TAG, "    %s", real_data);
                } else {
                    ESP_LOGI(TAG, "<<< Ping #%d: Datos recibidos (%d bytes)", i, rx_len);
                }
            } else {
                ESP_LOGI(TAG, "<<< Ping #%d: Respuesta recibida (%d bytes):", i, rx_len);
                ESP_LOGI(TAG, "    %s", data_start);
            }
        } else {
            ESP_LOGW(TAG, "<<< Ping #%d: Sin respuesta", i);
        }
        
        // Esperar entre pings
        if (i < 5) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "✓ Ping completado (5/5)");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 5. Cerrar conexión TCP
    ESP_LOGI(TAG, "\n[5/7] Cerrando conexión TCP...");
    result = sim7600_cipclose(0);
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "✓ Conexión TCP cerrada");
    } else {
        ESP_LOGW(TAG, "⚠ Advertencia al cerrar conexión");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 6. Desactivar contexto PDP
    ESP_LOGI(TAG, "\n[6/7] Desactivando contexto PDP...");
    result = sim7600_netclose();
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "✓ Contexto PDP desactivado");
    } else {
        ESP_LOGW(TAG, "⚠ Advertencia al desactivar contexto");
    }
    
    ESP_LOGI(TAG, "\n========================================");
    ESP_LOGI(TAG, "=== Prueba de ping TCP completada ===");
    ESP_LOGI(TAG, "========================================\n");
}

// ============================================================================
// Función: Envío de actualizaciones de scooter
// ============================================================================

/**
 * @brief Envía una actualización de ubicación del scooter al servidor TCP
 * 
 * @param link_num Número de enlace TCP (0-9)
 * @param scooter_id ID del scooter
 * @param latitude Latitud
 * @param longitude Longitud
 * @param battery Nivel de batería (0-100)
 * @param speed_kmh Velocidad en km/h
 * @return sim7600_response_t SIM7600_OK si se envió correctamente
 */
sim7600_response_t sim7600_send_scooter_update(int link_num, int scooter_id, 
                                                double latitude, double longitude,
                                                int battery, float speed_kmh)
{
    char json_buffer[256];
    int json_len;
    
    // Construir JSON de telemetría (sin ACKs - los ACKs se envían inmediatamente)
    json_len = snprintf(json_buffer, sizeof(json_buffer),
                       "{\"scooter_id\":%d,\"latitude\":%.6f,\"longitude\":%.6f,\"battery_level\":%d,\"speed_kmh\":%.1f}\n",
                       scooter_id, latitude, longitude, battery, speed_kmh);
    
    // Verificar que el JSON sea válido (debe tener al menos el formato básico)
    if (json_len <= 0) {
        ESP_LOGE(TAG, "✗ Error: Buffer JSON vacío");
        return SIM7600_ERROR;
    }
    
    // Si el JSON está truncado, ajustar para que sea válido
    if (json_len >= sizeof(json_buffer)) {
        // Truncar al tamaño máximo y cerrar correctamente
        json_len = sizeof(json_buffer) - 1;
        // Buscar el último '}' o ']' y cerrar ahí
        for (int i = json_len - 1; i >= 0; i--) {
            if (json_buffer[i] == '}' || json_buffer[i] == ']') {
                json_buffer[i + 1] = '\n';
                json_buffer[i + 2] = '\0';
                json_len = i + 2;
                break;
            }
        }
        ESP_LOGW(TAG, "⚠ JSON truncado a %d bytes para ajustar al buffer", json_len);
    }
    
    // Si el JSON está cerca del límite pero es válido, continuar (no es error crítico)
    if (json_len >= sizeof(json_buffer) - 5) {
        ESP_LOGW(TAG, "⚠ JSON cerca del límite (%d/%d bytes), pero válido", json_len, sizeof(json_buffer));
    }
    
    ESP_LOGI(TAG, "📤 Enviando actualización scooter #%d", scooter_id);
    ESP_LOGI(TAG, "   JSON: %s", json_buffer);
    
    // Enviar usando AT+CIPSEND
    sim7600_response_t result = sim7600_cipsend(link_num, (uint8_t *)json_buffer, json_len);
    
    if (result == SIM7600_OK) {
        vTaskDelay(pdMS_TO_TICKS(80));
        ESP_LOGI(TAG, "✅ Actualización enviada");
    }
    
    return result;
}

/**
 * @brief Inicializa conexión TCP y envía actualizaciones periódicas del scooter
 * 
 * @param server_ip IP o dominio del servidor
 * @param server_port Puerto del servidor
 */
void sim7600_scooter_update_loop(const char *server_ip, int server_port)
{
    sim7600_response_t result;
    int location_index = 0;
    int update_count = 0;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "=== Scooter Update Service ===");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🛴 Scooter ID: %d", SCOOTER_ID);
    ESP_LOGI(TAG, "📍 Ubicaciones alternadas: 2 puntos");
    ESP_LOGI(TAG, "🔋 Batería: %d%%", SCOOTER_BATTERY);
    ESP_LOGI(TAG, "⚡ Velocidad: %.1f km/h", SCOOTER_SPEED_KMH);
    ESP_LOGI(TAG, "🔄 Intervalo: %d segundos", UPDATE_INTERVAL_SEC);
    ESP_LOGI(TAG, "🌐 Servidor: %s:%d", server_ip, server_port);
    ESP_LOGI(TAG, "========================================\n");
    
    // 1. Configurar modo NO transparente
    ESP_LOGI(TAG, "[1/6] Configurando modo NO transparente...");
    result = sim7600_set_cipmode(0);
    if (result != SIM7600_OK) {
        ESP_LOGW(TAG, "⚠ No se pudo cambiar modo (puede que ya esté en modo no transparente)");
        ESP_LOGI(TAG, "   Continuando asumiendo modo no transparente (predeterminado)...");
    } else {
        ESP_LOGI(TAG, "✓ Modo no transparente configurado");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 1.5. Configurar modo buffer (AT+CIPRXGET=1)
    ESP_LOGI(TAG, "[1.5/6] Configurando modo buffer (AT+CIPRXGET=1)...");
    result = sim7600_set_buffer_mode(1);
    if (result != SIM7600_OK) {
        ESP_LOGE(TAG, "✗ Error configurando modo buffer");
        ESP_LOGW(TAG, "   Continuando sin modo buffer...");
    } else {
        ESP_LOGI(TAG, "✓ Modo buffer configurado - Los datos se almacenarán en buffer interno");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 2. Activar contexto PDP
    ESP_LOGI(TAG, "[2/6] Activando contexto PDP...");
    result = sim7600_netopen();
    if (result != SIM7600_OK) {
        ESP_LOGE(TAG, "✗ Error activando contexto PDP");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "✓ Contexto PDP activado");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 2.5. Configurar DNS
    ESP_LOGI(TAG, "[2.5/6] Configurando DNS...");
    result = sim7600_set_dns("8.8.8.8", "8.8.4.4");
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "✓ DNS configurado");
    } else {
        ESP_LOGW(TAG, "⚠ DNS no configurado (puede que ya esté configurado)");
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 3. Abrir conexión TCP
    ESP_LOGI(TAG, "[3/6] Abriendo conexión TCP...");
    result = sim7600_cipopen_tcp(0, server_ip, server_port);
    if (result != SIM7600_OK) {
        ESP_LOGE(TAG, "✗ Error abriendo conexión TCP");
        ESP_LOGE(TAG, "  Código de error común: 11 = Error DNS, 12 = Error de conexión");
        sim7600_netclose();
        return;
    }
    ESP_LOGI(TAG, "✓ Conexión TCP establecida");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 3.5. Crear mutex para proteger UART
    if (uart_mutex == NULL) {
        uart_mutex = xSemaphoreCreateMutex();
        if (uart_mutex == NULL) {
            ESP_LOGE(TAG, "✗ Error creando mutex para UART");
        } else {
            ESP_LOGI(TAG, "✓ Mutex UART creado");
        }
    }
    
    // 3.6. Crear cola de comandos
    ESP_LOGI(TAG, "[3.6/6] Creando cola de comandos...");
    command_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(command_queue_item_t));
    if (command_queue == NULL) {
        ESP_LOGE(TAG, "✗ Error creando cola de comandos");
    } else {
        ESP_LOGI(TAG, "✓ Cola de comandos creada (tamaño: %d)", COMMAND_QUEUE_SIZE);
    }
    
    // 3.7. Iniciar tarea procesadora de comandos
    ESP_LOGI(TAG, "[3.7/6] Iniciando tarea procesadora de comandos...");
    command_processor_active = true;
    xTaskCreate(sim7600_command_processor_task, "cmd_processor", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "✓ Tarea procesadora de comandos iniciada");
    
    // 3.8. Iniciar tarea de lectura asíncrona
    // En modo buffer, la tarea async solo detecta URCs, no lee datos directamente
    // Los datos se leen mediante polling en el loop principal
    ESP_LOGI(TAG, "[3.8/6] Iniciando tarea de lectura asíncrona...");
    async_read_active = true;
    xTaskCreate(sim7600_async_read_task, "sim7600_async_read", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "✓ Tarea de lectura asíncrona iniciada (modo: %s)", 
             "Buffer (URCs + polling)");
    vTaskDelay(pdMS_TO_TICKS(300));
    
    ESP_LOGI(TAG, "\n🚀 Iniciando envío de actualizaciones...\n");
    
    // 4. Loop de actualizaciones (con máquina de estados simplificada)
    current_state = STATE_RUN_IDLE;
    
    while (1) {
        update_count++;
        
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        ESP_LOGI(TAG, "[Ciclo #%d] | 📨 Mensajes recibidos del servidor: %d", 
                 update_count, server_messages_received);
        ESP_LOGI(TAG, "📍 Estado actual: %s", 
                 current_state == STATE_RUN_IDLE ? "RUN_IDLE" :
                 current_state == STATE_RX_DRAIN ? "RX_DRAIN" :
                 current_state == STATE_TX_PREPARE ? "TX_PREPARE" : "OTRO");
        
        // PRIORIDAD 1: Drenar buffer RX (modo buffer)
        // Hacer POLLING siempre, no depender solo de URCs
        // Esto asegura que no se pierdan datos aunque el URC no llegue
        // CRÍTICO: Todo el drenaje debe estar protegido para evitar intercalación con CIPSEND
        if (current_state != STATE_RX_DRAIN) {
            // Proteger toda la operación de drenaje con mutex
            // Esto evita que CIPSEND (ACKs, telemetría) se intercale
            if (uart_mutex != NULL) {
                xSemaphoreTake(uart_mutex, portMAX_DELAY);
            }
            
            // Consultar siempre el buffer (polling)
            int rest = sim7600_get_rx_buffer_length(0);
            if (rest > 0) {
                ESP_LOGI(TAG, "🔄 [PRIORIDAD 1] Datos RX detectados por polling (%d bytes), drenando buffer...", rest);
                current_state = STATE_RX_DRAIN;
                
                // El drenaje completo está protegido por el mutex
                // sim7600_drain_rx_buffer() NO debe tomar mutex (ya lo tenemos)
                result = sim7600_drain_rx_buffer(0);
                
                if (result == SIM7600_OK) {
                    ESP_LOGI(TAG, "✅ Buffer RX drenado correctamente");
                    rx_pending[0] = false; // Asegurar que esté limpio
                } else {
                    ESP_LOGW(TAG, "⚠ Error drenando buffer RX");
                    rx_pending[0] = false; // Limpiar flag en caso de error
                }
                current_state = STATE_RUN_IDLE;
            } else if (rx_pending[0]) {
                // Si estaba marcado pero ya no hay datos, limpiar flag
                ESP_LOGD(TAG, "📥 Flag rx_pending[0] estaba activo pero buffer vacío, limpiando");
                rx_pending[0] = false;
            }
            
            // Liberar mutex después del drenaje completo
            if (uart_mutex != NULL) {
                xSemaphoreGive(uart_mutex);
            }
        }
        
        // PRIORIDAD 2: Verificar conexión (si está cerrada, reconectar)
        // (Esto se maneja en el error handling de sim7600_send_scooter_update)
        
        // PRIORIDAD 3: Enviar telemetría (si no hay datos RX pendientes)
        if (!rx_pending[0]) {
            // Obtener ubicación alternada
            const location_t *loc = &locations[location_index];
            location_index = (location_index + 1) % 2;  // Alternar entre 0 y 1
            
            ESP_LOGI(TAG, "📍 Lat: %.6f, Lon: %.6f", loc->latitude, loc->longitude);
            
            current_state = STATE_TX_PREPARE;
            // Enviar actualización
            result = sim7600_send_scooter_update(0, SCOOTER_ID, 
                                                loc->latitude, loc->longitude,
                                                SCOOTER_BATTERY, SCOOTER_SPEED_KMH);
            current_state = STATE_RUN_IDLE;
            
            if (result != SIM7600_OK) {
            ESP_LOGE(TAG, "✗ Error enviando actualización #%d", update_count);
            
            // Verificar si la conexión está cerrada (error TCP/IP)
            // Intentar reconectar automáticamente
            ESP_LOGW(TAG, "🔄 Intentando reconectar...");
            
            // 1. Cerrar conexión actual si está abierta
            ESP_LOGI(TAG, "   Cerrando conexión actual...");
            sim7600_cipclose(0);
            vTaskDelay(pdMS_TO_TICKS(2000));
            
            // 2. Verificar estado de la red
            ESP_LOGI(TAG, "   Verificando estado de red...");
            char net_status[256];
            sim7600_netopen_status(net_status, sizeof(net_status));
            if (strstr(net_status, "+NETOPEN: 0") == NULL) {
                // Red cerrada, reactivar
                ESP_LOGI(TAG, "   Reactivando contexto PDP...");
                sim7600_netopen();
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
            
            // 3. Reconectar TCP
            ESP_LOGI(TAG, "   Reconectando TCP...");
            result = sim7600_cipopen_tcp(0, server_ip, server_port);
            if (result == SIM7600_OK) {
                ESP_LOGI(TAG, "✅ Reconexión exitosa");
                // Reintentar envío inmediatamente después de reconectar
                result = sim7600_send_scooter_update(0, SCOOTER_ID, 
                                                    loc->latitude, loc->longitude,
                                                    SCOOTER_BATTERY, SCOOTER_SPEED_KMH);
                if (result == SIM7600_OK) {
                    ESP_LOGI(TAG, "✅ Actualización enviada después de reconexión");
                } else {
                    ESP_LOGW(TAG, "⚠ Error después de reconexión, esperando %d segundos...", UPDATE_INTERVAL_SEC);
                }
            } else {
                ESP_LOGE(TAG, "✗ Error en reconexión, esperando %d segundos...", UPDATE_INTERVAL_SEC);
            }
            } // Cierre del if (result != SIM7600_OK)
        }
        
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
        
        // Esperar antes de la siguiente actualización; despertar cada RX_POLL_MS para revisar RX
        int wait_ms = UPDATE_INTERVAL_SEC * 1000;
        while (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(RX_POLL_MS));
            wait_ms -= RX_POLL_MS;
            if (rx_pending[0]) {
                break;
            }
        }
    }
    
    // Nota: Este código nunca se alcanza, pero por si acaso...
    ESP_LOGI(TAG, "[4/5] Cerrando conexión TCP...");
    sim7600_cipclose(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "[5/5] Desactivando contexto PDP...");
    sim7600_netclose();
}

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
    result = sim7600_send_command("AT\r\n", shared_response_buffer, sizeof(shared_response_buffer), RESPONSE_TIMEOUT_MS);
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "✓ SIM7600 responde correctamente");
    } else {
        ESP_LOGE(TAG, "✗ Error comunicándose con SIM7600");
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. Verificar valores soportados para AT+CGDCONT
    ESP_LOGI(TAG, "=== Verificando valores soportados AT+CGDCONT ===");
    result = sim7600_test_pdp_context(shared_response_buffer, sizeof(shared_response_buffer));
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
    result = sim7600_read_pdp_context(shared_response_buffer, sizeof(shared_response_buffer));
    if (result == SIM7600_OK) {
        ESP_LOGI(TAG, "Configuración actual:");
        // La respuesta contiene +CGDCONT: 1,"IP","internet",...
        ESP_LOGI(TAG, "%s", shared_response_buffer);
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

