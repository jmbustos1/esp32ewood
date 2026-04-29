/**
 * @file periph_uart.c
 * @brief Comunicacion UART1 con electronica usando protocolo binario de frames.
 *
 * UART1 remapeada a pines libres (GPIO4 TX, GPIO5 RX).
 * Tarea de recepcion parsea frames entrantes y actualiza estado interno.
 * Si no llegan datos, se mantiene ultimo valor conocido.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"

#include "app_config.h"
#include "frame_protocol.h"
#include "periph_uart.h"

static const char *TAG = "PERIPH";

/* ── Configuracion UART1 ─────────────────────────────────────────── */
#define PERIPH_UART       UART_NUM_1
#define PERIPH_TX_PIN     GPIO_NUM_4
#define PERIPH_RX_PIN     GPIO_NUM_5
#define PERIPH_BAUD_RATE  115200
#define PERIPH_BUF_SIZE   512

/* ── Estado interno (ultimo valor conocido) ──────────────────────── */
static bms_power_t  s_bms_power  = { .valid = false };
static bms_status_t s_bms_status = { .valid = false };
static lock_state_t s_lock_state = LOCK_STATE_UNKNOWN;
static SemaphoreHandle_t s_data_mutex = NULL;

/* Parser de frames */
static frame_parser_t s_parser;

/* ── Helpers ─────────────────────────────────────────────────────── */

static void process_bms_soc(const frame_parsed_t *f)
{
    /* MSGID 0x02: Voltaje(2B) + Corriente(2B) + Cap.residual(2B) + SoC(1B) */
    if (f->data_len < 7) {
        ESP_LOGW(TAG, "BMS SoC: data_len=%u < 7", f->data_len);
        return;
    }

    uint16_t raw_voltage  = ((uint16_t)f->data[0] << 8) | f->data[1];
    int16_t  raw_current  = (int16_t)(((uint16_t)f->data[2] << 8) | f->data[3]);
    uint16_t raw_capacity = ((uint16_t)f->data[4] << 8) | f->data[5];
    uint8_t  soc          = f->data[6];

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    s_bms_power.voltage     = raw_voltage * 0.01f;     /* 10mV → V */
    s_bms_power.current     = raw_current * 0.01f;     /* 10mA → A */
    s_bms_power.capacity_ah = raw_capacity * 0.01f;    /* 10mA → Ah */
    s_bms_power.soc         = soc;
    s_bms_power.valid       = true;
    xSemaphoreGive(s_data_mutex);

    ESP_LOGI(TAG, "BMS: %.2fV  %.2fA  SoC=%u%%",
             s_bms_power.voltage, s_bms_power.current, soc);
}

static void process_bms_control_status(const frame_parsed_t *f)
{
    /* MSGID 0x05: ControlStatus(1B) + ProtectionStatus(2B) + BalanceStatus(4B) */
    if (f->data_len < 7) {
        ESP_LOGW(TAG, "BMS Status: data_len=%u < 7", f->data_len);
        return;
    }

    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    s_bms_status.control_status = f->data[0];
    s_bms_status.protection_status = ((uint16_t)f->data[1] << 8) | f->data[2];
    s_bms_status.balance_status = ((uint32_t)f->data[3] << 24) |
                                   ((uint32_t)f->data[4] << 16) |
                                   ((uint32_t)f->data[5] << 8)  |
                                   f->data[6];
    s_bms_status.valid = true;
    xSemaphoreGive(s_data_mutex);

    ESP_LOGI(TAG, "BMS Status: ctrl=0x%02X prot=0x%04X bal=0x%08lX",
             s_bms_status.control_status,
             s_bms_status.protection_status,
             (unsigned long)s_bms_status.balance_status);
}

static void process_actuator_response(const frame_parsed_t *f)
{
    /*
     * Respuesta del modulo actuadores (MODULE 0x03).
     * Por ahora asumimos que un ANS confirma el ultimo comando enviado.
     * El contenido del MSG dependera de la implementacion del micro actuador.
     * Minimo: MSGID indica lock(0x01) o unlock(0x02) confirmado.
     */
    if (f->data_len < 1) return;

    lock_state_t new_state = LOCK_STATE_UNKNOWN;
    if (f->msg_id == 0x01) {
        new_state = LOCK_STATE_LOCKED;
    } else if (f->msg_id == 0x02) {
        new_state = LOCK_STATE_UNLOCKED;
    }

    if (new_state != LOCK_STATE_UNKNOWN) {
        xSemaphoreTake(s_data_mutex, portMAX_DELAY);
        s_lock_state = new_state;
        xSemaphoreGive(s_data_mutex);

        ESP_LOGI(TAG, "Actuador: %s",
                 new_state == LOCK_STATE_LOCKED ? "LOCKED" : "UNLOCKED");
    }
}

static void process_frame(const frame_parsed_t *f)
{
    /* Solo procesamos respuestas (ANS) y eventos (EVENT) */
    if (f->type != FRAME_TYPE_ANS && f->type != FRAME_TYPE_EVENT) {
        ESP_LOGD(TAG, "Frame tipo 0x%02X ignorado", f->type);
        return;
    }

    switch (f->module_id) {
    case MODULE_BMS:
        switch (f->msg_id) {
        case BMS_MSGID_VOLTAGE_SOC:
            process_bms_soc(f);
            break;
        case BMS_MSGID_CONTROL_STATUS:
            process_bms_control_status(f);
            break;
        default:
            ESP_LOGD(TAG, "BMS MSGID 0x%02X no manejado", f->msg_id);
            break;
        }
        break;

    case MODULE_ACTUATORS:
        process_actuator_response(f);
        break;

    case MODULE_SYSTEM:
        ESP_LOGI(TAG, "Sistema MSGID=0x%02X", f->msg_id);
        break;

    default:
        ESP_LOGD(TAG, "Module 0x%02X no manejado", f->module_id);
        break;
    }
}

/* ── Tarea de recepcion ──────────────────────────────────────────── */

static void periph_rx_task(void *pvParameters)
{
    uint8_t rx_byte;

    ESP_LOGI(TAG, "Tarea RX electronica iniciada");

    while (1) {
        int len = uart_read_bytes(PERIPH_UART, &rx_byte, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            if (frame_parser_feed(&s_parser, rx_byte)) {
                frame_parsed_t f = frame_parser_get_result(&s_parser);
                process_frame(&f);
            }
        }
    }
}

/* ── Envio de frames ─────────────────────────────────────────────── */

static int periph_send_frame(const uint8_t *frame, size_t len)
{
    int written = uart_write_bytes(PERIPH_UART, frame, len);
    if (written < 0 || (size_t)written != len) {
        ESP_LOGE(TAG, "Error enviando frame (%d/%zu)", written, len);
        return -1;
    }
    return 0;
}

/* ── API publica ─────────────────────────────────────────────────── */

int periph_uart_init(void)
{
    s_data_mutex = xSemaphoreCreateMutex();
    if (s_data_mutex == NULL) {
        ESP_LOGE(TAG, "Error creando mutex datos");
        return -1;
    }

    frame_parser_init(&s_parser);

    uart_config_t uart_config = {
        .baud_rate = PERIPH_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t err = uart_driver_install(PERIPH_UART, PERIPH_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error instalando driver UART1: %s", esp_err_to_name(err));
        return -1;
    }

    uart_param_config(PERIPH_UART, &uart_config);
    uart_set_pin(PERIPH_UART, PERIPH_TX_PIN, PERIPH_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART1 inicializada (TX=GPIO%d, RX=GPIO%d, %d bps)",
             PERIPH_TX_PIN, PERIPH_RX_PIN, PERIPH_BAUD_RATE);

    /* Iniciar tarea de recepcion */
    xTaskCreate(periph_rx_task, "periph_rx", 4096, NULL, 4, NULL);

    return 0;
}

int periph_request_bms_soc(uint8_t period_sec)
{
    uint8_t frame_buf[32];
    int len = frame_build_read(MODULE_BMS, BMS_MSGID_VOLTAGE_SOC, period_sec,
                               frame_buf, sizeof(frame_buf));
    if (len < 0) return -1;

    ESP_LOGI(TAG, "Solicitando BMS SoC (period=%us)", period_sec);
    return periph_send_frame(frame_buf, len);
}

int periph_request_bms_status(uint8_t period_sec)
{
    uint8_t frame_buf[32];
    int len = frame_build_read(MODULE_BMS, BMS_MSGID_CONTROL_STATUS, period_sec,
                               frame_buf, sizeof(frame_buf));
    if (len < 0) return -1;

    ESP_LOGI(TAG, "Solicitando BMS Status (period=%us)", period_sec);
    return periph_send_frame(frame_buf, len);
}

int periph_send_lock(void)
{
    /* WRITE al modulo actuadores: MODULE=0x03, MSGID=0x01 (lock) */
    uint8_t payload[3];
    payload[0] = MODULE_ACTUATORS;
    payload[1] = 0x01;  /* MSGID: lock */
    payload[2] = 0x00;  /* sin periodo */

    uint8_t frame_buf[32];
    int len = frame_build(FRAME_TYPE_WRITE, payload, sizeof(payload),
                          frame_buf, sizeof(frame_buf));
    if (len < 0) return -1;

    ESP_LOGI(TAG, "Enviando LOCK a actuadores");
    return periph_send_frame(frame_buf, len);
}

int periph_send_unlock(void)
{
    /* WRITE al modulo actuadores: MODULE=0x03, MSGID=0x02 (unlock) */
    uint8_t payload[3];
    payload[0] = MODULE_ACTUATORS;
    payload[1] = 0x02;  /* MSGID: unlock */
    payload[2] = 0x00;  /* sin periodo */

    uint8_t frame_buf[32];
    int len = frame_build(FRAME_TYPE_WRITE, payload, sizeof(payload),
                          frame_buf, sizeof(frame_buf));
    if (len < 0) return -1;

    ESP_LOGI(TAG, "Enviando UNLOCK a actuadores");
    return periph_send_frame(frame_buf, len);
}

bms_power_t periph_get_bms_power(void)
{
    bms_power_t copy;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    copy = s_bms_power;
    xSemaphoreGive(s_data_mutex);
    return copy;
}

bms_status_t periph_get_bms_status(void)
{
    bms_status_t copy;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    copy = s_bms_status;
    xSemaphoreGive(s_data_mutex);
    return copy;
}

lock_state_t periph_get_lock_state(void)
{
    lock_state_t copy;
    xSemaphoreTake(s_data_mutex, portMAX_DELAY);
    copy = s_lock_state;
    xSemaphoreGive(s_data_mutex);
    return copy;
}
