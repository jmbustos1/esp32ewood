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

/* ── Protocolo MODULE 0x03 (Actuadores) ──────────────────────────── */
/* Layout WRITE: MODULE|SMOD|ACTION|PERIOD                            */
#define ACTUATOR_SMOD_LOCKER  0x01  /* SubModule: candado electronico */
#define ACTUATOR_ACTION_ON    0xFF  /* Activar  (unlock)              */
#define ACTUATOR_ACTION_OFF   0x0F  /* Desactivar (lock)              */

/* ── Estado interno (ultimo valor conocido) ──────────────────────── */
static bms_power_t  s_bms_power  = { .valid = false };
static bms_status_t s_bms_status = { .valid = false };
static lock_state_t s_lock_state = LOCK_STATE_UNKNOWN;
static SemaphoreHandle_t s_data_mutex = NULL;

/* Parser de frames */
static frame_parser_t s_parser;

/* ── Helpers debug ───────────────────────────────────────────────── */

#if FRAME_DEBUG_TX || FRAME_DEBUG_FIELDS
static void log_hex_dump(const char *prefix, const uint8_t *data, size_t len)
{
    char buf[3 * 128 + 1];
    size_t pos = 0;
    size_t limit = len < 128 ? len : 128;
    for (size_t i = 0; i < limit && pos + 4 < sizeof(buf); i++) {
        int n = snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
        if (n <= 0) break;
        pos += n;
    }
    if (pos > 0 && buf[pos - 1] == ' ') buf[pos - 1] = '\0';
    ESP_LOGI(TAG, "%s [%zu bytes]: %s%s", prefix, len, buf, len > 128 ? " ...(truncado)" : "");
}
#endif

/* ── Helpers ─────────────────────────────────────────────────────── */

static void process_bms_soc(const frame_parsed_t *f)
{
    /* MSGID 0x02: Voltaje(2B) + Corriente(2B) + Cap.residual(2B) + SoC(1B) */
    if (f->data_len < 7) {
        ESP_LOGW(TAG, "BMS SoC: data_len=%u < 7", f->data_len);
        return;
    }

#if FRAME_DEBUG_FIELDS
    log_hex_dump("BMS SoC raw DATA", f->data, f->data_len);
#endif

    uint16_t raw_voltage  = ((uint16_t)f->data[0] << 8) | f->data[1];
    int16_t  raw_current  = (int16_t)(((uint16_t)f->data[2] << 8) | f->data[3]);
    uint16_t raw_capacity = ((uint16_t)f->data[4] << 8) | f->data[5];
    uint8_t  soc          = f->data[6];

#if FRAME_DEBUG_FIELDS
    ESP_LOGI(TAG, "  data[0..1] Voltage  = 0x%02X%02X (BE)        -> %u * 10mV = %.2f V",
             f->data[0], f->data[1], raw_voltage, raw_voltage * 0.01f);
    ESP_LOGI(TAG, "  data[2..3] Current  = 0x%02X%02X (BE signed) -> %d * 10mA = %.2f A",
             f->data[2], f->data[3], raw_current, raw_current * 0.01f);
    ESP_LOGI(TAG, "  data[4..5] Capacity = 0x%02X%02X (BE)        -> %u * 10mAh = %.2f Ah",
             f->data[4], f->data[5], raw_capacity, raw_capacity * 0.01f);
    ESP_LOGI(TAG, "  data[6]    SoC      = 0x%02X                -> %u %%",
             f->data[6], soc);
#endif

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

#if FRAME_DEBUG_FIELDS
    log_hex_dump("BMS Status raw DATA", f->data, f->data_len);
    ESP_LOGI(TAG, "  data[0]     Control    = 0x%02X (b0=chg, b1=dischg)", f->data[0]);
    ESP_LOGI(TAG, "  data[1..2]  Protection = 0x%02X%02X", f->data[1], f->data[2]);
    ESP_LOGI(TAG, "  data[3..6]  Balance    = 0x%02X%02X%02X%02X",
             f->data[3], f->data[4], f->data[5], f->data[6]);
#endif

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
     * Layout segun doc: MODULE|SMOD|ACTION|PERIOD.
     * El parser de frame_protocol.c carga posicionalmente:
     *   f->msg_id  = SMOD   (0x01 = LOCKER)
     *   f->period  = ACTION (0xFF = activado/unlocked, 0x0F = desactivado/locked)
     */
#if FRAME_DEBUG_FIELDS
    ESP_LOGI(TAG, "Actuador rsp: SMOD(msg_id)=0x%02X ACTION(period)=0x%02X data_len=%u",
             f->msg_id, f->period, f->data_len);
    if (f->data_len > 0) {
        log_hex_dump("  actuator DATA", f->data, f->data_len);
    }
#endif

    /* Solo nos interesa el submodulo LOCKER por ahora */
    if (f->msg_id != ACTUATOR_SMOD_LOCKER) return;

    lock_state_t new_state = LOCK_STATE_UNKNOWN;
    if (f->period == ACTUATOR_ACTION_ON) {
        new_state = LOCK_STATE_UNLOCKED;
    } else if (f->period == ACTUATOR_ACTION_OFF) {
        new_state = LOCK_STATE_LOCKED;
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
#if FRAME_DEBUG_FIELDS
                ESP_LOGI(TAG, "=== FRAME COMPLETO: TYPE=0x%02X MODULE=0x%02X MSGID=0x%02X PERIOD=0x%02X data_len=%u ===",
                         f.type, f.module_id, f.msg_id, f.period, f.data_len);
#endif
                process_frame(&f);
            }
        }
    }
}

/* ── Envio de frames ─────────────────────────────────────────────── */

static int periph_send_frame(const uint8_t *frame, size_t len)
{
#if FRAME_DEBUG_TX
    log_hex_dump("TX frame", frame, len);
    if (len >= 6) {
        uint16_t length_field = ((uint16_t)frame[2] << 8) | frame[3];
        uint16_t chks_field   = ((uint16_t)frame[len - 2] << 8) | frame[len - 1];
        ESP_LOGI(TAG, "  SOF=%02X%02X LENGTH=0x%04X TYPE=0x%02X CHKS=0x%04X",
                 frame[0], frame[1], length_field, frame[4], chks_field);
    }
#endif
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
    uint8_t payload[4];
    payload[0] = MODULE_ACTUATORS;
    payload[1] = ACTUATOR_SMOD_LOCKER;
    payload[2] = ACTUATOR_ACTION_OFF;  /* lock = desactivar */
    payload[3] = 0x00;

    uint8_t frame_buf[32];
    int len = frame_build(FRAME_TYPE_WRITE, payload, sizeof(payload),
                          frame_buf, sizeof(frame_buf));
    if (len < 0) return -1;

    ESP_LOGI(TAG, "Enviando LOCK a actuadores");
    return periph_send_frame(frame_buf, len);
}

int periph_send_unlock(void)
{
    uint8_t payload[4];
    payload[0] = MODULE_ACTUATORS;
    payload[1] = ACTUATOR_SMOD_LOCKER;
    payload[2] = ACTUATOR_ACTION_ON;  /* unlock = activar */
    payload[3] = 0x00;

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
