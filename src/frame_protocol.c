/**
 * @file frame_protocol.c
 * @brief Implementacion del protocolo binario de frames.
 */

#include <string.h>
#include <stdio.h>
#include "frame_protocol.h"
#include "app_config.h"
#include "esp_log.h"

static const char *FRAME_TAG = "FRAME";

/* ── Fletcher-16 ─────────────────────────────────────────────────── */

uint16_t fletcher16(const uint8_t *data, size_t len)
{
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;

    for (size_t i = 0; i < len; i++) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    return (sum2 << 8) | sum1;
}

/* ── Parser incremental ──────────────────────────────────────────── */

void frame_parser_init(frame_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = FPARSE_WAIT_SOF0;
}

#if FRAME_DEBUG_BYTES
/* Helper: hex dump compacto a un buffer string (devuelve longitud usada) */
static int frame_hex_to_str(const uint8_t *data, size_t len, char *out, size_t out_sz)
{
    size_t pos = 0;
    for (size_t i = 0; i < len && pos + 4 < out_sz; i++) {
        int n = snprintf(out + pos, out_sz - pos, "%02X ", data[i]);
        if (n <= 0) break;
        pos += n;
    }
    if (pos > 0 && out[pos - 1] == ' ') out[pos - 1] = '\0';
    return (int)pos;
}
#endif

bool frame_parser_feed(frame_parser_t *p, uint8_t byte)
{
#if FRAME_DEBUG_BYTES
    const char *state_name = "?";
    switch (p->state) {
    case FPARSE_WAIT_SOF0:    state_name = "WAIT_SOF0";    break;
    case FPARSE_WAIT_SOF1:    state_name = "WAIT_SOF1";    break;
    case FPARSE_WAIT_LEN_HI:  state_name = "WAIT_LEN_HI";  break;
    case FPARSE_WAIT_LEN_LO:  state_name = "WAIT_LEN_LO";  break;
    case FPARSE_WAIT_BODY:    state_name = "WAIT_BODY";    break;
    case FPARSE_WAIT_CHKS_HI: state_name = "WAIT_CHKS_HI"; break;
    case FPARSE_WAIT_CHKS_LO: state_name = "WAIT_CHKS_LO"; break;
    }
#endif

    switch (p->state) {

    case FPARSE_WAIT_SOF0:
        if (byte == FRAME_SOF_0) {
            p->state = FPARSE_WAIT_SOF1;
#if FRAME_DEBUG_BYTES
            ESP_LOGI(FRAME_TAG, "byte 0x%02X  %s -> WAIT_SOF1 (SOF0 match)", byte, state_name);
        } else {
            ESP_LOGD(FRAME_TAG, "byte 0x%02X  %s (ignorado, esperando SOF0)", byte, state_name);
#endif
        }
        break;

    case FPARSE_WAIT_SOF1:
        if (byte == FRAME_SOF_1) {
            p->state = FPARSE_WAIT_LEN_HI;
#if FRAME_DEBUG_BYTES
            ESP_LOGI(FRAME_TAG, "byte 0x%02X  %s -> WAIT_LEN_HI (SOF1 match, SOF completo A5 5A)", byte, state_name);
#endif
        } else {
            /* Resincronizar: si este byte es SOF_0, quedarse en WAIT_SOF1 */
            p->state = (byte == FRAME_SOF_0) ? FPARSE_WAIT_SOF1 : FPARSE_WAIT_SOF0;
#if FRAME_DEBUG_BYTES
            ESP_LOGW(FRAME_TAG, "byte 0x%02X  %s -> %s (SOF1 falla, resincronizando)",
                     byte, state_name,
                     p->state == FPARSE_WAIT_SOF1 ? "WAIT_SOF1" : "WAIT_SOF0");
#endif
        }
        break;

    case FPARSE_WAIT_LEN_HI:
        p->len_hi = byte;
        p->state = FPARSE_WAIT_LEN_LO;
#if FRAME_DEBUG_BYTES
        ESP_LOGI(FRAME_TAG, "byte 0x%02X  %s -> WAIT_LEN_LO (len_hi=0x%02X)", byte, state_name, byte);
#endif
        break;

    case FPARSE_WAIT_LEN_LO:
        p->length = ((uint16_t)p->len_hi << 8) | byte;
        p->body_idx = 0;
        if (p->length == 0 || p->length > (1 + FRAME_MAX_PAYLOAD)) {
            /* Length invalido, resincronizar */
            p->state = FPARSE_WAIT_SOF0;
#if FRAME_DEBUG_BYTES
            ESP_LOGE(FRAME_TAG, "byte 0x%02X  %s -> WAIT_SOF0 (length=0x%04X INVALIDO, max=%d)",
                     byte, state_name, p->length, 1 + FRAME_MAX_PAYLOAD);
#endif
        } else {
            p->state = FPARSE_WAIT_BODY;
#if FRAME_DEBUG_BYTES
            ESP_LOGI(FRAME_TAG, "byte 0x%02X  %s -> WAIT_BODY (LENGTH=0x%04X=%u bytes esperados)",
                     byte, state_name, p->length, p->length);
#endif
        }
        break;

    case FPARSE_WAIT_BODY:
        if (p->body_idx < sizeof(p->body)) {
            p->body[p->body_idx] = byte;
        }
        p->body_idx++;
#if FRAME_DEBUG_BYTES
        ESP_LOGI(FRAME_TAG, "byte 0x%02X  %s body[%u/%u]%s",
                 byte, state_name, p->body_idx - 1, p->length,
                 (p->body_idx == 1) ? " (TYPE)" :
                 (p->body_idx == 2) ? " (MODULE)" :
                 (p->body_idx == 3) ? " (MSGID)" :
                 (p->body_idx == 4) ? " (PERIOD)" : " (DATA)");
#endif
        if (p->body_idx >= p->length) {
            p->state = FPARSE_WAIT_CHKS_HI;
        }
        break;

    case FPARSE_WAIT_CHKS_HI:
        p->chks_received = (uint16_t)byte << 8;
        p->state = FPARSE_WAIT_CHKS_LO;
#if FRAME_DEBUG_BYTES
        ESP_LOGI(FRAME_TAG, "byte 0x%02X  %s -> WAIT_CHKS_LO (CHKS_HI=0x%02X)", byte, state_name, byte);
#endif
        break;

    case FPARSE_WAIT_CHKS_LO: {
        p->chks_received |= byte;

        /* Calcular checksum: sobre LENGTH (2 bytes) + BODY (length bytes) */
        uint8_t chks_data[FRAME_LENGTH_SIZE + 1 + FRAME_MAX_PAYLOAD];
        chks_data[0] = p->len_hi;
        chks_data[1] = (uint8_t)(p->length & 0xFF);
        uint16_t body_copy = (p->length <= sizeof(p->body)) ? p->length : sizeof(p->body);
        memcpy(chks_data + FRAME_LENGTH_SIZE, p->body, body_copy);

        uint16_t chks_calc = fletcher16(chks_data, FRAME_LENGTH_SIZE + body_copy);

        p->state = FPARSE_WAIT_SOF0;

        bool ok = (chks_calc == p->chks_received);

#if FRAME_DEBUG_BYTES
        ESP_LOGI(FRAME_TAG, "byte 0x%02X  WAIT_CHKS_LO -> CHKS recv=0x%04X calc=0x%04X %s",
                 byte, p->chks_received, chks_calc, ok ? "[OK MATCH]" : "[FAIL MISMATCH]");
        /* Dump bytes usados en el calculo */
        char hex_buf[3 * (FRAME_LENGTH_SIZE + 1 + FRAME_MAX_PAYLOAD) + 1];
        frame_hex_to_str(chks_data, FRAME_LENGTH_SIZE + body_copy, hex_buf, sizeof(hex_buf));
        ESP_LOGI(FRAME_TAG, "       chks input (%u bytes): %s",
                 (unsigned)(FRAME_LENGTH_SIZE + body_copy), hex_buf);
        /* Dump body completo (TYPE+PAYLOAD) */
        char body_buf[3 * (1 + FRAME_MAX_PAYLOAD) + 1];
        frame_hex_to_str(p->body, body_copy, body_buf, sizeof(body_buf));
        ESP_LOGI(FRAME_TAG, "       body completo: %s", body_buf);
#endif

        if (ok) {
            return true;  /* Frame valido */
        }

#if FRAME_ACCEPT_BAD_CHKS
        ESP_LOGW(FRAME_TAG, "CHKS invalido pero FRAME_ACCEPT_BAD_CHKS=1 -> procesando igual (DEBUG)");
        return true;
#else
        /* Checksum invalido: descartamos y resincronizamos */
        break;
#endif
    }
    }

    return false;
}

frame_parsed_t frame_parser_get_result(const frame_parser_t *p)
{
    frame_parsed_t f;
    memset(&f, 0, sizeof(f));

    f.length = p->length;
    f.type = p->body[0];

    /* PAYLOAD = body[1..length-1] → module_id, msg_id, period, data... */
    if (p->length >= 2) {
        f.module_id = p->body[1];
    }
    if (p->length >= 3) {
        f.msg_id = p->body[2];
    }
    if (p->length >= 4) {
        f.period = p->body[3];
    }
    if (p->length > 4) {
        f.data_len = p->length - 4;  /* TYPE(1) + MODULE(1) + MSGID(1) + PERIOD(1) = 4 */
        if (f.data_len > FRAME_MAX_PAYLOAD) f.data_len = FRAME_MAX_PAYLOAD;
        memcpy(f.data, p->body + 4, f.data_len);
    }

    return f;
}

/* ── Construccion de frames ──────────────────────────────────────── */

int frame_build(uint8_t type, const uint8_t *payload, uint16_t payload_len,
                uint8_t *out_buf, size_t out_size)
{
    /* LENGTH = 1 (TYPE) + payload_len */
    uint16_t length = 1 + payload_len;
    size_t total = FRAME_SOF_SIZE + FRAME_LENGTH_SIZE + length + FRAME_CHECKSUM_SIZE;

    if (total > out_size) return -1;

    /* SOF */
    out_buf[0] = FRAME_SOF_0;
    out_buf[1] = FRAME_SOF_1;

    /* LENGTH (Big Endian) */
    out_buf[2] = (uint8_t)(length >> 8);
    out_buf[3] = (uint8_t)(length & 0xFF);

    /* TYPE */
    out_buf[4] = type;

    /* PAYLOAD */
    if (payload_len > 0 && payload != NULL) {
        memcpy(out_buf + 5, payload, payload_len);
    }

    /* CHECKSUM: sobre LENGTH (2B) + TYPE (1B) + PAYLOAD (NB) */
    uint16_t chks = fletcher16(out_buf + 2, FRAME_LENGTH_SIZE + length);
    out_buf[total - 2] = (uint8_t)(chks >> 8);
    out_buf[total - 1] = (uint8_t)(chks & 0xFF);

    return (int)total;
}

int frame_build_read(uint8_t module_id, uint8_t msg_id, uint8_t period,
                     uint8_t *out_buf, size_t out_size)
{
    uint8_t payload[3];
    payload[0] = module_id;
    payload[1] = msg_id;
    payload[2] = period;

    return frame_build(FRAME_TYPE_READ, payload, sizeof(payload), out_buf, out_size);
}
