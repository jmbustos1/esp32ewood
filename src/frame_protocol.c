/**
 * @file frame_protocol.c
 * @brief Implementacion del protocolo binario de frames.
 */

#include <string.h>
#include "frame_protocol.h"

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

bool frame_parser_feed(frame_parser_t *p, uint8_t byte)
{
    switch (p->state) {

    case FPARSE_WAIT_SOF0:
        if (byte == FRAME_SOF_0) {
            p->state = FPARSE_WAIT_SOF1;
        }
        break;

    case FPARSE_WAIT_SOF1:
        if (byte == FRAME_SOF_1) {
            p->state = FPARSE_WAIT_LEN_HI;
        } else {
            /* Resincronizar: si este byte es SOF_0, quedarse en WAIT_SOF1 */
            p->state = (byte == FRAME_SOF_0) ? FPARSE_WAIT_SOF1 : FPARSE_WAIT_SOF0;
        }
        break;

    case FPARSE_WAIT_LEN_HI:
        p->len_hi = byte;
        p->state = FPARSE_WAIT_LEN_LO;
        break;

    case FPARSE_WAIT_LEN_LO:
        p->length = ((uint16_t)p->len_hi << 8) | byte;
        p->body_idx = 0;
        if (p->length == 0 || p->length > (1 + FRAME_MAX_PAYLOAD)) {
            /* Length invalido, resincronizar */
            p->state = FPARSE_WAIT_SOF0;
        } else {
            p->state = FPARSE_WAIT_BODY;
        }
        break;

    case FPARSE_WAIT_BODY:
        if (p->body_idx < sizeof(p->body)) {
            p->body[p->body_idx] = byte;
        }
        p->body_idx++;
        if (p->body_idx >= p->length) {
            p->state = FPARSE_WAIT_CHKS_HI;
        }
        break;

    case FPARSE_WAIT_CHKS_HI:
        p->chks_received = (uint16_t)byte << 8;
        p->state = FPARSE_WAIT_CHKS_LO;
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

        if (chks_calc == p->chks_received) {
            return true;  /* Frame valido */
        }
        /* Checksum invalido: descartamos y resincronizamos */
        break;
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
