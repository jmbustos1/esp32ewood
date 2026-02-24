# Contexto del proyecto esp32ewood — Skill / Oneshot

Documento de contexto para retomar el proyecto sin perder el hilo. Incluir este archivo (o su ruta) al pedir cambios o continuar desarrollo.

---

## 1. Qué es el proyecto

- **Objetivo:** Dispositivo IoT en un scooter que se comunica con un servidor en la nube por 4G.
- **Hardware:** ESP32 (DOIT DEVKIT V1) + módulo celular **SIM7600** (UART).
- **Framework:** ESP-IDF vía PlatformIO.
- **Funciones principales:**
  - Enviar al servidor telemetría cada 5 s: posición (lat/lon), batería, velocidad (JSON).
  - Recibir comandos del servidor (unlock, lock) y ejecutarlos.
  - Enviar ACK inmediato en NDJSON por cada comando recibido.

---

## 2. Comunicación SIM7600 (UART)

- **UART usado:** `UART_NUM_2`. **Pines:** TX = GPIO 17, RX = GPIO 16.
- **Modo:** No transparente, con **modo buffer** (`AT+CIPRXGET=1`). Los datos entrantes no van directo al UART; el módulo los guarda y notifica con URC `+CIPRXGET: 1,<link>`. El firmware consulta con `AT+CIPRXGET=4,<link>` (longitud) y `AT+CIPRXGET=2,<link>,<len>` (leer datos).
- **Flujo de envío:** Enviar AT (ej. `AT+CIPSEND=0,<len>`), esperar prompt **">"** del SIM7600, escribir los bytes del mensaje en el UART. El ">" es la señal de “listo para recibir datos”.
- **Sincronización:** `uart_mutex` protege el UART durante secuencias AT completas. La tarea async solo detecta URCs y marca `rx_pending[]`; el drenaje real se hace en el loop principal con `sim7600_drain_rx_buffer()`.

---

## 3. Máquina de estados (main.c)

Sí hay máquina de estados. El tipo y la variable son:

```c
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

static boot_state_t current_state = STATE_BOOT;
```

**Estados que se usan realmente en el loop:**

| Estado            | Dónde se asigna | Significado |
|-------------------|-----------------|-------------|
| `STATE_RUN_IDLE`  | Inicio del loop, tras drenar, tras enviar | Sin operación crítica en curso |
| `STATE_RX_DRAIN`  | Cuando `rest = sim7600_get_rx_buffer_length(0) > 0` | Se está drenando el buffer RX del socket |
| `STATE_TX_PREPARE`| Justo antes de `sim7600_send_scooter_update()` | Se va a enviar telemetría |

**Transiciones en `sim7600_scooter_update_loop`:**

1. Al entrar al `while(1)`: `current_state == STATE_RUN_IDLE` (se fija una vez antes del loop).
2. Si `current_state != STATE_RX_DRAIN` y hay datos (`rest > 0`): se pone `current_state = STATE_RX_DRAIN`, se llama `sim7600_drain_rx_buffer(0)`, luego `current_state = STATE_RUN_IDLE`.
3. Si `!rx_pending[0]`: se pone `current_state = STATE_TX_PREPARE`, se llama `sim7600_send_scooter_update(...)`, luego `current_state = STATE_RUN_IDLE`.
4. El resto de estados (`STATE_BOOT`, `STATE_NET_UP`, `STATE_SOCKET_CONNECTING`, `STATE_TX_WAIT_PROMPT`, `STATE_TX_SENDING`, `STATE_TX_WAIT_RESULT`, `STATE_ERROR_RECOVER`) están definidos pero **no se asignan** en el flujo actual; el código solo imprime "OTRO" si el estado no es RUN_IDLE, RX_DRAIN o TX_PREPARE.

---

## 4. Loop principal (código “duro”)

En `sim7600_scooter_update_loop`, dentro del `while(1)`:

```text
1. Log ciclo, mensajes recibidos, estado actual.

2. PRIORIDAD 1 — Drenar RX (solo si current_state != STATE_RX_DRAIN):
   - xSemaphoreTake(uart_mutex, portMAX_DELAY)
   - rest = sim7600_get_rx_buffer_length(0)   // asume mutex tomado
   - Si rest > 0:
     - current_state = STATE_RX_DRAIN
     - sim7600_drain_rx_buffer(0)             // asume mutex tomado
     - rx_pending[0] = false
     - current_state = STATE_RUN_IDLE
   - Si no rest pero rx_pending[0]: rx_pending[0] = false
   - xSemaphoreGive(uart_mutex)

3. PRIORIDAD 3 — Enviar telemetría (si !rx_pending[0]):
   - location_index alterna 0/1
   - current_state = STATE_TX_PREPARE
   - result = sim7600_send_scooter_update(0, SCOOTER_ID, lat, lon, battery, speed)
   - current_state = STATE_RUN_IDLE
   - Si result != OK: reconexión (cipclose, netopen si hace falta, cipopen_tcp, reintento de send).

4. Espera antes del siguiente ciclo:
   - wait_ms = UPDATE_INTERVAL_SEC * 1000
   - while (wait_ms > 0): vTaskDelay(RX_POLL_MS), wait_ms -= RX_POLL_MS; si rx_pending[0] break.
```

---

## 5. Mutex UART — quién toma y quién asume tomado

| Función | Mutex |
|--------|--------|
| `sim7600_send_command()` | Toma al entrar, libera en todos los return (OK, ERROR, TIMEOUT). |
| `sim7600_wait_for_response()` | **No** toma; asume que el llamador ya tiene el mutex. |
| `sim7600_get_rx_buffer_length()` | **No** toma; asume que el llamador (p. ej. `sim7600_drain_rx_buffer`) ya tiene el mutex. Evita deadlock con `sim7600_send_command`. |
| `sim7600_read_rx_buffer()` | **No** toma; mismo criterio. |
| `sim7600_drain_rx_buffer()` | **Toma** al inicio, **libera** en todos los return (éxito, error, MAX_ITERATIONS). |
| `sim7600_cipsend()` | **Toma** al inicio, **libera** al devolver (OK o ERROR). |

La tarea `sim7600_async_read_task` toma el mutex con timeout corto (50 ms), lee UART para detectar URCs, y libera; no llama a `drain` ni a `send_command` con mutex tomado.

---

## 6. Esqueleto de main.c (orden y roles)

Aproximado por bloques/funciones para ubicar código:

| Líneas aprox. | Contenido |
|---------------|-----------|
| 1–108        | Includes, defines (UART_SIM, pines, BUF_SIZE, timeouts), `boot_state_t`, `current_state`, `rx_pending[]`, `rest_len[]`, `buffer_mode_active`, config scooter (SCOOTER_ID, host, port, UPDATE_INTERVAL_SEC, RX_POLL_MS), `location_t`, `sim7600_response_t`, prototipos. |
| 110–385      | `sim7600_send_command`, `sim7600_wait_for_response` (lógica del ">", URC +CIPRXGET en buffer). |
| 397          | `sim7600_send` (envío crudo sin respuesta). |
| 418–620      | PDP, CIPMODE, NETOPEN/NETCLOSE, DNS. |
| 658–675      | `sim7600_set_buffer_mode`. |
| 683–780      | `sim7600_get_rx_buffer_length` (AT+CIPRXGET=4, parseo +CIPRXGET). |
| 784–893      | `sim7600_read_rx_buffer` (AT+CIPRXGET=2, leer datos; usa `read_len` del header). |
| 896–982      | `sim7600_drain_rx_buffer` (loop: get_rx_buffer_length → read_rx_buffer en chunks, parseo NDJSON por `\n`, `sim7600_process_server_command`; delay 15 ms; mutex tomado por la función). |
| 995–1092     | `sim7600_cipopen_tcp_transparent`, `sim7600_cipopen_tcp`, `sim7600_cipclose`. |
| 1098–1192    | `sim7600_cipsend` (AT+CIPSEND, esperar ">", detectar URC en buffer, uart_write_bytes datos, esperar +CIPSEND:, dar mutex). |
| 1215–1248    | `sim7600_read_data_non_transparent` (legacy/helper; no se usa en flujo buffer actual para datos de aplicación). |
| 1307–1471    | `sim7600_process_server_command` (parseo JSON, dedup por request_id, extracción command/timestamp/client_id/request_id/command_id, encolar en command_queue o fallback ACK). |
| 1476–1628    | `sim7600_async_read_task` (solo URCs: +CIPRXGET: 1,, +IPCLOSE, +CIPERROR; filtrar respuestas AT; marcar rx_pending). |
| 1632–1684    | `sim7600_command_processor_task` (xQueueReceive, unlock/lock, sim7600_send_ack). |
| 1689–1788    | Helpers transparentes, `sim7600_tcp_ping_test`. |
| 1789–1878    | `sim7600_send_ack` (armar NDJSON, sim7600_cipsend). |
| 1887–1942    | `sim7600_send_scooter_update` (snprintf JSON telemetría, sim7600_cipsend, delay 80 ms). |
| 1946–2190    | `sim7600_scooter_update_loop` (init: cipmode, buffer mode, netopen, DNS, cipopen_tcp, mutex, command_queue, command_processor task, async_read task; luego while(1) con prioridad RX drain, luego TX send, luego wait en chunks). |
| 2193–2272    | `app_main` (uart config, AT test, PDP, luego `sim7600_scooter_update_loop(SCOOTER_TCP_HOST, SCOOTER_TCP_PORT)` que no retorna). |

---

## 7. Variables globales / estáticas clave

- **UART / sync:** `uart_mutex`, `shared_response_buffer[BUF_SIZE]`, `shared_data_buffer[BUF_SIZE]`.
- **Estado:** `current_state` (boot_state_t), `rx_pending[MAX_LINKS]`, `rest_len[MAX_LINKS]`, `buffer_mode_active`.
- **Comandos:** `command_queue` (QueueHandle_t), `static_cmd_item` (command_queue_item_t), `server_command_buffer[512]`, `processed_request_ids[][]`, `processed_request_count`, `processed_request_index`, `server_messages_received`.
- **Config:** `locations[]`, constantes SCOOTER_*, SCOOTER_TCP_HOST/PORT, UPDATE_INTERVAL_SEC, RX_POLL_MS.

---

## 8. Constantes y configuración

| Constante / define        | Valor / uso |
|---------------------------|-------------|
| `UART_SIM`                | `UART_NUM_2` |
| `UART_TX` / `UART_RX`     | GPIO 17 / 16 |
| `SCOOTER_ID`              | 1 |
| `SCOOTER_TCP_HOST`        | "3.237.198.242" |
| `SCOOTER_TCP_PORT`        | 8201 |
| `UPDATE_INTERVAL_SEC`     | 5 |
| `RX_POLL_MS`              | 150 |
| `BUF_SIZE`                | 1024 |
| `RESPONSE_TIMEOUT_MS`     | 5000 |
| `MAX_LINKS`               | 10 |
| `COMMAND_QUEUE_SIZE`      | 20 |
| `MAX_PROCESSED_REQUESTS`  | 20 (dedup request_id) |
| `RX_CHUNK_SIZE`           | 256 (dentro de `sim7600_drain_rx_buffer`) |
| `MAX_ITERATIONS`          | 100 (drain, protección loop infinito) |
| `PENDING_ACK_QUEUE_SIZE` | 10 (obsoleto, no se usa) |

---

## 9. Estructuras de datos

**`command_queue_item_t`** (elemento de la cola de comandos):
- `command[256]`, `timestamp[64]`, `client_id[64]`, `request_id[64]`, `command_id[64]`, `message_id` (int).
- Se rellena en `sim7600_process_server_command` con `static_cmd_item` y se envía con `xQueueSend(..., 0)` (no bloqueante).

**`location_t`**:
- `latitude` (double), `longitude` (double).
- `locations[2]` con coordenadas fijas Concepción, Chile.

**`pending_ack_item_t`** y cola `pending_ack_queue`: obsoletos (modo buffer no los usa).

**Deduplicación:** `processed_request_ids[MAX_PROCESSED_REQUESTS][64]`, índice circular `processed_request_index`, `processed_request_count`. Se usa en `process_server_command` para ignorar comandos con el mismo `request_id` ya procesado.

---

## 10. Detalle de funciones clave

**`sim7600_wait_for_response(expected, response, response_size, timeout_ms)`**
- Asume mutex tomado. Lee UART en bloques (timeout 100 ms por lectura), hasta recibir `expected` o ERROR/TIMEOUT.
- Si `expected == ">"`: busca "AT+CIPSEND" en buffer, luego ">" en línea separada; descarta ">" que sea parte de "+IPD", "RECV FROM:", "+CIPRXGET:". Si detecta URC "+CIPRXGET: 1,<link>" lo marca en `rx_pending[link_num]`.
- Para "+NETOPEN:" exige código 0; para "CONNECT" rechaza "CONNECT FAIL". Devuelve SIM7600_OK, SIM7600_ERROR o SIM7600_TIMEOUT.

**`sim7600_get_rx_buffer_length(link_num)`**
- Envía `AT+CIPRXGET=4,<link_num>\r\n`, lee hasta OK/ERROR (timeout RESPONSE_TIMEOUT_MS). Parsea `+CIPRXGET: 4,<link>,<rest>` o `+CIPRXGET:4,<link>,<rest>` (con/sin espacio). Si respuesta "No data", devuelve 0. Actualiza `rest_len[link_num]`. Devuelve rest (-1 si error).

**`sim7600_read_rx_buffer(link_num, len, buffer, buffer_size)`**
- Envía `AT+CIPRXGET=2,<link_num>,<len>\r\n`. Respuesta: `+CIPRXGET: 2,<link>,<read_len>,<rest_len>\r\n<datos>\r\nOK`. Parsea el header para obtener `read_len` y `data_start`; copia datos hasta `\r\nOK`. Timeout 10 s, lecturas 100 ms. No toma mutex.

**`sim7600_drain_rx_buffer(link_num)`**
- Toma mutex al entrar. Loop: `rest = sim7600_get_rx_buffer_length(link_num)`; si rest==0 sale y devuelve OK; si rest>0 lee chunk min(rest, RX_CHUNK_SIZE) con `sim7600_read_rx_buffer`, parsea por `\n` cada frame y llama `sim7600_process_server_command(frame_start, frame_len)`; delay 15 ms. Máximo MAX_ITERATIONS (100). Libera mutex en todos los return.

**`sim7600_process_server_command(data, len)`**
- Si no hay '{', busca "unlock"/"lock" en texto plano y retorna. Si hay JSON: copia a `server_command_buffer`, extrae `request_id` con macro `EXTRACT_JSON_FIELD`; si ya está en `processed_request_ids` retorna; si no, lo añade al cache circular. Extrae command, timestamp, client_id, command_id; rellena `static_cmd_item` y hace `xQueueSend(command_queue, &static_cmd_item, 0)`. Si no hay cola, fallback: envía ACK directo para unlock/lock.

**`sim7600_send_command(cmd, response, response_size, timeout_ms)`**
- Toma mutex, escribe cmd, lee hasta OK/ERROR en línea (precedido por \r\n o inicio), libera mutex y retorna SIM7600_OK/ERROR/TIMEOUT. No hace flush del UART (para no perder URCs).

---

## 11. Secuencias de inicialización

**`app_main` (orden exacto):**
1. `uart_config`: 115200, 8N1, sin flow control. `uart_driver_install(UART_SIM, BUF_SIZE*2, 0, 0, NULL, 0)`, `uart_param_config`, `uart_set_pin(UART_SIM, UART_TX, UART_RX, ...)`.
2. `vTaskDelay(2000)`.
3. `sim7600_send_command("AT\r\n", ...)`; si falla return.
4. `vTaskDelay(1000)`.
5. `sim7600_test_pdp_context` (AT+CGDCONT=?).
6. `vTaskDelay(1000)`.
7. `sim7600_set_pdp_context(1, "IP", "bam.entelpcs.cl", NULL, 0, 0)`.
8. `vTaskDelay(1000)`.
9. `sim7600_read_pdp_context`.
10. `vTaskDelay(2000)`.
11. `sim7600_scooter_update_loop(SCOOTER_TCP_HOST, SCOOTER_TCP_PORT)` (no retorna).

**`sim7600_scooter_update_loop` (init antes del while(1)):**
1. `sim7600_set_cipmode(0)` (no transparente). `vTaskDelay(1000)`.
2. `sim7600_set_buffer_mode(1)`. `vTaskDelay(1000)`.
3. `sim7600_netopen()`. `vTaskDelay(3000)`, `vTaskDelay(2000)`.
4. `sim7600_set_dns("8.8.8.8", "8.8.4.4")`. `vTaskDelay(2000)`.
5. `sim7600_cipopen_tcp(0, server_ip, server_port)`. `vTaskDelay(1000)`.
6. Si `uart_mutex == NULL`: `uart_mutex = xSemaphoreCreateMutex()`.
7. `command_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(command_queue_item_t))`.
8. `command_processor_active = true`; `xTaskCreate(sim7600_command_processor_task, "cmd_processor", 4096, NULL, 4, NULL)`.
9. `async_read_active = true`; `xTaskCreate(sim7600_async_read_task, "sim7600_async_read", 4096, NULL, 5, NULL)`.
10. `vTaskDelay(300)`. Luego `current_state = STATE_RUN_IDLE` y `while(1)`.

**Reconexión (cuando `sim7600_send_scooter_update` devuelve != OK):**
- `sim7600_cipclose(0)`, `vTaskDelay(2000)`.
- `sim7600_netopen_status(net_status, ...)`; si no hay "+NETOPEN: 0" en net_status → `sim7600_netopen()`, `vTaskDelay(3000)`.
- `sim7600_cipopen_tcp(0, server_ip, server_port)`. Si OK, reintenta `sim7600_send_scooter_update` con la misma ubicación.

---

## 12. Formatos AT y respuestas parseadas

- **OK/ERROR:** En `sim7600_send_command` y `sim7600_get_rx_buffer_length`: se buscan en líneas (precedidos por \r\n o inicio de buffer).
- **+CIPRXGET: 4,<link>,<rest_len>** o **+CIPRXGET:4,...**: en `get_rx_buffer_length`; también "No data".
- **+CIPRXGET: 2,<link>,<read_len>,<rest_len>\r\n<data>\r\nOK**: en `read_rx_buffer`; se busca el header con sscanf y el inicio de datos tras `\r\n` del header.
- **">"**: en `wait_for_response`; debe estar en línea separada, tras "AT+CIPSEND", y no ser parte de +IPD, RECV FROM:, +CIPRXGET:.
- **+CIPSEND: <link>,<reqSendLength>,<cnfSendLength>**: en `cipsend` tras enviar datos; se comprueba req_len == cnf_len.
- **+NETOPEN: <err>**: err 0 éxito; "Network is already opened" se trata como éxito en `netopen`.
- **+CIPOPEN: <link>,<err>**: err 0 éxito en `cipopen_tcp`.
- **+CIPERROR: <code>**: se detecta en `wait_for_response` y se retorna ERROR.

**Telemetría enviada (JSON una línea):**
- `{"scooter_id":%d,"latitude":%.6f,"longitude":%.6f,"battery_level":%d,"speed_kmh":%.1f}\n`

**ACK (NDJSON):**
- `{"id":"<command_id>","type":"ack","status":"ok","command":"<unlock|lock>","original_timestamp":"<ts>"[,"client_id":"..."][,"request_id":"..."]}\n`

---

## 13. Archivos importantes

- **`src/main.c`** — Todo el firmware (~2272 líneas).
- **`platformio.ini`** — `esp32doit-devkit-v1`, `espressif32`, `espidf`.
- **`docs/FLUJO_PRESENTACION.md`** — Flujo para presentaciones (no técnico).
- **`docs/backups/main_estable.c`** — Respaldo de main.c estable.

---

## 14. Comandos útiles

- **Compilar:** `pio run`
- **Flashear:** `pio run --target upload` (opcional `--upload-port /dev/ttyACM0`).
- **Monitor serial:** `pio device monitor -p /dev/ttyACM0 -b 115200`.

---

## 15. Estado actual y próximos pasos

- **Código legacy:** Eliminado; solo modo buffer.
- **Recepción:** Polling RX_POLL_MS, drenaje 15 ms entre chunks, URC 15 ms en async task.
- **Comandos:** unlock y lock (cola + ACK NDJSON).
- **APN:** `bam.entelpcs.cl` (PDP contexto 1).
- **Próximo paso conocido:** Integrar BMS por UART (UART1 en GPIOs libres, p. ej. 4 y 5).

---

*Documento pensado para oneshot y continuidad del proyecto; incluye máquina de estados, flujo del loop, reglas de mutex, estructuras, detalle de funciones clave, secuencias de init y formatos AT tal como están en main.c.*
