# Contexto del proyecto esp32ewood

Documento de contexto para retomar el proyecto sin perder el hilo. Actualizado el 2026-03-16 basándose en el código fuente real.

---

## 1. Qué es el proyecto

- **Objetivo:** Dispositivo IoT montado en un scooter que se comunica con un servidor en la nube por 4G.
- **Hardware:** ESP32 (DOIT DEVKIT V1) + módulo celular **SIM7600** (UART).
- **Framework:** ESP-IDF vía PlatformIO.
- **Funciones principales:**
  - Enviar telemetría al servidor cada 5 s: posición (lat/lon), batería, velocidad (JSON).
  - Recibir comandos del servidor (unlock, lock) y ejecutarlos.
  - Enviar ACK inmediato en NDJSON por cada comando recibido.
  - Reconexión automática con máquina de estados y backoff.

---

## 2. Estructura de archivos

```text
src/
  app_config.h       — Configuración centralizada (UART, pines, constantes, backoff).
  sim7600.h          — API pública del driver SIM7600 (tipos, prototipos).
  sim7600.c          — Implementación completa del driver (AT, buffer, TCP, comandos, loop).
  connectivity.h     — Máquina de estados de conectividad (enum, prototipos).
  connectivity.c     — Implementación de recuperación con backoff.
  main.c             — app_main: init UART, prueba AT, PDP, arranca loop.
docs/
  CONTEXTO_PROYECTO.md   — Este archivo.
  DISEÑO_FALLAS_Y_MODULOS.md — Diseño original de fallas y separación en módulos.
  FLUJO_PRESENTACION.md  — Flujo simplificado para audiencia no técnica.
```

---

## 3. Comunicación SIM7600 (UART)

- **UART:** `UART_NUM_2`. **Pines:** TX = GPIO 17, RX = GPIO 16. **Baudrate:** 115200.
- **Modo:** No transparente (`AT+CIPMODE=0`), con **modo buffer** (`AT+CIPRXGET=1`).
- **Recepción:** Los datos entrantes no van directo al UART; el módulo los guarda y notifica con URC `+CIPRXGET: 1,<link>`. El firmware consulta con `AT+CIPRXGET=4,<link>` (longitud) y `AT+CIPRXGET=2,<link>,<len>` (leer datos).
- **Envío:** `AT+CIPSEND=0,<len>` → esperar prompt `>` → escribir bytes → esperar `+CIPSEND: <link>,<req>,<cnf>`.
- **Sincronización:** `uart_mutex` (SemaphoreHandle_t) protege el UART. Ver sección de mutex más abajo.

---

## 4. Dos máquinas de estados

El código tiene **dos** máquinas de estados independientes:

### 4.1 Máquina de conectividad (`connectivity_state_t` en connectivity.c)

Controla la **recuperación de conexión**. Es la máquina principal de fallas.

```c
typedef enum {
    CONNECTIVITY_MODULE_OFFLINE,   // Módulo no responde por UART
    CONNECTIVITY_MODULE_READY,     // AT OK; falta configurar red/socket
    CONNECTIVITY_NET_DOWN,         // Red no abierta (NETOPEN falló)
    CONNECTIVITY_NET_READY,        // Red abierta; falta socket
    CONNECTIVITY_SOCKET_DOWN,      // Socket cerrado; reintentar CIPOPEN
    CONNECTIVITY_RUNNING           // Red + socket OK; loop normal
} connectivity_state_t;
```

Estado almacenado en `s_state` (static en connectivity.c).

### 4.2 Máquina interna del loop (`boot_state_t` en sim7600.h)

Controla qué **operación** está haciendo el loop en cada momento.

```c
typedef enum {
    STATE_BOOT, STATE_NET_UP, STATE_SOCKET_CONNECTING,
    STATE_RUN_IDLE, STATE_TX_PREPARE, STATE_TX_WAIT_PROMPT,
    STATE_TX_SENDING, STATE_TX_WAIT_RESULT, STATE_RX_DRAIN,
    STATE_ERROR_RECOVER
} boot_state_t;
```

**Solo 3 estados se usan realmente en el loop:**

| Estado | Significado | Dónde se asigna |
|--------|-------------|-----------------|
| `STATE_RUN_IDLE` | Sin operación crítica en curso | Inicio, tras drenar, tras enviar |
| `STATE_RX_DRAIN` | Drenando buffer RX | Cuando `get_rx_buffer_length(0) > 0` |
| `STATE_TX_PREPARE` | Enviando telemetría | Justo antes de `send_scooter_update()` |

Los demás estados (`STATE_BOOT`, `STATE_NET_UP`, `STATE_SOCKET_CONNECTING`, `STATE_TX_WAIT_PROMPT`, `STATE_TX_SENDING`, `STATE_TX_WAIT_RESULT`, `STATE_ERROR_RECOVER`) están definidos pero **nunca se asignan**.

---

## 5. Loop principal (`sim7600_scooter_update_loop` en sim7600.c)

### 5.1 Inicialización (antes del while(1))

1. `connectivity_init()` — estado a MODULE_OFFLINE, resetea backoff.
2. Crear `uart_mutex` si no existe.
3. `sim7600_set_cipmode(0)` — modo no transparente.
4. `sim7600_set_buffer_mode(1)` — activar modo buffer.
5. `sim7600_netopen()` — activar red. Delays 3s + 2s.
6. `sim7600_set_dns("8.8.8.8", "8.8.4.4")`. Delay 2s.
7. `sim7600_cipopen_tcp(0, server_ip, server_port)`. Delay 1s.
8. Crear `command_queue` (xQueueCreate, tamaño COMMAND_QUEUE_SIZE=20).
9. Crear tarea `sim7600_command_processor_task` (prioridad 4, stack 4096).
10. Crear tarea `sim7600_async_read_task` (prioridad 5, stack 4096).
11. `connectivity_set_state(CONNECTIVITY_RUNNING)`.
12. `current_state = STATE_RUN_IDLE`.

### 5.2 Loop while(1)

```text
1. update_count++

2. Si connectivity != RUNNING:
   → connectivity_step_recovery(server_ip, server_port)
   → connectivity_wait_backoff()
   → continue

3. Health-check cada CONNECTIVITY_NET_CHECK_CYCLES (10) ciclos:
   - AT → si TIMEOUT → connectivity_notify_at_timeout()
         → si OK → connectivity_notify_at_ok()
   - Si ya no es RUNNING → continue
   - AT+NETOPEN? → si "+NETOPEN: 0" → connectivity_notify_send_failed(true)
   - Si ya no es RUNNING → continue

4. PRIORIDAD 1 — Drenar RX:
   - xSemaphoreTake(uart_mutex)
   - rest = sim7600_get_rx_buffer_length(0)
   - Si rest > 0:
     - current_state = STATE_RX_DRAIN
     - sim7600_drain_rx_buffer(0)
     - rx_pending[0] = false
     - current_state = STATE_RUN_IDLE
   - Si rx_pending[0] pero rest==0: limpiar flag
   - xSemaphoreGive(uart_mutex)

5. PRIORIDAD 3 — Enviar telemetría (si !rx_pending[0]):
   - Alternar ubicación (locations[0] o locations[1])
   - current_state = STATE_TX_PREPARE
   - sim7600_send_scooter_update(0, ...)
   - current_state = STATE_RUN_IDLE
   - Si falla: AT+NETOPEN? → decidir si NET_DOWN o SOCKET_DOWN
     → connectivity_notify_send_failed(network_error)

6. Espera (UPDATE_INTERVAL_SEC * 1000 ms):
   - Despertar cada RX_POLL_MS (150 ms)
   - Si rx_pending[0] → break (drenar primero)
```

---

## 6. Mutex UART — quién toma y quién asume tomado

| Función | Mutex |
|---------|-------|
| `sim7600_send_command()` | **Toma** al entrar, **libera** en todos los return |
| `sim7600_wait_for_response()` | **No toma**; asume llamador tiene mutex |
| `sim7600_get_rx_buffer_length()` | **No toma**; asume llamador tiene mutex |
| `sim7600_read_rx_buffer()` | **No toma**; asume llamador tiene mutex |
| `sim7600_drain_rx_buffer()` | **No toma**; asume llamador tiene mutex |
| `sim7600_cipsend()` | **Toma** al entrar, **libera** en todos los return |
| `sim7600_async_read_task` | Toma con timeout 50 ms, libera inmediatamente |

**NOTA:** `sim7600_netopen()`, `sim7600_cipopen_tcp()`, `sim7600_cipclose()` escriben al UART directamente y llaman a `sim7600_wait_for_response()` **sin tomar el mutex**. Esto es un riesgo potencial (ver sección de bugs).

---

## 7. Tareas FreeRTOS

| Tarea | Prioridad | Stack | Función |
|-------|-----------|-------|---------|
| `app_main` (implícita) | — | — | Init + loop principal (nunca retorna) |
| `sim7600_async_read_task` | 5 | 4096 | Detectar URCs (+CIPRXGET, +IPCLOSE, +CIPERROR) |
| `sim7600_command_processor_task` | 4 | 4096 | Procesar cola de comandos, enviar ACKs |

---

## 8. Variables globales/estáticas clave (sim7600.c)

| Variable | Tipo | Uso |
|----------|------|-----|
| `uart_mutex` | SemaphoreHandle_t | Proteger UART |
| `shared_response_buffer[BUF_SIZE]` | char | Buffer compartido para respuestas AT |
| `shared_data_buffer[BUF_SIZE]` | uint8_t | Buffer compartido para datos |
| `current_state` | boot_state_t | Estado del loop (RUN_IDLE/RX_DRAIN/TX_PREPARE) |
| `rx_pending[MAX_LINKS]` | bool | Flag de datos pendientes por link |
| `rest_len[MAX_LINKS]` | int | Bytes restantes por link |
| `buffer_mode_active` | bool | Si modo buffer está activo |
| `locations[2]` | location_t | Coordenadas fijas (Concepción, Chile) |
| `command_queue` | QueueHandle_t | Cola de comandos del servidor |
| `static_cmd_item` | command_queue_item_t | Buffer estático para encolar comandos |
| `server_command_buffer[512]` | char | Buffer para JSON del servidor |
| `processed_request_ids[20][64]` | char | Cache circular de deduplicación |
| `server_messages_received` | int | Contador de mensajes recibidos |
| `async_read_active` | bool | Control de tarea async |
| `command_processor_active` | bool | Control de tarea procesadora |

---

## 9. Constantes y configuración (app_config.h)

| Constante | Valor | Uso |
|-----------|-------|-----|
| `UART_SIM` | UART_NUM_2 | Puerto UART del SIM7600 |
| `UART_TX` / `UART_RX` | GPIO 17 / 16 | Pines UART |
| `BUF_SIZE` | 1024 | Tamaño de buffers |
| `RESPONSE_TIMEOUT_MS` | 5000 | Timeout respuestas AT |
| `MAX_LINKS` | 10 | Máximo de conexiones simultáneas |
| `SCOOTER_ID` | 1 | ID del scooter |
| `SCOOTER_BATTERY` | 85 | Nivel batería (fijo) |
| `SCOOTER_SPEED_KMH` | 15.5f | Velocidad (fija) |
| `UPDATE_INTERVAL_SEC` | 5 | Intervalo de telemetría |
| `RX_POLL_MS` | 150 | Polling de datos RX |
| `SCOOTER_TCP_HOST` | "98.92.176.224" | IP del servidor |
| `SCOOTER_TCP_PORT` | 8201 | Puerto del servidor |
| `CONNECTIVITY_AT_RETRY_MS` | 30000 | T1: retry AT (módulo offline) |
| `CONNECTIVITY_NET_RETRY_MS` | 30000 | T2: retry NETOPEN |
| `CONNECTIVITY_SOCKET_BACKOFF_LEN` | 5 | Pasos backoff socket |
| `CONNECTIVITY_AT_TIMEOUT_COUNT` | 3 | Timeouts AT → MODULE_OFFLINE |
| `CONNECTIVITY_NET_CHECK_CYCLES` | 10 | Cada N ciclos, health-check |
| `COMMAND_QUEUE_SIZE` | 20 | Tamaño cola de comandos |
| `MAX_PROCESSED_REQUESTS` | 20 | Dedup request_id |
| `PENDING_ACK_QUEUE_SIZE` | 10 | Obsoleto |

---

## 10. Formatos de mensajes

**Telemetría (JSON, una línea):**
```json
{"scooter_id":1,"latitude":-36.820100,"longitude":-73.044400,"battery_level":85,"speed_kmh":15.5}
```

**ACK (NDJSON):**
```json
{"id":"<command_id>","type":"ack","status":"ok","command":"unlock","original_timestamp":"<ts>","client_id":"<cid>","request_id":"<rid>"}
```

**Comando del servidor (JSON):**
```json
{"id":"c-xxx","command":"unlock","timestamp":"...","client_id":"...","request_id":"..."}
```

---

## 11. Comandos útiles

- **Compilar:** `pio run`
- **Flashear:** `pio run --target upload` (opcional `--upload-port /dev/ttyACM0`).
- **Monitor serial:** `pio device monitor -p /dev/ttyACM0 -b 115200`.

---

## 12. Estado actual y próximos pasos

- **Separación parcial completada:** sim7600.c, connectivity.c, app_config.h, main.c.
- **Pendiente:** Extraer `server_cmd.c/.h` (procesamiento de comandos sigue en sim7600.c).
- **Pendiente:** Implementar reset hardware PWRKEY del SIM7600.
- **Pendiente:** Limpiar código obsoleto (pending_ack_item_t, estados boot_state_t no usados).
- **Pendiente:** Proteger secuencias UART sin mutex en netopen/cipopen_tcp/cipclose.
- **Próximo paso conocido:** Integrar BMS por UART (UART1 en GPIOs libres).

---

*Documento generado a partir del código fuente real (2026-03-16). Incluye máquina de estados, flujo del loop, reglas de mutex, variables, constantes y formatos.*
