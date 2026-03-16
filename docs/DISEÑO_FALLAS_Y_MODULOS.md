# Diseño: control de fallas y separación en módulos

Documento actualizado (2026-03-16) que refleja el estado real de la implementación y marca qué está hecho, qué falta, y qué bugs se han identificado.

---

## Parte 1 — Control de fallas (IMPLEMENTADO)

### 1.1 Capas de conectividad

| Capa | Significado | Si falla |
|------|-------------|----------|
| **Módulo** | SIM7600 responde por UART (AT OK) | Reintentar AT cada T1; si OK → setup completo |
| **Red** | PDP/red abierta (NETOPEN) | Reintentar NETOPEN cada T2; no tocar CIPOPEN hasta que red esté OK |
| **Socket** | Conexión TCP al backend (CIPOPEN link 0) | Solo reintentar CIPOPEN con backoff; no tocar NETOPEN |

### 1.2 Estados (implementados en connectivity.c)

```c
typedef enum {
    CONNECTIVITY_MODULE_OFFLINE,   // Módulo no responde. Reintentar AT cada T1 (30s).
    CONNECTIVITY_MODULE_READY,     // AT OK; ejecutar setup completo.
    CONNECTIVITY_NET_DOWN,         // Red no abierta. Reintentar NETOPEN cada T2 (30s).
    CONNECTIVITY_NET_READY,        // Red abierta; falta socket.
    CONNECTIVITY_SOCKET_DOWN,      // Socket caído. Reintentar CIPOPEN con backoff.
    CONNECTIVITY_RUNNING           // Todo OK; loop normal.
} connectivity_state_t;
```

### 1.3 Transiciones (implementadas)

```text
RUNNING → send falla + red cerrada    → NET_DOWN
RUNNING → send falla + red OK         → SOCKET_DOWN
RUNNING → N timeouts AT (health-check) → MODULE_OFFLINE

MODULE_OFFLINE → AT OK        → MODULE_READY → run_full_setup()
                                  → Si NETOPEN falla  → NET_DOWN
                                  → Si CIPOPEN falla  → SOCKET_DOWN
                                  → Si todo OK        → RUNNING

NET_DOWN → NETOPEN OK         → NET_READY → intenta CIPOPEN
                                  → Si OK    → RUNNING
                                  → Si falla → SOCKET_DOWN

NET_READY → CIPOPEN OK        → RUNNING
NET_READY → CIPOPEN falla     → SOCKET_DOWN

SOCKET_DOWN → CIPOPEN OK      → RUNNING
SOCKET_DOWN → CIPOPEN falla   → queda en SOCKET_DOWN (con backoff)
```

### 1.4 Detección (implementada en sim7600_scooter_update_loop)

| Qué detectar | Cómo se detecta | Código |
|--------------|-----------------|--------|
| Módulo no responde | Health-check AT cada 10 ciclos; 3 timeouts seguidos → MODULE_OFFLINE | `connectivity_notify_at_timeout()` |
| Red cerrada | `AT+NETOPEN?` devuelve `+NETOPEN: 0` durante health-check o tras fallo de envío | `connectivity_notify_send_failed(true)` |
| Socket caído | `sim7600_send_scooter_update` falla + red sigue abierta | `connectivity_notify_send_failed(false)` |

### 1.5 Backoff (implementado en connectivity.c)

| Estado | Espera |
|--------|--------|
| MODULE_OFFLINE | T1 = 30s fijo |
| NET_DOWN | T2 = 30s fijo |
| SOCKET_DOWN | Exponencial: 10s → 30s → 60s → 120s → 300s (tope) |
| RUNNING | 1s (default, no aplica) |

Tras reconexión exitosa (→ RUNNING), el índice de backoff se resetea a 0.

### 1.6 Setup completo (run_full_setup en connectivity.c)

Secuencia idempotente ejecutada desde MODULE_READY:
1. `sim7600_set_cipmode(0)` — no transparente
2. `sim7600_set_buffer_mode(1)` — modo buffer
3. `sim7600_netopen()` — si falla → NET_DOWN, return
4. `sim7600_set_dns("8.8.8.8", "8.8.4.4")`
5. `sim7600_cipclose(0)` — limpiar link si estaba abierto
6. `sim7600_cipopen_tcp(0, server_ip, server_port)` — si falla → SOCKET_DOWN

### 1.7 Reset hardware PWRKEY (NO implementado)

Diseño original contemplaba reset por GPIO tras K intentos fallidos en MODULE_OFFLINE. Requiere definir `SIM7600_PWRKEY_GPIO` en app_config.h y función `sim7600_hardware_reset()`.

---

## Parte 2 — Separación del código

### 2.1 Estado actual de la separación

| Archivo | Estado | Contenido |
|---------|--------|-----------|
| **app_config.h** | ✅ Implementado | Defines UART, pines, constantes scooter, servidor, backoff, colas, `location_t` |
| **sim7600.h** | ✅ Implementado | `sim7600_response_t`, `boot_state_t`, todos los prototipos públicos |
| **sim7600.c** | ✅ Implementado | Driver AT completo + procesamiento de comandos + loop principal |
| **connectivity.h** | ✅ Implementado | `connectivity_state_t`, prototipos de recuperación |
| **connectivity.c** | ✅ Implementado | Máquina de estados, backoff, `run_full_setup`, `connectivity_step_recovery` |
| **main.c** | ✅ Implementado | `app_main`: init UART, AT test, PDP, arranca loop |
| **server_cmd.h/.c** | ❌ Pendiente | Procesamiento de comandos sigue dentro de sim7600.c |

### 2.2 Qué falta extraer de sim7600.c a server_cmd.c

- `command_queue_item_t` (typedef struct)
- `command_queue`, `static_cmd_item`, `server_command_buffer`
- `processed_request_ids[][]`, `processed_request_count`, `processed_request_index`
- `server_messages_received`
- `sim7600_process_server_command()`
- `sim7600_command_processor_task()`
- `sim7600_send_ack()`
- Macro `EXTRACT_JSON_FIELD`

Dependencias: server_cmd necesitaría llamar a `sim7600_cipsend()` para enviar ACKs.

### 2.3 Código obsoleto pendiente de eliminar

- `pending_ack_item_t` (struct)
- `pending_ack_queue` (QueueHandle_t, marcado `__attribute__((unused))`)
- `static_acks_buffer[PENDING_ACK_QUEUE_SIZE]` (marcado unused)
- `static_ack_obj_buffer[256]` (marcado unused)
- Estados no usados de `boot_state_t`: STATE_BOOT, STATE_NET_UP, STATE_SOCKET_CONNECTING, STATE_TX_WAIT_PROMPT, STATE_TX_SENDING, STATE_TX_WAIT_RESULT, STATE_ERROR_RECOVER

---

## Parte 3 — Bugs identificados y mejoras propuestas

### 3.1 Bugs

**BUG-1: Secuencias UART sin protección de mutex (MEDIO-ALTO)**
Las funciones `sim7600_netopen()`, `sim7600_cipopen_tcp()`, `sim7600_cipclose()` y `sim7600_cipopen_tcp_transparent()` escriben directamente al UART con `uart_write_bytes()` y luego llaman a `sim7600_wait_for_response()` **sin tomar el mutex**. La tarea `sim7600_async_read_task` podría consumir la respuesta antes de que `wait_for_response` la lea.

Archivos: sim7600.c líneas ~503, ~1029-1033, ~1064-1068, ~939.
Fix: Estas funciones deben tomar `uart_mutex` antes de escribir y liberarlo después de `wait_for_response`.

**BUG-2: uart_flush_input descarta URCs (MEDIO)**
Las funciones `sim7600_netopen()`, `sim7600_cipopen_tcp()`, `sim7600_cipclose()`, `sim7600_cipopen_tcp_transparent()` llaman a `uart_flush_input(UART_SIM)` antes de escribir comandos. Esto puede descartar URCs `+CIPRXGET: 1,<link>` que estaban pendientes, causando pérdida de datos del servidor.

Fix: Eliminar `uart_flush_input()` o reemplazarla por una lectura que parsee URCs antes de descartarlos.

**BUG-3: Race condition en rx_pending (BAJO)**
`rx_pending[]` es modificado por `sim7600_async_read_task` (pone true) y por el loop principal (pone false) sin protección atómica formal. En ESP32 (32-bit) un bool es atómico en la práctica, pero no es portable ni formalmente correcto.

Fix: Usar `atomic_bool` de `<stdatomic.h>` o proteger con mutex/spinlock.

### 3.2 Mejoras propuestas

**MEJORA-1: Extraer server_cmd.c/.h**
Mover todo el procesamiento de comandos fuera de sim7600.c. Reduce sim7600.c de 2175 a ~1700 líneas y mejora separación de responsabilidades.

**MEJORA-2: Limpiar boot_state_t**
Solo se usan 3 de 10 estados. Reducir el enum a los 3 usados o implementar los restantes.

**MEJORA-3: Eliminar código obsoleto**
Quitar `pending_ack_item_t`, `pending_ack_queue`, y buffers asociados (~20 líneas de código muerto).

**MEJORA-4: Parseo JSON más robusto**
`EXTRACT_JSON_FIELD` es una macro con parseo manual por `strchr`. No maneja: valores numéricos (busca comillas), strings con escape (`\"`), objetos anidados. Considerar cJSON (disponible en ESP-IDF) para parseo seguro.

**MEJORA-5: Reset hardware PWRKEY**
Implementar `sim7600_hardware_reset()` para recuperar de cuelgues del módulo que no responden a AT.

**MEJORA-6: Telemetría con datos reales**
Actualmente usa coordenadas fijas (Concepción, Chile), batería fija (85%), velocidad fija (15.5 km/h). Preparar para integración con GPS real y BMS.

---

*Documento actualizado a partir del código fuente real (2026-03-16).*
