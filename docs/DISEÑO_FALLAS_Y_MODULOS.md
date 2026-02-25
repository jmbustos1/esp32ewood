# Diseño: control de fallas y separación en módulos

Documento para implementar control de fallas (estados, detección, recuperación) y para separar el código en módulos/utils/variables y mejorar trazabilidad y legibilidad.

---

## Parte 1 — Control de fallas (estados y flujo)

### 1.1 Capas de conectividad

Se distinguen tres capas; cada una puede estar "OK" o "fallida". El comportamiento depende de qué capa falla.

| Capa | Significado | Si falla |
|------|-------------|----------|
| **Módulo** | SIM7600 responde por UART (AT OK). | Reintentar AT; cuando responda, ejecutar **setup completo** (idempotente). |
| **Red** | PDP/red abierta (NETOPEN). | Reintentar NETOPEN (y lo previo si hace falta); no rehacer CIPOPEN hasta que red esté OK. |
| **Socket** | Conexión TCP al backend (CIPOPEN link 0). | Solo **reintentar CIPOPEN** (y envíos); no tocar NETOPEN ni setup del módulo. |

### 1.2 Estados concretos (máquina de conectividad)

Un único enum (o variable de estado) que resume en qué capa estamos:

```text
CONNECTIVITY_MODULE_OFFLINE   — Módulo no responde (UART). Reintentar AT cada T1 (ej. 30 s).
CONNECTIVITY_MODULE_READY    — AT OK; falta configurar red/socket o recuperarlos.
CONNECTIVITY_NET_DOWN        — Red no abierta (sin datos SIM o módulo reseteado). Reintentar NETOPEN cada T2.
CONNECTIVITY_NET_READY       — Red abierta; falta socket o se cayó.
CONNECTIVITY_SOCKET_DOWN     — Backend caído o conexión cerrada. Reintentar solo CIPOPEN (backoff).
CONNECTIVITY_RUNNING         — Red + socket OK; loop normal (drenar RX, enviar telemetría).
```

Transiciones resumidas:

- **RUNNING** → si CIPSEND falla con +CIPERROR / +IPCLOSE → **SOCKET_DOWN** (solo reintentar CIPOPEN).
- **RUNNING** → si NETOPEN? indica red cerrada inesperadamente → **NET_DOWN** (reintentar NETOPEN; puede ser SIM7600 reseteado o pérdida datos SIM).
- **RUNNING** / **NET_READY** → si varios AT seguidos dan TIMEOUT → **MODULE_OFFLINE** (reintentar AT; puede ser cable desconectado).
- **MODULE_OFFLINE** → AT responde OK → **MODULE_READY** → ejecutar **setup completo** (CIPMODE, buffer, NETOPEN, DNS, CIPOPEN) → **NET_READY** o **RUNNING**.
- **NET_DOWN** → NETOPEN OK → **NET_READY** → CIPOPEN → **RUNNING**.
- **SOCKET_DOWN** → CIPOPEN OK → **RUNNING**.

### 1.3 Dónde detectar (en el código actual)

| Qué detectar | Dónde |
|--------------|--------|
| Módulo no responde | **Implementado:** cada `CONNECTIVITY_NET_CHECK_CYCLES` (ej. 10) ciclos en RUNNING se envía `AT\r\n`; si TIMEOUT/ERROR se llama `connectivity_notify_at_timeout()`. Tras `CONNECTIVITY_AT_TIMEOUT_COUNT` (ej. 3) timeouts seguidos → MODULE_OFFLINE. Si AT OK → `connectivity_notify_at_ok()` resetea el contador. |
| Red cerrada / módulo perdió estado | En el loop: antes de enviar telemetría, opcionalmente `sim7600_netopen_status()`; si indica red cerrada cuando debería estar abierta → pasar a NET_DOWN. O bien cuando CIPSEND falla y en la respuesta hay indicio de "no network". |
| Socket cerrado / backend caído | Ya parcialmente: +IPCLOSE y +CIPERROR en la async task; fallo de `sim7600_cipsend` o `sim7600_send_scooter_update`. Acción: no rehacer NETOPEN; solo marcar SOCKET_DOWN y reintentar CIPOPEN con backoff. |

### 1.4 Qué función llama a qué (recuperación)

| Estado actual | Acción (función / secuencia) |
|---------------|------------------------------|
| **MODULE_OFFLINE** | Cada T1: `sim7600_send_command("AT\r\n", ...)`. Si OK → estado **MODULE_READY** y llamar a **setup completo** (ver abajo). |
| **MODULE_READY** | Ejecutar **setup idempotente**: set_cipmode(0), set_buffer_mode(1), netopen (si no está abierta), set_dns, cipopen. Si todo OK → **RUNNING**. Si NETOPEN falla → **NET_DOWN**. |
| **NET_DOWN** | Cada T2: intentar netopen (y si hace falta set_cipmode/set_buffer_mode otra vez). Si NETOPEN OK → **NET_READY** y luego cipopen → **RUNNING**. |
| **NET_READY** | Intentar cipopen. Si OK → **RUNNING**. Si falla (backend caído) → **SOCKET_DOWN**. |
| **SOCKET_DOWN** | Reintentar solo **CIPOPEN** con backoff (10 s, 30 s, 1 min, tope). No NETCLOSE ni reconfigurar módulo. Si CIPOPEN OK → **RUNNING**. |
| **RUNNING** | Loop actual: drenar RX si hay datos, enviar telemetría si !rx_pending; si send falla, decidir por respuesta: si es error de red → NET_DOWN; si es cierre/error TCP → SOCKET_DOWN. |

**Setup idempotente (para MODULE_READY o tras ESP32 reset):**  
Misma secuencia que hoy antes del `while(1)` en `sim7600_scooter_update_loop`, pero con comprobaciones para no asumir "primera vez":  
- CIPMODE(0): ya se lee estado y solo se cambia si hace falta; antes se cierran conexiones y red.  
- Buffer mode(1): enviar comando; si ya está en 1, no rompe.  
- NETOPEN: si ya está abierta (o "already opened"), tratar como OK.  
- DNS: reconfigurar es idempotente.  
- **CIPOPEN:** Antes de abrir, comprobar si el link 0 ya está conectado (comando de estado de conexiones si el módulo lo soporta); si ya está abierto, cerrar ese link y luego CIPOPEN de nuevo, o omitir si la API lo permite. Así la rutina se puede ejecutar varias veces sin dejar estado inconsistente.

### 1.5 Backoff (evitar saturar)

- **MODULE_OFFLINE:** intervalo fijo T1 (ej. 30 s) entre intentos AT.  
- **NET_DOWN:** intervalo T2 (ej. 30 s o 1 min) entre intentos NETOPEN.  
- **SOCKET_DOWN:** backoff creciente (ej. 10 s → 30 s → 60 s, tope 5 min); tras éxito, resetear el backoff.

### 1.6 (Opcional) Reset hardware del SIM7600

Si el módulo deja de responder por UART (cuelgue, power glitch), además de reintentar AT cada T1 en MODULE_OFFLINE se puede añadir un **reset por hardware** cuando llevamos muchos intentos fallidos (ej. tras K ciclos en MODULE_OFFLINE sin respuesta):

- **PWRKEY:** pin del SIM7600 que, pulsado un tiempo (ej. 1–2 s), apaga/reinicia el módulo. El ESP32 debe controlar ese pin (GPIO) según el esquema del fabricante.
- Flujo: en `connectivity_step_recovery` para MODULE_OFFLINE, si un contador de intentos AT supera un umbral (ej. 5), ejecutar secuencia PWRKEY (low 1.5 s, high), esperar 10–15 s, volver a intentar AT. Requiere definir en `app_config.h` el pin (ej. `SIM7600_PWRKEY_GPIO`) y una función `sim7600_hardware_reset()` en sim7600.c.

---

## Parte 2 — Separación del código (módulos, utils, variables)

### 2.1 Objetivo

- **Trazabilidad:** Saber dónde está cada responsabilidad (AT, comandos servidor, config, loop principal).  
- **Legibilidad:** main.c corto; cada archivo con un propósito claro.  
- **Mantenibilidad:** Cambios en SIM7600 no tocan lógica de comandos; cambios en fallas no mezclan con parsing AT.

### 2.2 Estructura de directorios y archivos propuesta

```text
src/
  main.c              — app_main, init UART, creación de tareas, loop principal (orquestación).
  app_config.h        — Defines y config: UART_SIM, pines, BUF_SIZE, SCOOTER_*, host, port, intervals, APN.
  sim7600.h           — API pública del driver SIM7600 (prototipos, tipos sim7600_response_t, boot_state_t si se exponen).
  sim7600.c           — Todo lo AT: send_command, wait_for_response, PDP, CIPMODE, NETOPEN, DNS, buffer mode, get/read_rx_buffer, drain_rx_buffer, cipopen, cipclose, cipsend, send_scooter_update, async_read_task. Variables: uart_mutex, shared_response_buffer, shared_data_buffer, rx_pending, rest_len, buffer_mode_active, current_state.
  sim7600_internal.h  — (Opcional) Prototipos y tipos solo usados dentro de sim7600.c.
  server_cmd.h        — Procesamiento de comandos del servidor (prototipos, command_queue_item_t, etc.).
  server_cmd.c        — process_server_command, command_processor_task, send_ack, cola, dedup (processed_request_ids), server_command_buffer.
  connectivity.h      — Estados de conectividad (enum), prototipos de recuperación (opcional: connectivity_run_state_machine, connectivity_get_state).
  connectivity.c      — (Si se implementa máquina de fallas) Lógica de estados MODULE_OFFLINE → … → RUNNING, backoff, y llamadas a sim7600_* según estado.
  utils.h / utils.c   — (Opcional) Helpers genéricos si los hay (ej. extracción JSON, formateo). Si solo se usa en server_cmd, puede vivir ahí.
```

El `CMakeLists.txt` actual usa `FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)`, así que cualquier `*.c` bajo `src/` se compila; solo hay que añadir los nuevos `.c` en `src/` y los `.h` se incluyen por `#include`.

### 2.3 Qué va en cada archivo (contenido aproximado)

| Archivo | Contenido |
|---------|-----------|
| **app_config.h** | `UART_SIM`, `UART_TX`, `UART_RX`, `BUF_SIZE`, `RESPONSE_TIMEOUT_MS`, `MAX_LINKS`, `SCOOTER_ID`, `SCOOTER_BATTERY`, `SCOOTER_SPEED_KMH`, `UPDATE_INTERVAL_SEC`, `RX_POLL_MS`, `SCOOTER_TCP_HOST`, `SCOOTER_TCP_PORT`, `COMMAND_QUEUE_SIZE`, `MAX_PROCESSED_REQUESTS`, `RX_CHUNK_SIZE`, `MAX_ITERATIONS`; opcional `location_t` y `locations[]` o dejarlos en main/sim7600. |
| **sim7600.h** | `sim7600_response_t`, `boot_state_t` (si se exponen), prototipos: init (uart), send_command, set_pdp_context, set_cipmode, netopen, netclose, set_dns, set_buffer_mode, get_rx_buffer_length, read_rx_buffer, drain_rx_buffer, cipopen_tcp, cipclose, cipsend, send_scooter_update, async_read_task; opcional get_current_state / set_current_state si el loop principal necesita leer estado. |
| **sim7600.c** | Implementación de todo lo anterior; variables estáticas (mutex, buffers, rx_pending, rest_len, buffer_mode_active, current_state). Incluye `app_config.h`, `sim7600.h`. No incluye lógica de "qué comando ejecutar" del servidor; solo envía ACK si se le pasa el payload (desde server_cmd). |
| **server_cmd.h** | `command_queue_item_t`, prototipos: `process_server_command`, `command_processor_task`, `send_ack`; opcional `command_queue` getter si main necesita crear la cola y pasarla. |
| **server_cmd.c** | Cola de comandos, dedup, `process_server_command` (parseo JSON, EXTRACT_JSON_FIELD, encolar), `command_processor_task`, `send_ack` (que llamará a sim7600_cipsend). Depende de sim7600 (solo para enviar ACK) y de app_config si usa constantes. |
| **main.c** | `app_main`: config UART (usando defines de app_config), delay, AT test, PDP (set_pdp, read_pdp, test_pdp), luego `sim7600_scooter_update_loop` (o equivalente que use connectivity + sim7600). Creación de mutex, command_queue, tareas (command_processor, async_read). El loop principal: según estado de conectividad, llamar a drain/send o a rutinas de recuperación. Incluye app_config, sim7600, server_cmd, connectivity (si existe). |
| **connectivity.h / .c** | Enum de estados (MODULE_OFFLINE, …, RUNNING); variable de estado; backoff; función que "ejecuta un paso" según estado (reintentar AT, NETOPEN, CIPOPEN, o "loop normal"). Llama a sim7600_* y decide transiciones. Opcional: se puede integrar esta lógica dentro de main.c o dentro de sim7600_scooter_update_loop en una primera versión. |

### 2.4 Variables: dónde viven

| Variable / estado | Archivo |
|-------------------|---------|
| Pines, buffers size, timeouts, host, port, IDs | **app_config.h** (defines) o constantes en **app_config.c** si se prefiere no macros. |
| uart_mutex, shared_response_buffer, shared_data_buffer, rx_pending, rest_len, buffer_mode_active, current_state (boot_state_t) | **sim7600.c** (static). |
| command_queue, static_cmd_item, server_command_buffer, processed_request_ids, server_messages_received | **server_cmd.c** (static). |
| Estado de conectividad (MODULE_OFFLINE, etc.), contadores de backoff | **connectivity.c** (o main.c si no se extrae). |
| location_index, update_count (loop principal) | **main.c** (o dentro de la función que implementa el loop en sim7600.c si el loop sigue ahí). |

### 2.5 Dependencias entre módulos

- **main** → app_config, sim7600, server_cmd, connectivity (si existe).  
- **sim7600** → app_config (defines), FreeRTOS, UART, ESP log/timer. No conoce server_cmd; solo expone `send_ack` o bien server_cmd llama a `sim7600_cipsend` con el JSON de ACK.  
- **server_cmd** → sim7600 (para enviar ACK vía cipsend), app_config si usa tamaños/constantes.  
- **connectivity** → sim7600 (para AT, netopen, cipopen, etc.), app_config (timeouts de backoff).

### 2.6 Ventajas de esta separación

- **Trazabilidad:** Un fallo de AT → mirar sim7600.c; un fallo de comando/ACK → server_cmd.c; un fallo de "no reconecta" → connectivity.c o main.  
- **Legibilidad:** main.c se reduce a init + "run loop según estado"; cada módulo tiene un solo tema.  
- **Pruebas:** Se puede mockear sim7600 para probar server_cmd o la máquina de conectividad.  
- **Evolución:** Añadir BMS = nuevo módulo bms.c/bms.h; no ensuciar main ni sim7600.

---

## Orden sugerido de implementación

1. **Fallas:** Implementar la máquina de estados de conectividad (connectivity.h/.c o dentro de main) y los puntos de detección en el loop actual; hacer el setup idempotente (sobre todo CIPOPEN).  
2. **Separación:** Extraer **app_config.h** con defines; luego **sim7600.h** + **sim7600.c** moviendo todo el código AT y buffers; luego **server_cmd.h** + **server_cmd.c** con cola y process_server_command; por último refactorizar **main.c** para orquestar y, si se desea, **connectivity** en su propio módulo.

Si quieres, el siguiente paso puede ser implementar solo la extracción de **app_config.h** y un primer **sim7600.h** con la API pública y un esqueleto de **sim7600.c** para ir moviendo funciones sin romper el build.
