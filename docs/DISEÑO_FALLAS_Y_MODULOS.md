# Diseno: control de fallas y separacion en modulos

---

## Parte 1 — Control de fallas (estados y flujo)

### 1.1 Capas de conectividad

| Capa | Significado | Si falla |
|------|-------------|----------|
| **Modulo** | SIM7600 responde por UART (AT OK) | Reintentar AT; cuando responda, ejecutar setup completo (idempotente) |
| **Red** | PDP/red abierta (NETOPEN) | Reintentar NETOPEN; no rehacer CIPOPEN hasta que red este OK |
| **Socket** | Conexion TCP al backend (CIPOPEN link 0) | Solo reintentar CIPOPEN; no tocar NETOPEN ni setup del modulo |

### 1.2 Maquina de estados de conectividad

Implementada en `connectivity.h / connectivity.c`:

```c
typedef enum {
    CONNECTIVITY_MODULE_OFFLINE,   // Modulo no responde por UART. Reintentar AT cada 30s.
    CONNECTIVITY_MODULE_READY,     // AT OK; falta configurar red/socket.
    CONNECTIVITY_NET_DOWN,         // Red no abierta. Reintentar NETOPEN cada 30s.
    CONNECTIVITY_NET_READY,        // Red abierta; falta socket.
    CONNECTIVITY_SOCKET_DOWN,      // Socket cerrado. Reintentar CIPOPEN con backoff.
    CONNECTIVITY_RUNNING           // Red + socket OK; loop normal.
} connectivity_state_t;
```

### 1.3 Transiciones

```text
RUNNING → CIPSEND falla (+CIPERROR/+IPCLOSE) → SOCKET_DOWN
RUNNING → NETOPEN? indica red cerrada → NET_DOWN
RUNNING → N timeouts AT seguidos → MODULE_OFFLINE
MODULE_OFFLINE → AT OK → MODULE_READY → setup completo → RUNNING (o NET_DOWN/SOCKET_DOWN)
NET_DOWN → NETOPEN OK → NET_READY → CIPOPEN → RUNNING
SOCKET_DOWN → CIPOPEN OK → RUNNING
```

### 1.4 Deteccion

| Que detectar | Donde / como |
|--------------|--------------|
| Modulo no responde | Cada CONNECTIVITY_NET_CHECK_CYCLES (10) ciclos: AT health-check. N timeouts → MODULE_OFFLINE |
| Red cerrada | sim7600_netopen_status() tras fallo de envio; si "+NETOPEN: 0" → NET_DOWN |
| Socket cerrado | +IPCLOSE/+CIPERROR en async_read_task; fallo de cipsend → SOCKET_DOWN |

### 1.5 Recuperacion (connectivity_step_recovery)

| Estado | Accion |
|--------|--------|
| MODULE_OFFLINE | Enviar AT; si OK → MODULE_READY → run_full_setup() |
| MODULE_READY | Setup idempotente: CIPMODE(0), buffer(1), NETOPEN, DNS, CIPOPEN |
| NET_DOWN | Reintentar NETOPEN; si OK → NET_READY → intentar CIPOPEN |
| NET_READY | Intentar CIPOPEN |
| SOCKET_DOWN | Solo CIPOPEN (con backoff exponencial) |
| RUNNING | Nada; el loop hace drain/send |

### 1.6 Backoff (connectivity_wait_backoff)

| Estado | Intervalo |
|--------|-----------|
| MODULE_OFFLINE | 30s fijo (CONNECTIVITY_AT_RETRY_MS) |
| NET_DOWN | 30s fijo (CONNECTIVITY_NET_RETRY_MS) |
| SOCKET_DOWN | Exponencial: 10s → 30s → 1min → 2min → 5min (tope). Se resetea a 0 tras exito. |

### 1.7 (Opcional) Reset hardware del SIM7600

Si el modulo deja de responder tras muchos intentos en MODULE_OFFLINE:
- Pin PWRKEY del SIM7600: pulso low 1.5s → high → esperar 10-15s → reintentar AT.
- Requiere definir `SIM7600_PWRKEY_GPIO` en app_config.h.

---

## Parte 2 — Separacion en modulos (implementada)

### 2.1 Estructura actual

```text
src/
  main.c              — app_main: init UARTs, GPS, BMS, PDP, lanza update loop
  app_config.h         — Todos los defines centralizados
  sim7600.h/.c         — Driver SIM7600: AT, TCP, buffer mode, telemetria, comandos, ACK
  gps.h/.c             — GPS via AT+CGPSINFO (ultimo fix valido)
  frame_protocol.h/.c  — Protocolo binario: parser incremental, builder, Fletcher-16
  periph_uart.h/.c     — UART1: BMS (SoC, voltaje, proteccion) + actuadores (lock/unlock)
  connectivity.h/.c    — Maquina de estados de conectividad + backoff
```

### 2.2 Responsabilidades

| Archivo | Responsabilidad | Depende de |
|---------|----------------|------------|
| main.c | Orquestacion: init, config, lanzar loop | Todos |
| sim7600.c | Todo lo AT: envio TCP, recepcion, drain, cipsend, telemetria | app_config, gps, periph_uart, connectivity |
| gps.c | Obtener posicion GPS real del SIM7600 | sim7600 (send_command), app_config |
| frame_protocol.c | Parsear/construir frames binarios | Ninguno (standalone) |
| periph_uart.c | Comunicacion UART1 con electronica | frame_protocol, app_config |
| connectivity.c | Recovery de red con backoff | sim7600, app_config |

### 2.3 Variables: donde viven

| Variable / estado | Archivo |
|-------------------|---------|
| Pines, UARTs, timeouts, host, port, APN | app_config.h (defines) |
| uart_mutex, shared_buffers, rx_pending, current_state | sim7600.c (static) |
| command_queue, static_cmd_item, dedup cache | sim7600.c (static) |
| s_last_pos (GPS) | gps.c (static) |
| s_bms_power, s_bms_status, s_lock_state | periph_uart.c (static, protegido por s_data_mutex) |
| s_state, s_socket_backoff_index, s_at_timeout_count | connectivity.c (static) |

### 2.4 Flujo de datos entre modulos

```text
                    ┌─────────────────┐
                    │    main.c       │
                    │  (orquestacion) │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
         UART0 (USB)    UART2 (SIM7600)  UART1 (Electronica)
         Debug logs     sim7600.c        periph_uart.c
                        gps.c            frame_protocol.c
                        connectivity.c
                             │              │
                    ┌────────┴────────┐     │
                    │                 │     │
               TCP socket        GPS        │
               Telemetria +      Real       │
               Cmds server                  │
                                    ┌───────┴────────┐
                                    │  Protocolo     │
                                    │  binario       │
                                    └───────┬────────┘
                                            │
                              ┌─────────────┼──────────┐
                              │             │          │
                         BMS (0x01)   Actuadores   Sistema
                         SoC, V, A    Lock/Unlock   KeepAlive
                         Temp, Prot   (0x03)       (0xFF)
```

### 2.5 Principio "ultimo valor conocido"

Todos los modulos de datos (gps.c, periph_uart.c) mantienen internamente el ultimo valor valido recibido. Si una lectura falla o no hay datos nuevos, los getters retornan el valor anterior. Esto garantiza que la telemetria siempre tiene datos, aunque no sean frescos.

| Modulo | Getter | Si no hay datos |
|--------|--------|-----------------|
| GPS | gps_get_position() | valid=false, lat/lon=0 |
| BMS | periph_get_bms_power() | valid=false, soc=0 |
| Actuador | periph_get_lock_state() | LOCK_STATE_UNKNOWN → "unknown" |
