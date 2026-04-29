# Contexto del proyecto esp32ewood

Documento de contexto para retomar el proyecto sin perder el hilo.

---

## 1. Que es el proyecto

- **Objetivo:** Dispositivo IoT en un scooter que se comunica con un servidor en la nube por 4G.
- **Hardware:** ESP32 (DOIT DEVKIT V1) + modulo celular **SIM7600** (UART) + electronica BMS/actuadores (UART).
- **Framework:** ESP-IDF via PlatformIO.
- **Funciones principales:**
  - Enviar al servidor telemetria cada 5 s: posicion GPS real, bateria (BMS), velocidad, estado lock, voltaje.
  - Recibir comandos del servidor (unlock, lock) y reenviarlos a actuadores via UART1.
  - Enviar ACK inmediato en NDJSON por cada comando recibido.
  - Leer datos del BMS (SoC, voltaje, corriente, proteccion) via protocolo binario.

---

## 2. Arquitectura de UARTs

| UART | Uso | Pines | Baudrate |
|------|-----|-------|----------|
| UART0 | Debug console USB | GPIO1 / GPIO3 (fijos) | 115200 |
| UART2 | SIM7600 (TCP + GPS) | TX=GPIO17, RX=GPIO16 | 115200 |
| UART1 | Electronica (BMS, actuadores, IMU) | TX=GPIO4, RX=GPIO5 | 115200 |

---

## 3. Estructura de archivos

```text
src/
  main.c              — app_main: init UARTs, GPS, BMS polling, PDP+auth, lanza update loop
  app_config.h         — Defines: pines, UARTs, APN, servidor, intervalos, constantes
  sim7600.h / sim7600.c — Driver SIM7600: AT commands, TCP, buffer mode, telemetria, comandos servidor, ACK
  gps.h / gps.c        — GPS via AT+CGPSINFO: init, update, get_position (ultimo fix valido)
  frame_protocol.h/.c  — Protocolo binario de frames: parser incremental, builder, Fletcher-16
  periph_uart.h/.c     — UART1 electronica: lee BMS (SoC, voltaje, proteccion), envia lock/unlock a actuadores
  connectivity.h/.c    — Maquina de estados de conectividad y recuperacion con backoff
  CMakeLists.txt       — GLOB_RECURSE, incluye todo *.c automaticamente
```

---

## 4. Comunicacion SIM7600 (UART2)

- **Modo:** No transparente (`AT+CIPMODE=0`) con **modo buffer** (`AT+CIPRXGET=1`).
- Los datos entrantes se guardan en buffer del modulo; notifica con URC `+CIPRXGET: 1,<link>`.
- El firmware consulta con `AT+CIPRXGET=4,<link>` (longitud) y `AT+CIPRXGET=2,<link>,<len>` (leer datos).
- **Flujo de envio:** `AT+CIPSEND=0,<len>` → esperar prompt `>` → escribir bytes → esperar `+CIPSEND:` confirmacion.
- **Sincronizacion:** `uart_mutex` protege el UART durante secuencias AT completas.
- La tarea async solo detecta URCs y marca `rx_pending[]`; el drenaje real se hace en el loop principal.

---

## 5. GPS (via SIM7600, misma UART2)

- **Inicio:** `AT+CGPS=1,1` (modo standalone) lo mas temprano posible en el boot.
- **Lectura:** `AT+CGPSINFO` antes de cada telemetria.
- **Respuesta:** `+CGPSINFO:<lat>,<N/S>,<lon>,<E/W>,<date>,<time>,<alt>,<speed>,<course>`
- **Formato lat/lon:** NMEA `ddmm.mmmmmm` → se convierte a grados decimales.
- **Sin fix:** Campos vacios `+CGPSINFO:,,,,,,,,` → se mantiene ultima posicion valida.
- **Velocidad:** Viene en nudos, se convierte a km/h (x 1.852).

---

## 6. Protocolo binario electronica (UART1)

Comunicacion con microcontrolador BMS/actuadores usando frames binarios:

```
SOF (2B: 0xA5 0x5A) | LENGTH (2B Big Endian) | TYPE (1B) | PAYLOAD (NB) | CHECKSUM (2B Fletcher-16)
```

**LENGTH** = bytes desde TYPE hasta fin de PAYLOAD (inclusive).
**CHECKSUM** = Fletcher-16 calculado desde LENGTH hasta fin de PAYLOAD.

### Tipos (TYPE)

| Tipo | Codigo | Descripcion |
|------|--------|-------------|
| WRITE | 0x01 | Escribir dato en modulo/registro |
| READ | 0x02 | Solicitar lectura |
| ANS | 0x03 | Respuesta a solicitud |
| EVENT | 0x04 | Reporte de evento |
| ACK | 0x05 | Confirmacion de recepcion |
| ERROR | 0x06 | Mensaje mal procesado |

### Modulos (MODULE ID en PAYLOAD)

| Modulo | ID | Descripcion |
|--------|----|-------------|
| BMS | 0x01 | Bateria: voltaje, corriente, SoC, temperatura, proteccion |
| Acelerometro/IMU | 0x02 | Datos de movimiento |
| Actuadores ON/OFF | 0x03 | Lock/unlock fisico del scooter |
| Sistema | 0xFF | Boot, run, fault, keep-alive |

### Mensajes BMS relevantes

| MSGID | Datos | Uso |
|-------|-------|-----|
| 0x02 | Voltaje(2B 10mV), Corriente(2B 10mA), Cap.residual(2B 10mA), SoC(1B %) | battery_level y voltage en telemetria |
| 0x03 | N sondas(1B), T CMOS(2B 0.1K), T bateria(2B*n 0.1K) | Proteccion termica |
| 0x05 | ControlStatus(1B), ProtectionStatus(2B), BalanceStatus(4B) | Estado carga/descarga/proteccion |

### Actuadores (MODULE 0x03)

- **Lock:** WRITE con MSGID=0x01
- **Unlock:** WRITE con MSGID=0x02
- Respuesta ANS confirma ejecucion

### Parser

Parser incremental (maquina de estados) byte a byte. Resincroniza automaticamente si pierde SOF o checksum invalido.

---

## 7. Maquina de estados interna (boot_state_t)

```c
typedef enum {
    STATE_BOOT, STATE_NET_UP, STATE_SOCKET_CONNECTING,
    STATE_RUN_IDLE, STATE_TX_PREPARE, STATE_TX_WAIT_PROMPT,
    STATE_TX_SENDING, STATE_TX_WAIT_RESULT, STATE_RX_DRAIN,
    STATE_ERROR_RECOVER
} boot_state_t;
```

**Estados activos en el loop:**

| Estado | Significado |
|--------|-------------|
| STATE_RUN_IDLE | Sin operacion critica en curso |
| STATE_RX_DRAIN | Drenando buffer RX del socket |
| STATE_TX_PREPARE | Enviando telemetria |

---

## 8. Loop principal

En `sim7600_scooter_update_loop`, dentro del `while(1)`:

```text
1. Si no estamos en RUNNING → ejecutar recovery + backoff (connectivity.c)

2. Cada N ciclos: health-check AT + verificar estado de red

3. PRIORIDAD 1 — Drenar RX:
   - Tomar uart_mutex
   - Polling: sim7600_get_rx_buffer_length(0)
   - Si hay datos: STATE_RX_DRAIN → drain_rx_buffer → STATE_RUN_IDLE
   - Liberar uart_mutex

4. PRIORIDAD 2 — Enviar telemetria (si !rx_pending[0]):
   - gps_update()                          → obtener fix GPS
   - gps_get_position()                    → lat, lon, speed (ultimo valido)
   - periph_get_bms_power()                → SoC, voltaje (ultimo valido)
   - STATE_TX_PREPARE
   - sim7600_send_scooter_update(...)      → JSON con datos reales
   - STATE_RUN_IDLE
   - Si falla: detectar si es error de red o socket, transicionar estado

5. Espera UPDATE_INTERVAL_SEC en chunks de RX_POLL_MS; si rx_pending break.
```

---

## 9. Flujo de comandos lock/unlock

```text
Backend → TCP {"command":"unlock",...} → ESP32
  → sim7600_drain_rx_buffer() parsea NDJSON
  → sim7600_process_server_command() extrae campos, dedup por request_id
  → xQueueSend(command_queue, cmd_item)
  → sim7600_command_processor_task():
      → periph_send_unlock()  ← UART1 frame binario a actuadores
      → sim7600_send_ack()    ← TCP NDJSON al backend
```

---

## 10. Principio "ultimo valor conocido"

- **GPS sin fix** → envia ultima posicion valida (lat/lon 0 si nunca hubo fix)
- **BMS sin respuesta** → envia ultimo SoC recibido (0 si nunca llego)
- **Lock state desconocido** → reporta "unknown" hasta recibir confirmacion del actuador

---

## 11. Mutex UART — quien toma y quien asume tomado

| Funcion | Mutex |
|---------|-------|
| sim7600_send_command() | Toma al entrar, libera en todos los return |
| sim7600_wait_for_response() | NO toma; asume que el llamador ya tiene mutex |
| sim7600_get_rx_buffer_length() | NO toma; asume mutex tomado |
| sim7600_read_rx_buffer() | NO toma; mismo criterio |
| sim7600_drain_rx_buffer() | Toma al inicio, libera en todos los return |
| sim7600_cipsend() | Toma al inicio, libera al devolver |
| sim7600_async_read_task | Toma con timeout corto (50ms), solo detecta URCs |
| periph_uart (UART1) | Independiente, su propio s_data_mutex para datos BMS/lock |

---

## 12. Telemetria enviada (JSON)

```json
{"scooter_id":1,"latitude":-36.886767,"longitude":-73.044400,"battery_level":50,"speed_kmh":15.3,"lock_state":"locked","voltage":39.11}
```

- latitude/longitude: GPS real via AT+CGPSINFO
- battery_level: SoC% del BMS via UART1
- speed_kmh: GPS (nudos → km/h)
- lock_state: "locked" | "unlocked" | "unknown" (actuador via UART1)
- voltage: voltaje banco BMS

**ACK (NDJSON):**
```json
{"id":"<cmd_id>","type":"ack","status":"ok","command":"unlock","original_timestamp":"<ts>","client_id":"...","request_id":"..."}
```

---

## 13. Configuracion de red (APN)

- **APN:** `m2m.entel.cl` (Entel Chile M2M Manager)
- **Autenticacion:** PAP con usuario `entelpcs` y clave `entelpcs`
- **Secuencia:** `AT+CGDCONT=1,"IP","m2m.entel.cl"` → `AT+CGAUTH=1,1,"entelpcs","entelpcs"`
- **Servidor TCP:** `98.92.176.224:8201`
- **DNS:** `8.8.8.8`, `8.8.4.4`

---

## 14. Secuencia de inicializacion

**app_main (main.c):**
1. Init UART2 (SIM7600): 115200, 8N1
2. Init UART1 (electronica): periph_uart_init() → tarea RX en background
3. Solicitar BMS SoC periodico (cada 1s) y status (cada 10s)
4. Test AT con SIM7600
5. Iniciar GPS: AT+CGPS=1,1 (lo antes posible para TTFF)
6. Configurar PDP: AT+CGDCONT + AT+CGAUTH (APN M2M con PAP)
7. Leer configuracion PDP
8. Lanzar sim7600_scooter_update_loop() (no retorna)

**sim7600_scooter_update_loop (setup antes del while):**
1. connectivity_init()
2. Crear uart_mutex
3. AT+CIPMODE=0 (no transparente)
4. AT+CIPRXGET=1 (modo buffer)
5. AT+NETOPEN
6. AT+CIPDNSSET
7. AT+CIPOPEN=0,"TCP",host,port
8. Crear command_queue
9. Lanzar command_processor_task (prioridad 4)
10. Lanzar async_read_task (prioridad 5)
11. connectivity_set_state(RUNNING)
12. while(1) → loop principal

---

## 15. Constantes clave (app_config.h)

| Constante | Valor |
|-----------|-------|
| UART_SIM | UART_NUM_2 |
| UART_TX / UART_RX | GPIO17 / GPIO16 |
| PERIPH_TX_PIN / RX_PIN | GPIO4 / GPIO5 |
| SCOOTER_ID | 1 |
| UPDATE_INTERVAL_SEC | 5 |
| RX_POLL_MS | 150 |
| BUF_SIZE | 1024 |
| RESPONSE_TIMEOUT_MS | 5000 |
| APN_NAME | "m2m.entel.cl" |
| APN_USER / APN_PASS | "entelpcs" / "entelpcs" |
| SCOOTER_TCP_HOST | "98.92.176.224" |
| SCOOTER_TCP_PORT | 8201 |
| BMS_SOC_POLL_PERIOD_SEC | 1 |
| BMS_STATUS_POLL_PERIOD_SEC | 10 |
| COMMAND_QUEUE_SIZE | 20 |
| MAX_PROCESSED_REQUESTS | 20 |

---

## 16. Comandos utiles

- **Compilar:** `pio run`
- **Flashear:** `pio run --target upload` (opcional `--upload-port /dev/ttyACM0`)
- **Monitor serial:** `pio device monitor -p /dev/ttyACM0 -b 115200`

---

## 17. Estado actual

- GPS real implementado (AT+CGPSINFO), reemplaza coordenadas hardcodeadas
- BMS via UART1 con protocolo binario de frames (Fletcher-16)
- Lock/unlock se reenvian a actuadores via UART1
- Telemetria incluye lock_state y voltage
- APN actualizado a m2m.entel.cl con autenticacion PAP
- Principio "ultimo valor conocido" para GPS y BMS
- Build exitoso: RAM 5.3%, Flash 25.8%
