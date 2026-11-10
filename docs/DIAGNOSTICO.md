# Diagnóstico del firmware (bloque `d` de la telemetría)

Desde el 2026-11-10 el firmware agrega un bloque `"d":{...}` a la telemetría TCP que envía cada ~6 s. Sirve para que el backend sepa qué está pasando dentro del scooter: qué firmware corre, si se reinició y por qué, en qué estado está la conectividad y si el GPS y el BMS responden.

- **Código**: `src/diag.c` / `src/diag.h`. Se arma en `sim7600_send_scooter_update` (`src/sim7600.c`).
- **Costo**: solo lee valores en memoria. No envía comandos AT ni toma el UART, así no interfiere con la FSM ni con el TCP.
- **Apagado**: `DIAG_TELEMETRY_ENABLED 0` en `src/app_config.h`.
- **Backend**: guarda el último bloque en `scooters.last_diag` (JSONB) y registra en el log cuando cambia un campo clave. Ver "Del lado del backend" abajo.

## Cuándo se envía

No va en todos los mensajes:

- Cada `DIAG_EVERY_N_MESSAGES` mensajes (10, unos 60 s).
- **Y además apenas cambia un campo clave**: `st`, `tr`, `hb`, `gr`, `gv`, `bv` o `rr`. El primer mensaje después de arrancar siempre lo incluye.

Si el bloque no cabe en el buffer (512 bytes; en el peor caso el JSON completo usa ~341), no se envía, pero la telemetría sale igual. Nunca rompe el JSON.

## Ejemplo

```json
{"scooter_id":4,"device_secret":"…","latitude":-36.797062,"longitude":-73.059963,
 "battery_level":76,"speed_kmh":0.0,"lock_state":"locked","voltage":39.11,
 "d":{"fw":"a1b2c3d","up":5321,"rr":1,"st":5,"ps":4,"ts":4800,"tr":6,"hb":0,
      "gv":1,"fa":3,"gr":0,"bv":1}}
```

## Campos

| Clave | Nombre | Tipo / unidad | Significado |
|---|---|---|---|
| `fw` | Versión de firmware | texto (≤16) | `git describe` al compilar (hash corto). Termina en `-dirty` si se compiló con cambios sin commitear. |
| `up` | Uptime | segundos | Tiempo desde el último arranque del ESP32. Si baja, hubo un reinicio. |
| `rr` | Motivo del arranque | código | `esp_reset_reason()` del arranque actual. Tabla abajo. |
| `st` | Estado de la FSM | código | Estado actual de la conectividad (`src/connectivity.h`). Tabla abajo. |
| `ps` | Estado previo | código | Estado anterior de la FSM (desde dónde llegó a `st`). |
| `ts` | Tiempo en el estado | segundos | Cuánto lleva en `st`. Un `ts` alto fuera de `RUNNING` indica que quedó pegado. |
| `tr` | Transiciones | contador | Cambios de estado de la FSM desde el arranque. Si sube rápido, la conexión está inestable. |
| `hb` | Disparos del heartbeat | contador | Veces que el heartbeat (Fix D, CPU 1) forzó recuperación por 90 s sin `CIPSEND` OK. |
| `gv` | GPS válido | 0/1 | 1 si la posición enviada es un fix de menos de 150 s. 0 = la lat/lng enviada es vieja o 0,0. |
| `fa` | Edad del fix | segundos | Desde el último fix real. **-1 = nunca hubo fix desde el arranque** (en ese caso lat/lng = 0,0). |
| `gr` | Reciclajes del GNSS | contador | Veces que el Fix H apagó y prendió el motor GNSS (cada 30 lecturas sin fix, contando errores). |
| `bv` | BMS válido | 0/1 | 1 si el BMS respondió. 0 = `battery_level` y `voltage` no son reales. |

### `st` / `ps`: estados de la FSM

| Código | Estado | Qué hace |
|---|---|---|
| 0 | `MODULE_OFFLINE` | El SIM7600 no responde por UART. Reintenta `AT` cada 30 s. |
| 1 | `MODULE_READY` | Responde `AT`; falta configurar red. |
| 2 | `NET_DOWN` | Red sin abrir. Reintenta `NETOPEN` cada 30 s. |
| 3 | `NET_READY` | Red abierta; falta el socket. |
| 4 | `SOCKET_DOWN` | Socket cerrado. Reintenta `CIPOPEN` con backoff (10 s → 5 min). |
| 5 | `RUNNING` | Red y socket OK; telemetría normal. |

Un mensaje solo llega al backend estando en `RUNNING`, así que `st` casi siempre llega en 5. Lo útil es `ps`, `ts` y `tr`: cuentan por dónde pasó antes de volver.

### `rr`: motivo del arranque (ESP-IDF 5.5)

| Código | Motivo | Lectura |
|---|---|---|
| 1 | `POWERON` | Encendido o corte de energía. En el ESP32 clásico **el botón EN también se reporta así**. |
| 3 | `SW` | `esp_restart()` (reinicio pedido por el firmware). |
| 4 | `PANIC` | Excepción o crash. Revisar logs seriales. |
| 5 | `INT_WDT` | Watchdog de interrupciones: algo bloqueó una interrupción. |
| 6 | `TASK_WDT` | Watchdog de tareas: una tarea no devolvió la CPU. |
| 7 | `WDT` | Otro watchdog (RTC). |
| 9 | `BROWNOUT` | Caída de voltaje. Revisar alimentación o batería. |
| 0, 2, 8, 10+ | Otros | Poco probables en este hardware (desconocido, pin externo, deep sleep, SDIO, USB, JTAG…). |

## Cómo leerlo: casos típicos

| Síntoma en `d` | Interpretación |
|---|---|
| `up` bajó y `rr`=1 | Se cortó la energía o apretaron EN. |
| `up` bajó y `rr`=4, 5, 6 o 9 | Crash, watchdog o brownout: problema de firmware o de alimentación. |
| `gv`=0, `fa`=-1 por horas | El GPS nunca tuvo fix desde el arranque. Si `gr` no sube, el reciclaje no se está disparando; si sube, el motor no revive reciclando (caso scooter 4, nov 2026). |
| `gv`=0, `fa` alto, `gr` sube | Tuvo fix y lo perdió. Bajo techo o motor colgado. |
| `tr` y `hb` suben seguido | Conexión inestable; el heartbeat está forzando recuperaciones. |
| `bv`=0 | La batería reportada (0) no es real: el BMS no responde. |

## Del lado del backend

- El backend ignora campos que no conoce: un firmware con `d` funciona con un backend sin soporte, y al revés.
- Guarda el último bloque recibido en `scooters.last_diag` (JSONB) y la hora en `scooters.last_diag_at`.
- Escribe un log `📟` cuando cambia `fw`, `rr`, `ps`, `tr`, `hb`, `gr`, `gv` o `bv`, o cuando `up` baja (reinicio).

```sql
SELECT id, last_diag, last_diag_at FROM scooters ORDER BY id;
```

## Evolución

Al agregar o cambiar un campo: actualizar esta tabla, mantener las claves cortas y no reutilizar una clave con otro significado. Planificado para una etapa 2, con comandos AT a baja frecuencia: señal (`AT+CSQ`), tecnología de red (`AT+CPSI?`), satélites (`AT+CGNSSINFO`) y estado del motor GNSS (`AT+CGPS?`).
