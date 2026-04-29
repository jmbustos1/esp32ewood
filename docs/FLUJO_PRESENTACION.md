# Flujo del sistema IoT — Scooter (ESP32 + SIM7600)

**Para presentacion — lenguaje simple, audiencia no tecnica**

---

## Que es esto?

- Un **scooter electrico** que lleva un dispositivo con **ESP32** (cerebro), **SIM7600** (modulo 4G con GPS integrado) y un **sistema de gestion de bateria (BMS)**.
- Se comunica con un **servidor en la nube** para reportar su posicion y estado, y recibir ordenes.
- Lee datos reales de la bateria y controla el bloqueo/desbloqueo fisico del scooter.

---

## Flujo en 7 puntos

1. **Arranque**
   - El dispositivo se enciende, activa el GPS, conecta con el BMS y se conecta por 4G al servidor.

2. **Envio de posicion y estado (cada 5 segundos)**
   - El scooter envia al servidor: ubicacion GPS real, porcentaje de bateria, velocidad, voltaje y si esta bloqueado o desbloqueado.
   - El servidor usa estos datos para mostrar el scooter en un mapa en tiempo real.

3. **Lectura de bateria**
   - El BMS (sistema de gestion de bateria) reporta continuamente: nivel de carga, voltaje, corriente y estado de proteccion.
   - Si la bateria tiene un problema (sobrevoltaje, temperatura, etc.), el sistema lo detecta automaticamente.

4. **Recepcion de comandos**
   - El servidor puede enviar ordenes al scooter en cualquier momento: "desbloquear" o "bloquear".
   - El dispositivo revisa con frecuencia si llego alguna orden.

5. **Ejecucion del comando**
   - Cuando llega una orden de desbloqueo, el ESP32 la reenvia al sistema de actuadores del scooter, que ejecuta la accion fisicamente.
   - Inmediatamente confirma al servidor: "recibido y ejecutado".

6. **Reconexion automatica**
   - Si se pierde la conexion (4G o servidor), el dispositivo intenta reconectarse solo, con tiempos de espera crecientes para no saturar la red.

7. **Ultimo dato conocido**
   - Si el GPS pierde senal momentaneamente o el BMS no responde, el sistema sigue reportando el ultimo dato valido. Nunca deja de enviar informacion.

---

## Esquema

```
[Scooter / ESP32]
     |
     ├── UART2 ←→ [SIM7600 (4G + GPS)]  ←——4G——→  [Servidor en la nube]
     |                                                     |
     |    Envia: posicion, bateria, velocidad, lock_state (cada 5s)
     |    ——————————————————————————————————>
     |                                                     |
     |    Recibe: "desbloquear" / "bloquear"               |
     |    <——————————————————————————————————
     |                                                     |
     |    Envia: "OK, ejecutado" (ACK)                     |
     |    ——————————————————————————————————>
     |
     └── UART1 ←→ [Electronica del scooter]
                       |
                       ├── BMS: bateria, voltaje, temperatura
                       └── Actuador: lock / unlock fisico
```

---

## Datos que envia el scooter

| Dato | Fuente | Ejemplo |
|------|--------|---------|
| Ubicacion (lat/lon) | GPS del SIM7600 | -36.886767, -73.044400 |
| Velocidad | GPS | 15.3 km/h |
| Bateria (%) | BMS via UART | 50% |
| Voltaje | BMS via UART | 39.11 V |
| Estado candado | Actuador via UART | bloqueado / desbloqueado |

---

## Mensajes clave para la audiencia

- El scooter **reporta datos reales** de GPS y bateria, no simulados.
- La **posicion** se actualiza cada 5 segundos para el mapa.
- Las **ordenes** (desbloquear/bloquear) se ejecutan fisicamente en el scooter y se **confirman** al instante.
- Si hay fallos de red, el sistema **reintenta solo** con tiempos crecientes.
- Si pierde GPS o BMS momentaneamente, **sigue reportando** con el ultimo dato valido.
- La comunicacion con la bateria y actuadores usa un **protocolo industrial** con validacion de datos (checksum).
