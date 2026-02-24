# Flujo del sistema IoT — Scooter (ESP32 + SIM7600)

**Para presentación — lenguaje simple, audiencia no técnica**

---

## ¿Qué es esto?

- Un **scooter** (o vehículo) que lleva un dispositivo con **ESP32** (cerebro) y **SIM7600** (módulo 4G).
- Se comunica con un **servidor en la nube** para reportar su posición y recibir órdenes (por ejemplo: desbloquear o bloquear).

---

## Flujo en 6 puntos

1. **Arranque**
   - El dispositivo se enciende, se conecta por 4G a la red del operador y abre una conexión con el servidor.

2. **Envío de posición (cada 5 segundos)**
   - El scooter envía al servidor: ubicación (lat/lon), batería y velocidad.
   - El servidor usa estos datos para mostrar el scooter en un mapa en tiempo casi real.

3. **Recepción de comandos**
   - El servidor puede enviar órdenes al scooter en cualquier momento (por ejemplo: “desbloquear” o “bloquear”).
   - El dispositivo revisa con frecuencia si llegó alguna orden, para responder rápido.

4. **Confirmación al servidor**
   - Cuando el scooter recibe una orden, responde de inmediato: “recibí y ejecuté la orden”.
   - Así el backend (y la app) saben que el comando llegó bien.

5. **Reconexión automática**
   - Si se pierde la conexión (4G o servidor), el dispositivo intenta reconectarse solo, sin intervención del usuario.

6. **Resumen**
   - **Ida:** el scooter manda posición y estado cada pocos segundos.  
   - **Vuelta:** el servidor manda comandos (unlock/lock) y el scooter los ejecuta y confirma.

---

## Esquema muy simple



```
[Scooter / ESP32 + SIM7600]  <——4G——>  [Servidor en la nube]
        |                                      |
        |  Envía: posición, batería, velocidad (cada 5 s)
        |  ————————————————————————————>
        |                                      |
        |  Recibe: "desbloquear" / "bloquear"  |
        |  <————————————————————————————
        |                                      |
        |  Envía: "OK, ejecutado" (ACK)        |
        |  ————————————————————————————>
```

**Sobre el ">"** — Cada vez que el scooter envía algo (posición o ACK), por dentro pasa esto: el ESP32 le dice al módulo 4G "prepárate"; el módulo responde **">"** (listo, escribe); entonces el ESP32 escribe y envía el mensaje. El **">"** es solo esa señal de "listo" del módulo.

---

## Mensajes clave para la audiencia

- El scooter **no está solo**: habla con la nube de forma continua.
- La **posición** se actualiza cada pocos segundos para el mapa.
- Las **órdenes** (unlock/lock) se reciben y se **confirman** al instante.
- Si hay fallos de red, el sistema **reintenta solo** para volver a conectar.
