/**
 * @file config_local.example.h
 * @brief Template para config_local.h — copiar a `config_local.h` y llenar.
 *
 * INSTRUCCIONES:
 *   cp include/config_local.example.h include/config_local.h
 *   # editar include/config_local.h con los valores reales del dispositivo
 *
 * NUNCA commitear `config_local.h` (esta en .gitignore).
 */

#ifndef CONFIG_LOCAL_H
#define CONFIG_LOCAL_H

/* Identificador unico del scooter (cada dispositivo fisico tiene el suyo). */
#define SCOOTER_ID         0    /* <-- cambiar */

/* APN de la operadora movil (Entel M2M, Movistar, Claro, etc.). */
#define APN_NAME           "YOUR_APN_HERE"
#define APN_USER           "YOUR_APN_USER_HERE"
#define APN_PASS           "YOUR_APN_PASS_HERE"

/* Endpoint del backend. Puede ser hostname (requiere DNS) o IP. */
#define SCOOTER_TCP_HOST   "YOUR_BACKEND_HOST_HERE"
#define SCOOTER_TCP_PORT   0    /* <-- cambiar (ej. 8201) */

/* Secret compartido con el backend. Debe coincidir con env DEVICE_SECRET
 * del server. NO commitear el valor real. */
#define DEVICE_SECRET      "YOUR_DEVICE_SECRET_HERE"

#endif /* CONFIG_LOCAL_H */
