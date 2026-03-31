/**
 * @file gps.h
 * @brief Modulo GPS via SIM7600 (AT+CGPS / AT+CGPSINFO).
 *        Mantiene ultimo fix valido; si no hay fix retorna la ultima posicion conocida.
 */

#ifndef GPS_H
#define GPS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Datos de posicion GPS */
typedef struct {
    double latitude;        /**< Grados decimales (negativo = sur) */
    double longitude;       /**< Grados decimales (negativo = oeste) */
    float  speed_kmh;       /**< Velocidad sobre tierra en km/h */
    float  altitude;        /**< Altitud MSL en metros */
    float  course;          /**< Rumbo en grados */
    bool   valid;           /**< true si hay fix valido al menos una vez */
} gps_position_t;

/**
 * Inicia sesion GPS en modo standalone (AT+CGPS=1,1).
 * Llamar una vez durante el setup, idealmente lo mas temprano posible
 * para dar tiempo al TTFF (Time To First Fix).
 *
 * @return 0 OK, -1 error AT
 */
int gps_init(void);

/**
 * Detiene sesion GPS (AT+CGPS=0).
 * @return 0 OK, -1 error
 */
int gps_stop(void);

/**
 * Consulta AT+CGPSINFO, parsea la respuesta y actualiza la posicion interna.
 * Si no hay fix (campos vacios), mantiene la ultima posicion valida.
 *
 * @return 0 si se obtuvo fix nuevo, 1 si se retorna posicion anterior (sin fix), -1 error AT
 */
int gps_update(void);

/**
 * Retorna la ultima posicion conocida.
 * Si nunca hubo fix, valid==false y lat/lon son 0.
 */
gps_position_t gps_get_position(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
