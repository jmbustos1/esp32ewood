#!/bin/bash
# Prueba de conexión TCP al servidor del scooter (mismo host:puerto que app_config).
# Uso: ./test/nc_server.sh
# Con -z solo comprueba si el puerto está abierto; sin -z abre sesión interactiva.

HOST="3.237.198.242"
PORT="8201"

echo "Probando conexión a $HOST:$PORT (SCOOTER_TCP_HOST:SCOOTER_TCP_PORT)"
echo "  Con -z: solo verificar puerto abierto"
echo "  Sin -z: sesión interactiva (escribe y recibe NDJSON)"
echo ""

# Comprobar si el puerto está abierto (timeout 3 s)
if command -v nc &>/dev/null; then
  echo "  $ nc -vz -w 3 $HOST $PORT"
  nc -vz -w 3 "$HOST" "$PORT" && echo "  OK: puerto abierto" || echo "  FALLO: no se pudo conectar"
  echo ""
  echo "  Para sesión interactiva: nc -v $HOST $PORT"
else
  echo "  No se encontró 'nc'. Instala netcat-openbsd o netcat-traditional."
  echo "  Ejemplo manual: nc -v 3.237.198.242 8201"
fi
