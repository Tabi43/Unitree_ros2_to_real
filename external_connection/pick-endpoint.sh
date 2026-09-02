#!/bin/ash
# Entrypoint del bridge zenoh: sceglie il primo endpoint davvero raggiungibile
# e ci lancia sopra il bridge.
#
# Perche' serve: zenoh non sa farlo da solo. Il suo connect.timeout_ms e' un budget
# GLOBALE per tutta la lista di endpoint, non un timeout per singolo endpoint, e un
# endpoint morto costa un timeout TCP intero (~135 s misurati verso il robot con il
# cavo staccato: il SYN finisce nel vuoto, nessun RST, nessun ICMP). Quindi:
#   - con timeout_ms: -1  il bridge resta incastrato sul primo endpoint all'infinito
#                         e non prova mai gli altri (nessun topic, container "Up");
#   - con timeout_ms finito gli endpoint morti esauriscono il budget prima che i vivi
#                         abbiano il loro turno e il processo esce
#                         ("Failed to start Zenoh runtime ... Exiting").
# Qui invece ogni candidato costa al massimo 2 s: prima il link fisico, poi la porta.
set -u

# "iface|host|porta", in ordine di preferenza: vince il primo che risponde.
ENDPOINTS="${ZENOH_ENDPOINTS:-eno1|192.168.123.161|7447 wlo1|10.186.13.10|7447 tailscale0|100.120.120.52|7447}"

for e in $ENDPOINTS; do
  iface="${e%%|*}"; rest="${e#*|}"; host="${rest%%|*}"; port="${rest##*|}"

  # 1) Link fisico giu' (cavo staccato): scartato in microsecondi, senza toccare la
  #    rete. E' questo il gate che evita il buco nero da 135 s.
  if [ "$(cat "/sys/class/net/${iface}/carrier" 2>/dev/null)" != "1" ]; then
    echo "[pick] ${iface}: link giu', salto"
    continue
  fi

  # 2) Link su ma peer assente (es. siamo su un'altra rete): lo scopre in 2 s.
  if ! nc -z -w 2 "$host" "$port" 2>/dev/null; then
    echo "[pick] ${iface}: ${host}:${port} non risponde, salto"
    continue
  fi

  echo "[pick] ${iface}: ${host}:${port} OK -> avvio il bridge"
  # exec: il bridge diventa PID 1, cosi' "docker stop" lo segnala direttamente.
  exec /zenoh-bridge-ros2dds "$@" -e "tcp/${host}:${port}#iface=${iface}"
done

# Nessun endpoint disponibile. Usciamo in errore: con "--restart unless-stopped"
# Docker rilancia il container e il probe riparte da solo, senza bisogno di un
# loop di supervisione qui dentro.
echo "[pick] nessun endpoint raggiungibile" >&2
exit 1
