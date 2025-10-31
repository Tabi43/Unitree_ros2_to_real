#!/usr/bin/env bash
set -Eeuo pipefail
echo "[SETUP-DDS] Genero cyclonedds.xml con autodiscovery peers..."

: "${TEMPLATE_PATH:=}"                # preferito: path al template
: "${CYCLONEDDS_URI:=}"               # fallback: file://...
: "${FINAL_XML:=/etc/cyclonedds/cyclonedds.xml}"
: "${DDSPeers:=}"                     # override manuale "ip1;ip2" o "ip1,ip2"
: "${DDSPeers_SCAN:=1}"               # 1=ping veloce, 0=solo ARP/override
: "${DDSPeers_RANGE:=10-30}"          # range leggero per /24 (override se vuoi)
: "${PING_TIMEOUT:=0.2}"              # secondi

# 0) risolvi template
if [[ -z "$TEMPLATE_PATH" ]]; then TEMPLATE_PATH="${CYCLONEDDS_URI#file://}"; fi
if [[ -z "$TEMPLATE_PATH" || ! -f "$TEMPLATE_PATH" ]]; then
  echo "[SETUP-DDS] Template non trovato: $TEMPLATE_PATH" >&2; exit 1
fi
install -d -m 0755 "$(dirname "$FINAL_XML")"

# helper: uniq preservando ordine
uniq_keep() { awk '!a[$0]++'; }

# 1) seed manuale
SEED="$(echo "$DDSPeers" | tr ',' ';' | tr -s ';' '\n' | sed '/^$/d' | uniq_keep)"

# 2) interfacce IPv4 globali
# formato riga: "<if> <cidr> <ip> <pref>"
IFINFO="$(ip -o -4 addr show scope global 2>/dev/null | \
  awk '!/docker|br-|veth/ {
    gsub("/", " ");
    # campi: $2=if, $4=ip, $5=pref
    printf "%s %s/%s %s %s\n", $2, $4, $5, $4, $5
  }')"

# candidati da ARP
CANDIDATES="$(echo "$IFINFO" | while read -r IF CIDR IP PREF; do
  ip neigh show dev "$IF" 2>/dev/null | awk '{print $1}'
done | sed '/^$/d' | uniq_keep)"

# 3) scansione veloce opzionale (/24)
if [[ "$DDSPeers_SCAN" = "1" && -n "$IFINFO" ]]; then
  SCAN_TMP="$(mktemp /tmp/dds.scan.XXXXXX)"
  echo "$IFINFO" | while read -r IF CIDR IP PREF; do
    [[ "$PREF" != "24" ]] && continue
    BASE="$(echo "$IP" | awk -F. '{print $1"."$2"."$3"."}')"
    IFS='-' read -r L H <<<"$DDSPeers_RANGE"
    for o in $(seq "$L" "$H"); do
      TGT="${BASE}${o}"
      [[ "$TGT" == "$IP" ]] && continue
      timeout "$PING_TIMEOUT" ping -c1 -W1 "$TGT" >/dev/null 2>&1 && echo "$TGT"
    done
  done | uniq_keep > "$SCAN_TMP" 2>/dev/null || true
  if [[ -s "$SCAN_TMP" ]]; then
    CANDIDATES="$(printf "%s\n%s\n" "$CANDIDATES" "$(cat "$SCAN_TMP")" | uniq_keep)"
  fi
  rm -f "$SCAN_TMP"
fi

# 4) hint specifici per rete robot 192.168.123.0/24
if echo "$IFINFO" | grep -qE ' 192\.168\.123\.[0-9]+ '; then
  HINTS_TMP="$(mktemp /tmp/dds.hints.XXXXXX)"
  for h in 192.168.123.13 192.168.123.14 192.168.123.15 192.168.123.161; do
    timeout "$PING_TIMEOUT" ping -c1 -W1 "$h" >/dev/null 2>&1 && echo "$h"
  done | uniq_keep > "$HINTS_TMP" 2>/dev/null || true
  if [[ -s "$HINTS_TMP" ]]; then
    CANDIDATES="$(printf "%s\n%s\n" "$CANDIDATES" "$(cat "$HINTS_TMP")" | uniq_keep)"
  fi
  rm -f "$HINTS_TMP"
fi

# 5) unisci: manuale + scoperti, verifica reachability finale
ALLPEERS="$(printf "%s\n%s\n" "$SEED" "$CANDIDATES" | sed '/^$/d' | uniq_keep | while read -r ip; do
  timeout "$PING_TIMEOUT" ping -c1 -W1 "$ip" >/dev/null 2>&1 && echo "$ip"
done | uniq_keep)"

# se nessun peer, copia il template com'è
if [[ -z "$ALLPEERS" ]]; then
  cp -f -- "$TEMPLATE_PATH" "$FINAL_XML"
  echo "[SETUP-DDS] Nessun peer raggiungibile. Template copiato senza modifiche in: $FINAL_XML"
  exit 0
fi

# 6) scrivi XML: inietta <Peer/>; se <Peers> manca, lo crea
if grep -qE '<Peers([[:space:]>]|/>)' "$TEMPLATE_PATH"; then
  awk -v peers="$ALLPEERS" '
    function emit(p){ n=split(p,a,"\n"); for(i=1;i<=n;i++) if(length(a[i])) printf("        <Peer address=\"%s\"/>\n",a[i]); }
    /<Peers([[:space:]>]|\/>)/ { print; emit(peers); next }
    { print }
  ' "$TEMPLATE_PATH" > "$FINAL_XML"
else
  awk -v peers="$ALLPEERS" '
    function emitblock(p){
      print "      <Peers>";
      n=split(p,a,"\n");
      for(i=1;i<=n;i++) if(length(a[i])) printf("        <Peer address=\"%s\"/>\n",a[i]);
      print "      </Peers>";
    }
    /<\/Discovery>/ { emitblock(peers); found=1; print; next }
    { print }
    END {
      if (!found) {
        print "    <Discovery>";
        emitblock(peers);
        print "    </Discovery>";
      }
    }
  ' "$TEMPLATE_PATH" > "$FINAL_XML"
fi

echo "[SETUP-DDS] Peers trovati:"
echo "$ALLPEERS" | sed 's/^/  - /'
echo "[SETUP-DDS] File scritto in: $FINAL_XML"
