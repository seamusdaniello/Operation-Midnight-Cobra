#!/bin/bash
# =============================================================================
# check_network.sh — Quick health check for Pi + Arduino network
# All addresses are static (no DHCP) — defined in network/config.yaml.
# =============================================================================
# Usage: ./check_network.sh
# =============================================================================

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

ok()   { echo -e "  ${GREEN}✓${NC}  $1"; }
fail() { echo -e "  ${RED}✗${NC}  $1"; }
hdr()  { echo -e "\n${CYAN}── $1 ──${NC}"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ETHERNET_IFACE="eth0"

echo ""
echo "============================================================"
echo "  Network Health Check — Pi + Arduino team"
echo "  $(date)"
echo "============================================================"

# --- Pi interface status -----------------------------------------------------
hdr "Pi Interface ($ETHERNET_IFACE)"

PI_IP=$(ip addr show "$ETHERNET_IFACE" 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)
if [[ -n "$PI_IP" ]]; then
    ok "Interface up — IP: $PI_IP"
else
    fail "No IP on $ETHERNET_IFACE"
fi

# --- Load Arduino IPs from config.yaml ---------------------------------------
hdr "Arduino Connectivity (ping)"

mapfile -t ARDUINO_ENTRIES < <(python3 - "$SCRIPT_DIR/../network/config.yaml" <<'PYEOF'
import sys
import yaml

with open(sys.argv[1]) as f:
    cfg = yaml.safe_load(f)

for name, info in cfg.get("arduinos", {}).items():
    ip = info.get("ip")
    if ip:
        print(f"{name} {ip}")
PYEOF
)

if [[ ${#ARDUINO_ENTRIES[@]} -eq 0 ]]; then
    fail "No Arduino IPs found in config.yaml (only TODO placeholders so far?)"
fi

REACHABLE=0
for entry in "${ARDUINO_ENTRIES[@]}"; do
    read -r name ip <<< "$entry"
    if ping -c 1 -W 1 "$ip" &>/dev/null; then
        ok "$name ($ip) — reachable"
        ((REACHABLE++))
    else
        fail "$name ($ip) — no response"
    fi
done

# --- Summary -----------------------------------------------------------------
echo ""
echo "============================================================"
echo "  $REACHABLE / ${#ARDUINO_ENTRIES[@]} Arduinos reachable"
echo "============================================================"
echo ""

if [[ $REACHABLE -lt ${#ARDUINO_ENTRIES[@]} ]]; then
    echo "  Troubleshooting tips for unreachable Arduinos:"
    echo "  • Check physical Ethernet cable to that Arduino"
    echo "  • Ensure Ethernet Shield is properly seated on the Uno"
    echo "  • Verify the static IP set in that Arduino's own sketch"
    echo "  • Check switch port LEDs on the Cisco 3560-CX"
    echo ""
fi
