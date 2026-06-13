#!/bin/bash
# =============================================================================
# check_network.sh — Quick health check for Pi + Arduino network
# =============================================================================
# Usage: sudo ./check_network.sh
# =============================================================================

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

ok()   { echo -e "  ${GREEN}✓${NC}  $1"; }
fail() { echo -e "  ${RED}✗${NC}  $1"; }
warn() { echo -e "  ${YELLOW}!${NC}  $1"; }
hdr()  { echo -e "\n${CYAN}── $1 ──${NC}"; }

ARDUINO_IPS=("192.168.1.101" "192.168.1.102" "192.168.1.103" "192.168.1.104" "192.168.1.105" "192.168.1.106")
ETHERNET_IFACE="eth0"

echo ""
echo "============================================================"
echo "  Network Health Check — Pi + Arduino x6"
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

# --- DHCP server status ------------------------------------------------------
hdr "DHCP Server"

if systemctl is-active --quiet isc-dhcp-server; then
    ok "isc-dhcp-server is running"
else
    fail "isc-dhcp-server is NOT running"
    echo "       Fix: sudo systemctl restart isc-dhcp-server"
fi

# --- Active leases -----------------------------------------------------------
hdr "Active DHCP Leases"

LEASES_FILE="/var/lib/dhcp/dhcpd.leases"
if [[ -f "$LEASES_FILE" ]]; then
    LEASE_COUNT=$(grep -c "^lease" "$LEASES_FILE" 2>/dev/null || echo 0)
    ok "$LEASE_COUNT lease(s) recorded in $LEASES_FILE"
    echo ""
    # Print a clean lease summary
    awk '
      /^lease/ { ip=$2 }
      /hardware ethernet/ { mac=$3; gsub(/;/,"",mac) }
      /binding state active/ { printf "    %-18s %s\n", ip, mac }
    ' "$LEASES_FILE"
else
    warn "No leases file found yet"
fi

# --- Ping each Arduino -------------------------------------------------------
hdr "Arduino Connectivity (ping)"

REACHABLE=0
for ip in "${ARDUINO_IPS[@]}"; do
    idx=$(( ${ip##*.} - 100 ))
    label="arduino$((idx+1)) ($ip)"
    if ping -c 1 -W 1 "$ip" &>/dev/null; then
        ok "$label — reachable"
        ((REACHABLE++))
    else
        fail "$label — no response"
    fi
done

# --- Summary -----------------------------------------------------------------
echo ""
echo "============================================================"
echo "  $REACHABLE / ${#ARDUINO_IPS[@]} Arduinos reachable"
echo "============================================================"
echo ""

if [[ $REACHABLE -lt ${#ARDUINO_IPS[@]} ]]; then
    echo "  Troubleshooting tips for unreachable Arduinos:"
    echo "  • Check physical Ethernet cable to that Arduino"
    echo "  • Ensure Ethernet Shield is properly seated on the Uno"
    echo "  • Verify MAC reservation in /etc/dhcp/dhcpd.conf"
    echo "  • Run: sudo ./update_mac_reservations.sh to refresh MACs"
    echo "  • Check switch port LEDs on the Cisco 3560-CX"
    echo ""
fi
