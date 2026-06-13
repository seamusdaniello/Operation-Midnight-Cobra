#!/bin/bash
# =============================================================================
# update_mac_reservations.sh
# Run this AFTER the first discovery boot to pull real MACs from DHCP leases
# and update dhcpd.conf automatically.
# =============================================================================
# Usage:
#   chmod +x update_mac_reservations.sh
#   sudo ./update_mac_reservations.sh
# =============================================================================

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC}  $1"; }
success() { echo -e "${GREEN}[OK]${NC}    $1"; }
warning() { echo -e "${YELLOW}[WARN]${NC}  $1"; }
error()   { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

[[ $EUID -ne 0 ]] && error "Run this script with sudo."

LEASES_FILE="/var/lib/dhcp/dhcpd.leases"
DHCPD_CONF="/etc/dhcp/dhcpd.conf"

# --- Check leases file exists ------------------------------------------------
[[ ! -f "$LEASES_FILE" ]] && error "No leases file found at $LEASES_FILE. Make sure DHCP server has run and Arduinos have connected."

echo ""
echo "============================================================"
echo "  MAC Address Discovery & Reservation Update"
echo "============================================================"
echo ""

# --- Parse leases file for IP + MAC pairs ------------------------------------
info "Reading DHCP leases..."

declare -A IP_TO_MAC

while IFS= read -r line; do
    if [[ "$line" =~ ^lease\ ([0-9.]+) ]]; then
        current_ip="${BASH_REMATCH[1]}"
    fi
    if [[ "$line" =~ hardware\ ethernet\ ([0-9a-f:]+)\; ]]; then
        IP_TO_MAC["$current_ip"]="${BASH_REMATCH[1]}"
    fi
done < "$LEASES_FILE"

if [[ ${#IP_TO_MAC[@]} -eq 0 ]]; then
    error "No leases found. Connect Arduinos to the switch and wait for them to request IPs, then re-run."
fi

# --- Display discovered devices ----------------------------------------------
echo "  Discovered devices on network:"
echo ""
printf "  %-18s %-20s\n" "IP Address" "MAC Address"
printf "  %-18s %-20s\n" "----------" "-----------"

SORTED_IPS=($(for ip in "${!IP_TO_MAC[@]}"; do echo "$ip"; done | sort -t. -k4 -n))
for ip in "${SORTED_IPS[@]}"; do
    printf "  %-18s %-20s\n" "$ip" "${IP_TO_MAC[$ip]}"
done

echo ""

# --- Assign discovered MACs to Arduino slots (by IP order) -------------------
RESERVED_IPS=("192.168.1.101" "192.168.1.102" "192.168.1.103" "192.168.1.104" "192.168.1.105" "192.168.1.106")
HOSTNAMES=("arduino1" "arduino2" "arduino3" "arduino4" "arduino5" "arduino6")

# Filter to only IPs in the Arduino reserved range
ARDUINO_IPS=()
for ip in "${SORTED_IPS[@]}"; do
    octets=(${ip//./ })
    last=${octets[3]}
    if [[ $last -ge 100 && $last -le 200 ]]; then
        ARDUINO_IPS+=("$ip")
    fi
done

if [[ ${#ARDUINO_IPS[@]} -eq 0 ]]; then
    error "No devices found in the 192.168.1.100-200 range. Make sure Arduinos are connected and powered."
fi

info "Found ${#ARDUINO_IPS[@]} Arduino(s) in DHCP range"

# --- Confirm with user before writing ----------------------------------------
echo ""
echo "  The following reservations will be written to dhcpd.conf:"
echo ""
printf "  %-12s %-20s %-18s\n" "Hostname" "MAC Address" "Reserved IP"
printf "  %-12s %-20s %-18s\n" "--------" "-----------" "-----------"

for i in "${!ARDUINO_IPS[@]}"; do
    current_ip="${ARDUINO_IPS[$i]}"
    mac="${IP_TO_MAC[$current_ip]}"
    reserved_ip="${RESERVED_IPS[$i]}"
    hostname="${HOSTNAMES[$i]}"
    printf "  %-12s %-20s %-18s\n" "$hostname" "$mac" "$reserved_ip"
done

echo ""
read -p "  Write these reservations to dhcpd.conf? [y/N] " confirm
[[ "$confirm" != "y" && "$confirm" != "Y" ]] && { info "Aborted. No changes made."; exit 0; }

# --- Backup and rewrite dhcpd.conf -------------------------------------------
info "Backing up existing dhcpd.conf..."
cp "$DHCPD_CONF" "${DHCPD_CONF}.bak.$(date +%s)"
success "Backup saved"

info "Writing updated dhcpd.conf..."

cat > "$DHCPD_CONF" <<EOF
# =============================================================================
# dhcpd.conf — Raspberry Pi DHCP server for Arduino Uno x6
# Updated by update_mac_reservations.sh on $(date)
# =============================================================================

default-lease-time 86400;
max-lease-time 86400;
authoritative;

subnet 192.168.1.0 netmask 255.255.255.0 {
    range 192.168.1.100 192.168.1.200;
    option subnet-mask 255.255.255.0;
    option routers 192.168.1.1;
    option domain-name-servers 8.8.8.8;

EOF

for i in "${!ARDUINO_IPS[@]}"; do
    current_ip="${ARDUINO_IPS[$i]}"
    mac="${IP_TO_MAC[$current_ip]}"
    reserved_ip="${RESERVED_IPS[$i]}"
    hostname="${HOSTNAMES[$i]}"

    cat >> "$DHCPD_CONF" <<EOF
    host $hostname {
        hardware ethernet $mac;
        fixed-address $reserved_ip;
    }

EOF
done

echo "}" >> "$DHCPD_CONF"

success "dhcpd.conf updated"

# --- Validate and restart ----------------------------------------------------
info "Validating config..."
dhcpd -t -cf "$DHCPD_CONF" && success "Config valid" || warning "Config has warnings — check manually"

info "Restarting DHCP server..."
systemctl restart isc-dhcp-server
success "DHCP server restarted"

echo ""
echo "============================================================"
echo -e "  ${GREEN}MAC reservations updated successfully!${NC}"
echo "============================================================"
echo ""
echo "  Arduinos will now always receive the same IP on reconnect."
echo "  Simulink target IPs to configure:"
echo ""
for i in "${!ARDUINO_IPS[@]}"; do
    echo "    ${HOSTNAMES[$i]} → ${RESERVED_IPS[$i]}"
done
echo ""
echo "  Test connectivity:"
for i in "${!ARDUINO_IPS[@]}"; do
    echo "    ping ${RESERVED_IPS[$i]}"
done
echo ""
