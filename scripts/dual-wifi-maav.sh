#!/usr/bin/env bash
# Connect MWireless (internet on wlo1) + MAAVNetwork (drone AP on wlo1_1).
# MAAVNetwork uses manual IPv4 (no DHCP). Laptop: 10.77.0.4/24.
# Requires sudo to manage the second virtual Wi-Fi interface.
set -euo pipefail

PHY=phy0
PRIMARY=wlo1
SECONDARY=wlo1_1
INET_CONN=MWireless
DRONE_CONN=MAAVNetwork
# Your address on the drone AP (from Network settings)
CLIENT_IP=10.77.0.4/24
# Common drone/companion addresses on this subnet — pinged after connect
DRONE_CANDIDATES=(10.77.0.1 10.77.0.2 10.77.0.3 10.77.0.10 10.77.0.100 10.77.0.123)
DRONE_BSSID=

if [[ ${1:-} == "down" ]]; then
  if [[ $EUID -ne 0 ]]; then exec sudo "$0" "$@"; fi
  nmcli connection down "$DRONE_CONN" 2>/dev/null || true
  nmcli device disconnect "$SECONDARY" 2>/dev/null || true
  iw dev "$SECONDARY" del 2>/dev/null || true
  echo "Removed $SECONDARY; MWireless left as-is."
  exit 0
fi

if [[ $EUID -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

# Clean recreate of secondary vif (dual-vif can get stuck and hide 2.4 GHz)
nmcli connection down "$DRONE_CONN" 2>/dev/null || true
nmcli device disconnect "$SECONDARY" 2>/dev/null || true
iw dev "$SECONDARY" del 2>/dev/null || true
sleep 1

iw phy "$PHY" interface add "$SECONDARY" type managed

# Distinct MAC so the driver allows two stations
PRIMARY_MAC=$(cat /sys/class/net/"$PRIMARY"/address)
IFS=: read -r b0 b1 b2 b3 b4 b5 <<<"$PRIMARY_MAC"
b0=$(printf '%02x' $(( (0x$b0 | 0x02) & 0xfe )))
b5=$(printf '%02x' $(( (0x$b5 + 1) & 0xff )))
SECONDARY_MAC="$b0:$b1:$b2:$b3:$b4:$b5"
ip link set "$SECONDARY" down || true
ip link set "$SECONDARY" address "$SECONDARY_MAC"
ip link set "$SECONDARY" up
nmcli device set "$SECONDARY" managed yes

nmcli connection modify "$INET_CONN" \
  connection.interface-name "$PRIMARY" \
  ipv4.never-default no \
  ipv4.route-metric 600

# Static IP — this AP does not hand out DHCP leases
nmcli connection modify "$DRONE_CONN" \
  connection.interface-name "$SECONDARY" \
  connection.autoconnect no \
  802-11-wireless.ssid MAAVNetwork \
  802-11-wireless.mode infrastructure \
  802-11-wireless.cloned-mac-address permanent \
  ipv4.method manual \
  ipv4.addresses "$CLIENT_IP" \
  ipv4.gateway '' \
  ipv4.never-default yes \
  ipv4.route-metric 2000 \
  ipv4.may-fail no \
  ipv6.method disabled

if [[ -n "${DRONE_BSSID}" ]]; then
  nmcli connection modify "$DRONE_CONN" 802-11-wireless.bssid "$DRONE_BSSID"
else
  nmcli connection modify "$DRONE_CONN" 802-11-wireless.bssid ''
fi

echo "Ensuring $INET_CONN is up..."
nmcli connection up "$INET_CONN" ifname "$PRIMARY" || true

echo "Looking for MAAVNetwork..."
found=0
for i in $(seq 1 8); do
  nmcli device wifi rescan ifname "$SECONDARY" 2>/dev/null || nmcli device wifi rescan 2>/dev/null || true
  sleep 2
  if nmcli -t -f SSID device wifi list | grep -qx MAAVNetwork; then
    found=1
    nmcli -f SSID,CHAN,SIGNAL,BSSID device wifi list | grep -i MAAV || true
    break
  fi
  echo "  scan $i/8: not visible yet"
done

if [[ $found -ne 1 ]]; then
  echo
  echo "MAAVNetwork is not broadcasting right now."
  echo "Power on / wake the drone AP, confirm the SSID shows in:"
  echo "  nmcli device wifi list | grep -i MAAV"
  echo "Then re-run: $0"
  echo "To clean up the extra iface: $0 down"
  exit 1
fi

echo "Connecting $DRONE_CONN on $SECONDARY (static $CLIENT_IP)..."
nmcli connection up "$DRONE_CONN" ifname "$SECONDARY"

echo
echo "Active connections:"
nmcli -f NAME,DEVICE,TYPE connection show --active
echo
ip -br addr
ip route
echo
echo "Internet:"
ping -c 1 -W 2 8.8.8.8 || true
echo
echo "Probing 10.77.0.0/24 for the drone..."
alive=()
for ip in "${DRONE_CANDIDATES[@]}"; do
  if ping -c 1 -W 1 "$ip" &>/dev/null; then
    echo "  up: $ip"
    alive+=("$ip")
  fi
done
# Quick sweep of .1–.20 if none of the candidates answered
if [[ ${#alive[@]} -eq 0 ]]; then
  for i in $(seq 1 20); do
    ip="10.77.0.$i"
    [[ "$ip" == "10.77.0.4" ]] && continue
    if ping -c 1 -W 1 "$ip" &>/dev/null; then
      echo "  up: $ip"
      alive+=("$ip")
    fi
  done
fi
echo
if [[ ${#alive[@]} -gt 0 ]]; then
  echo "SSH:  ssh maav@drone1.local   (or: ssh drone1)"
else
  echo "No hosts answered ping. Try:  nmap -sn 10.77.0.0/24"
fi
