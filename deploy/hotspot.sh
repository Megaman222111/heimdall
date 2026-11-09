#!/bin/bash
# Makes the Pi start the RadarTracker hotspot on every boot (dashboard: http://192.168.4.1/).
# While the hotspot is up the Pi has no internet, because wlan0 can't do both.
# Back online until next boot: sudo nmcli connection down RadarTracker
# Remove for good:             sudo nmcli connection delete RadarTracker
set -e
NAME=RadarTracker
PASS=tracker123

# On the hotspot only: every hostname resolves to the Pi, so phones find the sign-in page.
mkdir -p /etc/NetworkManager/dnsmasq-shared.d
echo "address=/#/192.168.4.1" > /etc/NetworkManager/dnsmasq-shared.d/captive.conf

nmcli connection delete "$NAME" 2>/dev/null || true
nmcli connection add type wifi ifname wlan0 con-name "$NAME" ssid "$NAME" autoconnect yes \
  connection.autoconnect-priority 100 \
  802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 1 \
  wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
  wifi-sec.psk "$PASS" \
  ipv4.method shared ipv4.addresses 192.168.4.1/24 ipv6.method disabled
if [ "$1" = "--now" ]; then
  nmcli connection up "$NAME"
  echo "Hotspot up: join '$NAME' (password $PASS), then open http://192.168.4.1/"
else
  echo "Hotspot saved; it starts on the next boot. Use --now to start it immediately."
fi
