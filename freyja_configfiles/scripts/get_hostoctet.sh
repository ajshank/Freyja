#! /bin/bash
# Extract last octet from provided the interface (wlan0, eth0 etc).
# Echoes 0 if nothing found, or error.
# Usage: >> get_hostoctet.sh wlan0
HOST_OCTET=$(ifconfig ${1} | awk '/inet / {print $2}' | awk -F. '{print $4}')
case $HOST_OCTET in
    ''|*[!0-9]*) echo 0 ;;
    *) echo $HOST_OCTET ;;
esac
