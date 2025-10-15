#!/bin/bash
REALPATH=$(realpath "$0")
BIN_DIR=$(dirname "$REALPATH")

# Path to the robot config file
CONFIG_FILE="$BIN_DIR/../robots/robots.cfg"

HOSTS_FILE="/etc/hosts"

# Backup the original hosts file
cp "$HOSTS_FILE" "${HOSTS_FILE}.bak"

# Function to parse a line and extract value by key
extract_value() {
    echo "$1" | awk -F';' -v key="$2" '{
        for(i=1; i<=NF; i++) {
            if ($i ~ key " =") {
                sub(/^.*= /, "", $i);
                print $i;
                break;
            }
        }
    }'
}

# Read and process the config file
while IFS= read -r line || [ -n "$line" ]; do
    # Extract fields by key names
    name=$(extract_value "$line" "name")
    lan=$(extract_value "$line" "lan")
    wlan=$(extract_value "$line" "wlan")

    # Skip the line if any of the required fields are missing
    if [[ -z "$name" || -z "$lan" || -z "$wlan" ]]; then
        continue
    fi

    # Remove any existing entries with the same name
    sudo sed -i.bak "/[[:space:]]$name$/d" "$HOSTS_FILE" && rm "${HOSTS_FILE}.bak"

    # Append new entries for lan and wlan IPs
    echo "$lan $name" | sudo tee -a "$HOSTS_FILE" > /dev/null
    echo "$lan $name.lan" | sudo tee -a "$HOSTS_FILE" > /dev/null
    echo "$wlan $name" | sudo tee -a "$HOSTS_FILE" > /dev/null
    echo "$wlan $name.wlan" | sudo tee -a "$HOSTS_FILE" > /dev/null
done < "$CONFIG_FILE"

echo "Hosts file has been updated."