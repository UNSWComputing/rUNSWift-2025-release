#!/usr/bin/env bash

# find where xauthority is mounted from in host
xauthority_on_host=$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/home/ubuntu/.Xauthority"}}{{.Source}}{{end}}{{end}}' runswift-devcontainer 2>/dev/null)

# if it is mounted, and the file doesn't exist, then down the current container
if [[ $? -eq 0 && ! -z "$xauthority_on_host" && ! -e $xauthority_on_host ]]; then
    echo "Your .Xauthority inside the container was mounted from a host path that didn't exist. Recreating container..."
    docker compose down -t 3
fi

# start up the docker compose setup
# note: it is preferable to do this here instead of letting vscode do it since vscode randomly decides to build the image
# when it isn't necessary... :/
docker compose up -d --no-recreate