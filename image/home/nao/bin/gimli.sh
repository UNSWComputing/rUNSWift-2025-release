#!/usr/bin/env bash
cmd='aplay /home/nao/bin/gimli.wav'
sleep 300   # wait 3 minutes once
while true; do
  $cmd
  sleep 3
done

