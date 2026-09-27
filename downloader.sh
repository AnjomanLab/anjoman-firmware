#!/bin/bash

for i in 1 2 3 4; do
    ip=$((149 + $i))
    curl http://192.168.242.$ip/status
    curl http://192.168.242.$ip/download -o robot_${i}_v2.bin
done
