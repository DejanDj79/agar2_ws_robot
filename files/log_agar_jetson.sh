#!/bin/bash
echo "AgAR Jetson Service Log"

journalctl _SYSTEMD_INVOCATION_ID=`systemctl show -p InvocationID --value agar_jetson.service`
