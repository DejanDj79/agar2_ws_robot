#!/bin/bash
echo "AgAR Jetson Service Restart"

sudo systemctl restart agar_jetson.service

echo "AgAR Jetson Service successfully restarted!"
