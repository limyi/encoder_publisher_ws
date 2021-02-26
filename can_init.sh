#!/bin/bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
cansend can0 000#0100
sleep 2
cansend can0 000#0100
sudo chmod 666 /dev/ttyUSB*
