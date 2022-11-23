#!/bin/bash
set -v
set -e
sudo nmcli c up Hotspot
sudo ip link add br0 type bridge
sudo ip link set br0 up
sudo ip addr add dev br0 10.66.171.50/24
sudo ip link set enp0s25 promisc on
sudo ip link set enp0s25 master br0
sudo ip link set wlp2s0 promisc on
sudo ip link set wlp2s0 master br0
