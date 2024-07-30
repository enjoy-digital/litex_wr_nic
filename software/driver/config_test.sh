#!/usr/bin/bash
sudo ip addr add 192.168.0.10 dev enp1s0
sudo ip route add 192.168.0.2 dev enp1s0
