[> Build
--------

```
./acorn.py --build --flash
cd driver
make clean all
sudo ./init
```
[> Test
--------

Ping:
```
ping -I enp5s0 192.168.1.1
```

Iperf3 on Server:
```
iperf3 -s -B 192.168.1.121

```

Iperf3 on Client:
```
iperf3 -c 192.168.1.121 -B 192.168.1.92

```
