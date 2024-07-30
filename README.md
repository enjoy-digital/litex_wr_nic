```
                           __   _ __      _  __   _      _____      _  ___________
                          / /  (_) /____ | |/_/__| | /| / / _ \____/ |/ /  _/ ___/
                         / /__/ / __/ -_)>  </___/ |/ |/ / , _/___/    // // /__
                        /____/_/\__/\__/_/|_|    |__/|__/_/|_|   /_/|_/___/\___/
                        LiteX based White Rabbit PCIe NIC with PTM support.
                        Copyright (c) 2024 Warsaw University of Technology
                              Copyright (c) 2012-2024 Enjoy-Digital
```

![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)

## Prerequisites

### Software

White-rabbit contains a riscv the firmware must be build (for security) with
the toolchain recommended (see [section 2.2](https://ohwr.org/project/wr-cores/wikis/uploads/7cf8d2161b6e5fa86348455bbd022196/wrpc-user-manual-v5.0.pdf):


```bash
wget https://ohwr.org/project/wrpc-sw/wikis/uploads/9f9224d2249848ed3e854636de9c08dc/riscv-11.2-small.tgz
tar xJf riscv-11.2-small.tgz
```

## LiteX Acorn Baseboard mini

**NOTE**: SFP I2C interface is unconnected by default: *J1* and *J4* must be
closed using jumpers.

### riscv firmware

A pre-compiled firmware is provided, to rebuild it:

```bash
git clone https://ohwr.org/project/wrpc-sw.git
cd wrpc-sw
git checkout 5ac04dd53a16f3931c3cb8361dc0d6bdbbf82dc6
cp ../firmware/speca7_defconfig configs/
make speca7_defconfig
make
cp wrc.bram ../firmware/speca7_wrc.bram
```
**NOTE:** the riscv toolchain must be available in `PATH`

### Gateware
```bash
./acorn.py --build --load
```

## SDB

see **wrpc-user-manual-v5.0.pdf** and *wrpc-sw/tools/sdbfs.README*

A file must be stored into the SPI flash with parameters (cal, sfp, ...).

in *wrpc-sw* (after having build firmware):

```bash
./tools/gensdbfs -b 65536 tools/sdbfs-flash /tmp/sdb-wrpc.bin
```

*SDBFS* update/rewrite:
```
wrc# sfp erase
wrc# sfp add AXGE-1254-0531 180750 148326 1235332 333756144
wrc# sfp add AXGE-3454-0531 180750 148326 -1235332 333756144
```

## Test Notes

```
./acorn.py --csr-csv=csr.csv --build --load
litex_server --jtag --jtag-config=openocd_xc7_ft2232.cfg

socat - UDP-DATAGRAM:255.255.255.255:24000,broadcast
enter something
litescope_cli -r main_basesoc_basesoc_wrf_src_stb
```

## LiteEth NIC Notes

[> Build
--------

```
./acorn.py --build --flash
cd software/driver
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
