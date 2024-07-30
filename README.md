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

### Gateware
Official way requires `hdlmake` to generates `Makefile`

```bash
git clone https://ohwr.org/project/hdl-make.git

cd hdl-make

# by the past master branch was broken
git checkout origin/develop -b develop

pip3 install --user -e .

```

### Software

White-rabbit contains a riscv the firmware must be build (for security) with
the toolchain recommended (see [section 2.2](https://ohwr.org/project/wr-cores/wikis/uploads/7cf8d2161b6e5fa86348455bbd022196/wrpc-user-manual-v5.0.pdf):


```bash
wget https://ohwr.org/project/wrpc-sw/wikis/uploads/9f9224d2249848ed3e854636de9c08dc/riscv-11.2-small.tgz
tar xJf riscv-11.2-small.tgz
```

## Install

wr-cores repository

```bash
git clone https://ohwr.org/project/wr-cores.git

cd wr-cores

# checkout one commit before wrpc-v5 tag (wrpc-v5 tag is broken for clbv3:
# missing firmware).
git checkout 39825ec55291cb12492090093f27a50f9d0b73d9 -b wrpc-v5

git submodule update --init

```


### Building clv3 reference design (Artix based board)

Before calling `hdlmake` vivado must be in `PATH`

Official way:

```bash

cd syn/clbv3_ref_design

hdlmake

make

```

## Through LiteX

```bash
./clbv3.py --build
```

This script is only a wrapper for clbv3 target (vivado project + xdc)

## Specifics files for a board

These are located in three distincts location:

- *top/board_name*
  * a `.bmm` file (memory map)
  * the top: more or less instanciate a subtop + some signal
    handling/connections
  * the xdc file
  * Manifest.py (required for `hdlmake` and useless here)
- *syn/board_name*
- *board/board_name*

## LiteX Acorn Baseboard mini

**NOTE**: SFP I2C interface is unconnected by default: *J1* and *J4* must be
closed using jumpers.

### riscv firmware

- We assume repo is cloned at this repo root directory
- The firmware must be build before gateware

**NOTE:** the riscv toolchain must be available in `PATH`

```bash
git clone https://ohwr.org/project/wrpc-sw.git
cd wrpc-sw
git checkout 5ac04dd53a16f3931c3cb8361dc0d6bdbbf82dc6
cp ../firmware/speca7_defconfig configs/
make speca7_defconfig
make
cp wrc.bram ../firmware/speca7_wrc.bram
```

### Gateware
```bash
./acorn.py --build --load
```

## Xilinx ZC706

**NOTE** firmware must be updated (TBD) to configure PCA9548 (i2c switch) to
permit access to the SFP. By default only the switch (0x74 is visible).

See [configuration details](https://www.ti.com/lit/ds/symlink/pca9548a.pdf?ts=1712311423911&ref_url=https%253A%252F%252Fwww.google.com%252F)

**NOTE** *ZC706* has no onboard UART interface. An external USB <-> UART must be
connected to PMOD1 Pins 2,3 (J58).

### status

- SFP is working, sync and comm too
- no flash available
- SFP i2c requires modifying *wrpc-sw* to add code to enable i2c switch
- an external, 125MHz, must be tested instead of PLL's output

### riscv firmware

Currently this is similar to the acorn

### Gateware

```
./acorn.py --csr-csv=csr.csv --build --load
```

### GTX debug or DAC control (litescope)

```
litex_server --jtag --jtag-config=openocd_xc7z_smt2-nc.cfg
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
