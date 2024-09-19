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

[> Intro
--------

This project aims to create a White Rabbit NIC based on LiteX, LitePCIe, and LiteEth with PCIe PTM
support. The project currently has two designs:

1. **White Rabbit Core NIC:**
   - Includes a White Rabbit console on the UART interface.
   - Uses White Rabbit Fabric interface for network capabilities.

2. **LiteEth 1000BaseX NIC:**
   - Acts as a Linux NIC.
   - Tested with Iperf3.

The final goal is to merge these designs, removing the LiteEth 1000BaseX PHY, and connect directly
to the White Rabbit fabric interface:

![](doc/architecture.png)

[> Prerequisites / System setup
-------------------------------

These are required in order to build and use the FPGA design and associated software provided in
this project:
- Linux computer, PTM capable (Tested with Ubuntu 22.04.4 LTS).
- Python3, Xilinx Vivado installed.
- LiteX [installed]
  (https://github.com/enjoy-digital/litex/wiki/Installation#litex-installation-guide) and up to
  date.
- A LiteX-Acorn-Baseboard Mini.
- An Intel I225 board.
- A JTAG-HS2 Cable.
- A Logic Analyzer/Scope to observe PPS.

[> Xilinx PHY workaround / Implementation note
----------------------------------------------

From our understanding of the Xilinx PHY and [question]
(https://support.xilinx.com/s/question/0D54U00007HkzneSAB/receive-all-message-tlps-on-user-interface-7-series-fpga-integrated-block?language=en_US)
asked on Xilinx community forum, the Artix7's Xilinx PHY does not allow redirecting PTM TLP
messages to the AXI interface. For a PTM Requester, this then prevent receiving the PTM
Response/ResponseD TLP messages.

To work-around this limitation, a PCIePTMSniffer has been implemented: The module is sniffing the RX
Data between the GTPE2 and PCIE2 hardblocks and descrambling/decoding the PCIe traffic to
re-generate the PTM TLPs.

The re-generated PTM TLPs can then be re-injected in to LitePCIe core and use its PTM Depacketizer:

![](doc/ptm_sniffer.png)

The Xilinx PHY however allow generating the PTM Requests from the AXI interface, so a
PCIePTMInjector module hasn't been required.

[> Build and test White Rabbit NIC design
-----------------------------------------

The FPGA design can be build and tested with the following commands:

```sh
$ ./acorn_wr_nic.py --build --load
```

The WR console/gui should then be available on `/dev/ttyUSB2`:

```
wrc# gui

CLB3 WRPC Monitor wrpc-v5.0-9-g5ac04dd5 | Esc/q = exit; r = redraw

TAI Time: 1970-01-01-00:06:32  UTC offset: 0    PLL mode: BC  state: Locking
---+-------------------+-------------------------+---------+---------+-----
 # |        MAC        |       IP (source)       |    RX   |    TX   | VLAN
---+-------------------+-------------------------+---------+---------+-----
 0 | 22:33:44:55:66:77 |                         |       7 |     195 |    0

--- HAL ---|------------- PPSI ------------------------------------------------
 Itf | Frq |  Config   | MAC of peer port  |    PTP/EXT/PDETECT States   | Pro
-----+-----+-----------+-------------------+-----------------------------+-----
 wr0 |     | auto      | 00:00:00:00:00:00 | LISTENING/IDLE      /WA_MSG | R-W
Pro(tocol): R-RawEth, V-VLAN, U-UDP

--------------------------- Synchronization status ----------------------------
Link down, master mode or sync info not valid
```

The board should be should be able to communicate with another WR equipment and timing corrections
displayed even if not effective on the LiteX-Acorn-Baseboard-Mini.

When rebooting the Host PC, the board should also be enumerated and seen with `lspci` with PTM
capabilities.

TODO: Finish WR Fabric Interface <-> NIC to WR NIC functionnality.
TODO: Describe PTM tests.

[> Build and test LiteEth 1000BaseX NIC design
----------------------------------------------

The FPGA design can be build and tested with the following commands:

```sh
$ ./acorn_liteeth_nic.py --build --load
```

When rebooting the Host PC, the board should also be enumerated and seen with `lspci`.

The PCIe NIC driver can then be loaded:
```sh
cd software/driver
make clean all
sudo init.sh
```

The board can then be used as a Linux NIC and tested with tools like iperf3 ex:

Server (without the board):
```
iperf3 -s -B 192.168.1.122
```

Client (with the board):
```
iperf3 -c 192.168.1.122 -B 192.168.1.92
```

Note: Adapt the IP addresses to your network configuration.


[> Build the WR RISC-V firmware
-------------------------------

The WR Core integrates a RISC-V CPU running a WRPC firmware to configure and control peripherals and
provide status/diagnostics through the WRC console.

The project includes a pre-generated firmware integrated into the FPGA design at build-time. To
rebuild the firmware:

```sh
cd firmware
./build.py
```

This rebuilt firmware will then be used for the FPGA builds.

This rebuilt firmware will then be used for the FPGA builds. [Section 2.2]
(https://ohwr.org/project/wr-cores/wikis/uploads/7cf8d2161b6e5fa86348455bbd022196/wrpc-user-manual-v5.0.pdf)
recommends using a specific RISC-V toolchain to build the firmware  which `build.py` automatically
downloads (if not already done) and uses.

[> SDB
------

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
