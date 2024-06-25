# Build a custom PCIe Gigabyte Ethernet on your own!    
# This project is based on [litex](https://github.com/enjoy-digital/litex).  
# Step 1: Replace the corresponding files of original litex library.
1. Make sure you have installed litex and know its installation path.
2. Append the content of [soc.py](https://github.com/tongchen126/fpga_pcie_ethernet/tree/main/litex/litex/soc/integration) in this repo to your local library path located in litex/litex/soc/integration/soc.py.
3. Copy [wishbone_dma.py](https://github.com/tongchen126/fpga_pcie_ethernet/tree/main/litepcie/litepcie/frontend) to your local library path located in litepcie/litepcie/frontend/.
4. Then override the original file of your local liteeth project with the corresponding [files](https://github.com/tongchen126/fpga_pcie_ethernet/tree/main/liteeth/liteeth/mac).

# Step 2: Generate bitstream and driver header.
Suppose you are using a KC705 FPGA board. 
In the origianl [kc705 board file](https://github.com/litex-hub/litex-boards/blob/26a7f13a7f70b3ac9557d92a919610bbc3deb563/litex_boards/targets/xilinx_kc705.py#L80), it has something like this.
```
# Ethernet ---------------------------------------------------------------------------------
if with_ethernet:
    self.submodules.ethphy = LiteEthPHY(
        clock_pads = self.platform.request("eth_clocks"),
        pads       = self.platform.request("eth"),
        clk_freq   = self.clk_freq)
    self.add_ethernet(phy=self.ethphy)

# PCIe -------------------------------------------------------------------------------------
if with_pcie:
    self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x4"),
        data_width = 128,
        bar0_size  = 0x20000)
    self.add_pcie(phy=self.pcie_phy, ndmas=1)
```
To add PCIe Gigabyte Ethernet support, you can replace the previous lines with the following:
```
self.submodules.ethphy = LiteEthPHY(
    clock_pads = self.platform.request("eth_clocks", 0),
    pads       = self.platform.request("eth", 0),
    clk_freq   = self.clk_freq)

self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                                 data_width=64,
                                 bar0_size=0x20000)
self.add_ethernet_pcie(phy=self.ethphy, pcie_phy=self.pcie_phy)
```
Then you can generate bitstream, and don't forget to specify the [generate driver option](https://github.com/litex-hub/litex-boards/blob/26a7f13a7f70b3ac9557d92a919610bbc3deb563/litex_boards/targets/xilinx_kc705.py#L159).

# Step 3:
You should have driver and bitstream generated. The driver should be located in your generated path (kc705/driver/kernel/).
Copy this [ethernet driver file](https://github.com/tongchen126/fpga_pcie_ethernet/blob/main/driver/main.c) to the generated driver path (kc705/driver/kernel/) and override main.c. The driver is ready!

# Appendix:
RX: 912Mbit/s  
![rx](https://user-images.githubusercontent.com/31961076/156330014-80af9075-c623-4905-b8da-2cf98bfd3e52.png)  
TX: 858Mbit/s  
![tx](https://user-images.githubusercontent.com/31961076/156330058-7715e5d0-626e-4150-a42c-6d3426a1ec12.png)  
RX(371Mbit/s) and TX(868Mbit/s):  
![rx_tx](https://user-images.githubusercontent.com/31961076/156330114-2d5f2750-5217-4660-91fa-5ea3226845ab.png)


sudo ip addr add 192.168.1.122/24 dev enp5s0
sudo ip link set enp5s0 up
ip -s link show enp5s0
ping -I enp5s0 192.168.1.1