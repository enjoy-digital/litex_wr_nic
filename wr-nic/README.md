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

## Repository

Both gateware and software are present in the same repo:

```bash
git clone https://ohwr.org/project/wr-nic.git --recursive
cd wr-nic

# required because master is old and fixes are present
# with recent kernel
git checkout 28c5db8b96a7bca8effda3db14f9c3898c0aaac5
```

## Gateware

```bash
cd wr-nic/syn/nic
hdlmake
make
```

And it's fails due to missing/wrong xilinx IPs.

```
Checking expanded design ...
ERROR:NgdBuild:604 - logical block
   'cmp_gn4124_core/cmp_l2p_dma_master/cmp_data_fifo' with type 'l2p_fifo' could
   not be resolved. A pin name misspelling can cause this, a missing edif or ngc
   file, case mismatch between the block name and the edif or ngc file name, or
   the misspelling of a type name. Symbol 'l2p_fifo' is not supported in target
   'spartan6'.
ERROR:NgdBuild:604 - logical block
   'cmp_gn4124_core/cmp_l2p_dma_master/cmp_addr_fifo' with type 'l2p_fifo' could
   not be resolved. A pin name misspelling can cause this, a missing edif or ngc
   file, case mismatch between the block name and the edif or ngc file name, or
   the misspelling of a type name. Symbol 'l2p_fifo' is not supported in target
   'spartan6'.

```

This is concern
*wr-nic/hdl/ip_cores/gn4124-core/hdl/gn4124core/rtl/l2p_dma_master.vhd*

By adding:
```
echo 'xfile add ../../ip_cores/gn4124-core/hdl/spec/ip_cores/l2p_fifo.ngc' >> $@
```

after `hdlmake` step and before `make` the bitstream is correctly builded

## Software

kernel/software part is located in *sw* sub-directory. Latest commits has for
goal to fix build with kernel 5.15. First step is to apply to custom patches to
fix build with kernel > 5.15.

An useful tool to deal with multiples patches is `quilt` (`apt install quilt`).

```bash
cd wr-nic
quilt import /somewhere/white_rabbit_wut_tests/wr-nic/patches/*.patch
```

It's now possible to built/install drivers with:
```bash
cd wr-nic/sw
make
```
