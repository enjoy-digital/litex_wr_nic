
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

This repository also requires `hdlmake` to generates `Makefile`

```bash
git clone https://ohwr.org/project/hdl-make.git

cd hdl-make

# by the past master branch was broken
git checkout origin/develop -b develop

pip3 install --user -e .

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

### riscv firmware

- We assume repo is cloned at this repo root directory
- The firmware must be build before gateware


```bash
git clone https://ohwr.org/project/wrpc-sw.git
cd wrpc-sw
git checkout 5ac04dd53a16f3931c3cb8361dc0d6bdbbf82dc6
cp ../config_wrpc/acorn_defconfig configs/
make acorn_defconfig
make
cp wrc.bram ../
```

### Gateware
```bash
./acorn.py --build --load
```
