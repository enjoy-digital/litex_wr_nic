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

# Gateware

```bash
cd wr-nic/syn/nic
hdlmake
make
```

And it's fails due to missing/wrong xilinx IPs.
