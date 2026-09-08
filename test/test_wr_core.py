#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import sys
import subprocess
from types import SimpleNamespace

from migen import Instance, Record

from litex.gen import LiteXModule
from litex.soc.integration.soc import SoCRegion
from litex.soc.interconnect.csr_bus import CSRBankArray

from litex_wr_nic.gateware.wr_core import WhiteRabbitCore, add_white_rabbit


def core_kwargs():
    return dict(
        cpu_firmware = "unused.bram",
        sfp_pads     = Record([(name, 1) for name in ("txp", "txn", "rxp", "rxn")]),
        sfp_i2c_pads = Record([("sda", 1), ("scl", 1)]),
    )


def test_core_import_has_no_nic_or_soc_side_effects():
    # Use a fresh interpreter: other tests import the compatibility NIC class.
    script = """
import sys
from litex.soc.integration.soc_core import SoCMini
before = (dict(SoCMini.csr_map), dict(SoCMini.mem_map))
from litex_wr_nic.gateware.wr_core import WhiteRabbitCore
assert 'liteeth.mac.sram' not in sys.modules
assert 'litepcie.frontend.ptm' not in sys.modules
from litex_wr_nic.gateware.soc import LiteXWRNICSoC
assert before == (SoCMini.csr_map, SoCMini.mem_map)
"""
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
    subprocess.run([sys.executable, "-c", script], env=env, check=True)


def test_standalone_core_interfaces():
    core = WhiteRabbitCore(SimpleNamespace(device="xc7a50t"), **core_kwargs())
    assert core.bus.addressing == "word"
    assert core.cpu_bus is None
    assert core.sink is core.wrf_stream2wb.sink
    assert core.source is core.wrf_wb2stream.source
    fragment = core.get_fragment()
    instances = [s for s in fragment.specials if isinstance(s, Instance)]
    assert len(instances) == 1
    uart = next(item.expr for item in instances[0].items if item.name == "uart_rxd_i")
    assert uart.value == 1


def test_compatibility_adapter_registers_memory_and_csrs_once():
    soc          = LiteXModule()
    soc.platform = SimpleNamespace(device="xc7a50t")
    masters      = {}
    slaves       = {}
    soc.bus = SimpleNamespace(
        add_master = lambda **kwargs: masters.update(kwargs),
        add_slave  = lambda **kwargs: slaves.update(kwargs),
    )
    region = SoCRegion(origin=0x50000000, size=128*1024)
    core = add_white_rabbit(soc, cpu_memory_region=region, **core_kwargs())
    assert masters["region"] is region
    assert masters["master"] is core.cpu_bus
    assert slaves["slave"] is core.bus
    banks = CSRBankArray(soc, lambda name, memory: 5)
    assert [name for name, *_ in banks.banks] == ["wr_cpu_bridge"]
    assert [csr.name for csr in banks.banks[0][1]] == [
        "status", "error_count", "last_error_address",
    ]
    fragment = soc.get_fragment()
    assert sum(isinstance(s, Instance) and s.of == "xwrc_board_litex_wr_nic_wrapper"
        for s in fragment.specials) == 1
