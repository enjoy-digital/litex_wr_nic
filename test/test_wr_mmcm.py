#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Exercise generated backend RTL and real 7-series MMCM models with XSim."""

import shutil
import subprocess
from pathlib import Path

import pytest

from migen import *

from litex.gen import LiteXModule

from litex.build.xilinx import XilinxPlatform
from litex.soc.cores.clock import S7MMCM

from litex_wr_nic.gateware.wr_clock import WRMMCMBackend

# Constants ----------------------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parents[1]

# Simulation Helpers -------------------------------------------------------------------------------

class MMCMTestSoC(LiteXModule):
    def __init__(self, input_freq=None, output_freq=None, outputs=1):
        self.cd_sys = ClockDomain("sys")
        self.cd_wr  = ClockDomain("wr")
        self.cd_ps  = ClockDomain("ps")
        self.backend = backend = WRMMCMBackend("ps", "wr", timeout_cycles=64)
        ports = {
            "command_data":  backend.command.data,
            "command_load":  backend.command.load,
            "psen":          backend.psen,
            "psincdec":      backend.psincdec,
            "busy":          backend.busy,
            "fault":         backend.fault,
            "steps":         backend.steps,
            "accepted_data": backend.cdc.source.data,
            "accepted_load": backend.cdc.source.load,
            "status":        backend._status.status,
            "status_steps":  backend._steps.status,
            "superseded":    backend.cdc.superseded,
        }
        self.ios = {cd.clk for cd in (self.cd_sys, self.cd_wr, self.cd_ps)}
        self.ios.update(cd.rst for cd in (self.cd_sys, self.cd_wr, self.cd_ps))
        if input_freq is None:
            ports["psdone"] = backend.psdone
        else:
            self.cd_input = ClockDomain("input")
            self.ios.add(self.cd_input.clk)
            self.mmcm = mmcm = S7MMCM(speedgrade=-3, fractional=False)
            mmcm.register_clkin(self.cd_input.clk, input_freq)
            mmcm.expose_dps("ps", with_csr=False)
            for index in range(outputs):
                cd = ClockDomain(f"output{index}")
                setattr(self, f"cd_output{index}", cd)
                mmcm.create_clkout(cd, output_freq, margin=0)
                mmcm.params[f"p_CLKOUT{index}_USE_FINE_PS"] = "TRUE"
                self.ios.add(cd.clk)
            self.comb += [
                mmcm.reset.eq(self.cd_ps.rst),
                mmcm.psen.eq(backend.psen),
                mmcm.psincdec.eq(backend.psincdec),
                backend.psdone.eq(mmcm.psdone),
            ]
            ports["psdone"] = mmcm.psdone
            ports["locked"] = mmcm.locked
            self.config = mmcm.compute_config()

        # Explicit top-level port names keep the HDL bench independent of
        # Migen's generated internal names. Accepted commands are observed at
        # the CDC output; accumulator state is not part of the test interface.
        for name, source in ports.items():
            source.name_override = name
            self.ios.add(source)


def run_xsim(tmp_path, dut, bench, parameters=""):
    for tool in ("xvlog", "xelab", "xsim"):
        if shutil.which(tool) is None:
            pytest.skip(f"{tool} is required for the MMCM RTL regression")
    platform = XilinxPlatform("xc7a200tsbg484-3", [])
    platform.get_verilog(dut, ios=dut.ios, name="wr_mmcm_dut").write(str(tmp_path / "dut.v"))
    (tmp_path / "parameters.vh").write_text(parameters)

    def run(tool, *args):
        result = subprocess.run([tool, *map(str, args)],
            cwd            = tmp_path,
            capture_output = True,
            text           = True,
            timeout        = 180,
        )
        output = result.stdout + result.stderr
        (tmp_path / f"{tool}-output.log").write_text(output)
        assert result.returncode == 0, output
        assert "ERROR:" not in output, output
        return output

    vivado = Path(shutil.which("xsim")).resolve().parents[1]
    run("xvlog", "--sv", vivado / "data/verilog/src/glbl.v",
        tmp_path / "dut.v", ROOT / "test/hdl" / bench)
    run("xelab", "work.wr_mmcm_tb", "work.glbl", "-L", "unisims_ver", "--snapshot", "wr_mmcm_tb")
    output = run("xsim", "wr_mmcm_tb", "--runall")
    # XSim can return zero after a fatal assertion or Tcl startup failure.
    assert "PASS: MMCM" in output, output
    assert not any(message in output for message in ("Fatal:", "Error:", "Warning: [Unisim")), output

# Backend RTL Tests --------------------------------------------------------------------------------

def test_acorn_mmcm_configuration_supports_fine_phase_shifting():
    from acorn_wr_nic import _CRG
    from litex_boards.platforms import sqrl_acorn

    crg = _CRG(sqrl_acorn.Platform(), 125e6)
    for mmcm, frequency in [(crg.refclk_mmcm, 125e6), (crg.dmtd_mmcm, 62.5e6)]:
        config = mmcm.compute_config()
        assert config["clkfbout_mult"] == int(config["clkfbout_mult"])
        assert config["clkout0_divide"] == int(config["clkout0_divide"])
        assert config["clkout0_freq"] == frequency
        assert mmcm.params["p_CLKOUT0_USE_FINE_PS"] == "TRUE"


def test_mmcm_full_width_commands_resets_and_completion(tmp_path):
    run_xsim(tmp_path, MMCMTestSoC(), "wr_mmcm_backend_tb.sv")


@pytest.mark.parametrize("input_freq,output_freq,outputs", [
    (100e6, 125e6,  2), # M2SDR reference, both outputs use fine phase shifting.
    (100e6, 62.5e6, 1), # M2SDR DMTD.
    (200e6, 125e6,  1), # Acorn reference.
    (200e6, 62.5e6, 1), # Acorn DMTD.
])
def test_mmcm_unisim_clock_phase_and_wraparound(tmp_path, input_freq, output_freq, outputs):
    dut = MMCMTestSoC(input_freq, output_freq, outputs)
    config = dut.config
    # UG472: interpolated fine phase shifting requires integer division.
    assert config["clkfbout_mult"] == int(config["clkfbout_mult"])
    assert config["clkout0_divide"] == int(config["clkout0_divide"])
    parameters = (
        f"localparam real INPUT_PERIOD = {1e12/input_freq};\n"
        f"localparam real OUTPUT_PERIOD = {1e12/output_freq};\n"
        f"localparam real PHASE_STEP = {1e12/config['vco']/56};\n"
        f"localparam integer OUTPUTS = {outputs};\n"
        f"localparam integer WRAP_STEPS = {int(config['clkout0_divide'])*56};\n"
        + ("`define SECOND_OUTPUT\n" if outputs == 2 else "")
    )
    run_xsim(tmp_path, dut, "wr_mmcm_unisim_tb.sv", parameters)
