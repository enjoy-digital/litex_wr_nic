#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from liteeth.phy.a7_gtp import QPLLSettings, QPLL

# Shared QPLL --------------------------------------------------------------------------------------

class SharedQPLL(LiteXModule):
    def __init__(self, platform, with_pcie=False, with_eth=False, eth_refclk_freq=125e6, eth_refclk_from_pll=True):
        self.platform = platform
        # PCIe QPLL Settings.
        qpll_pcie_settings = QPLLSettings(
            refclksel  = 0b001,
            fbdiv      = 5,
            fbdiv_45   = 5,
            refclk_div = 1,
        )

        # Ethernet QPLL Settings.
        qpll_eth_settings = QPLLSettings(
            refclksel  = {True: 0b111, False: 0b001}[eth_refclk_from_pll],
            fbdiv      = 4,
            fbdiv_45   = {125e6: 5, 156.25e6 : 4}[eth_refclk_freq],
            refclk_div = 1,
        )

        # QPLL Configs.
        class QPLLConfig:
            def __init__(self, refclk, settings):
                self.refclk   = refclk
                self.settings = settings

        self.configs = configs = {}
        if with_pcie:
            configs["pcie"] = QPLLConfig(
                refclk   = ClockSignal("refclk_pcie"),
                settings = qpll_pcie_settings,
            )
        if with_eth:
            configs["eth"] = QPLLConfig(
                refclk   = ClockSignal("refclk_eth"),
                settings = qpll_eth_settings,
            )

        # Shared QPLL.
        self.qpll        = None
        self.channel_map = {}
        # Single QPLL configuration.
        if len(configs) == 1:
            name, config = next(iter(configs.items()))
            gtrefclk0, gtgrefclk0 = self.get_gt_refclks(config)
            self.qpll = QPLL(
                gtrefclk0     = gtrefclk0,
                gtgrefclk0    = gtgrefclk0,
                qpllsettings0 = config.settings,
                gtrefclk1     = None,
                gtgrefclk1    = None,
                qpllsettings1 = None,
            )
            self.channel_map[name] = 0
         # Dual QPLL configuration.
        elif len(configs) == 2:
            config_items = list(configs.items())
            gtrefclk0, gtgrefclk0 = self.get_gt_refclks(config_items[0][1])
            gtrefclk1, gtgrefclk1 = self.get_gt_refclks(config_items[1][1])
            self.qpll = QPLL(
                gtrefclk0     = gtrefclk0,
                gtgrefclk0    = gtgrefclk0,
                qpllsettings0 = config_items[0][1].settings,
                gtrefclk1     = gtrefclk1,
                gtgrefclk1    = gtgrefclk1,
                qpllsettings1 = config_items[1][1].settings,
            )
            self.channel_map[config_items[0][0]] = 0
            self.channel_map[config_items[1][0]] = 1

    def enable_pll_refclk(self):
        self.platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

    @staticmethod
    def get_gt_refclks(config):
        if config.settings.refclksel == 0b111:
            return None, config.refclk
        else:
            return config.refclk, None

    def get_channel(self, name):
        if name in self.channel_map:
            channel_index = self.channel_map[name]
            return self.qpll.channels[channel_index]
        else:
            raise ValueError(f"Invalid QPLL name: {name}")