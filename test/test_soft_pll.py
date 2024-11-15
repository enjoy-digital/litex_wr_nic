#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import time
from litex import RemoteClient

bus = RemoteClient()
bus.open()

num_measurements    = 100
delay_between_tests = 1

for i in range(num_measurements):
    print(f"dac_refclk: {bus.regs.refclk_dac_current.read()}")
    print(f"dac_dmtd:   {bus.regs.dmtd_dac_current.read()}")

    time.sleep(delay_between_tests)

bus.close()
