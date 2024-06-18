//-----------------------------------------------------------------------------
//
// (c) Copyright 2010-2011 Xilinx, Inc. All rights reserved.
//
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
//
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
//
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
//
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
//
//-----------------------------------------------------------------------------
// Project    : Series-7 Integrated Block for PCI Express
// File       : pcie_s7_gt_common.v
// Version    : 3.3
`timescale 1ns / 1ps

(* DowngradeIPIdentifiedWarnings = "yes" *)
module pcie_s7_gt_common
(
input               	PIPE_CLK,
input               	QPLL_QPLLPD,
input               	QPLL_QPLLRESET,
output                  QPLL_QPLLLOCK,
output                  QPLL_QPLLOUTCLK,
output                  QPLL_QPLLOUTREFCLK
);

        //---------- QPLL Wrapper ------------------------------------------
pcie_s7_qpll_wrapper qpll_wrapper_i
        (
        //---------- QPLL Clock Ports --------------
            .QPLL_GTGREFCLK                 (PIPE_CLK),
            .QPLL_QPLLLOCKDETCLK            (1'd0),
            .QPLL_QPLLOUTCLK                (QPLL_QPLLOUTCLK),
            .QPLL_QPLLOUTREFCLK             (QPLL_QPLLOUTREFCLK),
            .QPLL_QPLLLOCK                  (QPLL_QPLLLOCK),
        //---------- QPLL Reset Ports --------------
            .QPLL_QPLLPD                    (QPLL_QPLLPD),
            .QPLL_QPLLRESET                 (QPLL_QPLLRESET),
        //---------- QPLL DRP Ports ----------------
            .QPLL_DRPCLK                    (1'd0),
            .QPLL_DRPADDR                   (1'd0),
            .QPLL_DRPEN                     (1'd0),
            .QPLL_DRPDI                     (1'd0),
            .QPLL_DRPWE                     (1'd0),
            .QPLL_DRPDO                     (),
            .QPLL_DRPRDY                    ()
        );
  
endmodule
