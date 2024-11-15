//
// This file is part of LiteX-WR-NIC.
//
// Copyright (c) 2024 Warsaw University of Technology
// Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
// SPDX-License-Identifier: BSD-2-Clause

module ext_ram_tap (
    (* mark_debug = "true" *)
    input  wire [31:0]  ext_ram_i_adr,
    (* mark_debug = "true" *)
    output wire [31:0]  ext_ram_i_dat_r,

    output wire [31:0]  ext_ram_o_adr,
    input  wire [31:0]  ext_ram_o_dat_r
);

    assign ext_ram_o_adr   = ext_ram_i_adr;
    assign ext_ram_i_dat_r = ext_ram_o_dat_r;
endmodule
