//
// This file is part of LiteX-WR-NIC.
//
// Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
// SPDX-License-Identifier: BSD-2-Clause

module wr_mmcm_formal(input wire clk);
    localparam WIDTH = @WIDTH@;
    localparam TIMEOUT = @TIMEOUT@;
    localparam CENTER = 1 << (WIDTH - 1);

    (* anyseq *) reg rst;
    (* anyseq *) reg psdone;
    wire psen, psincdec, busy, fault, core_reset, accepted_load;
    wire [WIDTH-1:0] accepted_data;
    wire [31:0] steps;
    wr_mmcm_dut dut (
        .sys_clk(clk), .sys_rst(rst), .core_reset(core_reset),
        .psen(psen), .psincdec(psincdec), .psdone(psdone),
        .busy(busy), .fault(fault), .steps(steps),
        .accepted_data(accepted_data), .accepted_load(accepted_load)
    );

    reg past_valid = 0;
    reg outstanding = 0;
    reg active_direction = 0;
    reg [31:0] completed = 0;
    reg saw_increment = 0, saw_decrement = 0;

    always @(posedge clk) begin
        past_valid <= 1;
        if (!past_valid) assume(rst);
        if (core_reset) begin
            outstanding <= 0;
            active_direction <= 0;
            completed <= 0;
            saw_increment <= 0;
            saw_decrement <= 0;
        end else begin
            // Independent transaction tracker, driven only by MMCM pins.
            if (psen) begin
                assert(!outstanding);
                assert(busy);
                assert(!fault);
                outstanding <= 1;
                active_direction <= psincdec;
                if (psincdec) saw_increment <= 1;
                else saw_decrement <= 1;
            end
            if (psdone) begin
                outstanding <= 0;
                if (outstanding || psen) completed <= completed + 1;
            end
            if (outstanding && !psdone) assert(psincdec == active_direction);
            assert(busy == (outstanding || psen));
            assert(steps == completed);
            if (past_valid && !$past(core_reset)) begin
                if ($past(psen)) assert(!psen);
                if ($past(fault)) assert(fault);
            end

            // Non-vacuity: reach both directions, commands while busy,
            // neutral during a shift and completion concurrent with a load.
            cover(saw_increment && saw_decrement);
            cover(outstanding && accepted_load && accepted_data == CENTER);
            cover(outstanding && accepted_load &&
                (accepted_data < CENTER) != active_direction);
            cover(outstanding && psdone && accepted_load);
            if (TIMEOUT <= 32) cover(fault);
        end
    end
endmodule
