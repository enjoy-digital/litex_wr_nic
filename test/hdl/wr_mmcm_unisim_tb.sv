//
// This file is part of LiteX-WR-NIC.
//
// Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
// SPDX-License-Identifier: BSD-2-Clause

// Real MMCME2_ADV/BUFG/reset primitives from Vivado's unisims_ver library.
// Protocol and 1/56 VCO-period phase step: AMD UG472, Dynamic Phase Shift.
`timescale 1ps/1ps

module wr_mmcm_tb;
    `include "parameters.vh"

    reg sys_clk = 0, wr_clk = 0, ps_clk = 0, input_clk = 0;
    reg sys_rst = 1, wr_rst = 1, ps_rst = 1;
    reg [15:0] command_data = 32768;
    reg command_load = 0;
    wire psen, psincdec, psdone, busy, fault, locked;
    wire [31:0] steps, status_steps;
    wire [1:0] status;
    wire output0_clk;
    `ifdef SECOND_OUTPUT
        wire output1_clk;
    `endif

    wr_mmcm_dut dut (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .wr_clk(wr_clk),   .wr_rst(wr_rst),
        .ps_clk(ps_clk),   .ps_rst(ps_rst),
        .input_clk(input_clk),
        .command_data(command_data), .command_load(command_load),
        .psen(psen), .psincdec(psincdec), .psdone(psdone),
        .busy(busy), .fault(fault), .steps(steps),
        .status(status), .status_steps(status_steps), .locked(locked),
        `ifdef SECOND_OUTPUT
            .output1_clk(output1_clk),
        `endif
        .output0_clk(output0_clk)
    );

    always #4000 sys_clk = ~sys_clk;
    always #8000 wr_clk = ~wr_clk;
    always #2500 ps_clk = ~ps_clk;
    always #(INPUT_PERIOD/2) input_clk = ~input_clk;

    integer cycles = 0, issued = 0, completed = 0, started = 0, signed_steps = 0;
    reg pending = 0, active_direction = 0, previous_psen = 0, previous_psdone = 0;
    reg checking = 0;

    always @(posedge ps_clk) begin
        cycles = cycles + 1;
        if (checking) begin
            if (!locked || fault) $fatal(1, "MMCM lost lock or backend faulted");
            if (psen && previous_psen) $fatal(1, "PSEN wider than one cycle");
            if (psdone && previous_psdone) $fatal(1, "PSDONE wider than one cycle");
            if (pending && psincdec != active_direction)
                $fatal(1, "Direction changed before completion");
            if (psen) begin
                if (pending) $fatal(1, "Overlapping phase requests");
                pending = 1;
                active_direction = psincdec;
                started = cycles;
                issued = issued + 1;
            end
            if (psdone) begin
                if (!pending) $fatal(1, "Unsolicited completion");
                if (cycles - started != 12)
                    $fatal(1, "Unexpected completion latency: %0d", cycles - started);
                pending = 0;
                completed = completed + 1;
                signed_steps = signed_steps + (active_direction ? 1 : -1);
            end
        end
        previous_psen = psen;
        previous_psdone = psdone;
    end

    task command(input integer code);
        @(negedge wr_clk);
        command_data = code;
        command_load = 1;
        @(negedge wr_clk);
        command_load = 0;
    endtask

    function real wrap(input real value);
        wrap = value - $floor(value/OUTPUT_PERIOD)*OUTPUT_PERIOD;
    endfunction

    real baseline, edge_time, phase, expected, error;
    task check_phase;
        // Measure only after the command/PSDONE pipelines have drained.
        repeat (200) @(negedge ps_clk);
        if (busy || issued != completed || steps != completed)
            $fatal(1, "Completion accounting mismatch");
        if (status != 0 || status_steps != steps)
            $fatal(1, "CSR status did not converge");
        @(posedge output0_clk);
        edge_time = $realtime;
        phase = wrap(edge_time - baseline);
        expected = wrap(signed_steps*PHASE_STEP);
        error = wrap(phase - expected + OUTPUT_PERIOD/2) - OUTPUT_PERIOD/2;
        if (error > 10.0 || error < -10.0)
            $fatal(1, "Clock phase: steps=%0d measured=%0f expected=%0f error=%0f ps",
                signed_steps, phase, expected, error);
        @(posedge output0_clk);
        if ($realtime - edge_time < OUTPUT_PERIOD - 2.0 ||
            $realtime - edge_time > OUTPUT_PERIOD + 2.0)
            $fatal(1, "Incorrect settled output period");
        `ifdef SECOND_OUTPUT
            // Both fine-phase-enabled reference outputs must move together.
            fork
                begin @(posedge output0_clk); edge_time = $realtime; end
                begin @(posedge output1_clk); phase = $realtime; end
            join
            if (edge_time != phase) $fatal(1, "Reference outputs lost phase alignment");
        `endif
        $display("MMCM phase: signed_steps=%0d error=%0f ps", signed_steps, error);
    endtask

    integer before_steps;
    initial begin
        #200000;
        @(negedge ps_clk);
        sys_rst = 0;
        wr_rst = 0;
        ps_rst = 0;
        wait (locked);
        repeat (200) @(negedge ps_clk);
        checking = 1;
        @(posedge output0_clk);
        baseline = $realtime;
        check_phase();

        // Shift through more than a complete output period in each direction,
        // including all fine-phase and output-divider wrap boundaries.
        command(0);
        wait (completed >= WRAP_STEPS + 37);
        command(32768);
        check_phase();
        before_steps = completed;
        command(65535);
        wait (completed >= before_steps + 2*WRAP_STEPS + 73);
        command(32768);
        check_phase();

        // Reset during an outstanding shift, then acquire lock and tune again.
        command(0);
        wait (psen);
        repeat (3) @(negedge ps_clk);
        checking = 0;
        @(negedge ps_clk);
        ps_rst = 1;
        repeat (40) @(negedge ps_clk);
        ps_rst = 0;
        wait (!locked);
        wait (locked);
        repeat (200) @(negedge ps_clk);
        if (steps || busy || fault || psen) $fatal(1, "Reset did not restore neutral state");
        issued = 0;
        completed = 0;
        signed_steps = 0;
        pending = 0;
        checking = 1;
        @(posedge output0_clk);
        baseline = $realtime;
        command(0);
        wait (completed >= 17);
        command(32768);
        check_phase();
        $display("PASS: MMCM UNISIM phase, wraparound, completion and relock");
        $finish;
    end

    initial begin
        #2000000000;
        $fatal(1, "MMCM simulation watchdog");
    end
endmodule
