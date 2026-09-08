//
// This file is part of LiteX-WR-NIC.
//
// Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
// SPDX-License-Identifier: BSD-2-Clause

// Full-width RTL regression with independent clock/command and MMCM stimulus.
// The completion model deliberately includes responses outside UG472's normal
// 12-cycle contract to test backpressure, timeout and reset recovery.
`timescale 1ps/1ps

module wr_mmcm_tb;
    reg sys_clk = 0, wr_clk = 0, ps_clk = 0;
    reg sys_rst = 1, wr_rst = 1, ps_rst = 1;
    reg run_wr = 1, run_ps = 1;
    integer wr_half_period = 8000;
    reg [15:0] command_data = 32768;
    reg command_load = 0, psdone = 0;
    wire psen, psincdec, busy, fault, accepted_load;
    wire [15:0] accepted_data;
    wire [31:0] steps, status_steps, superseded;
    wire [1:0] status;

    wr_mmcm_dut dut (
        .sys_clk(sys_clk), .sys_rst(sys_rst),
        .wr_clk(wr_clk),   .wr_rst(wr_rst),
        .ps_clk(ps_clk),   .ps_rst(ps_rst),
        .command_data(command_data), .command_load(command_load),
        .psen(psen), .psincdec(psincdec), .psdone(psdone),
        .busy(busy), .fault(fault), .steps(steps),
        .accepted_load(accepted_load), .accepted_data(accepted_data),
        .status(status), .status_steps(status_steps), .superseded(superseded)
    );

    always #4000 sys_clk = ~sys_clk;
    always begin
        #(wr_half_period);
        if (run_wr) wr_clk = ~wr_clk;
    end
    always #2500 if (run_ps) ps_clk = ~ps_clk;

    integer cycles = 0, issued = 0, completed = 0, last_issue = 0;
    integer response_latency = 12, response_width = 1, response_due = -1000;
    integer active_code = 32768;
    reg resetting = 1, pending = 0, active_direction = 0, previous_psen = 0;
    reg force_done = 0, recording = 0, expected_direction = 0;
    longint unsigned integral = 0;
    integer measured_steps = 0;

    // Protocol scoreboard: no access to the accumulator/timer implementation.
    always @(posedge ps_clk) begin
        cycles = cycles + 1;
        if (resetting) begin
            pending = 0;
            previous_psen = 0;
            issued = 0;
            completed = 0;
            response_due = -1000;
            active_code = 32768;
        end else begin
            if (accepted_load) active_code = accepted_data;
            if (recording)
                integral = integral + ((active_code < 32768) ?
                    32768 - active_code : active_code - 32768);
            if (psen && previous_psen) $fatal(1, "PSEN wider than one cycle");
            if (pending && psincdec != active_direction)
                $fatal(1, "Direction changed during an outstanding shift");
            if (psen) begin
                if (pending || psdone || fault) $fatal(1, "Unsafe phase request");
                pending = 1;
                active_direction = psincdec;
                last_issue = cycles;
                issued = issued + 1;
                if (response_latency != 0) response_due = cycles + response_latency;
                if (recording) begin
                    if (psincdec != expected_direction) $fatal(1, "Wrong tuning polarity");
                    measured_steps = measured_steps + 1;
                end
            end
            if (pending && psdone) begin
                pending = 0;
                completed = completed + 1;
            end
            previous_psen = psen;
        end
    end

    // Drive away from the DUT sampling edge, avoiding testbench/DUT races.
    always @(negedge ps_clk)
        psdone = !resetting && (force_done ||
            (cycles >= response_due - 1 && cycles < response_due - 1 + response_width));

    task command(input integer code);
        @(negedge wr_clk);
        command_data = code;
        command_load = 1;
        @(negedge wr_clk);
        command_load = 0;
    endtask

    task reset_backend(input integer domain);
        resetting = 1;
        @(negedge ps_clk);
        if (domain == 0) wr_rst = 1;
        else ps_rst = 1;
        repeat (20) @(negedge ps_clk);
        wr_rst = 0;
        ps_rst = 0;
        repeat (20) @(negedge ps_clk);
        if (psen || busy || fault || steps) $fatal(1, "Reset failed");
        resetting = 0;
    endtask

    task drain;
        command(32768);
        repeat (200) @(negedge ps_clk);
        if (psen || busy || issued != completed || steps != completed)
            $fatal(1, "Completion accounting did not converge");
        if (status != 0 || status_steps != steps)
            $fatal(1, "CSR status did not converge");
    endtask

    task start_measurement(input integer direction);
        @(negedge ps_clk);
        integral = 0;
        measured_steps = 0;
        expected_direction = direction;
        recording = 1;
    endtask

    task finish_measurement;
        real expected;
        @(negedge ps_clk);
        recording = 0;
        expected = real'(integral)/524288.0;
        if (measured_steps < expected - 1.01 || measured_steps > expected + 1.01)
            $fatal(1, "Lost fractional phase: actual=%0d expected=%0f", measured_steps, expected);
        $display("MMCM rate: code=%0d actual=%0d expected=%0f", active_code, measured_steps, expected);
    endtask

    task check_rate(input integer code, input integer duration);
        reg refreshing;
        command(code);
        repeat (200) @(negedge ps_clk);
        refreshing = 1;
        fork
            begin
                while (refreshing) begin
                    command(code);
                    repeat (7) @(negedge wr_clk);
                end
            end
            begin
                start_measurement(code < 32768);
                repeat (duration) @(negedge ps_clk);
                finish_measurement();
                refreshing = 0;
            end
        join
        drain();
        if (fault) $fatal(1, "Unexpected rate-test timeout");
    endtask

    reg [31:0] random_state = 32'h13a5c079;
    function integer random_code;
        // Fixed seed, reproducible across simulators.
        random_state = random_state ^ (random_state << 13);
        random_state = random_state ^ (random_state >> 17);
        random_state = random_state ^ (random_state << 5);
        random_code = random_state[14:0];
    endfunction

    integer index, code, before_steps;
    initial begin
        #200000;
        sys_rst = 0;
        reset_backend(0);
        repeat (200) @(negedge ps_clk);
        if (psen || steps) $fatal(1, "Backend did not start neutral");

        // Real 16-bit arithmetic: endpoints and both one-LSB corrections.
        check_rate(0,     8192);
        check_rate(1,     8192);
        check_rate(16384, 8192);
        check_rate(32767, 2097152);
        check_rate(32768, 8192);
        check_rate(32769, 2097152);
        check_rate(49152, 8192);
        check_rate(65535, 8192);

        // Integrate a varying command stream in each direction. The expected
        // rate comes from the accepted code/time integral, not DUT state.
        // Also exercise a faster-than-PSCLK source and FIFO replacement.
        wr_half_period = 1700;
        for (integer direction = 0; direction < 2; direction = direction + 1) begin
            command(direction ? 16384 : 49152);
            repeat (100) @(negedge ps_clk);
            start_measurement(direction);
            for (index = 0; index < 1000; index = index + 1) begin
                code = random_code();
                command(direction ? code : 32769 + code%32767);
                repeat (index%7) @(negedge wr_clk);
            end
            finish_measurement();
            drain();
        end
        wr_half_period = 8000;

        // Slow and stretched completions limit the physical request rate.
        response_latency = 40;
        command(0);
        repeat (1000) @(negedge ps_clk);
        command(65535); // Reverse with a shift outstanding.
        repeat (1000) @(negedge ps_clk);
        drain();
        response_latency = 12;
        response_width = 20;
        command(0);
        repeat (1000) @(negedge ps_clk);
        drain();
        response_width = 1;

        // A stale high PSDONE must not acknowledge a new request.
        force_done = 1;
        before_steps = issued;
        command(0);
        repeat (200) @(negedge ps_clk);
        if (issued != before_steps) $fatal(1, "Request issued while PSDONE high");
        force_done = 0;
        repeat (200) @(negedge ps_clk);
        if (issued == before_steps) $fatal(1, "Requests did not resume");
        drain();

        // Completion on the timeout boundary takes priority; a completion
        // one cycle later is counted but must not clear the sticky fault.
        response_latency = 63;
        command(0);
        repeat (500) @(negedge ps_clk);
        drain();
        if (fault) $fatal(1, "Deadline completion lost to timeout");
        response_latency = 64;
        command(0);
        wait (fault);
        repeat (200) @(negedge ps_clk);
        if (busy || issued != completed) $fatal(1, "Late completion not counted");
        before_steps = issued;
        command(65535);
        repeat (200) @(negedge ps_clk);
        if (!fault || issued != before_steps) $fatal(1, "Fault was not sticky");
        reset_backend(0);

        response_latency = 0;
        command(0);
        wait (fault);
        repeat (100) @(negedge ps_clk);
        if (!busy || steps || issued != 1) $fatal(1, "Missing-completion behavior");
        reset_backend(1);
        response_latency = 12;
        check_rate(16384, 4096);

        // A paused PSCLK pauses both the MMCM transaction and the watchdog.
        command(0);
        wait (psen);
        @(negedge ps_clk);
        run_ps = 0;
        repeat (300) @(negedge wr_clk);
        if (fault) $fatal(1, "Stopped PSCLK advanced the watchdog");
        run_ps = 1;
        repeat (200) @(negedge ps_clk);
        if (fault) $fatal(1, "Clock restart failed to complete the pending shift");
        drain();

        // Stop PSCLK while a request is active, overflow the command FIFO,
        // then reset from WR while the destination clock remains stopped.
        command(0);
        wait (psen);
        @(negedge ps_clk);
        run_ps = 0;
        for (index = 0; index < 40; index = index + 1) command(65535 - index);
        if (superseded == 0) $fatal(1, "FIFO overflow scenario did not replace commands");
        resetting = 1;
        wr_rst = 1;
        repeat (20) @(negedge wr_clk);
        wr_rst = 0;
        repeat (20) @(negedge wr_clk);
        run_ps = 1;
        repeat (200) @(negedge ps_clk);
        resetting = 0;
        repeat (200) @(negedge ps_clk);
        if (accepted_load || psen || busy || fault || steps)
            $fatal(1, "Pre-reset command replayed after PSCLK restart");
        check_rate(49152, 4096);

        // Reset from PSCLK with the command source stopped, then restart it.
        @(negedge wr_clk);
        run_wr = 0;
        reset_backend(1);
        run_wr = 1;
        repeat (200) @(negedge ps_clk);
        if (accepted_load || psen || busy || fault || steps)
            $fatal(1, "Pre-reset command replayed after WR clock restart: load=%0d psen=%0d busy=%0d fault=%0d steps=%0d code=%0d",
                accepted_load, psen, busy, fault, steps, active_code);
        check_rate(16384, 4096);
        $display("PASS: MMCM full-width rate, updates, CDC, completion, fault and reset");
        $finish;
    end

    initial begin
        #30000000000.0;
        $fatal(1, "MMCM backend watchdog");
    end
endmodule
