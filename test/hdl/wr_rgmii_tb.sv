// This file is part of LiteX-WR-NIC.
// Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
// SPDX-License-Identifier: BSD-2-Clause

`timescale 1ns/1ps
module phy_tb;
  reg sys_clk = 0, sys_rst = 1;
  always #4 sys_clk = ~sys_clk;
  localparam real PEER_DELAY = 0.0;
  reg rx_clock_source = 0, rx_clock_enable = 0;
  always #4 if (rx_clock_enable) rx_clock_source = ~rx_clock_source;
  wire rx_clock_in;
  assign #(PEER_DELAY) rx_clock_in = rx_clock_source;
  reg [3:0] rx_data_in = 0;
  reg rx_ctl_in = 0;
  wire tx_clock_out, tx_ctl_out;
  wire [3:0] tx_data_out;
  reg [7:0] tx_byte = 0;
  reg tx_valid = 0, tx_error = 0;
  wire [7:0] rx_byte;
  wire rx_valid, rx_last, rx_error, ready, tx_domain_clock, rx_domain_clock;
  phy_dut dut(.*);
  integer rx_count = 0, rx_frames = 0, tx_count = 0, tx_edges = 0;
  always @(posedge tx_clock_out) tx_edges = tx_edges + 1;
  reg [3:0] tx_low;
  reg tx_en_rise;
  integer expected;

  always @(posedge rx_domain_clock) begin
    #0.1;
    if (ready && rx_valid) begin
      expected = ((rx_count * 13) + 7) & 255;
      if (rx_byte !== expected[7:0])
        $fatal(1, "RX byte %0d: %02x expected %02x", rx_count, rx_byte, expected);
      if (rx_error !== ((rx_frames == 1) && rx_count >= 7))
        $fatal(1, "RX error alignment at byte %0d frame %0d", rx_count, rx_frames);
      rx_count = rx_count + 1;
      if (rx_last) begin
        if (rx_count != 32) $fatal(1, "RX frame length %0d", rx_count);
        rx_count = 0;
        rx_frames = rx_frames + 1;
      end
    end
  end

  always @(posedge tx_clock_out) begin
    #(PEER_DELAY + 0.1);
    tx_low = tx_data_out;
    tx_en_rise = tx_ctl_out;
  end
  always @(negedge tx_clock_out) begin
    #(PEER_DELAY + 0.1);
    if (ready && tx_en_rise) begin
      expected = ((tx_count * 17) + 3) & 255;
      if ({tx_data_out, tx_low} !== expected[7:0])
        $fatal(1, "TX byte %0d: %02x expected %02x", tx_count, {tx_data_out, tx_low}, expected);
      if ((tx_ctl_out ^ tx_en_rise) !== (tx_count == 3))
        $fatal(1, "TX error alignment at byte %0d", tx_count);
      tx_count = tx_count + 1;
    end
  end

  task send_rx(input integer bad_frame);
    integer i, value;
    begin
      for (i=0; i<32; i=i+1) begin
        value = ((i * 13) + 7) & 255;
        @(posedge rx_clock_source); #0.1;
        rx_data_in = value[3:0]; rx_ctl_in = 1;
        @(negedge rx_clock_source); #0.1;
        rx_data_in = value[7:4]; rx_ctl_in = !(bad_frame && i == 7);
      end
      @(posedge rx_clock_source); #0.1;
      rx_ctl_in = 0; rx_data_in = 0;
      repeat (16) @(posedge rx_clock_source);
    end
  endtask

  initial begin
    #160; sys_rst = 0;
    #4000;
    if (ready) $fatal(1, "Absent peer clock was not detected at startup");
    if (tx_edges < 100) $fatal(1, "TX clock needs peer clock at startup");
    rx_clock_enable = 1;
    wait(ready); repeat (20) @(posedge sys_clk);
    fork
      begin
        send_rx(0); send_rx(1);
      end
      begin : transmit
        integer i;
        for (i=0; i<20; i=i+1) begin
          @(negedge tx_domain_clock);
          tx_byte = ((i * 17) + 3) & 255;
          tx_valid = 1;
          tx_error = (i == 3);
        end
        @(negedge tx_domain_clock); tx_valid = 0; tx_error = 0;
      end
    join
    repeat (40) @(posedge sys_clk);
    if (rx_frames != 2 || tx_count != 20)
      $fatal(1, "Missing frames: RX %0d TX bytes %0d", rx_frames, tx_count);
    begin : clock_recovery
      integer previous_edges;
      previous_edges = tx_edges;
      rx_clock_enable = 0;
      #4000;
      if (ready) $fatal(1, "Missing RX clock was not detected");
      if (tx_edges < previous_edges + 100) $fatal(1, "TX clock depends on peer RX clock");
      rx_clock_enable = 1;
      wait(ready); repeat (20) @(posedge sys_clk);
      send_rx(0);
      repeat (40) @(posedge sys_clk);
      if (rx_frames != 3) $fatal(1, "RX did not recover after clock restart");
    end
    $display("RGMII PHY PASS");
    $finish;
  end
  initial begin
    #2000000; $fatal(1, "PHY test timeout");
  end
endmodule
