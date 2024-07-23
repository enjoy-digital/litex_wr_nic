module wrf_snk_test (
    input  wire       wrf_clk,
    input  wire       wrf_send,
    output reg        wrf_valid,
    output wire       wrf_last,
    input  wire       wrf_ready,
    output reg [15:0] wrf_data
);

wire [47:0] MAC_ADDR = 48'h74563c4f4c6d; // Updated MAC address: 74:56:3c:4f:4c:6d
wire [15:0] ipv4_w8 = 16'hc0a8; // Destination IP 192.168
wire [15:0] ipv4_w9 = 16'h017A; // Destination IP .1.122

// constants for headers
wire[15:0] wrf_snk_status = 16'h0200; // last 4 bits 0 - isHP (high priority)
// 1 - err (contains error)
// 2 - vSMAC (valid source MAC else use from WR core)
// 3 - vCRC (valid check sum)
wire [15:0] EtherType = 16'h0800; // IPv4
wire [15:0] ipv4_w0 = 16'h4500; // IPV4 header: version, header length, DSCP, ECN
wire [15:0] ipv4_w1 = 16'd236; // length total transmission to WR: 256bytes = 128 words.
// 2 bytes WR status
// 18 bytes Eth frame
// 20 bytes IPv4 header
// => IPv4 total length = 256-20 = 236 bytes
wire [15:0] ipv4_w2 = 16'h0000; // identification
wire [15:0] ipv4_w3 = 16'h0000; // flags, fragment offset
wire [15:0] ipv4_w4 = 16'h3F11; // time to live (63), protocol (11-UDP):
wire [15:0] ipv4_w5 = 16'hF79A; // checksum per http://www.n-cg.net/hec.htm
wire [15:0] ipv4_w6 = 16'hc0a8; // source IP 192.168
wire [15:0] ipv4_w7 = 16'h0105; // source IP .1.5
//wire [15:0] ipv4_w8 = 16'hc0a8; // dest IP 192.168
//wire [15:0] ipv4_w9 = 16'h0111; // dest IP .1.17
wire [15:0] udp_w0 = 16'h1000; // UDP header source port
wire [15:0] udp_w1 = 16'h1000; // dest port
wire [15:0] udp_w2 = 16'd216; // length (header+data) = 216
wire [15:0] udp_w3 = 16'h0000; // checksum (0=disable)
// counter for sending data
reg [6:0] blkcntr;
wire cntron;
always @(posedge wrf_clk)
if(wrf_send)
    blkcntr <= 126;

else if(cntron & wrf_ready) // count down unless stalled
blkcntr <= blkcntr - 1;
assign cntron = |blkcntr;
// data assembly
always @(*)
case (blkcntr)
7'd126 : wrf_data <= MAC_ADDR[47:32];
7'd125 : wrf_data <= MAC_ADDR[31:16];
7'd124 : wrf_data <= MAC_ADDR[15:00];
7'd123 : wrf_data <= 0; // source mac insert by WR core
7'd122 : wrf_data <= 0;
7'd121 : wrf_data <= 0;
7'd120 : wrf_data <= EtherType;
7'd119 : wrf_data <= ipv4_w0;
7'd118 : wrf_data <= ipv4_w1;
7'd117 : wrf_data <= ipv4_w2;
7'd116 : wrf_data <= ipv4_w3;
7'd115 : wrf_data <= ipv4_w4;
7'd114 : wrf_data <= ipv4_w5;
7'd113 : wrf_data <= ipv4_w6;
7'd112 : wrf_data <= ipv4_w7;
7'd111 : wrf_data <= ipv4_w8;
7'd110 : wrf_data <= ipv4_w9;
7'd109 : wrf_data <= udp_w0;
7'd108 : wrf_data <= udp_w1;
7'd107 : wrf_data <= udp_w2;
7'd106 : wrf_data <= udp_w3;
default: wrf_data <= 16'h1234; // payload data ... tie to fifo from pulse processing
endcase
always @(posedge wrf_clk)
if(blkcntr>7'd0)
    wrf_valid <=1'b1;
else if (wrf_ready==1)
    wrf_valid <=1'b0;

assign wrf_last = (blkcntr == 0);

endmodule