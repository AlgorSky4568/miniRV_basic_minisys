`timescale 1ns / 1ps

module digled_wrap(
    input  wire         aclk,
    input  wire         aresetn,

    // AXI4 Slave — Write Address
    input  wire [31:0]  s_axi_awaddr,
    input  wire [ 7:0]  s_axi_awlen,
    input  wire [ 2:0]  s_axi_awsize,
    input  wire [ 1:0]  s_axi_awburst,
    input  wire         s_axi_awvalid,
    output wire         s_axi_awready,
    // AXI4 Slave — Write Data
    input  wire [31:0]  s_axi_wdata,
    input  wire [ 3:0]  s_axi_wstrb,
    input  wire         s_axi_wlast,
    input  wire         s_axi_wvalid,
    output wire         s_axi_wready,
    // AXI4 Slave — Write Response
    output wire [ 1:0]  s_axi_bresp,
    output wire         s_axi_bvalid,
    input  wire         s_axi_bready,
    // AXI4 Slave — Read Address
    input  wire [31:0]  s_axi_araddr,
    input  wire [ 7:0]  s_axi_arlen,
    input  wire [ 2:0]  s_axi_arsize,
    input  wire [ 1:0]  s_axi_arburst,
    input  wire         s_axi_arvalid,
    output wire         s_axi_arready,
    // AXI4 Slave — Read Data
    output wire [31:0]  s_axi_rdata,
    output wire [ 1:0]  s_axi_rresp,
    output wire         s_axi_rlast,
    output wire         s_axi_rvalid,
    input  wire         s_axi_rready,

    // DigLED pins (active low on Minisys)
    output wire [ 7:0]  dig_en,
    output wire [ 7:0]  dig_seg
);

    // ── Protocol Converter ──
    wire [31:0]  m_axi_awaddr;
    wire [ 2:0]  m_axi_awprot;
    wire         m_axi_awvalid;
    wire         m_axi_awready;
    wire [31:0]  m_axi_wdata;
    wire [ 3:0]  m_axi_wstrb;
    wire         m_axi_wvalid;
    wire         m_axi_wready;
    wire [ 1:0]  m_axi_bresp;
    wire         m_axi_bvalid;
    wire         m_axi_bready;
    wire [31:0]  m_axi_araddr;
    wire [ 2:0]  m_axi_arprot;
    wire         m_axi_arvalid;
    wire         m_axi_arready;
    wire [31:0]  m_axi_rdata;
    wire [ 1:0]  m_axi_rresp;
    wire         m_axi_rvalid;
    wire         m_axi_rready;

    axi_protocol_converter_0 U_conv (
        .aclk           (aclk),
        .aresetn        (aresetn),
        .s_axi_awaddr   (s_axi_awaddr),
        .s_axi_awlen    (s_axi_awlen),
        .s_axi_awsize   (s_axi_awsize),
        .s_axi_awburst  (s_axi_awburst),
        .s_axi_awlock   (1'b0),
        .s_axi_awcache  (4'h0),
        .s_axi_awprot   (3'h0),
        .s_axi_awregion (4'h0),
        .s_axi_awqos    (4'h0),
        .s_axi_awvalid  (s_axi_awvalid),
        .s_axi_awready  (s_axi_awready),
        .s_axi_wdata    (s_axi_wdata),
        .s_axi_wstrb    (s_axi_wstrb),
        .s_axi_wlast    (s_axi_wlast),
        .s_axi_wvalid   (s_axi_wvalid),
        .s_axi_wready   (s_axi_wready),
        .s_axi_bresp    (s_axi_bresp),
        .s_axi_bvalid   (s_axi_bvalid),
        .s_axi_bready   (s_axi_bready),
        .s_axi_araddr   (s_axi_araddr),
        .s_axi_arlen    (s_axi_arlen),
        .s_axi_arsize   (s_axi_arsize),
        .s_axi_arburst  (s_axi_arburst),
        .s_axi_arlock   (1'b0),
        .s_axi_arcache  (4'h0),
        .s_axi_arprot   (3'h0),
        .s_axi_arregion (4'h0),
        .s_axi_arqos    (4'h0),
        .s_axi_arvalid  (s_axi_arvalid),
        .s_axi_arready  (s_axi_arready),
        .s_axi_rdata    (s_axi_rdata),
        .s_axi_rresp    (s_axi_rresp),
        .s_axi_rlast    (s_axi_rlast),
        .s_axi_rvalid   (s_axi_rvalid),
        .s_axi_rready   (s_axi_rready),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awprot   (m_axi_awprot),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_bresp    (m_axi_bresp),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        .m_axi_araddr   (m_axi_araddr),
        .m_axi_arprot   (m_axi_arprot),
        .m_axi_arvalid  (m_axi_arvalid),
        .m_axi_arready  (m_axi_arready),
        .m_axi_rdata    (m_axi_rdata),
        .m_axi_rresp    (m_axi_rresp),
        .m_axi_rvalid   (m_axi_rvalid),
        .m_axi_rready   (m_axi_rready)
    );

    // ── AXI GPIO (All Outputs) ──
    wire [31:0] gpio_out;

    axi_gpio_dig U_gpio (
        .s_axi_aclk     (aclk),
        .s_axi_aresetn  (aresetn),
        .s_axi_awaddr   (m_axi_awaddr[8:0]),
        .s_axi_awvalid  (m_axi_awvalid),
        .s_axi_awready  (m_axi_awready),
        .s_axi_wdata    (m_axi_wdata),
        .s_axi_wstrb    (m_axi_wstrb),
        .s_axi_wvalid   (m_axi_wvalid),
        .s_axi_wready   (m_axi_wready),
        .s_axi_bresp    (m_axi_bresp),
        .s_axi_bvalid   (m_axi_bvalid),
        .s_axi_bready   (m_axi_bready),
        .s_axi_araddr   (m_axi_araddr[8:0]),
        .s_axi_arvalid  (m_axi_arvalid),
        .s_axi_arready  (m_axi_arready),
        .s_axi_rdata    (m_axi_rdata),
        .s_axi_rresp    (m_axi_rresp),
        .s_axi_rvalid   (m_axi_rvalid),
        .s_axi_rready   (m_axi_rready),
        .gpio_io_o      (gpio_out)
    );

    // ── 动态扫描控制器 ──
    // gpio_out[31:0] = 8 个 hex 位 (每4bit一个数码管)
    //   [31:28]=Dig7 ... [3:0]=Dig0
    // 每个数码管刷新 ~1ms, 8个数码管完整周期 ~8ms → 125Hz, 无闪烁

    // 扫描计数器: 0~49999 约 1ms@50MHz
    reg [15:0] scan_cnt;
    reg [2:0]  digit_sel;  // 当前扫描位 (0~7)
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            scan_cnt  <= 16'd0;
            digit_sel <= 3'd0;
        end else if (scan_cnt == 16'd49999) begin
            scan_cnt  <= 16'd0;
            digit_sel <= digit_sel + 3'd1;
        end else begin
            scan_cnt <= scan_cnt + 16'd1;
        end
    end

    // 取出当前位数码管对应的 4-bit hex 值
    wire [3:0] cur_hex = gpio_out[{digit_sel, 2'b0} +: 4];

    // 七段码译码器: 4-bit hex → 8-bit 段选 (共阳极, 低电平有效)
    // Minisys: seg[7:0] = {CA, CB, CC, CD, CE, CF, CG, DP}
    reg [7:0] seg_pattern;
    always @(*) begin
        case (cur_hex)
            4'h0: seg_pattern = 8'h03;
            4'h1: seg_pattern = 8'h9F;
            4'h2: seg_pattern = 8'h25;
            4'h3: seg_pattern = 8'h0D;
            4'h4: seg_pattern = 8'h99;
            4'h5: seg_pattern = 8'h49;
            4'h6: seg_pattern = 8'h41;
            4'h7: seg_pattern = 8'h1F;
            4'h8: seg_pattern = 8'h01;
            4'h9: seg_pattern = 8'h09;
            4'hA: seg_pattern = 8'h11;
            4'hB: seg_pattern = 8'hC1;
            4'hC: seg_pattern = 8'h63;
            4'hD: seg_pattern = 8'h85;
            4'hE: seg_pattern = 8'h61;
            4'hF: seg_pattern = 8'h71;
        endcase
    end

    // 输出: dig_en one-hot 低有效, dig_seg 低有效
    assign dig_en  = ~(8'h01 << digit_sel);
    assign dig_seg = seg_pattern;

endmodule
