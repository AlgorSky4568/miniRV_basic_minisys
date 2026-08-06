`timescale 1ns / 1ps

`include "defines.vh"

module miniRV_SoC(
    input  wire         fpga_clk,
    input  wire         fpga_rst,
    input  wire [23:0]  sw,
    output wire [23:0]  led,
    (*mark_debug = "true"*) output wire [ 7:0]  dig_en,
    (*mark_debug = "true"*) output wire [ 7:0]  dig_seg,
    input  wire         rx,
    (*mark_debug = "true"*) output wire         tx
);

    // ================================================================
    // Clock Selection
    // ================================================================
`ifdef RUN_TRACE
    (*mark_debug = "true"*) wire sys_clk = fpga_clk;
    (*mark_debug = "true"*) wire sys_rst = fpga_rst;
`else
    wire pll_clk1;
    (*mark_debug = "true"*) wire pll_lock;
    // 时序优化（Bug 22）：去除 LUT 门控时钟（原 sys_clk = pll_lock & pll_clk1，
    // 时钟经 LUT+布线产生 → skew 大）。改时钟直连 BUFG，PLL lock 并入复位
    // 保持（sys_clk 域两级同步），语义等价：PLL 未锁定时保持复位。
    (*mark_debug = "true"*) wire sys_clk = pll_clk1;
    reg rst_r1, rst_r2;
    always @(posedge sys_clk) begin
        rst_r1 <= fpga_rst | !pll_lock;
        rst_r2 <= rst_r1;
    end
    (*mark_debug = "true"*) wire sys_rst = rst_r2;

    clk_wiz_0 U_clkgen (
        .clk_in1    (fpga_clk),
        .locked     (pll_lock),
        .clk_out1   (pll_clk1)
    );
`endif

    // ================================================================
    // CPU �? AXI Bus Wires
    // ================================================================
    (*mark_debug = "true"*) wire [31:0] cpu_awaddr ;
    wire [ 7:0] cpu_awlen  ;
    wire [ 2:0] cpu_awsize ;
    wire [ 1:0] cpu_awburst;
    (*mark_debug = "true"*) wire        cpu_awvalid;
    (*mark_debug = "true"*) wire        cpu_awready;
    wire [31:0] cpu_wdata  ;
    wire [ 3:0] cpu_wstrb  ;
    wire        cpu_wlast  ;
    wire        cpu_wvalid ;
    wire        cpu_wready ;
    wire        cpu_bready ;
    wire [ 1:0] cpu_bresp  ;
    wire        cpu_bvalid ;
    wire [31:0] cpu_araddr ;
    wire [ 7:0] cpu_arlen  ;
    wire [ 2:0] cpu_arsize ;
    wire [ 1:0] cpu_arburst;
    wire        cpu_arvalid;
    wire        cpu_arready;
    wire        cpu_rready ;
    wire [31:0] cpu_rdata  ;
    wire [ 1:0] cpu_rresp  ;
    wire        cpu_rlast  ;
    wire        cpu_rvalid ;

    cpu_top U_cpu (
        .cpu_clk        (sys_clk),
        .cpu_rst        (sys_rst),
        .m_axi_awaddr   (cpu_awaddr),
        .m_axi_awlen    (cpu_awlen),
        .m_axi_awsize   (cpu_awsize),
        .m_axi_awburst  (cpu_awburst),
        .m_axi_awvalid  (cpu_awvalid),
        .m_axi_awready  (cpu_awready),
        .m_axi_wdata    (cpu_wdata),
        .m_axi_wstrb    (cpu_wstrb),
        .m_axi_wlast    (cpu_wlast),
        .m_axi_wvalid   (cpu_wvalid),
        .m_axi_wready   (cpu_wready),
        .m_axi_bready   (cpu_bready),
        .m_axi_bresp    (cpu_bresp),
        .m_axi_bvalid   (cpu_bvalid),
        .m_axi_araddr   (cpu_araddr),
        .m_axi_arlen    (cpu_arlen),
        .m_axi_arsize   (cpu_arsize),
        .m_axi_arburst  (cpu_arburst),
        .m_axi_arvalid  (cpu_arvalid),
        .m_axi_arready  (cpu_arready),
        .m_axi_rready   (cpu_rready),
        .m_axi_rdata    (cpu_rdata),
        .m_axi_rresp    (cpu_rresp),
        .m_axi_rlast    (cpu_rlast),
        .m_axi_rvalid   (cpu_rvalid)
    );

    // ================================================================
    // AXI Bus — bram_axi (Main Memory) + 5 Peripherals
    // ================================================================
    // Trace mode: CPU ↔ bram_axi 直连 (Verilator 不支持 IP 核仿真)
    // Board mode: CPU → AXI Crossbar → bram_axi + 5 外设

    // ── BRAM wires ──
    wire [31:0] bram_awaddr ;
    wire [ 7:0] bram_awlen  ;
    wire [ 2:0] bram_awsize ;
    wire [ 1:0] bram_awburst;
    wire        bram_awvalid;
    wire        bram_awready;
    wire [31:0] bram_wdata  ;
    wire [ 3:0] bram_wstrb  ;
    wire        bram_wlast  ;
    wire        bram_wvalid ;
    wire        bram_wready ;
    wire        bram_bready ;
    wire [ 1:0] bram_bresp  ;
    wire        bram_bvalid ;
    wire [31:0] bram_araddr ;
    wire [ 7:0] bram_arlen  ;
    wire [ 2:0] bram_arsize ;
    wire [ 1:0] bram_arburst;
    wire        bram_arvalid;
    wire        bram_arready;
    wire        bram_rready ;
    wire [31:0] bram_rdata  ;
    wire [ 1:0] bram_rresp  ;
    wire        bram_rlast  ;
    wire        bram_rvalid ;

    // ── Switch wires ──
    (*mark_debug = "true"*) wire [31:0] sw_awaddr ;
    wire [ 7:0] sw_awlen  ;
    wire [ 2:0] sw_awsize ;
    wire [ 1:0] sw_awburst;
    wire        sw_awvalid;
    wire        sw_awready;
    wire [31:0] sw_wdata  ;
    wire [ 3:0] sw_wstrb  ;
    wire        sw_wlast  ;
    wire        sw_wvalid ;
    wire        sw_wready ;
    wire        sw_bready ;
    wire [ 1:0] sw_bresp  ;
    wire        sw_bvalid ;
    wire [31:0] sw_araddr ;
    wire [ 7:0] sw_arlen  ;
    wire [ 2:0] sw_arsize ;
    wire [ 1:0] sw_arburst;
    wire        sw_arvalid;
    wire        sw_arready;
    wire        sw_rready ;
    wire [31:0] sw_rdata  ;
    wire [ 1:0] sw_rresp  ;
    wire        sw_rlast  ;
    wire        sw_rvalid ;

    // ── LED wires ──
    wire [31:0] led_awaddr ;
    wire [ 7:0] led_awlen  ;
    wire [ 2:0] led_awsize ;
    wire [ 1:0] led_awburst;
    wire        led_awvalid;
    wire        led_awready;
    wire [31:0] led_wdata  ;
    wire [ 3:0] led_wstrb  ;
    wire        led_wlast  ;
    wire        led_wvalid ;
    wire        led_wready ;
    wire        led_bready ;
    wire [ 1:0] led_bresp  ;
    wire        led_bvalid ;
    wire [31:0] led_araddr ;
    wire [ 7:0] led_arlen  ;
    wire [ 2:0] led_arsize ;
    wire [ 1:0] led_arburst;
    wire        led_arvalid;
    wire        led_arready;
    wire        led_rready ;
    wire [31:0] led_rdata  ;
    wire [ 1:0] led_rresp  ;
    wire        led_rlast  ;
    wire        led_rvalid ;

    // ── DigLED wires ──
    wire [31:0] digled_awaddr ;
    wire [ 7:0] digled_awlen  ;
    wire [ 2:0] digled_awsize ;
    wire [ 1:0] digled_awburst;
    wire        digled_awvalid;
    wire        digled_awready;
    wire [31:0] digled_wdata  ;
    wire [ 3:0] digled_wstrb  ;
    wire        digled_wlast  ;
    wire        digled_wvalid ;
    wire        digled_wready ;
    wire        digled_bready ;
    wire [ 1:0] digled_bresp  ;
    wire        digled_bvalid ;
    wire [31:0] digled_araddr ;
    wire [ 7:0] digled_arlen  ;
    wire [ 2:0] digled_arsize ;
    wire [ 1:0] digled_arburst;
    wire        digled_arvalid;
    wire        digled_arready;
    wire        digled_rready ;
    wire [31:0] digled_rdata  ;
    wire [ 1:0] digled_rresp  ;
    wire        digled_rlast  ;
    wire        digled_rvalid ;

    // ── UART wires ──
    wire [31:0] uart_awaddr ;
    wire [ 7:0] uart_awlen  ;
    wire [ 2:0] uart_awsize ;
    wire [ 1:0] uart_awburst;
    wire        uart_awvalid;
    wire        uart_awready;
    wire [31:0] uart_wdata  ;
    wire [ 3:0] uart_wstrb  ;
    wire        uart_wlast  ;
    wire        uart_wvalid ;
    wire        uart_wready ;
    wire        uart_bready ;
    wire [ 1:0] uart_bresp  ;
    wire        uart_bvalid ;
    wire [31:0] uart_araddr ;
    wire [ 7:0] uart_arlen  ;
    wire [ 2:0] uart_arsize ;
    wire [ 1:0] uart_arburst;
    wire        uart_arvalid;
    wire        uart_arready;
    wire        uart_rready ;
    wire [31:0] uart_rdata  ;
    wire [ 1:0] uart_rresp  ;
    wire        uart_rlast  ;
    wire        uart_rvalid ;

    // ── Timer wires ──
    wire [31:0] tim_awaddr ;
    wire [ 7:0] tim_awlen  ;
    wire [ 2:0] tim_awsize ;
    wire [ 1:0] tim_awburst;
    wire        tim_awvalid;
    wire        tim_awready;
    wire [31:0] tim_wdata  ;
    wire [ 3:0] tim_wstrb  ;
    wire        tim_wlast  ;
    wire        tim_wvalid ;
    wire        tim_wready ;
    wire        tim_bready ;
    wire [ 1:0] tim_bresp  ;
    wire        tim_bvalid ;
    wire [31:0] tim_araddr ;
    wire [ 7:0] tim_arlen  ;
    wire [ 2:0] tim_arsize ;
    wire [ 1:0] tim_arburst;
    wire        tim_arvalid;
    wire        tim_arready;
    wire        tim_rready ;
    wire [31:0] tim_rdata  ;
    wire [ 1:0] tim_rresp  ;
    wire        tim_rlast  ;
    wire        tim_rvalid ;

`ifdef RUN_TRACE
    // ================================================================
    // Trace 模式：CPU ↔ bram_axi 直连（绕过 Crossbar）
    // ================================================================
    assign bram_awaddr  = cpu_awaddr ;
    assign bram_awlen   = cpu_awlen  ;
    assign bram_awsize  = cpu_awsize ;
    assign bram_awburst = cpu_awburst;
    assign cpu_awready  = bram_awready;
    assign bram_awvalid = cpu_awvalid;
    assign bram_wdata   = cpu_wdata  ;
    assign bram_wstrb   = cpu_wstrb  ;
    assign bram_wlast   = cpu_wlast  ;
    assign bram_wvalid  = cpu_wvalid ;
    assign cpu_wready   = bram_wready;
    assign cpu_bresp    = bram_bresp ;
    assign cpu_bvalid   = bram_bvalid;
    assign bram_bready  = cpu_bready ;
    assign bram_araddr  = cpu_araddr ;
    assign bram_arlen   = cpu_arlen  ;
    assign bram_arsize  = cpu_arsize ;
    assign bram_arburst = cpu_arburst;
    assign bram_arvalid = cpu_arvalid;
    assign cpu_arready  = bram_arready;
    assign bram_rready  = cpu_rready ;
    assign cpu_rdata    = bram_rdata ;
    assign cpu_rresp    = bram_rresp ;
    assign cpu_rlast    = bram_rlast ;
    assign cpu_rvalid   = bram_rvalid;
`else
    // ================================================================
    // 下板模式：地址解码路由器
    //   0x0000_xxxx → BRAM   0xFFFF_0xxx → sw
    //   0xFFFF_1xxx → LED    0xFFFF_2xxx → digled
    //   0xFFFF_3xxx → UART   0xFFFF_4xxx → timer
    // ================================================================
    wire is_peri_aw = (cpu_awaddr[31:16] == 16'hFFFF);
    wire is_peri_ar = (cpu_araddr[31:16] == 16'hFFFF);
    wire [2:0] ps_aw = cpu_awaddr[14:12];
    wire [2:0] ps_ar = cpu_araddr[14:12];

    reg aw2bram; reg [2:0] aw2peri;
    always @(posedge sys_clk or posedge sys_rst)
        if (sys_rst) begin aw2bram <= 1'b0; aw2peri <= 3'd0; end
        else if (cpu_awvalid && cpu_awready) begin aw2bram <= !is_peri_aw; aw2peri <= ps_aw; end

    // W 通道组合路由: AW+W 同周期时用组合 ps_aw, 否则用寄存 aw2peri
    wire aw_hs = cpu_awvalid && cpu_awready;
    wire w2bram = aw_hs ? !is_peri_aw : aw2bram;
    wire [2:0] w2peri = aw_hs ? ps_aw : aw2peri;

    reg ar2bram; reg [2:0] ar2peri;
    always @(posedge sys_clk or posedge sys_rst)
        if (sys_rst) begin ar2bram <= 1'b0; ar2peri <= 3'd0; end
        else if (cpu_arvalid && cpu_arready) begin ar2bram <= !is_peri_ar; ar2peri <= ps_ar; end

    // R 通道组合路由: AR+R 同周期时用组合 ps_ar, 否则用寄存 ar2peri
    wire ar_hs = cpu_arvalid && cpu_arready;
    wire r2bram = ar_hs ? !is_peri_ar : ar2bram;
    wire [2:0] r2peri = ar_hs ? ps_ar : ar2peri;

    // AW
    assign bram_awaddr=cpu_awaddr; assign bram_awlen=cpu_awlen; assign bram_awsize=cpu_awsize; assign bram_awburst=cpu_awburst; assign bram_awvalid=cpu_awvalid&&!is_peri_aw;
    assign sw_awaddr=cpu_awaddr; assign sw_awlen=cpu_awlen; assign sw_awsize=cpu_awsize; assign sw_awburst=cpu_awburst; assign sw_awvalid=cpu_awvalid&&is_peri_aw&&ps_aw==0;
    assign led_awaddr=cpu_awaddr; assign led_awlen=cpu_awlen; assign led_awsize=cpu_awsize; assign led_awburst=cpu_awburst; assign led_awvalid=cpu_awvalid&&is_peri_aw&&ps_aw==1;
    assign digled_awaddr=cpu_awaddr; assign digled_awlen=cpu_awlen; assign digled_awsize=cpu_awsize; assign digled_awburst=cpu_awburst; assign digled_awvalid=cpu_awvalid&&is_peri_aw&&ps_aw==2;
    assign uart_awaddr=cpu_awaddr; assign uart_awlen=cpu_awlen; assign uart_awsize=cpu_awsize; assign uart_awburst=cpu_awburst; assign uart_awvalid=cpu_awvalid&&is_peri_aw&&ps_aw==3;
    assign tim_awaddr=cpu_awaddr; assign tim_awlen=cpu_awlen; assign tim_awsize=cpu_awsize; assign tim_awburst=cpu_awburst; assign tim_awvalid=cpu_awvalid&&is_peri_aw&&ps_aw==4;
    assign cpu_awready=!is_peri_aw?bram_awready:ps_aw==0?sw_awready:ps_aw==1?led_awready:ps_aw==2?digled_awready:ps_aw==3?uart_awready:ps_aw==4?tim_awready:1'b1;

    // W (用组合 w2bram/w2peri，避免 AW+W 同周期时寄存值滞后)
    assign bram_wdata=cpu_wdata; assign bram_wstrb=cpu_wstrb; assign bram_wlast=cpu_wlast; assign bram_wvalid=cpu_wvalid&&w2bram;
    assign sw_wdata=cpu_wdata; assign sw_wstrb=cpu_wstrb; assign sw_wlast=cpu_wlast; assign sw_wvalid=cpu_wvalid&&!w2bram&&w2peri==0;
    assign led_wdata=cpu_wdata; assign led_wstrb=cpu_wstrb; assign led_wlast=cpu_wlast; assign led_wvalid=cpu_wvalid&&!w2bram&&w2peri==1;
    assign digled_wdata=cpu_wdata; assign digled_wstrb=cpu_wstrb; assign digled_wlast=cpu_wlast; assign digled_wvalid=cpu_wvalid&&!w2bram&&w2peri==2;
    assign uart_wdata=cpu_wdata; assign uart_wstrb=cpu_wstrb; assign uart_wlast=cpu_wlast; assign uart_wvalid=cpu_wvalid&&!w2bram&&w2peri==3;
    assign tim_wdata=cpu_wdata; assign tim_wstrb=cpu_wstrb; assign tim_wlast=cpu_wlast; assign tim_wvalid=cpu_wvalid&&!w2bram&&w2peri==4;
    assign cpu_wready=w2bram?bram_wready:w2peri==0?sw_wready:w2peri==1?led_wready:w2peri==2?digled_wready:w2peri==3?uart_wready:w2peri==4?tim_wready:1'b1;

    // B
    assign bram_bready=cpu_bready&&aw2bram; assign sw_bready=cpu_bready&&!aw2bram&&aw2peri==0; assign led_bready=cpu_bready&&!aw2bram&&aw2peri==1;
    assign digled_bready=cpu_bready&&!aw2bram&&aw2peri==2; assign uart_bready=cpu_bready&&!aw2bram&&aw2peri==3; assign tim_bready=cpu_bready&&!aw2bram&&aw2peri==4;
    assign cpu_bvalid=aw2bram?bram_bvalid:aw2peri==0?sw_bvalid:aw2peri==1?led_bvalid:aw2peri==2?digled_bvalid:aw2peri==3?uart_bvalid:aw2peri==4?tim_bvalid:1'b0;
    assign cpu_bresp=aw2bram?bram_bresp:aw2peri==0?sw_bresp:aw2peri==1?led_bresp:aw2peri==2?digled_bresp:aw2peri==3?uart_bresp:aw2peri==4?tim_bresp:2'b00;

    // AR
    assign bram_araddr=cpu_araddr; assign bram_arlen=cpu_arlen; assign bram_arsize=cpu_arsize; assign bram_arburst=cpu_arburst; assign bram_arvalid=cpu_arvalid&&!is_peri_ar;
    assign sw_araddr=cpu_araddr; assign sw_arlen=cpu_arlen; assign sw_arsize=cpu_arsize; assign sw_arburst=cpu_arburst; assign sw_arvalid=cpu_arvalid&&is_peri_ar&&ps_ar==0;
    assign led_araddr=cpu_araddr; assign led_arlen=cpu_arlen; assign led_arsize=cpu_arsize; assign led_arburst=cpu_arburst; assign led_arvalid=cpu_arvalid&&is_peri_ar&&ps_ar==1;
    assign digled_araddr=cpu_araddr; assign digled_arlen=cpu_arlen; assign digled_arsize=cpu_arsize; assign digled_arburst=cpu_arburst; assign digled_arvalid=cpu_arvalid&&is_peri_ar&&ps_ar==2;
    assign uart_araddr=cpu_araddr; assign uart_arlen=cpu_arlen; assign uart_arsize=cpu_arsize; assign uart_arburst=cpu_arburst; assign uart_arvalid=cpu_arvalid&&is_peri_ar&&ps_ar==3;
    assign tim_araddr=cpu_araddr; assign tim_arlen=cpu_arlen; assign tim_arsize=cpu_arsize; assign tim_arburst=cpu_arburst; assign tim_arvalid=cpu_arvalid&&is_peri_ar&&ps_ar==4;
    assign cpu_arready=!is_peri_ar?bram_arready:ps_ar==0?sw_arready:ps_ar==1?led_arready:ps_ar==2?digled_arready:ps_ar==3?uart_arready:ps_ar==4?tim_arready:1'b1;

    // R (用组合 r2bram/r2peri，避免 AR+R 同周期时寄存值滞后)
    assign bram_rready=cpu_rready&&r2bram; assign sw_rready=cpu_rready&&!r2bram&&r2peri==0; assign led_rready=cpu_rready&&!r2bram&&r2peri==1;
    assign digled_rready=cpu_rready&&!r2bram&&r2peri==2; assign uart_rready=cpu_rready&&!r2bram&&r2peri==3; assign tim_rready=cpu_rready&&!r2bram&&r2peri==4;
    assign cpu_rdata=r2bram?bram_rdata:r2peri==0?sw_rdata:r2peri==1?led_rdata:r2peri==2?digled_rdata:r2peri==3?uart_rdata:r2peri==4?tim_rdata:32'h0;
    assign cpu_rresp=r2bram?bram_rresp:r2peri==0?sw_rresp:r2peri==1?led_rresp:r2peri==2?digled_rresp:r2peri==3?uart_rresp:r2peri==4?tim_rresp:2'b00;
    assign cpu_rlast=r2bram?bram_rlast:r2peri==0?sw_rlast:r2peri==1?led_rlast:r2peri==2?digled_rlast:r2peri==3?uart_rlast:r2peri==4?tim_rlast:1'b1;
    assign cpu_rvalid=r2bram?bram_rvalid:r2peri==0?sw_rvalid:r2peri==1?led_rvalid:r2peri==2?digled_rvalid:r2peri==3?uart_rvalid:r2peri==4?tim_rvalid:1'b0;
`endif

    // ── BRAM (两种模式共用) ──
    bram_axi U_bram (
        .s_aclk         (sys_clk),
        .s_aresetn      (!sys_rst),
        .s_axi_awid     (4'h6),
        .s_axi_awaddr   (bram_awaddr ),
        .s_axi_awlen    (bram_awlen  ),
        .s_axi_awsize   (bram_awsize ),
        .s_axi_awburst  (bram_awburst),
        .s_axi_awready  (bram_awready),
        .s_axi_awvalid  (bram_awvalid),
        .s_axi_wdata    (bram_wdata  ),
        .s_axi_wstrb    (bram_wstrb  ),
        .s_axi_wvalid   (bram_wvalid ),
        .s_axi_wlast    (bram_wlast  ),
        .s_axi_wready   (bram_wready ),
        .s_axi_bid      (),
        .s_axi_bready   (bram_bready ),
        .s_axi_bresp    (bram_bresp  ),
        .s_axi_bvalid   (bram_bvalid ),
        .s_axi_arid     (4'h6),
        .s_axi_araddr   (bram_araddr ),
        .s_axi_arlen    (bram_arlen  ),
        .s_axi_arsize   (bram_arsize ),
        .s_axi_arburst  (bram_arburst),
        .s_axi_arready  (bram_arready),
        .s_axi_arvalid  (bram_arvalid),
        .s_axi_rdata    (bram_rdata  ),
        .s_axi_rvalid   (bram_rvalid ),
        .s_axi_rlast    (bram_rlast  ),
        .s_axi_rid      (),
        .s_axi_rready   (bram_rready ),
        .s_axi_rresp    (bram_rresp  )
    );

`ifndef RUN_TRACE
    // ── 5 外设 (仅下板模式) ──

    switch_wrap U_switch (
        .aclk           (sys_clk),
        .aresetn        (!sys_rst),
        .s_axi_awaddr   (sw_awaddr ),
        .s_axi_awlen    (sw_awlen  ),
        .s_axi_awsize   (sw_awsize ),
        .s_axi_awburst  (sw_awburst),
        .s_axi_awvalid  (sw_awvalid),
        .s_axi_awready  (sw_awready),
        .s_axi_wdata    (sw_wdata  ),
        .s_axi_wstrb    (sw_wstrb  ),
        .s_axi_wlast    (sw_wlast  ),
        .s_axi_wvalid   (sw_wvalid ),
        .s_axi_wready   (sw_wready ),
        .s_axi_bresp    (sw_bresp  ),
        .s_axi_bvalid   (sw_bvalid ),
        .s_axi_bready   (sw_bready ),
        .s_axi_araddr   (sw_araddr ),
        .s_axi_arlen    (sw_arlen  ),
        .s_axi_arsize   (sw_arsize ),
        .s_axi_arburst  (sw_arburst),
        .s_axi_arvalid  (sw_arvalid),
        .s_axi_arready  (sw_arready),
        .s_axi_rdata    (sw_rdata  ),
        .s_axi_rresp    (sw_rresp  ),
        .s_axi_rlast    (sw_rlast  ),
        .s_axi_rvalid   (sw_rvalid ),
        .s_axi_rready   (sw_rready ),
        .sw_i           (sw)
    );

    led_wrap U_led (
        .aclk           (sys_clk),
        .aresetn        (!sys_rst),
        .s_axi_awaddr   (led_awaddr ),
        .s_axi_awlen    (led_awlen  ),
        .s_axi_awsize   (led_awsize ),
        .s_axi_awburst  (led_awburst),
        .s_axi_awvalid  (led_awvalid),
        .s_axi_awready  (led_awready),
        .s_axi_wdata    (led_wdata  ),
        .s_axi_wstrb    (led_wstrb  ),
        .s_axi_wlast    (led_wlast  ),
        .s_axi_wvalid   (led_wvalid ),
        .s_axi_wready   (led_wready ),
        .s_axi_bresp    (led_bresp  ),
        .s_axi_bvalid   (led_bvalid ),
        .s_axi_bready   (led_bready ),
        .s_axi_araddr   (led_araddr ),
        .s_axi_arlen    (led_arlen  ),
        .s_axi_arsize   (led_arsize ),
        .s_axi_arburst  (led_arburst),
        .s_axi_arvalid  (led_arvalid),
        .s_axi_arready  (led_arready),
        .s_axi_rdata    (led_rdata  ),
        .s_axi_rresp    (led_rresp  ),
        .s_axi_rlast    (led_rlast  ),
        .s_axi_rvalid   (led_rvalid ),
        .s_axi_rready   (led_rready ),
        .led_o          (led)
    );

    digled_wrap U_digled (
        .aclk           (sys_clk),
        .aresetn        (!sys_rst),
        .s_axi_awaddr   (digled_awaddr ),
        .s_axi_awlen    (digled_awlen  ),
        .s_axi_awsize   (digled_awsize ),
        .s_axi_awburst  (digled_awburst),
        .s_axi_awvalid  (digled_awvalid),
        .s_axi_awready  (digled_awready),
        .s_axi_wdata    (digled_wdata  ),
        .s_axi_wstrb    (digled_wstrb  ),
        .s_axi_wlast    (digled_wlast  ),
        .s_axi_wvalid   (digled_wvalid ),
        .s_axi_wready   (digled_wready ),
        .s_axi_bresp    (digled_bresp  ),
        .s_axi_bvalid   (digled_bvalid ),
        .s_axi_bready   (digled_bready ),
        .s_axi_araddr   (digled_araddr ),
        .s_axi_arlen    (digled_arlen  ),
        .s_axi_arsize   (digled_arsize ),
        .s_axi_arburst  (digled_arburst),
        .s_axi_arvalid  (digled_arvalid),
        .s_axi_arready  (digled_arready),
        .s_axi_rdata    (digled_rdata  ),
        .s_axi_rresp    (digled_rresp  ),
        .s_axi_rlast    (digled_rlast  ),
        .s_axi_rvalid   (digled_rvalid ),
        .s_axi_rready   (digled_rready ),
        .dig_en         (dig_en),
        .dig_seg        (dig_seg)
    );

    uart_wrap U_uart (
        .aclk           (sys_clk),
        .aresetn        (!sys_rst),
        .s_axi_awaddr   (uart_awaddr ),
        .s_axi_awlen    (uart_awlen  ),
        .s_axi_awsize   (uart_awsize ),
        .s_axi_awburst  (uart_awburst),
        .s_axi_awvalid  (uart_awvalid),
        .s_axi_awready  (uart_awready),
        .s_axi_wdata    (uart_wdata  ),
        .s_axi_wstrb    (uart_wstrb  ),
        .s_axi_wlast    (uart_wlast  ),
        .s_axi_wvalid   (uart_wvalid ),
        .s_axi_wready   (uart_wready ),
        .s_axi_bresp    (uart_bresp  ),
        .s_axi_bvalid   (uart_bvalid ),
        .s_axi_bready   (uart_bready ),
        .s_axi_araddr   (uart_araddr ),
        .s_axi_arlen    (uart_arlen  ),
        .s_axi_arsize   (uart_arsize ),
        .s_axi_arburst  (uart_arburst),
        .s_axi_arvalid  (uart_arvalid),
        .s_axi_arready  (uart_arready),
        .s_axi_rdata    (uart_rdata  ),
        .s_axi_rresp    (uart_rresp  ),
        .s_axi_rlast    (uart_rlast  ),
        .s_axi_rvalid   (uart_rvalid ),
        .s_axi_rready   (uart_rready ),
        .tx             (tx),
        .rx             (rx)
    );

    timer_wrap U_timer (
        .aclk           (sys_clk),
        .aresetn        (!sys_rst),
        .s_axi_awaddr   (tim_awaddr ),
        .s_axi_awlen    (tim_awlen  ),
        .s_axi_awsize   (tim_awsize ),
        .s_axi_awburst  (tim_awburst),
        .s_axi_awvalid  (tim_awvalid),
        .s_axi_awready  (tim_awready),
        .s_axi_wdata    (tim_wdata  ),
        .s_axi_wstrb    (tim_wstrb  ),
        .s_axi_wlast    (tim_wlast  ),
        .s_axi_wvalid   (tim_wvalid ),
        .s_axi_wready   (tim_wready ),
        .s_axi_bresp    (tim_bresp  ),
        .s_axi_bvalid   (tim_bvalid ),
        .s_axi_bready   (tim_bready ),
        .s_axi_araddr   (tim_araddr ),
        .s_axi_arlen    (tim_arlen  ),
        .s_axi_arsize   (tim_arsize ),
        .s_axi_arburst  (tim_arburst),
        .s_axi_arvalid  (tim_arvalid),
        .s_axi_arready  (tim_arready),
        .s_axi_rdata    (tim_rdata  ),
        .s_axi_rresp    (tim_rresp  ),
        .s_axi_rlast    (tim_rlast  ),
        .s_axi_rvalid   (tim_rvalid ),
        .s_axi_rready   (tim_rready )
    );
`endif

endmodule
