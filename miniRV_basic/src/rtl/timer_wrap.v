`timescale 1ns / 1ps

//==============================================================================
// timer_wrap — 计时器外设封装模块
//==============================================================================
// 功能：提供一个 64 位自由运行计数器，CPU 可通过 AXI 读取当前计时器值。
//
// 内部架构（自底向上数据流）：
//   64bit 自由运行计数器 timer (每时钟周期 +1)
//     ↓
//   AXI GPIO (All Inputs, Dual Channel) — 映射 timer 到两路 32bit 输入端口
//     ↓
//   Protocol Converter — 协议转换：AXI4-Lite → AXI4
//     ↓
//   AXI4 (到 Crossbar)
//
// 基地址：0xFFFF_4000（在 AXI Crossbar 中配置）
// 只读（CPU 写入无意义, 计数器不受软件控制）
//
// 寄存器映射（AXI GPIO 双通道）：
//   - 偏移 0x000: GPIO_DATA (Channel 1)    = timer[31:0]  (低 32 位)
//   - 偏移 0x008: GPIO2_DATA (Channel 2)   = timer[63:32] (高 32 位)
//
// 计时精度：
//   - 系统时钟 50MHz → 计数周期 20ns
//   - 32 位溢出周期 = 2^32 × 20ns ≈ 85.9 秒
//   - 64 位溢出周期 = 2^64 × 20ns ≈ 11,689 年 (实际永不溢出)
//
// 为什么用 64 位而非 32 位？
// - 32 位约 85 秒就溢出 → CPU 读到 timer 值前后可能不一致(低 32 位已翻转,
//   高 32 位还未读出)
// - 64 位永不溢出 → 无翻转问题, 两个 32 位半字一致性由软件负责 (先读高, 再读低,
//   再读高验证)
//
// 为什么用 All Inputs + Dual Channel？
// - timer 对 GPIO 来说是"输入"（从 GPIO 视角看, timer 是外部信号,
//   通过 gpio_io_i / gpio2_io_i 接入）
// - Dual Channel 模式：GPIO 有两个独立的 32 位输入端口
//   * Channel 1 (gpio_io_i)  → 偏移 0x000
//   * Channel 2 (gpio2_io_i) → 偏移 0x008
// - 这样 CPU 可以用两次 32 位读操作获取完整的 64 位计时值
//==============================================================================

module timer_wrap(
    //----------------------------------------------------------------------
    // 时钟与复位
    //----------------------------------------------------------------------
    input  wire         aclk,       // 系统时钟 (默认 50MHz)
    input  wire         aresetn,    // 异步复位, 低有效

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Address Channel (AW)
    // 计时器只读, 写通道保留但数据被 GPIO 内部忽略
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_awaddr,   // 写地址
    input  wire [ 7:0]  s_axi_awlen,    // 猝发长度
    input  wire [ 2:0]  s_axi_awsize,   // 每拍字节数
    input  wire [ 1:0]  s_axi_awburst,  // 猝发类型
    input  wire         s_axi_awvalid,  // Master 侧写地址有效
    output wire         s_axi_awready,  // Slave 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Data Channel (W)
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_wdata,    // 写数据 (对 timer 无影响)
    input  wire [ 3:0]  s_axi_wstrb,    // 字节使能
    input  wire         s_axi_wlast,    // 最后一拍标志
    input  wire         s_axi_wvalid,   // Master 侧写数据有效
    output wire         s_axi_wready,   // Slave 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Response Channel (B)
    //----------------------------------------------------------------------
    output wire [ 1:0]  s_axi_bresp,    // 写响应状态
    output wire         s_axi_bvalid,   // Slave 侧响应有效
    input  wire         s_axi_bready,   // Master 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Read Address Channel (AR)
    // CPU 读取计时器值
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_araddr,   // 读地址
    input  wire [ 7:0]  s_axi_arlen,    // 猝发长度
    input  wire [ 2:0]  s_axi_arsize,   // 每拍字节数
    input  wire [ 1:0]  s_axi_arburst,  // 猝发类型
    input  wire         s_axi_arvalid,  // Master 侧读地址有效
    output wire         s_axi_arready,  // Slave 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Read Data Channel (R)
    // 返回当前 timer 值
    //----------------------------------------------------------------------
    output wire [31:0]  s_axi_rdata,    // 读数据
    output wire [ 1:0]  s_axi_rresp,    // 读响应状态
    output wire         s_axi_rlast,    // 最后一拍标志
    output wire         s_axi_rvalid,   // Slave 侧读数据有效
    input  wire         s_axi_rready    // Master 侧就绪

    // 注意：timer_wrap 无外部 I/O 引脚 — 计时器完全在 FPGA 内部运行
);

    reg [63:0] timer;                   // 64 位计时器寄存器
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn)
            timer <= 64'h0;             // 复位时清零
        else
            timer <= timer + 64'h1;     // 每个时钟周期自增 1
    end


    // 第1级：AXI Protocol Converter — 协议转换 AXI4 → AXI4-Lite

    wire [31:0]  m_axi_awaddr;      // AXI4-Lite 写地址
    wire [ 2:0]  m_axi_awprot;      // AXI4-Lite 写保护类型
    wire         m_axi_awvalid;     // AXI4-Lite 写地址有效
    wire         m_axi_awready;     // AXI4-Lite 写地址就绪 (← GPIO)
    wire [31:0]  m_axi_wdata;       // AXI4-Lite 写数据
    wire [ 3:0]  m_axi_wstrb;       // AXI4-Lite 写字节使能
    wire         m_axi_wvalid;      // AXI4-Lite 写数据有效
    wire         m_axi_wready;      // AXI4-Lite 写数据就绪 (← GPIO)
    wire [ 1:0]  m_axi_bresp;       // AXI4-Lite 写响应状态 (← GPIO)
    wire         m_axi_bvalid;      // AXI4-Lite 写响应有效 (← GPIO)
    wire         m_axi_bready;      // AXI4-Lite 写响应就绪
    wire [31:0]  m_axi_araddr;      // AXI4-Lite 读地址
    wire [ 2:0]  m_axi_arprot;      // AXI4-Lite 读保护类型
    wire         m_axi_arvalid;     // AXI4-Lite 读地址有效
    wire         m_axi_arready;     // AXI4-Lite 读地址就绪 (← GPIO)
    wire [31:0]  m_axi_rdata;       // AXI4-Lite 读数据 (← GPIO)
    wire [ 1:0]  m_axi_rresp;       // AXI4-Lite 读响应状态 (← GPIO)
    wire         m_axi_rvalid;      // AXI4-Lite 读数据有效 (← GPIO)
    wire         m_axi_rready;      // AXI4-Lite 读数据就绪

    // Vivado IP 核实例化: AXI Protocol Converter
    axi_protocol_converter_0 U_conv (
        .aclk           (aclk),
        .aresetn        (aresetn),
        // ---- Slave 侧 (AXI4, 来自 Crossbar) ----
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
        // ---- Master 侧 (AXI4-Lite, 连 AXI GPIO) ----
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

    // 第2级：AXI GPIO (All Inputs, Dual Channel) — 映射 64 位 timer 值

    axi_gpio_timer U_gpio (
        .s_axi_aclk     (aclk),                     // AXI4-Lite 时钟
        .s_axi_aresetn  (aresetn),                  // AXI4-Lite 复位
        // ---- AXI4-Lite 接口 (来自 Protocol Converter) ----
        .s_axi_awaddr   (m_axi_awaddr[8:0]),        // 写地址
        .s_axi_awvalid  (m_axi_awvalid),
        .s_axi_awready  (m_axi_awready),
        .s_axi_wdata    (m_axi_wdata),
        .s_axi_wstrb    (m_axi_wstrb),
        .s_axi_wvalid   (m_axi_wvalid),
        .s_axi_wready   (m_axi_wready),
        .s_axi_bresp    (m_axi_bresp),
        .s_axi_bvalid   (m_axi_bvalid),
        .s_axi_bready   (m_axi_bready),
        .s_axi_araddr   (m_axi_araddr[8:0]),        // 读地址
        .s_axi_arvalid  (m_axi_arvalid),
        .s_axi_arready  (m_axi_arready),
        .s_axi_rdata    (m_axi_rdata),
        .s_axi_rresp    (m_axi_rresp),
        .s_axi_rvalid   (m_axi_rvalid),
        .s_axi_rready   (m_axi_rready),
        // ---- GPIO 双通道输入端口 ----
        .gpio_io_i      (timer[31:0]),              // Channel 1: timer 低 32 位
        .gpio2_io_i     (timer[63:32])              // Channel 2: timer 高 32 位
    );

endmodule
