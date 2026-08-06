`timescale 1ns / 1ps

//==============================================================================
// uart_wrap — UART 串口外设封装模块
//==============================================================================
// 功能：提供 UART 串行通信接口，CPU 可通过 AXI 总线收发数据。
//
// 内部架构（自顶向下数据流）：
//   AXI4 (来自 Crossbar)
//     ↓
//   Protocol Converter — 协议转换：AXI4 → AXI4-Lite
//     ↓
//   AXI Uartlite — 串口控制器
//     ↓
//   Minisys 引脚 — tx (发送), rx (接收)
//
// 基地址：0xFFFF_3000（在 AXI Crossbar 中配置）
//
// UART 参数（在 AXI Uartlite IP 核配置中设定）：
//   - 波特率：115200 bps
//   - 数据位：8bit
//   - 停止位：1bit
//   - 无校验位
//   - 时钟频率：50MHz（须与 clk_wiz_0 输出一致）
//
// AXI Uartlite 内部寄存器（详见 PG142 AXI Uartlite 手册）：
//   偏移     | 名称        | 功能
//   ---------|------------|-------------------------------
//   0x00     | RX_FIFO    | 接收 FIFO（只读, 32bit 中低 8bit 有效）
//   0x04     | TX_FIFO    | 发送 FIFO（只写, 写低 8bit 即发送一字节）
//   0x08     | STAT_REG   | 状态寄存器（只读）:
//            |            |   bit[0]: RX 数据有效 (rx_valid)
//            |            |   bit[1]: RX FIFO 满
//            |            |   bit[2]: TX FIFO 空 (tx_empty)
//            |            |   bit[3]: TX FIFO 满
//            |            |   bit[4]: 中断使能状态
//            |            |   bit[5]: 校验错 (overrun error)
//            |            |   bit[6]: 帧错误 (frame error)
//            |            |   bit[7]: 中断标志
//   0x0C     | CTRL_REG   | 控制寄存器（读写）:
//            |            |   bit[0]: 使能中断 (1=开)
//            |            |   bit[1]: 复位 RX FIFO  (写 1 清零)
//            |            |   bit[2]: 复位 TX FIFO  (写 1 清零)
//
// 典型 C 语言 UART 轮询发送流程：
//   1. 读 STAT_REG, 等待 bit[2] (TX FIFO Empty) = 1
//   2. 写字节到 TX_FIFO (偏移 0x04)
//
// 典型 C 语言 UART 轮询接收流程：
//   1. 读 STAT_REG, 等待 bit[0] (RX Valid) = 1
//   2. 读字节从 RX_FIFO (偏移 0x00)
//==============================================================================

module uart_wrap(
    //----------------------------------------------------------------------
    // 时钟与复位
    //----------------------------------------------------------------------
    input  wire         aclk,       // 系统时钟 (默认 50MHz)
    input  wire         aresetn,    // 异步复位, 低有效

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Address Channel (AW)
    // CPU 写 TX_FIFO 或 CTRL_REG 时触发
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_awaddr,   // 写地址
    input  wire [ 7:0]  s_axi_awlen,    // 猝发长度 (单拍=0)
    input  wire [ 2:0]  s_axi_awsize,   // 每拍字节数 (3'b010=4B)
    input  wire [ 1:0]  s_axi_awburst,  // 猝发类型
    input  wire         s_axi_awvalid,  // Master 侧写地址有效
    output wire         s_axi_awready,  // Slave 侧就绪 (来自 Protocol Converter)

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Data Channel (W)
    // CPU 发送的数据字节
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_wdata,    // 写数据 (只低 8 位有效 = 发送的字节)
    input  wire [ 3:0]  s_axi_wstrb,    // 字节使能 (4'hF=全部有效)
    input  wire         s_axi_wlast,    // 最后一拍标志
    input  wire         s_axi_wvalid,   // Master 侧写数据有效
    output wire         s_axi_wready,   // Slave 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Response Channel (B)
    //----------------------------------------------------------------------
    output wire [ 1:0]  s_axi_bresp,    // 写响应状态 (2'b00=OK)
    output wire         s_axi_bvalid,   // Slave 侧响应有效
    input  wire         s_axi_bready,   // Master 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Read Address Channel (AR)
    // CPU 读 RX_FIFO 或 STAT_REG 时触发
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_araddr,   // 读地址
    input  wire [ 7:0]  s_axi_arlen,    // 猝发长度
    input  wire [ 2:0]  s_axi_arsize,   // 每拍字节数
    input  wire [ 1:0]  s_axi_arburst,  // 猝发类型
    input  wire         s_axi_arvalid,  // Master 侧读地址有效
    output wire         s_axi_arready,  // Slave 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Read Data Channel (R)
    // 返回接收字节或状态
    //----------------------------------------------------------------------
    output wire [31:0]  s_axi_rdata,    // 读数据
    output wire [ 1:0]  s_axi_rresp,    // 读响应状态
    output wire         s_axi_rlast,    // 最后一拍标志
    output wire         s_axi_rvalid,   // Slave 侧读数据有效
    input  wire         s_axi_rready,   // Master 侧就绪

    //----------------------------------------------------------------------
    // Minisys UART 物理引脚
    //----------------------------------------------------------------------
    output wire         tx,             // 串行发送引脚 → 计算机 RX
    input  wire         rx              // 串行接收引脚 ← 计算机 TX
);

    //==========================================================================
    // 第1级：AXI Protocol Converter — 协议转换 AXI4 → AXI4-Lite
    //==========================================================================
    // 为什么需要协议转换？
    // - AXI Crossbar 使用 AXI4 协议（支持猝发传输）
    // - AXI Uartlite IP 核使用 AXI4-Lite 协议（仅单拍传输）
    // - Protocol Converter 自动剥离 AXI4 的猝发/锁/缓存等高级信号，
    //   转换为 Uartlite 能理解的简单 AXI4-Lite 读写

    wire [31:0]  m_axi_awaddr;      // AXI4-Lite 写地址 (→ Uartlite)
    wire [ 2:0]  m_axi_awprot;      // AXI4-Lite 写保护类型
    wire         m_axi_awvalid;     // AXI4-Lite 写地址有效
    wire         m_axi_awready;     // AXI4-Lite 写地址就绪 (← Uartlite)
    wire [31:0]  m_axi_wdata;       // AXI4-Lite 写数据
    wire [ 3:0]  m_axi_wstrb;       // AXI4-Lite 写字节使能
    wire         m_axi_wvalid;      // AXI4-Lite 写数据有效
    wire         m_axi_wready;      // AXI4-Lite 写数据就绪 (← Uartlite)
    wire [ 1:0]  m_axi_bresp;       // AXI4-Lite 写响应状态 (← Uartlite)
    wire         m_axi_bvalid;      // AXI4-Lite 写响应有效 (← Uartlite)
    wire         m_axi_bready;      // AXI4-Lite 写响应就绪
    wire [31:0]  m_axi_araddr;      // AXI4-Lite 读地址
    wire [ 2:0]  m_axi_arprot;      // AXI4-Lite 读保护类型
    wire         m_axi_arvalid;     // AXI4-Lite 读地址有效
    wire         m_axi_arready;     // AXI4-Lite 读地址就绪 (← Uartlite)
    wire [31:0]  m_axi_rdata;       // AXI4-Lite 读数据 (← Uartlite)
    wire [ 1:0]  m_axi_rresp;       // AXI4-Lite 读响应状态 (← Uartlite)
    wire         m_axi_rvalid;      // AXI4-Lite 读数据有效 (← Uartlite)
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
        // ---- Master 侧 (AXI4-Lite, 连 AXI Uartlite) ----
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

    // 第2级：AXI Uartlite — 串口收发控制器

    axi_uartlite_0 U_uart (
        .s_axi_aclk     (aclk),                     // AXI4-Lite 时钟 (50MHz)
        .s_axi_aresetn  (aresetn),                  // AXI4-Lite 复位 (低有效)
        // ---- AXI4-Lite 接口 (来自 Protocol Converter) ----
        .s_axi_awaddr   (m_axi_awaddr[3:0]),        // 写地址, 仅低 4 位 (4 个寄存器)
        .s_axi_awvalid  (m_axi_awvalid),
        .s_axi_awready  (m_axi_awready),
        .s_axi_wdata    (m_axi_wdata),
        .s_axi_wstrb    (m_axi_wstrb),
        .s_axi_wvalid   (m_axi_wvalid),
        .s_axi_wready   (m_axi_wready),
        .s_axi_bresp    (m_axi_bresp),
        .s_axi_bvalid   (m_axi_bvalid),
        .s_axi_bready   (m_axi_bready),
        .s_axi_araddr   (m_axi_araddr[3:0]),        // 读地址, 仅低 4 位
        .s_axi_arvalid  (m_axi_arvalid),
        .s_axi_arready  (m_axi_arready),
        .s_axi_rdata    (m_axi_rdata),
        .s_axi_rresp    (m_axi_rresp),
        .s_axi_rvalid   (m_axi_rvalid),
        .s_axi_rready   (m_axi_rready),
        // ---- UART 物理引脚 ----
        .tx             (tx),                       // 串行输出 → PC 的 RX
        .rx             (rx),                       // 串行输入 ← PC 的 TX
        .interrupt      ()                          // 中断输出 (未使用, 悬空)
    );

endmodule
