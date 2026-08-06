`timescale 1ns / 1ps

//==============================================================================
// switch_wrap — 拨码开关外设封装模块
//==============================================================================
// 功能：将 24 个拨码开关的物理状态反映到 AXI 地址空间, 供 CPU 读取。
//
// 内部架构（自底向上数据流）：
//   Minisys 引脚 sw_i[23:0] (物理开关)
//     ↓
//   AXI GPIO (All Inputs) — 采样开关状态, 存入 32bit 内部寄存器
//     ↓
//   Protocol Converter — 协议转换：AXI4-Lite → AXI4
//     ↓
//   AXI4 (到 Crossbar)
//
// 基地址：0xFFFF_0000（在 AXI Crossbar 中配置）
// 只读（CPU 写入无意义, 对物理开关无影响）
// 数据映射：
//   - gpio_io_i[31:24] = 8'h00 (高 8 位恒为零, 24 个开关的物理上方)
//   - gpio_io_i[23:0]  = sw_i[23:0] (24 个拨码开关状态,
//     开关拨上=1, 拨下=0)
//
// 为什么用 All Inputs？
// - 拨码开关是纯输入设备, CPU 只读不写
// - All Inputs 模式下 GPIO 内部有同步器, 将异步的开关信号同步到 aclk 域
// - 避免了亚稳态, CPU 读到的永远是一个稳定的采样值
//==============================================================================

module switch_wrap(
    //----------------------------------------------------------------------
    // 时钟与复位
    //----------------------------------------------------------------------
    input  wire         aclk,       // 系统时钟 (默认 50MHz)
    input  wire         aresetn,    // 异步复位, 低有效

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Address Channel (AW)
    // 开关是只读设备, 写通道仍存在但写入值被 GPIO 内部忽略
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_awaddr,   // 写地址
    input  wire [ 7:0]  s_axi_awlen,    // 猝发长度
    input  wire [ 2:0]  s_axi_awsize,   // 每拍字节数
    input  wire [ 1:0]  s_axi_awburst,  // 猝发类型
    input  wire         s_axi_awvalid,  // Master 侧写地址有效
    output wire         s_axi_awready,  // Slave 侧就绪 (来自 Protocol Converter)

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Data Channel (W)
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_wdata,    // 写数据 (对开关无实际影响)
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
    // CPU 读开关状态时触发
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_araddr,   // 读地址 (偏移 0x0000 读取 GPIO_DATA 寄存器)
    input  wire [ 7:0]  s_axi_arlen,    // 猝发长度
    input  wire [ 2:0]  s_axi_arsize,   // 每拍字节数
    input  wire [ 1:0]  s_axi_arburst,  // 猝发类型
    input  wire         s_axi_arvalid,  // Master 侧读地址有效
    output wire         s_axi_arready,  // Slave 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Read Data Channel (R)
    // 返回当前开关状态
    //----------------------------------------------------------------------
    output wire [31:0]  s_axi_rdata,    // 读数据 = {8'h00, sw_i[23:0]}
    output wire [ 1:0]  s_axi_rresp,    // 读响应状态
    output wire         s_axi_rlast,    // 最后一拍标志
    output wire         s_axi_rvalid,   // Slave 侧读数据有效
    input  wire         s_axi_rready,   // Master 侧就绪

    //----------------------------------------------------------------------
    // Minsys 拨码开关输入引脚
    //----------------------------------------------------------------------
    input  wire [23:0]  sw_i            // 24 个拨码开关 (拨上=1, 拨下=0)
);

    //==========================================================================
    // 第1级：AXI Protocol Converter — 协议转换 AXI4 → AXI4-Lite
    //==========================================================================
    // 为什么需要协议转换？
    // - AXI Crossbar 遵循 AXI4 协议（支持猝发传输、lock/cache/prot/region/qos）
    // - AXI GPIO IP 核遵循 AXI4-Lite 协议（每次只传输 1 个数据, 无猝发）
    // - Protocol Converter 在中间作桥接：
    //   * 读方向: AXI4 AR 通道 → AXI4-Lite AR 通道 → 等待 R 数据 → 回传
    //   * 写方向: AXI4 AW+W 通道 → AXI4-Lite AW+W → 等待 B 响应 → 回传

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
        .s_axi_awlock   (1'b0),        // 不支持 exclusive access
        .s_axi_awcache  (4'h0),        // 外设无缓存
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


    // 第2级：AXI GPIO — 采样拨码开关物理状态

    axi_gpio_switch U_gpio (
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
        .gpio_io_i      ({8'h0, sw_i})
    );

endmodule
