`timescale 1ns / 1ps
module led_wrap(
    //----------------------------------------------------------------------
    // 时钟与复位
    //----------------------------------------------------------------------
    input  wire         aclk,       // 系统时钟 (默认 50MHz)
    input  wire         aresetn,    // 异步复位, 低有效

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Address Channel (AW)
    // 来自 AXI Crossbar 的写地址请求
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_awaddr,   // 写地址 (仅低 9 位有效)
    input  wire [ 7:0]  s_axi_awlen,    // 猝发长度 (单拍=0)
    input  wire [ 2:0]  s_axi_awsize,   // 每拍字节数 (3'b010=4B)
    input  wire [ 1:0]  s_axi_awburst,  // 猝发类型
    input  wire         s_axi_awvalid,  // Master 侧写地址有效
    output wire         s_axi_awready,  // Slave 侧就绪 (来自 Protocol Converter)

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Data Channel (W)
    // CPU 要写入 LED 的值
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_wdata,    // 写数据 (低 24 位对应 24 个 LED)
    input  wire [ 3:0]  s_axi_wstrb,    // 字节使能 (4'hF=全部有效)
    input  wire         s_axi_wlast,    // 最后一拍标志
    input  wire         s_axi_wvalid,   // Master 侧写数据有效
    output wire         s_axi_wready,   // Slave 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Write Response Channel (B)
    // 写操作完成响应
    //----------------------------------------------------------------------
    output wire [ 1:0]  s_axi_bresp,    // 写响应状态 (2'b00=OK)
    output wire         s_axi_bvalid,   // Slave 侧响应有效
    input  wire         s_axi_bready,   // Master 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Read Address Channel (AR)
    // 来自 AXI Crossbar 的读地址请求 (LED 一般只写不读)
    //----------------------------------------------------------------------
    input  wire [31:0]  s_axi_araddr,   // 读地址
    input  wire [ 7:0]  s_axi_arlen,    // 猝发长度
    input  wire [ 2:0]  s_axi_arsize,   // 每拍字节数
    input  wire [ 1:0]  s_axi_arburst,  // 猝发类型
    input  wire         s_axi_arvalid,  // Master 侧读地址有效
    output wire         s_axi_arready,  // Slave 侧就绪

    //----------------------------------------------------------------------
    // AXI4 Slave 接口 — Read Data Channel (R)
    // 返回当前 LED 寄存器的值
    //----------------------------------------------------------------------
    output wire [31:0]  s_axi_rdata,    // 读数据
    output wire [ 1:0]  s_axi_rresp,    // 读响应状态
    output wire         s_axi_rlast,    // 最后一拍标志
    output wire         s_axi_rvalid,   // Slave 侧读数据有效
    input  wire         s_axi_rready,   // Master 侧就绪

    //----------------------------------------------------------------------
    // Minisys LED 输出引脚
    //----------------------------------------------------------------------
    output wire [23:0]  led_o           // 24 个 LED (高电平点亮)
);

    // 第1级：AXI Protocol Converter — 协议转换 AXI4 → AXI4-Lite

    wire [31:0]  m_axi_awaddr;      // AXI4-Lite 写地址 (→ GPIO)
    wire [ 2:0]  m_axi_awprot;      // AXI4-Lite 保护类型
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
    // 配置: AXI4 Slave → AXI4-Lite Master, Data Width=32bit
    axi_protocol_converter_0 U_conv (
        .aclk           (aclk),
        .aresetn        (aresetn),
        // ---- Slave 侧 (AXI4, 连接 AXI Crossbar) ----
        .s_axi_awaddr   (s_axi_awaddr),
        .s_axi_awlen    (s_axi_awlen),
        .s_axi_awsize   (s_axi_awsize),
        .s_axi_awburst  (s_axi_awburst),
        .s_axi_awlock   (1'b0),        // 不支持 locked/atomic 访问
        .s_axi_awcache  (4'h0),        // 外设无缓存属性
        .s_axi_awprot   (3'h0),        // 默认保护
        .s_axi_awregion (4'h0),        // 无区域划分
        .s_axi_awqos    (4'h0),        // 无 QoS
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
        // ---- Master 侧 (AXI4-Lite, 连接 AXI GPIO) ----
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

    // 第2级：AXI GPIO — 存储 CPU 写入的 LED 控制值

    wire [31:0] gpio_out;           // GPIO 输出值 = LED 控制数据

    // Vivado IP 核实例化: AXI GPIO (All Outputs, 32bit)
    axi_gpio_led U_gpio (
        .s_axi_aclk     (aclk),                     // AXI4-Lite 时钟
        .s_axi_aresetn  (aresetn),                  // AXI4-Lite 复位 (低有效)
        // ---- AXI4-Lite 接口 (来自 Protocol Converter) ----
        .s_axi_awaddr   (m_axi_awaddr[8:0]),        // 写地址, 仅低 9 位
        .s_axi_awvalid  (m_axi_awvalid),
        .s_axi_awready  (m_axi_awready),
        .s_axi_wdata    (m_axi_wdata),
        .s_axi_wstrb    (m_axi_wstrb),
        .s_axi_wvalid   (m_axi_wvalid),
        .s_axi_wready   (m_axi_wready),
        .s_axi_bresp    (m_axi_bresp),
        .s_axi_bvalid   (m_axi_bvalid),
        .s_axi_bready   (m_axi_bready),
        .s_axi_araddr   (m_axi_araddr[8:0]),        // 读地址, 仅低 9 位
        .s_axi_arvalid  (m_axi_arvalid),
        .s_axi_arready  (m_axi_arready),
        .s_axi_rdata    (m_axi_rdata),
        .s_axi_rresp    (m_axi_rresp),
        .s_axi_rvalid   (m_axi_rvalid),
        .s_axi_rready   (m_axi_rready),
        // ---- GPIO 输出端口 ----
        .gpio_io_o      (gpio_out)                  // 32bit 输出值
    );

    //==========================================================================
    // 输出映射：gpio_out[23:0] → led_o[23:0]
    //==========================================================================
    // - Minisys 开发板有 24 个 LED, 高电平点亮
    // - gpio_out 高 8 位 [31:24] 被丢弃（无对应物理 LED）
    // - 直接组合逻辑赋值, 零延迟 → LED 在当前时钟周期立即响应 AXI 写入
    assign led_o = gpio_out[23:0];

endmodule
