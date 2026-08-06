`timescale 1ns / 1ps

`include "defines.vh"
module cpu_top(
    input  wire         cpu_clk,        // CPU时钟输入 (接 sys_clk, 由SoC顶层管理)
    input  wire         cpu_rst,        // CPU复位输入 (接 sys_rst, 高电平有效)

    //------------ AXI4 Master 接口 (对外连接 SoC 总线) -----------------------
    // 写地址通道 (AW)
    output wire [31:0]  m_axi_awaddr,   // 写地址 (32bit字节地址)
    output wire [ 7:0]  m_axi_awlen,    // 写猝发长度 (burst beat数-1)
    output wire [ 2:0]  m_axi_awsize,   // 写传输位宽 (3'b010=4字节/beat)
    output wire [ 1:0]  m_axi_awburst,  // 写猝发类型 (2'b01=INCR递增猝发)
    input  wire         m_axi_awready,  // 写地址就绪 (从设备反馈)
    output wire         m_axi_awvalid,  // 写地址有效
    // 写数据通道 (W)
    output wire [31:0]  m_axi_wdata,    // 写数据 (32bit)
    input  wire         m_axi_wready,   // 写数据就绪 (从设备反馈)
    output wire [ 3:0]  m_axi_wstrb,    // 写字节使能 (bit[i]=1→写wdata[8i+7:8i])
    output wire         m_axi_wlast,    // 写猝发最后一拍标志
    output wire         m_axi_wvalid,   // 写数据有效
    // 写响应通道 (B)
    output wire         m_axi_bready,   // 写响应就绪 (主设备准备好接收)
    input  wire [ 1:0]  m_axi_bresp,    // 写响应状态 (2'b00=OK, 2'b10=SLVERR, 2'b11=DECERR)
    input  wire         m_axi_bvalid,   // 写响应有效 (从设备反馈)
    // 读地址通道 (AR)
    output wire [31:0]  m_axi_araddr,   // 读地址 (32bit字节地址)
    output wire [ 7:0]  m_axi_arlen,    // 读猝发长度
    output wire [ 2:0]  m_axi_arsize,   // 读传输位宽
    output wire [ 1:0]  m_axi_arburst,  // 读猝发类型
    input  wire         m_axi_arready,  // 读地址就绪 (从设备反馈)
    output wire         m_axi_arvalid,  // 读地址有效
    // 读数据通道 (R)
    input  wire [31:0]  m_axi_rdata,    // 读回数据 (32bit)
    output wire         m_axi_rready,   // 读数据就绪 (主设备准备好接收)
    input  wire [ 1:0]  m_axi_rresp,    // 读响应状态
    input  wire         m_axi_rlast,    // 读猝发最后一拍标志
    input  wire         m_axi_rvalid    // 读数据有效 (从设备反馈)
);

    //------------ cpu_core ↔ ICache 取指接口信号 -----------------------------
    // 与实验1 Inst_ROM 接口完全兼容，对 cpu_core 透明
    wire        cpu2ic_rreq;      // 取指请求 (高电平有效, 仅有效1个时钟周期)
    wire [31:0] cpu2ic_addr;      // 取指地址 (字节地址)
    wire        ic2cpu_valid;     // 指令有效标志 (高电平表示返回一条有效指令)
    wire [31:0] ic2cpu_inst;      // 指令机器码 (32bit)

    //------------ cpu_core ↔ DCache 数据访存接口信号 -------------------------
    // 与实验1 Data_RAM 接口完全兼容，对 cpu_core 透明
    wire [ 3:0] cpu2dc_ren;       // 读使能 (4'hF=读字, 按字节控制的其他组合)
    wire [31:0] cpu2dc_addr;      // 读/写地址 (字节地址)
    wire        dc2cpu_valid;     // 读数据有效标志
    wire [31:0] dc2cpu_rdata;     // 读回的数据 (32bit)
    wire [ 3:0] cpu2dc_wen;       // 写使能 (按bit控制: 4'hF=写字, 4'h3/4'hC=写半字, 4'h1/2/4/8=写字节)
    wire [31:0] cpu2dc_wdata;     // 写数据 (32bit)
    wire        dc2cpu_wresp;     // 写响应 (高电平有效, 表示写操作完成)

    //------------ ICache → axi_master 读接口信号 ----------------------------
    // ICache miss时, 通过此接口请求 axi_master 从总线读取一个128bit cache line
    wire        ic_dev_rrdy;                     // axi_master → ICache: 读通道就绪(可接受读请求)
    wire [ 3:0] ic_cpu_ren;                      // ICache → axi_master: 读使能(按32bit字, 全1=读整行)
    wire [31:0] ic_cpu_raddr;                    // ICache → axi_master: 读地址(对齐到128bit边界)
    wire        ic_dev_rvalid;                   // axi_master → ICache: 读数据有效(128bit已就绪)
    wire [`IC_BLK_SIZE-1:0] ic_dev_rdata;        // axi_master → ICache: 读数据块(128bit=4条指令)

    //------------ DCache → axi_master 读接口信号 ----------------------------
    // DCache read miss时, 通过此接口请求 axi_master 读取一个128bit cache line
    wire        dc_dev_rrdy;                     // axi_master → DCache: 读通道就绪
    wire [ 3:0] dc_cpu_ren;                      // DCache → axi_master: 读使能
    wire [31:0] dc_cpu_raddr;                    // DCache → axi_master: 读地址
    wire        dc_dev_rvalid;                   // axi_master → DCache: 读数据有效
    wire [`DC_BLK_SIZE-1:0] dc_dev_rdata;        // axi_master → DCache: 读数据块(128bit)

    //------------ DCache → axi_master 写接口信号 ----------------------------
    // DCache写穿(write-through): 每次CPU写数据, DCache同时写给axi_master
    wire        dc_dev_wrdy;                     // axi_master → DCache: 写通道就绪
    wire [ 3:0] dc_cpu_wen;                      // DCache → axi_master: 按字节写使能
    wire [31:0] dc_cpu_waddr;                    // DCache → axi_master: 写地址
    wire [31:0] dc_cpu_wdata;                    // DCache → axi_master: 写数据(32bit)

    //==========================================================================
    // cpu_core 例化
    //==========================================================================
    cpu_core U_core (
        .cpu_clk        (cpu_clk),               // CPU时钟
        .cpu_rst        (cpu_rst),               // CPU复位
        // 取指接口 (→ ICache)
        .ifetch_req     (cpu2ic_rreq),           // 取指请求
        .ifetch_addr    (cpu2ic_addr),           // 取指地址
        .ifetch_valid   (ic2cpu_valid),          // ← 指令有效标志
        .ifetch_inst    (ic2cpu_inst),           // ← 指令机器码
        // 数据访存接口 (→ DCache)
        .daccess_ren    (cpu2dc_ren),            // 读使能
        .daccess_addr   (cpu2dc_addr),           // 读/写地址
        .daccess_rvalid (dc2cpu_valid),          // ← 读数据有效
        .daccess_rdata  (dc2cpu_rdata),          // ← 读回数据
        .daccess_wen    (cpu2dc_wen),            // 写使能
        .daccess_wdata  (cpu2dc_wdata),          // 写数据
        .daccess_wresp  (dc2cpu_wresp)           // ← 写响应
    );

    //==========================================================================
    // ICache 例化 — 指令Cache
    //==========================================================================
    ICache U_icache (
        .cpu_clk        (cpu_clk),               // CPU时钟
        .cpu_rst        (cpu_rst),               // CPU复位
        // CPU侧接口 (与原IROM接口兼容)
        .inst_rreq      (cpu2ic_rreq),           // ← cpu_core: 取指请求
        .inst_addr      (cpu2ic_addr),           // ← cpu_core: 取指地址
        .inst_valid     (ic2cpu_valid),          // → cpu_core: 指令有效
        .inst_out       (ic2cpu_inst),           // → cpu_core: 指令机器码
        // 总线侧读接口 (连接 axi_master)
        .dev_rrdy       (ic_dev_rrdy),           // ← axi_master: 读通道就绪
        .cpu_ren        (ic_cpu_ren),            // → axi_master: 读使能
        .cpu_raddr      (ic_cpu_raddr),          // → axi_master: 读地址(128bit对齐)
        .dev_rvalid     (ic_dev_rvalid),         // ← axi_master: 读数据有效
        .dev_rdata      (ic_dev_rdata)           // ← axi_master: 读数据块[127:0]
    );

    //==========================================================================
    // DCache 例化 — 数据Cache
    //==========================================================================
    DCache U_dcache (
        .cpu_clk        (cpu_clk),               // CPU时钟
        .cpu_rst        (cpu_rst),               // CPU复位
        // CPU侧接口 (与原DRAM接口兼容)
        .data_ren       (cpu2dc_ren),            // ← cpu_core: 读使能
        .data_addr      (cpu2dc_addr),           // ← cpu_core: 读/写地址
        .data_valid     (dc2cpu_valid),          // → cpu_core: 读数据有效
        .data_rdata     (dc2cpu_rdata),          // → cpu_core: 读回数据
        .data_wen       (cpu2dc_wen),            // ← cpu_core: 写使能
        .data_wdata     (cpu2dc_wdata),          // ← cpu_core: 写数据
        .data_wresp     (dc2cpu_wresp),          // → cpu_core: 写响应
        // 总线侧写接口 (连接 axi_master, 写穿策略)
        .dev_wrdy       (dc_dev_wrdy),           // ← axi_master: 写通道就绪
        .cpu_wen        (dc_cpu_wen),            // → axi_master: 按字节写使能
        .cpu_waddr      (dc_cpu_waddr),          // → axi_master: 写地址
        .cpu_wdata      (dc_cpu_wdata),          // → axi_master: 写数据
        // 总线侧读接口 (连接 axi_master, 用于read miss refill)
        .dev_rrdy       (dc_dev_rrdy),           // ← axi_master: 读通道就绪
        .cpu_ren        (dc_cpu_ren),            // → axi_master: 读使能
        .cpu_raddr      (dc_cpu_raddr),          // → axi_master: 读地址(128bit对齐)
        .dev_rvalid     (dc_dev_rvalid),         // ← axi_master: 读数据有效
        .dev_rdata      (dc_dev_rdata)           // ← axi_master: 读数据块[127:0]
    );

    //==========================================================================
    // axi_master 例化 — AXI总线控制器
    //==========================================================================
    axi_master U_aximaster (
        .aclk           (cpu_clk),               // AXI总线时钟
        .areset         (cpu_rst),               // AXI总线复位 (高有效)

        // ICache 读接口 (取指miss → AXI AR→R)
        .ic_dev_rrdy    (ic_dev_rrdy),           // → ICache: 读通道就绪
        .ic_cpu_ren     (ic_cpu_ren),            // ← ICache: 读使能
        .ic_cpu_raddr   (ic_cpu_raddr),          // ← ICache: 读地址
        .ic_dev_rvalid  (ic_dev_rvalid),         // → ICache: 读数据有效
        .ic_dev_rdata   (ic_dev_rdata),          // → ICache: 读数据块

        // DCache 读接口 (读miss → AXI AR→R)
        .dc_dev_rrdy    (dc_dev_rrdy),           // → DCache: 读通道就绪
        .dc_cpu_ren     (dc_cpu_ren),            // ← DCache: 读使能
        .dc_cpu_raddr   (dc_cpu_raddr),          // ← DCache: 读地址
        .dc_dev_rvalid  (dc_dev_rvalid),         // → DCache: 读数据有效
        .dc_dev_rdata   (dc_dev_rdata),          // → DCache: 读数据块

        // DCache 写接口 (写穿 → AXI AW→W→B)
        .dc_dev_wrdy    (dc_dev_wrdy),           // → DCache: 写通道就绪
        .dc_cpu_wen     (dc_cpu_wen),            // ← DCache: 按字节写使能
        .dc_cpu_waddr   (dc_cpu_waddr),          // ← DCache: 写地址
        .dc_cpu_wdata   (dc_cpu_wdata),          // ← DCache: 写数据

        // AXI4 Master 接口 (对外连接 SoC 总线)
        // 写地址通道
        .m_axi_awaddr   (m_axi_awaddr),          // → 总线: 写地址
        .m_axi_awlen    (m_axi_awlen),           // → 总线: 写猝发长度
        .m_axi_awsize   (m_axi_awsize),          // → 总线: 写传输位宽
        .m_axi_awburst  (m_axi_awburst),         // → 总线: 写猝发类型
        .m_axi_awready  (m_axi_awready),         // ← 总线: 写地址就绪
        .m_axi_awvalid  (m_axi_awvalid),         // → 总线: 写地址有效
        // 写数据通道
        .m_axi_wdata    (m_axi_wdata),           // → 总线: 写数据
        .m_axi_wready   (m_axi_wready),          // ← 总线: 写数据就绪
        .m_axi_wstrb    (m_axi_wstrb),           // → 总线: 写字节使能
        .m_axi_wlast    (m_axi_wlast),           // → 总线: 写最后一拍
        .m_axi_wvalid   (m_axi_wvalid),          // → 总线: 写数据有效
        // 写响应通道
        .m_axi_bready   (m_axi_bready),          // → 总线: 写响应就绪
        .m_axi_bresp    (m_axi_bresp),           // ← 总线: 写响应状态
        .m_axi_bvalid   (m_axi_bvalid),          // ← 总线: 写响应有效
        // 读地址通道
        .m_axi_araddr   (m_axi_araddr),          // → 总线: 读地址
        .m_axi_arlen    (m_axi_arlen),           // → 总线: 读猝发长度
        .m_axi_arsize   (m_axi_arsize),          // → 总线: 读传输位宽
        .m_axi_arburst  (m_axi_arburst),         // → 总线: 读猝发类型
        .m_axi_arready  (m_axi_arready),         // ← 总线: 读地址就绪
        .m_axi_arvalid  (m_axi_arvalid),         // → 总线: 读地址有效
        // 读数据通道
        .m_axi_rdata    (m_axi_rdata),           // ← 总线: 读回数据
        .m_axi_rready   (m_axi_rready),          // → 总线: 读数据就绪
        .m_axi_rresp    (m_axi_rresp),           // ← 总线: 读响应状态
        .m_axi_rlast    (m_axi_rlast),           // ← 总线: 读最后一拍
        .m_axi_rvalid   (m_axi_rvalid)           // ← 总线: 读数据有效
    );



endmodule
