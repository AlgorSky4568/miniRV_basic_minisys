`timescale 1ns / 1ps

`include "defines.vh"

//---------- axi_master -- AXI4总线控制器 ----------
// ICache/DCache 简单访存接口 -> AXI4协议转换
// 仲裁优先级: DCache写 > DCache读 > ICache读
// 简化: rready/bready恒1, arsize/awsize=3'b010, arburst/awburst=INCR
// 猝发长度: Cache使能=BLK_LEN-1(3), 直通=0, I/O空间=0

module axi_master(
    input  wire         aclk,
    input  wire         areset,                 // 高电平有效

    //---------- ICache 读接口 ----------
    output reg          ic_dev_rrdy,            // 就绪 -> ICache
    input  wire [ 3:0]  ic_cpu_ren,             // 读使能 <- ICache
    input  wire [31:0]  ic_cpu_raddr,           // 读地址 <- ICache
    output reg          ic_dev_rvalid,          // 数据有效 -> ICache
    output reg  [`IC_BLK_SIZE-1:0] ic_dev_rdata,// 读数据 -> ICache

    //---------- DCache 写接口 ----------
    output reg          dc_dev_wrdy,            // 写就绪 -> DCache
    input  wire [ 3:0]  dc_cpu_wen,             // 写使能 <- DCache
    input  wire [31:0]  dc_cpu_waddr,           // 写地址 <- DCache
    input  wire [31:0]  dc_cpu_wdata,           // 写数据 <- DCache

    //---------- DCache 读接口 ----------
    output reg          dc_dev_rrdy,            // 读就绪 -> DCache
    input  wire [ 3:0]  dc_cpu_ren,             // 读使能 <- DCache
    input  wire [31:0]  dc_cpu_raddr,           // 读地址 <- DCache
    output reg          dc_dev_rvalid,          // 数据有效 -> DCache
    output reg  [`DC_BLK_SIZE-1:0] dc_dev_rdata,// 读数据 -> DCache

    //---------- AXI4 写地址通道 (AW) ----------
    output reg  [31:0]  m_axi_awaddr,
    output reg  [ 7:0]  m_axi_awlen,
    output reg  [ 2:0]  m_axi_awsize,
    output reg  [ 1:0]  m_axi_awburst,
    output reg          m_axi_awvalid,
    input  wire         m_axi_awready,

    //---------- AXI4 写数据通道 (W) ----------
    output reg  [31:0]  m_axi_wdata,
    output reg  [ 3:0]  m_axi_wstrb,
    output wire         m_axi_wlast,
    output reg          m_axi_wvalid,
    input  wire         m_axi_wready,

    //---------- AXI4 写响应通道 (B) ----------
    output reg          m_axi_bready,
    input  wire [ 1:0]  m_axi_bresp,
    input  wire         m_axi_bvalid,

    //---------- AXI4 读地址通道 (AR) ----------
    output reg  [31:0]  m_axi_araddr,
    output reg  [ 7:0]  m_axi_arlen,
    output reg  [ 2:0]  m_axi_arsize,
    output reg  [ 1:0]  m_axi_arburst,
    output reg          m_axi_arvalid,
    input  wire         m_axi_arready,

    //---------- AXI4 读数据通道 (R) ----------
    output reg          m_axi_rready,
    input  wire [31:0]  m_axi_rdata,
    input  wire [ 1:0]  m_axi_rresp,
    input  wire         m_axi_rlast,
    input  wire         m_axi_rvalid
);


    //---------- 请求检测: 4-bit字节使能归约为1-bit ----------
    wire ic_ren = |ic_cpu_ren;
    wire dc_ren = |dc_cpu_ren;
    wire ren    = ic_ren | dc_ren;
    
    wire dc_wen = |dc_cpu_wen;


    //---------- 读FSM 状态编码 ----------
    // R_IDLE --ren--> R_AR_SEND --arready--> R_DATA_WAIT --rvalid&&rlast--> R_IDLE
    
    
    
    localparam R_IDLE       = 2'b00;   // 等待读请求
    localparam R_AR_SEND    = 2'b01;   // 发送AR地址
    localparam R_DATA_WAIT  = 2'b10;   // 收集R数据beat
    
    localparam W_IDLE       = 2'b00;   // 等待写请求
    localparam W_AR_SEND    = 2'b01;   // 发送AW地址
    localparam W_DATA_SEND  = 2'b10;   // 发送W数据
    localparam W_RESP_WAIT  = 2'b11;   // 等待B响应

    localparam SRC_ICACHE = 1'b0;      // 请求源: ICache
    localparam SRC_DCACHE = 1'b1;      // 请求源: DCache


    //---------- 内部寄存器 ----------
    (*mark_debug = "true"*) reg  [1:0] r_state, r_nstat;
    reg        r_src;                             // 请求源: 0=IC, 1=DC
    reg  [31:0] r_latched_addr;                   // 读地址锁存
    reg  [1:0] r_cnt;                             // beat计数器
    reg  [`IC_BLK_SIZE-1:0] ic_rdata_buf;         // IC数据拼装缓冲
    reg  [`DC_BLK_SIZE-1:0] dc_rdata_buf;         // DC数据拼装缓冲
    reg        r_data_done;                       // 拼装完成标志
    
    (*mark_debug = "true"*) reg [1:0] w_state, w_nstat;
    reg [31:0] w_latched_addr;                     // 写地址锁存
    reg [31:0] w_latched_data;                     // 写数据锁存
    reg [ 3:0] w_latched_wen;                      // 字节使能锁存 (=wstrb)


    // 写通道: wlast 在 W_DATA_SEND 时为1
    assign m_axi_wlast = (w_state == W_DATA_SEND);


    //---------- 状态寄存器 (时序) ----------
    always @(posedge aclk or posedge areset) begin
        if (areset)
            r_state <= R_IDLE;
        else
            r_state <= r_nstat;
    end


    //---------- 下一状态逻辑 (组合) ----------
    always @(*) begin
        case (r_state)

            R_IDLE: begin
                // 有读请求：R_AR_SEND接收地址
                if (ren)
                    r_nstat = R_AR_SEND;
                else
                    r_nstat = R_IDLE;
            end

            R_AR_SEND: begin
                // 总线就绪转移
                if (m_axi_arready)
                    r_nstat = R_DATA_WAIT;
                else
                    r_nstat = R_AR_SEND;
            end

            R_DATA_WAIT: begin
                // 读到最后一位，并且拉取valid
                if (m_axi_rvalid && m_axi_rlast)
                    r_nstat = R_IDLE;
                else
                    r_nstat = R_DATA_WAIT;
            end

            default: r_nstat = R_IDLE;
        endcase
    end


    //---------- 输出逻辑 (时序) ----------
    always @(posedge aclk or posedge areset) begin
        if (areset) begin
            r_src          <= SRC_ICACHE;
            r_latched_addr <= 32'h0;
            r_cnt          <= 2'd0;
            ic_rdata_buf   <= 0;
            dc_rdata_buf   <= 0;
            r_data_done    <= 1'b0;

            ic_dev_rrdy    <= 1'b1;
            ic_dev_rvalid  <= 1'b0;
            ic_dev_rdata   <= 0;
            dc_dev_rrdy    <= 1'b1;
            dc_dev_rvalid  <= 1'b0;
            dc_dev_rdata   <= 0;

            // AXI AR空闲 (有效值在R_AR_SEND中设置)
            m_axi_araddr   <= 32'h0;
            m_axi_arlen    <= 8'd0;
            m_axi_arsize   <= 3'd0;
            m_axi_arburst  <= 2'd0;
            m_axi_arvalid  <= 1'b0;
            m_axi_rready   <= 1'b1;
        end
        else begin
            // 脉冲信号默认0，仅有效时置1
            ic_dev_rvalid <= 1'b0;
            dc_dev_rvalid <= 1'b0;

            case (r_state)

                //---------- R_IDLE: 等待读请求 ----------
                R_IDLE: begin
                    // 上次读完成: 输出拼装好的数据
                    if (r_data_done) begin
                        if (r_src == SRC_DCACHE) begin
                            dc_dev_rdata  <= dc_rdata_buf;
                            dc_dev_rvalid <= 1'b1;
                        end else begin
                            ic_dev_rdata  <= ic_rdata_buf;
                            ic_dev_rvalid <= 1'b1;
                        end
                        r_data_done <= 1'b0;
                    end

                    // 仲裁: 仅选中源看到 rrdy=1
                    // 未选中源保持 rrdy=0, Cache等待
                    // 优先级: DCache读 > ICache读
                    if (dc_ren) begin
                        dc_dev_rrdy    <= 1'b1;
                        ic_dev_rrdy    <= 1'b0;     // ICache等待
                        r_src          <= SRC_DCACHE;
                        r_latched_addr <= dc_cpu_raddr;
                    end else if (ic_ren) begin
                        ic_dev_rrdy    <= 1'b1;
                        dc_dev_rrdy    <= 1'b0;     // DCache等待
                        r_src          <= SRC_ICACHE;
                        r_latched_addr <= ic_cpu_raddr;
                    end else begin
                        ic_dev_rrdy    <= 1'b1;     // 无请求: 两侧均就绪
                        dc_dev_rrdy    <= 1'b1;
                    end

                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                end

                //---------- R_AR_SEND: 驱动AXI读地址通道 ----------
                R_AR_SEND: begin
                    m_axi_araddr  <= r_latched_addr;
                    m_axi_arsize  <= 3'b010;            // 2^(i-1)=4字节/拍
                    m_axi_arburst <= 2'b01;             // INCR递增 (通用)
                    m_axi_arvalid <= 1'b1;

                    // arlen: Cache使能=3 (4拍猝发), 直通=0 (单拍)
                    // I/O空间 (0xFFFF_xxxx) 始终单拍
                    if (r_src == SRC_DCACHE && r_latched_addr[31:16] == 16'hFFFF)
                        m_axi_arlen <= 8'd0;            // Uncache: I/O空间: 无猝发
                    else if (r_src == SRC_DCACHE)
                        m_axi_arlen <= `DC_BLK_LEN - 1; // 四个字节包
                    else
                        m_axi_arlen <= `IC_BLK_LEN - 1;

                    // 阻止新请求
                    ic_dev_rrdy <= 1'b0;
                    dc_dev_rrdy <= 1'b0;

                    if (m_axi_arready)
                        r_cnt <= 2'd0;
                end

                //---------- R_DATA_WAIT: 收集AXI读数据 ----------
                R_DATA_WAIT: begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                    ic_dev_rrdy   <= 1'b0;
                    dc_dev_rrdy   <= 1'b0;

                    if (m_axi_rvalid) begin
                        // 存储每beat数据:
                        
                        
                        
                        if (r_src == SRC_DCACHE)
                            dc_rdata_buf[r_cnt*32 +: 32] <= m_axi_rdata;
                        else
                            ic_rdata_buf[r_cnt*32 +: 32] <= m_axi_rdata;

                        if (m_axi_rlast) begin
                            // 全部beat收齐, 下一周期在R_IDLE输出
                            // (非阻塞赋值延迟1周期)
                            r_data_done <= 1'b1;
                            r_cnt       <= 2'd0;
                        end else begin
                            r_cnt <= r_cnt + 2'd1;
                        end
                    end
                end

                default: begin
                    ic_dev_rrdy <= 1'b1;
                    dc_dev_rrdy <= 1'b1;
                end

            endcase
        end
    end


    //---------- 状态寄存器 (时序) ----------
    always @(posedge aclk or posedge areset) begin
        if (areset)
            w_state <= W_IDLE;
        else
            w_state <= w_nstat;
    end

    //---------- 下一状态逻辑 (组合) ----------
    always @(*) begin
        case (w_state)

            W_IDLE: begin
                // 写请求：转移发送地址
                if (dc_wen)
                    w_nstat = W_AR_SEND;
                else
                    w_nstat = W_IDLE;
            end

            W_AR_SEND: begin
                // 等待总线能够接收地址
                if (m_axi_awready)
                    w_nstat = W_DATA_SEND;
                else
                    w_nstat = W_AR_SEND;
            end

            W_DATA_SEND: begin
                // 等待总线能够写地址
                if (m_axi_wready)
                    w_nstat = W_RESP_WAIT;
                else
                    w_nstat = W_DATA_SEND;
            end

            W_RESP_WAIT: begin
                // 写完（每次只写一次）
                if (m_axi_bvalid)
                    w_nstat = W_IDLE;
                else
                    w_nstat = W_RESP_WAIT;
            end

            default: w_nstat = W_IDLE;
        endcase
    end

    //---------- 写FSM 阶段3: 输出逻辑 (时序) ----------
    always @(posedge aclk or posedge areset) begin
        if (areset) begin
            w_latched_addr <= 32'h0;
            w_latched_data <= 32'h0;
            w_latched_wen  <= 4'h0;

            dc_dev_wrdy   <= 1'b1;     // 就绪

            // AW/W/B空闲 (有效值在各状态中设置)
            m_axi_awaddr  <= 32'h0;
            m_axi_awlen   <= 8'd0;
            m_axi_awsize  <= 3'd0;
            m_axi_awburst <= 2'd0;
            m_axi_awvalid <= 1'b0;

            m_axi_wdata   <= 32'h0;
            m_axi_wstrb   <= 4'h0;
            m_axi_wvalid  <= 1'b0;

            m_axi_bready  <= 1'b1;
        end
        else begin
            case (w_state)

                //---------- W_IDLE: 等待写请求 ----------
                W_IDLE: begin
                    dc_dev_wrdy <= 1'b1;               // 写就绪
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b1;

                    if (dc_wen) begin
                        // 接收到请求：锁存地址
                        w_latched_addr <= dc_cpu_waddr;
                        w_latched_data <= dc_cpu_wdata;
                        w_latched_wen  <= dc_cpu_wen;  // =wstrb: 字节级掩码
                    end
                end

                //---------- W_AR_SEND: 驱动AXI写地址 ----------
                // (所有AW参数在此设置)
                W_AR_SEND: begin
                    m_axi_awaddr  <= w_latched_addr;
                    m_axi_awlen   <= 8'd0;             // 单拍写,与读不同
                    m_axi_awsize  <= 3'b010;           // 4字节/拍
                    m_axi_awburst <= 2'b01;            // INCR
                    m_axi_awvalid <= 1'b1;

                    dc_dev_wrdy   <= 1'b0;             // 阻止新请求
                end

                //---------- W_DATA_SEND: 驱动AXI写数据 ----------
                W_DATA_SEND: begin
                    m_axi_wdata  <= w_latched_data;
                    m_axi_wstrb  <= w_latched_wen;     // 字节/半字/字掩码
                    m_axi_wvalid <= 1'b1;              // 拉高有效位

                    m_axi_awvalid <= 1'b0;             // 握手完成
                    dc_dev_wrdy   <= 1'b0;
                end

                //---------- W_RESP_WAIT: 等待写响应 ----------
                W_RESP_WAIT: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;      // 拉低有效位，握手完成
                    m_axi_bready  <= 1'b1;      // 拉高ready，等待valid
                    dc_dev_wrdy   <= 1'b0;
                end

                default: begin
                    dc_dev_wrdy   <= 1'b1;
                end

            endcase
        end
    end

endmodule
