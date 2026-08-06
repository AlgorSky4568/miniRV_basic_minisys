`timescale 1ns / 1ps

`include "defines.vh"

module DCache(
    input  wire         cpu_clk,
    input  wire         cpu_rst,        // 高电平有效
    //---------- CPU 接口 ----------
    input  wire [ 3:0]  data_ren,       // 读使能
    input  wire [31:0]  data_addr,      // 地址 (读写共享)
    output reg          data_valid,     // 读数据有效
    output reg  [31:0]  data_rdata,     // 读数据 (至CPU)
    input  wire [ 3:0]  data_wen,       // 写使能
    input  wire [31:0]  data_wdata,     // 写数据
    output reg          data_wresp,     // 写响应
    //---------- 写总线接口 ----------
    input  wire         dev_wrdy,       // 设备写就绪
    output reg  [ 3:0]  cpu_wen,        // 写使能 (至设备)
    output reg  [31:0]  cpu_waddr,      // 写地址
    output reg  [31:0]  cpu_wdata,      // 写数据
    //---------- 读总线接口 ----------
    input  wire         dev_rrdy,       // 设备读就绪
    output reg  [ 3:0]  cpu_ren,        // 读使能 (至设备)
    output reg  [31:0]  cpu_raddr,      // 读地址
    input  wire         dev_rvalid,     // 设备数据有效
    input  wire [127:0] dev_rdata       // 设备返回数据
);

    // 外设地址 (0xFFFF_xxxx) 绕过Cache
    wire uncached = (data_addr[31:16] == 16'hFFFF) & (data_ren != 4'h0 | data_wen != 4'h0) ? 1'b1 : 1'b0;

`ifdef ENABLE_DCACHE

    localparam CACHE_LINE_WIDTH = 1 + `DCACHE_TAG_WIDTH + 128;  // valid + tag + data
    localparam R_IDLE = 3'b000;
    localparam R_TAG_CHECK = 3'b001;
    localparam R_REFILL = 3'b010;
    localparam R_UC_REQ = 3'b011;
    localparam R_UC_WAIT = 3'b100;

    localparam W_IDLE = 3'b000;
    localparam W_TAG_CHECK = 3'b001;
    localparam W_RESP = 3'b010;
    localparam W_UC_REQ = 3'b011;
    localparam W_UC_RESP = 3'b100;

    (*mark_debug = "true"*) reg [2:0] r_state, r_nstat;
    (*mark_debug = "true"*) reg [2:0] w_state, w_nstat;
    reg [31:0] r_addr;      // 读地址锁存
    reg [3:0] ren_r;
    reg r_uncached;         // 读uncached标志
    reg [31:0] w_addr;      // 写地址锁存
    reg [3:0] wen_r;
    reg [31:0] data_r;      // 写数据锁存
    reg w_uncached;         // 写uncached标志
    reg [127:0] wr_cache_data; // 写Cache数据拼接
    reg [`DCACHE_LINE_COUNT-1:0] valid_bits;     // xsim BRAM初始化污染, 需寄存器存储
    reg [`DCACHE_LINE_COUNT*`DCACHE_TAG_WIDTH-1:0] tag_bits;

    wire[31:0]active_addr = (w_state != W_IDLE) ? w_addr : (r_state != R_IDLE) ? r_addr : data_addr;
    wire [`DCACHE_INDEX_WIDTH-1:0] cache_index = active_addr[`DCACHE_INDEX_WIDTH+3 : 4];
    wire [CACHE_LINE_WIDTH-1:0] cache_line_r;

    wire [`DCACHE_TAG_WIDTH-1:0] tag_from_cpu = (w_state != W_IDLE) ? w_addr[`DCACHE_TAG_WIDTH+`DCACHE_INDEX_WIDTH+3 : `DCACHE_INDEX_WIDTH+4] : r_addr[`DCACHE_TAG_WIDTH+`DCACHE_INDEX_WIDTH+3 : `DCACHE_INDEX_WIDTH+4];
    wire [1:0] offset = r_addr[3:2];
    wire       valid_bit      = valid_bits[cache_index];
    wire [`DCACHE_TAG_WIDTH-1:0] tag_from_cache = tag_bits[cache_index*`DCACHE_TAG_WIDTH +: `DCACHE_TAG_WIDTH];

    wire hit_r = (r_state == R_TAG_CHECK) && !r_uncached && valid_bit && (r_addr[`DCACHE_TAG_WIDTH+`DCACHE_INDEX_WIDTH+3 : `DCACHE_INDEX_WIDTH+4] == tag_from_cache);
    wire hit_w = (w_state == W_TAG_CHECK) && !w_uncached && valid_bit && (w_addr[`DCACHE_TAG_WIDTH+`DCACHE_INDEX_WIDTH+3 : `DCACHE_INDEX_WIDTH+4] == tag_from_cache);

    wire refill_we = (r_state == R_REFILL) && dev_rvalid;
    wire write_hit_we = (w_state == W_TAG_CHECK) && hit_w;
    wire cache_we = refill_we | write_hit_we;
    wire [CACHE_LINE_WIDTH-1:0] cache_line_w = refill_we ? {1'b1, r_addr[`DCACHE_TAG_WIDTH+`DCACHE_INDEX_WIDTH+3 : `DCACHE_INDEX_WIDTH+4], dev_rdata} : {1'b1, w_addr[`DCACHE_TAG_WIDTH+`DCACHE_INDEX_WIDTH+3 : `DCACHE_INDEX_WIDTH+4], wr_cache_data};

    wire wr_cached_resp = (w_state == W_RESP) && dev_wrdy && (cpu_wen == 4'h0);
    wire wr_uncached_resp = (w_state == W_UC_RESP) && dev_wrdy;

    // 时序优化（Bug 21）：CPU 侧读输出打拍 1 拍——截断 r_addr→hit 判定→BRAM
    // MUX→data_rdata 组合链（最差路径 DCache→CPU→NPC 的 Cache 段，37 级逻辑）。
    // 流水线 CPU 的 load 为多周期等待（ld_pending 等 daccess_rvalid），
    // 读数据晚 1 拍到达语义兼容（需 AXI Trace 回归验证）。
    reg         data_valid_c;
    reg  [31:0] data_rdata_c;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            data_valid <= 1'b0;
            data_rdata <= 32'h0;
        end else begin
            data_valid <= data_valid_c;
            data_rdata <= data_rdata_c;
        end
    end

    always @(*) begin
        data_valid_c = hit_r | ((r_state == R_REFILL) && dev_rvalid) | ((r_state == R_UC_WAIT) && dev_rvalid);
    end

    always @(*) begin
        if (r_state == R_REFILL) begin
            case (r_addr[3:2])
                2'b00:   data_rdata_c = dev_rdata[31:0];
                2'b01:   data_rdata_c = dev_rdata[63:32];
                2'b10:   data_rdata_c = dev_rdata[95:64];
                default: data_rdata_c = dev_rdata[127:96];
            endcase
        end else if (r_state == R_UC_WAIT) begin
            data_rdata_c = dev_rdata[31:0];
        end else begin
            case (offset)
                2'b00:   data_rdata_c = cache_line_r[31:0];
                2'b01:   data_rdata_c = cache_line_r[63:32];
                2'b10:   data_rdata_c = cache_line_r[95:64];
                default: data_rdata_c = cache_line_r[127:96];
            endcase
        end
    end

`ifdef RUN_TRACE
    // Trace框架(vsrc/ram.v)提供参数化行为模型, 无ena引脚, 需显式传参
    blk_mem_gen_1 #(.ADDR_BITS(`DCACHE_INDEX_WIDTH), .DATA_BITS(CACHE_LINE_WIDTH)) U_dsram (
`else
    // Vivado: blk_mem_gen_1 IP核 (C_HAS_ENA=0, 无ena引脚)
    blk_mem_gen_1 U_dsram (
`endif
        .clka   (cpu_clk),
        .wea    (cache_we),
        .addra  (cache_index),
        .dina   (cache_line_w),
        .douta  (cache_line_r)
    );

    //---------- Valid + Tag: 寄存器存储 (xsim BRAM初始化污染) ----------
    // 硬件上BRAM初始化为0, 可直接用cache_line_r
    // 时序优化：r_refill_done 用 dev_rvalid 直接判定（R_REFILL 态 dev_rvalid=1
    // ⟺ r_nstat==R_IDLE，转移条件即 dev_rvalid），去掉对 r_nstat 组合链的依赖
    wire r_refill_done = (r_state == R_REFILL) && dev_rvalid;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            valid_bits <= {`DCACHE_LINE_COUNT{1'b0}};
            tag_bits   <= {`DCACHE_LINE_COUNT*`DCACHE_TAG_WIDTH{1'b0}};
        end else if (r_refill_done) begin
            valid_bits[cache_index] <= 1'b1;
            tag_bits[cache_index*`DCACHE_TAG_WIDTH +: `DCACHE_TAG_WIDTH] <= r_addr[`DCACHE_TAG_WIDTH+`DCACHE_INDEX_WIDTH+3 : `DCACHE_INDEX_WIDTH+4];
        end
    end

    //---------- 读地址锁存 ----------
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            r_addr <= 32'h0;
            ren_r <= 4'h0;
            r_uncached <= 1'b0;
        end else begin
            if (r_state == R_IDLE && |data_ren) begin
                r_addr <= data_addr;
                ren_r <= data_ren;
                r_uncached <= uncached;
            end
        end
    end

    //---------- 读状态寄存器 ----------
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)
            r_state <= R_IDLE;
        else
            r_state <= r_nstat;
    end

    //---------- 读下一状态逻辑 ----------
    always @(*) begin
        case (r_state)
            R_IDLE: begin
                if (|data_ren) begin
                    if (uncached) begin
                        if (dev_rrdy)
                            r_nstat = R_UC_WAIT;
                        else
                            r_nstat = R_UC_REQ;
                    end else begin
                        r_nstat = R_TAG_CHECK;
                    end
                end else begin
                    r_nstat = R_IDLE;
                end
            end

            R_TAG_CHECK: begin
                if (hit_r)
                    r_nstat = R_IDLE;
                else if (dev_rrdy)
                    r_nstat = R_REFILL;
                else
                    r_nstat = R_TAG_CHECK;
            end

            R_REFILL: begin
                if (dev_rvalid)
                    r_nstat = R_IDLE;
                else
                    r_nstat = R_REFILL;
            end

            R_UC_REQ: begin
                if (dev_rrdy)
                    r_nstat = R_UC_WAIT;
                else
                    r_nstat = R_UC_REQ;
            end

            R_UC_WAIT: begin
                if (dev_rvalid)
                    r_nstat = R_IDLE;
                else
                    r_nstat = R_UC_WAIT;
            end

            default: begin
                r_nstat = R_IDLE;
            end
        endcase
    end

    //---------- 读总线控制 ----------
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            cpu_ren   <= 4'h0;
            cpu_raddr <= 32'h0;
        end else begin
            cpu_ren <= 4'h0;
            case (r_state)
                R_IDLE: begin
                    if (|data_ren && uncached && dev_rrdy) begin
                        cpu_ren   <= data_ren;
                        cpu_raddr <= data_addr;
                    end
                end
                R_TAG_CHECK: begin
                    if (!hit_r && dev_rrdy) begin
                        cpu_ren   <= 4'hF;
                        cpu_raddr <= {r_addr[31:4], 4'b0000};
                    end
                end
                R_UC_REQ: begin
                    if (dev_rrdy) begin
                        cpu_ren   <= ren_r;
                        cpu_raddr <= r_addr;
                    end
                end
            endcase
        end
    end

    //---------- 写状态寄存器 ----------
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)
            w_state <= W_IDLE;
        else
            w_state <= w_nstat;
    end

    //---------- 写地址/数据锁存 ----------
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            w_addr     <= 32'h0;
            wen_r      <= 4'h0;
            data_r     <= 32'h0;
            w_uncached <= 1'b0;
        end else begin
            if (w_state == W_IDLE && |data_wen) begin
                w_addr     <= data_addr;
                wen_r      <= data_wen;
                data_r     <= data_wdata;
                w_uncached <= uncached;
            end
        end
    end

    //---------- 写下一状态逻辑 ----------
    always @(*) begin
        case (w_state)
            W_IDLE: begin
                if (|data_wen) begin
                    if (uncached)
                        w_nstat = W_UC_REQ;
                    else
                        w_nstat = W_TAG_CHECK;
                end else begin
                    w_nstat = W_IDLE;
                end
            end

            W_TAG_CHECK: begin
                if (dev_wrdy)
                    w_nstat = W_RESP;
                else
                    w_nstat = W_TAG_CHECK;
            end

            W_RESP: begin
                if (wr_cached_resp)
                    w_nstat = W_IDLE;
                else
                    w_nstat = W_RESP;
            end

            W_UC_REQ: begin
                if (dev_wrdy)
                    w_nstat = W_UC_RESP;
                else
                    w_nstat = W_UC_REQ;
            end

            W_UC_RESP: begin
                if (wr_uncached_resp)
                    w_nstat = W_IDLE;
                else
                    w_nstat = W_UC_RESP;
            end

            default: begin
                w_nstat = W_IDLE;
            end
        endcase
    end

    //---------- 写总线控制 ----------
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            data_wresp <= 1'b0;
            cpu_wen    <= 4'h0;
            cpu_waddr  <= 32'h0;
            cpu_wdata  <= 32'h0;
        end else begin
            data_wresp <= 1'b0;
            cpu_wen    <= 4'h0;
            case (w_state)
                W_IDLE: begin
                end
                W_TAG_CHECK: begin
                    cpu_wen   <= wen_r;
                    cpu_waddr <= w_addr;
                    cpu_wdata <= data_r;
                end
                W_RESP: begin
                    if (wr_cached_resp)
                        data_wresp <= 1'b1;
                end
                W_UC_REQ: begin
                    cpu_wen   <= wen_r;
                    cpu_waddr <= w_addr;
                    cpu_wdata <= data_r;
                end
                W_UC_RESP: begin
                    if (wr_uncached_resp)
                        data_wresp <= 1'b1;
                end
            endcase
        end
    end

    always @(*) begin
        wr_cache_data = cache_line_r[127:0];
        case (w_addr[3:2])
            2'b00: begin
                if (wen_r[0]) wr_cache_data[7:0]    = data_r[7:0];
                if (wen_r[1]) wr_cache_data[15:8]   = data_r[15:8];
                if (wen_r[2]) wr_cache_data[23:16]  = data_r[23:16];
                if (wen_r[3]) wr_cache_data[31:24]  = data_r[31:24];
            end
            2'b01: begin
                if (wen_r[0]) wr_cache_data[39:32]  = data_r[7:0];
                if (wen_r[1]) wr_cache_data[47:40]  = data_r[15:8];
                if (wen_r[2]) wr_cache_data[55:48]  = data_r[23:16];
                if (wen_r[3]) wr_cache_data[63:56]  = data_r[31:24];
            end
            2'b10: begin
                if (wen_r[0]) wr_cache_data[71:64]  = data_r[7:0];
                if (wen_r[1]) wr_cache_data[79:72]  = data_r[15:8];
                if (wen_r[2]) wr_cache_data[87:80]  = data_r[23:16];
                if (wen_r[3]) wr_cache_data[95:88]  = data_r[31:24];
            end
            2'b11: begin
                if (wen_r[0]) wr_cache_data[103:96] = data_r[7:0];
                if (wen_r[1]) wr_cache_data[111:104]= data_r[15:8];
                if (wen_r[2]) wr_cache_data[119:112]= data_r[23:16];
                if (wen_r[3]) wr_cache_data[127:120]= data_r[31:24];
            end
        endcase
    end

`else
    //---------- 直通模式 (无Cache) ----------
    localparam R_IDLE  = 2'b00;
    localparam R_STAT0 = 2'b01;
    localparam R_STAT1 = 2'b11;
    reg [1:0] r_state, r_nstat;
    reg [3:0] ren_r;

    always @(posedge cpu_clk or posedge cpu_rst) begin
        r_state <= cpu_rst ? R_IDLE : r_nstat;
    end

    always @(*) begin
        case (r_state)
            R_IDLE:  r_nstat = (|data_ren) ? (dev_rrdy ? R_STAT1 : R_STAT0) : R_IDLE;
            R_STAT0: r_nstat = dev_rrdy ? R_STAT1 : R_STAT0;
            R_STAT1: r_nstat = dev_rvalid ? R_IDLE : R_STAT1;
            default: r_nstat = R_IDLE;
        endcase
    end

    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            data_valid <= 1'b0;
            cpu_ren    <= 4'h0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    data_valid <= 1'b0;
                    if (|data_ren) begin
                        if (dev_rrdy)
                            cpu_ren <= data_ren;
                        else
                            ren_r   <= data_ren;
                        cpu_raddr <= data_addr;
                    end else
                        cpu_ren   <= 4'h0;
                end
                R_STAT0: begin
                    cpu_ren    <= dev_rrdy ? ren_r : 4'h0;
                end
                R_STAT1: begin
                    cpu_ren    <= 4'h0;
                    data_valid <= dev_rvalid ? 1'b1 : 1'b0;
                    data_rdata <= dev_rvalid ? dev_rdata : 32'h0;
                end
                default: begin
                    data_valid <= 1'b0;
                    cpu_ren    <= 4'h0;
                end
            endcase
        end
    end

    localparam W_IDLE  = 2'b00;
    localparam W_STAT0 = 2'b01;
    localparam W_STAT1 = 2'b11;
    reg  [1:0] w_state, w_nstat;
    reg  [3:0] wen_r;
    wire       wr_resp = dev_wrdy & (cpu_wen == 4'h0) ? 1'b1 : 1'b0;

    always @(posedge cpu_clk or posedge cpu_rst) begin
        w_state <= cpu_rst ? W_IDLE : w_nstat;
    end

    always @(*) begin
        case (w_state)
            W_IDLE:  w_nstat = (|data_wen) ? (dev_wrdy ? W_STAT1 : W_STAT0) : W_IDLE;
            W_STAT0: w_nstat = dev_wrdy ? W_STAT1 : W_STAT0;
            W_STAT1: w_nstat = wr_resp ? W_IDLE : W_STAT1;
            default: w_nstat = W_IDLE;
        endcase
    end

    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            data_wresp <= 1'b0;
            cpu_wen    <= 4'h0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    data_wresp <= 1'b0;
                    if (|data_wen) begin
                        if (dev_wrdy)
                            cpu_wen <= data_wen;
                        else
                            wen_r   <= data_wen;
                        cpu_waddr  <= data_addr;
                        cpu_wdata  <= data_wdata;
                    end else
                        cpu_wen    <= 4'h0;
                end
                W_STAT0: begin
                    cpu_wen    <= dev_wrdy ? wen_r : 4'h0;
                end
                W_STAT1: begin
                    cpu_wen    <= 4'h0;
                    data_wresp <= wr_resp ? 1'b1 : 1'b0;
                end
                default: begin
                    data_wresp <= 1'b0;
                    cpu_wen    <= 4'h0;
                end
            endcase
        end
    end
`endif

endmodule
