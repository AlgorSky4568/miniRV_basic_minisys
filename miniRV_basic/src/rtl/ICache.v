`timescale 1ns / 1ps

`include "defines.vh"

// 主存 256KB, Cache 4KB, Block 128bit (4×32bit), `ICACHE_LINE_COUNT lines

module ICache(
    input  wire         cpu_clk,
    input  wire         cpu_rst,        // 高电平有效
    //---------- CPU接口 ----------
    input  wire         inst_rreq,      // 取指请求
    input  wire [31:0]  inst_addr,      // 取指地址
    output reg          inst_valid,     // 指令有效
    output reg  [31:0]  inst_out,       // 指令输出
    //---------- 读总线接口 ----------
    input  wire         dev_rrdy,       // 设备读就绪
    output reg  [ 3:0]  cpu_ren,        // 读使能 (至设备)
    output reg  [31:0]  cpu_raddr,      // 读地址
    input  wire         dev_rvalid,     // 设备数据有效
    input  wire [127:0] dev_rdata       // 设备返回数据
);

    function [31:0] pick_word;
        input [127:0] line_data;
        input [  1:0] word_offset;
        begin
            case (word_offset)
                2'b00:   pick_word = line_data[31:0];
                2'b01:   pick_word = line_data[63:32];
                2'b10:   pick_word = line_data[95:64];
                default: pick_word = line_data[127:96];
            endcase
        end
    endfunction

`ifdef ENABLE_ICACHE    /******** Do not modify this line ********/

    localparam CACHE_LINE_WIDTH = 1 + `ICACHE_TAG_WIDTH + 128;  // valid + tag + data
    localparam IDLE      = 2'b00;
    localparam TAG_CHECK = 2'b01;
    localparam REFILL    = 2'b10;

    (*mark_debug = "true"*) reg [1:0] state, nstat;
    reg [31:0] req_addr_r;
    reg [`ICACHE_LINE_COUNT-1:0] valid_bits;
    reg [`ICACHE_LINE_COUNT*`ICACHE_TAG_WIDTH-1:0] tag_bits;  // xsim BRAM初始化污染, 需寄存器存储 tag/valid

    wire [`ICACHE_TAG_WIDTH-1:0] tag_from_cpu   = req_addr_r[`ICACHE_TAG_WIDTH+`ICACHE_INDEX_WIDTH+3 : `ICACHE_INDEX_WIDTH+4];
    wire [1:0] offset         = req_addr_r[3:2];
    wire [CACHE_LINE_WIDTH-1:0] cache_line_r;
    wire [`ICACHE_INDEX_WIDTH-1:0] cache_index    = (state == IDLE) ? inst_addr[`ICACHE_INDEX_WIDTH+3 : 4] : req_addr_r[`ICACHE_INDEX_WIDTH+3 : 4];
    wire       valid_bit      = valid_bits[cache_index];
    wire [`ICACHE_TAG_WIDTH-1:0] tag_from_cache = tag_bits[cache_index*`ICACHE_TAG_WIDTH +: `ICACHE_TAG_WIDTH];

    // 时序优化（Bug 23）：hit 预计算——T 拍（IDLE && inst_rreq）用 inst_addr
    // 组合计算并锁存 hit_pre（与 req_addr_r 锁存同拍同值），T+1 拍 TAG_CHECK
    // 直接用寄存器 hit_pre 判定。等价变换：inst_valid 输出拍不变（T+1），
    // 但 nstat 组合链（原含 8bit tag 比较 6 级 LUT）被寄存器截断——CPU→ICache
    // 组合环（need_redirect→ifetch_req→nstat→inst_valid）缩短 ~2-3ns。
    wire [`ICACHE_INDEX_WIDTH-1:0] idx_pre   = inst_addr[`ICACHE_INDEX_WIDTH+3 : 4];
    wire hit_pre_c = valid_bits[idx_pre] &
                     (inst_addr[`ICACHE_TAG_WIDTH+`ICACHE_INDEX_WIDTH+3 : `ICACHE_INDEX_WIDTH+4]
                      == tag_bits[idx_pre*`ICACHE_TAG_WIDTH +: `ICACHE_TAG_WIDTH]);
    reg  hit_pre;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)
            hit_pre <= 1'b0;
        else if (state == IDLE && inst_rreq)
            hit_pre <= hit_pre_c;
    end

    wire hit_cur = (state == TAG_CHECK) && hit_pre;

    always @(*) begin
        inst_valid = hit_cur | ((state == REFILL) && dev_rvalid);
    end

    always @(*) begin
        case (state)
            REFILL: begin
                case (offset)
                    2'b00:   inst_out = dev_rdata[31:0];
                    2'b01:   inst_out = dev_rdata[63:32];
                    2'b10:   inst_out = dev_rdata[95:64];
                    default: inst_out = dev_rdata[127:96];
                endcase
            end
            default: begin
                case (offset)
                    2'b00:   inst_out = cache_line_r[31:0];
                    2'b01:   inst_out = cache_line_r[63:32];
                    2'b10:   inst_out = cache_line_r[95:64];
                    default: inst_out = cache_line_r[127:96];
                endcase
            end
        endcase
    end

    wire       cache_we     = (state == REFILL) && dev_rvalid;
    wire [CACHE_LINE_WIDTH-1:0] cache_line_w = {1'b1, tag_from_cpu, dev_rdata};

`ifdef RUN_TRACE
    // Trace框架(vsrc/ram.v)提供参数化行为模型, 无ena引脚, 需显式传参
    blk_mem_gen_1 #(.ADDR_BITS(`ICACHE_INDEX_WIDTH), .DATA_BITS(CACHE_LINE_WIDTH)) U_isram (
`else
    // Vivado: blk_mem_gen_1 IP核 (C_HAS_ENA=0, 无ena引脚)
    blk_mem_gen_1 U_isram (
`endif
        .clka   (cpu_clk),
        .wea    (cache_we),
        .addra  (cache_index),
        .dina   (cache_line_w),
        .douta  (cache_line_r)
    );

    //---------- Valid + Tag: 寄存器存储 (xsim BRAM初始化污染) ----------
    // 硬件上BRAM初始化为0, 可直接用cache_line_r[133]/[132:128]
    // 时序优化：refill_done 用 dev_rvalid 直接判定（REFILL 态 dev_rvalid=1 ⟺
    // nstat==IDLE，转移条件即 dev_rvalid），去掉对 nstat（组合链 → ifetch_req
    // → need_redirect 跨模块长路径）的依赖，缩短 tag_bits CE 关键路径
    wire refill_done = (state == REFILL) && dev_rvalid;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            valid_bits <= {`ICACHE_LINE_COUNT{1'b0}};
            tag_bits   <= {`ICACHE_LINE_COUNT*`ICACHE_TAG_WIDTH{1'b0}};
        end else if (refill_done) begin
            valid_bits[cache_index] <= 1'b1;
            tag_bits[cache_index*`ICACHE_TAG_WIDTH +: `ICACHE_TAG_WIDTH] <= tag_from_cpu;
        end
    end

    //---------- 地址锁存 ----------
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            req_addr_r <= 32'h0;
        end else begin
            if (state == IDLE && inst_rreq)
                req_addr_r <= inst_addr;
        end
    end

    //---------- 状态寄存器 ----------
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            state <= IDLE;
        end else begin
            state <= nstat;
        end
    end

    //---------- 下一状态逻辑 (指导书图2-4, BRAM WRITE_FIRST模式) ----------
    always @(*) begin
        case (state)
            IDLE: begin
                if (inst_rreq)
                    nstat = TAG_CHECK;
                else
                    nstat = IDLE;
            end
            TAG_CHECK: begin
                if (hit_pre)          // 预计算锁存值（T 拍 inst_addr 计算）
                    nstat = IDLE;
                else if (dev_rrdy)
                    nstat = REFILL;
                else
                    nstat = TAG_CHECK;
            end
            REFILL: begin
                if (dev_rvalid)
                    nstat = IDLE;
                else
                    nstat = REFILL;
            end
            default: begin
                nstat = IDLE;
            end
        endcase
    end

    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            cpu_ren   <= 4'h0;
            cpu_raddr <= 32'h0;
        end else begin
            if (state == TAG_CHECK && !hit_pre && dev_rrdy) begin
                cpu_ren   <= 4'hF;
                cpu_raddr <= {req_addr_r[31:4], 4'b0000};
            end
            else begin
                cpu_ren <= 4'h0;
            end
        end
    end

    /******** Do not modify the following code ********/
`else

    localparam IDLE  = 2'b00;
    localparam STAT0 = 2'b01;
    localparam POST  = 2'b10;
    localparam STAT1 = 2'b11;
    reg [1:0] state, nstat;
    reg post_wait;   // POST 第二拍（返回后共停 2 拍，等 CPU 更新 pc）

    always @(posedge cpu_clk or posedge cpu_rst) begin
        state <= cpu_rst ? IDLE : nstat;
    end

    // 流水线 SoC 集成（阶段 3a）：inst_rreq（ifetch_req）在流水线 CPU 下
    // 为电平信号（无访存/乘除时恒 1）——原直通状态机返回后回 IDLE 立即
    // 重新锁存 inst_addr，而此时 CPU 的 pc 尚未更新（Verilator 同沿语义：
    // 锁存读 pc 旧值），导致同一地址被请求 2 次、总线返回 2 遍同一指令、
    // IF/ID 重复捕获（VCD 实测：请求 0 返回 jal 两次）。新增 POST 空闲态
    // （停 2 拍）：返回（dev_rvalid）后 CPU 捕获拍（返回拍+1）沿前
    // inst_finished 判定仍失败（id_pc 尚未更新为 fetch_pc，PC 在返回拍+2
    // 才前进），锁存拍须推迟到 PC 更新之后才能锁到新地址。
    // req_stable（请求持续 2 拍）：乘除/访存完成拍（mul_done_pulse /
    // ld_st_done）ifetch_req 从 0 拉高，CPU 下一拍才看到 inst_finished 并
    // 更新 PC——若立即锁存（锁存拍=完成+1）与 PC 更新同沿竞争，锁存读
    // 旧 pc（乘除自身地址 0x10），总线返回乘除副本（mul 测试 VCD 实测）。
    // 请求持续 2 拍后再锁存，PC 已更新，锁到乘除后继地址（0x14）。
    reg req_r1, req_r2;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            req_r1 <= 1'b0;
            req_r2 <= 1'b0;
        end else begin
            req_r1 <= inst_rreq;
            req_r2 <= req_r1;
        end
    end
    wire req_stable = inst_rreq & req_r1 & req_r2;

    always @(*) begin
        case (state)
            IDLE:    nstat = (inst_rreq & req_stable) ? (dev_rrdy ? STAT1 : STAT0) : IDLE;
            STAT0:   nstat = dev_rrdy ? STAT1 : STAT0;
            STAT1:   nstat = dev_rvalid ? POST : STAT1;
            POST:    nstat = post_wait ? IDLE : POST;
            default: nstat = IDLE;
        endcase
    end

    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) post_wait <= 1'b0;
        else if (state == POST) post_wait <= ~post_wait;
        else post_wait <= 1'b0;
    end

    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            inst_valid <= 1'b0;
            cpu_ren    <= 4'h0;
        end else begin
            case (state)
                IDLE: begin
                    inst_valid <= 1'b0;
                    if (inst_rreq & req_stable) begin
                        cpu_ren    <= dev_rrdy ? 4'hF : 4'h0;
                        cpu_raddr  <= inst_addr;
                    end else begin
                        cpu_ren    <= 4'h0;
                        cpu_raddr  <= 32'h0;
                    end
                end
                STAT0: begin
                    cpu_ren    <= dev_rrdy ? 4'hF : 4'h0;
                end
                STAT1: begin
                    cpu_ren    <= 4'h0;
                    inst_valid <= dev_rvalid ? 1'b1 : 1'b0;
                    inst_out   <= dev_rvalid ? dev_rdata[31:0] : 32'h0;
                end
                default: begin
                    inst_valid <= 1'b0;
                    cpu_ren    <= 4'h0;
                end
            endcase
        end
    end

`endif

endmodule
