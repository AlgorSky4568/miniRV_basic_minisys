`timescale 1ns / 1ps

`include "defines.vh"

module cpu_top(
    input  wire         cpu_clk,        // 接sys_clk / sys_rst
    input  wire         cpu_rst        // high active
);

    wire        cpu2ic_rreq;
    wire [31:0] cpu2ic_addr;
    wire        ic2cpu_valid;
    wire [31:0] ic2cpu_inst;

    wire [ 3:0] cpu2dc_ren;
    wire [31:0] cpu2dc_addr;
    wire        dc2cpu_valid;
    wire [31:0] dc2cpu_rdata;
    wire [ 3:0] cpu2dc_wen;
    wire [31:0] cpu2dc_wdata;
    wire        dc2cpu_wresp;

    cpu_core U_core (
        .cpu_clk        (cpu_clk),
        .cpu_rst        (cpu_rst),
        // Instruction Fetch Interface
        .ifetch_req     (cpu2ic_rreq),
        .ifetch_addr    (cpu2ic_addr),
        .ifetch_valid   (ic2cpu_valid),
        .ifetch_inst    (ic2cpu_inst),
        // Data Access Interface
        .daccess_ren    (cpu2dc_ren),
        .daccess_addr   (cpu2dc_addr),
        .daccess_rvalid (dc2cpu_valid),
        .daccess_rdata  (dc2cpu_rdata),
        .daccess_wen    (cpu2dc_wen),
        .daccess_wdata  (cpu2dc_wdata),
        .daccess_wresp  (dc2cpu_wresp)
    );

    Inst_ROM U_irom (
        .cpu_clk        (cpu_clk),
        .cpu_rst        (cpu_rst),
        // Interface to CPU
        .inst_rreq      (cpu2ic_rreq),  //输入：取指请求
        .inst_addr      (cpu2ic_addr),  //输入：地址
        .inst_valid     (ic2cpu_valid), //输出：取值是否有效
        .inst_out       (ic2cpu_inst)   //输出：取值
    );

    Data_RAM U_dram (
        .cpu_clk        (cpu_clk),
        .cpu_rst        (cpu_rst),
        // Interface to CPU
        .data_ren       (cpu2dc_ren),       //读使能
        .data_addr      (cpu2dc_addr),      //读地址
        .data_valid     (dc2cpu_valid),     //读有效
        .data_rdata     (dc2cpu_rdata),     //读数据
        .data_wen       (cpu2dc_wen),       //写使能
        .data_wdata     (cpu2dc_wdata),     //写数据
        .data_wresp     (dc2cpu_wresp)      //写请求
    );

endmodule
