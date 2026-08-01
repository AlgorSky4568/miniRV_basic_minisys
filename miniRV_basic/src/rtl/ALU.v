`timescale 1ns / 1ps

`include "defines.vh"

module ALU (
    input  wire         rst,
    input  wire         clk,
    input  wire [ 4:0]  op,
    input  wire [31:0]  a,
    input  wire [31:0]  b,

    output reg  [31:0]  c,
    output reg          br,
    output wire         busy
);

    wire        mul_flag, mulu_flag;
    wire [63:0] mul_res , mulu_res ;
    wire        mul_busy, mulu_busy;
    wire        div_flag, divu_flag;
    wire [31:0] div_quo , divu_quo ;    // quotient
    wire [31:0] div_rem , divu_rem ;    // remainder
    wire        div_busy, divu_busy;
    reg  [ 4:0] op_r;

    always @(*) begin
        case (op_r != 5'h0 ? op_r : op)
            `ALU_ADD  : c = a + b;
            `ALU_SUB  : c = a - b;
            `ALU_OR   : c = a | b;
            `ALU_XOR  : c = a ^ b;
            `ALU_SLL  : c = a << b[4:0];
            `ALU_SRL  : c = a >> b[4:0];
            `ALU_SRA  : c = $signed(a) >>> b[4:0];
            `ALU_AND  : c = a & b;
            `ALU_SLT  : c = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            `ALU_SLTU : c = (a < b) ? 32'd1 : 32'd0;
            `ALU_MUL  : c = mul_res[31:0];
            `ALU_MULH : c = mul_res[63:32];
            `ALU_MULHU: c = mulu_res[63:32];
            `ALU_DIV  : c = div_quo;
            `ALU_DIVU : c = divu_quo;
            `ALU_REM  : c = div_rem;
            `ALU_REMU : c = divu_rem;
            default   : c = 32'h0;
        endcase
    end

    always @(*) begin
        case (op)
            `ALU_EQ  : br = a == b;
            `ALU_NE  : br = a != b;
            `ALU_LT  : br = $signed(a) < $signed(b);
            `ALU_LTU : br = a < b;
            `ALU_GE  : br = $signed(a) >= $signed(b);
            `ALU_GEU : br = a >= b;
            default  : br = 1'b0;
        endcase
    end

    assign mul_flag  = (op == `ALU_MUL || op == `ALU_MULH) ? 1'b1 : 1'b0;
    assign mulu_flag = (op == `ALU_MULHU) ? 1'b1 : 1'b0;
    assign div_flag  = (op == `ALU_DIV || op == `ALU_REM) ? 1'b1 : 1'b0;
    assign divu_flag = (op == `ALU_DIVU || op == `ALU_REMU) ? 1'b1 : 1'b0;
    assign busy      = mul_busy | mulu_busy | div_busy | divu_busy;

    // start 单拍脉冲：组合 flag 在指令驻留 EX 期间恒=1，若直接作为 start，
    // multiplier 完成（DONE→IDLE）后会因 start 仍=1 立即重启，busy 反复=1，
    // 导致 op_r 永不清除、后续指令的 alu_c 残留乘除结果。
    // 用 flag 上升沿产生单拍 start。
    reg mul_flag_r, mulu_flag_r, div_flag_r, divu_flag_r;
    always @(posedge clk) begin
        mul_flag_r  <= mul_flag;
        mulu_flag_r <= mulu_flag;
        div_flag_r  <= div_flag;
        divu_flag_r <= divu_flag;
    end
    wire mul_start  = mul_flag  & !mul_flag_r;
    wire mulu_start = mulu_flag & !mulu_flag_r;
    wire div_start  = div_flag  & !div_flag_r;
    wire divu_start = divu_flag & !divu_flag_r;

    always @(posedge clk) begin
        if (mul_flag | mulu_flag | div_flag | divu_flag)
            op_r <= op;
        else if (!busy)
            op_r <= 5'h0;
    end

    multiplier #(32) U_mul (
        .clk    (clk),
        .rst    (rst),
        .x      (a),
        .y      (b),
        .start  (mul_start),
        .z      (mul_res),
        .busy   (mul_busy)
    );

    multiplier #(33) U_mulu (
        .clk    (clk),
        .rst    (rst),
        .x      ({1'b0, a}),
        .y      ({1'b0, b}),
        .start  (mulu_start),
        .z      (mulu_res),
        .busy   (mulu_busy)
    );

    divider #(32) U_div (
        .clk    (clk),
        .rst    (rst),
        .x      (a[31] ? {1'b1, ~a[30:0] + 31'h1} : a),
        .y      (b[31] ? {1'b1, ~b[30:0] + 31'h1} : b),
        .start  (div_start),
        .z      (div_quo),
        .r      (div_rem),
        .busy   (div_busy)
    );

    divider #(33) U_divu (
        .clk    (clk),
        .rst    (rst),
        .x      ({1'b0, a}),
        .y      ({1'b0, b}),
        .start  (divu_start),
        .z      (divu_quo),
        .r      (divu_rem),
        .busy   (divu_busy)
    );

endmodule
