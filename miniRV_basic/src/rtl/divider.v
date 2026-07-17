`timescale 1ns / 1ps

module divider #(
    parameter WIDTH = 32
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [WIDTH-1:0] x,
    input  wire [WIDTH-1:0] y,
    input  wire       start,
    output reg [WIDTH-1:0] z,
    output reg [WIDTH-1:0] r,
    output wire       busy
);

    localparam IDLE    = 2'b00,
               COMPUTE = 2'b01,
               DONE    = 2'b10;

    reg [5:0] cnt;
    reg [1:0] state;
    reg [1:0] next_state;
    reg busy_sign;

    // 组合寄存器: {remainder, quotient}
    // remainder: WIDTH bits (余数幅值)
    // quotient:  WIDTH-1 bits (商幅值)
    reg [2*WIDTH-2:0] rem_quo;

    // 除数的绝对值 (WIDTH bits) —— 在 start 脉冲时锁存
    reg [WIDTH-1:0] divisor_r;
    reg         x_sign_r;     // 被除数符号
    reg         y_sign_r;     // 除数符号
    // ALU 已将输入编码为 {sign, magnitude}，除数幅值直接取低 WIDTH-1 位
    wire [WIDTH-1:0] divisor_mag = {1'b0, y[WIDTH-2:0]};

    // —— 下一周期的计算（纯组合逻辑）——
    // 左移后的 rem_quo
    wire [2*WIDTH-2:0] shifted = rem_quo << 1;
    // 移位后余数高 WIDTH 位减去除数（使用锁存的除数值）
    wire [WIDTH:0] rem_sub = {1'b0, shifted[2*WIDTH-2:WIDTH-1]} - {1'b0, divisor_r};
    // 够减? (rem_sub 最高位为 0 表示 >= 0)
    wire can_sub = !rem_sub[WIDTH];

    // 下一周期的 rem_quo 值
    wire [2*WIDTH-2:0] next_rem_quo = can_sub
        ? {rem_sub[WIDTH-1:0], shifted[WIDTH-2:1], 1'b1}   // 更新余数 + 商 LSB=1
        : {shifted[2*WIDTH-2:1], 1'b0};                      // 保持余数 + 商 LSB=0

    assign busy = busy_sign;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            cnt <= 0;
            z <= 0;
            r <= 0;
            busy_sign <= 0;
            rem_quo <= 0;
        end
        else begin
            state <= next_state;

            case (state)
                IDLE: if (start) begin
                    // 被除数幅值放入低位，高位填零
                    rem_quo <= {{(WIDTH-1){1'b0}}, x[WIDTH-2:0]};
                    // 锁存除数幅值和符号（避免后续周期 ALU 输入变化影响）
                    divisor_r <= divisor_mag;
                    x_sign_r <= x[WIDTH-1];
                    y_sign_r <= y[WIDTH-1];
                    cnt <= WIDTH - 1;
                    busy_sign <= 1;
                end else begin
                    busy_sign <= 0;
                end

                COMPUTE: begin
                    rem_quo <= next_rem_quo;
                    cnt <= cnt - 1;
                end

                DONE: begin
                    // 商的符号 = 被除数符号 ^ 除数符号（使用锁存值）
                    z <= (x_sign_r ^ y_sign_r) ?
                         (~{1'b0, rem_quo[WIDTH-2:0]} + 1'b1) :
                         {1'b0, rem_quo[WIDTH-2:0]};
                    // 余数的符号 = 被除数符号（使用锁存值）
                    r <= x_sign_r ?
                         (~rem_quo[2*WIDTH-2:WIDTH-1] + 1'b1) :
                         rem_quo[2*WIDTH-2:WIDTH-1];
                    busy_sign <= 0;
                end
            endcase
        end
    end

    always @(*) begin
        next_state = state;
        case (state)
            IDLE: if (start) next_state = COMPUTE;
            COMPUTE: if (cnt == 1) next_state = DONE;
            DONE: next_state = IDLE;
        endcase
    end

endmodule
