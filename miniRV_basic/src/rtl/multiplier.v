`timescale 1ns / 1ps
`include "defines.vh"

module multiplier #(
    parameter WIDTH = 32,
    parameter O_WID = 2 * WIDTH
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [WIDTH-1:0] x,
    input  wire [WIDTH-1:0] y,
    input  wire        start,
    output reg  [O_WID-1:0] z,
    output wire        busy
);

    localparam IDLE    = 2'b00,
               COMPUTE = 2'b01,
               DONE    = 2'b10;

    reg [1:0] state, next_state;
    reg [WIDTH:0] partial;
    reg [WIDTH:0] multiplier_reg;
    reg [WIDTH-1:0] multiplicand_reg;
    reg [WIDTH-1:0] neg_multiplicand;
    reg [5:0] cnt;

    reg busy_sign;
    assign busy = busy_sign;

    wire [1:0] op = multiplier_reg[1:0];

    wire [WIDTH:0] add_term;
    assign add_term = (op == 2'b01) ? {multiplicand_reg[WIDTH-1], multiplicand_reg} :
                      (op == 2'b10) ? {neg_multiplicand[WIDTH-1], neg_multiplicand} :
                      {(WIDTH+1){1'b0}};

    wire [WIDTH:0] sum = partial + add_term;
    wire [2*WIDTH+1:0] shift_reg = {{sum[WIDTH], sum}, multiplier_reg[WIDTH:1]};

    wire [WIDTH:0] next_partial = shift_reg[2*WIDTH+1:WIDTH+1];
    wire [WIDTH:0] next_multiplier = shift_reg[WIDTH:0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            partial <= 0;
            multiplier_reg <= 0;
            multiplicand_reg <= 0;
            neg_multiplicand <= 0;
            cnt <= 0;
            z <= 0;
            busy_sign <= 0;
        end else begin
            state <= next_state;

            case (state)
                IDLE: if (start) begin
                    partial <= 0;
                    multiplier_reg <= {y, 1'b0};
                    multiplicand_reg <= x;
                    neg_multiplicand <= ~x + 1'b1;
                    cnt <= WIDTH;
                    busy_sign <= 1;
                end else begin
                    busy_sign <= 0;
                end

                COMPUTE: begin
                    partial <= next_partial;
                    multiplier_reg <= next_multiplier;
                    cnt <= cnt - 1;
                end

                DONE: begin
                    z <= {partial[WIDTH-1:0], multiplier_reg[WIDTH:1]};
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
