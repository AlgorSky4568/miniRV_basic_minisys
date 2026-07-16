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
    output reg  [WIDTH-1:0] r,
    output wire        busy     
);

    localparam IDLE    = 2'b00,
            COMPUTE = 2'b01,
            DONE    = 2'b10;

    reg [5:0] cnt = 0;
    wire symbol = x[WIDTH-1] ^ y[WIDTH-1];
    wire [WIDTH-1:0] y1 = {1'b0,y[WIDTH-2:0]};
    wire [WIDTH-1:0] y2 = ~y1 + 1'b1;
    reg [WIDTH-2:0] result; //这是商
    reg [2*WIDTH - 2:0] partial;
    wire [2*WIDTH-2:0] y3 = {y1,{(WIDTH-1){1'b0}}};
    wire [2*WIDTH-2:0] y4 = {y2,{(WIDTH-1){1'b0}}};
    reg [1:0] state;
    reg [1:0] next_state;
    reg busy_sign;
    wire [2*WIDTH-2:0]next_partial1 = partial + y4;
    wire [2*WIDTH-2:0]next_partial2 = partial;

    assign busy = busy_sign;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            cnt <= 0;
            z <= 0;
            r <= 0;
            busy_sign <= 0;
            result <= 0;
            partial <= 0;
        end
        else begin
             state <= next_state;
            
            case (state)
                IDLE: if (start) begin
                    partial <= {{WIDTH{1'b0}},x[WIDTH-2:0]};
                    cnt <= WIDTH;
                    busy_sign <= 1;
                    result <= 0;
                end else begin
                    busy_sign <= 0;
                end
                
                COMPUTE: begin
                    if (cnt == 1) begin
                            if(next_partial1[2*WIDTH-2] == 1'b1)begin
                            partial <= next_partial2;
                        end
                        else begin
                            partial <= next_partial1;
                            result <= (result + 1'b1);

                        end
                    end
                    else begin
                        if(next_partial1[2*WIDTH - 2] == 1'b1)begin
                            partial <= next_partial2 << 1;
                            result  <= result << 1;
                        end
                        else begin
                            partial <= next_partial1 << 1;
                            result <= (result + 1'b1) << 1;

                        end
                        
                    end
                    cnt <= cnt - 1;
                    
                end
                
                DONE: begin
                    z <= {symbol, result};
                    r <= {x[WIDTH-1],partial[2*WIDTH-3:WIDTH-1]};
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

