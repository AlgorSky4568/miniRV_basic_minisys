`timescale 1ns / 1ps



`include "defines.vh"



module MREQ (

    input  wire [31:0]  ram_addr,



    input  wire [ 2:0]  ram_rop,

    output reg  [ 3:0]  da_ren,

    output wire [31:0]  da_addr,



    input  wire [ 3:0]  ram_wop,

    input  wire [31:0]  ram_wdata,

    output reg  [ 3:0]  da_wen,

    output reg  [31:0]  da_wdata

);



    wire [1:0] offset = ram_addr[1:0];



    assign da_addr = ram_addr;



    // 产生写访存请求（da_wen、da_wdata）

    always @(*) begin

        // default value

        da_wen   = 4'h0;    // da_wen表示哪个字节修改了

        da_wdata = ram_wdata;   // 只用管da_wen当中给对应的字段



        case (ram_wop)
            `RAM_WE_B: begin                            // sb
                case (offset)
                    2'b00: begin
                        da_wen   = 4'b0001;
                        // da_wdata = ram_wdata;         // 默认已初始化，字节0不变
                    end
                    2'b01: begin
                        da_wen   = 4'b0010;
                        da_wdata = {ram_wdata[31:16], ram_wdata[7:0], ram_wdata[7:0]}; // 仅[15:8]改为rs2[7:0]
                    end
                    2'b10: begin
                        da_wen   = 4'b0100;
                        da_wdata = {ram_wdata[31:24], ram_wdata[7:0], ram_wdata[15:0]}; // 仅[23:16]改为rs2[7:0]
                    end
                    2'b11: begin
                        da_wen   = 4'b1000;
                        da_wdata = {ram_wdata[7:0], ram_wdata[23:0]};                    // 仅[31:24]改为rs2[7:0]
                    end
                    default: da_wen = 4'h0;
                endcase
            end

            `RAM_WE_H: begin                            // sh
                if (offset[0] == 1'b0) begin            // 半字对齐检查
                    if (offset[1] == 1'b0) begin
                        da_wen   = 4'b0011;             // 低2字节
                        // da_wdata = ram_wdata;         // 默认已初始化，[15:0]不变
                    end else begin
                        da_wen   = 4'b1100;             // 高2字节
                        da_wdata = {ram_wdata[15:0], ram_wdata[15:0]}; // 仅[31:16]改为rs2[15:0]
                    end
                end else begin
                    da_wen = 4'h0;                      // 地址不对齐，不写入
                end
            end

            `RAM_WE_W:                                  // sw

                if (offset == 2'h0) begin

                    da_wen   = ram_wop; //4'b1111
                    da_wdata = ram_wdata;

                end else begin
                    da_wen = 4'b0000;
                end
            default: begin
                da_wen = 4'b0000;
            end
        endcase

    end



    // 产生读访存请求（da_ren）

    always @(*) begin

        if (ram_rop != `RAM_EXT_N) begin

            case (ram_rop)

                // TODO: 根据访存指令类型，判断偏移量offset是否满足对齐条件（字节对齐、半字对齐），

                //       只有在对齐时才能访存
                // 字节访问：默认对齐
                `RAM_EXT_B : da_ren = 4'hF; 
                `RAM_EXT_BU : da_ren = 4'hF;
                // 半字访问：offset必须是2的倍数
                `RAM_EXT_H  : da_ren = (offset[0] == 1'b0) ? 4'hF : 4'h0;
                `RAM_EXT_HU : da_ren = (offset[0] == 1'b0) ? 4'hF : 4'h0;
                default    : da_ren = (offset == 2'h0) ? 4'hF : 4'h0;                       // lw

            endcase

        end else da_ren = 4'h0;
    end



endmodule

