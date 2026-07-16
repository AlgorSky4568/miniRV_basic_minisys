`timescale 1ns / 1ps

`include "defines.vh"

module Controller (
    input  wire [ 6:0]  opcode,
    input  wire [ 2:0]  funct3,
    input  wire [ 6:0]  funct7,
    output wire [ 1:0]  npc_op,
    output wire [ 2:0]  sext_op,
    output wire         alua_sel,
    output wire         alub_sel,
    output wire [ 4:0]  alu_op,
    output wire         is_mul,
    output wire         is_div,
    output wire [ 2:0]  ram_r_op,
    output wire [ 3:0]  ram_w_op,
    output wire         rf_we,
    output wire [ 1:0]  rf_wsel
);

    //这一段表示确认哪个操作
    wire ADDI  = (opcode == 7'b0010011) && (funct3 == 3'b000);
    wire ORI   = (opcode == 7'b0010011) && (funct3 == 3'b110);
    wire SLLI  = (opcode == 7'b0010011) && (funct3 == 3'b001) && (funct7 == 7'b0000000);
    wire LW    = (opcode == 7'b0000011) && (funct3 == 3'b010);
    wire BEQ   = (opcode == 7'b1100011) && (funct3 == 3'b000);
    wire BNE   = (opcode == 7'b1100011) && (funct3 == 3'b001);
    wire LUI   = (opcode == 7'b0110111);
    wire JAL   = (opcode == 7'b1101111);
    wire MUL   = (opcode == 7'b0110011) && (funct3 == 3'b000) && (funct7 == 7'b0000001);
    wire MULH  = (opcode == 7'b0110011) && (funct3 == 3'b001) && (funct7 == 7'b0000001);
    wire MULHU = (opcode == 7'b0110011) && (funct3 == 3'b011) && (funct7 == 7'b0000001);
    wire DIV = (opcode == 7'b0110011) && (funct3 == 3'b100) && (funct7 == 7'b0000001);
    wire DIVU = (opcode == 7'b0110011) && (funct3 == 3'b101) && (funct7 == 7'b0000001);
    wire REM = (opcode == 7'b0110011) && (funct3 == 3'b110) && (funct7 == 7'b0000001);
    wire REMU = (opcode == 7'b0110011) && (funct3 == 3'b111) && (funct7 == 7'b0000001);
    wire SLT = (opcode == 7'b0110011) && (funct3 == 3'b010) && (funct7 == 7'b0000000);
    wire SLTI = (opcode == 7'b0010011) && (funct3 == 3'b010);
    wire SLTU = (opcode == 7'b0110011) && (funct3 == 3'b011) && (funct7 == 7'b0000000);
    wire SLTIU = (opcode == 7'b0010011) && (funct3 == 3'b011);
    wire OR = (opcode == 7'b0110011) && (funct3 == 3'b110) && (funct7 == 7'b0000000);
    wire AND = (opcode == 7'b0110011) && (funct3 == 3'b111) && (funct7 == 7'b0000000);
    wire ANDI = (opcode == 7'b0010011) && (funct3 == 3'b000);
    wire BLT = (opcode == 7'b1100011) && (funct3 == 3'b100);
    wire BGE = (opcode == 7'b1100011) && (funct3 == 3'b101);
    wire BLTU = (opcode == 7'b1100011) && (funct3 == 3'b110);
    wire BGEU = (opcode == 7'b1100011) && (funct3 == 3'b111);


 
    // npc_op，表示下一个指令是PC+4还是跳转
    wire NPC_OP_BRA = BEQ | BNE | BLT | BGE | BLTU | BGEU;
    wire NPC_OP_JMP = JAL;
    wire NPC_OP_PC4 = !NPC_OP_BRA & !NPC_OP_JMP;
    
    // rf_we，表示是否使用立即数
    wire RF_OP_WE = ADDI | ORI | SLLI | LW | LUI | JAL | SLTI | SLTIU | BLT | BGE | BLTU | BGEU | ANDI;
    
    // rf_wsel，表示写回的数据来自哪里
    wire WB_OP_ALU = ADDI | ORI | SLLI| MUL | MULH | MULHU | DIV | DIVU | REM | REMU | SLT | SLTI | SLTU | SLTIU | AND | ANDI | OR;
    wire WB_OP_RAM = LW;
    wire WB_OP_PC4 = JAL;
    wire WB_OP_EXT = LUI;
    
    // sext_op，表示该用哪种立即数
    wire EXT_OP_I = ADDI | ORI | SLLI | LW | SLTI | SLTIU | ANDI;
    wire EXT_OP_B = BEQ | BNE | BLT | BGE | BLTU | BGEU;
    wire EXT_OP_U = LUI;
    wire EXT_OP_J = JAL;
    
    // alu_op，表示ALU该进行哪种操作
    wire ALU_OP_ADD   = ADDI | LW;
    wire ALU_OP_OR    = ORI | OR;
    wire ALU_OP_SLL   = SLLI;
    wire ALU_OP_EQ    = BEQ;
    wire ALU_OP_NE    = BNE;
    wire ALU_OP_MUL   = MUL;
    wire ALU_OP_MULH    = MULH;
    wire ALU_OP_MULHU    = MULHU;
    wire ALU_OP_DIV = DIV;
    wire ALU_OP_DIVU = DIVU;
    wire ALU_OP_REM = REM;
    wire ALU_OP_REMU = REMU;
    wire ALU_OP_SLT = SLT | SLTI;
    wire ALU_OP_SLTU = SLTU | SLTIU;
    wire ALU_OP_AND = AND | ANDI;
    wire ALU_OP_LT = BLT;
    wire ALU_OP_GE = BGE;
    wire ALU_OP_LTU = BLTU;
    wire ALU_OP_GEU = BGEU;
    
    // alua_sel
    wire ALU_A_SEL_RS1 = ADDI | ORI | SLLI | LW | BEQ | BNE | JAL | MUL | MULH | MULHU | DIV | DIVU | REM | REMU | SLT | SLTI | SLTU | SLTIU | AND | ANDI | OR;
    wire ALU_A_SEL_PC  = 1'b0;
                        
    // alub_sel
    wire ALU_B_SEL_RS2 = BEQ | BNE | MUL | MULH | MULHU | DIV | DIVU | REM | REMU | SLT | SLTU | AND | OR;
    wire ALU_B_SEL_EXT = ADDI | ORI | SLLI | LW | JAL | SLTI | SLTIU | ANDI;
        
    // ram_r_op
    wire RAM_EXT_B  = 1'b0;
    wire RAM_EXT_BU = 1'b0;
    wire RAM_EXT_H  = 1'b0;
    wire RAM_EXT_HU = 1'b0;
    wire RAM_EXT_W  = LW;

    // ram_w_op
    wire RAM_W_B  = 1'b0;
    wire RAM_W_H  = 1'b0;
    wire RAM_W_W  = 1'b0;
    
    assign npc_op = {2{NPC_OP_PC4}} & `NPC_PC4
                  | {2{NPC_OP_BRA}} & `NPC_BRA
                  | {2{NPC_OP_JMP}} & `NPC_JMP;

    assign rf_we = RF_OP_WE;

    assign rf_wsel = {2{WB_OP_ALU}} & `WB_ALU
                   | {2{WB_OP_RAM}} & `WB_RAM
                   | {2{WB_OP_PC4}} & `WB_PC4
                   | {2{WB_OP_EXT}} & `WB_EXT;

    assign sext_op = {3{EXT_OP_I}} & `EXT_I
                   | {3{EXT_OP_B}} & `EXT_B
                   | {3{EXT_OP_U}} & `EXT_U
                   | {3{EXT_OP_J}} & `EXT_J;
                   
    assign alu_op = {5{ALU_OP_ADD  }} & `ALU_ADD
                  | {5{ALU_OP_OR   }} & `ALU_OR
                  | {5{ALU_OP_SLL  }} & `ALU_SLL
                  | {5{ALU_OP_EQ   }} & `ALU_EQ
                  | {5{ALU_OP_NE   }} & `ALU_NE
                  | {5{ALU_OP_MUL  }} & `ALU_MUL
                  | {5{ALU_OP_MULH }} & `ALU_MULH
                  | {5{ALU_OP_MULHU}} & `ALU_MULHU
                  | {5{ALU_OP_DIV}} & `ALU_DIV
                  | {5{ALU_OP_DIVU}} & `ALU_DIVU
                  | {5{ALU_OP_REM}} & `ALU_REM
                  | {5{ALU_OP_REMU}} & `ALU_REMU
                  | {5{ALU_OP_SLT}} & `ALU_SLT
                  | {5{ALU_OP_SLTU}} & `ALU_SLTU
                  | {5{ALU_OP_AND}} & `ALU_AND
                  | {5{ALU_OP_LT}} & `ALU_LT
                  | {5{ALU_OP_GE}} & `ALU_GE
                  | {5{ALU_OP_LTU}} & `ALU_LTU
                  | {5{ALU_OP_GEU}} & `ALU_GEU;


    assign alua_sel = ALU_A_SEL_PC & `ALU_A_PC | ALU_A_SEL_RS1 & `ALU_A_RS1;

    assign alub_sel = ALU_B_SEL_RS2 & `ALU_B_RS2 | ALU_B_SEL_EXT & `ALU_B_EXT;
  
    assign ram_r_op = {3{RAM_EXT_B }} & `RAM_EXT_B
                    | {3{RAM_EXT_BU}} & `RAM_EXT_BU
                    | {3{RAM_EXT_H }} & `RAM_EXT_H
                    | {3{RAM_EXT_HU}} & `RAM_EXT_HU
                    | {3{RAM_EXT_W }} & `RAM_EXT_W;

    assign ram_w_op = {4{RAM_W_B}} & `RAM_WE_B
                    | {4{RAM_W_H}} & `RAM_WE_H
                    | {4{RAM_W_W}} & `RAM_WE_W;

    assign is_mul = MUL | MULH | MULHU;
    assign is_div = DIV | DIVU | REM | REMU;

endmodule
