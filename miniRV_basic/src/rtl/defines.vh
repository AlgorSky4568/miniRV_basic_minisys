// `define RUN_TRACE

// `define ENABLE_ICACHE
// `define ENABLE_DCACHE
// `define USE_DDR

`define PC_INIT_VAL 32'h0

`define ALU_ADD     5'h00
`define ALU_SUB     5'h01
`define ALU_AND     5'h02    // B组: and, andi
`define ALU_OR      5'h03
`define ALU_XOR     5'h04
`define ALU_SLL     5'h05
`define ALU_SRL     5'h06
`define ALU_SRA     5'h07
`define ALU_EQ      5'h08
`define ALU_NE      5'h09
`define ALU_LT      5'h0A    // B组: blt
`define ALU_LTU     5'h0B    // B组: bltu
`define ALU_GE      5'h0C    // B组: bge
`define ALU_GEU     5'h0D    // B组: bgeu
`define ALU_SLT     5'h0E    // B组: slt, slti
`define ALU_SLTU    5'h0F    // B组: sltu, sltiu
`define ALU_MUL     5'h10    // B组: mul
`define ALU_MULH    5'h11    // B组: mulh
`define ALU_MULHU   5'h12    // B组: mulhu
`define ALU_DIV     5'h13    // B组: div
`define ALU_DIVU    5'h14    // B组: divu
`define ALU_REM     5'h15    // B组: rem
`define ALU_REMU    5'h16    // B组: remu

`define NPC_PC4     2'b00
`define NPC_JALR    2'b01
`define NPC_BRA     2'b10
`define NPC_JMP     2'b11

`define EXT_I       3'b000
`define EXT_S       3'b001
`define EXT_B       3'b010
`define EXT_U       3'b011
`define EXT_J       3'b100

`define WB_ALU      2'b00
`define WB_RAM      2'b01
`define WB_PC4      2'b10
`define WB_EXT      2'b11

`define ALU_A_RS1   1'b0
`define ALU_A_PC    1'b1

`define ALU_B_RS2   1'b0
`define ALU_B_EXT   1'b1

`define RAM_EXT_N   3'b000
`define RAM_EXT_W   3'b001
`define RAM_EXT_B   3'b010
`define RAM_EXT_BU  3'b011
`define RAM_EXT_H   3'b100
`define RAM_EXT_HU  3'b101

`define RAM_WE_N    4'b0000
`define RAM_WE_B    4'b0001
`define RAM_WE_H    4'b0011
`define RAM_WE_W    4'b1111

// Address Space
`define MEM_BLOCK_MEMORY    32'h0000_0000   // 512KB (0x0000_0000 ~ 0x0007_FFFF)
`define MEM_DDR3            32'h2000_0000   // 512MB (0x2000_0000 ~ 0x3FFF_FFFF)
`define PERI_ADDR_SWITCH    32'hFFFF_0000
`define PERI_ADDR_LED       32'hFFFF_1000
`define PERI_ADDR_DIGLED    32'hFFFF_2000
`define PERI_ADDR_UART      32'hFFFF_3000
`define PERI_ADDR_TIMER     32'hFFFF_4000

`ifdef ENABLE_ICACHE
    `define IC_BLK_LEN  4
    `define IC_BLK_SIZE (`IC_BLK_LEN*32)
`else
    `define IC_BLK_LEN  1
    `define IC_BLK_SIZE (`IC_BLK_LEN*32)
`endif

`ifdef ENABLE_DCACHE
    `define DC_BLK_LEN  4
    `define DC_BLK_SIZE (`DC_BLK_LEN*32)
`else
    `define DC_BLK_LEN  1
    `define DC_BLK_SIZE (`DC_BLK_LEN*32)
`endif
