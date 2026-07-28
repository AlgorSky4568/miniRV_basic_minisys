`timescale 1ns / 1ps

`include "defines.vh"

module cpu_core(
    input  wire         cpu_rst,
    input  wire         cpu_clk,

    // Instruction Fetch Interface 取址相关
    output wire         ifetch_req   /* verilator public */ ,
    output wire [31:0]  ifetch_addr  /* verilator public */ ,
    input  wire         ifetch_valid /* verilator public */ ,
    input  wire [31:0]  ifetch_inst,
    
    // Data Access Interface    输出：读使能，读地址，写使能，写地址 输入：读有效，读数据，写反应
    output reg  [ 3:0]  daccess_ren,
    output reg  [31:0]  daccess_addr,
    input  wire         daccess_rvalid,
    input  wire [31:0]  daccess_rdata,
    output reg  [ 3:0]  daccess_wen,
    output reg  [31:0]  daccess_wdata,
    input  wire         daccess_wresp
);

    // PC and NPC
    wire [31:0] pc;
    wire [31:0] npc;
    wire [31:0] pc4;
    wire [31:0] inst;

    // Controller
    wire [ 1:0] npc_op;
    wire [ 1:0] rf_wsel;
    wire [ 2:0] sext_op;
    wire [ 4:0] alu_op;
    wire        alua_sel;
    wire        alub_sel;
    wire [ 2:0] ram_rop;
    reg  [ 2:0] ram_rop_r;
    wire [ 3:0] ram_wop;
    reg  [ 3:0] ex_ram_wop;        // 流水线化的ram_wop
    wire        is_mul;
    wire        is_div;
    wire        is_mul_div;
    reg         mul_div_flag;       // 乘除法运算的标志位信号

    // Register File
    wire [31:0] rf_rd1;
    wire [31:0] rf_rd2;
    wire        rf_we;
    wire        rf_we1;
    reg  [ 4:0] rf_wR_r;
    wire [ 4:0] rf_wR;
    reg  [31:0] rf_wD;

    // Signed Extension
    wire [31:0] ext;

    // ALU
    wire [31:0] alu_a;
    wire [31:0] alu_b;
    wire [31:0] alu_c;
    reg  [31:0] alu_c_r;
    wire        br;
    wire        mul_div_busy;
    
    // Memory Access
    wire [ 3:0] da_ren;
    wire [31:0] da_addr;
    wire [ 3:0] da_wen;
    wire [31:0] da_wdata;
    wire [31:0] ram_ext;
    wire        is_ld_st;
    reg         ld_st_flag;
    wire        ld_st_done;         // 访存完成的标志位信号

    wire        inst_finished;      // 指令执行完成的标志位信号
    reg         inst_finished_r;    // 复位：0，没复位：随便

    // ============= ID阶段寄存器地址（用于RF读取和前递） =============
    wire [4:0]  id_rs1;            // ID阶段读寄存器1地址
    wire [4:0]  id_rs2;            // ID阶段读寄存器2地址

    assign id_rs1 = inst[19:15];
    assign id_rs2 = inst[24:20];

    /***************************** IF *****************************/
    reg rst_r;  // 取cpu_rst下降沿
    wire first_req = rst_r & !cpu_rst; // 复位信号下降沿，首次取指
    always @(posedge cpu_clk) rst_r <= cpu_rst;

    // 复位信号发生边沿变化时首次取指; 取指完成后取下一条指令
    assign ifetch_req  = first_req | ifetch_valid;
    assign ifetch_addr = pc;
    
    // npc计算器
    // 看输入的两位opcode，取决最后输出是:
    // 一般情况:PC+4
    // B型指令:PC+4(条件成立)或者PC+offset（条件不成立）
    // J型：PC+offset
    NPC U_NPC (
        .op         (npc_op),
        .pc         (pc),
        .offset     (ext),
        .br         (br),
        .alu_c      (alu_c),
        .npc        (npc),
        .pc4        (pc4)
    );
    
    // 完成inst_fetch之后pc变成npc
    PC U_PC (
        .clk        (cpu_clk),
        .rst        (cpu_rst),
        .npc        (npc),
        .fetch      (inst_finished),
        .pc         (pc)
    );
    
    /***************************** ID *****************************/
    // 按照约定的时序，ifetch_inst只在ifetch_valid有效时有效，且它们仅有效1个时钟.
    // 此处是为了避免ifetch_valid撤销后，ifetch_inst发生变化从而导致指令执行出错.
    assign inst = ifetch_valid ? ifetch_inst : 32'h13 /* NOP */ ;

    reg[31:0] id_pc;
    reg[31:0] id_inst;

    //将IF阶段取值和下一指令的PC存起来
    always@(posedge cpu_clk or posedge cpu_rst)begin
      if(cpu_rst) begin
        id_pc <= 32'b0;
        id_inst <= 32'b0;
      end
      else begin
        id_pc <= pc; 
        id_inst <= inst;
      end
    end

    
    Controller U_CU (
        // input
        .opcode         (inst[6:0]),
        .funct3         (inst[14:12]),
        .funct7         (inst[31:25]),
        // output
        .npc_op         (npc_op),
        .sext_op        (sext_op),
        .alu_op         (alu_op),
        .alua_sel       (alua_sel),
        .alub_sel       (alub_sel),
        .is_mul         (is_mul),
        .is_div         (is_div),
        .ram_r_op       (ram_rop),
        .ram_w_op       (ram_wop),
        .rf_we          (rf_we),
        .rf_wsel        (rf_wsel)
    );
    
    
    // 32个32位寄存器
    RF U_RF (
        .clk        (cpu_clk),
        .rR1        (id_rs1),
        .rR2        (id_rs2),
        .rD1        (rf_rd1),
        .rD2        (rf_rd2),
        .we         (rf_we1),
        .wR         (rf_wR),
        .wD         (rf_wD)
    );
    
    // 立即数生成器
    SEXT U_SEXT (
        .op         (sext_op),
        .imm        (inst[31:7]),
        .ext        (ext)
    );
    
    // 遇到访存指令时, 拉高ld_st_flag标志位，表示正在执行访存指令
    assign is_ld_st = (ram_rop != `RAM_EXT_N) | (ram_wop != `RAM_WE_N);
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if      (cpu_rst)    ld_st_flag <= 1'b0;
        else if (is_ld_st)   ld_st_flag <= 1'b1;
        else if (ld_st_done) ld_st_flag <= 1'b0;
    end

    // 遇到乘除法指令时，拉高mul_div_flag标志位，表示正在执行乘除法指令
    assign is_mul_div = is_mul | is_div;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if      (cpu_rst)       mul_div_flag <= 1'b0;
        else if (is_mul_div)    mul_div_flag <= 1'b1;
        else if (!mul_div_busy) mul_div_flag <= 1'b0;
    end

    // 访存、乘除法指令无法在1个时钟内执行完，故先把指令的目标寄存器缓存起来
    always @(posedge cpu_clk) begin
        if (is_ld_st | is_mul_div) rf_wR_r <= inst[11:7];
    end

    /***************************** EX *****************************/
    //这是从寄存器中取出来的两个数
    reg[31:0] ex_rd1;
    reg[31:0] ex_rd2;

    reg[31:0] ex_pc;
    reg[31:0] ex_ext; //立即数
    reg ex_alu_a_sel,ex_alu_b_sel; //操作数a和b的选择信号
    reg[4:0] ex_alu_op; //ALU的操作信号
    reg[2:0] ex_ram_rop; //访存读操作信号
    reg [1:0] ex_rf_wel; //确定用哪个数据写回
    reg ex_rf_we;  //表示是否写回
    reg [4:0] ex_rf_wR; //写回寄存器

    // EX流水线寄存器：使用前递后的数据（fwd_rd1/fwd_rd2）
    always@(posedge cpu_clk or posedge cpu_rst)begin
        if(cpu_rst)begin
            ex_rd1 <= 32'b0;
            ex_rd2 <= 32'b0;
            ex_pc <= 32'b0;
            ex_ext <= 32'b0;
            ex_alu_a_sel <= 1'b0;
            ex_alu_b_sel <= 1'b0;
            ex_alu_op <= 5'b0;
            ex_ram_rop <= 3'b0;
            ex_ram_wop <= 4'b0;
            ex_rf_wel <= 2'b0;
            ex_rf_we <= 1'b0;
            ex_rf_wR <= 5'b0;
        end
        else begin
            ex_rd1 <= rf_rd1;
            ex_rd2 <= rf_rd2;
            ex_pc <= id_pc;
            ex_ext <= ext;
            ex_alu_a_sel <= alua_sel;
            ex_alu_b_sel <= alub_sel;
            ex_alu_op <= alu_op;
            ex_ram_rop <= ram_rop;
            ex_ram_wop <= ram_wop;      // 流水线化ram_wop
            ex_rf_wel <= rf_wsel;
            ex_rf_we <= rf_we;
            ex_rf_wR <= id_inst[11:7];
        end
    end

    assign alu_a = ex_alu_a_sel ? ex_pc  : ex_rd1;
    assign alu_b = ex_alu_b_sel ? ex_ext : ex_rd2;

    ALU U_ALU (
        .rst        (cpu_rst),
        .clk        (cpu_clk),
        .op         (ex_alu_op),
        .a          (alu_a),
        .b          (alu_b),
        .br         (br),
        .c          (alu_c),
        .busy       (mul_div_busy)
    );

    /***************************** MEM *****************************/
    //需要从EX传递过来的信号：alu_c，几个写回信号,pc,立即数
    reg[31:0] mem_alu_c;
    reg [1:0] mem_rf_wel;
    reg       mem_rf_we;
    reg [4:0] mem_rf_wR;
    reg [31:0] mem_pc;
    reg [31:0] mem_ext;

    always@(posedge cpu_clk or posedge cpu_rst)begin
        if(cpu_rst)begin
            mem_alu_c <= 32'b0;
            mem_rf_wel <= 2'b0;
            mem_rf_we <= 1'b0;
            mem_rf_wR <= 5'b0;
            mem_pc <= 32'b0;
            mem_ext <= 32'b0;
        end
        else begin
            mem_alu_c <= alu_c;
            mem_rf_wel <= ex_rf_wel;
            mem_rf_we <= ex_rf_we;
            mem_rf_wR <= ex_rf_wR;
            mem_pc <= ex_pc;
            mem_ext <= ex_ext;
        end
    end


    MREQ U_MEM_REQ (
        .ram_addr   (alu_c),

        .ram_rop    (ex_ram_rop),
        .da_ren     (da_ren),
        .da_addr    (da_addr),

        .ram_wop    (ex_ram_wop),       // 使用流水线化后的ram_wop
        .ram_wdata  (ex_rd2),           // 使用流水线化后的rs2数据
        .da_wen     (da_wen),
        .da_wdata   (da_wdata)
    );

    MEXT U_MEM_EXT (
        .op             (ram_rop_r),
        .din            (daccess_rdata),
        .byte_offs      (alu_c_r[1:0]),
        .ext            (ram_ext)
    );

    always @(posedge cpu_clk) if (is_ld_st) alu_c_r   <= alu_c;
    always @(posedge cpu_clk) if (is_ld_st) ram_rop_r <= ram_rop;

    // Interface to Bus
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            daccess_ren   <= 4'h0;
            daccess_wen   <= 4'h0;
        end else begin
            daccess_ren   <= da_ren;
            daccess_addr  <= da_addr;
            daccess_wen   <= da_wen;
            daccess_wdata <= da_wdata;
        end
    end

    assign ld_st_done = daccess_rvalid | daccess_wresp;

    /***************************** WB *****************************/
    // 写回使能信号：
    // - Load指令：ld_st_flag置位且daccess_rvalid有效时写回
    // - 乘除法指令：mul_div_flag置位且运算结束时写回
    // - 其他单周期指令：ifetch_valid有效且mem_rf_we置位时写回
    //   注意：此处不再依赖ID阶段的is_ld_st/is_mul_div，避免多周期指令阻塞前一条指令的写回
    assign rf_we1 = ld_st_flag   & daccess_rvalid |
                    mul_div_flag & !mul_div_busy  |
                    ifetch_valid & mem_rf_we;

    assign rf_wR  = ld_st_flag | mul_div_flag ? rf_wR_r : mem_rf_wR;

    // 写回数据选择：
    // - WB_ALU: ALU计算结果（使用mem_pc+4计算返回地址，使用mem_ext作为立即数）
    // - WB_RAM: 访存读取数据（由ld_st_flag控制）
    always @(posedge cpu_clk) begin
        casex ({ld_st_flag, mem_rf_wel})
            {1'b0, `WB_ALU}: rf_wD = mem_alu_c;
            {1'b0, `WB_PC4}: rf_wD = mem_pc + 32'd4;     // 使用流水线化的PC+4，而非当前pc4
            {1'b0, `WB_EXT}: rf_wD = mem_ext;             // 使用流水线化的立即数，而非当前ext
            {1'b1, 2'b??  }: rf_wD = ram_ext;
            default        : rf_wD = 32'h0;
        endcase
    end

    // 指令完成信号：
    // - 访存指令：ld_st_flag置位且读写完成
    // - 乘除法指令：mul_div_flag置位且运算完成
    // - 单周期指令：ifetch_valid有效且当前ID阶段不是多周期指令（避免多周期指令在标志位置位前被误认为完成）
    assign inst_finished = ld_st_flag   & ld_st_done    |
                           mul_div_flag & !mul_div_busy |
                           ifetch_valid & !is_ld_st & !is_mul_div;


    always @(posedge cpu_clk or posedge cpu_rst) begin
        inst_finished_r <= cpu_rst ? 1'b0 : inst_finished;
    end


    /********************* Your CPU ends here *********************/

`ifdef RUN_TRACE
    wire [31:0] debug_wb_pc    /* verilator public */ ;     // WB阶段的PC
    wire        debug_wb_rf_we /* verilator public */ ;     // WB阶段的寄存器写使能
    wire [ 4:0] debug_wb_rf_wR /* verilator public */ ;     // WB阶段的目标寄存器   (若wb_rf_we为0，此项可为任意值)
    wire [31:0] debug_wb_rf_wD /* verilator public */ ;     // WB阶段写入寄存器的值 (若wb_rf_we为0，此项可为任意值)

    wire [31:0] debug_mem_pc    /* verilator public */ ;    // MEM阶段的PC
    wire [ 3:0] debug_mem_we    /* verilator public */ ;    // MEM阶段写访存时的写使能
    wire [31:0] debug_mem_waddr /* verilator public */ ;    // MEM阶段写访存时的写地址 (若mem_we为0，此项可为任意值)
    wire [31:0] debug_mem_wdata /* verilator public */ ;    // MEM阶段写访存时的写数据 (若mem_we为0，此项可为任意值)

    assign debug_wb_pc    = pc;
    assign debug_wb_rf_we = rf_we1;
    assign debug_wb_rf_wR = rf_wR;
    assign debug_wb_rf_wD = rf_wD;

    assign debug_mem_pc    = pc;
    assign debug_mem_we    = daccess_wen;
    assign debug_mem_waddr = daccess_addr;
    assign debug_mem_wdata = daccess_wdata;
`endif

endmodule
