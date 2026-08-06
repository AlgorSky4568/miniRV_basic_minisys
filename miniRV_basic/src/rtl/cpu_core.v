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
    wire        id_rf1; // 标识rs1是否被读取（Controller输出, wire类型, 避免Synth 8-685）
    wire        id_rf2; // 标识rs2是否被读取（Controller输出, wire类型, 避免Synth 8-685）

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
    wire        ex_is_ld_st;
    wire        mem_is_ld_st;
    reg         ld_st_flag;
    wire        ld_st_done;         // 访存完成的标志位信号

    wire        inst_finished;      // 指令执行完成的标志位信号
    reg         inst_finished_r;    // 复位：0，没复位：随便

    reg[31:0] ex_pc;
    reg [31:0] mem_pc;
    //数据冒险检测信号
    wire rs1_id_ex_hazard;
    wire rs2_id_ex_hazard;
    wire rs1_id_mem_hazard;
    wire rs2_id_mem_hazard;
    wire rs1_id_wb_hazard;
    wire rs2_id_wb_hazard;

    // ===== 数据前递相关信号（D5）=====
    wire ex_is_load;
    wire fwd_ex_rs1, fwd_ex_rs2;
    wire [31:0] ex_fwd_val;
    wire mem_data_ok;
    wire [31:0] mem_fwd_val;
    wire fwd_mem_rs1, fwd_mem_rs2;
    wire fwd_wb_rs1, fwd_wb_rs2;
    wire [31:0] fwd_r1, fwd_r2;
    wire load_use;
    wire mem_ld_pending_hazard;
    wire pipline_stop;
    wire mem_pipline_stop;
    wire wb_pipline_stop;

    // ===== EX 阶段分支重定向相关信号（D4）=====
    wire ex_bj_f;
    wire [31:0] ex_bj_target;
    wire ex_br_pending;
    wire need_redirect;
    reg  skip_fetch;

    // ===== 多周期 load/store 冻结与完成链（D6a）=====
    reg         ld_pending;
    reg  [ 4:0] ld_dest;
    reg  [31:0] ld_pc_val;
    reg         ld_done;
    wire        mem_ld_stall;
    wire        ex_mem_fwd1, ex_mem_fwd2;

    // ===== 乘除法完成链（D6b）=====
    wire        ex_is_mul_div;
    wire        id_is_mul_div;
    reg         mul_busy_r;
    wire        mul_done_pulse;
    reg         mul_div_wait;
    wire        mul_div_stall;
    reg         mul_post, mul_post2;
    reg  [ 4:0] mem_alu_op;
    wire        mem_is_mul_div;


    /***************************** IF *****************************/
    reg rst_r;  // 取cpu_rst下降沿
    wire first_req = rst_r & !cpu_rst; // 复位信号下降沿，首次取指
    always @(posedge cpu_clk) rst_r <= cpu_rst;


    wire pause_ifetch  = (mem_is_ld_st | is_ld_st | ex_is_ld_st) & !ld_st_done;
    wire resume_ifetch = ld_st_done | !mul_div_busy;

    assign ifetch_req  = !skip_fetch & !pause_ifetch & !inst_finished &
                         (first_req    |    // 复位后首次取指
                                        ifetch_valid |    // 上一条已取回，同时立即取下一条
                                        br      |    // 静态分支预测错误，立即用正确的地址取指
                                        need_redirect |    // D4：分支重定向拍强制请求目标地址
                                        resume_ifetch);   // 数据访存或乘除运算结束，继续取指
    // D4：分支重定向拍用目标地址取指（覆盖顺序地址 pc）；skip_fetch 时不再取指
    // （目标指令已在重定向拍被请求，返回后由 IF/ID 捕获，重复请求会被取两次）
    assign ifetch_addr = need_redirect ? ex_bj_target : pc;
    
    // npc计算器
    NPC U_NPC (
        .op         (npc_op),
        .pc         (pc),
        .offset     (ext),
        .br         (br),
        .alu_c      (alu_c),
        .pipline_stop(pipline_stop & !inst_finished),
        .npc        (npc),
        .pc4        (pc4)
    );
    
    // 完成inst_fetch之后pc变成npc
    PC U_PC (
        .clk        (cpu_clk),
        .rst        (cpu_rst),
        .npc        (need_redirect ? ex_bj_target : npc),
        .fetch      (need_redirect | inst_finished),
        .pc         (pc)
    );
    
    /***************************** ID *****************************/
    // 按照约定的时序，ifetch_inst只在ifetch_valid有效时有效，且它们仅有效1个时钟.
    // 此处是为了避免ifetch_valid撤销后，ifetch_inst发生变化从而导致指令执行出错.
    assign inst = ifetch_valid ? ifetch_inst : 32'h13 /* NOP */ ;
    wire [4:0]  id_rs1;            // ID阶段读寄存器1地址
    wire [4:0]  id_rs2;            // ID阶段读寄存器2地址

    reg[31:0] id_pc;
    reg[31:0] id_inst;

    reg[31:0] fetch_pc;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)       fetch_pc <= 32'h0;
        else if (ifetch_req) fetch_pc <= ifetch_addr;
    end


    assign id_rs1 = id_inst[19:15];
    assign id_rs2 = id_inst[24:20];


    //将IF阶段取值和下一指令的PC存起来
    always @(posedge cpu_clk or posedge cpu_rst) begin
      if (cpu_rst) begin
        id_pc <= 32'b0;
        id_inst <= 32'b0;
      end
      else if (need_redirect ||    // D4：分支重定向时 flush IF/ID（替代原 br——
         (mul_done_pulse & id_is_mul_div & (id_pc == ex_pc))) begin  // 乘除完成拍清空 EX 副本
        id_pc <= 32'b0;                 // br=1 但无需重定向（目标==ID 已持指令、
        id_inst <= 32'b0;               // 分支等 load 数据）时 flush 会丢正确路径指令）
      end
      else begin
        if(pipline_stop) begin
          if (!mem_pipline_stop & (id_pc != ex_pc)) begin   // ID/EX 推进拍：指令已进 EX
              id_pc <= 32'b0;
              id_inst <= 32'b0;
          end else begin
              id_pc <= id_pc;
              id_inst <= id_inst;
          end
        end
        else if (ifetch_valid & (((id_pc == fetch_pc) & (id_pc == ex_pc)) | (ex_pc == fetch_pc) |
                 (mem_is_ld_st & (mem_pc == fetch_pc)) |
                 (ld_done & (fetch_pc == ld_pc_val))) &
                 (fetch_pc != 32'h0)) begin
          id_pc <= 32'b0;
          id_inst <= 32'b0;
        end
        else begin
        id_pc <= fetch_pc;   // 指令的PC取请求地址的1拍延迟，与inst同步
        id_inst <= inst;
        end
      end
    end

    
    Controller U_CU (
        // input：译码ID阶段的指令（id_inst）。若用IF阶段的inst，IF空隙时
        // inst变NOP会误清控制信号，且暂停期间译码对象与冒险检测不一致
        .opcode         (id_inst[6:0]),
        .funct3         (id_inst[14:12]),
        .funct7         (id_inst[31:25]),
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
        .rf_wsel        (rf_wsel),
        .id_rf1         (id_rf1),
        .id_rf2         (id_rf2)
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
    
    // 立即数生成器：与Controller同步，取ID阶段指令的立即数位段
    SEXT U_SEXT (
        .op         (sext_op),
        .imm        (id_inst[31:7]),
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
        if (is_ld_st | is_mul_div) rf_wR_r <= id_inst[11:7];
    end

    /***************************** EX *****************************/
    //这是从寄存器中取出来的两个数
    reg[31:0] ex_rd1;
    reg[31:0] ex_rd2;
    reg[4:0] ex_rs1;
    reg[4:0] ex_rs2;


    reg[31:0] ex_ext; //立即数
    reg ex_alu_a_sel,ex_alu_b_sel; //操作数a和b的选择信号
    reg[4:0] ex_alu_op; //ALU的操作信号
    reg[2:0] ex_ram_rop; //访存读操作信号
    reg [1:0] ex_rf_wel; //确定用哪个数据写回
    reg ex_rf_we;  //表示是否写回
    reg [4:0] ex_rf_wR; //写回寄存器
    reg [1:0] ex_npc_op; //分支/跳转类型（D4：EX 阶段重定向判断用）


    wire hazard_A;
    

    // EX流水线寄存器：使用前递后的数据（fwd_rd1/fwd_rd2）
    // Synth 8-5413 修复：异步复位（cpu_rst）与同步 flush（need_redirect）拆开
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
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
            ex_rs1 <= 5'b0;
            ex_rs2 <= 5'b0;
            ex_npc_op <= 2'b0;
        end
        else if (need_redirect) begin   // D4：分支重定向时 flush ID/EX（替代原 br）
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
            ex_rs1 <= 5'b0;
            ex_rs2 <= 5'b0;
            ex_npc_op <= 2'b0;
        end
        else begin
            if(mem_pipline_stop || wb_pipline_stop)begin  //暂停ID/EX寄存器
                ex_rd1 <= ex_rd1;
                ex_rd2 <= ex_rd2;
                ex_pc <= ex_pc;
                ex_ext <= ex_ext;
                ex_alu_a_sel <= ex_alu_a_sel;
                ex_alu_b_sel <= ex_alu_b_sel;
                ex_alu_op <= ex_alu_op;
                ex_ram_rop <= ex_ram_rop;
                ex_ram_wop <= ex_ram_wop;
                ex_rf_wel <= ex_rf_wel;
                ex_rf_we <= ex_rf_we;
                ex_rf_wR <= ex_rf_wR;
                ex_rs1 <= ex_rs1;
                ex_rs2 <= ex_rs2;
                ex_npc_op <= ex_npc_op;

            end
            else if (mul_done_pulse & id_is_mul_div) begin
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
                ex_rs1 <= 5'b0;
                ex_rs2 <= 5'b0;
                ex_npc_op <= 2'b0;
            end
            else if (id_pc != ex_pc) begin
                ex_rd1 <= fwd_r1;   // 前递后操作数（EX > MEM > WB > 寄存器堆）
                ex_rd2 <= fwd_r2;
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
                ex_rs1 <= id_rs1;
                ex_rs2 <= id_rs2;
                ex_npc_op <= npc_op;
            end
            else begin
                // 冻结期 ID 是 EX 副本（id_pc==ex_pc）：EX 段保持正确操作数
                ex_rd1 <= ex_rd1;
                ex_rd2 <= ex_rd2;
                ex_pc <= ex_pc;
                ex_ext <= ex_ext;
                ex_alu_a_sel <= ex_alu_a_sel;
                ex_alu_b_sel <= ex_alu_b_sel;
                ex_alu_op <= ex_alu_op;
                ex_ram_rop <= ex_ram_rop;
                ex_ram_wop <= ex_ram_wop;
                ex_rf_wel <= ex_rf_wel;
                ex_rf_we <= ex_rf_we;
                ex_rf_wR <= ex_rf_wR;
                ex_rs1 <= ex_rs1;
                ex_rs2 <= ex_rs2;
                ex_npc_op <= ex_npc_op;
            end
        end
    end

    assign ex_is_ld_st = (ex_ram_rop != `RAM_EXT_N) | (ex_ram_wop != `RAM_WE_N);

    assign rs1_id_ex_hazard = (ex_rf_wR == id_rs1) & ex_rf_we & id_rf1 & (ex_rf_wR != 5'h0);
    assign rs2_id_ex_hazard = (ex_rf_wR == id_rs2) & ex_rf_we & id_rf2 & (ex_rf_wR != 5'h0);

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

    reg [31:0] mem_pc_r;   // MEM 段 PC 1 拍延迟（MEM 推进检测——冻结期同指令写回防重复计数）
    reg [31:0] mem_ext;
    reg[2:0] mem_ram_rop; //访存读操作信号
    reg  [ 3:0] mem_ram_wop;        // 流水线化的ram_wop
    reg [31:0] mem_rd2;


    always@(posedge cpu_clk or posedge cpu_rst)begin
        if(cpu_rst)begin
            mem_alu_c <= 32'b0;
            mem_rf_wel <= 2'b0;
            mem_rf_we <= 1'b0;
            mem_rf_wR <= 5'b0;
            mem_pc <= 32'b0;
            mem_ext <= 32'b0;
            mem_ram_rop <= 3'b0;
            mem_ram_wop <= 4'b0;
            mem_rd2 <= 32'b0;
            mem_alu_op <= 5'b0;   // D6b：mem 级 alu_op（防 mul 驻留 MEM 期间重复 WB）
        end
        else begin
            if(wb_pipline_stop)begin
                mem_alu_c <= mem_alu_c;
                mem_rf_wel <= mem_rf_wel;
                mem_rf_we <= mem_rf_we;
                mem_rf_wR <= mem_rf_wR;
                mem_pc <= mem_pc;
                mem_ext <= mem_ext;
                mem_ram_rop <= mem_ram_rop;
                mem_ram_wop <= mem_ram_wop;
                mem_rd2 <= mem_rd2;
                mem_alu_op <= mem_alu_op;   // D6b
            end
            else begin
            mem_alu_c <= alu_c;
            mem_rf_wel <= ex_rf_wel;
            mem_rf_we <= ex_rf_we;
            mem_rf_wR <= ex_rf_wR;
            mem_pc <= ex_pc;
            mem_ext <= ex_ext;
            mem_ram_rop <= ex_ram_rop;
            mem_ram_wop <= ex_ram_wop;
            mem_rd2 <= ex_rd2;
            mem_alu_op <= ex_alu_op;   // D6b
            end
        end
    end


    MREQ U_MEM_REQ (
        .ram_addr   (mem_alu_c),

        .ram_rop    (mem_ram_rop),
        .da_ren     (da_ren),
        .da_addr    (da_addr),

        .ram_wop    (mem_ram_wop),       // 使用流水线化后的ram_wop
        .ram_wdata  (mem_rd2),           // 使用流水线化后的rs2数据
        .da_wen     (da_wen),
        .da_wdata   (da_wdata)
    );

    MEXT U_MEM_EXT (
        .op             (ram_rop_r),
        .din            (daccess_rdata),
        .byte_offs      (alu_c_r[1:0]),
        .ext            (ram_ext)
    );

    assign mem_is_ld_st = (mem_ram_rop != `RAM_EXT_N) | (mem_ram_wop != `RAM_WE_N);

    // ===== 多周期 load/store 冻结与完成链（D6a）=====
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)                    ld_pending <= 1'b0;
        else if (daccess_rvalid | daccess_wresp) ld_pending <= 1'b0;
        else if (!daccess_rvalid & !daccess_wresp & mem_is_ld_st & !ld_pending & !ld_done) ld_pending <= 1'b1;
    end
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) ld_done <= 1'b0;
        else if (mem_is_ld_st & ld_done & (mem_pc == ld_pc_val)) ld_done <= 1'b1;
        else         ld_done <= ld_pending & (daccess_rvalid | daccess_wresp);
    end
    always @(posedge cpu_clk) begin
        if (mem_is_ld_st & !ld_pending) begin
            ld_dest   <= mem_rf_wR;
            ld_pc_val <= mem_pc;
            alu_c_r   <= mem_alu_c;
            ram_rop_r <= mem_ram_rop;
        end
    end
    assign mem_ld_stall = mem_is_ld_st & !(ld_done & (mem_pc == ld_pc_val)) &
                          !(daccess_rvalid | daccess_wresp);

    // ===== 乘除法完成链（D6b）=====
    assign ex_is_mul_div = (ex_alu_op >= 5'h10);     // ALU_MUL(0x10)..ALU_REMU(0x16)
    assign id_is_mul_div = (alu_op >= 5'h10);
    always @(posedge cpu_clk) mul_busy_r <= mul_div_busy;
    assign mul_done_pulse = mul_busy_r & !mul_div_busy;   // busy 下降沿 = 完成
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)                        mul_div_wait <= 1'b0;
        else if (ex_is_mul_div & !mul_div_wait) mul_div_wait <= 1'b1;
        else if (mul_done_pulse)            mul_div_wait <= 1'b0;
    end
    assign mul_div_stall = ex_is_mul_div & !mul_done_pulse;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            mul_post  <= 1'b0;
            mul_post2 <= 1'b0;
        end else begin
            mul_post  <= mul_done_pulse;
            mul_post2 <= mul_post;
        end
    end
    assign mem_is_mul_div = (mem_alu_op >= 5'h10);

    assign ex_mem_fwd1 = (mem_rf_wR == ex_rs1) & mem_rf_we & (ex_rs1 != 5'h0) &
                         mem_data_ok & (ex_pc != mem_pc);
    assign ex_mem_fwd2 = (mem_rf_wR == ex_rs2) & mem_rf_we & (ex_rs2 != 5'h0) &
                         mem_data_ok & (ex_pc != mem_pc);
    assign alu_a = ex_alu_a_sel ? ex_pc  : (ex_mem_fwd1 ? mem_fwd_val : ex_rd1);
    assign alu_b = ex_alu_b_sel ? ex_ext : (ex_mem_fwd2 ? mem_fwd_val : ex_rd2);

    // Interface to Bus
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            daccess_ren   <= 4'h0;
            daccess_wen   <= 4'h0;
        end else begin
            daccess_ren   <= (mem_is_ld_st & !ld_pending & !ld_done) ? da_ren : 4'h0;
            daccess_addr  <= da_addr;
            daccess_wen   <= (mem_is_ld_st & !ld_pending & !ld_done) ? da_wen : 4'h0;
            daccess_wdata <= da_wdata;
        end
    end

    assign ld_st_done = daccess_rvalid | daccess_wresp;
    assign rs1_id_mem_hazard = (mem_rf_wR == id_rs1) & mem_rf_we & id_rf1 & (mem_rf_wR != 5'h0);
    assign rs2_id_mem_hazard = (mem_rf_wR == id_rs2) & mem_rf_we & id_rf2 & (mem_rf_wR != 5'h0);

    /********************* EX 阶段分支重定向（D4）*********************/
    assign ex_bj_f = (ex_npc_op == `NPC_JMP) |
                     (ex_npc_op == `NPC_JALR) |
                     ((ex_npc_op == `NPC_BRA) & br);
    assign ex_bj_target = (ex_npc_op == `NPC_JALR) ? {alu_c[31:1], 1'b0} :
                          (ex_pc + ex_ext);
    assign ex_br_pending = mem_is_ld_st & !daccess_rvalid & mem_rf_we &
                           (mem_rf_wR != 5'h0) &
                           ((mem_rf_wR == ex_rs1) | (mem_rf_wR == ex_rs2));
    assign need_redirect = ex_bj_f & (ex_bj_target != id_pc) & !ex_br_pending;

    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)                        skip_fetch <= 1'b0;
        else if (need_redirect & !pause_ifetch) skip_fetch <= 1'b1;
        else                                skip_fetch <= 1'b0;
    end

    /***************************** WB *****************************/
    assign rf_we1 = (ld_pending & daccess_rvalid) |
                    mul_done_pulse |
                    mem_rf_we & !mem_is_ld_st & !mem_is_mul_div & (mem_pc != mem_pc_r);
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) mem_pc_r <= 32'b0;
        else         mem_pc_r <= mem_pc;
    end

    assign rf_wR  = (ld_pending & daccess_rvalid) ? ld_dest :
                    mul_done_pulse ? ex_rf_wR :
                    mem_rf_wR;

    always @(*) begin
        if (ld_pending & daccess_rvalid)
            rf_wD = ram_ext;    // Load data（进 MEM 首拍锁存的 ram_rop_r/alu_c_r 保证正确）
        else if (mul_done_pulse)
            rf_wD = alu_c;
        else begin
            case (mem_rf_wel)
                `WB_ALU: rf_wD = mem_alu_c;
                `WB_PC4: rf_wD = mem_pc + 32'd4;     // 使用流水线化的PC+4，而非当前pc4
                `WB_EXT: rf_wD = mem_ext;             // 使用流水线化的立即数，而非当前ext
                `WB_RAM: rf_wD = ram_ext;
                default: rf_wD = 32'h0;
            endcase
        end
    end

    assign rs1_id_wb_hazard = (rf_wR == id_rs1) & rf_we1 & id_rf1 & (rf_wR != 5'h0);
    assign rs2_id_wb_hazard = (rf_wR == id_rs2) & rf_we1 & id_rf2 & (rf_wR != 5'h0);

    /********************* 数据前递与停顿（D5）*********************/
    assign ex_is_load = (ex_ram_rop != `RAM_EXT_N);
    assign fwd_ex_rs1 = (ex_rf_wR == id_rs1) & ex_rf_we & id_rf1 & (ex_rf_wR != 5'h0) & !ex_is_load;
    assign fwd_ex_rs2 = (ex_rf_wR == id_rs2) & ex_rf_we & id_rf2 & (ex_rf_wR != 5'h0) & !ex_is_load;
    assign ex_fwd_val = (ex_rf_wel == `WB_EXT) ? ex_ext :
                        (ex_rf_wel == `WB_PC4) ? (ex_pc + 32'h4) :
                                                  alu_c;
    assign mem_data_ok = !mem_is_ld_st | daccess_rvalid;
    assign mem_fwd_val = mem_is_ld_st ? ram_ext :
                         (mem_rf_wel == `WB_EXT) ? mem_ext :
                         (mem_rf_wel == `WB_PC4) ? (mem_pc + 32'h4) :
                                                    mem_alu_c;
    assign fwd_mem_rs1 = (mem_rf_wR == id_rs1) & mem_rf_we & id_rf1 & (mem_rf_wR != 5'h0) & mem_data_ok;
    assign fwd_mem_rs2 = (mem_rf_wR == id_rs2) & mem_rf_we & id_rf2 & (mem_rf_wR != 5'h0) & mem_data_ok;
    assign fwd_wb_rs1 = (rf_wR == id_rs1) & rf_we1 & id_rf1 & (rf_wR != 5'h0);
    assign fwd_wb_rs2 = (rf_wR == id_rs2) & rf_we1 & id_rf2 & (rf_wR != 5'h0);
    assign fwd_r1 = fwd_ex_rs1 ? ex_fwd_val :
                    fwd_mem_rs1 ? mem_fwd_val :
                    fwd_wb_rs1  ? rf_wD :
                                  rf_rd1;
    assign fwd_r2 = fwd_ex_rs2 ? ex_fwd_val :
                    fwd_mem_rs2 ? mem_fwd_val :
                    fwd_wb_rs2  ? rf_wD :
                                  rf_rd2;

    assign load_use = ex_is_load &
                      ((ex_rf_wR == id_rs1) & id_rf1 | (ex_rf_wR == id_rs2) & id_rf2) &
                      (ex_rf_wR != 5'h0);
    assign mem_ld_pending_hazard = (rs1_id_mem_hazard | rs2_id_mem_hazard) & !mem_data_ok;
    assign pipline_stop     = mem_ld_pending_hazard | (mem_ld_stall & (id_pc != ex_pc)) |
                              mul_div_stall | mul_post | mul_post2 |
                              (mul_done_pulse & id_is_mul_div) | !ifetch_valid;
    assign mem_pipline_stop = mem_ld_pending_hazard | mem_ld_stall |
                              mul_div_stall | mul_post | mul_post2 |
                              (!ifetch_valid & !pause_ifetch & !mul_done_pulse);
    assign wb_pipline_stop  = mem_ld_pending_hazard | mem_ld_stall |
                              mul_div_stall | mul_post | mul_post2;
    assign inst_finished = ld_st_flag   & ld_st_done    |
                           mul_done_pulse & !(id_is_mul_div & (id_pc != ex_pc)) |
                           (id_is_mul_div & !mul_div_wait & (pc == (id_pc + 32'h4)) &
                            !mul_post & !mul_post2 & ifetch_valid) |
                           ifetch_valid & !is_ld_st & !is_mul_div & (id_pc == fetch_pc) &
                           !mem_ld_stall & !ex_is_ld_st;


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

    assign debug_wb_pc    = (ld_pending & daccess_rvalid) ? ld_pc_val :
                            mul_done_pulse ? ex_pc :
                            mem_pc;
    assign debug_wb_rf_we = rf_we1;
    assign debug_wb_rf_wR = rf_wR;
    assign debug_wb_rf_wD = rf_wD;

    assign debug_mem_pc    = mem_pc;  // MEM阶段PC：同理应连mem_pc而非IF阶段pc
    assign debug_mem_we    = daccess_wen;
    assign debug_mem_waddr = daccess_addr;
    assign debug_mem_wdata = daccess_wdata;
`endif

endmodule
