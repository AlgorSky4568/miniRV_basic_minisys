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
    reg[31:0] ex_pc;
    reg [31:0] mem_pc;

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
    wire        id_rf1;
    wire        id_rf2; //这两者用来标识rs1和rs2是否被读取了

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

    //数据冒险检测信号
    wire rs1_id_ex_hazard;
    wire rs2_id_ex_hazard;
    wire rs1_id_mem_hazard;
    wire rs2_id_mem_hazard;
    wire rs1_id_wb_hazard;
    wire rs2_id_wb_hazard;

    // ===== 数据前递相关信号（D5）=====
    // 声明在模块顶部（NPC/IF/ID/ID/EX/EX/MEM 段需提前使用 pipline_stop 等），
    // 赋值集中在下部"数据前递与停顿"块（需引用 ex_*/mem_*/WB 段信号，避免 use-before-declaration）
    wire ex_is_load;                    // EX 阶段是 load 指令
    wire fwd_ex_rs1, fwd_ex_rs2;        // EX→ID 前递命中
    wire [31:0] ex_fwd_val;             // EX 前递值（按写回来源分源）
    wire mem_data_ok;                   // MEM 数据就绪（非 load，或 load 的 daccess_rvalid 已到）
    wire [31:0] mem_fwd_val;            // MEM 前递值（按写回来源分源）
    wire fwd_mem_rs1, fwd_mem_rs2;      // MEM→ID 前递命中
    wire fwd_wb_rs1, fwd_wb_rs2;        // WB→ID 前递命中
    wire [31:0] fwd_r1, fwd_r2;         // 前递后操作数（EX > MEM > WB > 寄存器堆）
    wire load_use;                      // load-use 冒险（EX 中 load 的目标 == ID 源寄存器）
    wire mem_ld_pending_hazard;         // MEM 中 load 数据未到且 ID 依赖它
    wire pipline_stop;                  // PC+IF/ID 冻结
    wire mem_pipline_stop;              // ID/EX 冻结
    wire wb_pipline_stop;               // EX/MEM 冻结

    // ===== EX 阶段分支重定向相关信号（D4）=====
    // 声明在模块顶部（IF 段 ifetch_req/ifetch_addr/PC/IF-ID 捕获需提前使用），
    // 赋值集中在下部"EX 阶段分支重定向（D4）"块（需引用 EX/MEM 段信号）
    wire ex_bj_f;                       // EX 阶段是跳转/分支且条件成立（需重定向）
    wire [31:0] ex_bj_target;           // EX 阶段重定向目标地址
    wire ex_br_pending;                 // EX 分支依赖 MEM 中未返回数据的 load（抑制重定向）
    wire need_redirect;                 // 需要 EX 阶段分支重定向
    reg  skip_fetch;                    // 重定向后跳过 1 拍取指（防目标指令被取两次）

    // ===== 多周期 load/store 冻结与完成链（D6a）=====
    // 以修复版（Exp2_A_pipeline_fixed）为基准 merge：原 ld_st_flag 电平标志
    // 无法让 load 驻留 MEM 等数据（无依赖时 1 拍滑过，rvalid 晚到后 WB 报
    // 后继指令的 PC），且连续 ld/st 时 rf_wR_r 全局锁存被下一条覆盖。
    // ld_pending/ld_dest/ld_pc_val：本条 ld/st 的"进行中 + 身份"（进 MEM 首拍锁存）
    // ld_done：完成脉冲（rvalid/wresp 后 1 拍）；mem_ld_stall：MEM 冻结判定
    reg         ld_pending;
    reg  [ 4:0] ld_dest;
    reg  [31:0] ld_pc_val;
    reg         ld_done;
    wire        mem_ld_stall;
    // EX 侧 MEM 前递（修复版 Bug 14 Fix 5）：load-use 依赖指令冻结在 EX 等
    // 数据，rvalid 到达后按 ex_rs1/ex_rs2 重新前递（否则用进 EX 时锁存的
    // 旧操作数计算——lw 测试 addi x6,x14,0 得旧 x14；bne x8,x10 误跳）
    wire        ex_mem_fwd1, ex_mem_fwd2;

    // ===== 乘除法完成链（D6b）=====
    // 以修复版（Exp2_A_pipeline_fixed）为基准 merge：原 mul_div_flag 电平
    // 标志（is_mul_div 置位、!busy 清除）+ rf_wR_r 全局目标锁存无法正确
    // 完成乘除 WB——busy 回落拍（!busy 由 1→0 持续 1 拍）触发 WB 时乘除
    // 指令仍在 EX（mem_pc 报错、alu_c 组合未就绪）；连续乘除时 rf_wR_r
    // 被下一条覆盖（WReg 报错）。修复版用 busy 下降沿单拍 mul_done_pulse
    // + WB 取 EX 阶段身份（ex_pc/ex_rf_wR/alu_c）+ mul_post 双拍（等 ALU
    // 的 op_r 清除）解决。
    wire        ex_is_mul_div;      // EX 阶段是乘除指令（ex_alu_op >= ALU_MUL）
    wire        id_is_mul_div;      // ID 阶段是乘除指令（alu_op >= ALU_MUL）
    reg         mul_busy_r;         // busy 延迟 1 拍（下降沿检测用）
    wire        mul_done_pulse;     // busy 下降沿单拍 = 乘除完成脉冲
    reg         mul_div_wait;       // mul 进 EX 置位、完成清除（EX 有乘除进行中）
    wire        mul_div_stall;      // EX 乘除未完成 → 三路冻结
    reg         mul_post, mul_post2; // 完成后 2 拍冻结（等 ALU 的 op_r 清除）
    reg  [ 4:0] mem_alu_op;         // MEM 级 alu_op（mul 驻留 MEM 期间防重复 WB）
    wire        mem_is_mul_div;     // MEM 阶段是乘除指令


    /***************************** IF *****************************/
    reg rst_r;  // 取cpu_rst下降沿
    wire first_req = rst_r & !cpu_rst; // 复位信号下降沿，首次取指
    always @(posedge cpu_clk) rst_r <= cpu_rst;


    // D6b：pause_ifetch 移除乘除部分（(mul_div_busy | is_mul_div)）——
    // 乘除冻结期间取指必须持续：PC 冻结在乘除指令的地址（inst_finished 的
    // mul 项保证），总线反复返回该地址的指令副本（IF/ID 冻结挡），乘除
    // 完成/气泡结束后 IF/ID 捕获它、PC 同步前进（修复版 6-ifetch.md 的
    // "哈佛结构取指持续"语义）。若 pause 停总线，总线出现空隙（valid=0），
    // 恢复拍 IF/ID 捕获 NOP → 乘除后继指令（连续乘除的下一条）永久丢失
    // （修复版 Bug 16 Fix 1/6 同源的取指流衔接）。
    wire pause_ifetch  = (mem_is_ld_st | is_ld_st | ex_is_ld_st) & !ld_st_done;
    wire resume_ifetch = ld_st_done | !mul_div_busy;

    assign ifetch_req  = !skip_fetch & !pause_ifetch & (first_req    |    // 复位后首次取指
                                        ifetch_valid |    // 上一条已取回，同时立即取下一条
                                        br      |    // 静态分支预测错误，立即用正确的地址取指
                                        need_redirect |    // D4：分支重定向拍强制请求目标地址
                                        resume_ifetch);   // 数据访存或乘除运算结束，继续取指
    // D4：分支重定向拍用目标地址取指（覆盖顺序地址 pc）；skip_fetch 时不再取指
    // （目标指令已在重定向拍被请求，返回后由 IF/ID 捕获，重复请求会被取两次）
    assign ifetch_addr = need_redirect ? ex_bj_target : pc;
    
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
        .pipline_stop(pipline_stop),
        .npc        (npc),
        .pc4        (pc4)
    );
    
    // 完成inst_fetch之后pc变成npc
    // D4：分支重定向优先于暂停——need_redirect 时强制 PC 更新为目标地址。
    // 否则分支在 EX 解析时若正值访存/乘除冻结（inst_finished=0），PC 保持顺序
    // 地址，冻结结束后 need_redirect 已消失（EX 已 flush）→ PC 顺序前进丢失
    // 重定向目标，执行流落入 fail 路径
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

    // fetch_pc：取指请求地址的1拍延迟，与IROM（1拍延迟读）返回的ifetch_inst同步。
    // ifetch_inst对应的是上一拍请求的地址（ifetch_addr=pc），而pc此刻已前进，
    // 若直接用pc作为ID阶段PC会整体错位一条指令（VCD实测：mem_wR=@0x4的rd但
    // mem_pc=0x8，全流水线PC超前一条指令）
    reg[31:0] fetch_pc;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)       fetch_pc <= 32'h0;
        else if (ifetch_req) fetch_pc <= ifetch_addr;
    end


    // ID阶段的源寄存器号必须取自流水线寄存器id_inst（而非IF阶段的inst）：
    // inst随IF阶段每拍变化，若用inst做冒险检测的源寄存器号，暂停期间（IF
    // 仍在向前取指）id_rs1/id_rs2会变成IF阶段新指令的寄存器号，与ID阶段
    // 真正等待的指令不一致，RAW冒险漏检/误检
    assign id_rs1 = id_inst[19:15];
    assign id_rs2 = id_inst[24:20];


    //将IF阶段取值和下一指令的PC存起来
    always@(posedge cpu_clk or posedge cpu_rst)begin
      if(cpu_rst || need_redirect) begin   // D4：分支重定向时 flush IF/ID（替代原 br——
        id_pc <= 32'b0;                    // br=1 但无需重定向（目标==ID 已持指令、
        id_inst <= 32'b0;                  // 分支等 load 数据）时 flush 会丢正确路径指令）
      end
      else begin
        if(pipline_stop) begin
          id_pc <= id_pc;
          id_inst <= id_inst;
        end
        else if (ifetch_valid & ((id_pc == fetch_pc) | (ex_pc == fetch_pc) |
                 // D6b（修复版 Bug 16 Fix 5）：连续 load 链中 PC 冻结在 load
                 // 自身地址（前一条 load 的 ex_is_ld_st 把 PC 冻结在"load 后
                 // 第二条"的地址，该地址恰是本条 load 的地址），load 驻留 MEM
                 // 期间总线反复返回它自身（ID/EX 均已气泡化，id/ex 检查不
                 // 命中）——mul 测试 test_6 的 lw x19 @0xa8 在 ID/EX 双气泡
                 // 时被重复捕获执行（CMP 38 起流错位：R 报 @0xac/WReg=20）。
                 // mem_is_ld_st 限定不误伤自循环分支等普通 mem_pc 命中。
                 (mem_is_ld_st & (mem_pc == fetch_pc)) |
                 // 上述场景的残留副本：load 完成拍后 1 拍（ld_done=1）总线
                 // 上还有冻结期最后 1 个请求（load 自身地址）的输出，此时
                 // load 已离开 MEM（mem 检查不命中），按 ld_pc_val 身份丢弃。
                 (ld_done & (fetch_pc == ld_pc_val))) &
                 (fetch_pc != 32'h0)) begin
          // 总线副本丢弃（D4 配套 + D6b 补 mem/ld_done 两层）：重定向拍
          // skip_fetch 使取指停 1 拍 → inst_valid 1 拍延迟产生 1 拍 valid 空隙
          // → inst_finished=0 → PC 冻结 1 拍 → 同一地址被请求 2 拍 → 总线连续
          // 2 个捕获窗返回同一指令（VCD 实测 addi x2@0x8 被 IF/ID 捕获两次 →
          // EX 重复捕获 → MEM 重复 WB → CMP 3 失配）。总线返回的地址与
          // ID/EX/MEM 中已有指令相同时为副本，插气泡丢弃（id 已被气泡清空
          // 时靠 ex_pc 命中——修复版同款结构）
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
    // D6b：mul_div_flag 电平标志已被 mul_done_pulse（busy 下降沿单拍）完成
    // 链替代（WB/inst_finished 均改用它），此处保留赋值块作记录（无消费方）。
    // 电平标志的 "!busy 回落" 在 busy 回落后持续多拍，作为 WB 触发会在乘除
    // 指令已离开 EX 后误发（busy 回落拍 alu_c 组合未就绪）——修复版 Bug 16
    // 的原始根因，现已由 mul_done_pulse 解决。
    assign is_mul_div = is_mul | is_div;
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if      (cpu_rst)       mul_div_flag <= 1'b0;
        else if (is_mul_div)    mul_div_flag <= 1'b1;
        else if (!mul_div_busy) mul_div_flag <= 1'b0;
    end

    // 访存、乘除法指令无法在1个时钟内执行完，故先把指令的目标寄存器缓存起来
    // is_ld_st/is_mul_div由ID阶段指令译码（id_inst），故目标寄存器号必须取
    // id_inst[11:7]——若仍取inst[11:7]，会把IF阶段下一指令的rd锁存进rf_wR_r
    // D6b：rf_wR_r 的乘除消费已改为 ex_rf_wR（EX 阶段自有 rd）——全局锁存
    // 在连续乘除/ld-st 时被下一条指令覆盖（mul 测试 CMP 3：WReg=10 应=6）。
    // ld 消费同样已由 ld_dest（进 MEM 首拍锁存）替代（D6a）。保留赋值块
    // 作记录（无消费方）。
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
    always@(posedge cpu_clk or posedge cpu_rst)begin
        if(cpu_rst || need_redirect)begin   // D4：分支重定向时 flush ID/EX（替代原 br）
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
                // D6b：连续乘除（mul/div 紧跟 mul/div）完成拍插 1 拍气泡。
                // ALU 的 multiplier/divider 用 flag（ex_alu_op>=0x10）上升沿
                // 产生 start 脉冲（第 1 轮 D2 merge）；连续乘除的 flag 在
                // 前一条完成、下一条立即进 EX 时永不回落，start 边沿不触发
                // → 运算器永不启动 → busy 恒 0 → mul_done_pulse 永不出现
                // → 流水线死锁（修复版 Bug 16 Fix 6：test_6 4 条连续 mul
                // @0xbc..0xc8 的 VCD 铁证）。气泡让 flag 在 mul_post 窗口内
                // 回落（EX=NOP），下一条乘除进 EX 时获得自己的 start 上升沿。
                // 同拍 pipline_stop 的 (mul_done_pulse & id_is_mul_div) 冻结
                // IF/ID、inst_finished 的 mul 项冻结 PC——ID 中下一条乘除
                // 保持、总线反复返回其地址副本，气泡结束拍（pulse+3）三路
                // 同步前进：ID/EX 捕获下一条乘除、IF/ID 捕获下下条、PC+4。
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
        end
    end

    // D6a：alu_a/alu_b 赋值移至 MEM 段（需引用 mem_rf_wR/mem_rf_we/mem_data_ok/
    // mem_fwd_val——声明在 MEM 段之后，避免 use-before-declaration）

    assign ex_is_ld_st = (ex_ram_rop != `RAM_EXT_N) | (ex_ram_wop != `RAM_WE_N);

    assign rs1_id_ex_hazard = (ex_rf_wR == id_rs1) & ex_rf_we & id_rf1 & (ex_rf_wR != 5'h0);
    assign rs2_id_ex_hazard = (ex_rf_wR == id_rs2) & ex_rf_we & id_rf2 & (ex_rf_wR != 5'h0);
    // 用当前ID指令的id_rf1/id_rf2（而非alua_sel/alub_sel）判断"ID阶段指令是否读取rs1/rs2"——
    // LUI 的 rs1/rs2 位段是立即数的一部分，若用 !alua_sel/!alub_sel 会让伪寄存器号误判为已读

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
    // 原实现问题（修复版 Bug 14 定位的同类根因）：mem_is_ld_st 无冻结——
    // 无依赖 load 1 拍滑过 MEM，rvalid 晚到（Data_RAM 的 data_valid<=|ren
    // 为 1 拍延迟）时 MEM 已持后继指令，WB 事件按 mem_pc 记录（值/寄存器
    // 正确、PC 报后继指令）；alu_c_r/ram_rop_r 每拍重锁（连续 ld/st 时被
    // 下一条覆盖，MEXT 字节偏移/扩展类型错）。
    // ld_pending：load/store 进 MEM 置位（首拍后），rvalid/wresp 清除。
    // 注意置位条件含 !daccess_rvalid——若读请求未单拍门控（见下方
    // daccess_ren），残留 rvalid 会卡住置位（Bug 16 Fix 4 教训）。
    // D6b 加 !ld_done：完成（rvalid/wresp 已回）但指令因 EX/MEM 冻结
    // （mul_div_stall 等）仍驻留 MEM 时，禁止重新置位（否则 ld_done
    // 的"完成"身份丢失，daccess_ren 经 !ld_pending 门控重发请求 →
    // rvalid 第二次 → 重复 WB——mul 测试 test_6 CMP 43 铁证：lw x23
    // @0xb8 完成驻留期 ld_pending 重置 + ren 重发 → x23 第二次 WB）。
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)                    ld_pending <= 1'b0;
        else if (daccess_rvalid | daccess_wresp) ld_pending <= 1'b0;
        else if (!daccess_rvalid & !daccess_wresp & mem_is_ld_st & !ld_pending & !ld_done) ld_pending <= 1'b1;
    end
    // ld_done：完成脉冲（rvalid/wresp 的 posedge 置位，保持 1 拍）。
    // D6b 自保持：完成但指令仍驻留 MEM（mem_pc==ld_pc_val——EX/MEM 被
    // mul_div_stall 等冻结，完成指令无法离开）期间持续=1，直到指令离开
    // MEM（mem_is_ld_st=0 清除）——ld_pending 重置/请求重发的抑制依据。
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) ld_done <= 1'b0;
        else if (mem_is_ld_st & ld_done & (mem_pc == ld_pc_val)) ld_done <= 1'b1;
        else         ld_done <= ld_pending & (daccess_rvalid | daccess_wresp);
    end
    // 指令身份锁存（进 MEM 首拍，ld_pending 置位前）：WB 的 rd 与
    // debug_wb_pc 用 ld_dest/ld_pc_val——连续 ld/st（store→load 链、mul
    // 测试 test_6 的 8 条连续 lw）时 rf_wR_r 全局锁存被下一条覆盖，必须
    // 用本条指令自己的锁存。alu_c_r/ram_rop_r 同理只锁首拍。
    always @(posedge cpu_clk) begin
        if (mem_is_ld_st & !ld_pending) begin
            ld_dest   <= mem_rf_wR;
            ld_pc_val <= mem_pc;
            alu_c_r   <= mem_alu_c;
            ram_rop_r <= mem_ram_rop;
        end
    end
    // mem_ld_stall：load/store 在 MEM 且未完成 → 冻结 EX/MEM（load 驻留等
    // rvalid/wresp，防 1 拍滑过）。rvalid/wresp 到达时组合放行（恢复拍
    // EX/MEM 与 ID/EX 同步前进）。ld_done 只在"完成指令仍驻留 MEM"
    // （mem_pc==ld_pc_val——完成拍 EX/MEM 因多周期等原因保持）时放行：
    // store→load 链中完成拍 EX/MEM 前进、新 load 进 MEM，ld_done 不能
    // 解除新指令的冻结（修复版 Bug 15 Fix 1：sw 测试 CMP 7 教训）。
    assign mem_ld_stall = mem_is_ld_st & !(ld_done & (mem_pc == ld_pc_val)) &
                          !(daccess_rvalid | daccess_wresp);

    // ===== 乘除法完成链（D6b）=====
    // 修复版 Bug 16 Fix 1/2/6 的完整机制：
    // - ex_is_mul_div 组合检测：mul 进 EX 的第一拍立即生效（multiplier 的
    //   busy 第二拍才=1，仅用 busy 会让 mul 第一拍被挤出 EX，WB 无效结果）
    // - mul_done_pulse（busy 下降沿单拍）：完成拍 ① WB 触发（rf_we1 term2，
    //   数据/PC/目标取 EX 阶段）② EX/MEM 放行（mul 进 MEM——mem_is_mul_div
    //   防驻留期间重复 WB）③ ID/EX 放行（EX 更新为后继指令）④ mul_div_wait
    //   清除。用下降沿而不用 !busy 电平：!busy 在回落前持续多拍，电平触发
    //   会在乘除指令离开 EX 后误发（修复版原始 mul 测试 CMP 4 签名：
    //   R: PC=0x0/WBValue=0——busy 回落拍 WB 时 MEM 为空、alu_c 未就绪）
    // - mul_post/mul_post2（完成拍后 2 拍冻结 EX/MEM/ID/EX/IF/ID + PC）：
    //   等 ALU 的 op_r 清除（op_r 在 start 回落 2 拍后清 0），否则 EX/MEM
    //   捕获的 mem_alu_c 是乘除残留值（后续普通指令写回错数据）
    // - mul_div_wait：mul 进 EX 置位、完成清除——"EX 有乘除进行中"标志
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

    // D6a：EX 侧 MEM 前递 + alu_a/alu_b 选择器（修复版 Bug 14 Fix 5）。
    // load-use 依赖指令在 load_use 拍被 ID/EX 捕获（EX 中 load 的目标==ID
    // 源，数据未到只能捕获旧值），随后在 EX 冻结等数据；rvalid 到达后
    // （mem_data_ok=1）按 EX 自己的源寄存器号（ex_rs1/ex_rs2）重新前递，
    // 否则依赖指令用旧操作数计算（lw 测试：addi x6,x14,0 得到旧 x14；
    // mul 测试：lw x8 → bne x8,x10 在 EX 等待数据期间 x8=0 误跳 fail）。
    // 正常流水（未冻结）时该前递与 ID/EX 捕获时的一致，等价无害。
    assign ex_mem_fwd1 = (mem_rf_wR == ex_rs1) & mem_rf_we & (ex_rs1 != 5'h0) & mem_data_ok;
    assign ex_mem_fwd2 = (mem_rf_wR == ex_rs2) & mem_rf_we & (ex_rs2 != 5'h0) & mem_data_ok;
    assign alu_a = ex_alu_a_sel ? ex_pc  : (ex_mem_fwd1 ? mem_fwd_val : ex_rd1);
    assign alu_b = ex_alu_b_sel ? ex_ext : (ex_mem_fwd2 ? mem_fwd_val : ex_rd2);

    // Interface to Bus
    // D6a：读/写请求只发 1 拍（进 MEM 首拍，ld_pending 置位前）。未门控时
    // load/store 驻留 MEM（mem_ld_stall 冻结）期间每拍重发 da_ren/da_wen，
    // Data_RAM 的 data_valid/data_wresp 是请求的 1 拍延迟副本（每拍=1，
    // 连续多拍）——下一条 ld/st 在残留 rvalid/wresp 期间进 MEM 会被
    // ld_pending 置位条件（!rvalid）卡住、被 mem_ld_stall 放行条件
    // （!rvalid）放走，1 拍滑过 MEM 从未 WB；store 的 debug_mem_we 也会
    // 每拍重复记录（Trace 失配）（修复版 Bug 16 Fix 4 / Bug 15 同类）。
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst) begin
            daccess_ren   <= 4'h0;
            daccess_wen   <= 4'h0;
        end else begin
            // D6b：门控加 !ld_done——完成（rvalid/wresp 已回）但仍驻留 MEM
            // 的 load/store 不重发请求（重发会产生第二次 rvalid/wresp →
            // 重复 WB / debug_mem_we 重复记录，mul 测试 test_6 CMP 43 场景）
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
    // 分支/跳转在 EX 阶段才完成求值（br 由 ALU 比较得出、JALR 目标由 ALU 加法
    // 得出），而 NPC 的 op/offset 取 ID 阶段指令的译码——分支在 EX 解析时 ID
    // 已持分支后继指令（npc_op=NPC_PC4），ID 的 NPC 输出顺序地址，EX 的 br 被
    // 忽略。解法：EX 捕获分支类型（ex_npc_op）与目标，need_redirect 时把
    // PC/取指地址直接改到目标，同时 flush IF/ID 与 ID/EX 中错误路径的指令。
    assign ex_bj_f = (ex_npc_op == `NPC_JMP) |
                     (ex_npc_op == `NPC_JALR) |
                     ((ex_npc_op == `NPC_BRA) & br);
    assign ex_bj_target = (ex_npc_op == `NPC_JALR) ? {alu_c[31:1], 1'b0} :
                          (ex_pc + ex_ext);
    // ex_br_pending：EX 分支的源寄存器 == MEM 中 load 的目标且数据未到
    // （rvalid 未回）时，br 由旧操作数求出（不可信），需抑制重定向，等数据
    // 到达（rvalid=1，fwd 数据生效、br 重算）后再重定向——否则 PC 被指到错误
    // 目标（修复版 mul 测试：lw x8 → bne x8,x10，bne 在 EX 等待数据期间
    // x8=0，误跳 fail 路径）。注：队友 D5 框架下依赖 load 的分支会被冻结在
    // ID（mem_ld_pending_hazard 冻结 ID/EX）直到数据就绪，此信号为防御性
    // 保护（与 D6 暂停体系重构联动时必要）
    assign ex_br_pending = mem_is_ld_st & !daccess_rvalid & mem_rf_we &
                           (mem_rf_wR != 5'h0) &
                           ((mem_rf_wR == ex_rs1) | (mem_rf_wR == ex_rs2));
    // 仅当目标 != ID 中已捕获的指令（顺序路径 = 分支+4）时才重定向：
    // 目标恰为分支+4 时 ID 中的指令就是正确路径，无需重定向、不可 flush
    assign need_redirect = ex_bj_f & (ex_bj_target != id_pc) & !ex_br_pending;

    // skip_fetch：重定向拍已用 ifetch_addr=ex_bj_target 请求了目标指令，下一拍
    // PC=目标地址，若照常取指会重复请求目标（目标被取两次），故跳过 1 拍取指
    // （目标指令在 skip 拍返回并由 IF/ID 捕获——Inst_ROM 的 inst_valid=req 的
    // 1 拍延迟保证该拍 valid=1 且内容为目标指令）
    // D6a 限定 !pause_ifetch：分支重定向与多周期冻结重叠时（load/store 驻留
    // MEM 或乘除运算中，pause_ifetch=1 → ifetch_req 被 !pause 挡住，重定向拍
    // 未发出目标请求），无需 skip——否则恢复拍（rvalid/wresp）的
    // resume_ifetch 请求被 skip_fetch 挡住，目标地址从未被请求（重定向丢失、
    // 取指流卡死）。分支测试（无多周期在途）时 pause=0，行为与原先一致。
    always @(posedge cpu_clk or posedge cpu_rst) begin
        if (cpu_rst)                        skip_fetch <= 1'b0;
        else if (need_redirect & !pause_ifetch) skip_fetch <= 1'b1;
        else                                skip_fetch <= 1'b0;
    end

    /***************************** WB *****************************/
    // 写回使能信号：
    // - Load指令：ld_pending（本条 ld/st 进 MEM 的身份标志）且 rvalid 时写回
    //   （D6a：原 ld_st_flag 电平标志在连续 ld/st 时无法区分"哪条指令完成"——
    //   store→load 链中 store 的 wresp 会清掉 load 阶段仍需要的标志，load WB
    //   丢失；且无冻结时 rvalid 晚到，WB 已按后继指令记录）
    // - 乘除法指令：mul_done_pulse（busy 下降沿单拍）时写回（D6b——原
    //   mul_div_flag & !mul_div_busy 电平项在 busy 回落拍触发，此时乘除指令
    //   仍在 EX（mem_pc 报错）、alu_c 组合未就绪（WBValue=0），且电平标志
    //   在乘除离开 EX 后仍可能触发误 WB）
    // - 其他单周期指令：mem_rf_we置位且MEM中非多周期指令时写回
    //   （D6a：不再用 ifetch_valid 门控——load/store 冻结期间取指总线被
    //   pause 冻结（ifetch_valid=0），load 后面的指令在 MEM 中的 WB 若依赖
    //   ifetch_valid 会被丢失（lw 测试 CMP 8：addi x1@0x24 在 MEM 期间总线
    //   已因 lw@0x28 进 ID 而 pause，WB 丢失）。WB 事件按 MEM 阶段内容判定，
    //   与取指总线无关（修复版 normal_wb 无 valid 门控）。正常流水时
    //   ifetch_valid 恒=1，时序与原来一致（29 个通过测试无回归）。
    //   !mem_is_ld_st：load/store 驻留 MEM 冻结期间每拍 mem_rf_we=1，排除
    //   防重复 WB；load 的 WB 由第一项 (ld_pending & daccess_rvalid) 处理。
    //   D6b：!mem_is_mul_div——mul 完成拍后进 MEM 驻留 mul_post 窗口期间
    //   mem_rf_we=1（mul 的 rf_we），必须排除防重复 WB（mul 的 WB 已由
    //   mul_done_pulse 在 EX 阶段完成））
    //   注意：此处不再依赖ID阶段的is_ld_st/is_mul_div，避免多周期指令阻塞前一条指令的写回
    assign rf_we1 = (ld_pending & daccess_rvalid) |
                    mul_done_pulse |
                    mem_rf_we & !mem_is_ld_st & !mem_is_mul_div;

    // D6a：load WB 的目标寄存器用进 MEM 首拍锁存的 ld_dest（rf_wR_r 全局
    // 锁存会被下一条 ld/st 在 ID 阶段的 is_ld_st 覆盖——mul 测试 test_6 的
    // 连续 lw 后，前一条 lw 的 WB 报后一条的 rd）
    // D6b：乘除 WB 用 EX 阶段自己的目标寄存器 ex_rf_wR（完成拍组合值——
    // 乘除完成时指令仍在 EX，mem_rf_wR 是前一条指令的；rf_wR_r 全局锁存
    // 会被下一条乘除/ld-st 在 ID 阶段覆盖）
    assign rf_wR  = (ld_pending & daccess_rvalid) ? ld_dest :
                    mul_done_pulse ? ex_rf_wR :
                    mem_rf_wR;

    // 写回数据选择：
    // - WB_ALU: ALU计算结果（使用mem_pc+4计算返回地址，使用mem_ext作为立即数）
    // - WB_RAM: 访存读取数据（由ld_pending控制）
    // 必须用组合逻辑（always @(*)，阻塞赋值）而非posedge锁存：rf_we1/rf_wR
    // 均为组合信号，指令X在MEM的当拍即产生WB事件（下个posedge RF采样写入）。
    // 若rf_wD为posedge锁存（每拍锁存上一拍MEM的数据），WB当拍RF采样到的
    // 仍是上一条指令的值——整个WB数据恒滞后一条指令（实测WBValue恒为jal
    // @0x0的PC+4=0x4）。组合逻辑下WB当拍rf_wD即当前MEM指令的数据。
    // D6a：load 分支用 (ld_pending & daccess_rvalid) 而非 ld_st_flag——
    // 1) 连续 ld/st 时 ld_st_flag 电平无法标识"哪条 load 在完成"（store→load
    //    链中 store 的 wresp 清除 ld_st_flag 后，下一条 load 完成时 casex 落
    //    default 写 0，load 数据丢失）；
    // 2) casex 的 {1'b1, 2'b??} 分支用 ld_st_flag 电平会在 load 位于 ID/EX
    //    阶段（ld_st_flag 已置位、MEM 中是 load 后面的普通指令）时把该普通
    //    指令的 WB 数据劫持为 ram_ext（lw 测试 CMP 8：addi x1@0x24 在 MEM
    //    期间 lw@0x28 在 ID，ld_st_flag=1 → addi 的 WB 数据变成 DRAM[0] 的
    //    0x0040006f）。load 的 WB 完全由第一分支 (ld_pending & daccess_rvalid)
    //    处理（ram_rop_r/alu_c_r 进 MEM 首拍锁存保证扩展正确），casex 仅按
    //    mem_rf_wel 分源即可（WB_RAM 分支为防御性，正常不会命中——load 在
    //    MEM 时第一分支已覆盖，且 term3 的 !mem_is_ld_st 排除重复 WB）。
    always @(*) begin
        if (ld_pending & daccess_rvalid)
            rf_wD = ram_ext;    // Load data（进 MEM 首拍锁存的 ram_rop_r/alu_c_r 保证正确）
        else if (mul_done_pulse)
            rf_wD = alu_c;      // D6b：乘除结果（EX 组合输出——完成拍组合值
                                // 即运算结果；mem_alu_c 要等 mul_post 窗口
                                // 结束后才捕获到正确值，不能用于 WB）
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
    // 前递检测：EX/MEM/WB 中正在产生结果（写回）的指令 rd 与 ID 源寄存器匹配时，
    // 直接把数据旁路到 ID 操作数，无需停顿。优先级：EX > MEM > WB > 寄存器堆原值。
    // EX 前递排除 load（load 结果要到 MEM 返回才有，用下方 load_use 停顿等待）。
    assign ex_is_load = (ex_ram_rop != `RAM_EXT_N);
    assign fwd_ex_rs1 = (ex_rf_wR == id_rs1) & ex_rf_we & id_rf1 & (ex_rf_wR != 5'h0) & !ex_is_load;
    assign fwd_ex_rs2 = (ex_rf_wR == id_rs2) & ex_rf_we & id_rf2 & (ex_rf_wR != 5'h0) & !ex_is_load;
    // EX 前递值按写回来源分源：WB_EXT(LUI)→ex_ext；WB_PC4(JAL/JALR)→ex_pc+4；其余→ALU结果
    assign ex_fwd_val = (ex_rf_wel == `WB_EXT) ? ex_ext :
                        (ex_rf_wel == `WB_PC4) ? (ex_pc + 32'h4) :
                                                  alu_c;
    // MEM 前递：数据就绪才前递（MEM 中 load 的 ram_ext 需 daccess_rvalid 到达）
    assign mem_data_ok = !mem_is_ld_st | daccess_rvalid;
    assign mem_fwd_val = mem_is_ld_st ? ram_ext :
                         (mem_rf_wel == `WB_EXT) ? mem_ext :
                         (mem_rf_wel == `WB_PC4) ? (mem_pc + 32'h4) :
                                                    mem_alu_c;
    assign fwd_mem_rs1 = (mem_rf_wR == id_rs1) & mem_rf_we & id_rf1 & (mem_rf_wR != 5'h0) & mem_data_ok;
    assign fwd_mem_rs2 = (mem_rf_wR == id_rs2) & mem_rf_we & id_rf2 & (mem_rf_wR != 5'h0) & mem_data_ok;
    // WB 前递（最低优先级）：WB 事件（rf_we1）成立时 rf_wD 即为当前写回数据
    assign fwd_wb_rs1 = (rf_wR == id_rs1) & rf_we1 & id_rf1 & (rf_wR != 5'h0);
    assign fwd_wb_rs2 = (rf_wR == id_rs2) & rf_we1 & id_rf2 & (rf_wR != 5'h0);
    // 前递后操作数（优先级：EX > MEM > WB > 寄存器堆）
    assign fwd_r1 = fwd_ex_rs1 ? ex_fwd_val :
                    fwd_mem_rs1 ? mem_fwd_val :
                    fwd_wb_rs1  ? rf_wD :
                                  rf_rd1;
    assign fwd_r2 = fwd_ex_rs2 ? ex_fwd_val :
                    fwd_mem_rs2 ? mem_fwd_val :
                    fwd_wb_rs2  ? rf_wD :
                                  rf_rd2;

    // 流水线停顿：仅前递无法解决的冒险才停顿——
    // 1) load-use：EX 中 load 的目标 == ID 源寄存器（数据要等 MEM 返回，前递不可行）
    // 2) MEM 中 load 数据未到（mem_data_ok=0）且 ID 依赖它（fwd_mem 被门控，需等待）
    // 3) D6a：load/store 驻留 MEM 等 rvalid/wresp（mem_ld_stall）
    // 普通 RAW 冒险（ALU/LUI/JAL 等 EX/MEM/WB 结果）全部由前递解决，不停顿。
    assign load_use = ex_is_load &
                      ((ex_rf_wR == id_rs1) & id_rf1 | (ex_rf_wR == id_rs2) & id_rf2) &
                      (ex_rf_wR != 5'h0);
    assign mem_ld_pending_hazard = (rs1_id_mem_hazard | rs2_id_mem_hazard) & !mem_data_ok;
    // pipline_stop：PC+IF/ID 冻结（保证 ID 中等待的指令不被覆盖）。
    // D6a：load_use 移出——load_use 拍 ID/EX 前进（load 进 MEM、依赖指令进 EX）
    // 时 IF/ID 也必须前进（总线已 pause 为空，捕获 NOP 气泡），否则 ID 保持
    // 依赖指令副本（id_pc==ex_pc），恢复拍 ID/EX 与 EX/MEM 同时前进会把
    // 依赖指令双重推进（EX<=ID 且 MEM<=EX，执行两次）。PC 冻结由
    // inst_finished=0 保证（pause 期间 ifetch_valid=0，见 inst_finished 定义）。
    // mem_ld_stall 项加 (id_pc != ex_pc) 限定（修复版 id_stall 同款）：
    // store 完成拍 resume 请求的下一条指令（如 lui@0x1c）在 lw 冻结期到达
    // 总线——若 IF/ID 无条件冻结会丢失该指令（sw 测试 CMP 7：lui x7@0x1c
    // 丢失，执行流落入 0x20，VCD 逐拍验证）。id_pc==ex_pc（ID 是 EX 的
    // 副本/气泡，无未捕获指令）时允许 IF/ID 捕获总线上的新鲜指令；
    // id_pc!=ex_pc（ID 持尚未进 EX 的指令）时冻结防覆盖。
    // mem_pipline_stop：ID/EX 冻结（EX 中 load 不被依赖指令覆盖；load 驻留
    // MEM 时等数据）。load_use 移出同上——ID/EX 前进让 load 从 EX 进 MEM
    // 发出访存请求，依赖指令以旧值进 EX、由 ex_mem_fwd 在数据到达时重算。
    // wb_pipline_stop：EX/MEM 冻结（load/store 驻留 MEM 直到 rvalid/wresp——
    // mem_ld_stall 使无依赖 load 也不再 1 拍滑过，WB 按本条指令的
    // ld_pc_val 记录；同时保证 fwd_mem 数据可用）
    // D6b：三路暂停加入乘除项（对应修复版 pc_stall/id_stall/ex_stall/
    // mem_stall 的 mul 项）：
    // - mul_div_stall：乘除驻留 EX 等运算完成——三路全冻结（EX/MEM 冻结
    //   防 mul 被挤出；ID/EX 冻结防 EX 中 mul 被后继覆盖；IF/ID 冻结防
    //   总线副本覆盖 ID——乘除指令的 WB 在完成拍由 mul_done_pulse 直接
    //   产生，指令本身可不必进 MEM，但冻结保持流水线身份链完整）
    // - mul_post/mul_post2：完成拍后 2 拍冻结（等 ALU op_r 清除，见 D6b
    //   完成链注释）；同时冻结 PC（inst_finished 的 term3 在 IF/ID 冻结时
    //   id_pc!=fetch_pc 自动为 0）与 IF/ID（ID 中乘除后继指令不被总线
    //   副本覆盖——修复版 Bug 16 Fix 2：addi x28 被 dup-check 气泡覆盖）
    // - mul_done_pulse & id_is_mul_div：连续乘除完成拍 IF/ID 冻结（ID 中
    //   下一条乘除保持，总线反复返回其地址副本，气泡结束后捕获——修复版
    //   Bug 16 Fix 6）。完成拍 ID/EX 走气泡分支（ID/EX always 块），
    //   EX/MEM 正常放行（mul 进 MEM）——该拍 mem_pipline_stop/wb_pipline_stop
    //   的 mul 项均为 0（pulse 时 mul_div_stall=0、mul_post 未置位）
    assign pipline_stop     = mem_ld_pending_hazard | (mem_ld_stall & (id_pc != ex_pc)) |
                              mul_div_stall | mul_post | mul_post2 |
                              (mul_done_pulse & id_is_mul_div);
    assign mem_pipline_stop = mem_ld_pending_hazard | mem_ld_stall |
                              mul_div_stall | mul_post | mul_post2;
    assign wb_pipline_stop  = mem_ld_pending_hazard | mem_ld_stall |
                              mul_div_stall | mul_post | mul_post2;
    // 指令完成信号：
    // - 访存指令：ld_st_flag置位且读写完成
    // - 乘除法指令：mul_div_flag置位且运算完成
    // - 单周期指令：ifetch_valid有效且当前ID阶段不是多周期指令（避免多周期指令在标志位置位前被误认为完成）
    //   D6a 新增 (id_pc == fetch_pc) 限定：PC 前进必须对应"刚捕获的真实指令"——
    //   多周期冻结/恢复期间 IF/ID 会捕获 NOP 气泡（id_pc 为冻结地址、inst 为
    //   NOP），气泡未消耗取指流（无对应请求），其"完成"若触发 NPC 的 pc+4
    //   前进会跳过尚未请求的地址（sw 测试 CMP 8：lui@0x1c 捕获后 ID 中气泡
    //   (0x18,NOP) 在 ifetch_valid=1（lui 返回）时误触发 inst_finished，
    //   pc 0x20→0x24 跳过 addi@0x20，VCD 逐拍验证）。气泡的 id_pc 与
    //   fetch_pc 不一致（fetch_pc 已随恢复请求前进），故可精确排除。
    // D6b：inst_finished 的乘除项改 mul_done_pulse（busy 下降沿单拍）——
    // 原 mul_div_flag & !mul_div_busy 电平项在乘除指令位于 ID/EX 期间
    // 持续=1（mul_div_flag 在 ID 即置位、busy 未启动时 !busy=1），PC 每拍
    // 前进，把乘除后继指令甩在总线/丢失（修复版 Bug 16 Fix 1：mul 测试
    // CMP 6，lui@0x18 丢失）。改为完成拍单脉冲后：
    // - mul 在 ID/EX 期间 term2=0 → PC 冻结（取指流停在乘除地址）
    // - 完成拍（pulse）term2=1 → PC 前进（请求乘除后继指令）
    // - 连续乘除完成拍（ID 仍是乘除）term2 被 !id_is_mul_div 排除 → PC 冻结
    //   （总线保持下一条乘除的地址副本，气泡结束后捕获）
    // - term4（id_is_mul_div & !mul_div_wait & pc==id_pc+4 & !mul_post/
    //   !mul_post2）：连续乘除气泡恢复拍 PC 前进——PC 冻结在 id_pc+4
    //   （下一条乘除的地址，总线反复返回它），IF/ID 捕获它时 PC 必须前进
    //   到 id_pc+8（=新 ID+4）使 dup-check 错位、总线请求下下条（修复版
    //   pc_stall 同款限定的逆语义：修复版冻结 PC 于 pc==id_pc+8、放行于
    //   pc==id_pc+4；此处 PC 使能 = 修复版"放行"拍的事件）
    assign inst_finished = ld_st_flag   & ld_st_done    |
                           mul_done_pulse & !id_is_mul_div |
                           (id_is_mul_div & !mul_div_wait & (pc == (id_pc + 32'h4)) &
                            !mul_post & !mul_post2) |
                           ifetch_valid & !is_ld_st & !is_mul_div & (id_pc == fetch_pc) &
                           !mem_ld_stall & !ex_is_ld_st;
    // D6b（修复版 Bug 16 Fix 5 链上修复）：term4 加 !mem_ld_stall——load/store
    // 驻留 MEM 期间 PC 必须冻结（修复版 ld_stall_comb 语义）。mul 测试 test_6
    // 连续 load 链（8 条连续 lw @0x9c..0xb8）的 VCD 铁证：lw x18 @0xa4 进 MEM
    // 等 rvalid 期间，ID 是气泡（id_pc=0xa4）、总线返回 x18 副本（IF/ID 经
    // dup-check 插气泡丢弃），但组合 term4 的 (id_pc==fetch_pc)（0xa4==0xa4）
    // 恰好命中 → inst_finished=1 → PC 从 0xa8 跳到 0xac → lw x19 @0xa8 从未
    // 被请求 → 执行流错位（CMP 38：R 报 @0xac/WReg=20，lw x19 丢失）。
    // !mem_ld_stall 后：x18 驻留期间 PC 冻结在 0xa8，rvalid 恢复拍
    // resume_ifetch 请求 0xa8、lw x19 正常捕获。
    // D6b 再加 !ex_is_ld_st（修复版 pc_stall 的 ex_is_ld_st 项）：load/store
    // 在 EX 阶段时 PC 也必须冻结——否则取指流超前（lw x23 @0xb8 在 EX 时 PC
    // 已前进到 0xbc 并捕获 mul@0xbc 进 ID，x23 进 MEM 时 mul 同拍进 EX），
    // x23 完成（rvalid）时 EX=mul 运算中（mul_div_stall 冻结 EX/MEM）→ x23
    // 驻留 MEM 已完成的指令 → ld_pending 重新置位（置位条件 mem_is_ld &
    // !ld_pending）→ daccess_ren 重发（门控 !ld_pending）→ rvalid 第二次
    // → 第二次 WB（CMP 43：R 重复报 @0xb8/x23/0xffffffe4）。修复版对照
    // （VCD 逐拍）：lw x23 在 EX 时 PC 冻结在 0xb8、总线反复返回 x23 副本
    // （IF/ID dup-check 插气泡、ID 保持气泡），x23 进 MEM 时 EX=气泡，
    // x23 完成拍 EX 为空 → EX/MEM 正常前进、x23 离开 → mul 才进 EX。


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

    assign debug_wb_pc    = (ld_pending & daccess_rvalid) ? ld_pc_val :  // D6a：load WB 用
                                      // 进 MEM 首拍锁存的 PC（load 驻留 MEM 等
                                      // rvalid，此时 mem_pc 仍是本条 load 的 PC，
                                      // 与 mem_pc 一致；锁存值在连续 ld/st 链中
                                      // 保证 identity——修复版同款）
                            mul_done_pulse ? ex_pc :                    // D6b：乘除 WB 用
                                      // EX 阶段 PC（乘除完成时指令仍在 EX，
                                      // mem_pc 是前一条指令的或为空——
                                      // mul 测试 CMP 4 原签名 R: PC=0x0）
                            mem_pc;   // WB阶段PC：应连MEM阶段PC。连pc（IF阶段）时，
                                      // 写回发生时pc已超前数拍，Trace比对CMP 1即失配
    assign debug_wb_rf_we = rf_we1;
    assign debug_wb_rf_wR = rf_wR;
    assign debug_wb_rf_wD = rf_wD;

    assign debug_mem_pc    = mem_pc;  // MEM阶段PC：同理应连mem_pc而非IF阶段pc
    assign debug_mem_we    = daccess_wen;
    assign debug_mem_waddr = daccess_addr;
    assign debug_mem_wdata = daccess_wdata;
`endif

endmodule
