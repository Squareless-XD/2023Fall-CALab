`define BYTE    8
`define HALF    16
`define WORD    32
`define WIDTH   32
`define R0      5'b00000

module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,    // instruction SRAM port enable pin
    output wire [ 3:0] inst_sram_we,    // instruction SRAM byte-writes
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,    // data SRAM port enable pin
    output wire [ 3:0] data_sram_we,    // data SRAM byte-writes
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);

    /* --------------------------------------
        Declarations
    -------------------------------------- */

    reg reset;
    always @(posedge clk) begin
        reset <= ~resetn;
    end

    // reg         valid;
    // always @(posedge clk) begin
    //     if (reset) begin
    //         valid <= 1'b0;
    //     end
    //     else begin
    //         valid <= 1'b1;
    //     end
    // end

    wire [31:0] seq_pc; // sequential PC value, PC+4
    wire [31:0] nextpc; // next PC value, branch target
    wire        br_taken; // branch taken or not
    wire [31:0] br_target; // branch target, PC+4 or branch target
    wire [31:0] inst; // instruction
    reg  [31:0] pc; // PC

    wire [11:0] alu_op; // alu operation control signal
    // wire        load_op;
    wire        src1_is_pc; // whether the first operand of ALU is PC
    wire        src2_is_imm; // whether the second operand of ALU is immediate
    wire        res_from_mem; // whether the result comes from memory
    wire        dst_is_r1; // whether the destination register is r1
    wire        gr_we;  // general register write enable
    wire        mem_we; // memory write enable
    wire        src_reg_is_rd; // whether the source register is rd
    wire [ 4:0] dest; // destination register number
    wire [31:0] rj_value; // value of register rj
    wire [31:0] rkd_value; // value of register rk or rd
    wire [31:0] imm; // immediate value
    // wire [31:0] br_offs; // branch offset
    // wire [31:0] jirl_offs; // jump and link (instruction: jirl) offset

    // instruction parts
    wire [ 5:0] op_31_26;
    wire [ 3:0] op_25_22;
    wire [ 6:0] op_21_15;
    // wire [ 4:0] op_19_15;
    wire [ 4:0] rd;
    wire [ 4:0] rj;
    wire [ 4:0] rk;
    wire [11:0] imm12;
    wire [15:0] imm16;
    wire [19:0] imm20;
    wire [25:0] imm26;

    // instruction parts after decode
    // wire [63:0] op_31_26_d;
    // wire [15:0] op_25_22_d;
    // wire [ 3:0] op_21_20_d;
    // wire [31:0] op_19_15_d;

    // instruction name
    wire        inst_add_w;
    wire        inst_sub_w;
    wire        inst_slt;
    wire        inst_sltu;
    wire        inst_nor;
    wire        inst_and;
    wire        inst_or;
    wire        inst_xor;
    wire        inst_sll_w;
    wire        inst_srl_w;
    wire        inst_sra_w;
    wire        inst_slli_w;
    wire        inst_srli_w;
    wire        inst_srai_w;
    wire        inst_slti;
    wire        inst_sltui;
    wire        inst_addi_w;
    wire        inst_andi;
    wire        inst_ori;
    wire        inst_xori;
    wire        inst_lu12i_w;
    wire        inst_pcaddu12i;

    // wire        inst_ld_b;
    // wire        inst_ld_h;
    // wire        inst_ld_w;
    // wire        inst_st_b;
    // wire        inst_st_h;
    // wire        inst_st_w;
    // wire        inst_ld_bu;
    // wire        inst_ld_hu;

    wire        inst_jirl;
    wire        inst_b;
    wire        inst_bl;
    wire        inst_beq;
    wire        inst_bne;
    wire        inst_blt;
    wire        inst_bge;
    wire        inst_bltu;
    wire        inst_bgeu;

    wire        need_ui5;  // unsigned immediate, 5 bit
    wire        need_si12; // signed immediate, 12 bit
    wire        need_ui12; // new
    wire        need_si16; // signed immediate, 16 bit
    wire        need_si20; // signed immediate, 20 bit
    wire        need_si26; // signed immediate, 26 bit
    wire        src2_is_4; // whether the second operand of ALU is 4

    wire [ 4:0] rf_raddr1; // register file read address 1
    wire [31:0] rf_rdata1; // register file read data 1
    wire [ 4:0] rf_raddr2; // register file read address 2
    wire [31:0] rf_rdata2; // register file read data 2
    wire        rf_we;     // register file write enable
    wire [ 4:0] rf_waddr;  // register file write address
    wire [31:0] rf_wdata;  // register file write data

    wire [31:0] alu_src1;   // ALU source 1
    wire [31:0] alu_src2;   // ALU source 2
    wire [31:0] alu_result; // ALU result

    wire [31:0] mem_result; // the result read from memory (data SRAM)
    wire [31:0] final_result; // the final result that will be written to register file


    // pipeline controllers 
    reg  [Stage_Num - 1 : 0] pipe_valid;        // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    wire [Stage_Num - 1 : 0] pipe_allowin;      // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    wire [Stage_Num - 1 : 0] pipe_ready_go;     // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    wire [Stage_Num - 2 : 0] pipe_tonext_valid; // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB. "4" bits
    // wire [Stage_Num - 1 : 0] pipe_stay;         // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    // wire [Stage_Num - 1 : 0] pipe_refresh;      // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    // wire [Stage_Num - 1 : 0] pipe_leave;        // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    // wire [Stage_Num - 1 : 0] pipe_enter;        // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.





    /* --------------------------------------------------------------------
        Pipeline control (except pipe_ready_go[Stage_Num - 1 : 1])
    -------------------------------------------------------------------- */


    genvar i;

    parameter Stage_Num = 5;
    assign pipe_allowin[Stage_Num - 2 : 0] = ~pipe_valid[Stage_Num - 2 : 0]
                                           | pipe_ready_go[Stage_Num - 2 : 0] & pipe_allowin[Stage_Num - 1 : 1]
    ;
    assign pipe_allowin[Stage_Num - 1] = ~pipe_valid[Stage_Num - 1] | pipe_ready_go[Stage_Num - 1];

    assign pipe_tonext_valid[Stage_Num - 2 : 0] = pipe_allowin[Stage_Num - 1 : 1] & pipe_ready_go[Stage_Num - 2 : 0];
    // assign pipe_stay    = pipe_valid & ~{1'b1, pipe_tonext_valid};
    // assign pipe_refresh = pipe_valid & {pipe_tonext_valid, 1'b1}; // NOTE: 1'b1 is for PC refresh
    // assign pipe_leave   = pipe_valid & {1'b1, pipe_tonext_valid};
    // assign pipe_enter   = {pipe_valid[3:0], 1'b1} & {pipe_tonext_valid, pipe_allowin[0]}; // NOTE: 1'b1 is for PC refresh

    // valid signal control in pipeline
    always @(posedge clk) begin
        if (reset) begin
            pipe_valid <= 5'b00000;
        end
        else begin
            if (pipe_allowin[0]) begin
                pipe_valid[0] <= 1'b1;
            end
            if (br_taken) begin
                pipe_valid[1] <= 1'b0;
                if (pipe_tonext_valid[2]) begin
                    pipe_valid[2] <= 1'b0;
                end
            end
            else  begin
                if (pipe_allowin[1]) begin
                    pipe_valid[1] <= pipe_ready_go[0];
                end
                if (pipe_allowin[2]) begin
                    pipe_valid[2] <= pipe_ready_go[1];
                end
            end
            if (pipe_allowin[3]) begin
                pipe_valid[3] <= pipe_ready_go[2];
            end
            if (pipe_allowin[4]) begin
                pipe_valid[4] <= pipe_ready_go[3];
            end
        end
    end




    /* --------------------------------------
        IF stage
    -------------------------------------- */

    // pc register
    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'h1bfffffc; // trick: to make nextpc be 0x1c000000 during reset
        end
        else if (br_taken) begin
            pc <= nextpc;
        end
        else if (pipe_allowin[0]) begin
            pc <= nextpc;
        end
    end

    assign inst = inst_sram_rdata; // instruction memory read data

    // store inst into a register, in case of IF staying more than 1 cycle
    reg first_IF;
    reg [31:0] inst_IF_reg;
    always @(posedge clk) begin
        if (reset) begin
            first_IF <= 1'b1;
        end
        else if (br_taken) begin
            first_IF <= 1'b1;
        end
        else if (pipe_tonext_valid[0]) begin
            first_IF <= 1'b1;
        end
        else begin
            first_IF <= !pipe_valid[0];
        end
    end

    always @(posedge clk) begin
        if (first_IF) begin
            inst_IF_reg <= inst;
        end
    end


    /* pre-if stage begin */

    // next PC calculation
    assign seq_pc = pc + 3'h4; // PC+4, the next PC value.
    assign nextpc = br_taken ? br_target : seq_pc; // calculate the next PC value. 

    // instruction memory (SRAM)
    assign inst_sram_en    = 1'b1;      // instruction memory enable
    assign inst_sram_we    = 4'b0000;   // instruction memory byte-writes
    assign inst_sram_addr  = nextpc;    // instruction memory address
    assign inst_sram_wdata = 32'b0;     // instruction memory write data

    /* pre-if stage end */

    assign pipe_ready_go[0] = pipe_valid[0];



    /* --------------------------------------
        IF -> ID
    -------------------------------------- */

    reg [31:0] inst_ID;
    reg [31:0] pc_ID;

    always @(posedge clk) begin
        if (pipe_tonext_valid[0]) begin
            if (first_IF) begin
                inst_ID <= inst;
            end else begin
                inst_ID <= inst_IF_reg;
            end
            pc_ID   <= pc;
        end
    end

    /* --------------------------------------
        ID stage
    -------------------------------------- */

    // cut the instruction into several parts
    assign op_31_26 = inst_ID[31:26];
    assign op_25_22 = inst_ID[25:22];
    assign op_21_15 = inst_ID[21:15];
    // assign op_19_15 = inst_ID[19:15];

    // register numbers
    assign rd  = inst_ID[ 4: 0];
    assign rj  = inst_ID[ 9: 5];
    assign rk  = inst_ID[14:10];

    // different immediate values
    assign imm12 = inst_ID[21:10];
    assign imm16 = inst_ID[25:10];
    assign imm20 = inst_ID[24: 5];
    assign imm26 = {inst_ID[ 9: 0], inst_ID[25:10]};

    wire br_inst;       // branch & jump
    wire ld_inst;       // load
    wire st_inst;       // store
    wire lu_inst;       // load upper
    wire tlb_inst;      // translation lookaside buffer
    wire ertn_inst;     // exception return
    wire cache_inst;    // cache operation
    wire csr_inst;      // control and status register
    wire alui_inst;     // ALU immediate
    wire sfti_inst;     // shifter immediate
    wire syscall_inst;  // system call
    wire div_inst;      // divide
    wire mul_inst;      // multiply
    wire sftr_inst;     // shifter register
    wire alur_inst;     // ALU register
    wire rdcn_inst;     // read control and status register


    assign br_inst = op_31_26[4];
    assign ld_inst = (op_31_26[4:3] == 2'b01) & ~op_25_22[2];
    assign st_inst = (op_31_26[4:3] == 2'b01) &  op_25_22[2];
    assign lu_inst = (op_31_26[4:2] == 3'b001);
    assign tlb_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0001)
                    && ({op_25_22[3], op_25_22[0]} == 2'b11)
                    && (op_21_15[0] || (rk[2] ^ rk[1]))
    ;
    assign ertn_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0001)
                     && ({op_25_22[3], op_25_22[0]} == 2'b11)
                     && (~op_21_15[0] && rk[2] && rk[1])
    ;
    assign cache_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0001)
                      && ({op_25_22[3], op_25_22[0]} == 2'b10)
    ;
    assign csr_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0001)
                    && (~op_25_22[3])
    ;
    assign alui_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
                     && (op_25_22[3])
    ;
    assign sfti_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
                     && ({op_25_22[3], op_25_22[0]} == 2'b01)
    ;
    assign syscall_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
                        && ({op_25_22[3], op_25_22[0]} == 2'b00)
                        && ({op_21_15[6], op_21_15[4]} == 2'b11)
    ;
    assign div_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
                    && ({op_25_22[3], op_25_22[0]} == 2'b00)
                    && ({op_21_15[6], op_21_15[4]} == 2'b10)
    ;
    assign mul_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
                    && ({op_25_22[3], op_25_22[0]} == 2'b00)
                    && ({op_21_15[5:3]} == 4'b111)
    ;
    assign sftr_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
                     && ({op_25_22[3], op_25_22[0]} == 2'b00)
                     && (({op_21_15[5:3]} == 3'b110) || ({op_21_15[6:2]} == 5'b01011))
    ;
    assign alur_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
                     && ({op_25_22[3], op_25_22[0]} == 2'b00)
                     && (op_21_15[5:4] == 2'b10)
                     && !(op_21_15[3] && op_21_15[2])
    ;
    assign rdcn_inst = ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
                     && ({op_25_22[3], op_25_22[0]} == 2'b00)
                     && (op_21_15[6:5] == 2'b00)
                    //  && (op_21_15[3:2] == 2'b11)
    ;

    // // decode the instruction parts
    // decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
    // decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
    // decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
    // decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

    // // decode the instruction
    // assign inst_add_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
    // assign inst_sub_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
    // assign inst_slt     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
    // assign inst_sltu    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
    // assign inst_nor     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
    // assign inst_and     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
    // assign inst_or      = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
    // assign inst_xor     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
    // assign inst_slli_w  = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
    // assign inst_srli_w  = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
    // assign inst_srai_w  = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
    // assign inst_addi_w  = op_31_26_d[6'h00] & op_25_22_d[4'ha];
    // assign inst_ld_w    = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
    // assign inst_st_w    = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
    // assign inst_jirl    = op_31_26_d[6'h13];
    // assign inst_b       = op_31_26_d[6'h14];
    // assign inst_bl      = op_31_26_d[6'h15];
    // assign inst_beq     = op_31_26_d[6'h16];
    // assign inst_bne     = op_31_26_d[6'h17];
    // assign inst_lu12i_w = op_31_26_d[6'h05] & ~inst_ID[25];

    assign inst_add_w   = alur_inst && (op_21_15[3:1] == 3'b000);
    assign inst_sub_w   = alur_inst && (op_21_15[3:1] == 3'b001);
    assign inst_slt     = alur_inst && ({op_21_15[2], op_21_15[0]} == 2'b10);
    assign inst_sltu    = alur_inst && ({op_21_15[2], op_21_15[0]} == 2'b11);
    assign inst_nor     = alur_inst && ({op_21_15[3], op_21_15[1:0]} == 3'b100);
    assign inst_and     = alur_inst && ({op_21_15[3], op_21_15[1:0]} == 3'b101);
    assign inst_or      = alur_inst && ({op_21_15[3], op_21_15[1:0]} == 3'b110);
    assign inst_xor     = alur_inst && ({op_21_15[3], op_21_15[1:0]} == 3'b111);
    assign inst_sll_w   = sftr_inst && (op_21_15[1:0] == 2'b10);
    assign inst_srl_w   = sftr_inst && (op_21_15[1:0] == 2'b11);
    assign inst_sra_w   = sftr_inst && (op_21_15[1:0] == 2'b00);

    assign inst_slli_w  = sfti_inst && (op_21_15[4:3] == 2'b00);
    assign inst_srli_w  = sfti_inst && (op_21_15[4:3] == 2'b01);
    assign inst_srai_w  = sfti_inst && (op_21_15[4:3] == 2'b10);
    assign inst_slti    = alui_inst && (op_25_22[2:0] == 3'b000);
    assign inst_sltui   = alui_inst && (op_25_22[2:0] == 3'b001);
    assign inst_addi_w  = alui_inst && (op_25_22[2:0] == 3'b010);
    assign inst_andi    = alui_inst && (op_25_22[2:0] == 3'b101);
    assign inst_ori     = alui_inst && (op_25_22[2:0] == 3'b110);
    assign inst_xori    = alui_inst && (op_25_22[2:0] == 3'b111);
    assign inst_lu12i_w   = lu_inst && ~op_31_26[1];
    assign inst_pcaddu12i = lu_inst &&  op_31_26[1];

    // assign inst_ld_b    = ld_inst && (op_25_22 == 4'b0000);
    // assign inst_ld_h    = ld_inst && (op_25_22 == 4'b0001);
    // assign inst_ld_w    = ld_inst && (op_25_22 == 4'b0010);
    // assign inst_st_b    = st_inst && (op_25_22 == 4'b0100);
    // assign inst_st_h    = st_inst && (op_25_22 == 4'b0101);
    // assign inst_st_w    = st_inst && (op_25_22 == 4'b0110);
    // assign inst_ld_bu   = ld_inst && (op_25_22 == 4'b1000);
    // assign inst_ld_hu   = ld_inst && (op_25_22 == 4'b1001);

    assign inst_jirl = br_inst && (op_31_26[3:2] == 2'b00);
    assign inst_b    = br_inst && (op_31_26[2:0] == 3'b100);
    assign inst_bl   = br_inst && (op_31_26[2:0] == 3'b101);
    assign inst_beq  = br_inst && (op_31_26[2:0] == 3'b110);
    assign inst_bne  = br_inst && (op_31_26[2:0] == 3'b111);
    assign inst_blt  = br_inst && ({op_31_26[3], op_31_26[1:0]} == 3'b100);
    assign inst_bge  = br_inst && ({op_31_26[3], op_31_26[1:0]} == 3'b101);
    assign inst_bltu = br_inst && ({op_31_26[3], op_31_26[1:0]} == 3'b110);
    assign inst_bgeu = br_inst && ({op_31_26[3], op_31_26[1:0]} == 3'b111);

    // calculate alu operation control signal
    // assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
    //                     | inst_jirl | inst_bl;
    // assign alu_op[ 1] = inst_sub_w;
    // assign alu_op[ 2] = inst_slt;
    // assign alu_op[ 3] = inst_sltu;
    // assign alu_op[ 4] = inst_and;
    // assign alu_op[ 5] = inst_nor;
    // assign alu_op[ 6] = inst_or;
    // assign alu_op[ 7] = inst_xor;
    // assign alu_op[ 8] = inst_slli_w;
    // assign alu_op[ 9] = inst_srli_w;
    // assign alu_op[10] = inst_srai_w;
    // assign alu_op[11] = inst_lu12i_w;
    assign alu_op[ 0] = inst_add_w || inst_addi_w || ld_inst || st_inst || inst_jirl || inst_bl || inst_pcaddu12i;
    assign alu_op[ 1] = inst_sub_w;
    assign alu_op[ 2] = inst_slt || inst_slti || inst_blt || inst_bge;
    assign alu_op[ 3] = inst_sltu || inst_sltui || inst_bltu || inst_bgeu;
    assign alu_op[ 4] = inst_and || inst_andi;
    assign alu_op[ 5] = inst_nor;
    assign alu_op[ 6] = inst_or || inst_ori;
    assign alu_op[ 7] = inst_xor || inst_xori;
    assign alu_op[ 8] = inst_sll_w || inst_slli_w;
    assign alu_op[ 9] = inst_srl_w || inst_srli_w;
    assign alu_op[10] = inst_sra_w || inst_srai_w;
    assign alu_op[11] = inst_lu12i_w;


    assign need_ui5  =  inst_slli_w || inst_srli_w || inst_srai_w;
    assign need_si12 =  inst_slti || inst_sltui || inst_addi_w || ld_inst || st_inst;
    assign need_ui12 =  inst_andi || inst_ori || inst_xori;
    assign need_si16 =  br_inst && !(inst_b || inst_bl);
    assign need_si20 =  inst_lu12i_w || inst_pcaddu12i;
    assign need_si26 =  inst_b || inst_bl;
    assign src2_is_4 =  inst_jirl || inst_bl;

    assign imm = {`WIDTH{need_ui5 | need_si12}} & {{20{imm12[11]}}, imm12[11:0]} // ui5 only care the last 5 bits
               | {`WIDTH{need_ui12}}            & { 20'b0         , imm12[11:0]}
               | {`WIDTH{need_si16}}            & {{14{imm16[15]}}, imm16[15:0],  2'b0}
               | {`WIDTH{need_si20}}            & {    imm20[19:0],              12'b0} 
               | {`WIDTH{need_si26}}            & {{ 4{imm26[25]}}, imm26[25:0],  2'b0}
            //    | {`WIDTH{src2_is_4}}            & 32'h4                
    ;

    // assign br_offs = need_si26 ? {{ 4{imm26[25]}}, imm26[25:0], 2'b0} :
    //                              {{14{imm16[15]}}, imm16[15:0], 2'b0} ;

    // assign jirl_offs = {{14{imm16[15]}}, imm16[15:0], 2'b0};

    assign src_reg_is_rd = br_inst && (op_31_26[3] || op_31_26[2] && op_31_26[1]) || st_inst; // actually load don't need to read this register

    assign src1_is_pc = inst_jirl || inst_bl || inst_pcaddu12i;

    assign src2_is_imm = sfti_inst
                      || alui_inst
                      || ld_inst
                      || st_inst
                      || lu_inst
                    //   || inst_jirl
                    //   || inst_bl
    ;
    wire src2_is_4;
    assign src2_is_4 = inst_jirl || inst_bl;

    assign res_from_mem = ld_inst;
    assign dst_is_r1    = inst_bl;
    assign gr_we        = !st_inst && !(br_inst && !inst_jirl && !inst_bl);
    assign mem_we       = st_inst;
    assign dest         = dst_is_r1 ? 5'd1 : rd;

    assign rf_raddr1 = rj;
    assign rf_raddr2 = src_reg_is_rd ? rd : rk;

    regfile u_regfile(
        .clk    (clk      ),
        .raddr1 (rf_raddr1),
        .rdata1 (rf_rdata1),
        .raddr2 (rf_raddr2),
        .rdata2 (rf_rdata2),
        .we     (rf_we    ),
        .waddr  (rf_waddr ),
        .wdata  (rf_wdata )
    );

    wire rf_ren1; // only for blocking ID stage
    wire rf_ren2; // only for blocking ID stage

    assign rf_ren1 = br_inst && !(inst_b || inst_bl)
                  || ld_inst
                  || st_inst
                  || alui_inst
                  || sfti_inst
                  || sftr_inst
                  || alur_inst
    ;

    assign rf_ren2 = br_inst && !(inst_jirl || inst_b || inst_bl)
                //   || ld_inst
                  || st_inst
                //   || alui_inst
                //   || sfti_inst
                  || sftr_inst
                  || alur_inst
    ;

    assign rj_value  = hzd_alu_EX_r1  ? alu_result     :
                       hzd_alu_MEM_r1 ? alu_result_MEM :
                       hzd_alu_WB_r1  ? alu_result_WB  :
                       hzd_ld_WB_r1   ? mem_result     :
                       rf_rdata1
    ;
    assign rkd_value = hzd_alu_EX_r2  ? alu_result     :
                       hzd_alu_MEM_r2 ? alu_result_MEM :
                       hzd_alu_WB_r2  ? alu_result_WB  :
                       hzd_ld_WB_r2   ? mem_result     :
                       rf_rdata2
    ;

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Pipeline Blocking Control
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

    wire rd_EX_r1_eq;
    wire rd_EX_r2_eq;
    wire rd_MEM_r1_eq;
    wire rd_MEM_r2_eq;
    wire rd_WB_r1_eq;
    wire rd_WB_r2_eq;

    assign rd_EX_r1_eq  = (rf_raddr1 != `R0) && (rf_raddr1 == dest_EX)  && pipe_valid[2];
    assign rd_EX_r2_eq  = (rf_raddr2 != `R0) && (rf_raddr2 == dest_EX)  && pipe_valid[2];
    assign rd_MEM_r1_eq = (rf_raddr1 != `R0) && (rf_raddr1 == dest_MEM) && pipe_valid[3];
    assign rd_MEM_r2_eq = (rf_raddr2 != `R0) && (rf_raddr2 == dest_MEM) && pipe_valid[3];
    assign rd_WB_r1_eq  = (rf_raddr1 != `R0) && (rf_raddr1 == dest_WB)  && pipe_valid[4];
    assign rd_WB_r2_eq  = (rf_raddr2 != `R0) && (rf_raddr2 == dest_WB)  && pipe_valid[4];

    wire hzd_alu_EX_r1;
    wire hzd_alu_EX_r2;
    wire hzd_alu_MEM_r1;
    wire hzd_alu_MEM_r2;
    wire hzd_alu_WB_r1;
    wire hzd_alu_WB_r2;
    wire hzd_ld_EX_r1;
    wire hzd_ld_EX_r2;
    wire hzd_ld_MEM_r1;
    wire hzd_ld_MEM_r2;
    wire hzd_ld_WB_r1;
    wire hzd_ld_WB_r2;

    assign hzd_alu_EX_r1  = rd_EX_r1_eq  && gr_we_EX  && !ld_inst_EX;
    assign hzd_alu_EX_r2  = rd_EX_r2_eq  && gr_we_EX  && !ld_inst_EX;
    assign hzd_alu_MEM_r1 = rd_MEM_r1_eq && gr_we_MEM && !ld_inst_MEM;
    assign hzd_alu_MEM_r2 = rd_MEM_r2_eq && gr_we_MEM && !ld_inst_MEM;
    assign hzd_alu_WB_r1  = rd_WB_r1_eq  && gr_we_WB  && !ld_inst_WB;
    assign hzd_alu_WB_r2  = rd_WB_r2_eq  && gr_we_WB  && !ld_inst_WB;
    assign hzd_ld_EX_r1   = rd_EX_r1_eq  && ld_inst_EX;
    assign hzd_ld_EX_r2   = rd_EX_r2_eq  && ld_inst_EX;
    assign hzd_ld_MEM_r1  = rd_MEM_r1_eq && ld_inst_MEM;
    assign hzd_ld_MEM_r2  = rd_MEM_r2_eq && ld_inst_MEM;
    assign hzd_ld_WB_r1   = rd_WB_r1_eq  && ld_inst_WB;
    assign hzd_ld_WB_r2   = rd_WB_r2_eq  && ld_inst_WB;

    wire ID_hazard;
    
    assign ID_hazard = hzd_ld_EX_r1  && rf_ren1
                    || hzd_ld_EX_r2  && rf_ren2
                    || hzd_ld_MEM_r1 && rf_ren1
                    || hzd_ld_MEM_r2 && rf_ren2
    ;

    // assign pipe_block[2] = st_inst_MEM && rf_raddr1_MEM
    assign pipe_ready_go[1] = pipe_valid[1] && !ID_hazard;


    /* --------------------------------------
        ID -> EX
    -------------------------------------- */

    reg [11:0] alu_op_EX;
    reg [31:0] rj_value_EX;
    reg [31:0] rkd_value_EX;
    reg [31:0] pc_EX;

    reg [3:0] op_25_22_EX;
    reg [6:0] op_21_15_EX;

    reg [4:0] rf_raddr1_EX;
    reg [4:0] rf_raddr2_EX;

    // reg br_inst_EX;       // branch & jump
    reg ld_inst_EX;          // load
    reg st_inst_EX;          // store
    // reg lu_inst_EX;       // load upper
    // reg tlb_inst_EX;      // translation lookaside buffer
    // reg ertn_inst_EX;     // exception return
    // reg cache_inst_EX;    // cache operation
    // reg csr_inst_EX;      // control and status register
    // reg alui_inst_EX;     // ALU immediate
    // reg sfti_inst_EX;     // shifter immediate
    // reg syscall_inst_EX;  // system call
    // reg div_inst_EX;      // divide
    // reg mul_inst_EX;      // multiply
    // reg sftr_inst_EX;     // shifter register
    // reg alur_inst_EX;     // ALU register
    // reg rdcn_inst_EX;     // read control and status register

    // reg inst_add_w_EX;
    // reg inst_sub_w_EX;
    // reg inst_slt_EX;
    // reg inst_sltu_EX;
    // reg inst_nor_EX;
    // reg inst_and_EX;
    // reg inst_or_EX;
    // reg inst_xor_EX;
    // reg inst_slli_w_EX;
    // reg inst_srli_w_EX;
    // reg inst_srai_w_EX;
    // reg inst_addi_w_EX;
    // reg inst_ld_w_EX;
    // reg inst_st_w_EX;
    reg inst_jirl_EX;
    reg inst_b_EX;
    reg inst_bl_EX;
    reg inst_beq_EX;
    reg inst_bne_EX;
    reg inst_blt_EX;
    reg inst_bge_EX;
    reg inst_bltu_EX;
    reg inst_bgeu_EX;
    // reg inst_lu12i_w_EX;

    reg [31:0] imm_EX;
    // reg [31:0] br_offs_EX;
    // reg [31:0] jirl_offs_EX;
    reg src1_is_pc_EX;
    reg src2_is_imm_EX;
    reg src2_is_4_EX;
    reg mem_we_EX;
    reg res_from_mem_EX;
    reg gr_we_EX;
    reg [4:0] dest_EX;

    always @(posedge clk) begin
        if (pipe_tonext_valid[1]) begin
            alu_op_EX    <= alu_op;
            rj_value_EX  <= rj_value;
            rkd_value_EX <= rkd_value;
            pc_EX        <= pc_ID;

            op_25_22_EX <= op_25_22;
            op_21_15_EX <= op_21_15;

            rf_raddr1_EX <= rf_raddr1;
            rf_raddr2_EX <= rf_raddr2;

            // br_inst_EX      <= br_inst;
            ld_inst_EX      <= ld_inst;
            st_inst_EX      <= st_inst;
            // lu_inst_EX      <= lu_inst;
            // tlb_inst_EX     <= tlb_inst;
            // ertn_inst_EX    <= ertn_inst;
            // cache_inst_EX   <= cache_inst;
            // csr_inst_EX     <= csr_inst;
            // alui_inst_EX    <= alui_inst;
            // sfti_inst_EX    <= sfti_inst;
            // syscall_inst_EX <= syscall_inst;
            // div_inst_EX     <= div_inst;
            // mul_inst_EX     <= mul_inst;
            // sftr_inst_EX    <= sftr_inst;
            // alur_inst_EX    <= alur_inst;
            // rdcn_inst_EX    <= rdcn_inst;

            // inst_add_w_EX   <= inst_add_w;
            // inst_sub_w_EX   <= inst_sub_w;
            // inst_slt_EX     <= inst_slt;
            // inst_sltu_EX    <= inst_sltu;
            // inst_nor_EX     <= inst_nor;
            // inst_and_EX     <= inst_and;
            // inst_or_EX      <= inst_or;
            // inst_xor_EX     <= inst_xor;
            // inst_slli_w_EX  <= inst_slli_w;
            // inst_srli_w_EX  <= inst_srli_w;
            // inst_srai_w_EX  <= inst_srai_w;
            // inst_addi_w_EX  <= inst_addi_w;
            // inst_ld_w_EX    <= inst_ld_w;
            // inst_st_w_EX    <= inst_st_w;
            inst_jirl_EX    <= inst_jirl;
            inst_b_EX       <= inst_b;
            inst_bl_EX      <= inst_bl;
            inst_beq_EX     <= inst_beq;
            inst_bne_EX     <= inst_bne;
            inst_blt_EX     <= inst_blt;
            inst_bge_EX     <= inst_bge;
            inst_bltu_EX    <= inst_bltu;
            inst_bgeu_EX    <= inst_bgeu;
            // inst_lu12i_w_EX <= inst_lu12i_w;

            imm_EX           <= imm;
            // br_offs_EX       <= br_offs;
            // jirl_offs_EX     <= jirl_offs;
            src1_is_pc_EX    <= src1_is_pc;
            src2_is_imm_EX   <= src2_is_imm;
            src2_is_4_EX     <= src2_is_4;

            mem_we_EX       <= mem_we;
            res_from_mem_EX <= res_from_mem;
            gr_we_EX        <= gr_we;
            dest_EX         <= dest;
        end
    end

    /* --------------------------------------
        EX stage
    -------------------------------------- */

    reg first_EX;
    always @(posedge clk) begin
        if (reset) begin
            first_EX <= 1'b1;
        end
        else if (pipe_tonext_valid[1]) begin
            first_EX <= 1'b1;
        end
        else begin
            first_EX <= 1'b0;
        end
    end
    wire rj_eq_rd;
    assign rj_eq_rd = (rj_value_EX == rkd_value_EX); // for branch instructions
    assign br_taken = (
        inst_jirl_EX                ||
        inst_b_EX                   ||
        inst_bl_EX                  ||
        inst_beq_EX  &&  rj_eq_rd   ||
        inst_bne_EX  && !rj_eq_rd   ||
        inst_blt_EX  &&  alu_result ||
        inst_bge_EX  && !alu_result ||
        inst_bltu_EX &&  alu_result ||
        inst_bgeu_EX && !alu_result
    ) && pipe_valid[2] && first_EX;

    // branch target (base + offset) calculation (use only one adder)
    wire        br_src_sel;
    wire [31:0] br_off_src1;
    wire [31:0] br_off_src2;

    assign br_src_sel = inst_jirl_EX;
    assign br_off_src1 = br_src_sel ? rj_value_EX : pc_EX;
    // assign br_off_src2 = br_src_sel ? br_offs : jirl_offs_EX;
    assign br_off_src2 = imm_EX;

    assign br_target = br_off_src1 + br_off_src2;

    assign alu_src1 = src1_is_pc_EX ? pc_EX[31:0] : rj_value_EX;
    wire src2_is_rkd;
    assign src2_is_rkd = ~(src2_is_imm_EX | src2_is_4_EX);
    assign alu_src2 = {`WIDTH{src2_is_imm_EX}} & imm_EX 
                    | {`WIDTH{src2_is_rkd   }} & rkd_value_EX
                    | {`WIDTH{src2_is_4_EX  }} & 32'h4
    ;

    alu u_alu (
        // .alu_src2_n ( ),
        // .alu_cin    ( ),
        .alu_op     (alu_op_EX ),
        .alu_src1   (alu_src1  ),
        .alu_src2   (alu_src2  ),
        .alu_result (alu_result)
    );

/*
    module alu(
    input  wire        alu_src2_n,
    input  wire        alu_cin,
    input  wire [11:0] alu_op,
    input  wire [31:0] alu_src1,
    input  wire [31:0] alu_src2,
    output wire [31:0] alu_result
);

*/
    assign pipe_ready_go[2] = pipe_valid[2];


    /* --------------------------------------
        EX -> MEM
    -------------------------------------- */

    reg [31:0] pc_MEM;
    reg [31:0] alu_result_MEM;
    reg [31:0] rkd_value_MEM;

    reg [ 3:0] op_25_22_MEM;

    reg [4:0] rf_raddr1_MEM;
    reg [4:0] rf_raddr2_MEM;

    reg ld_inst_MEM;       // load
    reg st_inst_MEM;       // store

    reg res_from_mem_MEM;
    reg gr_we_MEM;
    reg mem_we_MEM;
    reg [4:0] dest_MEM;

    always @(posedge clk) begin
        if (pipe_tonext_valid[2]) begin
            pc_MEM         <= pc_EX;
            alu_result_MEM <= alu_result;
            rkd_value_MEM  <= rkd_value_EX;

            op_25_22_MEM <= op_25_22_EX;

            rf_raddr1_MEM <= rf_raddr1_EX;
            rf_raddr2_MEM <= rf_raddr2_EX;
        
            ld_inst_MEM  <= ld_inst_EX;
            st_inst_MEM  <= st_inst_EX;

            res_from_mem_MEM <= res_from_mem_EX;
            gr_we_MEM        <= gr_we_EX;
            mem_we_MEM       <= mem_we_EX;
            dest_MEM         <= dest_EX;
        end
    end

    /* --------------------------------------
        MEM stage
    -------------------------------------- */

    assign pipe_ready_go[3] = pipe_valid[3];


    wire byte_we;
    wire half_we;
    wire word_we;
    wire signed_we;
    wire [3:0] mask_w_dataram;
    wire [31:0] data_w_dataram;

    // assign inst_ld_b    = ld_inst && (op_25_22 == 4'b0000);
    // assign inst_ld_h    = ld_inst && (op_25_22 == 4'b0001);
    // assign inst_ld_w    = ld_inst && (op_25_22 == 4'b0010);
    // assign inst_st_b    = st_inst && (op_25_22 == 4'b0100);
    // assign inst_st_h    = st_inst && (op_25_22 == 4'b0101);
    // assign inst_st_w    = st_inst && (op_25_22 == 4'b0110);
    // assign inst_ld_bu   = ld_inst && (op_25_22 == 4'b1000);
    // assign inst_ld_hu   = ld_inst && (op_25_22 == 4'b1001);
    assign byte_we = (op_25_22_MEM[1:0] == 2'b00);
    assign half_we =  op_25_22_MEM[0];
    assign word_we =  op_25_22_MEM[1];
    assign signed_we = ~op_25_22_MEM[3];

    assign mask_w_dataram = {
        word_we,
        word_we,
        half_we || word_we,
        byte_we || half_we || word_we
    };

    assign data_w_dataram = {`WIDTH{byte_we}} & {4{rkd_value_MEM[`BYTE - 1 : 0]}}
                          | {`WIDTH{half_we}} & {2{rkd_value_MEM[`HALF - 1 : 0]}}
                          | {`WIDTH{word_we}} &    rkd_value_MEM
    ;

    assign data_sram_en    = 1'b1;              // data memory enable
    assign data_sram_we    = {4{mem_we_MEM && pipe_valid[3]}}
                           & mask_w_dataram
    ;       // data memory byte-writes (not considering disalignment)
    assign data_sram_addr  = alu_result_MEM;    // data memory address
    assign data_sram_wdata = data_w_dataram;    // data memory write data

    /* --------------------------------------
        MEM -> WB
    -------------------------------------- */

    reg [31:0] pc_WB;
    reg [31:0] alu_result_WB;
    // reg [31:0] mem_result_WB;

    // reg [4:0] rf_raddr1_WB;
    // reg [4:0] rf_raddr2_WB;

    reg ld_inst_WB;       // load
    reg st_inst_WB;       // store

    reg byte_we_WB;
    reg half_we_WB;
    reg word_we_WB;
    reg signed_we_WB;

    reg res_from_mem_WB;
    reg gr_we_WB;
    reg [4:0] dest_WB;

    always @(posedge clk) begin
        if (reset) begin
            pc_WB         <= 32'h0;
            // alu_result_WB <= 32'h0;
            // mem_result_WB <= 32'h0;

            res_from_mem_WB <= 1'b0;
            gr_we_WB        <= 1'b0;
            dest_WB         <= 5'h0;
        end
        else if (pipe_tonext_valid[3]) begin
            pc_WB         <= pc_MEM;
            alu_result_WB <= alu_result_MEM;
            // mem_result_WB <= mem_result;
            
            // rf_raddr1_WB <= rf_raddr1_MEM;
            // rf_raddr2_WB <= rf_raddr2_MEM;

            ld_inst_WB <= ld_inst_MEM;
            st_inst_WB <= st_inst_MEM;

            byte_we_WB     <= byte_we;
            half_we_WB     <= half_we;
            word_we_WB     <= word_we;
            signed_we_WB <= signed_we;
        
            res_from_mem_WB <= res_from_mem_MEM;
            gr_we_WB        <= gr_we_MEM;
            dest_WB         <= dest_MEM;
        end
    end


    /* --------------------------------------
        WB stage
    -------------------------------------- */

    // data read from MEM section
    wire [31:0] data_orig;
    wire [31:0] data_r_dataram;
    assign data_orig = data_sram_rdata;
    assign data_r_dataram = {`WIDTH{byte_we_WB}} & {{24{data_orig[ 7] & signed_we_WB}}, data_orig[ 7:0]}
                          | {`WIDTH{half_we_WB}} & {{16{data_orig[15] & signed_we_WB}}, data_orig[15:0]}
                          | {`WIDTH{word_we_WB}} &      data_orig
    ;

    assign mem_result = data_r_dataram;

    assign final_result = res_from_mem_WB ? mem_result : alu_result_WB;

    assign rf_we    = gr_we_WB && pipe_valid[4];
    assign rf_waddr = dest_WB;
    assign rf_wdata = final_result;

    // debug info generate
    assign debug_wb_pc       = pc_WB;
    assign debug_wb_rf_we    = {4{rf_we}};
    assign debug_wb_rf_wnum  = dest_WB;
    assign debug_wb_rf_wdata = final_result;

    assign pipe_ready_go[4] = pipe_valid[4];

endmodule
