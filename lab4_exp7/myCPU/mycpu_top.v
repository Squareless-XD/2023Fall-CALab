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

    reg         reset;
    always @(posedge clk) reset <= ~resetn;

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
    wire [31:0] br_offs; // branch offset
    wire [31:0] jirl_offs; // jump and link (instruction: jirl) offset

    // instruction parts
    wire [ 5:0] op_31_26;
    wire [ 3:0] op_25_22;
    wire [ 1:0] op_21_20;
    wire [ 4:0] op_19_15;
    wire [ 4:0] rd;
    wire [ 4:0] rj;
    wire [ 4:0] rk;
    wire [11:0] i12;
    wire [19:0] i20;
    wire [15:0] i16;
    wire [25:0] i26;

    // instruction parts after decode
    wire [63:0] op_31_26_d;
    wire [15:0] op_25_22_d;
    wire [ 3:0] op_21_20_d;
    wire [31:0] op_19_15_d;

    // instruction name
    wire        inst_add_w;
    wire        inst_sub_w;
    wire        inst_slt;
    wire        inst_sltu;
    wire        inst_nor;
    wire        inst_and;
    wire        inst_or;
    wire        inst_xor;
    wire        inst_slli_w;
    wire        inst_srli_w;
    wire        inst_srai_w;
    wire        inst_addi_w;
    wire        inst_ld_w;
    wire        inst_st_w;
    wire        inst_jirl;
    wire        inst_b;
    wire        inst_bl;
    wire        inst_beq;
    wire        inst_bne;
    wire        inst_lu12i_w;

    wire        need_ui5;  // unsigned immediate, 5 bit
    wire        need_si12; // signed immediate, 12 bit
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
    wire [Stage_Num - 1 : 0] pipe_stay;         // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    wire [Stage_Num - 1 : 0] pipe_refresh;      // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    wire [Stage_Num - 1 : 0] pipe_leave;        // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
    wire [Stage_Num - 1 : 0] pipe_enter;        // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.


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
    assign pipe_stay    = pipe_valid & ~{1'b1, pipe_tonext_valid};
    assign pipe_refresh = pipe_valid & {pipe_tonext_valid, 1'b1}; // NOTE: 1'b1 is for PC refresh
    assign pipe_leave   = pipe_valid & {1'b1, pipe_tonext_valid};
    assign pipe_enter   = {pipe_valid[3:0], 1'b1} & {pipe_tonext_valid, pipe_allowin[0]}; // NOTE: 1'b1 is for PC refresh

    // valid signal control in pipeline
    always @(posedge clk) begin
        if (reset) begin
            pipe_valid <= 5'b00000;
        end
        else begin
            pipe_valid <= {pipe_tonext_valid, pipe_allowin[0]} | pipe_stay;
        end
    end


    /* --------------------------------------
        IF stage
    -------------------------------------- */

    // pc register
    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'h1bfffffc;     // trick: to make nextpc be 0x1c000000 during reset 
        end
        else begin
            pc <= nextpc;
        end
    end

    assign inst = inst_sram_rdata; // instruction memory read data

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
        inst_ID <= inst;
        pc_ID   <= pc;
    end

    /* --------------------------------------
        ID stage
    -------------------------------------- */

    // cut the instruction into several parts
    assign op_31_26 = inst_ID[31:26];
    assign op_25_22 = inst_ID[25:22];
    assign op_21_20 = inst_ID[21:20];
    assign op_19_15 = inst_ID[19:15];

    // register numbers
    assign rd  = inst_ID[ 4: 0];
    assign rj  = inst_ID[ 9: 5];
    assign rk  = inst_ID[14:10];

    // different immediate values
    assign i12 = inst_ID[21:10];
    assign i20 = inst_ID[24: 5];
    assign i16 = inst_ID[25:10];
    assign i26 = {inst_ID[ 9: 0], inst_ID[25:10]};

    // decode the instruction parts
    decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
    decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
    decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
    decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

    // decode the instruction
    assign inst_add_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
    assign inst_sub_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
    assign inst_slt     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
    assign inst_sltu    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
    assign inst_nor     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
    assign inst_and     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
    assign inst_or      = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
    assign inst_xor     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
    assign inst_slli_w  = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
    assign inst_srli_w  = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
    assign inst_srai_w  = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
    assign inst_addi_w  = op_31_26_d[6'h00] & op_25_22_d[4'ha];
    assign inst_ld_w    = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
    assign inst_st_w    = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
    assign inst_jirl    = op_31_26_d[6'h13];
    assign inst_b       = op_31_26_d[6'h14];
    assign inst_bl      = op_31_26_d[6'h15];
    assign inst_beq     = op_31_26_d[6'h16];
    assign inst_bne     = op_31_26_d[6'h17];
    assign inst_lu12i_w = op_31_26_d[6'h05] & ~inst_ID[25];

    // calculate alu operation control signal
    assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                        | inst_jirl | inst_bl;
    assign alu_op[ 1] = inst_sub_w;
    assign alu_op[ 2] = inst_slt;
    assign alu_op[ 3] = inst_sltu;
    assign alu_op[ 4] = inst_and;
    assign alu_op[ 5] = inst_nor;
    assign alu_op[ 6] = inst_or;
    assign alu_op[ 7] = inst_xor;
    assign alu_op[ 8] = inst_slli_w;
    assign alu_op[ 9] = inst_srli_w;
    assign alu_op[10] = inst_srai_w;
    assign alu_op[11] = inst_lu12i_w;


    assign need_ui5  =  inst_slli_w | inst_srli_w | inst_srai_w;
    assign need_si12 =  inst_addi_w | inst_ld_w | inst_st_w;
    assign need_si16 =  inst_jirl | inst_beq | inst_bne;
    assign need_si20 =  inst_lu12i_w;
    assign need_si26 =  inst_b | inst_bl;
    assign src2_is_4 =  inst_jirl | inst_bl;

    assign imm = src2_is_4 ? 32'h4                      :
                 need_si20 ? {i20[19:0], 12'b0}         :
    /*need_ui5 || need_si12*/{{20{i12[11]}}, i12[11:0]} ;

    assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                                 {{14{i16[15]}}, i16[15:0], 2'b0} ;

    assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

    assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w;

    assign src1_is_pc = inst_jirl | inst_bl;

    assign src2_is_imm = inst_slli_w  |
                         inst_srli_w  |
                         inst_srai_w  |
                         inst_addi_w  |
                         inst_ld_w    |
                         inst_st_w    |
                         inst_lu12i_w |
                         inst_jirl    |
                         inst_bl      ;

    assign res_from_mem = inst_ld_w;
    assign dst_is_r1    = inst_bl;
    assign gr_we        = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b;
    assign mem_we       = inst_st_w;
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

    assign rj_value  = rf_rdata1;
    assign rkd_value = rf_rdata2;

    assign pipe_ready_go[1] = pipe_valid[1];


    /* --------------------------------------
        ID -> EX
    -------------------------------------- */

    reg [11:0] alu_op_EX;
    reg [31:0] rj_value_EX;
    reg [31:0] rkd_value_EX;
    reg [31:0] pc_EX;

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
    // reg inst_lu12i_w_EX;

    reg [31:0] imm_EX;
    reg [31:0] br_offs_EX;
    reg [31:0] jirl_offs_EX;
    reg src1_is_pc_EX;
    reg src2_is_imm_EX;

    reg mem_we_EX;
    reg res_from_mem_EX;
    reg gr_we_EX;
    reg [4:0] dest_EX;

    always @(posedge clk) begin
        alu_op_EX    <= alu_op;
        rj_value_EX  <= rj_value;
        rkd_value_EX <= rkd_value;
        pc_EX        <= pc_ID;

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
        // inst_lu12i_w_EX <= inst_lu12i_w;

        imm_EX           <= imm;
        br_offs_EX       <= br_offs;
        jirl_offs_EX     <= jirl_offs;
        src1_is_pc_EX    <= src1_is_pc;
        src2_is_imm_EX   <= src2_is_imm;
    
        mem_we_EX       <= mem_we;
        res_from_mem_EX <= res_from_mem;
        gr_we_EX        <= gr_we;
        dest_EX         <= dest;
    end

    /* --------------------------------------
        EX stage
    -------------------------------------- */

    assign rj_eq_rd = (rj_value_EX == rkd_value_EX); // for branch instructions
    assign br_taken = (
        inst_beq_EX  &&  rj_eq_rd ||
        inst_bne_EX  && !rj_eq_rd ||
        inst_jirl_EX              ||
        inst_bl_EX                ||
        inst_b_EX
    ) && pipe_valid[2];

    assign br_target = (inst_beq_EX || inst_bne_EX || inst_bl_EX || inst_b_EX) ? (pc_EX + br_offs_EX) :
                                                                   /*inst_jirl*/ (rj_value_EX + jirl_offs_EX);

    assign alu_src1 = src1_is_pc_EX ? pc_EX[31:0] : rj_value_EX;
    assign alu_src2 = src2_is_imm_EX ? imm_EX : rkd_value_EX;

    alu u_alu(
        .alu_op     (alu_op_EX ),
        .alu_src1   (alu_src1  ),
        .alu_src2   (alu_src2  ),
        .alu_result (alu_result)
    );

    // pre-MEM stage 
    assign data_sram_en    = 1'b1;                              // data memory enable
    assign data_sram_we    = {4{mem_we_EX && pipe_valid[2]}};   // data memory byte-writes
    assign data_sram_addr  = alu_result;                        // data memory address
    assign data_sram_wdata = rkd_value_EX;                      // data memory write data

    assign pipe_ready_go[2] = pipe_valid[2];


    /* --------------------------------------
        EX -> MEM
    -------------------------------------- */

    reg [31:0] pc_MEM;
    reg [31:0] alu_result_MEM;

    reg res_from_mem_MEM;
    reg gr_we_MEM;
    reg [4:0] dest_MEM;

    always @(posedge clk) begin
        pc_MEM         <= pc_EX;
        alu_result_MEM <= alu_result;

        res_from_mem_MEM <= res_from_mem_EX;
        gr_we_MEM        <= gr_we_EX;
        dest_MEM         <= dest_EX;
    end

    /* --------------------------------------
        MEM stage
    -------------------------------------- */

    assign mem_result = data_sram_rdata;

    assign pipe_ready_go[3] = pipe_valid[3];


    /* --------------------------------------
        MEM -> WB
    -------------------------------------- */

    reg [31:0] pc_WB;
    reg [31:0] alu_result_WB;
    reg [31:0] mem_result_WB;

    reg res_from_mem_WB;
    reg gr_we_WB;
    reg [4:0] dest_WB;

    always @(posedge clk) begin
        if (reset) begin
            pc_WB         <= 32'h0;
            alu_result_WB <= 32'h0;
            mem_result_WB <= 32'h0;

            res_from_mem_WB <= 1'b0;
            gr_we_WB        <= 1'b0;
            dest_WB         <= 5'h0;
        end
        else begin
            pc_WB         <= pc_MEM;
            alu_result_WB <= alu_result_MEM;
            mem_result_WB <= mem_result;

            res_from_mem_WB <= res_from_mem_MEM;
            gr_we_WB        <= gr_we_MEM;
            dest_WB         <= dest_MEM;
        end
    end

    /* --------------------------------------
        WB stage
    -------------------------------------- */

    assign final_result = res_from_mem_WB ? mem_result_WB : alu_result_WB;

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
