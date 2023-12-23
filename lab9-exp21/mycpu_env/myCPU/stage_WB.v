`define RADDR  5
`define BYTE   8
`define HALF   16
`define WORD   32
`define WIDTH  32
`define DWIDTH 64
`define R0     5'b00000
`define TRUE   1'b1
`define FALSE  1'b0
`define LOG2TLBNUM 4

module stage_WB (
    input  wire                  clk,
    input  wire                  reset, // localized reset signal

    // input from stage controller
    input  wire                  pipe_tonext_valid_MEM, // allow WB stage to accept new instruction
    input  wire                  pipe_valid_WB, // WB stage is valid
    input  wire                  time_interupt,

    // input from MEM
    input  wire [`WIDTH - 1 : 0] pc_MEM, alu_div_rdcntv_result_MEM, rj_value_MEM, rkd_value_MEM,
    input  wire [`DWIDTH - 1: 0] mul_result,   // mul
    input  wire [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM,
    input  wire [         3 : 0] mask_dataram,
    input  wire                  byte_we_MEM, half_we_MEM, word_we_MEM, signed_we_MEM,
    input  wire                  csr_inst_MEM, ertn_inst_MEM, rdcn_inst_MEM,
    input  wire                  mul_inst_MEM, // mul
    input  wire                  inst_tlbrd_MEM, inst_tlbwr_MEM, inst_tlbfill_MEM,
    input  wire [         2 : 0] mul_op_MEM, // mul
    input  wire                  rdcn_op_MEM, // rdcn
    input  wire                  res_from_mem_MEM, gpr_we_MEM,
    input  wire [`WIDTH - 1 : 0] ld_res_from_MEM,
    input  wire [         5 : 0] ecode_MEM_m,
    input  wire                  csr_we_MEM, csr_wmask_en_MEM,
    input  wire [        13 : 0] csr_code_MEM,
    input  wire                  has_int_MEM, needs_refresh_MEM,
    input  wire                  exc_from_IF_MEM,

    // input from csr module
    input  wire [`WIDTH - 1 : 0] csr_rvalue,

    // output
    output wire                  gpr_we_WB,

    output wire                  rf_we,    // register file write enable
    output wire [`RADDR - 1 : 0] rf_waddr, // register file write address
    output wire [`RADDR - 1 : 0] dest_WB,  // destination register number
    output wire [`WIDTH - 1 : 0] rf_wdata, // register file write data

    output wire [`WIDTH - 1 : 0] pc_WB,
    output wire [         3 : 0] debug_wb_rf_we,    // debug info
    output wire [`RADDR - 1 : 0] debug_wb_rf_wnum,  // debug info
    output wire [`WIDTH - 1 : 0] debug_wb_rf_wdata, // debug info
    
    output wire [         8 : 0] csr_waddr,
    output wire                  csr_we,
    output wire [`WIDTH - 1 : 0] csr_wmask,
    output wire [`WIDTH - 1 : 0] csr_wvalue,
    output wire [         8 : 0] csr_raddr,
    output wire                  ertn_flush,
    output wire                  WB_ex, needs_refresh_WB,
    output wire [        31 : 0] WB_vaddr,
    output wire [         5 : 0] ecode_noint, // ecode, without considering interrupt
    output wire [         8 : 0] WB_esubcode,

    output wire                  ertn_inst_WB,
    output wire                  has_int_WB,
    output wire                  rdcn_inst_WB,

    // Connect to CSR and TLB, for TLBRD, TLBWR and TLBFILL
    input  wire[`LOG2TLBNUM-1:0] csr_tlbidx_index,
    input  wire [         9 : 0] csr_asid_asid,
    output wire                  csr_tlbidx_ne_we, csr_tlbidx_ps_we, csr_asid_asid_we,
    output wire                  w_tlbidx_ne,
    output wire [         5 : 0] w_tlbidx_ps,
    output wire [         9 : 0] w_asid_asid,
    output wire                  csr_tlbe_we,
    output wire [        18 : 0] w_tlbehi_vppn,
    output wire [`WIDTH - 1 : 0] w_tlbelo0, w_tlbelo1,
    input  wire [        18 : 0] csr_tlbehi_vppn,
    input  wire                  csr_tlbidx_ne,
    input  wire [         5 : 0] csr_tlbidx_ps,
    input  wire [         5 : 0] csr_estat_ecode,
    input  wire [`WIDTH - 1 : 0] csr_tlbelo0, csr_tlbelo1,
    output wire[`LOG2TLBNUM-1:0] tlb_r_index,
    input  wire                  tlb_r_e,
    input  wire [        18 : 0] tlb_r_vppn,
    input  wire [         5 : 0] tlb_r_ps,
    input  wire [         9 : 0] tlb_r_asid,
    input  wire                  tlb_r_g,
    input  wire [        19 : 0] tlb_r_ppn0,
    input  wire [         1 : 0] tlb_r_plv0,
    input  wire [         1 : 0] tlb_r_mat0,
    input  wire                  tlb_r_d0,
    input  wire                  tlb_r_v0,
    input  wire [        19 : 0] tlb_r_ppn1,
    input  wire [         1 : 0] tlb_r_plv1,
    input  wire [         1 : 0] tlb_r_mat1,
    input  wire                  tlb_r_d1,
    input  wire                  tlb_r_v1,
    output wire                  tlb_we, //w(rite) e(nable)
    output wire[`LOG2TLBNUM-1:0] tlb_w_index,
    output wire                  tlb_w_e,
    output wire [        18 : 0] tlb_w_vppn,
    output wire [         5 : 0] tlb_w_ps,
    output wire [         9 : 0] tlb_w_asid,
    output wire                  tlb_w_g,
    output wire [        19 : 0] tlb_w_ppn0,
    output wire [         1 : 0] tlb_w_plv0,
    output wire [         1 : 0] tlb_w_mat0,
    output wire                  tlb_w_d0,
    output wire                  tlb_w_v0,
    output wire [        19 : 0] tlb_w_ppn1,
    output wire [         1 : 0] tlb_w_plv1,
    output wire [         1 : 0] tlb_w_mat1,
    output wire                  tlb_w_d1,
    output wire                  tlb_w_v1
);

    /* --------------------------------------
        MEM -> WB
    -------------------------------------- */

    wire [`WIDTH - 1 : 0] calc_res = mul_inst_MEM 
        ? ( {`WIDTH{mul_op_MEM[1] | mul_op_MEM[2]}} & mul_result[`DWIDTH - 1 : `WIDTH]
          | {`WIDTH{mul_op_MEM[0]                }} & mul_result[`WIDTH  - 1 : 0     ] )
        : alu_div_rdcntv_result_MEM;

    // wire [`WIDTH - 1 : 0] pc_WB;
    // wire [4:0] dest_WB;
    wire [`WIDTH - 1 : 0] rj_value_WB;
    wire [`WIDTH - 1 : 0] rkd_value_WB;

    wire [3:0] mask_dataram_WB;
    wire byte_we_WB;
    wire half_we_WB;
    wire word_we_WB;
    wire signed_we_WB;
    wire [`WIDTH - 1 : 0] calc_res_WB; // ALU result(including mul and div)
    // wire ld_inst_WB; // load
    wire res_from_mem_WB;
    // wire gpr_we_WB;
    wire rdcn_op_WB;
    wire rdcntid_valid;

    wire inst_tlbwr_WB, inst_tlbfill_WB, inst_tlbrd_WB;

    wire exc_from_IF_WB;

    reg [`LOG2TLBNUM - 1 : 0] tlbfill_index_ymr;

    wire ppl_men = pipe_tonext_valid_MEM;

    Reg_norst #(`WIDTH) pc_WB_inst        (clk, pc_MEM,        pc_WB,        ppl_men);
    Reg_norst #(`WIDTH) calc_res_WB_inst  (clk, calc_res,      calc_res_WB,  ppl_men);
    Reg_norst #(`WIDTH) rj_value_WB_ins   (clk, rj_value_MEM,  rj_value_WB,  ppl_men);
    Reg_norst #(`WIDTH) rkd_value_WB_inst (clk, rkd_value_MEM, rkd_value_WB, ppl_men);
    Reg_norst #(`RADDR) dest_WB_inst      (clk, dest_MEM,      dest_WB,      ppl_men);

    Reg_norst #(4) mask_dataram_WB_inst (clk, mask_dataram,  mask_dataram_WB, ppl_men);
    Reg_norst #(1) byte_we_WB_inst      (clk, byte_we_MEM,   byte_we_WB,      ppl_men);
    Reg_norst #(1) half_we_WB_inst      (clk, half_we_MEM,   half_we_WB,      ppl_men);
    Reg_norst #(1) word_we_WB_inst      (clk, word_we_MEM,   word_we_WB,      ppl_men);
    Reg_norst #(1) signed_we_WB_inst    (clk, signed_we_MEM, signed_we_WB,    ppl_men);

    Reg_norst #(1) res_from_mem_WB_inst (clk, res_from_mem_MEM, res_from_mem_WB, ppl_men);
    Reg_norst #(1) exc_from_IF_WB_inst  (clk, exc_from_IF_MEM,  exc_from_IF_WB,  ppl_men);

    Reg #(1, 1'b0) gpr_we_WB_inst (clk, reset, gpr_we_MEM, gpr_we_WB, ppl_men);



    wire [ 5:0] ecode_WB;
    wire [13:0] csr_code_WB;
    wire        csr_we_WB;
    wire        csr_wmask_en_WB;
    wire        csr_inst_WB;
    // wire        rdcn_inst_WB;
    // wire        has_int_WB;
    // wire        ertn_inst_WB;
    Reg_norst #(6 ) ecode_WB_inst         (clk, ecode_MEM_m,       ecode_WB,         ppl_men);
    Reg_norst #(14) csr_code_WB_inst      (clk, csr_code_MEM,      csr_code_WB,      ppl_men);
    Reg_norst #(1 ) csr_we_WB_inst        (clk, csr_we_MEM,        csr_we_WB,        ppl_men);
    Reg_norst #(1 ) csr_wmask_en_WB_inst  (clk, csr_wmask_en_MEM,  csr_wmask_en_WB,  ppl_men);
    Reg_norst #(1 ) csr_inst_WB_inst      (clk, csr_inst_MEM,      csr_inst_WB,      ppl_men);
    Reg_norst #(1 ) ertn_inst_WB_inst     (clk, ertn_inst_MEM,     ertn_inst_WB,     ppl_men);
    Reg_norst #(1 ) rdcn_inst_WB_inst     (clk, rdcn_inst_MEM,     rdcn_inst_WB,     ppl_men);
    Reg_norst #(1 ) rdcn_op_WB_inst       (clk, rdcn_op_MEM,       rdcn_op_WB,       ppl_men);
    Reg_norst #(1 ) has_int_WB_inst       (clk, has_int_MEM,       has_int_WB,       ppl_men);
    Reg_norst #(1 ) needs_refresh_WB_inst (clk, needs_refresh_MEM, needs_refresh_WB, ppl_men);

    Reg_norst #(1 ) inst_tlbrd_WB_inst   (clk, inst_tlbrd_MEM,   inst_tlbrd_WB,   ppl_men);
    Reg_norst #(1 ) inst_tlbwr_WB_inst   (clk, inst_tlbwr_MEM,   inst_tlbwr_WB,   ppl_men);
    Reg_norst #(1 ) inst_tlbfill_WB_inst (clk, inst_tlbfill_MEM, inst_tlbfill_WB, ppl_men);

    /* --------------------------------------
        WB stage
    -------------------------------------- */

    // register file write enable
    wire [`BYTE - 1 : 0] byte_rd_rs;
    wire [`HALF - 1 : 0] half_rd_rs;

    // data read from MEM section
    wire [`WIDTH - 1 : 0] data_r_dataram;
    wire [`WIDTH - 1 : 0] final_result;

    // the result read from memory (data SRAM)
    assign byte_rd_rs = {`BYTE{mask_dataram_WB[0]}} & ld_res_from_MEM[`BYTE * 0 +: `BYTE]
                      | {`BYTE{mask_dataram_WB[1]}} & ld_res_from_MEM[`BYTE * 1 +: `BYTE]
                      | {`BYTE{mask_dataram_WB[2]}} & ld_res_from_MEM[`BYTE * 2 +: `BYTE]
                      | {`BYTE{mask_dataram_WB[3]}} & ld_res_from_MEM[`BYTE * 3 +: `BYTE]
    ;
    assign half_rd_rs = {`HALF{mask_dataram_WB[0]}} & ld_res_from_MEM[`HALF * 0 +: `HALF]
                      | {`HALF{mask_dataram_WB[2]}} & ld_res_from_MEM[`HALF * 1 +: `HALF]
    ;

    assign data_r_dataram = {`WIDTH{byte_we_WB}} & {{`WORD - `BYTE{byte_rd_rs[`BYTE - 1] & signed_we_WB}}, byte_rd_rs}
                          | {`WIDTH{half_we_WB}} & {{`WORD - `HALF{half_rd_rs[`HALF - 1] & signed_we_WB}}, half_rd_rs}
                          | {`WIDTH{word_we_WB}} &                                                    ld_res_from_MEM
    ;

    assign rdcntid_valid = rdcn_inst_WB & rdcn_op_WB;
    // the final result that will be written to register file
    assign final_result = res_from_mem_WB                                   ? data_r_dataram
                        : ((csr_inst_WB && !rdcn_inst_WB) || rdcntid_valid) ? csr_rvalue
                        : calc_res_WB
    ;

    // register file write control
    assign rf_we    = gpr_we_WB && pipe_valid_WB && (!ecode_WB && !has_int_WB); 
    assign rf_waddr = dest_WB;
    assign rf_wdata = final_result;

    // debug info generate
    // assign debug_wb_pc       = pc_WB;
    assign debug_wb_rf_we    = {4{rf_we}};
    assign debug_wb_rf_wnum  = dest_WB;
    assign debug_wb_rf_wdata = final_result;

    // csr module, reading and writing lines
    assign csr_waddr   = csr_code_WB[8:0];
    assign csr_we      = csr_we_WB && (!ecode_WB && !has_int_WB)  && pipe_valid_WB; // can write, valid, no exception 
    assign csr_wmask   = rj_value_WB | {32{~csr_wmask_en_WB}};
    assign csr_wvalue  = rkd_value_WB;
    assign csr_raddr   = rdcntid_valid ? 9'h40 : csr_code_WB[8:0];

    assign ertn_flush  = ertn_inst_WB && (!ecode_WB && !has_int_WB) && pipe_valid_WB; 
    assign WB_ex       = ((|ecode_WB) || has_int_WB) && pipe_valid_WB; // if any exception exists 
    // assign WB_pc       = pc_WB;
    assign WB_vaddr    = exc_from_IF_WB ? pc_WB : calc_res_WB;
    assign ecode_noint = ecode_WB & {6{!has_int_WB}}; 
    assign WB_esubcode = 8'b0; //temporarily assigned as zero. only ADEM can set it to 8'b1

    // TLBRD
    assign tlb_r_index      = csr_tlbidx_index;
    assign csr_tlbidx_ne_we = pipe_valid_WB && inst_tlbrd_WB && !WB_ex;
    assign csr_tlbidx_ps_we = csr_tlbidx_ne_we;
    assign csr_asid_asid_we = csr_tlbidx_ne_we;
    assign w_tlbidx_ne      = !tlb_r_e;
    assign w_tlbidx_ps      = {6{tlb_r_e}} & tlb_r_ps;
    assign csr_tlbe_we      = csr_tlbidx_ne_we;
    assign w_asid_asid      = {10{tlb_r_e}} & tlb_r_asid;
    assign w_tlbehi_vppn    = {19{tlb_r_e}} & tlb_r_vppn;
    assign w_tlbelo0        = {32{tlb_r_e}} & {4'd0, tlb_r_ppn0, 1'd0, tlb_r_g, tlb_r_mat0, tlb_r_plv0, tlb_r_d0, tlb_r_v0};
    assign w_tlbelo1        = {32{tlb_r_e}} & {4'd0, tlb_r_ppn1, 1'd0, tlb_r_g, tlb_r_mat1, tlb_r_plv1, tlb_r_d1, tlb_r_v1};

    // tlbfill_index_ymr
    /*
     * We use it as the random index for TLBFILL.
     * Every time a TLBFILL instruction is executed,
     * this counter will be increased by 1.
     */
    always @(posedge clk) begin
        if (reset)
            tlbfill_index_ymr <= `LOG2TLBNUM'd0;
        else if (pipe_valid_WB && inst_tlbfill_WB && !WB_ex)
            tlbfill_index_ymr <= tlbfill_index_ymr + `LOG2TLBNUM'd1;
    end

    // TLBWR, TLBFILL
    assign tlb_we      = pipe_valid_WB && (inst_tlbwr_WB || inst_tlbfill_WB) && !WB_ex;
    assign tlb_w_index = inst_tlbwr_WB ? csr_tlbidx_index : tlbfill_index_ymr;
    assign tlb_w_e     = csr_estat_ecode == 6'h3f || ~csr_tlbidx_ne;
    assign tlb_w_vppn  = csr_tlbehi_vppn;
    assign tlb_w_ps    = csr_tlbidx_ps;
    assign tlb_w_asid  = csr_asid_asid; // Should it be `csr_asid_asid`?
    assign tlb_w_g     = csr_tlbelo0[6];
    assign tlb_w_ppn0  = csr_tlbelo0[27:8];
    assign tlb_w_plv0  = csr_tlbelo0[3:2];
    assign tlb_w_mat0  = csr_tlbelo0[5:4];
    assign tlb_w_d0    = csr_tlbelo0[1];
    assign tlb_w_v0    = csr_tlbelo0[0];
    assign tlb_w_ppn1  = csr_tlbelo1[27:8];
    assign tlb_w_plv1  = csr_tlbelo1[3:2];
    assign tlb_w_mat1  = csr_tlbelo1[5:4];
    assign tlb_w_d1    = csr_tlbelo1[1];
    assign tlb_w_v1    = csr_tlbelo1[0];

endmodule