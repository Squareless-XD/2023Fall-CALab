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
`define SMALL_PAGE_SIZE 6'd12

module stage_MEM (
    input  wire                  clk,
    input  wire                  reset, // localized reset signal

    // connect to stage controller
    input  wire                  pipe_tonext_valid_EX, pipe_valid_MEM, pipe_allowin_WB,

    // input from EX
    input  wire [`WIDTH - 1 : 0] pc_EX, alu_div_rdcntv_result, rj_value_EX, rkd_value_EX,
    input  wire [         3 : 0] op_25_22_EX,
    input  wire [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX,
    input  wire                  byte_we_EX, half_we_EX, word_we_EX, signed_we_EX,
    input  wire                  ld_inst_EX, st_inst_EX, csr_inst_EX, ertn_inst_EX, rdcn_inst_EX,
    input  wire                  mul_inst_EX, // mul
    input  wire                  inst_tlbsrch_EX, inst_tlbrd_EX, inst_tlbwr_EX,
    input  wire                  inst_tlbfill_EX, inst_invtlb_EX,
    input  wire [`RADDR - 1 : 0] invtlb_op_EX,
    input  wire [         2 : 0] mul_op_EX, // mul
    input  wire [         2 : 0] rdcn_op_EX, // rdcn
    input  wire                  res_from_mem_EX, gpr_we_EX, mem_we_EX,
    input  wire [         5 : 0] ecode_EX_m,
    input  wire                  csr_we_EX, csr_wmask_en_EX,
    input  wire [        13 : 0] csr_code_EX,
    input  wire                  br_from_WB, // branch valid signal, at WB stage, calculated in cpu_top
    input  wire                  has_int_EX, needs_refresh_EX,
    input  wire                  exc_from_IF_EX,

    // output to CPU-top module
    output wire                  pipe_ready_go_MEM, // calculated in MEM stage

    // output to ID
    output wire                  MEM_ex_ertn, // if any exception exists or is ertn

    // output to WB
    output wire [`WIDTH - 1 : 0] pc_MEM, alu_div_rdcntv_result_MEM, rj_value_MEM, rkd_value_MEM,
    output reg  [`WIDTH - 1 : 0] ld_res_from_MEM,
    output wire [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM,
    output wire [         3 : 0] mask_dataram,
    output wire                  byte_we_MEM, half_we_MEM, word_we_MEM, signed_we_MEM,
    output wire                  ld_inst_MEM, csr_inst_MEM, ertn_inst_MEM, rdcn_inst_MEM,
    output wire                  inst_tlbrd_MEM, inst_tlbwr_MEM, inst_tlbfill_MEM,
    output wire                  mul_inst_MEM, // mul
    output wire [         2 : 0] mul_op_MEM, // mul
    output wire                  rdcn_op_MEM,  // only rdcn_op_EX[2] is useful at WB
    output wire                  res_from_mem_MEM, gpr_we_MEM,
    output wire                  has_int_MEM, needs_refresh_MEM,
    output wire [         5 : 0] ecode_MEM_m,
    output wire                  csr_we_MEM, csr_wmask_en_MEM,
    output wire [        13 : 0] csr_code_MEM,
    output wire                  exc_from_IF_MEM,

    // connect to main memory
    output wire                  data_sram_req, // data SRAM request
    output wire                  data_sram_wr, // data SRAM write
    output wire [         1 : 0] data_sram_size,
    output wire [         3 : 0] data_sram_wstrb, // data SRAM byte-writes
    output wire [`WIDTH - 1 : 0] data_sram_addr, data_sram_wdata,
    input  wire                  data_sram_addr_ok, data_sram_data_ok,
    input  wire [`WIDTH - 1 : 0] data_sram_rdata,

    // connect to CSR and TLB, for TLBSRCH and INVTLB
    input  wire                  csr_crmd_pg,
    input  wire [         1 : 0] csr_crmd_plv,
    input  wire [`WIDTH - 1 : 0] csr_dmw0,
    input  wire [`WIDTH - 1 : 0] csr_dmw1,
    input  wire [         9 : 0] csr_asid_asid,
    input  wire [        18 : 0] csr_tlbehi_vppn,
    output wire                  csr_tlbidx_ne_we, csr_tlbidx_index_we,
    output wire                  w_tlbidx_ne,
    output wire[`LOG2TLBNUM-1:0] w_tlbidx_index,
    output wire [        18 : 0] tlb_s1_vppn,
    output wire                  tlb_s1_va_bit12,
    output wire [         9 : 0] tlb_s1_asid,
    input  wire                  tlb_s1_found,
    input  wire[`LOG2TLBNUM-1:0] tlb_s1_index,
    input  wire [        19 : 0] tlb_s1_ppn,
    input  wire [         5 : 0] tlb_s1_ps,
    input  wire [         1 : 0] tlb_s1_plv, tlb_s1_mat,
    input  wire                  tlb_s1_d, tlb_s1_v,
    output wire                  tlb_invtlb_valid,
    output wire [         4 : 0] tlb_invtlb_op
);

    /* --------------------------------------
        EX -> MEM
    -------------------------------------- */

    // wire [`WIDTH - 1 : 0] pc_MEM;
    // wire [`WIDTH - 1 : 0] alu_result_MEM;
    // wire [`WIDTH - 1 : 0] rkd_value_MEM;

    wire [3:0] op_25_22_MEM;
    // wire [`RADDR - 1 : 0] rf_raddr1_MEM;
    // wire [`RADDR - 1 : 0] rf_raddr2_MEM;
    // wire [`RADDR - 1 : 0] dest_MEM;

    // wire ld_inst_MEM;
    wire st_inst_MEM;

    // wire mul_inst_EX; // mul
    // wire [2:0] mul_op_EX; // mul

    // wire res_from_mem_MEM;
    // wire gpr_we_MEM;
    wire mem_we_MEM;

    wire inst_tlbsrch_MEM, inst_invtlb_MEM;
    wire [`RADDR - 1:0] invtlb_op_MEM;

    // data memory write "width" selection
    // half/byte write enable selection
    wire [3:0] h_we_sel;
    wire [3:0] b_we_sel;

    wire [31:0] data_sram_vaddr;
    wire flag_dmw0_hit, flag_dmw1_hit;
    wire flag_tlb_hit;

    wire [5:0] ecode_MEM;

    wire ppl_men = pipe_tonext_valid_EX;

    Reg_norst #(`WIDTH) pc_MEM_inst                       (clk, pc_EX,                 pc_MEM,                    ppl_men);
    Reg_norst #(`WIDTH) alu_div_rdcntv_result_MEM_inst    (clk, alu_div_rdcntv_result, alu_div_rdcntv_result_MEM, ppl_men);
    Reg_norst #(`WIDTH) rj_value_MEM_inst                 (clk, rj_value_EX,           rj_value_MEM,              ppl_men);
    Reg_norst #(`WIDTH) rkd_value_MEM_inst                (clk, rkd_value_EX,          rkd_value_MEM,             ppl_men);

    Reg_norst #(4)      op_25_22_MEM_inst     (clk, op_25_22_EX,  op_25_22_MEM,  ppl_men);
    Reg_norst #(`RADDR) rf_raddr1_MEM_inst    (clk, rf_raddr1_EX, rf_raddr1_MEM, ppl_men);
    Reg_norst #(`RADDR) rf_raddr2_MEM_inst    (clk, rf_raddr2_EX, rf_raddr2_MEM, ppl_men);
    Reg_norst #(`RADDR) dest_MEM_inst         (clk, dest_EX,      dest_MEM,      ppl_men);
    Reg_norst #(`RADDR) invtlb_op_MEM_inst    (clk, invtlb_op_EX, invtlb_op_MEM, ppl_men);

    Reg_norst #(1) byte_we_MEM_inst   (clk, byte_we_EX,   byte_we_MEM,   ppl_men);
    Reg_norst #(1) half_we_MEM_inst   (clk, half_we_EX,   half_we_MEM,   ppl_men);
    Reg_norst #(1) word_we_MEM_inst   (clk, word_we_EX,   word_we_MEM,   ppl_men);
    Reg_norst #(1) signed_we_MEM_inst (clk, signed_we_EX, signed_we_MEM, ppl_men);

    Reg_norst #(1) ld_inst_MEM_inst   (clk, ld_inst_EX,   ld_inst_MEM,   ppl_men);
    Reg_norst #(1) st_inst_MEM_inst   (clk, st_inst_EX,   st_inst_MEM,   ppl_men);
    Reg_norst #(1) ertn_inst_MEM_inst (clk, ertn_inst_EX, ertn_inst_MEM, ppl_men);
    Reg_norst #(1) rdcn_inst_MEM_inst (clk, rdcn_inst_EX, rdcn_inst_MEM, ppl_men);
    Reg_norst #(1) mul_inst_MEM_inst  (clk, mul_inst_EX,  mul_inst_MEM,  ppl_men);

    Reg_norst #(1) inst_tlbwr_MEM_inst   (clk, inst_tlbwr_EX,   inst_tlbwr_MEM,   ppl_men);
    Reg_norst #(1) inst_tlbfill_MEM_inst (clk, inst_tlbfill_EX, inst_tlbfill_MEM, ppl_men);
    Reg_norst #(1) inst_tlbrd_MEM_inst   (clk, inst_tlbrd_EX,   inst_tlbrd_MEM,   ppl_men);
    Reg_norst #(1) inst_tlbsrch_MEM_inst (clk, inst_tlbsrch_EX, inst_tlbsrch_MEM, ppl_men);
    Reg_norst #(1) inst_invtlb_MEM_inst  (clk, inst_invtlb_EX,  inst_invtlb_MEM,  ppl_men);

    Reg_norst #(3) mul_op_MEM_inst  (clk, mul_op_EX,     mul_op_MEM,  ppl_men);
    Reg_norst #(1) rdcn_op_MEM_inst (clk, rdcn_op_EX[2], rdcn_op_MEM, ppl_men);

    Reg_norst #(1) res_from_mem_MEM_inst (clk, res_from_mem_EX, res_from_mem_MEM, ppl_men);
    Reg_norst #(1) gpr_we_MEM_inst       (clk, gpr_we_EX,       gpr_we_MEM,       ppl_men);
    Reg_norst #(1) mem_we_MEM_inst       (clk, mem_we_EX,       mem_we_MEM,       ppl_men);


    Reg_norst #(6 ) ecode_MEM_inst         (clk, ecode_EX_m,       ecode_MEM,         ppl_men);
    Reg_norst #(14) csr_code_MEM_inst      (clk, csr_code_EX,      csr_code_MEM,      ppl_men);
    Reg_norst #(1 ) csr_we_MEM_inst        (clk, csr_we_EX,        csr_we_MEM,        ppl_men);
    Reg_norst #(1 ) csr_wmask_en_MEM_inst  (clk, csr_wmask_en_EX,  csr_wmask_en_MEM,  ppl_men);
    Reg_norst #(1 ) csr_inst_MEM_inst      (clk, csr_inst_EX,      csr_inst_MEM,      ppl_men);  // forward rdcntid modify
    Reg_norst #(1 ) has_int_MEM_inst       (clk, has_int_EX,       has_int_MEM,       ppl_men);
    Reg_norst #(1 ) needs_refresh_MEM_inst (clk, needs_refresh_EX, needs_refresh_MEM, ppl_men);
    Reg_norst #(1 ) exc_from_IF_MEM_inst   (clk, exc_from_IF_EX,   exc_from_IF_MEM,   ppl_men);

    /* --------------------------------------
        MEM stage
    -------------------------------------- */

    // FSM
    reg  [3:0] current_state;
    reg  [3:0] next_state;
    localparam NMREQ = 4'b0001, // No memory request
               AWAIT = 4'b0010, // Waiting for ...addr_ok
               DWAIT = 4'b0100, // Waiting for ...data_ok
               RDONE = 4'b1000; // Request is done
    
    always @(posedge clk) begin
        if (reset || !pipe_valid_MEM)
            current_state <= NMREQ;
        else
            current_state <= next_state;
    end

    always @(*) begin
        case (current_state)
        NMREQ:
            if (!data_sram_req)
                next_state = NMREQ;
            else if (!data_sram_addr_ok)
                next_state = AWAIT;
            else
                next_state = DWAIT;
        AWAIT:
            if (data_sram_addr_ok)
                next_state = DWAIT;
            else
                next_state = AWAIT;
        DWAIT:
            if (data_sram_data_ok)
                next_state = RDONE;
            else
                next_state = DWAIT;
        default: // RDONE
            if (pipe_allowin_WB)
                next_state = NMREQ;
            else // Waiting for ...allowin_WB
                next_state = RDONE;
        endcase
    end

    decoder_x #(.width(2)) dec_2_4_load_store(
        .in  (alu_div_rdcntv_result_MEM[1:0]),
        .out (b_we_sel)
    );
    assign h_we_sel = {{2{b_we_sel[2]}}, {2{b_we_sel[0]}}};

    // data memory write enable & write data
    wire [`WIDTH - 1 : 0] data_w_dataram;
    assign mask_dataram = {4{word_we_MEM}}
                        | {4{half_we_MEM}} & h_we_sel
                        | {4{byte_we_MEM}} & b_we_sel
    ;
    assign data_w_dataram = {`WIDTH{byte_we_MEM}} & {4{rkd_value_MEM[`BYTE - 1 : 0]}}
                          | {`WIDTH{half_we_MEM}} & {2{rkd_value_MEM[`HALF - 1 : 0]}}
                          | {`WIDTH{word_we_MEM}} &    rkd_value_MEM
    ;

    assign flag_dmw0_hit = csr_crmd_pg && data_sram_vaddr[31:29] == csr_dmw0[31:29]
                        && csr_dmw0[csr_crmd_plv] == 1'b1;
    assign flag_dmw1_hit = csr_crmd_pg && data_sram_vaddr[31:29] == csr_dmw1[31:29]
                        && csr_dmw1[csr_crmd_plv] == 1'b1 && !flag_dmw0_hit;
    assign flag_tlb_hit = !flag_dmw0_hit && !flag_dmw1_hit
                        && tlb_s1_found && csr_crmd_plv <= tlb_s1_plv;
    
    assign data_sram_req   = pipe_valid_MEM && !MEM_ex_ertn && (ld_inst_MEM || st_inst_MEM)
                          && (current_state == NMREQ || current_state == AWAIT) && !br_from_WB;
    assign data_sram_wr    = data_sram_req && st_inst_MEM;
    assign data_sram_size  = {2{ld_inst_MEM              }} & 2'd2 |
                             {2{st_inst_MEM & word_we_MEM}} & 2'd2 |
                             {2{st_inst_MEM & half_we_MEM}} & 2'd1 |
                             {2{st_inst_MEM & byte_we_MEM}} & 2'd0;
    assign data_sram_wstrb = {4{mem_we_MEM}} & mask_dataram;

    // Virtual address
    assign data_sram_vaddr = {alu_div_rdcntv_result_MEM[`WIDTH - 1 :2],
                              (half_we_MEM | byte_we_MEM) & alu_div_rdcntv_result_MEM[1],
                              byte_we_MEM & alu_div_rdcntv_result_MEM[0]};
    assign data_sram_wdata = data_w_dataram;

    // ld_res_from_MEM
    always @(posedge clk) begin
        if (reset)
            ld_res_from_MEM <= 32'd0;
        else if (current_state == DWAIT && data_sram_data_ok)
            ld_res_from_MEM <= data_sram_rdata;
    end

    assign pipe_ready_go_MEM = pipe_valid_MEM &&
        (current_state == NMREQ && !data_sram_req || current_state == RDONE);

    assign ecode_MEM_m = ecode_MEM != 6'd0 ? ecode_MEM :
                        /*
                         * Ignore MMU exception if the instruction doesn't access memory
                         * or direct translation mode is enabled or DMW0/1 is hit.
                         */
                         (!ld_inst_MEM && !st_inst_MEM || !csr_crmd_pg
                         || flag_dmw0_hit || flag_dmw1_hit) ? 6'h00 :
                         !tlb_s1_found                      ? 6'h3F : // TLB refill
                         !tlb_s1_v && ld_inst_MEM           ? 6'h01 : // Load page fault
                         !tlb_s1_v && st_inst_MEM           ? 6'h02 : // Store page fault
                         !flag_tlb_hit                      ? 6'h07 : // Page previlege ..
                         !tlb_s1_d && st_inst_MEM           ? 6'h04 : // Page modified
                         6'h00;

    // Physical address
    assign data_sram_addr = !csr_crmd_pg ? data_sram_vaddr : (
            {32{flag_dmw0_hit}} & {csr_dmw0[27:25], data_sram_vaddr[28:0]} |
            {32{flag_dmw1_hit}} & {csr_dmw1[27:25], data_sram_vaddr[28:0]} |
            {32{flag_tlb_hit }} & {tlb_s1_ppn[19:9],
                tlb_s1_ps == `SMALL_PAGE_SIZE ? tlb_s1_ppn[8:0] : data_sram_vaddr[20:12],
                data_sram_vaddr[11:0]
            }
    );

    assign MEM_ex_ertn = pipe_valid_MEM && 
        (has_int_MEM || ecode_MEM_m != 6'd0 || ertn_inst_MEM);

    // TLBSRCH
    assign csr_tlbidx_ne_we    = pipe_valid_MEM && inst_tlbsrch_MEM && !MEM_ex_ertn && !br_from_WB;
    assign w_tlbidx_ne         = !tlb_s1_found;
    assign csr_tlbidx_index_we = pipe_valid_MEM && inst_tlbsrch_MEM && tlb_s1_found && !MEM_ex_ertn && !br_from_WB;
    assign w_tlbidx_index      = tlb_s1_index;
    assign tlb_s1_vppn         = inst_invtlb_MEM  ? rkd_value_MEM[31:13] :
                                 inst_tlbsrch_MEM ? csr_tlbehi_vppn      :
                                 data_sram_vaddr[31:13];
    assign tlb_s1_va_bit12     = data_sram_vaddr[12];
    assign tlb_s1_asid         = inst_invtlb_MEM ? rj_value_MEM[9:0] : csr_asid_asid;
    
    // INVTLB
    assign tlb_invtlb_valid    = pipe_valid_MEM && inst_invtlb_MEM;
    assign tlb_invtlb_op       = invtlb_op_MEM;
endmodule