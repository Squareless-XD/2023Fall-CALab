`define RADDR  5
`define BYTE   8
`define HALF   16
`define WORD   32
`define WIDTH  32
`define DWIDTH 64
`define R0     5'b00000
`define TRUE   1'b1
`define FALSE  1'b0

`define TLBNUM 16
`define LOG2TLBNUM 4
`define LOG2TLBNUM24 20 // 24-`LOG2TLBNUM

module mycpu(
	input  wire        clk,
	input  wire        resetn,
	// inst sram interface
	output wire        inst_sram_req,
	output wire        inst_sram_wr,    // 0 (Read Only)
	output wire        inst_sram_bypass,
	output wire [ 1:0] inst_sram_size,  // 2 (4 B)
	output wire [ 3:0] inst_sram_wstrb, // 0
	output wire [31:0] inst_sram_addr,
	output wire [31:0] inst_sram_wdata, // 0
	input  wire        inst_sram_addr_ok,
	input  wire        inst_sram_data_ok,
	input  wire [31:0] inst_sram_rdata,
	// data sram interface
	output wire        data_sram_req,
	output wire        data_sram_wr,
	output wire [ 1:0] data_sram_size,  // 0: 1 B, 1: 2B, 2: 4B.
	output wire [ 3:0] data_sram_wstrb,
	output wire [31:0] data_sram_addr,
	output wire [31:0] data_sram_wdata,
	input  wire        data_sram_addr_ok,
	input  wire        data_sram_data_ok,
	input  wire [31:0] data_sram_rdata,
	// trace debug interface
	output wire [31:0] debug_wb_pc,
	output wire [ 3:0] debug_wb_rf_we,
	output wire [ 4:0] debug_wb_rf_wnum,
	output wire [31:0] debug_wb_rf_wdata
);

/* --------------------------------------
	Declarations
  ------------------------------------ */

	parameter Stage_Num = 5; // number of pipeline stages

	reg reset;

	// pipeline controllers 
	wire [Stage_Num - 1 : 0] pipe_ready_go;     // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
	wire                     pipe_ready_go_preIF;
	wire [Stage_Num - 1 : 0] pipe_allowin;      // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
	wire [Stage_Num - 2 : 0] pipe_tonext_valid; // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB. "4" bits
	reg  [Stage_Num - 1 : 0] pipe_valid;        // 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.

	wire                     flag_cancel_IF;

	// register equality detection
	wire rd_EX_r1_eq, rd_MEM_r1_eq, rd_WB_r1_eq; // r1 in ID stage is equal to r1 in EX/MEM/WB stage
	wire rd_EX_r2_eq, rd_MEM_r2_eq, rd_WB_r2_eq; // r2 in ID stage is equal to r2 in EX/MEM/WB stage

	// hazard detection
	wire hzd_alu_EX_r1, hzd_alu_MEM_r1; // ALU type hazard for r1
	wire hzd_alu_EX_r2, hzd_alu_MEM_r2; // ALU type hazard for r2
	wire hzd_ld_mul_csr_EX_r1, hzd_ld_mul_csr_MEM_r1; // load type hazard for r1
	wire hzd_ld_mul_csr_EX_r2, hzd_ld_mul_csr_MEM_r2; // load type hazard for r2
	wire hzd_WB_r1, hzd_WB_r2;
	wire hzd_csrw_EX, hzd_csrw_MEM, hzd_csrw_WB;

	wire calc_done; // calculation ready, for mul/div instructions
	wire ID_hazard; // ID stage installed/bubbled for hazard

	/*--- IF declaration ---*/
	wire [`WIDTH - 1 : 0] inst_final;
	wire [`WIDTH - 1 : 0] pc; // sequential PC value, PC+4
	wire [         5 : 0] ecode;
	/*--- IF declaration ---*/

	/*--- ID declaration ---*/
	wire [        12 : 0] alu_op; // ALU operation code
	wire [`WIDTH - 1 : 0] rj_value, rkd_value, imm; // rj, rkd, immediate
	wire [`WIDTH - 1 : 0] pc_ID; // PC

	wire [         3 : 0] op_25_22; // opcode[25:22]
	wire [         6 : 0] op_21_15; // opcode[21:15]
	wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest; // register file read address
	wire                  byte_we, half_we, word_we, signed_we; // byte, half, word, signed write enable
	wire                  ld_inst, st_inst, div_inst, mul_inst, ertn_inst, csr_inst, rdcn_inst; // load, store, div, mul, ertn, csr, syscall
	wire                  br_taken_sure, br_taken_yes, br_taken_no; // branch taken or not
	wire                  inst_tlbsrch, inst_tlbwr, inst_tlbfill, inst_tlbrd, inst_invtlb;
	wire                  src1_is_pc, src2_is_imm, src2_is_4, br_src_sel; // source operand control signal
	wire                  res_from_mem, gpr_we, mem_we; // result from memory, general register write enable, memory write enable
	wire                  rf_ren1, rf_ren2; // register file read enable
	wire [         5 : 0] ecode_ID_m; // exception code
	wire [         2 : 0] rdcn_op;
	wire [`RADDR - 1 : 0] invtlb_op; // invtlb opcode
	wire                  csr_we, csr_wmask_en; // CSR write enable, CSR write mask enable
	wire                  has_int_ID, needs_refresh_ID;
	wire                  exc_from_IF_ID;
	wire [`WIDTH - 1 : 0] br_target; // branch target, PC+4 or branch target
	/*--- ID declaration ---*/

	/*--- EX declaration ---*/
	wire [`WIDTH - 1 : 0] pc_EX, rj_value_EX, rkd_value_EX; // PC, ALU result, rkd
	wire [`WIDTH - 1 : 0] alu_div_rdcntv_result; // PC, rkd
	wire [`DWIDTH - 1: 0] mul_result; // to WB - skip MEM
	wire                  inst_tlbsrch_EX, inst_tlbwr_EX, inst_tlbfill_EX;
	wire                  inst_tlbrd_EX, inst_invtlb_EX;
	wire [         3 : 0] op_25_22_EX; // opcode[25:22]
	wire [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX; // register file read address
	wire                  ld_inst_EX, st_inst_EX, csr_inst_EX, ertn_inst_EX, rdcn_inst_EX; // load, store, ertn, csr, syscall
	wire                  mul_inst_EX;
	wire [         2 : 0] mul_op_EX;
	wire                  res_from_mem_EX, gpr_we_EX, mem_we_EX; // result from memory, general register write enable, memory write enable
	wire [`RADDR - 1 : 0] invtlb_op_EX; // invtlb opcode
	wire                  has_int_EX, needs_refresh_EX;
	wire                  exc_from_IF_EX;
	wire                  EX_ex_ertn;

	// wire                  br_taken, calc_done; // calculation ready, for mul/div instructions
	wire [         5 : 0] ecode_EX_m;
	wire [         2 : 0] rdcn_op_EX;

	wire                  csr_we_EX, csr_wmask_en_EX;
	wire [        13 : 0] csr_code_EX;
	/*--- EX declaration ---*/

	/*--- MEM declaration ---*/
	wire [`WIDTH - 1 : 0] pc_MEM, alu_div_rdcntv_result_MEM, rj_value_MEM, rkd_value_MEM; // PC, ALU result, rkd
	wire [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM; // register file read address
	wire [         3 : 0] mask_dataram; // data memory byte-write mask
	wire                  byte_we_MEM, half_we_MEM, word_we_MEM, signed_we_MEM; // byte, half, word, signed write enable
	wire                  ld_inst_MEM, csr_inst_MEM, ertn_inst_MEM, rdcn_inst_MEM; // load, csr, ertn, syscall
	wire                  inst_tlbwr_MEM, inst_tlbfill_MEM, inst_tlbrd_MEM;
	wire                  mul_inst_MEM;
	wire [         2 : 0] mul_op_MEM;
	wire                  rdcn_op_MEM;
	wire                  res_from_mem_MEM, gpr_we_MEM; // result from memory, general register write enable
	wire                  has_int_MEM, needs_refresh_MEM;
	wire                  exc_from_IF_MEM;
	wire                  MEM_ex_ertn;

	wire [         5 : 0] ecode_MEM_m;
	wire                  csr_we_MEM, csr_wmask_en_MEM;
	wire [        13 : 0] csr_code_MEM;
	wire [`WIDTH - 1 : 0] ld_res_from_MEM;
	/*--- MEM declaration ---*/

	/*--- WB declaration ---*/
	wire                  gpr_we_WB; // general register write enable

	wire                  rf_we;    // register file write enable
	wire [`RADDR - 1 : 0] rf_waddr; // register file write address
	wire [`RADDR - 1 : 0] dest_WB;  // destination register number
	wire [`WIDTH - 1 : 0] rf_wdata; // register file write data
	wire [`WIDTH - 1 : 0] pc_WB;

	// wire [         8 : 0] csr_waddr;
	// wire                  csr_we_WB;
	// wire [`WIDTH - 1 : 0] csr_wmask;
	// wire [`WIDTH - 1 : 0] csr_wvalue;
	// wire [         8 : 0] csr_raddr;
	// wire                  ertn_flush;
	// wire                  WB_ex;
	// wire [        31 : 0] WB_vaddr;
	// wire [         5 : 0] ecode_noint;
	// wire [         8 : 0] WB_esubcode;

	wire                  ertn_inst_WB;
	wire                  has_int_WB;
	wire                  rdcn_inst_WB;
/*--- WB declaration ---*/

	assign debug_wb_pc = pc_WB;
	// rdcn_inst
	reg  [`DWIDTH - 1 : 0] tick;


/* --------------------------------------------------------------------
	reset signal generation
  ------------------------------------------------------------------ */

	always @(posedge clk) begin
		reset <= ~resetn;
	end

	// rdcn tick
	always @ (posedge clk) begin
		if(reset) 
			tick <= 64'b0;
		else
			tick <= tick + 64'b1;
	end


/* --------------------------------------------------------------------
	Wires for Control and Status Registers Module
  ------------------------------------------------------------------ */

	wire [ 8:0] csr_waddr; // WB
	wire [ 8:0] csr_raddr; // WB
	wire        csr_we_WB; // WB
	wire [31:0] csr_wmask; // WB
	wire [31:0] csr_wvalue; // WB
	wire [31:0] csr_rvalue; // output WB
	wire [31:0] cpuid = 32'b0; // not certain now
	wire [ 7:0] hw_int_in = 8'b0; // not certain now
	wire        ipi_int_in = 1'b0; // not certain now
	wire [31:0] ex_entry; // output EX
	wire        has_int; // output, not certain now
	wire [31:0] era_value; // output EX
	wire        ertn_flush; // WB
	wire        WB_ex; // WB
	// wire [31:0] WB_pc; // WB
	wire [31:0] WB_vaddr; // WB
	wire [ 5:0] ecode_noint; // WB
	wire [ 5:0] WB_ecode;
	wire [ 8:0] WB_esubcode; // WB
	wire [ 5:0] csr_estat_ecode;
	wire [ 9:0] csr_asid_asid;
	wire [18:0] csr_tlbehi_vppn;
	wire [31:0] csr_tlbidx, csr_tlbelo0, csr_tlbelo1;
	wire [31:0] csr_dmw0, csr_dmw1;
	wire        csr_crmd_da, csr_crmd_pg;
	wire [ 1:0] csr_crmd_plv;
	wire [ 1:0] csr_crmd_datf, csr_crmd_datm;
	wire        csr_tlbe_we, csr_asid_asid_we;
	wire [18:0] w_tlbehi_vppn;
	wire        csr_tlbidx_ne_we;
	/*
	 * Both TLBSRCH and TLBRD will write TLBIDX::NE,
	 * so two groups of writting signals of TLBIDX::NE are needed.
	 */
	wire        csr_tlbidx_ne_we_MEM, csr_tlbidx_ne_we_WB;
	wire        csr_tlbidx_ps_we;
	wire        csr_tlbidx_index_we;
	wire        w_tlbidx_ne_MEM, w_tlbidx_ne_WB;
	wire [`LOG2TLBNUM - 1:0] w_tlbidx_index;
	wire [ 5:0] w_tlbidx_ps;
	wire [31:0] w_tlbidx, w_tlbelo0, w_tlbelo1;
	wire [ 9:0] w_asid_asid;

/* --------------------------------------------------------------------
	Wires for Translation Lookaside Buffer Module
  ------------------------------------------------------------------ */

	wire [18:0] tlb_s0_vppn;
	wire        tlb_s0_va_bit12;
	wire [ 9:0] tlb_s0_asid;
	wire        tlb_s0_found;
	wire [`LOG2TLBNUM - 1:0] tlb_s0_index;
	wire [19:0] tlb_s0_ppn;
	wire [ 5:0] tlb_s0_ps;
	wire [ 1:0] tlb_s0_plv, tlb_s0_mat;
	wire        tlb_s0_d, tlb_s0_v;

	wire [18:0] tlb_s1_vppn;
	wire        tlb_s1_va_bit12;
	wire [ 9:0] tlb_s1_asid;
	wire        tlb_s1_found;
	wire [`LOG2TLBNUM - 1:0] tlb_s1_index;
	wire [19:0] tlb_s1_ppn;
	wire [ 5:0] tlb_s1_ps;
	wire [ 1:0] tlb_s1_plv, tlb_s1_mat;
	wire        tlb_s1_d, tlb_s1_v;

	wire        tlb_invtlb_valid;
	wire [ 4:0] tlb_invtlb_op;

	wire        tlb_we;
	wire [`LOG2TLBNUM - 1:0] tlb_w_index;
	wire        tlb_w_e;
	wire [18:0] tlb_w_vppn;
	wire [ 5:0] tlb_w_ps;
	wire [ 9:0] tlb_w_asid;
	wire        tlb_w_g;
	wire [19:0] tlb_w_ppn0;
	wire [ 1:0] tlb_w_plv0, tlb_w_mat0;
	wire        tlb_w_d0, tlb_w_v0;
	wire [19:0] tlb_w_ppn1;
	wire [ 1:0] tlb_w_plv1, tlb_w_mat1;
	wire        tlb_w_d1, tlb_w_v1;

	wire [`LOG2TLBNUM - 1:0] tlb_r_index;
	wire [18:0] tlb_r_vppn;
	wire [ 5:0] tlb_r_ps;
	wire [ 9:0] tlb_r_asid;
	wire        tlb_r_g;
	wire [19:0] tlb_r_ppn0;
	wire [ 1:0] tlb_r_plv0, tlb_r_mat0;
	wire        tlb_r_d0, tlb_r_v0;
	wire [19:0] tlb_r_ppn1;
	wire [ 1:0] tlb_r_plv1, tlb_r_mat1;
	wire        tlb_r_d1, tlb_r_v1;

	assign WB_ecode = ecode_noint;
	csr u_csr (
		.clk (clk),
		.rst (reset),

		// Ports for inst access
		.csr_waddr  (csr_waddr),  // write address
		.csr_raddr  (csr_raddr),  // read  address
		.csr_we     (csr_we_WB),  // write enable
		.csr_wmask  (csr_wmask),  // write mask
		.csr_wvalue (csr_wvalue), // value to be written
		.csr_rvalue (csr_rvalue), // value read

		// Ports for interacting with CPU hardware
		.cpuid        (cpuid),                      // CPU ID, just 0 is OK
		.hw_int_in    (hw_int_in),                  // hardware interrupt
		.ipi_int_in   (ipi_int_in),                 // IPI interrupt (0)
		.ex_entry     (ex_entry),                   // exception entry
		.has_int      (has_int),                    // has interrupt
		.era_value    (era_value),                  // exception return address
		.ertn_flush   (ertn_flush),                 // ERTN inst (ertn)
		.WB_ex        (WB_ex && !needs_refresh_WB), // exc from WB
		.WB_pc        (pc_WB),
		.WB_vaddr     (WB_vaddr),                   // bad vaddr
		.WB_ecode     (WB_ecode),                   // exception code
		.WB_esubcode  (WB_esubcode),
		.estate_ecode (csr_estat_ecode),

		// MMU CSR read ports
		.asid_asid   (csr_asid_asid),
		.tlbehi_vppn (csr_tlbehi_vppn),
		.tlbidx      (csr_tlbidx),
		.tlbelo0     (csr_tlbelo0),
		.tlbelo1     (csr_tlbelo1),
		.dmw0        (csr_dmw0),
		.dmw1        (csr_dmw1),
		.crmd_da     (csr_crmd_da),
		.crmd_pg     (csr_crmd_pg),
		.crmd_plv    (csr_crmd_plv),
		.crmd_datf   (csr_crmd_datf),
		.crmd_datm   (csr_crmd_datm),

		// MMU CSR write ports
		.tlbe_we         (csr_tlbe_we),
		.w_tlbehi_vppn   (w_tlbehi_vppn),
		.tlbidx_ne_we    (csr_tlbidx_ne_we),
		.tlbidx_ps_we    (csr_tlbidx_ps_we),
		.tlbidx_index_we (csr_tlbidx_index_we),
		.w_tlbidx        (w_tlbidx),
		.w_tlbelo0       (w_tlbelo0),
		.w_tlbelo1       (w_tlbelo1),
		.asid_asid_we    (csr_asid_asid_we),
		.w_asid_asid     (w_asid_asid)
	);

	assign csr_tlbidx_ne_we = csr_tlbidx_ne_we_MEM || csr_tlbidx_ne_we_WB;
	/*
	 * NOTE: `csr_tlbidx_ne_we_MEM` and `csr_tlbidx_ne_we_WB` can
	 * NOT be 1 at the same time.
	 * `w_tlbidx` is unnecessary to be strict like this,
	 * because `csr_tlbidx_*_we` will control the parts to be written.
	 */
	assign w_tlbidx = {
	/* 31:   NE */  csr_tlbidx_ne_we_MEM ? w_tlbidx_ne_MEM :
	                csr_tlbidx_ne_we_WB ? w_tlbidx_ne_WB : csr_tlbidx[31],
	/* 30:    0 */  1'd0,
	/* 29~24:PS */  csr_tlbidx_ps_we ? w_tlbidx_ps : csr_tlbidx[29:24],
	/* 23:04: 0 */  `LOG2TLBNUM24'd0,
	/* 03~00:Index */csr_tlbidx_index_we ? w_tlbidx_index : csr_tlbidx[`LOG2TLBNUM - 1:0]
	};

	tlb u_tlb (
		.clk (clk),
		// search port 0
		.s0_vppn     (tlb_s0_vppn),
		.s0_va_bit12 (tlb_s0_va_bit12),
		.s0_asid     (tlb_s0_asid),
		.s0_found    (tlb_s0_found),
		.s0_index    (tlb_s0_index),
		.s0_ppn      (tlb_s0_ppn),
		.s0_ps       (tlb_s0_ps),
		.s0_plv      (tlb_s0_plv),
		.s0_mat      (tlb_s0_mat),
		.s0_d        (tlb_s0_d),
		.s0_v        (tlb_s0_v),
		// search port 1
		.s1_vppn     (tlb_s1_vppn),
		.s1_va_bit12 (tlb_s1_va_bit12),
		.s1_asid     (tlb_s1_asid),
		.s1_found    (tlb_s1_found),
		.s1_index    (tlb_s1_index),
		.s1_ppn      (tlb_s1_ppn),
		.s1_ps       (tlb_s1_ps),
		.s1_plv      (tlb_s1_plv),
		.s1_mat      (tlb_s1_mat),
		.s1_d        (tlb_s1_d),
		.s1_v        (tlb_s1_v),
		// inv
		.invtlb_valid (tlb_invtlb_valid),
		.invtlb_op    (tlb_invtlb_op),
		// write port
		.we      (tlb_we),
		.w_index (tlb_w_index),
		.w_e     (tlb_w_e),
		.w_vppn  (tlb_w_vppn),
		.w_ps    (tlb_w_ps),
		.w_asid  (tlb_w_asid),
		.w_g     (tlb_w_g),
		.w_ppn0  (tlb_w_ppn0),
		.w_plv0  (tlb_w_plv0),
		.w_mat0  (tlb_w_mat0),
		.w_d0    (tlb_w_d0),
		.w_v0    (tlb_w_v0),
		.w_ppn1  (tlb_w_ppn1),
		.w_plv1  (tlb_w_plv1),
		.w_mat1  (tlb_w_mat1),
		.w_d1    (tlb_w_d1),
		.w_v1    (tlb_w_v1),
		// read port
		.r_index (tlb_r_index),
		.r_e     (tlb_r_e),
		.r_vppn  (tlb_r_vppn),
		.r_ps    (tlb_r_ps),
		.r_asid  (tlb_r_asid),
		.r_g     (tlb_r_g),
		.r_ppn0  (tlb_r_ppn0),
		.r_plv0  (tlb_r_plv0),
		.r_mat0  (tlb_r_mat0),
		.r_d0    (tlb_r_d0),
		.r_v0    (tlb_r_v0),
		.r_ppn1  (tlb_r_ppn1),
		.r_plv1  (tlb_r_plv1),
		.r_mat1  (tlb_r_mat1),
		.r_d1    (tlb_r_d1),
		.r_v1    (tlb_r_v1)
	);


/* --------------------------------------------------------------------
	Pipeline control
  ------------------------------------------------------------------ */

	/* pre-if stage end */
	assign pipe_ready_go[1] = pipe_valid[1] && !ID_hazard;
	assign pipe_ready_go[2] = pipe_valid[2] && calc_done;
	assign pipe_ready_go[4] = pipe_valid[4];
	/*
	 * pipe_ready_go[0] is connected to stage IF, and
	 * pipe_ready_go[3] is connected to stage MEM.
	 */

	// judge whether the stage allows new instruction to enter
	assign pipe_allowin[Stage_Num - 2 : 0] =
		~pipe_valid[Stage_Num - 2 : 0] | pipe_ready_go[Stage_Num - 2 : 0] & pipe_allowin[Stage_Num - 1 : 1];
	assign pipe_allowin[Stage_Num - 1] =
		~pipe_valid[Stage_Num - 1] | pipe_ready_go[Stage_Num - 1];

	// judge whether the stage is ready to go
	assign pipe_tonext_valid[Stage_Num - 2 : 0] =
		pipe_allowin[Stage_Num - 1 : 1] & pipe_ready_go[Stage_Num - 2 : 0];

	// branch valid from WB stage
	wire br_from_WB = WB_ex || (has_int_WB || ertn_inst_WB) && pipe_valid[4];

	// valid signal control in pipeline
	always @(posedge clk) begin
		if (reset) begin
				pipe_valid <= 5'b00000;
		end
		else begin
			// IF stage
			if ((br_taken || br_from_WB) && pipe_ready_go[0] && (!pipe_ready_go_preIF ||
					!inst_sram_req) // Exception exists in preIF
				|| flag_cancel_IF && inst_sram_data_ok && !(pipe_ready_go_preIF &&
					pipe_allowin[0])
				)
				/*
				 * Before exp19: When these 3 signals are all 1:
				 * inst_sram_data_ok being 1 means that the wrong inst comes,
				 * but pipe_ready_go_preIF being 1 means that the right inst also comes,
				 * so we should not clear pipe_valid_IF directly!
				 * However, since exp19, both pipe_ready_go_preIF and pipe_allowin[#IF] being 1
				 * are needed.
				 * NOTE: (pipe_ready_go_preIF && inst_sram_req) and
				 * (pipe_ready_go_preIF && pipe_allowin[#IF]) are not equivalent! The situation
				 * that makes them different (the former is 0 while the latter is 1) should
				 * be considered carefully, which has caused a bug!
				 */
				pipe_valid[0] <= 0;
			else if (pipe_allowin[0]) begin
				pipe_valid[0] <= pipe_ready_go_preIF;
			end

			// ID stage
			if (br_taken || br_from_WB) begin
				pipe_valid[1] <= 1'b0;
			end
			else  begin
				if (pipe_allowin[1]) begin
					pipe_valid[1] <= pipe_ready_go[0];
				end
			end

			// EX stage
			if (br_from_WB) begin
				pipe_valid[2] <= 1'b0;
			end
			else if (br_taken) begin
				if (pipe_tonext_valid[2]) begin
					pipe_valid[2] <= 1'b0;
				end
			end
			else  begin
				if (pipe_allowin[2]) begin
					pipe_valid[2] <= pipe_ready_go[1];
				end
			end

			// MEM stage
			if (br_from_WB) begin
				pipe_valid[3] <= 1'b0;
			end
			else if (pipe_allowin[3]) begin
				pipe_valid[3] <= pipe_ready_go[2];
			end

			// WB stage
			if (br_from_WB) begin
				pipe_valid[4] <= 1'b0;
			end
			else if (pipe_allowin[4]) begin
				pipe_valid[4] <= pipe_ready_go[3];
			end
		end
	end

	// NOTE: pre-IF is moved into stage_IF.v

/* --------------------------------------------------------------------
	SRAM Interface to upper level module
  ------------------------------------------------------------------ */
	assign inst_sram_wr = 1'b0; // 0 (Read Only)
	assign inst_sram_size = 2'd2; // 2 (4 B)
	assign inst_sram_wstrb = 4'b0; // 0
	assign inst_sram_wdata = 32'b0; // 0


	/*
	 * NOTE:
	 * here we can modify bypass type
	 * for in Fall 2023 term the func/ is wrong: I-$ always disabled
	 * we cannot test bypass except set bypass as false here
	 */
	assign inst_sram_bypass =
		( csr_crmd_da & ~csr_crmd_pg) && (csr_crmd_datf == 2'b00)
	 || (~csr_crmd_da &  csr_crmd_pg) && (
			(csr_dmw0[31:29] == inst_sram_addr[31:29]) ? (csr_dmw0[5:4] == 2'b00)
		  : (csr_dmw1[31:29] == inst_sram_addr[31:29]) ? (csr_dmw1[5:4] == 2'b00)
		  :                                              (tlb_s0_mat    == 2'b00)
		)
	;
	// assign inst_sram_bypass = `FALSE;


	/*
	 * Direct Address Translation Mode: DA=1, PG=0
	 * Mapping Address Translation Mode: DA=0, PG=1
	 *   Direct Mapping Address Translation Mode: in DMW0/1 window
	 *   Page Table Mapping Address Translation Mode: others
	 * DMW0~DMW1:
	 *   31:29 - 28 - 27:25 - 24:6 - 5:4 - 3    - 2:1 - 0
	 *   VSEG  - 0  - PSEG  - 0    - MAT - PLV3 - 0   - PLV0
	 * Memory Access Type / Data Address Type Fetch
	 *   0: Strong Sequence, uncacheable.  1: consistent, cacheable.  2/3: reserved
	 */


/* --------------------------------------------------------------------
	Pipeline Hazard Control
  ------------------------------------------------------------------ */

	// register equality detection
	assign rd_EX_r1_eq  = (rf_raddr1 != `R0) && (rf_raddr1 == dest_EX)  && pipe_valid[2];
	assign rd_EX_r2_eq  = (rf_raddr2 != `R0) && (rf_raddr2 == dest_EX)  && pipe_valid[2];
	assign rd_MEM_r1_eq = (rf_raddr1 != `R0) && (rf_raddr1 == dest_MEM) && pipe_valid[3];
	assign rd_MEM_r2_eq = (rf_raddr2 != `R0) && (rf_raddr2 == dest_MEM) && pipe_valid[3];
	assign rd_WB_r1_eq  = (rf_raddr1 != `R0) && (rf_raddr1 == dest_WB)  && pipe_valid[4];
	assign rd_WB_r2_eq  = (rf_raddr2 != `R0) && (rf_raddr2 == dest_WB)  && pipe_valid[4];

	// hazard detection
	assign hzd_alu_EX_r1  = rd_EX_r1_eq  && gpr_we_EX  && !(ld_inst_EX || mul_inst_EX || csr_inst_EX);
	assign hzd_alu_EX_r2  = rd_EX_r2_eq  && gpr_we_EX  && !(ld_inst_EX || mul_inst_EX || csr_inst_EX);
	assign hzd_alu_MEM_r1 = rd_MEM_r1_eq && gpr_we_MEM && !(ld_inst_MEM || mul_inst_MEM || csr_inst_MEM);
	assign hzd_alu_MEM_r2 = rd_MEM_r2_eq && gpr_we_MEM && !(ld_inst_MEM || mul_inst_MEM || csr_inst_MEM);
	assign hzd_ld_mul_csr_EX_r1  = rd_EX_r1_eq  && (ld_inst_EX || mul_inst_EX || csr_inst_EX); // mul
	assign hzd_ld_mul_csr_EX_r2  = rd_EX_r2_eq  && (ld_inst_EX || mul_inst_EX || csr_inst_EX); // mul
	assign hzd_ld_mul_csr_MEM_r1 = rd_MEM_r1_eq && (ld_inst_MEM || mul_inst_MEM || csr_inst_MEM); // mul
	assign hzd_ld_mul_csr_MEM_r2 = rd_MEM_r2_eq && (ld_inst_MEM || mul_inst_MEM || csr_inst_MEM); // mul
	assign hzd_WB_r1 = rd_WB_r1_eq && gpr_we_WB, hzd_WB_r2 = rd_WB_r2_eq && gpr_we_WB;
	assign hzd_csrw_EX = pipe_valid[2] && csr_we_EX &&
			(csr_code_EX[8:0] == 9'h00 || csr_code_EX[8:0] == 9'h04 || // CRMD, ECFG
			csr_code_EX[8:0] == 9'h41 || csr_code_EX[8:0] == 9'h44 || // TCFG, TICLR
			csr_code_EX[8:0] == 9'h11), // TLBEHI
		hzd_csrw_MEM = pipe_valid[3] && csr_we_MEM &&
			(csr_code_MEM[8:0] == 9'h00 || csr_code_MEM[8:0] == 9'h04 || // CRMD, ECFG
			csr_code_MEM[8:0] == 9'h41 || csr_code_MEM[8:0] == 9'h44 || // TCFG, TICLR
			csr_code_MEM[8:0] == 9'h11), // TLBEHI
		hzd_csrw_WB = pipe_valid[4] && csr_we_WB &&
			(csr_waddr[8:0] == 9'h00 || csr_waddr[8:0] == 9'h04 || // CRMD, ECFG
			csr_waddr[8:0] == 9'h41 || csr_waddr[8:0] == 9'h44 || // TCFG, TICLR
			csr_waddr[8:0] == 9'h11); // TLBEHI
	

	// hazard that stalls ID stage
	assign ID_hazard = hzd_ld_mul_csr_EX_r1  && rf_ren1
					|| hzd_ld_mul_csr_EX_r2  && rf_ren2
					|| hzd_ld_mul_csr_MEM_r1 && rf_ren1
					|| hzd_ld_mul_csr_MEM_r2 && rf_ren2
					/*|| hzd_rdcn_WB_r1 && rf_ren1
					|| hzd_rdcn_WB_r2 && rf_ren2*/
					|| hzd_csrw_EX || hzd_csrw_MEM || hzd_csrw_WB
	;



/* --------------------------------------------------------------------
	Pipeline Stages Instantiation
  ------------------------------------------------------------------ */

/*
module stage_IF (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// connect to stage controller
	input  wire                  pipe_allowin_IF, // allow IF stage to accept new instruction
	input  wire                  pipe_tonext_valid_IF, // IF stage is ready to go
	input  wire                  pipe_valid_IF, // IF stage is valid
	output wire 				 pipe_ready_go_preIF,
	output wire					 pipe_ready_go_IF,
	output reg					 flag_cancel_IF,

	// input about exception and ertn
	input  wire					 WB_ex, ertn_flush, needs_refresh_WB,
	input  wire	[`WIDTH - 1 : 0] ex_entry, era_value, pc_WB,
	
	// connect to I-SRAM
	output wire					 inst_sram_req,
	output wire [`WIDTH - 1 : 0] inst_sram_addr,
	input  wire					 inst_sram_addr_ok,
	input  wire					 inst_sram_data_ok,
	input  wire [`WIDTH - 1 : 0] inst_sram_rdata,	// instruction memory read data
	
	// input from EX stage
	input  wire                  br_taken, // branch taken or not
	input  wire [`WIDTH - 1 : 0] br_target,
	
	// output to ID (not only)
	output wire [`WIDTH - 1 : 0] inst_final, // final instruction
	output reg  [`WIDTH - 1 : 0] pc, // PC
	output wire [         5 : 0] ecode,

	// connect to CSR and TLB
	input  wire					 csr_crmd_pg,
	input  wire [		  1 : 0] csr_crmd_plv,
	input  wire [`WIDTH - 1 : 0] csr_dmw0,
	input  wire [`WIDTH - 1 : 0] csr_dmw1,
	output wire [        18 : 0] tlb_s0_vppn,
	output wire					 tlb_s0_va_bit12,
	//output wire [         9 : 0] tlb_s0_asid,
	input  wire					 tlb_s0_found,
	input  wire[`LOG2TLBNUM-1:0] tlb_s0_index,
	input  wire [        19 : 0] tlb_s0_ppn,
	input  wire [         5 : 0] tlb_s0_ps,
	input  wire [         1 : 0] tlb_s0_plv, tlb_s0_mat,
	input  wire					 tlb_s0_d, tlb_s0_v
);
endmodule
*/

	stage_IF stage_IF_inst (
		.clk					(clk),
		.reset					(reset),

		.pipe_allowin_IF		(pipe_allowin[0]),
		.pipe_tonext_valid_IF	(pipe_tonext_valid[0]),
		.pipe_valid_IF			(pipe_valid[0]),
		.pipe_ready_go_preIF	(pipe_ready_go_preIF),
		.pipe_ready_go_IF		(pipe_ready_go[0]),
		.flag_cancel_IF			(flag_cancel_IF),

		.WB_ex					(WB_ex),
		.ertn_flush				(ertn_flush),
		.needs_refresh_WB		(needs_refresh_WB),
		.ex_entry				(ex_entry),
		.era_value				(era_value),
		.pc_WB					(pc_WB),

		.inst_sram_req			(inst_sram_req),
		.inst_sram_addr			(inst_sram_addr),
		.inst_sram_addr_ok		(inst_sram_addr_ok),
		.inst_sram_data_ok		(inst_sram_data_ok),
		.inst_sram_rdata		(inst_sram_rdata),

		.br_taken				(br_taken),
		.br_target				(br_target),

		.inst_final				(inst_final),
		.pc						(pc),
		.ecode					(ecode),

		.csr_crmd_pg			(csr_crmd_pg),
		.csr_crmd_plv			(csr_crmd_plv),
		.csr_dmw0				(csr_dmw0),
		.csr_dmw1				(csr_dmw1),
		.tlb_s0_vppn			(tlb_s0_vppn),
		.tlb_s0_va_bit12		(tlb_s0_va_bit12),
		//.tlb_s0_asid			(tlb_s0_asid),
		.tlb_s0_found			(tlb_s0_found),
		.tlb_s0_index			(tlb_s0_index),
		.tlb_s0_ppn				(tlb_s0_ppn),
		.tlb_s0_ps				(tlb_s0_ps),
		.tlb_s0_plv				(tlb_s0_plv),
		.tlb_s0_mat				(tlb_s0_mat),
		.tlb_s0_d				(tlb_s0_d),
		.tlb_s0_v				(tlb_s0_v)
	);
	assign tlb_s0_asid = csr_asid_asid;

/*
module stage_ID (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_IF, pipe_tonext_valid_ID, pipe_valid_ID,
	input  wire                  hzd_alu_EX_r1, hzd_alu_MEM_r1, hzd_WB_r1,
	input  wire                  hzd_alu_EX_r2, hzd_alu_MEM_r2, hzd_WB_r2,
	input  wire [`WIDTH - 1 : 0] alu_div_rdcntv_result, alu_div_rdcntv_result_MEM, rf_wdata_WB,
	input  wire                  has_int, EX_MEM_WB_ex_ertn, br_from_WB,

	// input from IF stage
	input  wire [`WIDTH - 1 : 0] inst_final, pc,
	input  wire [         5 : 0] ecode,

	// input from WB stage
	input  wire                  rf_we,		// register file write enable
	input  wire [`RADDR - 1 : 0] rf_waddr,	// register file write address
	input  wire [`WIDTH - 1 : 0] rf_wdata,	// register file write data

	// output to EX
	output wire [        12 : 0] alu_op,
	output wire [`WIDTH - 1 : 0] rj_value, rkd_value, imm, pc_ID,
	output wire [         3 : 0] op_25_22,
	output wire [         6 : 0] op_21_15,
	output wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest,
	output wire 				 byte_we, half_we, word_we, signed_we,
	output wire                  ld_inst, st_inst, div_inst, mul_inst,
	output wire                  ertn_inst, csr_inst, rdcn_inst,
	output wire                  br_taken_sure, br_taken_yes, br_taken_no,
	output wire					 inst_tlbsrch, inst_tlbrd, inst_tlbwr, inst_tlbfill, inst_invtlb,
	output wire [`RADDR - 1 : 0] invtlb_op,
	output wire                  src1_is_pc, src2_is_imm, src2_is_4, br_src_sel,
	output wire                  res_from_mem, gpr_we, mem_we,
	output wire                  rf_ren1, rf_ren2,
	output wire [         5 : 0] ecode_ID_m,
	output wire [         2 : 0] rdcn_op,
	output wire 				 csr_we, csr_wmask_en, // csr write enable, csr write mask enable (inst_csrxchg)
	output wire                  has_int_ID, needs_refresh_ID,
	// exception happens at IF, such as TLB refill and page previlege fault
	output wire					 exc_from_IF_ID
);
endmodule
*/

	stage_ID stage_ID_inst (
		.clk						(clk),
		.reset						(reset),

		.pipe_tonext_valid_IF		(pipe_tonext_valid[0]),
		.pipe_tonext_valid_ID		(pipe_tonext_valid[1]),
		.pipe_valid_ID				(pipe_valid[1]),
		.hzd_alu_EX_r1				(hzd_alu_EX_r1),
		.hzd_alu_MEM_r1				(hzd_alu_MEM_r1),
		.hzd_WB_r1					(hzd_WB_r1),
		.hzd_alu_EX_r2				(hzd_alu_EX_r2),
		.hzd_alu_MEM_r2				(hzd_alu_MEM_r2),
		.hzd_WB_r2					(hzd_WB_r2),
		.alu_div_rdcntv_result		(alu_div_rdcntv_result),
		.alu_div_rdcntv_result_MEM	(alu_div_rdcntv_result_MEM),
		.rf_wdata_WB				(rf_wdata),	/*These three are forwarding */
		.has_int					(has_int),
		.EX_MEM_WB_ex_ertn			(EX_ex_ertn || MEM_ex_ertn || br_from_WB),
		.br_from_WB					(br_from_WB),

		.inst_final			(inst_final),
		.pc					(pc),
		.ecode				(ecode),

		.rf_we				(rf_we),
		.rf_waddr			(rf_waddr),
		.rf_wdata			(rf_wdata),
	
		.alu_op				(alu_op),
		.rj_value			(rj_value),
		.rkd_value			(rkd_value),
		.imm				(imm),
		.pc_ID				(pc_ID),
		.op_25_22			(op_25_22),
		.op_21_15			(op_21_15),
		.rf_raddr1			(rf_raddr1),
		.rf_raddr2			(rf_raddr2),
		.dest				(dest),
		.byte_we			(byte_we),
		.half_we			(half_we),
		.word_we			(word_we),
		.signed_we			(signed_we),
		.ld_inst			(ld_inst),
		.st_inst			(st_inst),
		.div_inst			(div_inst),
		.mul_inst			(mul_inst),
		.ertn_inst			(ertn_inst),
		.csr_inst			(csr_inst),
		.rdcn_inst			(rdcn_inst),
		.br_taken_sure		(br_taken_sure),
		.br_taken_yes		(br_taken_yes),
		.br_taken_no		(br_taken_no),
		.inst_tlbsrch		(inst_tlbsrch),
		.inst_tlbrd			(inst_tlbrd),
		.inst_tlbwr			(inst_tlbwr),
		.inst_tlbfill		(inst_tlbfill),
		.inst_invtlb		(inst_invtlb),
		.invtlb_op			(invtlb_op),
		.src1_is_pc			(src1_is_pc),
		.src2_is_imm		(src2_is_imm),
		.src2_is_4			(src2_is_4),
		.br_src_sel			(br_src_sel),
		.res_from_mem		(res_from_mem),
		.gpr_we				(gpr_we),
		.mem_we				(mem_we),
		.rf_ren1			(rf_ren1),
		.rf_ren2			(rf_ren2),
		.ecode_ID_m			(ecode_ID_m),
		.rdcn_op			(rdcn_op),
		.csr_we				(csr_we),
		.csr_wmask_en		(csr_wmask_en),
		.has_int_ID			(has_int_ID),
		.needs_refresh_ID	(needs_refresh_ID),
		.exc_from_IF_ID		(exc_from_IF_ID)
	);

/*
module stage_EX (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_ID, pipe_valid_EX,
	input  wire [`DWIDTH - 1: 0] tick, // tick signal, for rdcn instruction

	// input from ID
	input  wire [        12 : 0] alu_op,
	input  wire [`WIDTH - 1 : 0] rj_value, rkd_value, pc_ID, imm,
	input  wire [         3 : 0] op_25_22,
	input  wire [         6 : 0] op_21_15,
	input  wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest,
	input  wire                  byte_we, half_we, word_we, signed_we,
	input  wire                  ld_inst, st_inst, div_inst, mul_inst,
	input  wire                  ertn_inst, csr_inst, rdcn_inst,
	input  wire                  br_taken_sure, br_taken_yes, br_taken_no,
	input  wire					 inst_tlbsrch, inst_tlbrd, inst_tlbwr, inst_tlbfill, inst_invtlb,
	input  wire [`RADDR - 1 : 0] invtlb_op,
	input  wire                  src1_is_pc, src2_is_imm, src2_is_4, br_src_sel,
	input  wire                  res_from_mem, gpr_we, mem_we,
	input  wire [         5 : 0] ecode_ID_m,
	input  wire [         2 : 0] rdcn_op,
	input  wire                  csr_we, csr_wmask_en,
	input  wire                  has_int_ID, needs_refresh_ID,
	input  wire                  exc_from_IF_ID,

	// input from MEM, WB
	input  wire                  MEM_ex_ertn, br_from_WB,

	// output to MEM
	output wire [`WIDTH - 1 : 0] pc_EX, rj_value_EX, rkd_value_EX,
	output wire [`WIDTH - 1 : 0] alu_div_rdcntv_result, 
	output wire [`DWIDTH - 1: 0] mul_result, // mul - to WB
	output wire [         3 : 0] op_25_22_EX,
	output wire [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX,
	output wire                  byte_we_EX, half_we_EX, word_we_EX, signed_we_EX,
	output wire                  ld_inst_EX, st_inst_EX, csr_inst_EX, ertn_inst_EX, rdcn_inst_EX,
	output wire                  inst_tlbsrch_EX, inst_tlbrd_EX, inst_tlbwr_EX,
	output wire                  inst_tlbfill_EX, inst_invtlb_EX,
	output wire [`RADDR - 1 : 0] invtlb_op_EX,              // invtlb
	output wire                  mul_inst_EX,               // mul
	output wire [         2 : 0] mul_op_EX,                 // mul
	output wire                  res_from_mem_EX, gpr_we_EX, mem_we_EX,
	output wire                  has_int_EX, needs_refresh_EX,
	output wire                  exc_from_IF_EX,
	output wire                  EX_ex_ertn, // if any exception exists

	// output
	output wire [`WIDTH - 1 : 0] br_target, // next PC value, branch target
	output wire                  br_taken, calc_done,   // calculation ready, for mul/div instructions
	output wire [         5 : 0] ecode_EX_m,
	output wire [         2 : 0] rdcn_op_EX,

	output wire                  csr_we_EX, csr_wmask_en_EX,
	output wire [        13 : 0] csr_code_EX
);
endmodule
*/

	stage_EX stage_EX_inst (
		.clk					(clk),
		.reset					(reset),
	
		.pipe_tonext_valid_ID	(pipe_tonext_valid[1]),
		.pipe_valid_EX			(pipe_valid[2]),
		.tick					(tick),
	
		.alu_op					(alu_op),
		.rj_value				(rj_value),
		.rkd_value				(rkd_value),
		.pc_ID					(pc_ID),
		.imm					(imm),
		.op_25_22				(op_25_22),
		.op_21_15				(op_21_15),
		.rf_raddr1				(rf_raddr1),
		.rf_raddr2				(rf_raddr2),
		.dest					(dest),
		.byte_we				(byte_we),
		.half_we				(half_we),
		.word_we				(word_we),
		.signed_we				(signed_we),
		.ld_inst				(ld_inst),
		.st_inst				(st_inst),
		.div_inst				(div_inst),
		.mul_inst				(mul_inst),
		.ertn_inst				(ertn_inst),
		.csr_inst				(csr_inst),
		.rdcn_inst				(rdcn_inst),
		.br_taken_sure			(br_taken_sure),
		.br_taken_yes			(br_taken_yes),
		.br_taken_no			(br_taken_no),
		.inst_tlbsrch			(inst_tlbsrch),
		.inst_tlbrd				(inst_tlbrd),
		.inst_tlbwr				(inst_tlbwr),
		.inst_tlbfill			(inst_tlbfill),
		.inst_invtlb			(inst_invtlb),
		.invtlb_op				(invtlb_op),
		.src1_is_pc				(src1_is_pc),
		.src2_is_imm			(src2_is_imm),
		.src2_is_4				(src2_is_4),
		.br_src_sel				(br_src_sel),
		.res_from_mem			(res_from_mem),
		.gpr_we					(gpr_we),
		.mem_we					(mem_we),
		.ecode_ID_m				(ecode_ID_m),
		.rdcn_op				(rdcn_op),
		.csr_we					(csr_we),
		.csr_wmask_en			(csr_wmask_en),
		.has_int_ID				(has_int_ID),
		.needs_refresh_ID		(needs_refresh_ID),
		.exc_from_IF_ID			(exc_from_IF_ID),

		.MEM_ex_ertn			(MEM_ex_ertn),
		.br_from_WB				(br_from_WB),
	
		.pc_EX					(pc_EX),
		.rj_value_EX			(rj_value_EX),
		.rkd_value_EX			(rkd_value_EX),
		.alu_div_rdcntv_result	(alu_div_rdcntv_result),
		.mul_result				(mul_result),
		.op_25_22_EX			(op_25_22_EX),
		.rf_raddr1_EX			(rf_raddr1_EX),
		.rf_raddr2_EX			(rf_raddr2_EX),
		.dest_EX				(dest_EX),
		.byte_we_EX				(byte_we_EX),
		.half_we_EX				(half_we_EX),
		.word_we_EX				(word_we_EX),
		.signed_we_EX			(signed_we_EX),
		.ld_inst_EX				(ld_inst_EX),
		.st_inst_EX				(st_inst_EX),
		.csr_inst_EX			(csr_inst_EX),
		.ertn_inst_EX			(ertn_inst_EX),
		.rdcn_inst_EX			(rdcn_inst_EX),
		.inst_tlbsrch_EX		(inst_tlbsrch_EX),
		.inst_tlbrd_EX			(inst_tlbrd_EX),
		.inst_tlbwr_EX			(inst_tlbwr_EX),
		.inst_tlbfill_EX		(inst_tlbfill_EX),
		.inst_invtlb_EX			(inst_invtlb_EX),
		.invtlb_op_EX			(invtlb_op_EX),
		.mul_inst_EX			(mul_inst_EX),
		.mul_op_EX				(mul_op_EX),
		.res_from_mem_EX		(res_from_mem_EX),
		.gpr_we_EX				(gpr_we_EX),
		.mem_we_EX				(mem_we_EX),
		.has_int_EX				(has_int_EX),
		.needs_refresh_EX		(needs_refresh_EX),
		.exc_from_IF_EX			(exc_from_IF_EX),
		.EX_ex_ertn				(EX_ex_ertn),

		.br_target				(br_target),
		.br_taken				(br_taken),
		.calc_done				(calc_done),
		.ecode_EX_m				(ecode_EX_m),
		.rdcn_op_EX				(rdcn_op_EX),

		.csr_we_EX				(csr_we_EX),
		.csr_wmask_en_EX		(csr_wmask_en_EX),
		.csr_code_EX			(csr_code_EX)
	);

/*
module stage_MEM (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// connect to stage controller
	input  wire                  pipe_tonext_valid_EX, pipe_valid_MEM, pipe_ready_go_MEM, pipe_allowin_WB,

	// input from EX
	input  wire [`WIDTH - 1 : 0] pc_EX, alu_div_rdcntv_result, rj_value_EX, rkd_value_EX,
	input  wire [         3 : 0] op_25_22_EX,
	input  wire [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX,
	input  wire 				 byte_we_EX, half_we_EX, word_we_EX, signed_we_EX,
	input  wire                  ld_inst_EX, st_inst_EX, csr_inst_EX, ertn_inst_EX, rdcn_inst_EX,
	input  wire					 mul_inst_EX, // mul
	input  wire					 inst_tlbsrch_EX, inst_tlbrd_EX, inst_tlbwr_EX,
	input  wire					 inst_tlbfill_EX, inst_invtlb_EX,
	input  wire [`RADDR - 1 : 0] invtlb_op_EX,
	input  wire [		  2 : 0] mul_op_EX, // mul
	input  wire [		  2 : 0] rdcn_op_EX, // rdcn
	input  wire                  res_from_mem_EX, gpr_we_EX, mem_we_EX,
	input  wire [         5 : 0] ecode_EX_m,
	input  wire 				 csr_we_EX, csr_wmask_en_EX,
	input  wire [        13 : 0] csr_code_EX,
	input  wire 				 br_from_WB, // branch valid signal, at WB stage, calculated in cpu_top
	input  wire                  has_int_EX, needs_refresh_EX,
	input  wire					 exc_from_IF_EX,

	// output to WB
	output wire [`WIDTH - 1 : 0] pc_MEM, alu_div_rdcntv_result_MEM, rj_value_MEM, rkd_value_MEM,
	output reg  [`WIDTH - 1 : 0] ld_res_from_MEM,
	output wire [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM,
	output wire [         3 : 0] mask_dataram,
	output wire 				 byte_we_MEM, half_we_MEM, word_we_MEM, signed_we_MEM,
	output wire 				 ld_inst_MEM, csr_inst_MEM, ertn_inst_MEM, rdcn_inst_MEM,
	output wire					 inst_tlbrd_MEM, inst_tlbwr_MEM, inst_tlbfill_MEM,
	output wire					 mul_inst_MEM, // mul
	output wire [		  2 : 0] mul_op_MEM, // mul
	output wire 				 rdcn_op_MEM,  // only rdcn_op_EX[2] is useful at WB
	output wire 				 res_from_mem_MEM, gpr_we_MEM,
	output wire                  has_int_MEM, needs_refresh_MEM,
	output wire [         5 : 0] ecode_MEM_m,
	output wire 				 csr_we_MEM, csr_wmask_en_MEM,
	output wire [        13 : 0] csr_code_MEM,
	output wire					 exc_from_IF_MEM,

	// output to ID
	output wire					 MEM_ex_ertn,	// if any exception exists or is ertn

	// connect to main memory
	output wire                  data_sram_req,	// data SRAM request
	output wire					 data_sram_wr,	// data SRAM write
	output wire [		  1 : 0] data_sram_size,
	output wire [         3 : 0] data_sram_wstrb,	// data SRAM byte-writes
	output wire [`WIDTH - 1 : 0] data_sram_addr, data_sram_wdata,
	input  wire					 data_sram_addr_ok, data_sram_data_ok,
	input  wire [`WIDTH - 1 : 0] data_sram_rdata,

	// connect to CSR and TLB, for TLBSRCH and INVTLB
	input  wire					 csr_crmd_pg,
	input  wire [		  1 : 0] csr_crmd_plv,
	input  wire [`WIDTH - 1 : 0] csr_dmw0,
	input  wire [`WIDTH - 1 : 0] csr_dmw1,
	input  wire [		  9 : 0] csr_asid_asid,
	input  wire [		 18 : 0] csr_tlbehi_vppn,
	output wire					 csr_tlbidx_ne_we, csr_tlbidx_index_we,
	output wire					 w_tlbidx_ne,
	output wire[`LOG2TLBNUM-1:0] w_tlbidx_index,
	output wire [        18 : 0] tlb_s1_vppn,
	output wire					 tlb_s1_va_bit12,
	output wire [         9 : 0] tlb_s1_asid,
	input  wire					 tlb_s1_found,
	input  wire[`LOG2TLBNUM-1:0] tlb_s1_index,
	input  wire [        19 : 0] tlb_s1_ppn,
	input  wire [         5 : 0] tlb_s1_ps,
	input  wire [         1 : 0] tlb_s1_plv, tlb_s1_mat,
	input  wire					 tlb_s1_d, tlb_s1_v,
	output wire					 tlb_invtlb_valid,
	output wire [         4 : 0] tlb_invtlb_op
);
endmodule
*/

	stage_MEM stage_MEM_inst (
		.clk						(clk),
		.reset						(reset),

		.pipe_tonext_valid_EX		(pipe_tonext_valid[2]),
		.pipe_valid_MEM				(pipe_valid[3]),
		.pipe_ready_go_MEM			(pipe_ready_go[3]),
		.pipe_allowin_WB			(pipe_allowin[4]),

		.pc_EX						(pc_EX),
		.alu_div_rdcntv_result		(alu_div_rdcntv_result),
		.rj_value_EX				(rj_value_EX),
		.rkd_value_EX				(rkd_value_EX),
		.op_25_22_EX				(op_25_22_EX),
		.rf_raddr1_EX				(rf_raddr1_EX),
		.rf_raddr2_EX				(rf_raddr2_EX),
		.dest_EX					(dest_EX),
		.byte_we_EX					(byte_we_EX),
		.half_we_EX					(half_we_EX),
		.word_we_EX					(word_we_EX),
		.signed_we_EX				(signed_we_EX),
		.ld_inst_EX					(ld_inst_EX),
		.st_inst_EX					(st_inst_EX),
		.csr_inst_EX				(csr_inst_EX),
		.ertn_inst_EX				(ertn_inst_EX),
		.rdcn_inst_EX				(rdcn_inst_EX),
		.mul_inst_EX				(mul_inst_EX),
		.inst_tlbsrch_EX			(inst_tlbsrch_EX),
		.inst_tlbrd_EX				(inst_tlbrd_EX),
		.inst_tlbwr_EX				(inst_tlbwr_EX),
		.inst_tlbfill_EX			(inst_tlbfill_EX),
		.inst_invtlb_EX				(inst_invtlb_EX),
		.invtlb_op_EX				(invtlb_op_EX),
		.mul_op_EX					(mul_op_EX),
		.rdcn_op_EX					(rdcn_op_EX),
		.res_from_mem_EX			(res_from_mem_EX),
		.gpr_we_EX					(gpr_we_EX),
		.mem_we_EX					(mem_we_EX),
		.ecode_EX_m					(ecode_EX_m),
		.csr_we_EX					(csr_we_EX),
		.csr_wmask_en_EX			(csr_wmask_en_EX),
		.csr_code_EX				(csr_code_EX),
		.br_from_WB					(br_from_WB),
		.has_int_EX					(has_int_EX),
		.needs_refresh_EX			(needs_refresh_EX),
		.exc_from_IF_EX				(exc_from_IF_EX),

		.pc_MEM						(pc_MEM),
		.alu_div_rdcntv_result_MEM	(alu_div_rdcntv_result_MEM),
		.rj_value_MEM				(rj_value_MEM),
		.rkd_value_MEM				(rkd_value_MEM),
		.ld_res_from_MEM			(ld_res_from_MEM),
		.rf_raddr1_MEM				(rf_raddr1_MEM),
		.rf_raddr2_MEM				(rf_raddr2_MEM),
		.dest_MEM					(dest_MEM),
		.mask_dataram				(mask_dataram),
		.byte_we_MEM				(byte_we_MEM),
		.half_we_MEM				(half_we_MEM),
		.word_we_MEM				(word_we_MEM),
		.signed_we_MEM				(signed_we_MEM),
		.ld_inst_MEM				(ld_inst_MEM),
		.csr_inst_MEM				(csr_inst_MEM),
		.ertn_inst_MEM				(ertn_inst_MEM),
		.rdcn_inst_MEM				(rdcn_inst_MEM),
		.inst_tlbrd_MEM				(inst_tlbrd_MEM),
		.inst_tlbwr_MEM				(inst_tlbwr_MEM),
		.inst_tlbfill_MEM			(inst_tlbfill_MEM),
		.mul_inst_MEM				(mul_inst_MEM),
		.mul_op_MEM					(mul_op_MEM),
		.rdcn_op_MEM				(rdcn_op_MEM),
		.res_from_mem_MEM			(res_from_mem_MEM),
		.gpr_we_MEM					(gpr_we_MEM),
		.has_int_MEM				(has_int_MEM),
		.needs_refresh_MEM			(needs_refresh_MEM),
		.ecode_MEM_m				(ecode_MEM_m),
		.csr_we_MEM					(csr_we_MEM),
		.csr_wmask_en_MEM			(csr_wmask_en_MEM),
		.csr_code_MEM				(csr_code_MEM),
		.exc_from_IF_MEM			(exc_from_IF_MEM),

		.MEM_ex_ertn			(MEM_ex_ertn),

		.data_sram_req			(data_sram_req),
		.data_sram_wr			(data_sram_wr),
		.data_sram_size			(data_sram_size),
		.data_sram_wstrb		(data_sram_wstrb),
		.data_sram_addr			(data_sram_addr),
		.data_sram_wdata		(data_sram_wdata),
		.data_sram_addr_ok		(data_sram_addr_ok),
		.data_sram_data_ok		(data_sram_data_ok),
		.data_sram_rdata		(data_sram_rdata),

		.csr_crmd_pg			(csr_crmd_pg),
		.csr_crmd_plv			(csr_crmd_plv),
		.csr_dmw0				(csr_dmw0),
		.csr_dmw1				(csr_dmw1),
		.csr_asid_asid			(csr_asid_asid),
		.csr_tlbehi_vppn		(csr_tlbehi_vppn),
		.csr_tlbidx_ne_we		(csr_tlbidx_ne_we_MEM),
		.csr_tlbidx_index_we	(csr_tlbidx_index_we),
		.w_tlbidx_ne			(w_tlbidx_ne_MEM),
		.w_tlbidx_index			(w_tlbidx_index),
		.tlb_s1_vppn			(tlb_s1_vppn),
		.tlb_s1_va_bit12		(tlb_s1_va_bit12),
		.tlb_s1_asid			(tlb_s1_asid),
		.tlb_s1_found			(tlb_s1_found),
		.tlb_s1_index			(tlb_s1_index),
		.tlb_s1_ppn				(tlb_s1_ppn),
		.tlb_s1_ps				(tlb_s1_ps),
		.tlb_s1_plv				(tlb_s1_plv),
		.tlb_s1_mat				(tlb_s1_mat),
		.tlb_s1_d				(tlb_s1_d),
		.tlb_s1_v				(tlb_s1_v),
		.tlb_invtlb_valid		(tlb_invtlb_valid),
		.tlb_invtlb_op			(tlb_invtlb_op)
	);

/*
module stage_WB (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_MEM, // allow WB stage to accept new instruction
	input  wire                  pipe_valid_WB, // WB stage is valid
	input  wire 				 time_interupt,

	// input from MEM
	input  wire [`WIDTH - 1 : 0] pc_MEM, alu_div_rdcntv_result_MEM, rj_value_MEM, rkd_value_MEM,
	input  wire [`DWIDTH - 1: 0] mul_result,   // mul
	input  wire [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM,
	input  wire [         3 : 0] mask_dataram,
	input  wire                  byte_we_MEM, half_we_MEM, word_we_MEM, signed_we_MEM,
	input  wire					 csr_inst_MEM, ertn_inst_MEM, rdcn_inst_MEM,
	input  wire					 mul_inst_MEM, // mul
	input  wire					 inst_tlbrd_MEM, inst_tlbwr_MEM, inst_tlbfill_MEM,
	input  wire [ 		  2 : 0] mul_op_MEM, // mul
	input  wire 				 rdcn_op_MEM, // rdcn
	input  wire                  res_from_mem_MEM, gpr_we_MEM,
	input  wire [`WIDTH - 1 : 0] ld_res_from_MEM,
	input  wire [         5 : 0] ecode_MEM_m,
	input  wire 				 csr_we_MEM, csr_wmask_en_MEM,
	input  wire [        13 : 0] csr_code_MEM,
	input  wire                  has_int_MEM, needs_refresh_MEM,
	input  wire					 exc_from_IF_MEM,

	// input from csr module
	input  wire [`WIDTH - 1 : 0] csr_rvalue,

	// output
	output wire                  gpr_we_WB,

	output wire                  rf_we,			// register file write enable
	output wire [`RADDR - 1 : 0] rf_waddr,		// register file write address
	output wire [`RADDR - 1 : 0] dest_WB,		// destination register number
	output wire [`WIDTH - 1 : 0] rf_wdata,		// register file write data

	output wire [`WIDTH - 1 : 0] pc_WB,
	output wire [         3 : 0] debug_wb_rf_we,	// debug info
	output wire [`RADDR - 1 : 0] debug_wb_rf_wnum,	// debug info
	output wire [`WIDTH - 1 : 0] debug_wb_rf_wdata,	// debug info
	
	output wire [         8 : 0] csr_waddr,
	output wire 				 csr_we,
	output wire [`WIDTH - 1 : 0] csr_wmask,
	output wire [`WIDTH - 1 : 0] csr_wvalue,
	output wire [         8 : 0] csr_raddr,
	output wire 				 ertn_flush,
	output wire 				 WB_ex, needs_refresh_WB,
	output wire [        31 : 0] WB_vaddr,
	output wire [         5 : 0] ecode_noint, // ecode, without considering interrupt
	output wire [         8 : 0] WB_esubcode,

	output wire 				 ertn_inst_WB,
	output wire					 has_int_WB,
	output wire 				 rdcn_inst_WB,

	// Connect to CSR and TLB, for TLBRD, TLBWR and TLBFILL
	input  wire[`LOG2TLBNUM-1:0] csr_tlbidx_index,
	input  wire [		  9 : 0] csr_asid_asid,
	output wire					 csr_tlbidx_ne_we, csr_tlbidx_ps_we, csr_asid_asid_we,
	output wire					 w_tlbidx_ne,
	output wire [	      5 : 0] w_tlbidx_ps,
	output wire [		  9 : 0] w_asid_asid,
	output wire					 csr_tlbe_we,
	output wire [		 18 : 0] w_tlbehi_vppn,
	output wire [`WIDTH - 1 : 0] w_tlbelo0, w_tlbelo1,
	input  wire [		 18 : 0] csr_tlbehi_vppn,
	input  wire					 csr_tlbidx_ne,
	input  wire [		  5 : 0] csr_tlbidx_ps,
	input  wire [		  5 : 0] csr_estat_ecode,
	input  wire [`WIDTH - 1 : 0] csr_tlbelo0, csr_tlbelo1,
    output wire[`LOG2TLBNUM-1:0] tlb_r_index,
	input  wire            		 tlb_r_e,
	input  wire [ 		 18 : 0] tlb_r_vppn,
	input  wire [  	  	  5 : 0] tlb_r_ps,
	input  wire [  	  	  9 : 0] tlb_r_asid,
	input  wire           	 	 tlb_r_g,
	input  wire [ 		 19 : 0] tlb_r_ppn0,
	input  wire [	  	  1 : 0] tlb_r_plv0,
	input  wire [	  	  1 : 0] tlb_r_mat0,
	input  wire  	          	 tlb_r_d0,
	input  wire  	          	 tlb_r_v0,
	input  wire [ 		 19 : 0] tlb_r_ppn1,
	input  wire [	  	  1 : 0] tlb_r_plv1,
	input  wire [	  	  1 : 0] tlb_r_mat1,
	input  wire           		 tlb_r_d1,
	input  wire          		 tlb_r_v1,
    output wire		             tlb_we, //w(rite) e(nable)
    output wire[`LOG2TLBNUM-1:0] tlb_w_index,
    output wire		             tlb_w_e,
    output wire [ 		 18 : 0] tlb_w_vppn,
    output wire [		  5 : 0] tlb_w_ps,
    output wire [		  9 : 0] tlb_w_asid,
    output wire					 tlb_w_g,
    output wire [		 19 : 0] tlb_w_ppn0,
    output wire [		  1 : 0] tlb_w_plv0,
    output wire [		  1 : 0] tlb_w_mat0,
    output wire		             tlb_w_d0,
    output wire			         tlb_w_v0,
    output wire [		 19 : 0] tlb_w_ppn1,
    output wire [		  1 : 0] tlb_w_plv1,
    output wire [		  1 : 0] tlb_w_mat1,
    output wire		             tlb_w_d1,
    output wire 	             tlb_w_v1
);
endmodule
*/

	stage_WB stage_WB_inst (
		.clk		(clk),
		.reset		(reset),

		.pipe_tonext_valid_MEM		(pipe_tonext_valid[3]),
		.pipe_valid_WB				(pipe_valid[4]),
		.time_interupt				(time_interupt),

		.pc_MEM						(pc_MEM),
		.alu_div_rdcntv_result_MEM	(alu_div_rdcntv_result_MEM),
		.rj_value_MEM				(rj_value_MEM),
		.rkd_value_MEM				(rkd_value_MEM),
		.mul_result					(mul_result),
		.rf_raddr1_MEM				(rf_raddr1_MEM),
		.rf_raddr2_MEM				(rf_raddr2_MEM),
		.dest_MEM					(dest_MEM),
		.mask_dataram				(mask_dataram),
		.byte_we_MEM				(byte_we_MEM),
		.half_we_MEM				(half_we_MEM),
		.word_we_MEM				(word_we_MEM),
		.signed_we_MEM				(signed_we_MEM),
		.csr_inst_MEM				(csr_inst_MEM),
		.ertn_inst_MEM				(ertn_inst_MEM),
		.rdcn_inst_MEM				(rdcn_inst_MEM),
		.mul_inst_MEM				(mul_inst_MEM),
		.inst_tlbrd_MEM				(inst_tlbrd_MEM),
		.inst_tlbwr_MEM				(inst_tlbwr_MEM),
		.inst_tlbfill_MEM			(inst_tlbfill_MEM),
		.mul_op_MEM					(mul_op_MEM),
		.rdcn_op_MEM				(rdcn_op_MEM),
		.res_from_mem_MEM			(res_from_mem_MEM),
		.gpr_we_MEM					(gpr_we_MEM),
		.ld_res_from_MEM			(ld_res_from_MEM),
		.ecode_MEM_m				(ecode_MEM_m),
		.csr_we_MEM					(csr_we_MEM),
		.csr_wmask_en_MEM			(csr_wmask_en_MEM),
		.csr_code_MEM				(csr_code_MEM),
		.has_int_MEM				(has_int_MEM),
		.needs_refresh_MEM			(needs_refresh_MEM),
		.exc_from_IF_MEM			(exc_from_IF_MEM),

		.csr_rvalue			(csr_rvalue),

		.gpr_we_WB			(gpr_we_WB),
		
		.rf_we				(rf_we),
		.rf_waddr			(rf_waddr),
		.dest_WB			(dest_WB),
		.rf_wdata			(rf_wdata),

		.pc_WB				(pc_WB),
		.debug_wb_rf_we		(debug_wb_rf_we),
		.debug_wb_rf_wnum	(debug_wb_rf_wnum),
		.debug_wb_rf_wdata	(debug_wb_rf_wdata),

		.csr_waddr			(csr_waddr),
		.csr_we				(csr_we_WB),
		.csr_wmask			(csr_wmask),
		.csr_wvalue			(csr_wvalue),
		.csr_raddr			(csr_raddr),
		.ertn_flush			(ertn_flush),
		.WB_ex				(WB_ex),
		.needs_refresh_WB	(needs_refresh_WB),
		.WB_vaddr			(WB_vaddr),
		.ecode_noint		(ecode_noint),
		.WB_esubcode		(WB_esubcode),

		.ertn_inst_WB	(ertn_inst_WB),
		.has_int_WB		(has_int_WB),
		.rdcn_inst_WB	(rdcn_inst_WB),
	
		.csr_tlbidx_index	(csr_tlbidx[`LOG2TLBNUM - 1:0]),
		.csr_asid_asid		(csr_asid_asid),
		.csr_tlbidx_ne_we	(csr_tlbidx_ne_we_WB),
		.csr_tlbidx_ps_we	(csr_tlbidx_ps_we),
		.csr_asid_asid_we	(csr_asid_asid_we),
		.w_tlbidx_ne		(w_tlbidx_ne_WB),
		.w_tlbidx_ps		(w_tlbidx_ps),
		.w_asid_asid		(w_asid_asid),
		.csr_tlbe_we		(csr_tlbe_we),
		.w_tlbehi_vppn		(w_tlbehi_vppn),
		.w_tlbelo0			(w_tlbelo0),
		.w_tlbelo1			(w_tlbelo1),
		.csr_tlbehi_vppn	(csr_tlbehi_vppn),
		.csr_tlbidx_ne		(csr_tlbidx[31]),
		.csr_tlbidx_ps		(csr_tlbidx[29:24]),
		.csr_estat_ecode	(csr_estat_ecode),
		.csr_tlbelo0		(csr_tlbelo0),
		.csr_tlbelo1		(csr_tlbelo1),
		.tlb_r_index		(tlb_r_index),
		.tlb_r_e			(tlb_r_e),
		.tlb_r_vppn			(tlb_r_vppn),
		.tlb_r_ps			(tlb_r_ps),
		.tlb_r_asid			(tlb_r_asid),
		.tlb_r_g			(tlb_r_g),
		.tlb_r_ppn0			(tlb_r_ppn0),
		.tlb_r_plv0			(tlb_r_plv0),
		.tlb_r_mat0			(tlb_r_mat0),
		.tlb_r_d0			(tlb_r_d0),
		.tlb_r_v0			(tlb_r_v0),
		.tlb_r_ppn1			(tlb_r_ppn1),
		.tlb_r_plv1			(tlb_r_plv1),
		.tlb_r_mat1			(tlb_r_mat1),
		.tlb_r_d1			(tlb_r_d1),
		.tlb_r_v1			(tlb_r_v1),
		.tlb_we				(tlb_we),
		.tlb_w_index		(tlb_w_index),
		.tlb_w_e			(tlb_w_e),
		.tlb_w_vppn			(tlb_w_vppn),
		.tlb_w_ps			(tlb_w_ps),
		.tlb_w_asid			(tlb_w_asid),
		.tlb_w_g			(tlb_w_g),
		.tlb_w_ppn0			(tlb_w_ppn0),
		.tlb_w_plv0			(tlb_w_plv0),
		.tlb_w_mat0			(tlb_w_mat0),
		.tlb_w_d0			(tlb_w_d0),
		.tlb_w_v0			(tlb_w_v0),
		.tlb_w_ppn1			(tlb_w_ppn1),
		.tlb_w_plv1			(tlb_w_plv1),
		.tlb_w_mat1			(tlb_w_mat1),
		.tlb_w_d1			(tlb_w_d1),
		.tlb_w_v1			(tlb_w_v1)
	);

endmodule


module mycpu_top (
	input  wire        aclk,
	input  wire        aresetn,
	// cpu_axi interface for upper module
	output wire [ 3:0] arid,
	output wire [31:0] araddr,
	output wire [ 7:0] arlen,
	output wire [ 2:0] arsize,
	output wire [ 1:0] arburst,
	output wire [ 1:0] arlock,
	output wire [ 3:0] arcache,
	output wire [ 2:0] arprot,
	output wire        arvalid,
	input  wire        arready,

	input  wire [ 3:0] rid,
	input  wire [31:0] rdata,
	input  wire [ 1:0] rresp,
	input  wire        rlast,
	input  wire        rvalid,
	output wire        rready,

	output wire [ 3:0] awid,
	output wire [31:0] awaddr,
	output wire [ 7:0] awlen,
	output wire [ 2:0] awsize,
	output wire [ 1:0] awburst,
	output wire [ 1:0] awlock,
	output wire [ 3:0] awcache,
	output wire [ 2:0] awprot,
	output wire        awvalid,
	input  wire        awready,

	output wire [ 3:0] wid,
	output wire [31:0] wdata,
	output wire [ 3:0] wstrb,
	output wire        wlast,
	output wire        wvalid,
	input  wire        wready,

	input  wire [ 3:0] bid,
	input  wire [ 1:0] bresp,
	input  wire        bvalid,
	output wire        bready,
	// trace debug interface
	output wire [31:0] debug_wb_pc,
	output wire [ 3:0] debug_wb_rf_we,
	output wire [ 4:0] debug_wb_rf_wnum,
	output wire [31:0] debug_wb_rf_wdata
);
/* --------------------------------------------------------------------
	Reset Signal & Clock
  ------------------------------------------------------------------ */
	wire clk; // rename clk signal
	assign clk = aclk;

	wire reset;
	Reg_norst #(1) reset_mycpu_top_inst (clk, ~aresetn, reset, 1'b1);
/* --------------------------------------------------------------------
	Declarations
  ------------------------------------------------------------------ */

    // CPU interface wires declarations
	wire        cpu_clk; // bridge to CPU core
	wire        cpu_resetn; // bridge to CPU core

	wire        cpu_inst_sram_req;
	wire        cpu_inst_sram_wr;      // 0 (Read Only)
	wire [ 1:0] cpu_inst_sram_size;    // 2 (4 B)
	wire [ 3:0] cpu_inst_sram_wstrb;   // 0
	wire [31:0] cpu_inst_sram_addr;
	wire [31:0] cpu_inst_sram_wdata;   // 0
	wire        cpu_inst_sram_addr_ok; // bridge to CPU core
	wire        cpu_inst_sram_data_ok; // bridge to CPU core
	wire [31:0] cpu_inst_sram_rdata;   // bridge to CPU core

	wire        cpu_data_sram_req;
	wire        cpu_data_sram_wr;
	wire [ 1:0] cpu_data_sram_size; // 0: 1 B, 1: 2B, 2: 4B.
	wire [ 3:0] cpu_data_sram_wstrb;
	wire [31:0] cpu_data_sram_addr;
	wire [31:0] cpu_data_sram_wdata;
	wire        cpu_data_sram_addr_ok; // bridge to CPU core
	wire        cpu_data_sram_data_ok; // bridge to CPU core
	wire [31:0] cpu_data_sram_rdata; // bridge to CPU core

	wire [31:0] cpu_debug_wb_pc;
	wire [ 3:0] cpu_debug_wb_rf_we;
	wire [ 4:0] cpu_debug_wb_rf_wnum;
	wire [31:0] cpu_debug_wb_rf_wdata;

	// state machine
	parameter AR_STATE_NUM   = 3,
			  R_STATE_NUM    = 3,
			  AW_W_STATE_NUM = 4,
			  B_STATE_NUM    = 3;

	reg [AR_STATE_NUM   - 1 : 0] ar_state;
	reg [R_STATE_NUM    - 1 : 0] r_state;
	reg [AW_W_STATE_NUM - 1 : 0] aw_w_state;
	reg [B_STATE_NUM    - 1 : 0] b_state;

	parameter AR_IDLE = 3'b001,
			  AR_WAIT = 3'b010,
			  AR_DONE = 3'b100;
	parameter R_IDLE = 3'b001,
			  R_WAIT = 3'b010,
			  R_DONE = 3'b100;
	parameter AW_W_IDLE    = 4'b0001,
			  AW_W_WAIT_AW = 4'b0010,
			  AW_W_WAIT_W  = 4'b0100,
			  AW_W_DONE    = 4'b1000;
	parameter B_IDLE = 3'b001,
			  B_WAIT = 3'b010,
			  B_DONE = 3'b100;

	wire rd_req; // read request notation
	wire wr_req_no_rd; // write request notation (with no read request) NOTE: might be changed
	
	// axi interface of read address channel
	reg [ 3:0] reg_arid;
	reg [31:0] reg_araddr;
	reg [ 7:0] reg_arlen;
	reg [ 2:0] reg_arsize;
	reg [ 1:0] reg_arburst;
	reg [ 1:0] reg_arlock;
	reg [ 3:0] reg_arcache;
	reg [ 2:0] reg_arprot;
	// reg        reg_arvalid;
	reg        reg_arready;

	wire from_data_rd_req;

	// axi interface of read data channel
	reg [ 3:0] reg_rid;
	reg [31:0] reg_rdata; // temp, need to be changed
	reg        reg_rlast; // temp, need to be changed
	reg        reg_rvalid; // temp, need to be changed

	// axi interface of write address and write data channel
	reg [ 3:0] reg_awid;
	reg [31:0] reg_awaddr;
	reg [ 7:0] reg_awlen;
	reg [ 2:0] reg_awsize;
	reg [ 1:0] reg_awburst;
	reg [ 1:0] reg_awlock;
	reg [ 3:0] reg_awcache;
	reg [ 2:0] reg_awprot;
	// reg        reg_awvalid;

	reg [ 3:0] reg_wid;
	reg [31:0] reg_wdata;
	reg [ 3:0] reg_wstrb;
	reg        reg_wlast;
	// reg        reg_wvalid;

	wire from_data_wr_req;

	// axi interface of write response channel
	reg [3:0] reg_bid;

	// for calculate the signal sending back to CPU
	// wire icache_rd_addr_ok;
	wire rd_data_sram_addr_ok;
	// wire icache_rd_data_ok;
	wire rd_data_sram_data_ok;
	// wire icache_wr_addr_ok;
	wire wr_data_sram_addr_ok;
	// wire icache_wr_data_ok;
	wire wr_data_sram_data_ok;

/* --------------------------------------------------------------------
	Cache Interface
  ------------------------------------------------------------------ */

/*
module cache(
    input           clk,
    input           resetn,

    // CACHE <----> CPU
    input           valid,        // REQ valid
    input           op,           // 1 - write;  0 - read
    // input           bypass,       // 1 - uncacheable (bypass);  0 - cacheable
    input   [ 7:0]  index,        // ADDR[11:4]
    input   [19:0]  tag,          // gen(paddr) 
    input   [ 3:0]  offset,       // ADDR[3:0]
    input   [ 3:0]  wstrb,        // write strobe(en)
    input   [31:0]  wdata,        // write DATA      
    output          addr_ok,      // ADDR trans ok - read: ADDR recved; write: ADDR recved & data recved 
    output          data_ok,      // DATA trans ok - read: DATA returned; write: DATA written
    output  [31:0]  rdata,        // read DATA

    // CACHE <----> AXI
    output          rd_req,       // read REQ valid
    output  [ 2:0]  rd_type,      // 3'b000-b; 3'b001-h; 3'b010-w; 3'b100-cache line
    output  [31:0]  rd_addr,      // read begin ADDR
    input           rd_rdy,       // can recv read REQ (for handshake)
    input           ret_valid,    // read ret valid
    input           ret_last,     // read ret last
    input   [31:0]  ret_data,     // read ret DATA

    output          wr_req,       // write REQ valid
    output  [ 2:0]  wr_type,      // 3'b000-b; 3'b001-h; 3'b010-w; 3'b100-cache line
    output  [31:0]  wr_addr,      // write begin ADDR
    output  [ 3:0]  wr_wstrb,     // write strobe(en)
    output  [127:0] wr_data,      // write DATA
    input           wr_rdy        // can recv write REQ (for handshake); wr_rdy MUST be ahead of wr_req
);
endmodule
*/

	// wire         clk;
	// wire         resetn;
	wire         icache_valid;
	wire         icache_op;
	wire         icache_bypass;
	wire [  7:0] icache_index;
	wire [ 19:0] icache_tag;
	wire [  3:0] icache_offset;
	wire [  3:0] icache_wstrb;
	wire [ 31:0] icache_wdata;
	wire         icache_addr_ok;	// output to CPU core
	wire         icache_data_ok;	// output to CPU core
	wire [ 31:0] icache_rdata;		// output to CPU core

	wire         icache_rd_req;		// output to AXI
	wire [  2:0] icache_rd_type;	// output to AXI
	wire [ 31:0] icache_rd_addr;	// output to AXI
	wire         icache_rd_rdy;
	wire         icache_ret_valid;
	wire         icache_ret_last;
	wire [ 31:0] icache_ret_data;

	wire         icache_wr_req;		// output to AXI
	wire [  2:0] icache_wr_type;	// output to AXI
	wire [ 31:0] icache_wr_addr;	// output to AXI
	wire [  3:0] icache_wr_wstrb;	// output to AXI
	wire [127:0] icache_wr_data;	// output to AXI
	wire         icache_wr_rdy;

	assign icache_valid  = cpu_inst_sram_req;
	assign icache_op     = cpu_inst_sram_wr;
	assign icache_bypass = cpu_inst_sram_bypass; // new wire
	assign icache_index  = cpu_inst_sram_addr[11:4];
	assign icache_tag    = cpu_inst_sram_addr[31:12];
	assign icache_offset = cpu_inst_sram_addr[3:0];
	assign icache_wstrb  = cpu_inst_sram_wstrb;
	assign icache_wdata  = cpu_inst_sram_wdata;
	// assign icache_addr_ok;	// output to CPU core
	// assign icache_data_ok;	// output to CPU core
	// assign icache_rdata;		// output to CPU core

	// assign icache_rd_req;	// output to AXI
	// assign icache_rd_type;	// output to AXI
	// assign icache_rd_addr;	// output to AXI
	assign icache_rd_rdy    = reg_arready && reg_arid == 4'b0;
	assign icache_ret_valid = reg_rvalid && reg_rid == 4'b0;
	assign icache_ret_last  = reg_rlast && reg_rid == 4'b0;
	assign icache_ret_data  = reg_rdata;

	// assign icache_wr_req;	// output to AXI
	// assign icache_wr_type;	// output to AXI
	// assign icache_wr_addr;	// output to AXI
	// assign icache_wr_wstrb;	// output to AXI
	// assign icache_wr_data;	// output to AXI
	assign icache_wr_rdy = 1'b1;

	cache icache_instance (
		.clk		(clk),
		.resetn		(aresetn),

		.valid		(icache_valid),
		.op			(icache_op),
		.bypass		(icache_bypass),
		.index		(icache_index),
		.tag		(icache_tag),
		.offset		(icache_offset),
		.wstrb		(icache_wstrb),
		.wdata		(icache_wdata),
		.addr_ok	(icache_addr_ok),
		.data_ok	(icache_data_ok),
		.rdata		(icache_rdata),

		.rd_req		(icache_rd_req),
		.rd_type	(icache_rd_type),
		.rd_addr	(icache_rd_addr),
		.rd_rdy		(icache_rd_rdy),
		.ret_valid	(icache_ret_valid),
		.ret_last	(icache_ret_last),
		.ret_data	(icache_ret_data),

		.wr_req		(icache_wr_req),
		.wr_type	(icache_wr_type),
		.wr_addr	(icache_wr_addr),
		.wr_wstrb	(icache_wr_wstrb),
		.wr_data	(icache_wr_data),
		.wr_rdy		(icache_wr_rdy)
	);

/* --------------------------------------------------------------------
	Instantiation
  ------------------------------------------------------------------ */
	assign cpu_clk               = aclk;
	assign cpu_resetn            = aresetn;
	assign cpu_inst_sram_addr_ok = icache_addr_ok;
	assign cpu_inst_sram_data_ok = icache_data_ok;
	assign cpu_inst_sram_rdata   = icache_rdata;

	mycpu cpu_inner (
		.clk				(cpu_clk),
		.resetn				(cpu_resetn),

		.inst_sram_req		(cpu_inst_sram_req),
		.inst_sram_wr		(cpu_inst_sram_wr), // 0 (Read Only)
		.inst_sram_bypass	(cpu_inst_sram_bypass),
		.inst_sram_size		(cpu_inst_sram_size), // 2 (4 B)
		.inst_sram_wstrb	(cpu_inst_sram_wstrb), // 0
		.inst_sram_addr		(cpu_inst_sram_addr),
		.inst_sram_wdata	(cpu_inst_sram_wdata), // 0
		.inst_sram_addr_ok	(cpu_inst_sram_addr_ok),
		.inst_sram_data_ok	(cpu_inst_sram_data_ok),
		.inst_sram_rdata	(cpu_inst_sram_rdata),

		.data_sram_req		(cpu_data_sram_req),
		.data_sram_wr		(cpu_data_sram_wr),
		.data_sram_size		(cpu_data_sram_size),	// 0: 1 B, 1: 2B, 2: 4B.
		.data_sram_wstrb	(cpu_data_sram_wstrb),
		.data_sram_addr		(cpu_data_sram_addr),
		.data_sram_wdata	(cpu_data_sram_wdata),
		.data_sram_addr_ok	(cpu_data_sram_addr_ok),
		.data_sram_data_ok	(cpu_data_sram_data_ok),
		.data_sram_rdata	(cpu_data_sram_rdata),

		.debug_wb_pc		(cpu_debug_wb_pc),
		.debug_wb_rf_we		(cpu_debug_wb_rf_we),
		.debug_wb_rf_wnum	(cpu_debug_wb_rf_wnum),
		.debug_wb_rf_wdata	(cpu_debug_wb_rf_wdata)
	);


/* --------------------------------------------------------------------
	Four Different State Machines
  ------------------------------------------------------------------ */


	assign rd_req = icache_rd_req || cpu_data_sram_req && !cpu_data_sram_wr;
	assign wr_req_no_rd = !rd_req && cpu_data_sram_req && cpu_data_sram_wr;

	// read address channel
	always @(posedge clk) begin
		if (reset)
			ar_state <= AR_IDLE;
		else begin
			case (ar_state)
			AR_IDLE: begin
				if (rd_req && r_state == R_IDLE)
					ar_state <= AR_WAIT;
				else
					ar_state <= AR_IDLE;
			end
			AR_WAIT: begin
				if (arready && arvalid)
					ar_state <= AR_DONE;
				else
					ar_state <= AR_WAIT;
			end
			// AR_DONE: begin
			// 	ar_state <= AR_IDLE;
			// end
			default: begin
				ar_state <= AR_IDLE;
			end
			endcase
		end
	end

	// read data channel
	always @(posedge clk) begin
		if (reset)
			r_state <= R_IDLE;
		else begin
			case (r_state)
			R_IDLE: begin
				if (arready && arvalid && ar_state == AR_WAIT) // same with AR_WAIT -> AR_DONE
					r_state <= R_WAIT;
				else
					r_state <= R_IDLE;
			end
			R_WAIT: begin
				if (rvalid && rlast && rready)
					r_state <= R_DONE;
				else
					r_state <= R_WAIT;
			end
			// R_DONE: begin
			// 		r_state <= R_IDLE;
			// end
			default: begin
				r_state <= R_IDLE;
			end
			endcase
		end
	end

	// write address and write data channel
	always @(posedge clk) begin
		if (reset)
			aw_w_state <= AW_W_IDLE;
		else begin
			case (aw_w_state)
			AW_W_IDLE: begin
				if (wr_req_no_rd && b_state == B_IDLE)
					aw_w_state <= AW_W_WAIT_AW;
				else
					aw_w_state <= AW_W_IDLE;
			end
			AW_W_WAIT_AW: begin
				if (awready && awvalid)
					aw_w_state <= AW_W_WAIT_W;
				else
					aw_w_state <= AW_W_WAIT_AW;
			end
			AW_W_WAIT_W: begin
				if (wready && wvalid)
					aw_w_state <= AW_W_DONE;
				else
					aw_w_state <= AW_W_WAIT_W;
			end
			// AW_W_DONE: begin
			// 	aw_w_state <= AW_W_IDLE;
			// end
			default: begin
				aw_w_state <= AW_W_IDLE;
			end
			endcase
		end
	end

	// write response channel
	always @(posedge clk) begin
		if (reset)
			b_state <= B_IDLE;
		else begin
			case (b_state)
			B_IDLE: begin
				if (wready && wvalid && aw_w_state == AW_W_WAIT_W) // same with AW_W_WAIT_W -> AW_W_DONE
					b_state <= B_WAIT;
				else
					b_state <= B_IDLE;
			end
			B_WAIT: begin
				if (bvalid)
					b_state <= B_DONE;
				else
					b_state <= B_WAIT;
			end
			// B_DONE: begin
			// 	b_state <= B_IDLE;
			// end
			default: begin
				b_state <= B_IDLE;
			end
			endcase
		end
	end

/* --------------------------------------------------------------------
	Read Address Channel
  ------------------------------------------------------------------ */

	assign from_data_rd_req = cpu_data_sram_req && !cpu_data_sram_wr;
	wire [1:0] icache_arsize_low2bit = {icache_rd_type[1] | ~icache_bypass, icache_rd_type[0]};

	always @(posedge clk) begin
		if (ar_state == AR_IDLE) begin
			reg_arid    <= from_data_rd_req ? 4'd1 : 4'd0;
			reg_araddr  <= from_data_rd_req ? cpu_data_sram_addr : icache_rd_addr;
			reg_arlen   <= (from_data_rd_req || icache_bypass) ? 8'd0 : 8'd3;
			reg_arsize  <= {1'b0, from_data_rd_req ? cpu_data_sram_size : icache_arsize_low2bit};
			reg_arburst <= 2'b01;
			reg_arlock  <= 2'b00;
			reg_arcache <= 4'b0000;
			reg_arprot  <= 3'b000;
		end
	end

	always @(posedge clk) begin
        if (reset) begin
            reg_arready <= 1'b0;
        end
		else begin
			reg_arready <= arready;
		end
	end

	// assign icache_rd_addr_ok = ar_state == AR_DONE && reg_arid == 4'b0;
	assign rd_data_sram_addr_ok = ar_state == AR_DONE && reg_arid == 4'b1;


/* --------------------------------------------------------------------
	Read Data Channel
  ------------------------------------------------------------------ */

	always @(posedge clk) begin
		if (reset) begin
			reg_rid    <= 4'b0;
			reg_rdata  <= 32'b0;
			reg_rlast  <= 1'b0;
			reg_rvalid <= 1'b0;
		end
		else if (r_state == R_WAIT) begin
			reg_rid    <= rid;
			reg_rdata  <= rdata;
			reg_rlast  <= rlast;
			reg_rvalid <= rvalid;
		end
		else if (r_state == R_DONE) begin
			reg_rvalid <= 1'b0;
		end
	end

	// assign icache_rd_data_ok = r_state == R_DONE && reg_rid == 4'b0;
	assign rd_data_sram_data_ok = r_state == R_DONE && reg_rid == 4'b1;

	// assign icache_ret_data     = reg_rdata;
	assign cpu_data_sram_rdata = reg_rdata;


/* --------------------------------------------------------------------
	Write Address Channel & Write Channel
  ------------------------------------------------------------------ */

	assign from_data_wr_req = cpu_data_sram_req &&  cpu_data_sram_wr;
	// wire [1:0] icache_awsize_low2bit = {icache_wr_type[1] | ~icache_bypass, icache_wr_type[0]};

	always @(posedge clk) begin
		if (aw_w_state == AW_W_IDLE) begin
			reg_awid    <= 4'd1; // actually only 1'b1 is possible, for inst sram dose not write
			reg_awaddr  <= cpu_data_sram_addr;
			reg_awlen   <= 8'd0;
			reg_awsize  <= {1'b0, cpu_data_sram_size};
			reg_awburst <= 2'b01;
			reg_awlock  <= 2'b00;
			reg_awcache <= 4'b0000;
			reg_awprot  <= 3'b000;

			reg_wid    <= 4'd1; // actually only 1'b1 is possible, for inst sram dose not write
			reg_wdata  <= cpu_data_sram_wdata;
			reg_wstrb  <= cpu_data_sram_wstrb;
			reg_wlast  <= 1'b1;
		end
	end

	// assign icache_wr_addr_ok = aw_w_state == AW_W_DONE && reg_wid == 4'b0;
	assign wr_data_sram_addr_ok = aw_w_state == AW_W_DONE && reg_wid == 4'b1;


/* --------------------------------------------------------------------
	Write Response Channel
  ------------------------------------------------------------------ */

	always @(posedge clk) begin
		if (reset) begin
			reg_bid <= 4'b0;
		end
		else if (b_state == B_WAIT) begin
			reg_bid <= bid;
		end
	end

	// assign icache_wr_data_ok = b_state == B_DONE && reg_bid == 4'b0;
	assign wr_data_sram_data_ok = b_state == B_DONE && reg_bid == 4'b1;


/* --------------------------------------------------------------------
	To CPU OK Signal
  ------------------------------------------------------------------ */
	// NOTE: two situations are mutually exclusive, so we simply use OR to combine them
	// assign icache_addr_ok        = icache_rd_addr_ok    || icache_wr_addr_ok;
	assign cpu_data_sram_addr_ok = rd_data_sram_addr_ok || wr_data_sram_addr_ok;
	// assign icache_data_ok        = icache_rd_data_ok    || icache_wr_data_ok;
	assign cpu_data_sram_data_ok = rd_data_sram_data_ok || wr_data_sram_data_ok;


/* --------------------------------------------------------------------
	Top Interface to AXI Bus
  ------------------------------------------------------------------ */
	// wire [ 3:0] arid;
	// wire [31:0] araddr;
	// wire [ 7:0] arlen;
	// wire [ 2:0] arsize;
	// wire [ 1:0] arburst;
	// wire [ 1:0] arlock;
	// wire [ 3:0] arcache;
	// wire [ 2:0] arprot;
	// wire        arvalid;
	// wire        arready; // input
	assign arid    = reg_arid;
	assign araddr  = reg_araddr;
	assign arlen   = reg_arlen;
	assign arsize  = reg_arsize;
	assign arburst = reg_arburst;
	assign arlock  = reg_arlock;
	assign arcache = reg_arcache;
	assign arprot  = reg_arprot;
	assign arvalid = ar_state == AR_WAIT && !reset;

	// wire [ 3:0] rid; // input
	// wire [31:0] rdata;   // input
	// wire [ 1:0] rresp;   // input
	// wire        rlast;   // input
	// wire        rvalid;  // input
	// wire        rready;
	assign rready = r_state == R_WAIT || reset;

	// wire [ 3:0] awid;
	// wire [31:0] awaddr;
	// wire [ 7:0] awlen;
	// wire [ 2:0] awsize;
	// wire [ 1:0] awburst;
	// wire [ 1:0] awlock;
	// wire [ 3:0] awcache;
	// wire [ 2:0] awprot;
	// wire        awvalid;
	// wire        awready; // input
	assign awid    = reg_awid; 
	assign awaddr  = reg_awaddr;
	assign awlen   = reg_awlen;
	assign awsize  = reg_awsize;
	assign awburst = reg_awburst;
	assign awlock  = reg_awlock;
	assign awcache = reg_awcache;
	assign awprot  = reg_awprot;
	assign awvalid = aw_w_state == AW_W_WAIT_AW && !reset;

	// wire [ 3:0] wid;
	// wire [31:0] wdata;
	// wire [ 3:0] wstrb;
	// wire        wlast;
	// wire        wvalid;
	// wire        wready;  // input
	assign wid    = reg_wid;
	assign wdata  = reg_wdata;
	assign wstrb  = reg_wstrb;
	assign wlast  = reg_wlast;
	assign wvalid = aw_w_state == AW_W_WAIT_W && !reset;

	// wire [ 3:0] bid; // input
	// wire [ 1:0] bresp;   // input
	// wire        bvalid;  // input
	// wire        bready;
	assign bready = b_state == B_WAIT || reset;


/* --------------------------------------------------------------------
	Trace Debug Interface
  ------------------------------------------------------------------ */
	assign debug_wb_pc       = cpu_debug_wb_pc;
	assign debug_wb_rf_we    = cpu_debug_wb_rf_we;
	assign debug_wb_rf_wnum  = cpu_debug_wb_rf_wnum;
	assign debug_wb_rf_wdata = cpu_debug_wb_rf_wdata;

endmodule
