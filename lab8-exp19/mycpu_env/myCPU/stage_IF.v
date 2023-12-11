`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000
`define TRUE	1'b1
`define FALSE	1'b0
`define LOG2TLBNUM 4
`define SMALL_PAGE_SIZE 6'd12

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
	/*
	 * tlb_s0_asid is going to be driven in mycpu,
	 * while tlb_s1_asid is driven in stage_MEM.
	 */
	input  wire					 tlb_s0_found,
	input  wire[`LOG2TLBNUM-1:0] tlb_s0_index,
	input  wire [        19 : 0] tlb_s0_ppn,
	input  wire [         5 : 0] tlb_s0_ps,
	input  wire [         1 : 0] tlb_s0_plv, tlb_s0_mat,
	input  wire					 tlb_s0_d, tlb_s0_v
);

	reg  flag_first_IF;
	wire [`WIDTH - 1 : 0] inst_IF_reg;	// holds instruction until ID allows in
	wire [`WIDTH - 1 : 0] nextpc; // next PC value
	wire flag_dmw0_hit, flag_dmw1_hit;
	wire flag_tlb_hit;
	wire [         5 : 0] ecode_MMU_preIF;
	reg  [		   5 : 0] ecode_MMU_IF;

	/* --------------------------------------
		IF stage
	-------------------------------------- */

	// Reg #(width, reset_val) inst_name (clk, rst, input, output, wen);

	// pc, ecode_MMU_IF
	always @(posedge clk) begin
		if (reset) begin
			pc <= 32'h1bfffffc;
			ecode_MMU_IF <= 6'd0;
		end
		else if (pipe_ready_go_preIF && pipe_allowin_IF) begin
			/*
			 * Note that we only send request to I-SRAM when pipe_allowin_IF is 1,
			 * so pipe_ready_go_preIF being 1 means that pipe_allowin_IF is also 1,
			 * before exp19, which indicates that the instruction can move to stage IF.
			 * However, since exp19, pipe_ready_go_preIF can be 1 even if pipe_allowin_IF is 0.
			 * So we need to check pipe_allowin_IF here.
			 */
			pc <= nextpc;
			ecode_MMU_IF <= ecode_MMU_preIF;
		end
	end

	// store inst into a register, in case of it staying more than 1 cycle
	// flag_first_IF
	always @(posedge clk) begin
		if (reset)
			flag_first_IF <= 0;
		else if (pipe_ready_go_preIF && pipe_allowin_IF)
			/*
			 * Instruction hasn't been fetched (...data_ok is 0), or
			 * it is at the first cycle of instruction (...data_ok is 1).
			 */
			flag_first_IF <= 1;
		else if (inst_sram_data_ok)
			/*
			 * Instruction is stored in inst_IF_reg,
			 * or has been accepted by stage ID.
			 */
			flag_first_IF <= 0;
	end

	// flag_cancel_IF
	always @(posedge clk) begin
		if (reset)
			flag_cancel_IF <= 0;
		else if (flag_cancel_IF && inst_sram_data_ok)
			flag_cancel_IF <= 0;
		else if ((!pipe_ready_go_IF || pipe_ready_go_IF && pipe_allowin_IF) &&
				(br_taken || WB_ex || ertn_flush) &&
				!(pipe_valid_IF && ecode_MMU_IF != 6'h00 || ecode_MMU_preIF != 6'h00))
			/*
			 * Set a flag to clear the next instruction,
			 * even if pipe_valid[0] is NOT 1.
			 * If MMU exception exists, don't set this flag! If that happens,
			 * inst_sram_req is set to 0, so we don't need to set this flag.
			 */
			flag_cancel_IF <= 1;
	end

	assign pipe_ready_go_IF = pipe_valid_IF && (
		(inst_sram_data_ok || !flag_first_IF) && !flag_cancel_IF
		|| ecode_MMU_IF != 6'h00);

	// instruction reading and storing
	// get final instruction
	Reg #(`WIDTH, `WIDTH'b0) inst_IF_reg_inst
		(clk, reset, inst_sram_rdata, inst_IF_reg, flag_first_IF && inst_sram_data_ok);

	assign inst_final = flag_first_IF ? inst_sram_rdata : inst_IF_reg;

	// PC not aligned
	wire pc_n_align = (nextpc[1:0] != 2'b00);

	// Ecode in IF stage
	// Note that ADEF (instruction address error) is regarded as a MMU exception since exp19.
	assign ecode = ecode_MMU_IF;

	/* --------------------------------------
		pre-IF stage
	-------------------------------------- */

	reg [`WIDTH - 1 : 0] br_ex_ertn_target;
	reg br_ex_ertn_target_valid;
	reg flag_cancel_preIF;

	assign tlb_s0_vppn = nextpc[31:13];
	assign tlb_s0_va_bit12 = nextpc[12];
	assign flag_dmw0_hit = csr_crmd_pg && nextpc[31:29] == csr_dmw0[31:29]
						&& csr_dmw0[csr_crmd_plv] == 1'b1;
	/*
	 * What if both DMW0 and DMW1 hit? I don't know.
	 * So just deal with it like this.
	 */
	assign flag_dmw1_hit = csr_crmd_pg && nextpc[31:29] == csr_dmw1[31:29]
						&& csr_dmw1[csr_crmd_plv] == 1'b1 && !flag_dmw0_hit;
	assign flag_tlb_hit = !flag_dmw0_hit && !flag_dmw1_hit
						&& tlb_s0_found && csr_crmd_plv <= tlb_s0_plv;
	assign inst_sram_addr = !csr_crmd_pg ? nextpc : (
					{32{flag_dmw0_hit}} & {csr_dmw0[27:25], nextpc[28:0]} |
					{32{flag_dmw1_hit}} & {csr_dmw1[27:25], nextpc[28:0]} |
					{32{flag_tlb_hit }} & {tlb_s0_ppn[19:9],
						tlb_s0_ps == `SMALL_PAGE_SIZE ? tlb_s0_ppn[8:0] : nextpc[20:12],
						nextpc[11:0]
					}
	);
	assign ecode_MMU_preIF = pc_n_align ? 6'h8 :
				{6{csr_crmd_pg && !flag_dmw0_hit && !flag_dmw1_hit}} & (
				!tlb_s0_found	? 6'h3F :	// TLB refill
				!tlb_s0_v		? 6'h03 :	// inst page fault
				!flag_tlb_hit	? 6'h07 :	// page previlege fault
				6'h00						// no MMU exception
	);

	/*
	 * Only when pipe_allowin_IF is 1 can preIF send request to I-SRAM,
	 * so when pipe_ready_go_preIF is 1, pipe_allowin_IF is also 1.
	 * Besides, if MMU exception exists, don't send request!
	 */
	assign inst_sram_req = pipe_allowin_IF && !reset
						&& ecode_MMU_preIF == 6'h00
						&& !(pipe_valid_IF && ecode_MMU_IF != 6'h00);
	assign pipe_ready_go_preIF = inst_sram_req && inst_sram_addr_ok
						|| ecode_MMU_preIF != 6'h00;

	// flag_cancel_preIF
	always @(posedge clk) begin
		if (reset)
			flag_cancel_preIF <= 0;
		else if ((br_taken || WB_ex || ertn_flush) &&
			(inst_sram_req && !inst_sram_addr_ok))
			/*
			 * When a request is waiting for OK and branch signal comes,
			 * set flag_cancel_preIF and change nextpc after OK.
			 */
			flag_cancel_preIF <= 1;
		else if (flag_cancel_preIF && inst_sram_req && inst_sram_addr_ok)
			flag_cancel_preIF <= 0;
	end

	// br_ex_ertn_target_valid
	always @(posedge clk) begin
		if (reset)
			br_ex_ertn_target_valid <= 0;
		else if (br_taken || WB_ex || ertn_flush)
			br_ex_ertn_target_valid <= 1;
		else if (!flag_cancel_preIF && inst_sram_req && inst_sram_addr_ok)
			br_ex_ertn_target_valid <= 0;
	end

	// br_ex_ertn_target
	always @(posedge clk) begin
		if (reset)
			br_ex_ertn_target <= 32'h0;
		else if (WB_ex)
			/* Refresh and exception have the highest priority */
			br_ex_ertn_target <= (needs_refresh_WB ? pc_WB : ex_entry);
		else if (ertn_flush)
			/* Infact, WB_ex and ertn_flush cannot be 1 at the same time */
			br_ex_ertn_target <= era_value;
		else if (br_taken)
			br_ex_ertn_target <= br_target;
	end

	assign nextpc = (br_ex_ertn_target_valid && !flag_cancel_preIF
		? br_ex_ertn_target : pc + 32'd4);

endmodule