`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000
`define TRUE	1'b1
`define FALSE	1'b0

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
	input  wire					 WB_ex, ertn_flush,
	input  wire	[`WIDTH - 1 : 0] ex_entry, era_value,
	
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
	output wire [         5 : 0] ecode
);

	reg  flag_first_IF;
	wire [`WIDTH - 1 : 0] inst_IF_reg;	// holds instruction until ID allows in
	wire [`WIDTH - 1 : 0] nextpc; // next PC value

	/* --------------------------------------
		IF stage
	-------------------------------------- */

	// Reg #(width, reset_val) inst_name (clk, rst, input, output, wen);

	// pc register
	//Reg #(`WIDTH, `WIDTH'h1bfffffc) pc_reg_inst (clk, reset, nextpc, pc, pipe_allowin_IF || br_taken || br_from_WB);
	always @(posedge clk) begin
		if (reset)
			pc <= 32'h1bfffffc;
		else if (pipe_ready_go_preIF)
			/*
			 * Note that we only send request to I-SRAM when pipe_allowin_IF is 1,
			 * so pipe_ready_go_preIF being 1 means that pipe_allowin_IF is also 1,
			 * which indicates that the instruction can move to stage IF.
			 */
			pc <= nextpc;
	end

	// store inst into a register, in case of it staying more than 1 cycle
	// flag_first_IF
	always @(posedge clk) begin
		if (reset)
			flag_first_IF <= 0;
		else if (pipe_ready_go_preIF)
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
				(br_taken || WB_ex || ertn_flush))
			/*
			 * Set a flag to clear the next instruction,
			 * even if pipe_valid[0] is NOT 1.
			 */
			flag_cancel_IF <= 1;
	end

	assign pipe_ready_go_IF = pipe_valid_IF &&
		(inst_sram_data_ok || !flag_first_IF) && !flag_cancel_IF;

	// instruction reading and storing
	// get final instruction
	Reg #(`WIDTH, `WIDTH'b0) inst_IF_reg_inst
		(clk, reset, inst_sram_rdata, inst_IF_reg, flag_first_IF && inst_sram_data_ok);

	assign inst_final = flag_first_IF ? inst_sram_rdata : inst_IF_reg;

	// PC not aligned
	wire pc_n_align = (pc[1:0] != 2'b00);

	// ADEF
	assign ecode = pc_n_align ? 6'h8 : 6'b000000;

	/* --------------------------------------
		pre-IF stage
	-------------------------------------- */

	reg [`WIDTH - 1 : 0] br_ex_ertn_target;
	reg br_ex_ertn_target_valid;
	reg flag_cancel_preIF;

	assign inst_sram_addr = nextpc;

	/*
	 * Only when pipe_allowin_IF is 1 can preIF send request to I-SRAM,
	 * so when pipe_ready_go_preIF is 1, pipe_allowin_IF is also 1.
	 */
	assign inst_sram_req = pipe_allowin_IF && !reset;
	assign pipe_ready_go_preIF = inst_sram_req && inst_sram_addr_ok;

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
			/* Exception has the highest priority */
			br_ex_ertn_target <= ex_entry;
		else if (ertn_flush)
			/* Infact, WB_ex and ertn_flush cannot be 1 at the same time */
			br_ex_ertn_target <= era_value;
		else if (br_taken)
			br_ex_ertn_target <= br_target;
	end

	assign nextpc = (br_ex_ertn_target_valid && !flag_cancel_preIF
		? br_ex_ertn_target : pc + 32'd4);

endmodule