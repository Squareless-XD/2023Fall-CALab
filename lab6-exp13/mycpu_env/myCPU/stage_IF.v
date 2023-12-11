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

	// input from stage controller
	input  wire                  pipe_allowin_IF, // allow IF stage to accept new instruction
	input  wire                  pipe_tonext_valid_IF, // IF stage is ready to go
	input  wire                  pipe_valid_IF, // IF stage is valid
	input  wire                  br_from_WB,
	input  wire [`WIDTH - 1 : 0] nextpc, // next PC value, branch target
	
	// input from pre-IF stage
	input  wire [`WIDTH - 1 : 0] inst_sram_rdata,	// instruction memory read data
	
	// input from EX stage
	input  wire                  br_taken, // branch taken or not
	
	// output to ID (not only)
	output wire [`WIDTH - 1 : 0] inst_final, // final instruction
	output wire [`WIDTH - 1 : 0] pc, // PC
	output wire [         5 : 0] ecode
);

	// reg  [`WIDTH - 1 : 0] pc; // PC
	wire first_IF;
	wire [`WIDTH - 1 : 0] inst; // instruction
	wire [`WIDTH - 1 : 0] inst_IF_reg;
	// wire [`WIDTH - 1 : 0] inst_final; // final instruction

	/* --------------------------------------
		IF stage
	-------------------------------------- */

	// Reg #(width, reset_val) inst_name (clk, rst, input, output, wen);

	// pc register
	Reg #(`WIDTH, `WIDTH'h1bfffffc) pc_reg_inst (clk, reset, nextpc, pc, pipe_allowin_IF || br_taken || br_from_WB);

	// store inst into a register, in case of IF staying more than 1 cycle
	Reg #(1, 1'b1) first_IF_inst (clk, reset || br_taken || br_from_WB || pipe_tonext_valid_IF, !pipe_valid_IF, first_IF, `TRUE);

	// instruction reading and storing
	assign inst = inst_sram_rdata; // instruction memory read data

	// get final instruction
	Reg #(`WIDTH, `WIDTH'b0) inst_IF_reg_inst (clk, reset, inst, inst_IF_reg, first_IF);

	assign inst_final = first_IF ? inst : inst_IF_reg;

	// PC not aligned
	wire pc_n_align = (pc[1:0] != 2'b00);

	// ADEF
	assign ecode = pc_n_align ? 6'h8 : 6'b000000;

endmodule