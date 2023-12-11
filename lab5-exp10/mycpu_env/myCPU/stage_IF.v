`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000

module stage_IF (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_allowin_IF, // allow IF stage to accept new instruction
	input  wire                  pipe_tonext_valid_IF, // IF stage is ready to go
	input  wire                  pipe_valid_IF, // IF stage is valid
	input  wire [`WIDTH - 1 : 0] nextpc, // next PC value, branch target
	
	// input from pre-IF stage
	input  wire [`WIDTH - 1 : 0] inst_sram_rdata,	// instruction memory read data
	
	// input from EX stage
	input  wire                  br_taken, // branch taken or not
	
	// output to ID (not only)
	output wire [`WIDTH - 1 : 0] inst_final, // final instruction
	output reg  [`WIDTH - 1 : 0] pc	// PC
);

	// reg  [`WIDTH - 1 : 0] pc; // PC
	reg  first_IF;
	wire [`WIDTH - 1 : 0] inst; // instruction
	reg  [`WIDTH - 1 : 0] inst_IF_reg;
	// wire [`WIDTH - 1 : 0] inst_final; // final instruction

	/* --------------------------------------
		IF stage
	-------------------------------------- */

	// pc register
	always @(posedge clk) begin
		if (reset) begin
			pc <= `WIDTH'h1bfffffc; // trick: to make nextpc be 0x1c000000 during reset
		end else if (br_taken || pipe_allowin_IF) begin
			pc <= nextpc;
		end
	end

	// store inst into a register, in case of IF staying more than 1 cycle
	always @(posedge clk) begin
		if (reset || br_taken || pipe_tonext_valid_IF) begin
			first_IF <= 1'b1;
		end else begin
			first_IF <= !pipe_valid_IF;
		end
	end

	// instruction reading and storing
	assign inst = inst_sram_rdata; // instruction memory read data
	always @(posedge clk) begin
		if (first_IF) begin
			inst_IF_reg <= inst;
		end
	end
	assign inst_final = first_IF ? inst : inst_IF_reg;

endmodule