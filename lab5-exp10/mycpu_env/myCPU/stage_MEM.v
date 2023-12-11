`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000

module stage_MEM (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_EX, // allow MEM stage to accept new instruction
	input  wire                  pipe_valid_MEM, // MEM stage is valid

	// input from EX
	input  wire [`WIDTH - 1 : 0] pc_EX, alu_result, rkd_value_EX,
	input  wire [         3 : 0] op_25_22_EX,
	input  wire [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX,
	input  wire                  ld_inst_EX, st_inst_EX,
	input  wire					 mul_inst_EX, // mul
	input  wire [		  2 : 0] mul_op_EX, // mul
	input  wire                  res_from_mem_EX, gr_we_EX, mem_we_EX,

	// output to WB
	output reg  [`WIDTH - 1 : 0] pc_MEM, alu_result_MEM,
	// output wire [`WIDTH - 1 : 0] mem_result,
	output reg  [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM,
	output wire [         3 : 0] mask_dataram,
	output wire                  byte_we, half_we, word_we, signed_we,
	output reg                   ld_inst_MEM,
	output reg					 mul_inst_MEM, // mul
	output reg  [		  2 : 0] mul_op_MEM, // mul
	output reg                   res_from_mem_MEM, gr_we_MEM,

	// output
	output wire                  data_sram_en,	// data SRAM port enable pin
	output wire [         3 : 0] data_sram_we,	// data SRAM byte-writes
	output wire [`WIDTH - 1 : 0] data_sram_addr, data_sram_wdata
);

	/* --------------------------------------
		EX -> MEM
	-------------------------------------- */

	// reg [`WIDTH - 1 : 0] pc_MEM;
	// reg [`WIDTH - 1 : 0] alu_result_MEM;
	reg [`WIDTH - 1 : 0] rkd_value_MEM;

	reg [3:0] op_25_22_MEM;
	// reg [`RADDR - 1 : 0] rf_raddr1_MEM;
	// reg [`RADDR - 1 : 0] rf_raddr2_MEM;
	// reg [`RADDR - 1 : 0] dest_MEM;

	// reg ld_inst_MEM;
	reg st_inst_MEM;

	// reg res_from_mem_MEM;
	// reg gr_we_MEM;
	reg mem_we_MEM;

	always @(posedge clk) begin
		if (pipe_tonext_valid_EX) begin
			pc_MEM         <= pc_EX;
			alu_result_MEM <= alu_result;
			rkd_value_MEM  <= rkd_value_EX;

			op_25_22_MEM  <= op_25_22_EX;
			rf_raddr1_MEM <= rf_raddr1_EX;
			rf_raddr2_MEM <= rf_raddr2_EX;
			dest_MEM      <= dest_EX;
		
			ld_inst_MEM <= ld_inst_EX;
			st_inst_MEM <= st_inst_EX;

			mul_inst_MEM <= mul_inst_EX; // mul
			mul_op_MEM   <= mul_op_EX; // mul

			res_from_mem_MEM <= res_from_mem_EX;
			gr_we_MEM        <= gr_we_EX;
			mem_we_MEM       <= mem_we_EX;
		end
	end

	/* --------------------------------------
		MEM stage
	-------------------------------------- */

	assign byte_we = (op_25_22_MEM[1:0] == 2'b00);
	assign half_we =  op_25_22_MEM[0];
	assign word_we =  op_25_22_MEM[1];
	assign signed_we = ~op_25_22_MEM[3];

	wire [3:0] h_we_sel;
	wire [3:0] b_we_sel;
	decoder_2_4 dec_2_4_load_store(
		.in  (alu_result_MEM[1:0]),
		.out (b_we_sel)
	);
	assign h_we_sel = {{2{b_we_sel[2]}}, {2{b_we_sel[0]}}};

	wire [`WIDTH - 1 : 0] data_w_dataram;
	assign mask_dataram = {4{word_we}}
						| {4{half_we}} & h_we_sel
						| {4{byte_we}} & b_we_sel
	;
	assign data_w_dataram = {`WIDTH{byte_we}} & {4{rkd_value_MEM[`BYTE - 1 : 0]}}
						  | {`WIDTH{half_we}} & {2{rkd_value_MEM[`HALF - 1 : 0]}}
						  | {`WIDTH{word_we}} &    rkd_value_MEM
	;

	wire [`RADDR - 1 : 0] data_we;
	assign data_we         = {4{mem_we_MEM && pipe_valid_MEM}};
	assign data_sram_en    = 1'b1;	// data memory enable
	assign data_sram_we    = data_we & mask_dataram;	// data memory byte-writes (aligned)
	assign data_sram_addr  = {alu_result_MEM[`WIDTH - 1 :2], 2'b00};	// data memory address
	assign data_sram_wdata = data_w_dataram;	// data memory write data



endmodule