`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000

module stage_WB (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_MEM, // allow WB stage to accept new instruction
	input  wire                  pipe_valid_WB, // WB stage is valid
	input  wire [`WIDTH - 1 : 0] data_sram_rdata,

	// input from MEM
	input  wire [`WIDTH - 1 : 0] pc_MEM, alu_result_MEM, 
	input  wire [`DWIDTH - 1: 0] mul_result,   // mul
	input  wire [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM,
	input  wire [         3 : 0] mask_dataram,
	input  wire                  byte_we, half_we, word_we, signed_we,
	input  wire                  ld_inst_MEM,
	input  wire					 mul_inst_MEM, // mul
	input  wire  [		  2 : 0] mul_op_MEM, // mul
	input  wire                  res_from_mem_MEM, gr_we_MEM,

	// output
	output reg                   ld_inst_WB,
	output reg					 mul_inst_WB, // mul - to make mul treat as ld in forwarding
	output reg                   gr_we_WB,

	output wire                  rf_we,			// register file write enable
	output wire [`RADDR - 1 : 0] rf_waddr,		// register file write address
	output reg  [`RADDR - 1 : 0] dest_WB,			// destination register number
	output wire [`WIDTH - 1 : 0] rf_wdata,		// register file write data
	output reg  [`WIDTH - 1 : 0] calc_result_WB,	// ALU result
	output wire [`WIDTH - 1 : 0] mem_mul_result, 

	output wire [`WIDTH - 1 : 0] debug_wb_pc,			// debug info
	output wire [         3 : 0] debug_wb_rf_we,		// debug info
	output wire [`RADDR - 1 : 0] debug_wb_rf_wnum,	// debug info
	output wire [`WIDTH - 1 : 0] debug_wb_rf_wdata	// debug info
);

	/* --------------------------------------
		MEM -> WB
	-------------------------------------- */

	reg [`WIDTH - 1 : 0] pc_WB;
	// reg [`WIDTH - 1 : 0] alu_result_WB;
	// reg [`WIDTH - 1 : 0] mem_result_WB;

	// reg [4:0] rf_raddr1_WB;
	// reg [4:0] rf_raddr2_WB;
	// reg [4:0] dest_WB;

	reg [3:0] mask_dataram_WB;
	reg byte_we_WB;
	reg half_we_WB;
	reg word_we_WB;
	reg signed_we_WB;

	// reg ld_inst_WB;	// load
	reg res_from_mem_WB;
	// reg gr_we_WB;

	always @(posedge clk) begin
		if (reset) begin
			pc_WB         <= `WIDTH'h0;
			// alu_result_WB <= `WIDTH'h0;
			// mem_result_WB <= `WIDTH'h0;

			res_from_mem_WB <= 1'b0;
			gr_we_WB        <= 1'b0;
			dest_WB         <= `RADDR'h0;
		end
		else if (pipe_tonext_valid_MEM) begin
			pc_WB         <= pc_MEM;
			calc_result_WB <= mul_inst_MEM ? 
							( {`WIDTH{mul_op_MEM[1] | mul_op_MEM[2]}} & mul_result[`DWIDTH - 1 : `WIDTH]
							| {`WIDTH{mul_op_MEM[0]                }} & mul_result[`WIDTH  - 1 : 0     ] )
							: alu_result_MEM;
			// mem_result_WB <= mem_result;
			
			// rf_raddr1_WB <= rf_raddr1_MEM;
			// rf_raddr2_WB <= rf_raddr2_MEM;
			dest_WB         <= dest_MEM;

			mask_dataram_WB <= mask_dataram;
			byte_we_WB      <= byte_we;
			half_we_WB      <= half_we;
			word_we_WB      <= word_we;
			signed_we_WB    <= signed_we;
		
			ld_inst_WB  <= ld_inst_MEM;
			mul_inst_WB <= mul_inst_MEM; // mul

			res_from_mem_WB <= res_from_mem_MEM;
			gr_we_WB        <= gr_we_MEM;
		end
	end

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
	assign byte_rd_rs = {`BYTE{mask_dataram_WB[0]}} & data_sram_rdata[`BYTE * 0 +: `BYTE]
					  | {`BYTE{mask_dataram_WB[1]}} & data_sram_rdata[`BYTE * 1 +: `BYTE]
					  | {`BYTE{mask_dataram_WB[2]}} & data_sram_rdata[`BYTE * 2 +: `BYTE]
					  | {`BYTE{mask_dataram_WB[3]}} & data_sram_rdata[`BYTE * 3 +: `BYTE]
	;
	// assign byte_rd_rs = {`BYTE{mask_dataram_WB[0]}} & data_sram_rdata[ 7: 0]
	// 				  | {`BYTE{mask_dataram_WB[1]}} & data_sram_rdata[15: 8]
	// 				  | {`BYTE{mask_dataram_WB[2]}} & data_sram_rdata[23:16]
	// 				  | {`BYTE{mask_dataram_WB[3]}} & data_sram_rdata[`WIDTH - 1 : 24]
	// ;
	assign half_rd_rs = {`HALF{mask_dataram_WB[0]}} & data_sram_rdata[`HALF * 0 +: `HALF]
					  | {`HALF{mask_dataram_WB[2]}} & data_sram_rdata[`HALF * 1 +: `HALF]
	;

	assign data_r_dataram = {`WIDTH{byte_we_WB}} & {{`WORD - `BYTE{byte_rd_rs[`BYTE - 1] & signed_we_WB}}, byte_rd_rs}
						  | {`WIDTH{half_we_WB}} & {{`WORD - `HALF{half_rd_rs[`HALF - 1] & signed_we_WB}}, half_rd_rs}
						  | {`WIDTH{word_we_WB}} &                                                    data_sram_rdata
	;

	assign mem_mul_result = mul_inst_WB ? calc_result_WB : data_r_dataram;  // for ld or mul forwarding
	// mul_inst_WB and res_from_mem_WB can't be all 1 at the same time

	// the final result that will be written to register file
	assign final_result = res_from_mem_WB ? mem_mul_result : calc_result_WB;



	assign rf_we    = gr_we_WB && pipe_valid_WB;
	assign rf_waddr = dest_WB;
	assign rf_wdata = final_result;

	// debug info generate
	assign debug_wb_pc       = pc_WB;
	assign debug_wb_rf_we    = {4{rf_we}};
	assign debug_wb_rf_wnum  = dest_WB;
	assign debug_wb_rf_wdata = final_result;



endmodule