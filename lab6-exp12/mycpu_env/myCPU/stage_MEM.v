`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000
`define TRUE	1'b1
`define FALSE	1'b0

module stage_MEM (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_EX, // allow MEM stage to accept new instruction
	input  wire                  pipe_valid_MEM, // MEM stage is valid

	// input from EX
	input  wire [`WIDTH - 1 : 0] pc_EX, alu_div_rdcntv_result, rj_value_EX, rkd_value_EX,
	input  wire [         3 : 0] op_25_22_EX,
	input  wire [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX,
	input  wire 				 byte_we_EX, half_we_EX, word_we_EX, signed_we_EX,
	input  wire                  ld_inst_EX, st_inst_EX, csr_inst_EX, ertn_inst_EX, rdcn_inst_EX,
	input  wire					 mul_inst_EX, // mul
	input  wire [		  2 : 0] mul_op_EX, // mul
	input  wire [		  2 : 0] rdcn_op_EX, // rdcn
	input  wire                  res_from_mem_EX, gpr_we_EX, mem_we_EX,
	input  wire [         5 : 0] ecode_EX_m,
	input  wire 				 csr_we_EX, csr_wmask_en_EX,
	input  wire [        13 : 0] csr_code,
	input  wire 				 br_from_WB, // branch valid signal, at WB stage, calculated in cpu_top
	input  wire                  has_int_EX,              

	// output to WB
	output wire [`WIDTH - 1 : 0] pc_MEM, alu_div_rdcntv_result_MEM, rj_value_MEM, rkd_value_MEM,
	output wire [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM,
	output wire [         3 : 0] mask_dataram,
	output wire 				 byte_we_MEM, half_we_MEM, word_we_MEM, signed_we_MEM,
	output wire 				 ld_inst_MEM, csr_inst_MEM, ertn_inst_MEM, rdcn_inst_MEM,
	output wire					 mul_inst_MEM, // mul
	output wire [		  2 : 0] mul_op_MEM, // mul
	output wire 				 rdcn_op_MEM,  // only rdcn_op_EX[2] is useful at WB
	output wire 				 res_from_mem_MEM, gpr_we_MEM,
	output wire                  has_int_MEM,

	// output
	output wire                  data_sram_en,	// data SRAM port enable pin
	output wire [         3 : 0] data_sram_we,	// data SRAM byte-writes
	output wire [`WIDTH - 1 : 0] data_sram_addr, data_sram_wdata,
	output wire [         5 : 0] ecode_MEM_m,
	output wire 				 csr_we_MEM, csr_wmask_en_MEM,
	output wire [        13 : 0] csr_code_MEM
);

	/* --------------------------------------
		EX -> MEM
	-------------------------------------- */

	// Reg #(width, reset_val) inst_name (clk, rst, input, output, wen);
	// Reg_norst #(width) inst_name (clk, input, output, wen);

	// wire [`WIDTH - 1 : 0] pc_MEM;
	// wire [`WIDTH - 1 : 0] alu_result_MEM;
	// wire [`WIDTH - 1 : 0] rkd_value_MEM;

	wire [3:0] op_25_22_MEM;
	// wire [`RADDR - 1 : 0] rf_raddr1_MEM;
	// wire [`RADDR - 1 : 0] rf_raddr2_MEM;
	// wire [`RADDR - 1 : 0] dest_MEM;

	// wire ld_inst_MEM;
	wire st_inst_MEM;

	// wire mul_inst_EX, // mul
	// wire [2:0] mul_op_EX, // mul

	// wire res_from_mem_MEM;
	// wire gpr_we_MEM;
	wire mem_we_MEM;

	wire ppl_men = pipe_tonext_valid_EX;

	Reg_norst #(`WIDTH	) pc_MEM_inst			(clk, pc_EX,		pc_MEM,			ppl_men);
	Reg_norst #(`WIDTH	) alu_div_rdcntv_result_MEM_inst	(clk, alu_div_rdcntv_result,	alu_div_rdcntv_result_MEM,	ppl_men);
	Reg_norst #(`WIDTH	) rj_value_MEM_ins		(clk, rj_value_EX,	rj_value_MEM,	ppl_men);
	Reg_norst #(`WIDTH	) rkd_value_MEM_inst	(clk, rkd_value_EX,	rkd_value_MEM,	ppl_men);

	Reg_norst #(4		) op_25_22_MEM_inst		(clk, op_25_22_EX,	op_25_22_MEM,	ppl_men);
	Reg_norst #(`RADDR	) rf_raddr1_MEM_inst	(clk, rf_raddr1_EX,	rf_raddr1_MEM,	ppl_men);
	Reg_norst #(`RADDR	) rf_raddr2_MEM_inst	(clk, rf_raddr2_EX,	rf_raddr2_MEM,	ppl_men);
	Reg_norst #(`RADDR	) dest_MEM_inst			(clk, dest_EX,		dest_MEM,		ppl_men);

	Reg_norst #(1		) byte_we_MEM_inst		(clk, byte_we_EX,	byte_we_MEM,	ppl_men);
	Reg_norst #(1		) half_we_MEM_inst		(clk, half_we_EX,	half_we_MEM,	ppl_men);
	Reg_norst #(1		) word_we_MEM_inst		(clk, word_we_EX,	word_we_MEM,	ppl_men);
	Reg_norst #(1		) signed_we_MEM_inst	(clk, signed_we_EX,	signed_we_MEM,	ppl_men);

	Reg_norst #(1		) ld_inst_MEM_inst		(clk, ld_inst_EX,		ld_inst_MEM,		ppl_men);
	Reg_norst #(1		) st_inst_MEM_inst		(clk, st_inst_EX,		st_inst_MEM,		ppl_men);
	Reg_norst #(1		) ertn_inst_MEM_inst	(clk, ertn_inst_EX,		ertn_inst_MEM,		ppl_men);
	Reg_norst #(1		) rdcn_inst_MEM_inst	(clk, rdcn_inst_EX,		rdcn_inst_MEM,		ppl_men);

	Reg_norst #(1		) mul_inst_MEM_ins		(clk, mul_inst_EX,		mul_inst_MEM,	ppl_men);
	Reg_norst #(3		) mul_op_MEM_inst		(clk, mul_op_EX,		mul_op_MEM,		ppl_men);
	Reg_norst #(1		) rdcn_op_MEM_inst		(clk, rdcn_op_EX[2],	rdcn_op_MEM,	ppl_men);

	Reg_norst #(1		) res_from_mem_MEM_inst	(clk, res_from_mem_EX,	res_from_mem_MEM,	ppl_men);
	Reg_norst #(1		) gpr_we_MEM_inst		(clk, gpr_we_EX,		gpr_we_MEM,			ppl_men);
	Reg_norst #(1		) mem_we_MEM_inst		(clk, mem_we_EX,		mem_we_MEM,			ppl_men);


	wire [5:0] ecode_MEM;
	Reg_norst #(6 ) ecode_MEM_ins			(clk, ecode_EX_m,		ecode_MEM,			ppl_men);
	Reg_norst #(14) csr_code_MEM_ins 		(clk, csr_code,			csr_code_MEM,		ppl_men);
	Reg_norst #(1 ) csr_we_MEM_ins			(clk, csr_we_EX,		csr_we_MEM,			ppl_men);
	Reg_norst #(1 ) csr_wmask_en_MEM_ins	(clk, csr_wmask_en_EX,	csr_wmask_en_MEM,	ppl_men);
	Reg_norst #(1 ) csr_inst_MEM_ins		(clk, csr_inst_EX /*| (rdcn_op_EX[2] & rdcn_inst_EX)*/,	csr_inst_MEM, ppl_men);  // forward rdcntid modify
	Reg_norst #(1 ) has_int_MEM_inst		(clk, has_int_EX,		has_int_MEM,		ppl_men);
	// Reg_norst #(32) csr_rvalue_MEM_ins		(clk, csr_rvalue,		csr_rvalue_MEM,		ppl_men);

	/* --------------------------------------
		MEM stage
	-------------------------------------- */

	// data memory write "width" selection

	// half/byte write enable selection
	wire [3:0] h_we_sel;
	wire [3:0] b_we_sel;
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

	// data memory read control
	// wire [`RADDR - 1 : 0] data_we;
	// assign data_we         = ;
	assign data_sram_en    = pipe_valid_MEM && !ecode_MEM && !br_from_WB;	// data memory enable /* NOTE: SHOULD BE MODIFIED */
	assign data_sram_we    = {4{mem_we_MEM}} & mask_dataram;	// data memory byte-writes (aligned)
	assign data_sram_addr  = {alu_div_rdcntv_result_MEM[`WIDTH - 1 :2], 2'b00};	// data memory address
	assign data_sram_wdata = data_w_dataram;	// data memory write data

	// wire [5:0] ecode_MEM_m; // modified exception code
	assign ecode_MEM_m = ecode_MEM;


endmodule