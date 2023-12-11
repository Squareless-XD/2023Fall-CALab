`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000
`define TRUE	1'b1
`define FALSE	1'b0

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
	input  wire [ 		  2 : 0] mul_op_MEM, // mul
	input  wire 				 rdcn_op_MEM, // rdcn
	input  wire                  res_from_mem_MEM, gpr_we_MEM,
	input  wire [`WIDTH - 1 : 0] ld_res_from_MEM,
	input  wire [         5 : 0] ecode_MEM_m,
	input  wire 				 csr_we_MEM, csr_wmask_en_MEM,
	input  wire [        13 : 0] csr_code_MEM,
	// input  wire [`WIDTH - 1 : 0] csr_rvalue_MEM,
	input  wire                  has_int_MEM,

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
	output wire 				 WB_ex,
	output wire [        31 : 0] WB_vaddr,
	output wire [         5 : 0] ecode_noint, // ecode, without considering interrupt
	output wire [         8 : 0] WB_esubcode,

	output wire 				 ertn_inst_WB,
	output wire					 has_int_WB,
	output wire 				 rdcn_inst_WB
);

	/* --------------------------------------
		MEM -> WB
	-------------------------------------- */

	wire [`WIDTH - 1 : 0] calc_res = mul_inst_MEM 
		? ( {`WIDTH{mul_op_MEM[1] | mul_op_MEM[2]}} & mul_result[`DWIDTH - 1 : `WIDTH]
		  | {`WIDTH{mul_op_MEM[0]                }} & mul_result[`WIDTH  - 1 : 0     ] )
		: alu_div_rdcntv_result_MEM;

	// Reg #(width, reset_val) inst_name (clk, rst, input, output, wen);
	// Reg_norst #(width) inst_name (clk, input, output, wen);

	// wire [`WIDTH - 1 : 0] pc_WB;
	// wire [4:0] dest_WB;
	wire [`WIDTH - 1 : 0] rj_value_WB;
	wire [`WIDTH - 1 : 0] rkd_value_WB;

	wire [3:0] mask_dataram_WB;
	wire byte_we_WB;
	wire half_we_WB;
	wire word_we_WB;
	wire signed_we_WB;
	wire [`WIDTH - 1 : 0] calc_res_WB;	// ALU result(including mul and div)
	// wire ld_inst_WB;	// load
	wire res_from_mem_WB;
	// wire gpr_we_WB;
	wire rdcn_op_WB;
	wire rdcntid_valid;

	wire ppl_men = pipe_tonext_valid_MEM;

	Reg_norst #(`WIDTH	) pc_WB_inst		(clk,	pc_MEM,			pc_WB,			ppl_men);
	Reg_norst #(`WIDTH	) calc_res_WB_inst	(clk,	calc_res,		calc_res_WB,	ppl_men);
	Reg_norst #(`WIDTH	) rj_value_WB_ins	(clk,	rj_value_MEM,	rj_value_WB,	ppl_men);
	Reg_norst #(`WIDTH	) rkd_value_WB_inst	(clk,	rkd_value_MEM,	rkd_value_WB,	ppl_men);
	Reg_norst #(`RADDR	) dest_WB_inst		(clk,	dest_MEM,		dest_WB,		ppl_men);

	Reg_norst #(4		) mask_dataram_WB_inst	(clk,	mask_dataram,	mask_dataram_WB,	ppl_men);
	Reg_norst #(1		) byte_we_WB_inst		(clk,	byte_we_MEM,	byte_we_WB,			ppl_men);
	Reg_norst #(1		) half_we_WB_inst		(clk,	half_we_MEM,	half_we_WB,			ppl_men);
	Reg_norst #(1		) word_we_WB_inst		(clk,	word_we_MEM,	word_we_WB,			ppl_men);
	Reg_norst #(1		) signed_we_WB_inst		(clk,	signed_we_MEM,	signed_we_WB,		ppl_men);

	Reg_norst #(1		) res_from_mem_WB_inst	(clk,	res_from_mem_MEM,	res_from_mem_WB,	ppl_men);
	Reg #(1, 1'b0) gpr_we_WB_inst (clk, reset, gpr_we_MEM, gpr_we_WB, ppl_men);


	wire [ 5:0] ecode_WB;
	wire [13:0] csr_code_WB;
	wire        csr_we_WB;
	wire        csr_wmask_en_WB;
	wire        csr_inst_WB;
	//wire        rdcn_inst_WB;
	//wire		has_int_WB;
	// wire        ertn_inst_WB;
	Reg_norst #(6 ) ecode_WB_inst			(clk, ecode_MEM_m,		ecode_WB,			ppl_men);
	Reg_norst #(14) csr_code_WB_inst		(clk, csr_code_MEM,		csr_code_WB,		ppl_men);
	Reg_norst #(1 ) csr_we_WB_inst			(clk, csr_we_MEM,		csr_we_WB,			ppl_men);
	Reg_norst #(1 ) csr_wmask_en_WB_inst	(clk, csr_wmask_en_MEM,	csr_wmask_en_WB,	ppl_men);
	Reg_norst #(1 ) csr_inst_WB_inst		(clk, csr_inst_MEM,		csr_inst_WB,		ppl_men);
	Reg_norst #(1 ) ertn_inst_WB_inst		(clk, ertn_inst_MEM,	ertn_inst_WB,		ppl_men);
	Reg_norst #(1 ) rdcn_inst_WB_inst		(clk, rdcn_inst_MEM,	rdcn_inst_WB,		ppl_men);
	Reg_norst #(1 ) rdcn_op_WB_inst			(clk, rdcn_op_MEM,		rdcn_op_WB,			ppl_men);
	Reg_norst #(1 ) has_int_WB_inst			(clk, has_int_MEM,		has_int_WB,			ppl_men);

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
	assign final_result = res_from_mem_WB 			                        ? data_r_dataram
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

	//assign hw_int_in   = 

	assign ertn_flush  = ertn_inst_WB && (!ecode_WB && !has_int_WB) && pipe_valid_WB; 
	assign WB_ex       = ((|ecode_WB) || has_int_WB) && pipe_valid_WB; // if any exception exists 
	// assign WB_pc       = pc_WB;
	assign WB_vaddr    = calc_res_WB;
	assign ecode_noint = ecode_WB & {6{!has_int_WB}}; 
	assign WB_esubcode = 8'b0; //temporarily assigned as zero. only ADEM can set it to 8'b1

endmodule