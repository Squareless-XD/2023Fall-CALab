`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000
`define TRUE	1'b1
`define FALSE	1'b0

module stage_EX (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_ID, // allow EX stage to accept new instruction
	input  wire                  pipe_valid_EX, // EX stage is valid
	input  wire [`DWIDTH - 1: 0] tick, // tick signal, for rdcn instruction

	// input from ID
	input  wire [        12 : 0] alu_op,
	input  wire [`WIDTH - 1 : 0] rj_value, rkd_value, pc_ID, imm,
	input  wire [         3 : 0] op_25_22,
	input  wire [         6 : 0] op_21_15,
	input  wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest,
	input  wire 				 byte_we, half_we, word_we, signed_we,
	input  wire                  ld_inst, st_inst, div_inst, mul_inst, csr_inst, ertn_inst, rdcn_inst,
	input  wire                  br_taken_sure, br_taken_yes, br_taken_no,
	input  wire                  src1_is_pc, src2_is_imm, src2_is_4, br_src_sel,
	input  wire                  res_from_mem, gpr_we, mem_we,
	input  wire [         5 : 0] ecode_ID_m,
	input  wire [         2 : 0] rdcn_op,
	input  wire 				 csr_we, csr_wmask_en,
	input  wire                  has_int_ID,

	// input from MEM, WB
	input  wire					 MEM_ex_ertn, br_from_WB,

	// output to MEM
	output wire [`WIDTH - 1 : 0] pc_EX, rj_value_EX, rkd_value_EX,
	output wire [`WIDTH - 1 : 0] alu_div_rdcntv_result, 
	output wire [`DWIDTH - 1: 0] mul_result,				// mul - to WB
	output wire [         3 : 0] op_25_22_EX,
	output wire [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX,
	output wire 				 byte_we_EX, half_we_EX, word_we_EX, signed_we_EX,
	output wire                  ld_inst_EX, st_inst_EX, csr_inst_EX, ertn_inst_EX, rdcn_inst_EX,
	output wire                  mul_inst_EX,               // mul
	output wire [         2 : 0] mul_op_EX,                 // mul
	output wire                  res_from_mem_EX, gpr_we_EX, mem_we_EX,
	output wire                  has_int_EX,
	output wire					 EX_ex_ertn,	// if any exception exists

	// output
	output wire [`WIDTH - 1 : 0] br_target, // next PC value, branch target
	output wire                  br_taken, calc_done,   // calculation ready, for mul/div instructions
	output wire [         5 : 0] ecode_EX_m,
	output wire [         2 : 0] rdcn_op_EX,

	output wire 				 csr_we_EX, csr_wmask_en_EX,
	output wire [        13 : 0] csr_code_EX
);

	wire first_EX; // first cycle in EX stage

	// branch target (base + offset) calculation (use only one adder)
	wire [`WIDTH - 1 : 0] br_off_src1, br_off_src2;

	wire                  src2_is_rkd;
	wire [`WIDTH - 1 : 0] alu_src1, alu_src2; // ALU sources
	wire [`WIDTH - 1 : 0] alu_result_part;

	// multiplier
	wire [         2 : 0] mul_op;
	wire [`WIDTH - 1 : 0] mul_src1;
	wire [`WIDTH - 1 : 0] mul_src2;
	// wire [`DWIDTH - 1 : 0] mul_result;
	// divider
	wire [         3 : 0] div_op;
	wire [`WIDTH - 1 : 0] div_src1;
	wire [`WIDTH - 1 : 0] div_src2;
	wire [`WIDTH - 1 : 0] div_result;
	wire				  div_en;
	wire [`WIDTH - 1 : 0] div_q;     		
	wire [`WIDTH - 1 : 0] div_r;
	wire                  div_complete;

	wire                  use_alu_inst;
	// wire [`WIDTH - 1 : 0] alu_result;
	// wire                  br_taken, calc_done;



	/* --------------------------------------
		ID -> EX
	-------------------------------------- */

	// Reg #(width, reset_val) inst_name (clk, rst, input, output, wen);
	// Reg_norst #(width) inst_name (clk, input, output, wen);

	wire [12:0] alu_op_EX;
	// wire [`WIDTH - 1 : 0] rj_value_EX;
	// wire [`WIDTH - 1 : 0] rkd_value_EX;
	// wire [`WIDTH - 1 : 0] pc_EX;
	wire [`WIDTH - 1 : 0] imm_EX;

	// wire [3:0] op_25_22_EX;
	wire [6:0] op_21_15_EX;
	// wire [4:0] rf_raddr1_EX;
	// wire [4:0] rf_raddr2_EX;
	// wire [4:0] dest_EX;

	// wire ld_inst_EX;		// load
	// wire st_inst_EX;		// store
	wire div_inst_EX;	// divide
	// wire mul_inst_EX;	// multiply

	wire br_taken_sure_EX, br_taken_yes_EX, br_taken_no_EX;

	wire src1_is_pc_EX, src2_is_imm_EX, src2_is_4_EX, br_src_sel_EX;
	// wire res_from_mem_EX;
	// wire gpr_we_EX;
	// wire mem_we_EX;

	wire ppl_men = pipe_tonext_valid_ID;

	Reg_norst #(13		) alu_op_EX_inst	(clk, alu_op,		alu_op_EX,		ppl_men);
	Reg_norst #(`WIDTH	) rj_value_EX_ins	(clk, rj_value,		rj_value_EX,	ppl_men);
	Reg_norst #(`WIDTH	) rkd_value_EX_ins	(clk, rkd_value,	rkd_value_EX,	ppl_men);
	Reg_norst #(`WIDTH	) pc_EX_inst		(clk, pc_ID,		pc_EX,			ppl_men);
	Reg_norst #(`WIDTH	) imm_EX_inst		(clk, imm,			imm_EX,			ppl_men);

	Reg_norst #(4		) op_25_22_EX_inst	(clk, op_25_22,		op_25_22_EX,	ppl_men);
	Reg_norst #(7		) op_21_15_EX_inst	(clk, op_21_15,		op_21_15_EX,	ppl_men);
	Reg_norst #(`RADDR	) rf_raddr1_EX_inst	(clk, rf_raddr1,	rf_raddr1_EX,	ppl_men);
	Reg_norst #(`RADDR	) rf_raddr2_EX_inst	(clk, rf_raddr2,	rf_raddr2_EX,	ppl_men);
	Reg_norst #(`RADDR	) dest_EX_inst		(clk, dest,			dest_EX,		ppl_men);

	Reg_norst #(1		) byte_we_EX_inst	(clk, byte_we,		byte_we_EX,		ppl_men);
	Reg_norst #(1		) half_we_EX_inst	(clk, half_we,		half_we_EX,		ppl_men);
	Reg_norst #(1		) word_we_EX_inst	(clk, word_we,		word_we_EX,		ppl_men);
	Reg_norst #(1		) signed_we_EX_inst	(clk, signed_we,	signed_we_EX,	ppl_men);

	Reg_norst #(1		) ld_inst_EX_inst		(clk, ld_inst,		ld_inst_EX,			ppl_men);
	Reg_norst #(1		) st_inst_EX_inst		(clk, st_inst,		st_inst_EX,			ppl_men);
	Reg_norst #(1		) div_inst_EX_inst		(clk, div_inst,		div_inst_EX,		ppl_men);
	Reg_norst #(1		) mul_inst_EX_inst		(clk, mul_inst,		mul_inst_EX,		ppl_men);
	Reg_norst #(1		) ertn_inst_EX_inst		(clk, ertn_inst,	ertn_inst_EX,		ppl_men);
	Reg_norst #(1		) rdcn_inst_EX_inst		(clk, rdcn_inst,	rdcn_inst_EX,		ppl_men);

	Reg_norst #(1		) br_taken_sure_EX_inst	(clk, br_taken_sure,	br_taken_sure_EX,	ppl_men);
	Reg_norst #(1		) br_taken_yes_EX_inst	(clk, br_taken_yes,		br_taken_yes_EX,	ppl_men);
	Reg_norst #(1		) br_taken_no_EX_inst	(clk, br_taken_no,		br_taken_no_EX,		ppl_men);

	Reg_norst #(1		) src1_is_pc_EX_inst	(clk, src1_is_pc,	src1_is_pc_EX,	ppl_men);
	Reg_norst #(1		) src2_is_imm_EX_inst	(clk, src2_is_imm,	src2_is_imm_EX,	ppl_men);
	Reg_norst #(1		) src2_is_4_EX_inst		(clk, src2_is_4,	src2_is_4_EX,	ppl_men);
	Reg_norst #(1		) br_src_sel_EX_inst	(clk, br_src_sel,	br_src_sel_EX,	ppl_men);

	Reg_norst #(1		) res_from_mem_EX_inst	(clk, res_from_mem,	res_from_mem_EX,	ppl_men);
	Reg_norst #(1		) gpr_we_EX_inst		(clk, gpr_we,		gpr_we_EX,			ppl_men);
	Reg_norst #(1		) mem_we_EX_inst		(clk, mem_we,		mem_we_EX,			ppl_men);
	


	wire [5:0] ecode_EX;
	// wire [13:0] csr_code_EX;
	Reg_norst #(6 ) ecode_EX_ins		(clk, ecode_ID_m,	ecode_EX,			ppl_men);
	Reg_norst #(1 ) csr_we_EX_ins		(clk, csr_we,		csr_we_EX,			ppl_men);
	Reg_norst #(1 ) csr_wmask_en_EX_ins	(clk, csr_wmask_en,	csr_wmask_en_EX,	ppl_men);
	Reg_norst #(1 ) csr_inst_EX_ins		(clk, csr_inst | (rdcn_op[2] & rdcn_inst),		csr_inst_EX,		ppl_men);
	Reg_norst #(3 ) rdcn_op_EX_ins      (clk, rdcn_op,		rdcn_op_EX,			ppl_men);
	Reg_norst #(1 ) has_int_EX_inst		(clk, has_int_ID,	has_int_EX,			ppl_men);
	/* --------------------------------------
		EX stage
	-------------------------------------- */

	Reg #(1, 1'b1) first_EX_inst (clk, reset || pipe_tonext_valid_ID, 1'b0, first_EX, `TRUE);

	// whether branch is taken
	assign br_taken = pipe_valid_EX && first_EX && (
		br_taken_sure_EX                ||
		br_taken_yes_EX  &&  alu_div_rdcntv_result ||
		br_taken_no_EX   && !alu_div_rdcntv_result
	) && !EX_ex_ertn && !MEM_ex_ertn && !br_from_WB;
	// Don't send branch signal if exception or ertn

	// wire [`WIDTH - 1 : 0] br_target; // branch target, PC+4 or branch target
	// branch target addition sources, and branch target
	assign br_off_src1 = br_src_sel_EX ? rj_value_EX : pc_EX;
	assign br_off_src2 = imm_EX;
	assign br_target = br_off_src1 + br_off_src2;

	// ALU sources
	assign src2_is_rkd = ~src2_is_imm_EX & ~src2_is_4_EX;
	assign alu_src1 = src1_is_pc_EX ? pc_EX : rj_value_EX;
	assign alu_src2 = {`WIDTH{src2_is_imm_EX}} & imm_EX 
					| {`WIDTH{src2_is_rkd   }} & rkd_value_EX
					| {`WIDTH{src2_is_4_EX  }} & `WIDTH'h4
	;

	// ALU instance
	alu u_alu (
		.alu_op     (alu_op_EX ),
		.alu_src1   (alu_src1  ),
		.alu_src2   (alu_src2  ),
		.alu_result (alu_result_part) // modified
	);

	// multiplier control
	assign mul_op   = {
		op_21_15_EX[1:0] == 2'b10,
		op_21_15_EX[1:0] == 2'b01,
		op_21_15_EX[1:0] == 2'b00
	};
	assign mul_src1 = rj_value_EX;
	assign mul_src2 = rkd_value_EX;
	assign mul_op_EX = mul_op;

	// multiplier
	mul u_muler (
		.mul_clk	(clk       ),
		.reset    	(reset     ),
		.x   		(mul_src1  ),
		.y   	    (mul_src2  ),
		.mul_signed (mul_op[0] | mul_op[1]),
		.result     (mul_result)
	);

	// divider control
	assign div_op = {
		op_21_15_EX[1:0] == 2'b11,
		op_21_15_EX[1:0] == 2'b10,
		op_21_15_EX[1:0] == 2'b01,
		op_21_15_EX[1:0] == 2'b00
	};
	assign div_src1 = rj_value_EX;
	assign div_src2 = rkd_value_EX;
	assign div_en = div_inst_EX && first_EX && !EX_ex_ertn;


	// divider
	div u_diver (
		.div_clk    (clk    		        ),
		.reset      (reset || !pipe_valid_EX), /* NOTE: MAYBE IT SHOULD BE MODIFIED. */
		.div		(div_en 		        ), // enable/begin signal
		.div_signed (div_op[0] | div_op[1]  ),
		.x  	    (div_src1  		        ), // dividend
		.y          (div_src2  		        ), // divisor
		.s          (div_q     		        ), // quotient
		.r          (div_r     		        ), // remainder
		.complete   (div_complete	        )  // complete signal
	);

	assign div_result = {`WIDTH{div_op[0] | div_op[2]}} & div_q
					  | {`WIDTH{div_op[1] | div_op[3]}} & div_r
	;

	assign use_alu_inst = ~mul_inst_EX & ~div_inst_EX & ~rdcn_inst_EX;

	// only alive in EX stage
	// ***** alu_result NAME SHOULD BE MODIFIED
	assign alu_div_rdcntv_result = {`WIDTH{div_inst_EX 		           }} & div_result
					  		     | {`WIDTH{rdcn_inst_EX & rdcn_op_EX[1]}} & tick[`WIDTH  - 1 : 0     ]
					  		     | {`WIDTH{rdcn_inst_EX & rdcn_op_EX[0]}} & tick[`DWIDTH - 1 : `WIDTH]
					  		     | {`WIDTH{use_alu_inst                }} & alu_result_part
	;

	// calculate ready
	assign calc_done = div_inst_EX && (div_complete && !div_en) ||
		// calc done when div complete and no new div coming
		!div_inst_EX || EX_ex_ertn;
		// Don't waste time doing div if trap(interrupt or exception) is marked
	/*
	 * Note that when mul inst which needs 2 cycles comes,
	 * calc_done means that the first cycle is done. And the
	 * second cycle will be done at the same time of passing
	 * MEM stage.
	 */


	// get part of immediate
	// control and status register (CSR) index/addr
	assign csr_code_EX = imm_EX[15:2];

	// ALE, Address Not ALigned Exception
	wire addr_naligned = (mem_we_EX || res_from_mem_EX) && pipe_valid_EX && (
		(word_we_EX && alu_div_rdcntv_result[1:0] != 2'b00) ||
		(half_we_EX && alu_div_rdcntv_result[  0] != 1'b0 )
	);

	// wire [5:0] ecode_ID_m; // modified exception code
	assign ecode_EX_m = (ecode_EX != 6'b0) ? ecode_EX
					  : addr_naligned      ? 6'h9
					  :                      6'b0
	;
	assign EX_ex_ertn = pipe_valid_EX &&
		(ecode_EX != 6'd0 || has_int_EX || ertn_inst_EX);
endmodule