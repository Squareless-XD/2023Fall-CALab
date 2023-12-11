`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000
`define TRUE	1'b1
`define FALSE	1'b0

`define ADD  0
`define SUB  1
`define SLT  2
`define SLTU 3
`define AND  4
`define NOR  5
`define OR   6
`define XOR  7
`define SLL  8
`define SRL  9
`define SRA 10
`define LUI 11
`define EQ  12

module stage_ID (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_IF, pipe_tonext_valid_ID, pipe_valid_ID,
	input  wire                  hzd_alu_EX_r1, hzd_alu_MEM_r1, hzd_WB_r1,
	input  wire                  hzd_alu_EX_r2, hzd_alu_MEM_r2, hzd_WB_r2,
	input  wire [`WIDTH - 1 : 0] alu_div_rdcntv_result, alu_div_rdcntv_result_MEM, rf_wdata_WB,
	input  wire                  has_int,
	input  wire					 EX_MEM_WB_ex_ertn, br_from_WB,
	/*
	 * If an instruction at EX or MEM is ertn or marked exception,
	 * mark the instruction at ID as interrupt to wipe its effects.
	 */

	// input from IF stage
	input  wire [`WIDTH - 1 : 0] inst_final,
	input  wire [`WIDTH - 1 : 0] pc,
	input  wire [         5 : 0] ecode,

	// input from WB stage
	input  wire                  rf_we,		// register file write enable
	input  wire [`RADDR - 1 : 0] rf_waddr,	// register file write address
	input  wire [`WIDTH - 1 : 0] rf_wdata,	// register file write data

	// output to EX
	output wire [        12 : 0] alu_op,
	output wire [`WIDTH - 1 : 0] rj_value, rkd_value, imm,
	output wire [`WIDTH - 1 : 0] pc_ID,
	output wire [         3 : 0] op_25_22,
	output wire [         6 : 0] op_21_15,
	output wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest,
	output wire 				 byte_we, half_we, word_we, signed_we,
	output wire                  ld_inst, st_inst, div_inst, mul_inst, ertn_inst, csr_inst, rdcn_inst,
	output wire                  br_taken_sure, br_taken_yes, br_taken_no,
	output wire					 inst_tlbrd, inst_tlbfill, inst_tlbsrch, inst_tlbwr, inst_invtlb,
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
	// declaration
	wire illegal_inst;


	/* --------------------------------------
		IF -> ID
	-------------------------------------- */

	// Reg #(width, reset_val) inst_name (clk, rst, input, output, wen);
	// Reg_norst #(width) inst_name (clk, input, output, wen);

	wire [`WIDTH - 1 : 0] inst_ID;
	// wire [`WIDTH - 1 : 0] pc_ID;
	reg flag_refresh;

	// if you have a better name, modification is recommended
	// pipeline move enable
	wire ppl_men = pipe_tonext_valid_IF;

	Reg_norst #(`WIDTH) inst_ID_inst	(clk, inst_final,	inst_ID,	ppl_men);
	Reg_norst #(`WIDTH) pc_ID_ins		(clk, pc,			pc_ID,		ppl_men);


	wire [5:0] ecode_ID;
	Reg_norst #(6) ecode_ID_ins		(clk, ecode, 	ecode_ID, 	ppl_men);
	//Reg_norst #(1) has_int_ID_ins 	(clk, has_int,	has_int_ID_truely, ppl_men);

	/* See the commends near `flag_refresh` for details */
	assign has_int_ID = pipe_valid_ID && (has_int || EX_MEM_WB_ex_ertn || flag_refresh);
	assign needs_refresh_ID = pipe_valid_ID && flag_refresh;

	/* --------------------------------------
		ID stage
	-------------------------------------- */


	// cut the instruction into several parts
	wire [ 5:0] op_31_26;
	// wire [ 3:0] op_25_22;
	// wire [ 6:0] op_21_15;
	assign op_31_26 = inst_ID[31:26];
	assign op_25_22 = inst_ID[25:22];
	assign op_21_15 = inst_ID[21:15];

	wire op30_26_1 = (op_31_26[4:0] == 5'b00001);
	wire op30_26_0 = (op_31_26[4:0] == 5'b00000);
	wire op30_22_0 = op30_26_0 && (op_25_22[3:0] == 4'b0000);

	// decode results
	wire [15:0] op_29_26_dec;
	wire [ 7:0] op_24_22_dec;

	decoder_x #(.width(4)) inst_op_29_26_dec (
		.in  (op_31_26[3:0]),
		.out (op_29_26_dec)
	);
	decoder_x #(.width(3)) inst_op_24_22_dec (
		.in  (op_25_22[2:0]),
		.out (op_24_22_dec)
	);

	// register numbers
	wire [`RADDR - 1 : 0] rd;
	wire [`RADDR - 1 : 0] rj;
	wire [`RADDR - 1 : 0] rk;
	assign rd = inst_ID[ 4: 0];
	assign rj = inst_ID[ 9: 5];
	assign rk = inst_ID[14:10];

	// different immediate values
	wire [11:0] imm12;
	wire [15:0] imm16;
	wire [19:0] imm20;
	wire [25:0] imm26;
	assign imm12 = inst_ID[21:10];
	assign imm16 = inst_ID[25:10];
	assign imm20 = inst_ID[24: 5];
	assign imm26 = {inst_ID[ 9: 0], inst_ID[25:10]};

	wire [8:0] csr_code_ID;
	assign csr_code_ID = imm[10:2];

	wire br_inst;		// branch & jump
	// wire ld_inst;		// load
	// wire st_inst;		// store
	wire lu_inst;		// load upper
	wire tlb_inst;		// translation lookaside buffer
	// wire ertn_inst;		// exception return
	wire cache_inst;	// cache operation
	// wire csr_inst;		// control and status register
	wire alui_inst;		// ALU immediate
	wire sfti_inst;		// shifter immediate
	wire syscall_inst;	// system call
	wire break_inst;	// system call
	// wire div_inst;		// divide
	// wire mul_inst;		// multiply
	wire sftr_inst;		// shifter register
	wire alur_inst;		// ALU register
	// wire rdcn_inst;		// read control and status register
	assign br_inst		= op_31_26[4];
	assign ld_inst		= (op_31_26[4:0] == 5'b01010) && !op_25_22[2];
	assign st_inst		= (op_31_26[4:0] == 5'b01010) &&  op_25_22[2];
	assign lu_inst		= (op_31_26[4:2] == 3'b001);
	assign tlb_inst		= ~inst_ID[31] && op30_26_1 && (op_25_22[3:0] == 4'b1001)
						&& op_21_15[6:2] == 5'b00100 &&
						(op_21_15[1] && op_21_15[0] ||	// INVTLB
						~op_21_15[1] && ~op_21_15[0] && (rk[2] ^ rk[1]))
	;
	assign ertn_inst	=  op30_26_1 && (op_25_22[3:0] == 4'b1001)
						&& (op_21_15 == 7'b0010000)
						&& (rk == 5'b01110) && (rj == `R0) && (rd == `R0)
	;
	assign cache_inst	= op30_26_1 && (op_25_22[3:0] == 4'b1000);
	assign csr_inst		= op30_26_1 && (op_25_22[3:2] == 2'b00);
	assign alui_inst	= op30_26_0 && op_25_22[3];
	assign sfti_inst	= op30_26_0 && (op_25_22[3:0] == 4'b0001);
	assign syscall_inst	= op30_22_0 && (op_21_15[6:0] == 7'b1010110);
	assign break_inst	= op30_22_0 && (op_21_15[6:0] == 7'b1010100);
	assign div_inst		= op30_22_0 && (op_21_15[6:2] == 5'b10000);
	assign mul_inst		= op30_22_0 && (op_21_15[6:2] == 5'b01110);
	assign sftr_inst	= op30_22_0 && ((op_21_15[6:0] == 7'b0110000) || (op_21_15[6:1] == 6'b010111));
	assign alur_inst	= op30_22_0 && (op_21_15[6:4] == 3'b010) && !sftr_inst;
	assign rdcn_inst	= op30_22_0 && (op_21_15[6:5] == 2'b00);


	// judge illegal instructions
	wire br_inst_ill		= br_inst && (
		op_29_26_dec[ 0] || op_29_26_dec[ 1] || op_29_26_dec[ 2] || op_29_26_dec[12] |
		op_29_26_dec[13] || op_29_26_dec[14] || op_29_26_dec[15]
	);
	wire ld_inst_ill		= ld_inst && (
			// {op_25_22[3], op_25_22[1:0]} == 3'b000 || // ld.b
			// {op_25_22[3], op_25_22[1:0]} == 3'b001 || // ld.h
			// {op_25_22[3], op_25_22[1:0]} == 3'b010 || // ld.w
		{op_25_22[3], op_25_22[1:0]} == 3'b011 ||
			// {op_25_22[3], op_25_22[1:0]} == 3'b100 || // ld.bu
			// {op_25_22[3], op_25_22[1:0]} == 3'b101    // ld.hu
		{op_25_22[3], op_25_22[1:0]} == 3'b110 ||
		{op_25_22[3], op_25_22[1:0]} == 3'b111
	);
	wire st_inst_ill		= st_inst && (op_25_22[3] || (
			// op_25_22[1:0] == 2'b00 || // sd.b
			// op_25_22[1:0] == 2'b01 || // sd.h
			// op_25_22[1:0] == 2'b10 || // sd.w
		op_25_22[1:0] == 2'b11
	));
	wire lu_inst_ill		= lu_inst && (!op_31_26[0] || op_25_22[3]);

	wire tlb_inst_ill		= tlb_inst && (
		(inst_tlbfill || inst_tlbrd || inst_tlbsrch || inst_tlbwr) && (rj != `R0 || rd != `R0)
	||	inst_invtlb && (rd[4:3] != 2'd0 || rd[2:0] == 3'b111 || rd[2] == 1'd0 && rj != `R0
					||  rd[2:0] != 3'd5 && rd[2:0] != 3'b110 && rk != `R0
					||  rj_value[31:10] != 22'd0)
	);

	// wire ertn_inst_ill		= 1'b0;
	wire cache_inst_ill		= cache_inst && 1'b1;
	// wire csr_inst_ill		= 1'b0;
	wire alui_inst_ill		= alui_inst && (
			// op_24_22_dec[0] || op_24_22_dec[1] || op_24_22_dec[2] || // slti sltiu addi.w
		op_24_22_dec[3] || op_24_22_dec[4]
			// op_24_22_dec[5] || op_24_22_dec[6] || op_24_22_dec[7]    // andi ori xori
	);
	wire sfti_inst_ill		= sfti_inst && ({op_21_15[6:5], op_21_15[2:0]} != 5'b00001 ||
			// op_21_15[4:3] == 2'b00 || // slli.w
			// op_21_15[4:3] == 2'b01 || // srli.w
			// op_21_15[4:3] == 2'b10 || // srai.w
		op_21_15[4:3] == 2'b11
	);
	// wire break_inst_ill	    = 1'b0;
	// wire syscall_inst_ill	= 1'b0;
	// wire div_inst_ill		= 1'b0;
	wire mul_inst_ill		= mul_inst && (
			// op_21_15[1:0] == 2'b00 || // mul.w
			// op_21_15[1:0] == 2'b01 || // mulh.w
			// op_21_15[1:0] == 2'b10 || // mulh.wu
		op_21_15[1:0] == 2'b11
	);
	// wire sftr_inst_ill		= 1'b0;
	wire alur_inst_ill		= alur_inst && (
			// op_21_15[3:0] == 4'b0000 || // add.w
		op_21_15[3:0] == 4'b0001 ||
			// op_21_15[3:0] == 4'b0010 || // sub.w
		op_21_15[3:0] == 4'b0011 ||
			// op_21_15[3:0] == 4'b0100 || // slt
			// op_21_15[3:0] == 4'b0101 || // sltu
		op_21_15[3:0] == 4'b0110 ||
		op_21_15[3:0] == 4'b0111 ||
			// op_21_15[3:0] == 4'b1000 || // nor
			// op_21_15[3:0] == 4'b1001 || // and
			// op_21_15[3:0] == 4'b1010 || // or
			// op_21_15[3:0] == 4'b1011 || // xor
		op_21_15[3:0] == 4'b1100 ||
		op_21_15[3:0] == 4'b1101
			// op_21_15[3:0] == 4'b1110 || // 3r shift: sll.w
			// op_21_15[3:0] == 4'b1111    // 3r shift: srl.2
	);
	wire rdcn_inst_ill		= rdcn_inst && !(op_21_15[4:0] == 5'b0 && rk[4:1] == 4'b1100 &&
											 (rk[0] == 1'b0 && (rj == `R0 || rd == `R0)
										   || rk[0] == 1'b1 && rj == `R0));

	// used for instruction illegality
	assign illegal_inst =  op_31_26[5]
						|| br_inst_ill
						|| ld_inst_ill
						|| st_inst_ill
						|| lu_inst_ill
						|| tlb_inst_ill
						// || ertn_inst_ill
						|| cache_inst_ill
						// || csr_inst_ill
						|| alui_inst_ill
						|| sfti_inst_ill
						// || break_inst_ill
						// || syscall_inst_ill
						// || div_inst_ill
						|| mul_inst_ill
						// || sftr_inst_ill
						|| alur_inst_ill
						|| rdcn_inst_ill
						|| !( br_inst
							|| ld_inst
							|| st_inst
							|| lu_inst
							|| tlb_inst
							|| ertn_inst
							|| cache_inst
							|| csr_inst
							|| alui_inst
							|| sfti_inst
							|| syscall_inst
							|| break_inst
							|| div_inst
							|| mul_inst
							|| sftr_inst
							|| alur_inst
							|| rdcn_inst)
	;

	/*--- decode the instruction ---*/
	// instructions of registers
	wire inst_add_w   = alur_inst && (op_21_15[3:1] == 3'b000);
	wire inst_sub_w   = alur_inst && (op_21_15[3:1] == 3'b001);
	wire inst_slt     = alur_inst && ({op_21_15[2], op_21_15[0]} == 2'b10);
	wire inst_sltu    = alur_inst && ({op_21_15[2], op_21_15[0]} == 2'b11);
	wire inst_nor     = alur_inst && ({op_21_15[3], op_21_15[1:0]} == 3'b100);
	wire inst_and     = alur_inst && ({op_21_15[3], op_21_15[1:0]} == 3'b101);
	wire inst_or      = alur_inst && ({op_21_15[3], op_21_15[1:0]} == 3'b110);
	wire inst_xor     = alur_inst && ({op_21_15[3], op_21_15[1:0]} == 3'b111);
	wire inst_sll_w   = sftr_inst && (op_21_15[1:0] == 2'b10);
	wire inst_srl_w   = sftr_inst && (op_21_15[1:0] == 2'b11);
	wire inst_sra_w   = sftr_inst && (op_21_15[1:0] == 2'b00);

	// immediate operation
	wire inst_slli_w  = sfti_inst && (op_21_15[4:3] == 2'b00);
	wire inst_srli_w  = sfti_inst && (op_21_15[4:3] == 2'b01);
	wire inst_srai_w  = sfti_inst && (op_21_15[4:3] == 2'b10);
	wire inst_slti    = alui_inst &&  op_24_22_dec[0];
	wire inst_sltui   = alui_inst &&  op_24_22_dec[1];
	wire inst_addi_w  = alui_inst &&  op_24_22_dec[2];
	wire inst_andi    = alui_inst &&  op_24_22_dec[5];
	wire inst_ori     = alui_inst &&  op_24_22_dec[6];
	wire inst_xori    = alui_inst &&  op_24_22_dec[7];
	wire inst_lu12i_w   = lu_inst && !op_31_26[1];
	wire inst_pcaddu12i = lu_inst &&  op_31_26[1];

	// branch instructions
	wire inst_jirl = br_inst && op_29_26_dec[ 3];
	wire inst_b    = br_inst && op_29_26_dec[ 4];
	wire inst_bl   = br_inst && op_29_26_dec[ 5];
	wire inst_beq  = br_inst && op_29_26_dec[ 6];
	wire inst_bne  = br_inst && op_29_26_dec[ 7];
	wire inst_blt  = br_inst && op_29_26_dec[ 8];
	wire inst_bge  = br_inst && op_29_26_dec[ 9];
	wire inst_bltu = br_inst && op_29_26_dec[10];
	wire inst_bgeu = br_inst && op_29_26_dec[11];

	assign inst_tlbsrch = tlb_inst && ~op_21_15[0] && rk == 5'b01010;
	assign inst_tlbwr   = tlb_inst && ~op_21_15[0] && rk == 5'b01100;
	assign inst_tlbfill = tlb_inst && ~op_21_15[0] && rk == 5'b01101;
	assign inst_tlbrd   = tlb_inst && ~op_21_15[0] && rk == 5'b01011;
	assign inst_invtlb  = tlb_inst && op_21_15[0];

	// calculate alu operation control signal
	assign alu_op[`ADD ] = inst_add_w || inst_addi_w || ld_inst || st_inst || inst_jirl || inst_bl || inst_pcaddu12i;
	assign alu_op[`SUB ] = inst_sub_w;
	assign alu_op[`SLT ] = inst_slt || inst_slti || inst_blt || inst_bge;
	assign alu_op[`SLTU] = inst_sltu || inst_sltui || inst_bltu || inst_bgeu;
	assign alu_op[`AND ] = inst_and || inst_andi;
	assign alu_op[`NOR ] = inst_nor;
	assign alu_op[`OR  ] = inst_or || inst_ori;
	assign alu_op[`XOR ] = inst_xor || inst_xori;
	assign alu_op[`SLL ] = inst_sll_w || inst_slli_w;
	assign alu_op[`SRL ] = inst_srl_w || inst_srli_w;
	assign alu_op[`SRA ] = inst_sra_w || inst_srai_w;
	assign alu_op[`LUI ] = inst_lu12i_w;
	assign alu_op[`EQ  ] = inst_beq || inst_bne;

	// control signals of immediate's required bits
	wire need_ui5  = inst_slli_w || inst_srli_w || inst_srai_w;
	wire need_si12 = inst_slti || inst_sltui || inst_addi_w || ld_inst || st_inst;
	wire need_ui12 = inst_andi || inst_ori || inst_xori;
	wire need_si16 = br_inst && !(inst_b || inst_bl) || csr_inst; // use part of 16-bits offset
	wire need_si20 = inst_lu12i_w || inst_pcaddu12i;
	wire need_si26 = inst_b || inst_bl;

	// immediate value
	assign imm = {`WIDTH{need_ui5 | need_si12}} & {{20{imm12[11]}}, imm12[11:0]} // ui5 only care last 5 bits
			   | {`WIDTH{need_ui12}}            & { 20'b0         , imm12[11:0]}
			   | {`WIDTH{need_si16}}            & {{14{imm16[15]}}, imm16[15:0],  2'b0}
			   | {`WIDTH{need_si20}}            & {                 imm20[19:0], 12'b0}
			   | {`WIDTH{need_si26}}            & {{ 4{imm26[25]}}, imm26[25:0],  2'b0}
	;

	// control signals of whether the branch should be taken
	// 	inst_jirl_EX                ||
	// 	inst_b_EX                   ||
	// 	inst_bl_EX                  ||
	// 	inst_beq_EX  &&  alu_result ||
	// 	inst_bne_EX  && !alu_result ||
	// 	inst_blt_EX  &&  alu_result ||
	// 	inst_bge_EX  && !alu_result ||
	// 	inst_bltu_EX &&  alu_result ||
	// 	inst_bgeu_EX && !alu_result
	assign br_taken_sure = inst_jirl || inst_b || inst_bl;
	assign br_taken_yes  = inst_beq || inst_blt || inst_bltu;
	assign br_taken_no   = inst_bne || inst_bge || inst_bgeu;

	// control signals of ALU oprand source and destination register
	wire src_reg_is_rd = br_inst && (op_31_26[3] || op_31_26[2] && op_31_26[1]) || st_inst || csr_inst; // source register is rd
	assign src1_is_pc  = inst_jirl || inst_bl || inst_pcaddu12i; // source 1 is pc
	assign src2_is_imm = sfti_inst || alui_inst || ld_inst || st_inst || lu_inst; // source 2 is immediate
	assign src2_is_4   = inst_jirl || inst_bl; // source 2 is 4
	wire dst_is_r1     = inst_bl; // destination register is r1
	wire dst_is_rj     = rdcn_inst && rdcn_op[2]; // destination register is rj
	assign br_src_sel  = inst_jirl; // branch source select

	// control signals of write enable
	assign res_from_mem  = ld_inst; // result from memory
	assign gpr_we        = !tlb_inst && !st_inst && !(br_inst && !inst_jirl && !inst_bl) || csr_inst || rdcn_inst; // general purpose register write enable
	assign mem_we        = st_inst; // memory write enable

	// register file
	wire [`WIDTH - 1 : 0] rf_rdata1;	// register file read data 1
	wire [`WIDTH - 1 : 0] rf_rdata2;	// register file read data 2
	assign rf_raddr1 = rj;
	assign rf_raddr2 = src_reg_is_rd ? rd : rk;

	regfile u_regfile(
		.clk    (clk      ),
		.raddr1 (rf_raddr1),
		.rdata1 (rf_rdata1),
		.raddr2 (rf_raddr2),
		.rdata2 (rf_rdata2),
		.we     (rf_we    ),
		.waddr  (rf_waddr ),
		.wdata  (rf_wdata )
	);

	/*--- only til exp13 ---*/
	// register file read enable: for installing ID stage
	assign rf_ren1 = br_inst && !(inst_b || inst_bl)
				  || ld_inst
				  || st_inst
				  || csr_inst && rj != `R0 && rj != 5'd1
				  || alui_inst
				  || sfti_inst
				  || div_inst
				  || mul_inst
				  || sftr_inst
				  || alur_inst
	;
	assign rf_ren2 = br_inst && !(inst_jirl || inst_b || inst_bl)
				  || st_inst
				  || csr_inst
				  || div_inst
				  || mul_inst
				  || sftr_inst
				  || alur_inst
	;

	// register read data, considering data hazard
	assign rj_value  = hzd_alu_EX_r1  ? alu_div_rdcntv_result     :
					   hzd_alu_MEM_r1 ? alu_div_rdcntv_result_MEM :
					   hzd_WB_r1	  ? rf_wdata_WB  :
					   rf_rdata1
	;
	assign rkd_value = hzd_alu_EX_r2  ? alu_div_rdcntv_result     :
					   hzd_alu_MEM_r2 ? alu_div_rdcntv_result_MEM :
					   hzd_WB_r2	  ? rf_wdata_WB  :
					   rf_rdata2
	;

	// wire [`RADDR - 1 : 0] dest; // destination register number
	assign dest = dst_is_rj ? rj :
				  dst_is_r1 ? 5'd1 : 
				  rd
	;



	// control and status instructions
	assign csr_we = csr_inst && (rj != 5'b0);
	assign csr_wmask_en = (rj != 5'b0) && (rj != 5'b1);

	// wire [5:0] ecode_ID_m; // modified exception code
	assign ecode_ID_m = (ecode_ID != 6'b0) ? ecode_ID // if exists, keep it
					  : ( {6{syscall_inst}} & 6'hb
						| {6{break_inst  }} & 6'hc
						| {6{illegal_inst}} & 6'hd
						//| {6{!(syscall_inst || break_inst || illegal_inst)}} & 6'b0
					  )
	;
	assign exc_from_IF_ID = (ecode_ID != 6'd0);

	// rdcn_op - onehot[2:id, 1:l, 0:h]
	assign rdcn_op = {rk[0] == 1'b0 && rd == 5'b0,
					  rk[0] == 1'b0 && rj == 5'b0,
					  rk[0] == 1'b1
	};

	assign invtlb_op = rd;

	// memory access control
	assign byte_we = (op_25_22[1:0] == 2'b00);
	assign half_we =  op_25_22[0];
	assign word_we =  op_25_22[1];
	assign signed_we = ~op_25_22[3];

	// flag_refresh
	always @(posedge clk) begin
		if (reset)
			flag_refresh <= 1'd0;
		else if (EX_MEM_WB_ex_ertn)
			flag_refresh <= 1'd0;
		else if (pipe_tonext_valid_ID && (
					csr_we && (csr_code_ID == 9'h0 || csr_code_ID == 9'h18	||	// CRMD, ASID
					csr_code_ID == 9'h180 || csr_code_ID == 9'h181)			||	// DMW0, DMW1
					inst_tlbwr || inst_tlbrd || inst_tlbfill || inst_invtlb
				)
			)
			/*
			 * When these instructions are detected, set this flag to invalidate all instructions
			 * passing from ID stage by marking them interrupt until the first of them
			 * reaches WB stage and refresh the pipeline.
			 */
			flag_refresh <= 1'd1;
	end

endmodule