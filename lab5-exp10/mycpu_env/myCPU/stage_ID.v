`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000

module stage_ID (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_IF,
	input  wire                  hzd_alu_EX_r1, hzd_alu_MEM_r1, hzd_alu_WB_r1, hzd_ld_mul_WB_r1,
	input  wire                  hzd_alu_EX_r2, hzd_alu_MEM_r2, hzd_alu_WB_r2, hzd_ld_mul_WB_r2,
	input  wire [`WIDTH - 1 : 0] alu_result, alu_result_MEM, calc_result_WB, /*mem_result*/mem_mul_result, // mul

	// input from IF stage
	input  wire [`WIDTH - 1 : 0] inst_final,
	input  wire [`WIDTH - 1 : 0] pc,

	// input from WB stage
	input  wire                  rf_we,		// register file write enable
	input  wire [`RADDR - 1 : 0] rf_waddr,	// register file write address
	input  wire [`WIDTH - 1 : 0] rf_wdata,	// register file write data

	// output to EX
	output wire [        11 : 0] alu_op,
	output wire [`WIDTH - 1 : 0] rj_value, rkd_value, imm,
	output reg  [`WIDTH - 1 : 0] pc_ID,
	output wire [         3 : 0] op_25_22,
	output wire [         6 : 0] op_21_15,
	output wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest,
	output wire                  ld_inst, st_inst, div_inst, mul_inst,
	output wire                  inst_jirl, inst_b, inst_bl, inst_beq, inst_bne, inst_blt, inst_bge, inst_bltu, inst_bgeu,
	output wire                  src1_is_pc, src2_is_imm, src2_is_4,
	output wire                  res_from_mem, gr_we, mem_we,
	output wire                  rf_ren1, rf_ren2
);

	/* --------------------------------------
		IF -> ID
	-------------------------------------- */

	reg [`WIDTH - 1 : 0] inst_ID;
	// reg [`WIDTH - 1 : 0] pc_ID;

	always @(posedge clk) begin
		if (pipe_tonext_valid_IF) begin
			inst_ID <= inst_final;
			pc_ID   <= pc;
		end
	end

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

	// register numbers
	wire [`RADDR - 1 : 0] rd;
	wire [`RADDR - 1 : 0] rj;
	wire [`RADDR - 1 : 0] rk;
	assign rd  = inst_ID[ 4: 0];
	assign rj  = inst_ID[ 9: 5];
	assign rk  = inst_ID[14:10];

	// different immediate values
	wire [11:0] imm12;
	wire [15:0] imm16;
	wire [19:0] imm20;
	wire [25:0] imm26;
	assign imm12 = inst_ID[21:10];
	assign imm16 = inst_ID[25:10];
	assign imm20 = inst_ID[24: 5];
	assign imm26 = {inst_ID[ 9: 0], inst_ID[25:10]};

	wire br_inst;		// branch & jump
	// wire ld_inst;		// load
	// wire st_inst;		// store
	wire lu_inst;		// load upper
	wire tlb_inst;		// translation lookaside buffer
	wire ertn_inst;		// exception return
	wire cache_inst;	// cache operation
	wire csr_inst;		// control and status register
	wire alui_inst;		// ALU immediate
	wire sfti_inst;		// shifter immediate
	wire syscall_inst;	// system call
	// wire div_inst;		// divide
	// wire mul_inst;		// multiply
	wire sftr_inst;		// shifter register
	wire alur_inst;		// ALU register
	wire rdcn_inst;		// read control and status register
	assign br_inst = op_31_26[4];
	assign ld_inst = (op_31_26[4:3] == 2'b01) & ~op_25_22[2];
	assign st_inst = (op_31_26[4:3] == 2'b01) &  op_25_22[2];
	assign lu_inst = (op_31_26[4:2] == 3'b001);
	assign tlb_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0001)
					&& ({op_25_22[3], op_25_22[0]} == 2'b11)
					&& (op_21_15[0] || (rk[2] ^ rk[1]))
	;
	assign ertn_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0001)
					 && ({op_25_22[3], op_25_22[0]} == 2'b11)
					 && (~op_21_15[0] && rk[2] && rk[1])
	;
	assign cache_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0001)
					  && ({op_25_22[3], op_25_22[0]} == 2'b10)
	;
	assign csr_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0001)
					&& (~op_25_22[3])
	;
	assign alui_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
					 && (op_25_22[3])
	;
	assign sfti_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
					 && ({op_25_22[3], op_25_22[0]} == 2'b01)
	;
	assign syscall_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
						&& ({op_25_22[3], op_25_22[0]} == 2'b00)
						&& ({op_21_15[6], op_21_15[4]} == 2'b11)
	;
	assign div_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
					&& ({op_25_22[3], op_25_22[0]} == 2'b00)
					&& ({op_21_15[6], op_21_15[4]} == 2'b10)
	;
	assign mul_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
					&& ({op_25_22[3], op_25_22[0]} == 2'b00)
					&& ({op_21_15[5:3]} == 4'b111)
	;
	assign sftr_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
					 && ({op_25_22[3], op_25_22[0]} == 2'b00)
					 && (({op_21_15[5:3]} == 3'b110) || ({op_21_15[6:2]} == 5'b01011))
	;
	assign alur_inst =  ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
					 && ({op_25_22[3], op_25_22[0]} == 2'b00)
					 && (op_21_15[5:4] == 2'b10)
					 && !(op_21_15[3] && op_21_15[2])
	;
	assign rdcn_inst = ({op_31_26[4:2], op_31_26[0]} == 4'b0000)
					 && ({op_25_22[3], op_25_22[0]} == 2'b00)
					 && (op_21_15[6:5] == 2'b00)
					//  && (op_21_15[3:2] == 2'b11)
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
	wire inst_slti    = alui_inst && (op_25_22[2:0] == 3'b000);
	wire inst_sltui   = alui_inst && (op_25_22[2:0] == 3'b001);
	wire inst_addi_w  = alui_inst && (op_25_22[2:0] == 3'b010);
	wire inst_andi    = alui_inst && (op_25_22[2:0] == 3'b101);
	wire inst_ori     = alui_inst && (op_25_22[2:0] == 3'b110);
	wire inst_xori    = alui_inst && (op_25_22[2:0] == 3'b111);
	wire inst_lu12i_w   = lu_inst && ~op_31_26[1];
	wire inst_pcaddu12i = lu_inst &&  op_31_26[1];

	// branch instructions
	assign inst_jirl = br_inst && (op_31_26[3:2] == 2'b00);
	assign inst_b    = br_inst && (op_31_26[2:0] == 3'b100);
	assign inst_bl   = br_inst && (op_31_26[2:0] == 3'b101);
	assign inst_beq  = br_inst && (op_31_26[2:0] == 3'b110);
	assign inst_bne  = br_inst && (op_31_26[2:0] == 3'b111);
	assign inst_blt  = br_inst && ({op_31_26[3], op_31_26[1:0]} == 3'b100);
	assign inst_bge  = br_inst && ({op_31_26[3], op_31_26[1:0]} == 3'b101);
	assign inst_bltu = br_inst && ({op_31_26[3], op_31_26[1:0]} == 3'b110);
	assign inst_bgeu = br_inst && ({op_31_26[3], op_31_26[1:0]} == 3'b111);

	// calculate alu operation control signal
	assign alu_op[ 0] = inst_add_w || inst_addi_w || ld_inst || st_inst || inst_jirl || inst_bl || inst_pcaddu12i;
	assign alu_op[ 1] = inst_sub_w;
	assign alu_op[ 2] = inst_slt || inst_slti || inst_blt || inst_bge;
	assign alu_op[ 3] = inst_sltu || inst_sltui || inst_bltu || inst_bgeu;
	assign alu_op[ 4] = inst_and || inst_andi;
	assign alu_op[ 5] = inst_nor;
	assign alu_op[ 6] = inst_or || inst_ori;
	assign alu_op[ 7] = inst_xor || inst_xori;
	assign alu_op[ 8] = inst_sll_w || inst_slli_w;
	assign alu_op[ 9] = inst_srl_w || inst_srli_w;
	assign alu_op[10] = inst_sra_w || inst_srai_w;
	assign alu_op[11] = inst_lu12i_w;

	// control signals of immediate's required bits
	wire need_ui5  = inst_slli_w || inst_srli_w || inst_srai_w;
	wire need_si12 = inst_slti || inst_sltui || inst_addi_w || ld_inst || st_inst;
	wire need_ui12 = inst_andi || inst_ori || inst_xori;
	wire need_si16 = br_inst && !(inst_b || inst_bl);
	wire need_si20 = inst_lu12i_w || inst_pcaddu12i;
	wire need_si26 = inst_b || inst_bl;

	// immediate value
	assign imm = {`WIDTH{need_ui5 | need_si12}} & {{20{imm12[11]}}, imm12[11:0]} // ui5 only care last 5 bits
			   | {`WIDTH{need_ui12}}            & { 20'b0         , imm12[11:0]}
			   | {`WIDTH{need_si16}}            & {{14{imm16[15]}}, imm16[15:0],  2'b0}
			   | {`WIDTH{need_si20}}            & {                 imm20[19:0], 12'b0} 
			   | {`WIDTH{need_si26}}            & {{ 4{imm26[25]}}, imm26[25:0],  2'b0}
	;

	// control signals of ALU oprand source and destination register
	wire src_reg_is_rd = br_inst && (op_31_26[3] || op_31_26[2] && op_31_26[1]) || st_inst;
	assign src1_is_pc  = inst_jirl || inst_bl || inst_pcaddu12i;
	assign src2_is_imm = sfti_inst || alui_inst || ld_inst || st_inst || lu_inst;
	assign src2_is_4   = inst_jirl || inst_bl;
	wire dst_is_r1     = inst_bl;

	// control signals of write enable
	assign res_from_mem  = ld_inst;
	assign gr_we         = !st_inst && !(br_inst && !inst_jirl && !inst_bl);
	assign mem_we        = st_inst;

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

	/*--- only til exp11 ---*/
	// for installing ID stage
	assign rf_ren1 = br_inst && !(inst_b || inst_bl)
				  || ld_inst
				  || st_inst
				  || alui_inst
				  || sfti_inst
				  || div_inst
				  || mul_inst
				  || sftr_inst
				  || alur_inst
	;
	assign rf_ren2 = br_inst && !(inst_jirl || inst_b || inst_bl)
				  || st_inst
				  || div_inst
				  || mul_inst
				  || sftr_inst
				  || alur_inst
	;

	// control signals of register
	assign rj_value  = hzd_alu_EX_r1  ? alu_result     :
					   hzd_alu_MEM_r1 ? alu_result_MEM :
					   hzd_alu_WB_r1  ? calc_result_WB  :
					   hzd_ld_mul_WB_r1   ? mem_mul_result :
					   rf_rdata1
	;
	assign rkd_value = hzd_alu_EX_r2  ? alu_result     :
					   hzd_alu_MEM_r2 ? alu_result_MEM :
					   hzd_alu_WB_r2  ? calc_result_WB  :
					   hzd_ld_mul_WB_r2   ? mem_mul_result :
					   rf_rdata2
	;

	// wire [`RADDR - 1 : 0] dest; // destination register number
	assign dest = dst_is_r1 ? 5'd1 : rd;



endmodule