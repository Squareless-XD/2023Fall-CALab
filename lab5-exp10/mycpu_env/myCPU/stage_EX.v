`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000

module stage_EX (
	input  wire                  clk,
	input  wire                  reset, // localized reset signal

	// input from stage controller
	input  wire                  pipe_tonext_valid_ID, // allow EX stage to accept new instruction
	input  wire                  pipe_valid_EX, // EX stage is valid

	// input from ID
	input  wire [        11 : 0] alu_op,
	input  wire [`WIDTH - 1 : 0] rj_value, rkd_value, pc_ID, imm,
	input  wire [         3 : 0] op_25_22,
	input  wire [         6 : 0] op_21_15,
	input  wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest,
	input  wire                  ld_inst, st_inst, div_inst, mul_inst,
	input  wire                  inst_jirl, inst_b, inst_bl, inst_beq, inst_bne, inst_blt, inst_bge, inst_bltu, inst_bgeu,
	input  wire                  src1_is_pc, src2_is_imm, src2_is_4,
	input  wire                  res_from_mem, gr_we, mem_we,

	// output to MEM
	output reg  [`WIDTH - 1 : 0] pc_EX, rkd_value_EX,
	output wire [`WIDTH - 1 : 0] alu_result, 
	output wire [`DWIDTH - 1: 0] mul_result,				// mul - to WB
	output reg  [         3 : 0] op_25_22_EX,
	output reg  [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX,
	output reg                   ld_inst_EX, st_inst_EX,	// store
	output reg					 mul_inst_EX,               // mul
	output wire [		  2 : 0] mul_op_EX,                 // mul
	output reg                   res_from_mem_EX, gr_we_EX, mem_we_EX,

	// output
	output wire [`WIDTH - 1 : 0] br_target, // branch target, PC+4 or branch target
	output wire                  br_taken, calc_done   // calculation ready, for mul/div instructions
);

	/* --------------------------------------
		ID -> EX
	-------------------------------------- */

	reg [11:0] alu_op_EX;
	reg [`WIDTH - 1 : 0] rj_value_EX;
	// reg [`WIDTH - 1 : 0] rkd_value_EX;
	// reg [`WIDTH - 1 : 0] pc_EX;
	reg [`WIDTH - 1 : 0] imm_EX;

	// reg [3:0] op_25_22_EX;
	reg [6:0] op_21_15_EX;
	// reg [4:0] rf_raddr1_EX;
	// reg [4:0] rf_raddr2_EX;
	// reg [4:0] dest_EX;

	// reg ld_inst_EX;		// load
	// reg st_inst_EX;		// store
	reg div_inst_EX;	// divide
	// reg mul_inst_EX;	// multiply

	reg inst_jirl_EX;
	reg inst_b_EX;
	reg inst_bl_EX;
	reg inst_beq_EX;
	reg inst_bne_EX;
	reg inst_blt_EX;
	reg inst_bge_EX;
	reg inst_bltu_EX;
	reg inst_bgeu_EX;

	reg src1_is_pc_EX;
	reg src2_is_imm_EX;
	reg src2_is_4_EX;
	// branch target (base + offset) calculation (use only one adder)
	wire [`WIDTH - 1 : 0] br_off_src1;
	wire [`WIDTH - 1 : 0] br_off_src2;
	// ALU calculation
	// wire br_src_sel;
	// wire src2_is_rkd;
	wire br_src_sel = inst_jirl_EX;
	wire src2_is_rkd = ~src2_is_imm_EX & ~src2_is_4_EX;

	wire [`WIDTH - 1 : 0] alu_src1;   // ALU source 1
	wire [`WIDTH - 1 : 0] alu_src2;   // ALU source 2
	reg first_EX;
	// wire rj_eq_rd;
	wire rj_eq_rd = (rj_value_EX == rkd_value_EX); // for branch instructions
	wire [`WIDTH - 1 : 0] alu_result_part;
	
	// multiplier
	wire [         2 : 0] mul_op;
	wire [`WIDTH - 1 : 0] mul_src1;
	wire [`WIDTH - 1 : 0] mul_src2;
	//wire [`WIDTH - 1 : 0] mul_result;
	// divider
	wire [ 3:0] div_op;
	wire [`WIDTH - 1 : 0] div_src1;
	wire [`WIDTH - 1 : 0] div_src2;
	wire [`WIDTH - 1 : 0] div_result;
	wire				  div_en;
	wire [`WIDTH - 1 : 0] div_q;     		
	wire [`WIDTH - 1 : 0] div_r;
	wire                  div_complete;

	wire use_alu_inst = ~mul_inst_EX & ~div_inst_EX;
	// reg res_from_mem_EX;
	// reg gr_we_EX;
	// reg mem_we_EX;

	always @(posedge clk) begin
		if (pipe_tonext_valid_ID) begin
			alu_op_EX    <= alu_op;
			rj_value_EX  <= rj_value;
			rkd_value_EX <= rkd_value;
			pc_EX        <= pc_ID;
			imm_EX       <= imm;

			op_25_22_EX <= op_25_22;
			op_21_15_EX <= op_21_15;
			rf_raddr1_EX <= rf_raddr1;
			rf_raddr2_EX <= rf_raddr2;
			dest_EX      <= dest;

			ld_inst_EX  <= ld_inst;
			st_inst_EX  <= st_inst;
			div_inst_EX <= div_inst;
			mul_inst_EX <= mul_inst;

			inst_jirl_EX <= inst_jirl;
			inst_b_EX    <= inst_b;
			inst_bl_EX   <= inst_bl;
			inst_beq_EX  <= inst_beq;
			inst_bne_EX  <= inst_bne;
			inst_blt_EX  <= inst_blt;
			inst_bge_EX  <= inst_bge;
			inst_bltu_EX <= inst_bltu;
			inst_bgeu_EX <= inst_bgeu;

			src1_is_pc_EX  <= src1_is_pc;
			src2_is_imm_EX <= src2_is_imm;
			src2_is_4_EX   <= src2_is_4;

			res_from_mem_EX <= res_from_mem;
			gr_we_EX        <= gr_we;
			mem_we_EX       <= mem_we;
		end
	end

	/* --------------------------------------
		EX stage
	-------------------------------------- */

	always @(posedge clk) begin
		if (reset || pipe_tonext_valid_ID) begin
			first_EX <= 1'b1;
		end
		else begin
			first_EX <= 1'b0;
		end
	end

	assign br_taken = (
		inst_jirl_EX                ||
		inst_b_EX                   ||
		inst_bl_EX                  ||
		inst_beq_EX  &&  rj_eq_rd   ||
		inst_bne_EX  && !rj_eq_rd   ||
		inst_blt_EX  &&  alu_result ||
		inst_bge_EX  && !alu_result ||
		inst_bltu_EX &&  alu_result ||
		inst_bgeu_EX && !alu_result
	) && pipe_valid_EX && first_EX;


	assign br_off_src1 = br_src_sel ? rj_value_EX : pc_EX;
	assign br_off_src2 = imm_EX;
	assign br_target = br_off_src1 + br_off_src2;

	assign alu_src1 = src1_is_pc_EX ? pc_EX[`WIDTH - 1 : 0] : rj_value_EX;

	assign alu_src2 = {`WIDTH{src2_is_imm_EX}} & imm_EX 
					| {`WIDTH{src2_is_rkd   }} & rkd_value_EX
					| {`WIDTH{src2_is_4_EX  }} & `WIDTH'h4
	;

	alu u_alu (
		.alu_op     (alu_op_EX ),
		.alu_src1   (alu_src1  ),
		.alu_src2   (alu_src2  ),
		.alu_result (alu_result_part) // modified
	);

	assign mul_op   = {
		op_21_15_EX[1:0] == 2'b10,
		op_21_15_EX[1:0] == 2'b01,
		op_21_15_EX[1:0] == 2'b00
	};
	assign mul_src1 = rj_value_EX;
	assign mul_src2 = rkd_value_EX;
	assign mul_op_EX = mul_op;

	mul u_muler (
		.mul_clk	(clk       ),
		.reset    	(reset     ),
		.x   		(mul_src1  ),
		.y   	    (mul_src2  ),
		.mul_signed (mul_op[0] | mul_op[1]),
		.result     (mul_result)
	);

	assign div_op = {
		op_21_15_EX[1:0] == 2'b11,
		op_21_15_EX[1:0] == 2'b10,
		op_21_15_EX[1:0] == 2'b01,
		op_21_15_EX[1:0] == 2'b00
	};
	assign div_src1 = rj_value_EX;
	assign div_src2 = rkd_value_EX;
	assign div_en = div_inst_EX & first_EX;

	div u_diver (
		.div_clk    (clk    		),
		.reset      (reset  		),
		.div		(div_en 		),
		.div_signed (div_op[0] | div_op[1]),
		.x  	    (div_src1  		),
		.y          (div_src2  		),
		.s          (div_q     		),
		.r          (div_r     		),
		.complete   (div_complete	)
	);

	assign div_result = {`WIDTH{div_op[0] | div_op[2]}} & div_q
					  | {`WIDTH{div_op[1] | div_op[3]}} & div_r
	;

	// only alive in EX stage
	assign alu_result = {`WIDTH{div_inst_EX }} & div_result
					  | {`WIDTH{use_alu_inst}} & alu_result_part
	;

	// calculate ready
	assign calc_done = div_inst_EX ? (div_complete && !div_en) // calc done when div complete and no new div coming
					 : ~(mul_inst_EX && first_EX) 
	;  


endmodule