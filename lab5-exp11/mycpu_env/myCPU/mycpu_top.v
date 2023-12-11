`define RADDR	5
`define BYTE	8
`define HALF	16
`define WORD	32
`define WIDTH	32
`define DWIDTH	64
`define R0		5'b00000

module mycpu_top(
	input  wire        clk,
	input  wire        resetn,
	// inst sram interface
	output wire        inst_sram_en,	// instruction SRAM port enable pin
	output wire [ 3:0] inst_sram_we,	// instruction SRAM byte-writes
	output wire [31:0] inst_sram_addr,
	output wire [31:0] inst_sram_wdata,
	input  wire [31:0] inst_sram_rdata,
	// data sram interface
	output wire        data_sram_en,	// data SRAM port enable pin
	output wire [ 3:0] data_sram_we,	// data SRAM byte-writes
	output wire [31:0] data_sram_addr,
	output wire [31:0] data_sram_wdata,
	input  wire [31:0] data_sram_rdata,
	// trace debug interface
	output wire [31:0] debug_wb_pc,
	output wire [ 3:0] debug_wb_rf_we,
	output wire [ 4:0] debug_wb_rf_wnum,
	output wire [31:0] debug_wb_rf_wdata
);

/* --------------------------------------
	Declarations
  ------------------------------------ */

	parameter Stage_Num = 5; // number of pipeline stages

	reg reset;

	// pipeline controllers 
	wire [Stage_Num - 1 : 0] pipe_ready_go;		// 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
	wire [Stage_Num - 1 : 0] pipe_allowin;		// 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.
	wire [Stage_Num - 2 : 0] pipe_tonext_valid;	// 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB. "4" bits
	reg  [Stage_Num - 1 : 0] pipe_valid;		// 0: IF. 1: ID. 2: EX. 3: MEM. 4: WB.

	// next PC calculation
	wire                  br_taken; // branch taken or not
	wire [`WIDTH - 1 : 0] br_target; // branch target, PC+4 or branch target
	wire [`WIDTH - 1 : 0] seq_pc; // sequential PC value, PC+4
	wire [`WIDTH - 1 : 0] nextpc; // next PC value, branch target

	// register equality detection
	wire rd_EX_r1_eq, rd_MEM_r1_eq, rd_WB_r1_eq; // r1 in ID stage is equal to r1 in EX/MEM/WB stage
	wire rd_EX_r2_eq, rd_MEM_r2_eq, rd_WB_r2_eq; // r2 in ID stage is equal to r2 in EX/MEM/WB stage

	// hazard detection
	wire hzd_alu_EX_r1, hzd_alu_MEM_r1; // ALU type hazard for r1
	wire hzd_alu_EX_r2, hzd_alu_MEM_r2; // ALU type hazard for r2
	wire hzd_ld_mul_EX_r1, hzd_ld_mul_MEM_r1; // load type hazard for r1
	wire hzd_ld_mul_EX_r2, hzd_ld_mul_MEM_r2; // load type hazard for r2
	wire hzd_WB_r1, hzd_WB_r2;

	wire calc_done; // calculation ready, for mul/div instructions
	wire ID_hazard; // ID stage installed/bubbled for hazard

	/*--- IF declaration ---*/
	wire [`WIDTH - 1 : 0] inst_final;
	wire [`WIDTH - 1 : 0] pc;
	/*--- IF declaration ---*/

	/*--- ID declaration ---*/
	wire [        12 : 0] alu_op; // ALU operation code
	wire [`WIDTH - 1 : 0] rj_value, rkd_value, imm; // rj, rkd, immediate
	wire [`WIDTH - 1 : 0] pc_ID; // PC

	wire [         3 : 0] op_25_22; // opcode[25:22]
	wire [         6 : 0] op_21_15; // opcode[21:15]
	wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest; // register file read address

	wire                  ld_inst, st_inst, div_inst, mul_inst; // instruction type - class
	wire                  br_taken_sure, br_taken_yes, br_taken_no; // branch taken or not
	wire                  src1_is_pc, src2_is_imm, src2_is_4, br_src_sel; // source operand control signal
	wire                  res_from_mem, gr_we, mem_we; // result from memory, general register write enable, memory write enable
	wire                  rf_ren1, rf_ren2; // register file read enable
	/*--- ID declaration ---*/

	/*--- EX declaration ---*/
	wire [`WIDTH - 1 : 0] pc_EX, rkd_value_EX; // PC, ALU result, rkd
	wire [`WIDTH - 1 : 0] alu_result; // PC, rkd
	wire [`DWIDTH - 1: 0] mul_result; // to WB - skip MEM
	wire [         3 : 0] op_25_22_EX; // opcode[25:22]
	wire [`RADDR - 1 : 0] rf_raddr1_EX, rf_raddr2_EX, dest_EX; // register file read address
	wire                  ld_inst_EX, st_inst_EX;	// store
	wire				  mul_inst_EX;
	wire [         2 : 0] mul_op_EX;
	wire                  res_from_mem_EX, gr_we_EX, mem_we_EX; // result from memory, general register write enable, memory write enable
	/*--- EX declaration ---*/

	/*--- MEM declaration ---*/
	wire [`WIDTH - 1 : 0] pc_MEM, alu_result_MEM; // PC, ALU result
	wire [`RADDR - 1 : 0] rf_raddr1_MEM, rf_raddr2_MEM, dest_MEM; // register file read address
	wire [         3 : 0] mask_dataram; // data memory byte-write mask
	wire                  byte_we, half_we, word_we, signed_we; // byte, half, word, signed write enable
	wire                  ld_inst_MEM; // load instruction
	wire                  res_from_mem_MEM, gr_we_MEM; // result from memory, general register write enable
	wire				  mul_inst_MEM;
	wire [         2 : 0] mul_op_MEM;
	/*--- MEM declaration ---*/

	/*--- WB declaration ---*/
	wire                  gr_we_WB; // general register write enable

	wire                  rf_we;    // register file write enable
	wire [`RADDR - 1 : 0] rf_waddr; // register file write address
	wire [`RADDR - 1 : 0] dest_WB;  // destination register number
	wire [`WIDTH - 1 : 0] rf_wdata; // register file write data
	/*--- WB declaration ---*/


/* --------------------------------------------------------------------
	reset signal generation
  ------------------------------------------------------------------ */

	always @(posedge clk) begin
		reset <= ~resetn;
	end


/* --------------------------------------------------------------------
	Pipeline control
  ------------------------------------------------------------------ */

	/* pre-if stage end */
	assign pipe_ready_go[0] = pipe_valid[0];
	assign pipe_ready_go[1] = pipe_valid[1] && !ID_hazard;
	assign pipe_ready_go[2] = pipe_valid[2] && calc_done;
	assign pipe_ready_go[3] = pipe_valid[3];
	assign pipe_ready_go[4] = pipe_valid[4];

	// judge whether the stage allows new instruction to enter
	assign pipe_allowin[Stage_Num - 2 : 0] =
		~pipe_valid[Stage_Num - 2 : 0] | pipe_ready_go[Stage_Num - 2 : 0] & pipe_allowin[Stage_Num - 1 : 1];
	assign pipe_allowin[Stage_Num - 1] =
		~pipe_valid[Stage_Num - 1] | pipe_ready_go[Stage_Num - 1];

	// judge whether the stage is ready to go
	assign pipe_tonext_valid[Stage_Num - 2 : 0] =
		pipe_allowin[Stage_Num - 1 : 1] & pipe_ready_go[Stage_Num - 2 : 0];

	// valid signal control in pipeline
	always @(posedge clk) begin
		if (reset) begin
				pipe_valid <= 5'b00000;
		end
		else begin
			// IF stage
			if (pipe_allowin[0]) begin
				pipe_valid[0] <= 1'b1;
			end

			// ID stage
			if (br_taken) begin
				pipe_valid[1] <= 1'b0;
			end
			else  begin
				if (pipe_allowin[1]) begin
					pipe_valid[1] <= pipe_ready_go[0];
				end
			end

			// EX stage
			if (br_taken) begin
				if (pipe_tonext_valid[2]) begin
					pipe_valid[2] <= 1'b0;
				end
			end
			else  begin
				if (pipe_allowin[2]) begin
					pipe_valid[2] <= pipe_ready_go[1];
				end
			end

			// MEM stage
			if (pipe_allowin[3]) begin
				pipe_valid[3] <= pipe_ready_go[2];
			end

			// WB stage
			if (pipe_allowin[4]) begin
				pipe_valid[4] <= pipe_ready_go[3];
			end
		end
	end

	/* pre-if stage begin */

	// next PC calculation
	assign seq_pc = pc + 3'h4; // PC+4, the next PC value.
	assign nextpc = br_taken ? br_target : seq_pc; // calculate the next PC value. 

	// instruction memory (SRAM)
	assign inst_sram_en    = 1'b1;		// instruction memory enable
	assign inst_sram_we    = 4'b0000;	// instruction memory byte-writes
	assign inst_sram_addr  = nextpc;	// instruction memory address
	assign inst_sram_wdata = `WIDTH'b0;	// instruction memory write data


/* --------------------------------------------------------------------
	Pipeline Hazard Control
  ------------------------------------------------------------------ */

	// register equality detection
	assign rd_EX_r1_eq  = (rf_raddr1 != `R0) && (rf_raddr1 == dest_EX)  && pipe_valid[2];
	assign rd_EX_r2_eq  = (rf_raddr2 != `R0) && (rf_raddr2 == dest_EX)  && pipe_valid[2];
	assign rd_MEM_r1_eq = (rf_raddr1 != `R0) && (rf_raddr1 == dest_MEM) && pipe_valid[3];
	assign rd_MEM_r2_eq = (rf_raddr2 != `R0) && (rf_raddr2 == dest_MEM) && pipe_valid[3];
	assign rd_WB_r1_eq  = (rf_raddr1 != `R0) && (rf_raddr1 == dest_WB)  && pipe_valid[4];
	assign rd_WB_r2_eq  = (rf_raddr2 != `R0) && (rf_raddr2 == dest_WB)  && pipe_valid[4];

	// hazard detection
	assign hzd_alu_EX_r1  = rd_EX_r1_eq  && gr_we_EX  && !ld_inst_EX;
	assign hzd_alu_EX_r2  = rd_EX_r2_eq  && gr_we_EX  && !ld_inst_EX;
	assign hzd_alu_MEM_r1 = rd_MEM_r1_eq && gr_we_MEM && !ld_inst_MEM;
	assign hzd_alu_MEM_r2 = rd_MEM_r2_eq && gr_we_MEM && !ld_inst_MEM;
	assign hzd_ld_mul_EX_r1   = rd_EX_r1_eq  && (ld_inst_EX || mul_inst_EX); // mul
	assign hzd_ld_mul_EX_r2   = rd_EX_r2_eq  && (ld_inst_EX || mul_inst_EX); // mul
	assign hzd_ld_mul_MEM_r1  = rd_MEM_r1_eq && (ld_inst_MEM || mul_inst_MEM); // mul
	assign hzd_ld_mul_MEM_r2  = rd_MEM_r2_eq && (ld_inst_MEM || mul_inst_MEM); // mul
	assign hzd_WB_r1 = rd_WB_r1_eq && gr_we_WB, hzd_WB_r2 = rd_WB_r2_eq && gr_we_WB;

	// hazard that installs ID stage
	assign ID_hazard = hzd_ld_mul_EX_r1  && rf_ren1
					|| hzd_ld_mul_EX_r2  && rf_ren2
					|| hzd_ld_mul_MEM_r1 && rf_ren1
					|| hzd_ld_mul_MEM_r2 && rf_ren2
	;


/* --------------------------------------------------------------------
	Pipeline Stages Instantiation
  ------------------------------------------------------------------ */

	/*
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
	*/
	stage_IF stage_IF_inst (
		.clk					(clk),
		.reset					(reset),

		.pipe_allowin_IF		(pipe_allowin[0]),
		.pipe_tonext_valid_IF	(pipe_tonext_valid[0]),
		.pipe_valid_IF			(pipe_valid[0]),
		.nextpc					(nextpc),

		.inst_sram_rdata		(inst_sram_rdata),

		.br_taken				(br_taken),

		.inst_final				(inst_final),
		.pc						(pc)
	);

	/*
	module stage_ID (
		input  wire                  clk,
		input  wire                  reset, // localized reset signal

		// input from stage controller
		input  wire                  pipe_tonext_valid_IF, pipe_tonext_valid_ID,
		input  wire                  hzd_alu_EX_r1, hzd_alu_MEM_r1, hzd_alu_WB_r1, hzd_ld_mul_WB_r1,
		input  wire                  hzd_alu_EX_r2, hzd_alu_MEM_r2, hzd_alu_WB_r2, hzd_ld_mul_WB_r2,
		input  wire [`WIDTH - 1 : 0] alu_result, alu_result_MEM

		// input from IF stage
		input  wire [`WIDTH - 1 : 0] inst_final,
		input  wire [`WIDTH - 1 : 0] pc,

		// input from WB stage
		input  wire                  rf_we,		// register file write enable
		input  wire [`RADDR - 1 : 0] rf_waddr,	// register file write address
		input  wire [`WIDTH - 1 : 0] rf_wdata,	// register file write data

		// output to EX
		output wire [        12 : 0] alu_op,
		output wire [`WIDTH - 1 : 0] rj_value, rkd_value, imm,
		output reg  [`WIDTH - 1 : 0] pc_ID,
		output wire [         3 : 0] op_25_22,
		output wire [         6 : 0] op_21_15,
		output wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest,
		output wire                  ld_inst, st_inst, div_inst, mul_inst,
		output wire                  br_taken_sure, br_taken_yes, br_taken_no,
		output wire                  src1_is_pc, src2_is_imm, src2_is_4, br_src_sel,
		output wire                  res_from_mem, gr_we, mem_we,
		output wire                  rf_ren1, rf_ren2
	};
	*/
	stage_ID stage_ID_inst (
		.clk (clk),
		.reset (reset),

		.pipe_tonext_valid_IF (pipe_tonext_valid[0]), .pipe_tonext_valid_ID(pipe_tonext_valid[1]),
		.hzd_alu_EX_r1 (hzd_alu_EX_r1), .hzd_alu_MEM_r1 (hzd_alu_MEM_r1), .hzd_WB_r1(hzd_WB_r1),
		.hzd_alu_EX_r2 (hzd_alu_EX_r2), .hzd_alu_MEM_r2 (hzd_alu_MEM_r2), .hzd_WB_r2(hzd_WB_r2),
		.alu_result (alu_result), .alu_result_MEM (alu_result_MEM),
		.rf_wdata_WB(rf_wdata),	/*These three are forwarding */

		.inst_final (inst_final),
		.pc (pc),

		.rf_we (rf_we),
		.rf_waddr (rf_waddr),
		.rf_wdata (rf_wdata),
	
		.alu_op (alu_op),
		.rj_value (rj_value), .rkd_value (rkd_value), .imm (imm),
		.pc_ID (pc_ID),
		.op_25_22 (op_25_22),
		.op_21_15 (op_21_15),
		.rf_raddr1 (rf_raddr1), .rf_raddr2 (rf_raddr2), .dest (dest),
		.ld_inst (ld_inst), .st_inst (st_inst), .div_inst (div_inst), .mul_inst (mul_inst),
		.br_taken_sure (br_taken_sure), .br_taken_yes (br_taken_yes), .br_taken_no (br_taken_no),
		.src1_is_pc (src1_is_pc), .src2_is_imm (src2_is_imm), .src2_is_4 (src2_is_4), .br_src_sel(br_src_sel),
		.res_from_mem (res_from_mem), .gr_we (gr_we), .mem_we (mem_we),
		.rf_ren1 (rf_ren1), .rf_ren2 (rf_ren2)
	);	
	
	/*
	module stage_EX (
		input  wire                  clk,
		input  wire                  reset, // localized reset signal

		// input from stage controller
		input  wire                  pipe_tonext_valid_ID, // allow EX stage to accept new instruction
		input  wire                  pipe_valid_EX, // EX stage is valid

		// input from ID
		input  wire [        12 : 0] alu_op,
		input  wire [`WIDTH - 1 : 0] rj_value, rkd_value, pc_ID, imm,
		input  wire [         3 : 0] op_25_22,
		input  wire [         6 : 0] op_21_15,
		input  wire [`RADDR - 1 : 0] rf_raddr1, rf_raddr2, dest,
		input  wire                  ld_inst, st_inst, div_inst, mul_inst,
		input  wire                  br_taken_sure, br_taken_yes, br_taken_no,
		input  wire                  src1_is_pc, src2_is_imm, src2_is_4, br_src_sel,
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
	*/
	stage_EX stage_EX_inst (
		.clk (clk),
		.reset (reset),
	
		.pipe_tonext_valid_ID (pipe_tonext_valid[1]),
		.pipe_valid_EX (pipe_valid[2]),
	
		.alu_op (alu_op),
		.rj_value (rj_value), .rkd_value (rkd_value), .pc_ID (pc_ID), .imm (imm),
		.op_25_22 (op_25_22),
		.op_21_15 (op_21_15),
		.rf_raddr1 (rf_raddr1), .rf_raddr2 (rf_raddr2), .dest (dest),
		.ld_inst (ld_inst), .st_inst (st_inst), .div_inst (div_inst), .mul_inst (mul_inst),
		.br_taken_sure (br_taken_sure), .br_taken_yes (br_taken_yes), .br_taken_no (br_taken_no),
		.src1_is_pc (src1_is_pc), .src2_is_imm (src2_is_imm), .src2_is_4 (src2_is_4), .br_src_sel(br_src_sel),
		.res_from_mem (res_from_mem), .gr_we (gr_we), .mem_we (mem_we),
	
		.pc_EX (pc_EX), .rkd_value_EX (rkd_value_EX),
		.alu_result (alu_result), .mul_result (mul_result),
		.op_25_22_EX (op_25_22_EX),
		.rf_raddr1_EX (rf_raddr1_EX), .rf_raddr2_EX (rf_raddr2_EX), .dest_EX (dest_EX),
		.ld_inst_EX (ld_inst_EX), .st_inst_EX (st_inst_EX),
		.mul_inst_EX (mul_inst_EX), .mul_op_EX (mul_op_EX),
		.res_from_mem_EX (res_from_mem_EX), .gr_we_EX (gr_we_EX), .mem_we_EX (mem_we_EX),
	
		.br_target (br_target),
		.br_taken (br_taken), .calc_done (calc_done)
	
	);

	/*
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
	*/
	stage_MEM stage_MEM_inst (
		.clk (clk),
		.reset (reset),

		.pipe_tonext_valid_EX (pipe_tonext_valid[2]),
		.pipe_valid_MEM (pipe_valid[3]),

		.pc_EX (pc_EX), .alu_result (alu_result), .rkd_value_EX (rkd_value_EX),
		.op_25_22_EX (op_25_22_EX),
		.rf_raddr1_EX (rf_raddr1_EX), .rf_raddr2_EX (rf_raddr2_EX), .dest_EX (dest_EX),
		.ld_inst_EX (ld_inst_EX), .st_inst_EX (st_inst_EX),
		.mul_inst_EX (mul_inst_EX), .mul_op_EX (mul_op_EX),
		.res_from_mem_EX (res_from_mem_EX), .gr_we_EX (gr_we_EX), .mem_we_EX (mem_we_EX),

		.pc_MEM (pc_MEM), .alu_result_MEM (alu_result_MEM),
		.rf_raddr1_MEM (rf_raddr1_MEM), .rf_raddr2_MEM (rf_raddr2_MEM), .dest_MEM (dest_MEM),
		.mask_dataram (mask_dataram),
		.byte_we (byte_we), .half_we (half_we), .word_we (word_we), .signed_we (signed_we),
		.ld_inst_MEM (ld_inst_MEM),
		.mul_inst_MEM (mul_inst_MEM), .mul_op_MEM (mul_op_MEM),
		.res_from_mem_MEM (res_from_mem_MEM), .gr_we_MEM (gr_we_MEM),

		.data_sram_en (data_sram_en),
		.data_sram_we (data_sram_we),
		.data_sram_addr (data_sram_addr), .data_sram_wdata (data_sram_wdata)
	);
	

	/*
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

		output wire [`WIDTH - 1 : 0] debug_wb_pc,			// debug info
		output wire [         3 : 0] debug_wb_rf_we,		// debug info
		output wire [`RADDR - 1 : 0] debug_wb_rf_wnum,	// debug info
		output wire [`WIDTH - 1 : 0] debug_wb_rf_wdata	// debug info
	);
	*/
	stage_WB stage_WB_inst (
		.clk (clk),
		.reset (reset),

		.pipe_tonext_valid_MEM (pipe_tonext_valid[3]),
		.pipe_valid_WB (pipe_valid[4]),
		.data_sram_rdata (data_sram_rdata),

		.pc_MEM (pc_MEM), .alu_result_MEM (alu_result_MEM),
		.mul_result (mul_result),
		.rf_raddr1_MEM (rf_raddr1_MEM), .rf_raddr2_MEM (rf_raddr2_MEM), .dest_MEM (dest_MEM),
		.mask_dataram (mask_dataram),
		.byte_we (byte_we), .half_we (half_we), .word_we (word_we), .signed_we (signed_we),
		.mul_inst_MEM (mul_inst_MEM), .mul_op_MEM (mul_op_MEM),
		.res_from_mem_MEM (res_from_mem_MEM), .gr_we_MEM (gr_we_MEM),

		.gr_we_WB (gr_we_WB),
		
		.rf_we (rf_we),
		.rf_waddr (rf_waddr),
		.dest_WB (dest_WB),
		.rf_wdata (rf_wdata),

		.debug_wb_pc (debug_wb_pc),
		.debug_wb_rf_we (debug_wb_rf_we),
		.debug_wb_rf_wnum (debug_wb_rf_wnum),
		.debug_wb_rf_wdata (debug_wb_rf_wdata)
	);


endmodule
