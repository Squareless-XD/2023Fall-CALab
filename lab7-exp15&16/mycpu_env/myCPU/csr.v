/*
 * csr.v: module for CSR
 */

`define CSR_CRMD 9'h00
`define CSR_CRMD_PLV 1:0
`define CSR_CRMD_IE 2

`define CSR_PRMD 9'h01
`define CSR_PRMD_PPLV 1:0
`define CSR_PRMD_PIE 2

`define CSR_ECFG 9'h04
`define CSR_ECFG_LIE 12:0

`define CSR_ESTAT 9'h05
`define CSR_ESTAT_IS 12:0
`define CSR_ESTAT_ECODE 21:16
`define CSR_ESTAT_ESUBCODE 30:22

`define CSR_ERA 9'h06

`define CSR_BADV 9'h07

`define CSR_EENTRY 9'h0c
`define CSR_EENTRY_VA 31:6

`define CSR_SAVE0 9'h30
`define CSR_SAVE1 9'h31
`define CSR_SAVE2 9'h32
`define CSR_SAVE3 9'h33

`define CSR_TID 9'h40

`define CSR_TCFG 9'h41
`define CSR_TCFG_EN 0
`define CSR_TCFG_PERIODIC 1
`define CSR_TCFG_INITVAL 31:2

`define CSR_TVAL 9'h42

`define CSR_TICLR 9'h44
`define CSR_TICLR_CLR 0

`define ECODE_ADE 6'h8
`define ECODE_ALE 6'h9

`define ESUBCODE_ADEF 9'h0

module csr (
	input	wire		clk,
	input	wire		rst,

	// Ports for inst access
	input	wire [ 8:0]	csr_waddr, 	// write address
	input	wire [ 8:0]	csr_raddr,	// read  address
	input	wire		csr_we, 	// write enable
	input	wire [31:0]	csr_wmask, 	// write mask
	input	wire [31:0]	csr_wvalue,	// value to be written
	output	wire [31:0]	csr_rvalue,	// value read

	// Ports for interacting with CPU hardware
	input	wire [31:0] cpuid,		// CPU ID, just 0 is OK
	input	wire [ 7:0]	hw_int_in, 	// hardware interrupt
	input	wire		ipi_int_in, // IPI interrupt
	output	wire [31:0] ex_entry, 	// exception entry
	output	wire		has_int, 	// has interrupt
	output	wire [31:0]	era_value,	// exception return address
	input	wire		ertn_flush, // ERTN inst
	input	wire		WB_ex, 		// exc from WB
	input	wire [31:0] WB_pc,
	input	wire [31:0] WB_vaddr, 	// bad vaddr
	input	wire [ 5:0]	WB_ecode,	// exception code
	input	wire [ 8:0] WB_esubcode,// exception subcode
	output  wire [31:0] csr_tid
	/* --------------------------
		waiting for more wires
	 -------------------------- */
);

	// 0x00: CRMD
	reg [1:0] csr_crmd_plv;
	reg csr_crmd_ie;
	// Wires below haven't been implemented
	wire csr_crmd_da, csr_crmd_pg;
	wire [1:0] csr_crmd_datf, csr_crmd_datm;

	// 0x01: PRMD
	reg [1:0] csr_prmd_pplv;
	reg csr_prmd_pie;

	// 0x04: ECFG
	reg [12:0] csr_ecfg_lie;

	// 0x05: ESTAT
	reg [12:0] csr_estat_is;
	reg [5:0] csr_estat_ecode;
	reg [8:0] csr_estat_esubcode;

	// 0x06: ERA
	reg [31:0] csr_era_pc;

	// 0x07: BADV
	reg [31:0] csr_badv_vaddr;

	// 0x0c: EENTRY
	reg [25:0] csr_eentry_va;

	// 0x30~0x33: SAVE0~3
	reg [31:0] csr_save0_data, csr_save1_data,
		csr_save2_data, csr_save3_data;

	// 0x40: TID
	reg [31:0] csr_tid_tid;

	// 0x41: TCFG
	reg csr_tcfg_en, csr_tcfg_periodic;
	reg [29:0] csr_tcfg_initval;
	wire [31:0] csr_tcfg_next_value;

	// 0x42: TVAL
	wire [31:0] csr_tval_timeval;

	// Timer counter
	reg [31:0] timer_ymr;

	/*
	 * Flag of exception caused by error address
	 * from WB stage.
	 */
	wire flag_WB_ex_addr_err;

	// Value read from CSR.
	wire [31:0] rval_crmd, rval_prmd, rval_ecfg, rval_estat, rval_era,
				rval_badv, rval_eentry, rval_save0, rval_save1, rval_save2,
				rval_save3, rval_tid, rval_tcfg, rval_tval, rval_ticlr;

	// CRMD::PLV, IE
	always @(posedge clk) begin
		if (rst) begin
			csr_crmd_plv <= 2'd0;
			csr_crmd_ie  <= 1'd0;
		end
		else if (WB_ex) begin
			csr_crmd_plv <= 2'd0;
			csr_crmd_ie  <= 1'd0;
		end
		else if (ertn_flush) begin
			csr_crmd_plv <= csr_prmd_pplv;
			csr_crmd_ie  <= csr_prmd_pie;
		end
		else if (csr_we && csr_waddr == `CSR_CRMD) begin
			csr_crmd_plv <= csr_wmask[`CSR_CRMD_PLV] & csr_wvalue[`CSR_CRMD_PLV]
						 | ~csr_wmask[`CSR_CRMD_PLV] & csr_crmd_plv;
			csr_crmd_ie  <= csr_wmask[`CSR_CRMD_IE] & csr_wvalue[`CSR_CRMD_IE]
						 | ~csr_wmask[`CSR_CRMD_IE] & csr_crmd_ie;
		end
	end
	// CRMD::DA, PG, DATF, DATM
	assign csr_crmd_da = 1'd1,		csr_crmd_pg = 1'd0,
		   csr_crmd_datf = 2'd0,	csr_crmd_datm = 2'd0;

	// PRMD::PPLV, PIE
	always @(posedge clk) begin
		if (WB_ex) begin
			csr_prmd_pplv <= csr_crmd_plv;
			csr_prmd_pie  <= csr_crmd_ie;
		end
		else if (csr_we && csr_waddr == `CSR_PRMD) begin
			csr_prmd_pplv <= csr_wmask[`CSR_PRMD_PPLV] & csr_wvalue[`CSR_PRMD_PPLV]
						  | ~csr_wmask[`CSR_PRMD_PPLV] & csr_prmd_pplv;
			csr_prmd_pie  <= csr_wmask[`CSR_PRMD_PIE] & csr_wvalue[`CSR_PRMD_PIE]
						  | ~csr_wmask[`CSR_PRMD_PIE] & csr_prmd_pie;
		end
	end

	// ECFG::LIE
	always @(posedge clk) begin
		if (rst) begin
			csr_ecfg_lie <= 13'd0;
		end
		else if (csr_we && csr_waddr ==`CSR_ECFG) begin
			csr_ecfg_lie <= csr_wmask[`CSR_ECFG_LIE] & 13'h1bff & csr_wvalue[`CSR_ECFG_LIE]
						 | ~csr_wmask[`CSR_ECFG_LIE] & 13'h1bff & csr_ecfg_lie;
		end
	end

	// ESTAT::IS, Ecode, Esubcode
	always @(posedge clk) begin
		// Software interrupt
		if (rst) begin
			csr_estat_is[1:0] <= 2'd0;
		end
		else if (csr_we && csr_waddr == `CSR_ESTAT) begin
			csr_estat_is[1:0] <= csr_wmask[1:0] & csr_wvalue[1:0]
							  | ~csr_wmask[1:0] & csr_estat_is[1:0];
		end

		// Hardware interrupt
		csr_estat_is[9:2] <= hw_int_in[7:0];

		csr_estat_is[10] <= 1'd0;

		// Timer interrupt
		if (rst) begin
			csr_estat_is[11] <= 1'd0;
		end
		else if (csr_tcfg_en && timer_ymr == 32'd0) begin
			/*
			 * timer_ymr will not hold at 0 even if Periodic is 0,
			 * because it will decrease to -1 (32'hffffffff) and stop.
			 */
			csr_estat_is[11] <= 1'd1;
		end
		else if ((csr_wmask[`CSR_TICLR_CLR] & csr_wvalue[`CSR_TICLR_CLR]) &&
				 csr_we && csr_waddr == `CSR_TICLR) begin
			csr_estat_is[11] <= 1'd0;
		end

		// IPI interrupt
		csr_estat_is[12] <= ipi_int_in;

		if (WB_ex) begin
			csr_estat_ecode    <= WB_ecode;
			csr_estat_esubcode <= WB_esubcode;
		end
	end

	// ERA::PC
	always @(posedge clk) begin
		if (WB_ex) begin
			csr_era_pc <= WB_pc;
		end
		else if (csr_we && csr_waddr == `CSR_ERA) begin
			csr_era_pc <= csr_wmask[31:0] & csr_wvalue[31:0]
					   | ~csr_wmask[31:0] & csr_era_pc;
		end
	end

	// BADV::VAddr
	assign flag_WB_ex_addr_err = WB_ex && (WB_ecode == `ECODE_ADE || WB_ecode == `ECODE_ALE);
	always @(posedge clk) begin
		if (flag_WB_ex_addr_err) begin
			csr_badv_vaddr <= (WB_ecode == `ECODE_ADE && WB_esubcode == `ESUBCODE_ADEF)
							? WB_pc
							: WB_vaddr;
		end
		else if (csr_we && csr_waddr == `CSR_BADV) begin
			csr_badv_vaddr <= csr_wmask[31:0] & csr_wvalue[31:0]
						   | ~csr_wmask[31:0] & csr_badv_vaddr;
		end
	end

	// EENTRY::VA
	always @(posedge clk) begin
		if (csr_we && csr_waddr == `CSR_EENTRY) begin
			csr_eentry_va <= csr_wmask[`CSR_EENTRY_VA] & csr_wvalue[`CSR_EENTRY_VA]
						  | ~csr_wmask[`CSR_EENTRY_VA] & csr_eentry_va;
		end
	end

	// SAVE0~3::Data
	always @(posedge clk) begin
		if (csr_we && csr_waddr == `CSR_SAVE0) begin
			csr_save0_data <= csr_wmask[31:0] & csr_wvalue[31:0]
						   | ~csr_wmask[31:0] & csr_save0_data;
		end
		if (csr_we && csr_waddr == `CSR_SAVE1) begin
			csr_save1_data <= csr_wmask[31:0] & csr_wvalue[31:0]
						   | ~csr_wmask[31:0] & csr_save1_data;
		end
		if (csr_we && csr_waddr == `CSR_SAVE2) begin
			csr_save2_data <= csr_wmask[31:0] & csr_wvalue[31:0]
						   | ~csr_wmask[31:0] & csr_save2_data;
		end
		if (csr_we && csr_waddr == `CSR_SAVE3) begin
			csr_save3_data <= csr_wmask[31:0] & csr_wvalue[31:0]
						   | ~csr_wmask[31:0] & csr_save3_data;
		end
	end

	// TID::TID
	always @(posedge clk) begin
		if (rst) begin
			csr_tid_tid <= cpuid;
		end
		else if (csr_we && csr_waddr == `CSR_TID) begin
			csr_tid_tid <= csr_wmask[31:0] & csr_wvalue[31:0]
						| ~csr_wmask[31:0] & csr_tid_tid;
		end
	end

	// TCFG::En, Periodic, InitVal
	always @(posedge clk) begin
		// En
		if (rst) begin
			csr_tcfg_en <= 1'd0;
		end
		else if (csr_we && csr_waddr == `CSR_TCFG) begin
			csr_tcfg_en <= csr_wmask[`CSR_TCFG_EN] & csr_wvalue[`CSR_TCFG_EN]
						| ~csr_wmask[`CSR_TCFG_EN] & csr_tcfg_en;
		end

		// Periodic, InitVal
		if (csr_we && csr_waddr == `CSR_TCFG) begin
			csr_tcfg_periodic <= csr_wmask[`CSR_TCFG_PERIODIC] & csr_wvalue[`CSR_TCFG_PERIODIC]
							  | ~csr_wmask[`CSR_TCFG_PERIODIC] & csr_tcfg_periodic;
		end
		if (csr_we && csr_waddr == `CSR_TCFG) begin
			csr_tcfg_initval <= csr_wmask[`CSR_TCFG_INITVAL] & csr_wvalue[`CSR_TCFG_INITVAL]
							 | ~csr_wmask[`CSR_TCFG_INITVAL] & csr_tcfg_initval;
		end
	end

	/*
	 * csr_tcfg_next_value: Value of TCFG at next cycle,
	 * WHEN csr_we is 1 AND csr_waddr is `CSR_TCFG.
	 */
	assign csr_tcfg_next_value =  csr_wmask[31:0] & csr_wvalue
							   | ~csr_wmask[31:0] & {csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};

	// timer_ymr
	always @(posedge clk) begin
		if (rst) begin
			/*
			 * timer_ymr being -1 (32'hffffffff) means that either timer
			 * hasn't been initialized or it has decreased to 0 and
			 * Periodic is not enabled. So it should stop decreasing.
			 * Note that the low 2 bits of TCFG::InitVal must be 0, so
			 * InitVal cannot be 32'hffffffff.
			 */
			timer_ymr <= 32'hffffffff;
		end
		else if (csr_we && csr_waddr == `CSR_TCFG &&
			csr_tcfg_next_value[`CSR_TCFG_EN]) begin
			/*
			 * When an instruction is trying to write TCFG and writting
			 * En to 1, load the InitVal to be written to timer_ymr immediately.
			 */
			timer_ymr <= {csr_tcfg_next_value[`CSR_TCFG_INITVAL], 2'd0};
		end
		/*
		 * Once timer_ymr decreases to 0, ESTAT::IS[11] will be set to 1
		 * at the next cycle. It will keep at 1 until an inst writes 1 to
		 * TICLR::CLR.
		 */
		else if (csr_tcfg_en && timer_ymr != 32'hffffffff) begin
			if (timer_ymr == 32'd0 && csr_tcfg_periodic) begin
				timer_ymr <= {csr_tcfg_initval, 2'd0};
			end
			else begin
				timer_ymr <= timer_ymr - 32'd1;
			end
		end
	end

	// TVAL::TimeVal
	assign csr_tval_timeval = timer_ymr[31:0];

	assign rval_crmd	= {23'd0, csr_crmd_datm, csr_crmd_datf, csr_crmd_pg, csr_crmd_da, csr_crmd_ie, csr_crmd_plv};
	assign rval_prmd	= {29'd0, csr_prmd_pie, csr_prmd_pplv};
	assign rval_ecfg	= {19'd0, csr_ecfg_lie};
	assign rval_estat	= {1'd0, csr_estat_esubcode, csr_estat_ecode, 3'd0, csr_estat_is};
	assign rval_era		=  csr_era_pc;
	assign rval_badv	=  csr_badv_vaddr;
	assign rval_eentry	= {csr_eentry_va, 6'd0};
	assign rval_save0	=  csr_save0_data;
	assign rval_save1	=  csr_save1_data;
	assign rval_save2	=  csr_save2_data;
	assign rval_save3	=  csr_save3_data;
	assign rval_tid		=  csr_tid_tid;
	assign rval_tcfg	= {csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};
	assign rval_tval	=  csr_tval_timeval;
	assign rval_ticlr	=  32'd0;

	assign csr_rvalue = {32{csr_raddr == `CSR_CRMD	}} & rval_crmd
					  | {32{csr_raddr == `CSR_PRMD	}} & rval_prmd
					  | {32{csr_raddr == `CSR_ECFG	}} & rval_ecfg
					  | {32{csr_raddr == `CSR_ESTAT	}} & rval_estat
					  | {32{csr_raddr == `CSR_ERA	}} & rval_era
					  | {32{csr_raddr == `CSR_BADV	}} & rval_badv
					  | {32{csr_raddr == `CSR_EENTRY}} & rval_eentry
					  | {32{csr_raddr == `CSR_SAVE0	}} & rval_save0
					  | {32{csr_raddr == `CSR_SAVE1	}} & rval_save1
					  | {32{csr_raddr == `CSR_SAVE2	}} & rval_save2
					  | {32{csr_raddr == `CSR_SAVE3	}} & rval_save3
					  | {32{csr_raddr == `CSR_TID	}} & rval_tid
					  | {32{csr_raddr == `CSR_TCFG	}} & rval_tcfg
					  | {32{csr_raddr == `CSR_TVAL	}} & rval_tval
					  | {32{csr_raddr == `CSR_TICLR	}} & rval_ticlr
	;

	assign ex_entry = {csr_eentry_va, 6'd0};
	assign has_int = ((csr_estat_is[12:0] & csr_ecfg_lie[12:0]) != 13'd0) && (csr_crmd_ie == 1'd1);
	assign era_value = rval_era;

endmodule
