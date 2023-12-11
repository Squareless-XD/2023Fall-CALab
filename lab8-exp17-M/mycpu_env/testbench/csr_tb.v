`define CSR_CRMD 9'h00
`define CSR_CRMD_PLV 1:0
`define CSR_CRMD_IE 2

`define CSR_PRMD 9'h01
`define CSR_PRMD_PPLV 1:0
`define CSR_PRMD_PIE 2

`define CSR_ECFG 9'h04
`define CSR_ECFG_LIE 12:0

`define CSR_ESTAT 9'h05
`define csr_rvalue 12:0
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

module csr_test_g();
	reg		clk;
	reg		rst;
	/* Ports for inst access */
	reg [ 8:0]	csr_num;
	/* Number;or Address */
	reg		csr_we;
	/* Write enable */
	reg [31:0]	csr_wmask;
	/* Write mask */
	reg [31:0]	csr_wvalue;
	wire [31:0]	csr_rvalue;
	/* Ports for interacting with CPU hardware */
	reg [31:0] cpuid;
	reg [ 7:0]	hw_int_in;
	/* Hardware interrupt */
	reg		ipi_int_in;
	/* IPI interrupt */
	wire [31:0] ex_entry;
	/* exception entry */
	wire		has_int;
	/* has interrupt */
	reg		ertn_flush;
	/* ERTN inst */
	reg		WB_ex;
	/* exc from WB */
	reg		WB_pc;
	reg		WB_vaddr;
	/* bad vaddr */
	reg [ 5:0]	WB_ecode;
	reg [ 8:0] WB_esubcode;

	csr csr_test_inst(clk, rst, csr_num, csr_num, csr_we, csr_wmask, csr_wvalue, csr_rvalue, cpuid, hw_int_in, ipi_int_in, ex_entry, has_int, ertn_flush, WB_ex, WB_pc, WB_vaddr, WB_ecode, WB_esubcode);

	always #5 clk = ~clk;

	initial begin
		/* reset */
		clk = 0;
		cpuid = 32'd5033;
		hw_int_in = 8'd0;
		ipi_int_in = 0;
		ertn_flush = 0;
		WB_ex = 0;
		rst = 1;
		#20;
		rst = 0;
	
		/* test CRMD */
		csr_num = `CSR_CRMD;
		csr_we = 1;
		csr_wmask = ~32'd0;
		csr_wvalue = ~32'd0;
		#10;

		csr_wvalue = 32'd0;
		#10;

		/* test PRMD */
		csr_num = `CSR_PRMD;
		csr_wmask = ~32'd0;
		csr_wvalue = ~32'd0;
		#10;
		csr_we = 0;

		/* test ERTN */
		ertn_flush = 1;
		csr_num = `CSR_CRMD;
		#10;
		ertn_flush = 0;
		if (csr_rvalue[2:0] == 3'b111)
			$display("[time@%t] PRMD ertn test passed!", $time);
		else begin
			$display("[time@%t] PRMD ertn test failed: CRMD::IE,PLV is %b", $time, csr_rvalue[2:0]);
			$finish;
		end
		
		/* test ESTAT */
		WB_ex = 1;
		csr_num = `CSR_ESTAT;
		WB_esubcode = 9'd50;
		WB_ecode = 6'd33;
		#10;
		WB_ex = 0;
		if (csr_rvalue[`CSR_ESTAT_ESUBCODE] == 9'd50 &&
			csr_rvalue[`CSR_ESTAT_ECODE] == 6'd33)
			$display("[time@%t] ESTAT exc test passed!", $time);
		else begin
			$display("[time@%t] ESTAT exc test failed: ESTAT::Esubcode is %d, Ecode is %d",
			$time, csr_rvalue[`CSR_ESTAT_ESUBCODE], csr_rvalue[`CSR_ESTAT_ECODE]);
			$finish;
		end

		csr_we = 1;
		csr_wmask = ~32'd0;
		csr_wvalue = ~32'd0;
		hw_int_in = 8'd1;
		#10;
		csr_we = 0;
		if (csr_rvalue != { 1'd0,9'd50,6'd33,3'd0,3'd0,8'd1,2'd3 }) begin
			$display("[time@%t] ESTAT test failed!", $time);
			$finish;
		end
		else
			$display("[time@%t] ESTAT test passed!", $time);

		/* test ERA, EENTRY, SAVE0, BADV, TID */
		csr_num = `CSR_TID;
		csr_we = 0;
		#10;
		if (csr_rvalue != 32'd5033) begin
			$display("[time@%t] TID init test failed!", $time);
			$finish;
		end

		csr_we = 1;
		csr_wmask = ~32'd0;
		csr_wvalue = { 16'd5033,16'd1 };
		csr_num = `CSR_ERA;
		#10;
		if (csr_rvalue != {16'd5033,16'd1}) begin
			$display("[time@%t] ERA test failed!", $time);
			$finish;
		end
		else
			$display("[time@%t] ERA test passed!", $time);

		csr_num = `CSR_EENTRY;
		#10;
		if (csr_rvalue != {16'd5033,16'd0}) begin
			$display("[time@%t] EENTRY test failed!", $time);
			$finish;
		end
		else
			$display("[time@%t] EENTRY test passed!", $time);
	
		csr_num = `CSR_SAVE0;
		#10;
		if (csr_rvalue != {16'd5033,16'd1}) begin
			$display("[time@%t] SAVE0 test failed!", $time);
			$finish;
		end
		else
			$display("[time@%t] SAVE0 test passed!", $time);
		
		csr_num = `CSR_BADV;
		#10;
		if (csr_rvalue != {16'd5033,16'd1}) begin
			$display("[time@%t] BADV test failed!", $time);
			$finish;
		end
		else
			$display("[time@%t] BADV test passed!", $time);
		
		csr_num = `CSR_TID;
		#10;
		if (csr_rvalue != {16'd5033,16'd1}) begin
			$display("[time@%t] TID test failed!", $time);
			$finish;
		end
		else
			$display("[time@%t] TID test passed!", $time);

		/* test function of wmask */
		csr_num = `CSR_SAVE1;
		csr_wmask = ~32'd0;
		csr_wvalue = 32'h12345678;
		#10;

		csr_wmask = 32'hf0f0f0f0;
		csr_wvalue = 32'h87654321;
		#10;

		if (csr_rvalue != 32'h82644628) begin
			$display("[time@%t] wmask test failed!", $time);
			$finish;
		end
		else
			$display("[time@%t] wmask test passed!", $time);

		csr_we = 0;
		/* test Timer */
		csr_num = `CSR_TCFG;
		csr_we = 1;
		csr_wmask = { ~32'd0 };
		csr_wvalue = 32'd0;
		#10;
		csr_wmask = { 1'd0,~31'd0 };
		/* Test wmast together */
		csr_wvalue = { 1'd1, 29'd2, 2'd3 };
		#10;
		csr_we = 0;

		csr_num = `CSR_ESTAT;
		#130;
		if (csr_rvalue[11] != 1) begin
			$display("[time@%t] Timer interrupt test failed!", $time);
			$finish;
		end

		csr_num = `CSR_TICLR;
		csr_we = 1;
		csr_wmask = ~32'd0;
		csr_wvalue = 32'd1;
		#9;
		csr_num = `CSR_ESTAT;
		#1;
		csr_we = 0;
		if (csr_rvalue[11] != 0) begin
			$display("[time@%t] TI clear test failed!", $time);
			$finish;
		end

		csr_num = `CSR_ESTAT;
		#130;
		if (csr_rvalue[11] != 1) begin
			$display("[time@%t] Timer interrupt(periodic) test failed!", $time);
			$finish;
		end

		csr_num = `CSR_TICLR;
		csr_we = 1;
		csr_wmask = ~32'd0;
		csr_wvalue = 32'd1;
		#10;
	
		csr_num = `CSR_TCFG;
		csr_we = 1;
		csr_wmask = { 1'd0,~31'd0 };
		/* Test periodic = 0 */
		csr_wvalue = { 1'd1, 29'd2, 2'd1 };
		#10;
		csr_we = 0;

		csr_num = `CSR_ESTAT;
		#130;
		if (csr_rvalue[11] != 1) begin
			$display("[time@%t] Timer interrupt(no periodic) test failed!", $time);
			$finish;
		end
		csr_num = `CSR_TVAL;
		#1;
		if (csr_rvalue != 32'hffffffff) begin
			$display("[time@%t] TVAL test failed!", $time);
			$finish;
		end
		else
			$display("[time@%t] Timer interrupt test passed!", $time);

		#10;
		$display("Test passed!");
		$finish;
	end


	initial begin
		$dumpfile("csr.vcd");
		$dumpvars(0, csr_test_g);
	end
endmodule
