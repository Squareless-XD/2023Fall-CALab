`timescale 10 ns / 1 ns

`define RND32 ({$random} % (33'd1 << 32))
`define RND2 ({$random} % 4)
`define RND34 (`RND32 + (`RND2 << 32))

module mult34_test_g();
	reg [33:0] x;
	reg [33:0] y;
	wire [67:0] p;

	mult34 mult34_test_inst(x, y, p);

	integer i;

	initial begin
		for (i = 0; i < 20; i = i + 1) begin
			x = `RND34;
			y = `RND34;
			#10;
		end
	end

	initial begin
		$dumpfile("mult34.vcd");
		$dumpvars(0, mult34_test_g);
	end
endmodule
