module divider(
	input  wire [ 3:0] div_op,
	input  wire [31:0] div_src1,
	input  wire [31:0] div_src2,
	output wire [31:0] div_result
);

	// mul_op: mul.w, mulh.w, mulh.wu, div.w, mod.w, div.wu, mod.wu

	wire op_div_w;
	wire op_mod_w;
	wire op_div_wu;
	wire op_mod_wu;

	assign op_div_w  = div_op[0];
	assign op_mod_w  = div_op[1];
	assign op_div_wu = div_op[2];
	assign op_mod_wu = div_op[3];

	// 66-bit signed divtiply
	wire signed   [31:0] m_div_res;
	wire unsigned [31:0] m_div_res_u;
	wire signed   [31:0] m_mod_res;
	wire unsigned [31:0] m_mod_res_u;
	assign m_div_res   = $signed(div_src1) / $signed(div_src2);
	assign m_mod_res   = $signed(div_src1) % $signed(div_src2);
	assign m_div_res_u =         div_src1  /         div_src2;
	assign m_mod_res_u =         div_src1  %         div_src2;



	// final result mux
	assign div_result = ({32{op_div_w }} & m_div_res  )
					  | ({32{op_mod_w }} & m_mod_res  )
					  | ({32{op_div_wu}} & m_div_res_u)
					  | ({32{op_mod_wu}} & m_mod_res_u)
	;
endmodule
