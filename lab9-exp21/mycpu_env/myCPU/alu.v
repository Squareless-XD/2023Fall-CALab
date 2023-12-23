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

module alu (
    input  wire [12:0] alu_op,
    input  wire [31:0] alu_src1,
    input  wire [31:0] alu_src2,
    output wire [31:0] alu_result
);

    wire op_add;   // add operation
    wire op_sub;   // sub operation
    wire op_slt;   // signed compared and set less than
    wire op_sltu;  // unsigned compared and set less than
    wire op_and;   // bitwise and
    wire op_nor;   // bitwise nor
    wire op_or;    // bitwise or
    wire op_xor;   // bitwise xor
    wire op_sll;   // logic left shift
    wire op_srl;   // logic right shift
    wire op_sra;   // arithmetic right shift
    wire op_lui;   // Load Upper Immediate
    wire op_eq;    // equal

    // control code decomposition
    assign op_add  = alu_op[`ADD ];
    assign op_sub  = alu_op[`SUB ];
    assign op_slt  = alu_op[`SLT ];
    assign op_sltu = alu_op[`SLTU];
    assign op_and  = alu_op[`AND ];
    assign op_nor  = alu_op[`NOR ];
    assign op_or   = alu_op[`OR  ];
    assign op_xor  = alu_op[`XOR ];
    assign op_sll  = alu_op[`SLL ];
    assign op_srl  = alu_op[`SRL ];
    assign op_sra  = alu_op[`SRA ];
    assign op_lui  = alu_op[`LUI ];
    assign op_eq   = alu_op[`EQ  ];

    wire [31:0] add_sub_result;
    wire [31:0] slt_result;
    wire [31:0] sltu_result;
    wire [31:0] and_result;
    wire [31:0] nor_result;
    wire [31:0] or_result;
    wire [31:0] xor_result;
    wire [31:0] lui_result;
    wire [31:0] sll_result;
    wire [63:0] sr64_result;
    wire [31:0] sr_result;
    wire [31:0] eq_result;


    // 32-bit adder
    wire [31:0] adder_a;
    wire [31:0] adder_b;
    wire        adder_cin;
    wire [31:0] adder_result;
    wire        adder_cout;

    assign adder_a   = alu_src1;
    assign adder_b   = (op_sub | op_slt | op_sltu) ? ~alu_src2 : alu_src2;  //src1 - src2 rj-rk
    assign adder_cin = (op_sub | op_slt | op_sltu) ? 1'b1      : 1'b0;
    assign {adder_cout, adder_result} = adder_a + adder_b + adder_cin;

    // ADD, SUB result
    assign add_sub_result = adder_result;

    // SLT result
    assign slt_result[31:1] = 31'b0;   //rj < rk 1
    assign slt_result[0]    = (alu_src1[31] & ~alu_src2[31])
                            | ((alu_src1[31] ~^ alu_src2[31]) & adder_result[31]);

    // SLTU result
    assign sltu_result[31:1] = 31'b0;
    assign sltu_result[0]    = ~adder_cout;

    // bitwise operation
    assign and_result = alu_src1 & alu_src2;
    assign or_result  = alu_src1 | alu_src2;
    assign nor_result = ~or_result;
    assign xor_result = alu_src1 ^ alu_src2;
    assign lui_result = alu_src2;

    // SLL result
    assign sll_result = alu_src1 << alu_src2[4:0];   //rj << i5

    // SRL, SRA result
    assign sr64_result = {{32{op_sra & alu_src1[31]}}, alu_src1[31:0]} >> alu_src2[4:0]; //rj >> i5
    assign sr_result   = sr64_result[31:0];

    assign eq_result   = (alu_src1 == alu_src2) ? 1'b1 : 1'b0;

    // final result mux
    assign alu_result = ({32{op_add|op_sub}} & add_sub_result)
                      | ({32{op_slt       }} & slt_result)
                      | ({32{op_sltu      }} & sltu_result)
                      | ({32{op_and       }} & and_result)
                      | ({32{op_nor       }} & nor_result)
                      | ({32{op_or        }} & or_result)
                      | ({32{op_xor       }} & xor_result)
                      | ({32{op_lui       }} & lui_result)
                      | ({32{op_sll       }} & sll_result)
                      | ({32{op_srl|op_sra}} & sr_result)
                      | ({32{op_eq        }} & eq_result)
    ;

endmodule
