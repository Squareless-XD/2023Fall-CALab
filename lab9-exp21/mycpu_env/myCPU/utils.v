// module decoder_2_4(
//     input  wire [ 1:0] in,
//     output wire [ 3:0] out
// );

// genvar i;
// generate for (i=0; i<4; i=i+1) begin : gen_for_dec_2_4
//     assign out[i] = (in == i);
// end endgenerate

// endmodule

// module decoder_3_8(
//     input  wire [ 2:0] in,
//     output wire [ 7:0] out
// );

// genvar i;
// generate for (i=0; i<8; i=i+1) begin : gen_for_dec_3_8
//     assign out[i] = (in == i);
// end endgenerate

// endmodule


// module decoder_4_16(
//     input  wire [ 3:0] in,
//     output wire [15:0] out
// );

// genvar i;
// generate for (i=0; i<16; i=i+1) begin : gen_for_dec_4_16
//     assign out[i] = (in == i);
// end endgenerate

// endmodule


// module decoder_5_32(
//     input  wire [ 4:0] in,
//     output wire [31:0] out
// );

// genvar i;
// generate for (i=0; i<32; i=i+1) begin : gen_for_dec_5_32
//     assign out[i] = (in == i);
// end endgenerate

// endmodule


// module decoder_6_64(
//     input  wire [ 5:0] in,
//     output wire [63:0] out
// );

// genvar i;
// generate for (i=0; i<64; i=i+1) begin : gen_for_dec_6_64
//     assign out[i] = (in == i);
// end endgenerate

// endmodule

module decoder_x #(
    parameter width = 3
)(
    input  wire [       width - 1 : 0] in,
    output wire [(1 << width) - 1 : 0] out
);

    parameter o_width = 1 << width;
    genvar i;
    generate
        for (i = 0; i < o_width; i = i + 1) begin : gen_for_dec_x
            assign out[i] = (in == i);
        end
    endgenerate

endmodule

// flipflop with reset value
// Reg #(width, reset_val) inst_name (clk, rst, input, output, wen);
module Reg #(WIDTH = 1, RESET_VAL = 0) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire [WIDTH - 1 : 0] din,
    output reg  [WIDTH - 1 : 0] dout,
    input  wire                 wen
);
    always @(posedge clk) begin
        if (rst) dout <= RESET_VAL;
        else if (wen) dout <= din;
    end
endmodule

// flipflop without any reset value
// Reg_norst #(width) inst_name (clk, input, output, wen);
module Reg_norst #(WIDTH = 1) (
    input  wire                 clk,
    input  wire [WIDTH - 1 : 0] din,
    output reg  [WIDTH - 1 : 0] dout,
    input  wire                 wen
);
    always @(posedge clk) begin
        if (wen) dout <= din;
    end
endmodule

// // 使用触发器模板的示例
// module example(
//     input clk,
//     input rst,
//     input [3:0] in,
//     output [3:0] out
// );
//     // 位宽为1比特, 复位值为1'b1, 写使能一直有效
//     Reg #(1, 1'b1) i0 (clk, rst, in[0], out[0], 1'b1);
//     // 位宽为3比特, 复位值为3'b0, 写使能为out[0]
//     Reg #(3, 3'b0) i1 (clk, rst, in[3:1], out[3:1], out[0]);
// endmodule


// // 选择器模板内部实现
// module MuxKeyInternal #(NR_KEY = 2, KEY_LEN = 1, DATA_LEN = 1, HAS_DEFAULT = 0) (
//     output reg  [DATA_LEN-1:0] out,
//     input  wire [KEY_LEN-1:0] key,
//     input  wire [DATA_LEN-1:0] default_out,
//     input  wire [NR_KEY*(KEY_LEN + DATA_LEN)-1:0] lut
// );

//     localparam PAIR_LEN = KEY_LEN + DATA_LEN;
//     wire [PAIR_LEN-1:0] pair_list [NR_KEY-1:0];
//     wire [KEY_LEN-1:0] key_list [NR_KEY-1:0];
//     wire [DATA_LEN-1:0] data_list [NR_KEY-1:0];

//     generate
//         for (genvar n = 0; n < NR_KEY; n = n + 1) begin
//             assign pair_list[n] = lut[PAIR_LEN*(n+1)-1 : PAIR_LEN*n];
//             assign data_list[n] = pair_list[n][DATA_LEN-1:0];
//             assign key_list[n]  = pair_list[n][PAIR_LEN-1:DATA_LEN];
//         end
//     endgenerate

//     reg [DATA_LEN-1 : 0] lut_out;
//     reg hit;
//     integer i;
//     always @(*) begin
//         lut_out = 0;
//         hit = 0;
//         for (i = 0; i < NR_KEY; i = i + 1) begin
//             lut_out = lut_out | ({DATA_LEN{key == key_list[i]}} & data_list[i]);
//             hit = hit | (key == key_list[i]);
//         end
//         if (!HAS_DEFAULT) out = lut_out;
//         else out = (hit ? lut_out : default_out);
//     end
// endmodule

// // 不带默认值的选择器模板
// module MuxKey #(NR_KEY = 2, KEY_LEN = 1, DATA_LEN = 1) (
//     output [DATA_LEN-1:0] out,
//     input [KEY_LEN-1:0] key,
//     input [NR_KEY*(KEY_LEN + DATA_LEN)-1:0] lut
// );
//     MuxKeyInternal #(NR_KEY, KEY_LEN, DATA_LEN, 0) i0 (out, key, {DATA_LEN{1'b0}}, lut);
// endmodule

// // 带默认值的选择器模板
// module MuxKeyWithDefault #(NR_KEY = 2, KEY_LEN = 1, DATA_LEN = 1) (
//     output [DATA_LEN-1:0] out,
//     input [KEY_LEN-1:0] key,
//     input [DATA_LEN-1:0] default_out,
//     input [NR_KEY*(KEY_LEN + DATA_LEN)-1:0] lut
// );
//     MuxKeyInternal #(NR_KEY, KEY_LEN, DATA_LEN, 1) i0 (out, key, default_out, lut);
// endmodule

