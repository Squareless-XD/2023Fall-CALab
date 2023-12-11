`define LARGE_PAGE_SIZE 6'd22
`define SMALL_PAGE_SIZE 6'd12

module tlb
#(
    parameter TLBNUM = 16
)
(
    input   wire            clk,

    // search port 0 (for fetch)
    input   wire [ 18:0]    s0_vppn,
    input   wire            s0_va_bit12,
    input   wire [  9:0]    s0_asid,
    output  wire            s0_found,
    output  wire [$clog2(TLBNUM) - 1:0] s0_index,
    output  wire [ 19:0]    s0_ppn,
    output  wire [  5:0]    s0_ps,
    output  wire [  1:0]    s0_plv,
    output  wire [  1:0]    s0_mat,
    output  wire            s0_d,
    output  wire            s0_v,

    // search port 1 (for load/store)
    input   wire [ 18:0]    s1_vppn,
    input   wire            s1_va_bit12,
    input   wire [  9:0]    s1_asid,
    output  wire            s1_found,
    output  wire [$clog2(TLBNUM) - 1:0] s1_index,
    output  wire [ 19:0]    s1_ppn,
    output  wire [  5:0]    s1_ps,
    output  wire [  1:0]    s1_plv,
    output  wire [  1:0]    s1_mat,
    output  wire            s1_d,
    output  wire            s1_v,

    // invtlb opcode
    input   wire            invtlb_valid,
    input   wire [ 4:0]     invtlb_op,

    // write port
    input   wire            we, //w(rite) e(nable)
    input   wire [$clog2(TLBNUM) - 1:0] w_index,
    input   wire            w_e,
    input   wire [ 18:0]    w_vppn,
    input   wire [  5:0]    w_ps,
    input   wire [  9:0]    w_asid,
    input   wire w_g,
    input   wire [ 19:0]    w_ppn0,
    input   wire [  1:0]    w_plv0,
    input   wire [  1:0]    w_mat0,
    input   wire            w_d0,
    input   wire            w_v0,
    input   wire [ 19:0]    w_ppn1,
    input   wire [  1:0]    w_plv1,
    input   wire [  1:0]    w_mat1,
    input   wire            w_d1,
    input   wire            w_v1,

    // read port
    input   wire [$clog2(TLBNUM) - 1:0] r_index,
    output  wire            r_e,
    output  wire [ 18:0]    r_vppn,
    output  wire [  5:0]    r_ps,
    output  wire [  9:0]    r_asid,
    output  wire            r_g,
    output  wire [ 19:0]    r_ppn0,
    output  wire [  1:0]    r_plv0,
    output  wire [  1:0]    r_mat0,
    output  wire            r_d0,
    output  wire            r_v0,
    output  wire [ 19:0]    r_ppn1,
    output  wire [  1:0]    r_plv1,
    output  wire [  1:0]    r_mat1,
    output  wire            r_d1,
    output  wire            r_v1
    );

reg  [TLBNUM - 1:0]  tlb_e;
reg  [TLBNUM - 1:0]  tlb_ps4MB; //pagesize 1:4MB, 0:4KB
reg  [ 18:0]         tlb_vppn [TLBNUM - 1:0];
reg  [  9:0]         tlb_asid [TLBNUM - 1:0];
reg                  tlb_g    [TLBNUM - 1:0];

reg  [ 19:0]         tlb_ppn0 [TLBNUM - 1:0];
reg  [  1:0]         tlb_plv0 [TLBNUM - 1:0];
reg  [  1:0]         tlb_mat0 [TLBNUM - 1:0];
reg                  tlb_d0   [TLBNUM - 1:0];
reg                  tlb_v0   [TLBNUM - 1:0];

reg  [ 19:0]         tlb_ppn1 [TLBNUM - 1:0];
reg  [  1:0]         tlb_plv1 [TLBNUM - 1:0];
reg  [  1:0]         tlb_mat1 [TLBNUM - 1:0];
reg                  tlb_d1   [TLBNUM - 1:0];
reg                  tlb_v1   [TLBNUM - 1:0];

wire [TLBNUM - 1:0]  match0;   // match for search port 0
wire [TLBNUM - 1:0]  match1;   // match for search port 1

wire page_sel0;
wire page_sel1;                // for double-page num

wire [TLBNUM - 1:0]  cond1;    // G == 0
wire [TLBNUM - 1:0]  cond2;    // G == 1
wire [TLBNUM - 1:0]  cond3;    // ASID match
wire [TLBNUM - 1:0]  cond4;    // VPPN PS match

wire [TLBNUM - 1:0]  invtlb_cond [ 31:0];  
// [4:0] op => up to 32 ops, must set op 7:31 16'b0

/********** SEARCH *********/
genvar i;
generate
    for(i = 0; i < TLBNUM; i = i + 1) begin
        assign match0[i] = (s0_vppn[18:10] == tlb_vppn[i][18:10])
                        && (tlb_ps4MB[i] || s0_vppn[9:0] == tlb_vppn[i][9:0])
                        && ((s0_asid == tlb_asid[i]) || tlb_g[i]);
        assign match1[i] = (s1_vppn[18:10] == tlb_vppn[i][18:10])
                        && (tlb_ps4MB[i] || s1_vppn[9:0] == tlb_vppn[i][9:0])
                        && ((s1_asid == tlb_asid[i]) || tlb_g[i]);
    end
endgenerate

assign s0_found = (|match0);
assign s1_found = (|match1);

assign s0_index =   match0[ 1] ? 4'd1  :
                    match0[ 2] ? 4'd2  :
                    match0[ 3] ? 4'd3  :
                    match0[ 4] ? 4'd4  :
                    match0[ 5] ? 4'd5  :
                    match0[ 6] ? 4'd6  :
                    match0[ 7] ? 4'd7  :
                    match0[ 8] ? 4'd8  :
                    match0[ 9] ? 4'd9  :
                    match0[10] ? 4'd10 :
                    match0[11] ? 4'd11 :
                    match0[12] ? 4'd12 :
                    match0[13] ? 4'd13 :
                    match0[14] ? 4'd14 :
                    match0[15] ? 4'd15 :
                    4'd0;

assign s1_index =   match1[ 1] ? 4'd1  :
                    match1[ 2] ? 4'd2  :
                    match1[ 3] ? 4'd3  :
                    match1[ 4] ? 4'd4  :
                    match1[ 5] ? 4'd5  :
                    match1[ 6] ? 4'd6  :
                    match1[ 7] ? 4'd7  :
                    match1[ 8] ? 4'd8  :
                    match1[ 9] ? 4'd9  :
                    match1[10] ? 4'd10 :
                    match1[11] ? 4'd11 :
                    match1[12] ? 4'd12 :
                    match1[13] ? 4'd13 :
                    match1[14] ? 4'd14 :
                    match1[15] ? 4'd15 :
                    4'd0;

assign page_sel0    = tlb_ps4MB[s0_index] ? s0_vppn[9] : s0_va_bit12;
assign s0_ps        = tlb_ps4MB[s0_index] ? `LARGE_PAGE_SIZE : `SMALL_PAGE_SIZE;
assign s0_ppn       = page_sel0 ? tlb_ppn1[s0_index] : tlb_ppn0[s0_index];
assign s0_plv       = page_sel0 ? tlb_plv1[s0_index] : tlb_plv0[s0_index];
assign s0_mat       = page_sel0 ? tlb_mat1[s0_index] : tlb_mat0[s0_index];
assign s0_d         = page_sel0 ? tlb_d1  [s0_index] : tlb_d0  [s0_index];
assign s0_v         = page_sel0 ? tlb_v1  [s0_index] : tlb_v0  [s0_index];


assign page_sel1    = tlb_ps4MB[s1_index] ? s1_vppn[9] : s1_va_bit12;
assign s1_ps        = tlb_ps4MB[s1_index] ? `LARGE_PAGE_SIZE : `SMALL_PAGE_SIZE;
assign s1_ppn       = page_sel1 ? tlb_ppn1[s1_index] : tlb_ppn0[s1_index];
assign s1_plv       = page_sel1 ? tlb_plv1[s1_index] : tlb_plv0[s1_index];
assign s1_mat       = page_sel1 ? tlb_mat1[s1_index] : tlb_mat0[s1_index];
assign s1_d         = page_sel1 ? tlb_d1  [s1_index] : tlb_d0  [s1_index];
assign s1_v         = page_sel1 ? tlb_v1  [s1_index] : tlb_v0  [s1_index];

/********** READ ***********/
assign r_e    = tlb_e    [r_index];
assign r_vppn = tlb_vppn [r_index];
assign r_ps   = tlb_ps4MB[r_index] ? `LARGE_PAGE_SIZE : `SMALL_PAGE_SIZE;
assign r_asid = tlb_asid [r_index];
assign r_g    = tlb_g    [r_index];

assign r_ppn0 = tlb_ppn0 [r_index];
assign r_plv0 = tlb_plv0 [r_index];
assign r_mat0 = tlb_mat0 [r_index];
assign r_d0   = tlb_d0   [r_index];
assign r_v0   = tlb_v0   [r_index];

assign r_ppn1 = tlb_ppn1 [r_index];
assign r_plv1 = tlb_plv1 [r_index];
assign r_mat1 = tlb_mat1 [r_index];
assign r_d1   = tlb_d1   [r_index];
assign r_v1   = tlb_v1   [r_index];

/************ WRITE ***********/
always @ (posedge clk) begin
    if(we) begin
        tlb_e    [w_index] <= w_e;
        tlb_ps4MB[w_index] <= (w_ps == `LARGE_PAGE_SIZE);

        tlb_vppn [w_index] <= w_vppn;
        tlb_asid [w_index] <= w_asid;
        tlb_g    [w_index] <= w_g;

        tlb_ppn0 [w_index] <= w_ppn0;
        tlb_plv0 [w_index] <= w_plv0;
        tlb_mat0 [w_index] <= w_mat0;
        tlb_d0   [w_index] <= w_d0;
        tlb_v0   [w_index] <= w_v0;

        tlb_ppn1 [w_index] <= w_ppn1;
        tlb_plv1 [w_index] <= w_plv1;
        tlb_mat1 [w_index] <= w_mat1;
        tlb_d1   [w_index] <= w_d1;
        tlb_v1   [w_index] <= w_v1;
    end
    else if (invtlb_valid)
        tlb_e    <= ~invtlb_cond[invtlb_op];
    end

/************* INVTLB *************/
generate
    for(i = 0; i < TLBNUM; i = i + 1) begin
       assign cond1[i] = ~tlb_g[i];
       assign cond2[i] = tlb_g[i];
       assign cond3[i] = (s1_asid == tlb_asid[i]);
       assign cond4[i] = (s1_vppn[18:10] == tlb_vppn[i][18:10]) 
                      && (tlb_ps4MB[i] || (s1_vppn[9:0] == tlb_vppn[i][9:0]));
    end
endgenerate

// cond 1 - need to invalidate
assign invtlb_cond[0] = 16'hffff;   // all 
assign invtlb_cond[1] = 16'hffff;   // same as op == 0
assign invtlb_cond[2] = cond2;      // G == 1
assign invtlb_cond[3] = cond1;      // G == 0
assign invtlb_cond[4] = cond1 & cond3;          // G == 0 && ASID match
assign invtlb_cond[5] = cond1 & cond3 & cond4;  // G == 0 && ASID match && VA match
assign invtlb_cond[6] = (cond1 | cond3) & cond4;  // (G == 1 || ASID match) && VA match

// set useless op to 16'b0
generate
    for (i = 7; i < 32; i = i + 1) begin
        assign invtlb_cond[i] = 16'b0;
    end
endgenerate

// for exec invtlb
// tlbe: 0 - invalid; 1 - valid
/*
 * This `always` block is moved to "Write"
 * to avoid multiple drivers on tlb_e.
 *

always @ (posedge clk) begin
    if(!we && invtlb_valid)
        tlb_e <= ~invtlb_cond[invtlb_op]; 
end
*/

endmodule
