
/*
 * 32-bit multiplier supporting signed and unsigned
 */
module mul (
    input wire mul_clk, input wire reset,
    input wire [31:0] x, input wire [31:0] y,
    input wire mul_signed, output wire [63:0] result
);

    wire [3:0] res_h; /* Unused */

    mult34 u_mult34 (
        .mul_clk(mul_clk), .rst(reset),
        .x({ {2{mul_signed & x[31]}},x[31:0] }),
        .y({ {2{mul_signed & y[31]}},y[31:0] }),
        .p({ res_h,result })
    );

endmodule

/*
 * Multiplier using 2bits-Booth and Wallace tree
 */
module mult34 (
    input wire mul_clk,
    input wire rst,
    input wire [33:0] x,
    input wire [33:0] y,
    output wire [67:0] p
);

    wire [34:0] pp [16:0]; /* partial products */
    wire [16:0] c;

    genvar i;
    generate
        for (i = 1; i < 17; i = i + 1) begin : g_partial_product_gen
            partial_product_gen u_partial_product_gen (
                .x(x), .y(y[2 * i + 1:2 * i - 1]),
                .p(pp[i]), .c(c[i])
            );
        end
    endgenerate
    partial_product_gen u_partial_product_gen_0 (
        .x(x), .y({ y[1],y[0],1'd0 }),
        .p(pp[0]), .c(c[0])
    );

    wire [67:0] ppe [16:0]; /* sign-extended pp */
    generate
        for (i = 1; i < 17; i = i + 1) begin : g_ppe
            assign ppe[i] = { {(33 - 2 * i){pp[i][34]}},pp[i],{(2 * i){c[i]}} };
        end
    endgenerate
    assign ppe[0] = { {33{pp[0][34]}},pp[0] };

    wire [16:0] wt_n [67:0]; /* n of wallace_tree */
    genvar j;
    generate
        for (i = 0; i < 68; i = i + 1) begin : g_wt_n
            for (j = 0; j < 17; j = j + 1) begin
                assign wt_n[i][j] = ppe[j][i];
            end
        end
    endgenerate

    wire [14:0] wt_co [67:0]; /* co of wallace_tree */
    wire [67:0] wt_c, wt_s;
    generate
        for (i = 1; i < 68; i = i + 1) begin : g_wallace_tree
            wallace_tree u_wallace_tree (
                .n(wt_n[i]), .ci(wt_co[i - 1]),
                .co(wt_co[i]), .c(wt_c[i]), .s(wt_s[i])
            );
        end
    endgenerate
    wallace_tree u_wallace_tree_0 (
        .n(wt_n[0]), .ci(c[14:0]),
        .co(wt_co[0]), .c(wt_c[0]), .s(wt_s[0])
    );

/* 2-stage pipeline */
    reg [67:0] add1_reg, add2_reg;
    reg cin_reg;

    always @(posedge mul_clk) begin
        if (rst) begin
            add1_reg <= 68'd0;
            add2_reg <= 68'd0;
            cin_reg <= 0;
        end
        else begin
            add1_reg <= wt_s;
            add2_reg <= { wt_c[66:0],c[15] };
            cin_reg <= c[16];
        end
    end

    wire cout; /* Unused */
    //assign { cout,p } = wt_s + { wt_c[66:0],c[15] } + c[16];
    assign { cout,p } = add1_reg + add2_reg + cin_reg;

endmodule

/*
 * Generate 35-bit partital products
 */
module partial_product_gen (
    input  wire [33:0] x,
    input  wire [ 2:0] y, /* y_i+1, y_i, y_i-1 */
    output wire [34:0] p,
    output wire [ 0:0] c
);

    wire select_mx, select_ax, select_m2x, select_a2x;
    /* m: minus; a: add. */

    assign select_mx = (y == 3'b101 || y == 3'b110),
        select_ax = (y == 3'b001 || y == 3'b010),
        select_m2x = (y == 3'b100), select_a2x = (y == 3'b011);
    
    genvar i;
    generate
        for (i = 1; i < 34; i = i + 1) begin : g_partial_product
            assign p[i] = (
                select_mx & ~x[i] |
                select_ax & x[i] |
                select_m2x & ~x[i - 1] |
                select_a2x & x[i - 1]
            );
        end
    endgenerate

    assign p[0] = select_mx & ~x[0] | select_ax & x[0] |
        select_m2x & ~0 | select_a2x & 0, /* consider x[-1] is 0 */
        p[34] = select_mx & ~x[33] | select_ax & x[33] | /* consider x[34] is x[33] */
            select_m2x & ~x[33] | select_a2x & x[33];
    assign c = select_mx | select_m2x;

endmodule

/*
 * Wallace tree for adding 17 1-bit
 */
module wallace_tree (
    input wire [16:0] n,
    input wire [14:0] ci,
    output wire [14:0] co,
    output wire [0:0] c,
    output wire [0:0] s
);

    wire [14:0] st; /* temp sum */

    /* Level 1 (lowest): 17 -> 12 */
    genvar i;
    generate
        for (i = 0; i < 5; i = i + 1) begin : level1
            assign { co[i],st[i] } = n[3 * i] + n[3 * i + 1] + n[3 * i + 2];
        end
    endgenerate
    assign { co[5],st[5] } = n[15] + n[16]; /* HALF */

    /* Level 2: 12 -> 8 */
    assign { co[6],st[6] } = st[0] + st[1] + st[2],
        { co[7],st[7] } = st[3] + st[4] + st[5],
        { co[8],st[8] } = ci[0] + ci[1] + ci[2],
        { co[9],st[9] } = ci[3] + ci[4] + ci[5];
    
    /* Level 3: 8 -> 6 */
    assign { co[10],st[10] } = st[6] + st[7] + st[8],
        { co[11],st[11] } = st[9] + ci[6] + ci[7];
    
    /* Level 4: 6 -> 4 */
    assign { co[12],st[12] } = st[10] + st[11] + ci[8],
        { co[13],st[13] } = ci[9] + ci[10] + ci[11];
    
    /* Level 5: 4 -> 3 */
    assign { co[14],st[14] } = st[12] + st[13] + ci[12];

    /* Level 6: 3 -> 2 */
    assign { c,s } = st[14] + ci[13] + ci[14];
 endmodule
