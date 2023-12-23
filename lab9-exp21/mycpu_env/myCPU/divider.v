module div (
    input wire div_clk, input wire reset,
    input wire div, input wire div_signed,
    input wire [31:0] x, /* Dividend */
    input wire [31:0] y, /* Divisor */
    output reg [31:0] s, /* Quotient */
    output reg [31:0] r, /* Remainder */
    output wire complete
);

    wire rst, div_en;

    assign rst = reset, div_en = div;
    
    localparam S_RST = 3'b001,
        S_RUN = 3'b010, S_CMP = 3'b100;
    
    reg [2:0] current_state, next_state;
    wire flag_prepare, flag_running, flag_rlast;

    reg [63:0] x_r; /* Abs of dividend */
    reg [32:0] y_r; /* Abs of divisor */
    reg [31:0] q_r; /* Abs of quotient */
    reg sign_q, sign_r; /* Sign of quotient & remainder */
    reg [5:0] do_ymr; /* Counter of steps */
    wire [32:0] y_n; /* -y_r */
    wire [32:0] diff;
    /*
     * At the last step of RUNNING, q_r[0] being 0
     * means that an additional y_r has been subtracted,
     * so the remainder should be recovered by
     * adding y_r[31:0] to x_r[63:32].
     */
    wire [31:0] r_recovered;
    wire qn; /* New bit of quotient */

    always @(posedge div_clk) begin
        if (rst)
            current_state <= S_RST;
        else
            current_state <= next_state;
    end

    always @(*) begin
        case (current_state)
        S_RST: /* Reset */
            if (div_en != 0)
                next_state = S_RUN;
            else
                next_state = S_RST;
        S_RUN: /* Running */
            if (do_ymr == 6'd0)
                next_state = S_CMP;
            else
                next_state = S_RUN;
        default: /* Complete */
            if (div_en != 0)
                next_state = S_RUN;
            else
                next_state = S_CMP;
        endcase
    end

    assign flag_prepare = 
        (current_state != S_RUN && next_state == S_RUN),
        flag_running = (current_state == S_RUN),
        /* The last step of RUNNING */
        flag_rlast = (do_ymr == 6'd1),
        /* RUNNING has done, next state is COMPLETE */
        flag_done = (do_ymr == 6'd0);

    /* do_ymr */
    always @(posedge div_clk) begin
        if (rst)
            do_ymr <= 6'd0;
        else if (flag_prepare)
            /* Prepare to divide */
            do_ymr <= 6'd32;
        else if (flag_running)
            do_ymr <= do_ymr - 6'd1;
    end

    /* sign_q/r */
    always @(posedge div_clk) begin
        if (rst) begin
            sign_q <= 0;
            sign_r <= 0;
        end
        else if (flag_prepare) begin
            if (div_signed) begin
                sign_q <= x[31] ^ y[31];
                /* Sign of remainder is the same as dividend */
                sign_r <= x[31];
            end
            else begin
                /* unsigned */
                sign_q <= 0;
                sign_r <= 0;
            end
        end
    end

    assign y_n = ~y_r + 33'd1,
        /* q_r[0]==0 means that last new bit of quotient is 0, so do add */
        diff = x_r[63:31] + (q_r[0] ? y_n : y_r),
        /* 1 if diff is pos, 0 if diff is neg */
        qn = (diff[32] ? 0 : 1);

    /* x_r */
    always @(posedge div_clk) begin
        if (rst)
            x_r <= 64'd0;
        else if (flag_prepare)
            /* Prepare to divide */
            x_r <= {
                32'd0,
                (div_signed && x[31] ? ~x + 32'd1 : x)
            };
        else if (flag_running)
            x_r <= { diff[31:0],x_r[30:0],1'd0 };
    end

    assign r_recovered = (q_r[0] ? x_r[63:32] : x_r[63:32] + y_r[31:0]);

    /* y_r */
    always @(posedge div_clk) begin
        if (rst)
            y_r <= 33'd0;
        else if (flag_prepare)
            y_r <= {
                1'd0,
                (div_signed && y[31] ? ~y + 32'd1 : y)
            };
    end

    /* q_r */
    always @(posedge div_clk) begin
        if (rst)
            q_r <= 32'd0;
        else if (flag_prepare)
            q_r <= ~32'd0;
        else if (flag_running)
            q_r <= { q_r[30:0],qn };
    end

    /* s/r */
    always @(posedge div_clk) begin
        if (rst) begin
            s <= 32'd0;
            r <= 32'd0;
        end
        else if (flag_done) begin
            s <= (sign_q ? ~q_r + 32'd1 : q_r);
            r <= (sign_r ? ~r_recovered + 32'd1 : r_recovered);
        end
    end

    assign complete = (current_state == S_CMP);
endmodule