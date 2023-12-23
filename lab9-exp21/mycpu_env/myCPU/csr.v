/*
 * csr.v: module for CSR
 */

`define CSR_CRMD 9'h00
`define CSR_CRMD_PLV 1:0
`define CSR_CRMD_IE 2
`define CSR_CRMD_DA 3
`define CSR_CRMD_PG 4
`define CSR_CRMD_DATF 6:5
`define CSR_CRMD_DATM 8:7

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

`define CSR_TLBIDX 9'h10
`define CSR_TLBIDX_NE 31
`define CSR_TLBIDX_PS 29:24
`define CSR_TLBIDX_INDEX `LOG2TLBNUM - 1:0

`define CSR_TLBEHI 9'h11
`define CSR_TLBEHI_VPPN 31:13

`define CSR_TLBELO0 9'h12
`define CSR_TLBELO1 9'h13
`define CSR_TLBELO_PPN 27:8
`define CSR_TLBELO_G 6
`define CSR_TLBELO_MAT 5:4
`define CSR_TLBELO_PLV 3:2
`define CSR_TLBELO_D 1
`define CSR_TLBELO_V 0

`define CSR_ASID 9'h18
`define CSR_ASID_ASIDBITS 23:16
`define CSR_ASID_ASID 9:0

`define CSR_TLBRENTRY 9'h88
`define CSR_TLBRENTRY_PA 31:6

`define CSR_DMW0 9'h180
`define CSR_DMW1 9'h181
`define CSR_DMW_VSEG 31:29
`define CSR_DMW_PSEG 27:25
`define CSR_DMW_MAT 5:4
`define CSR_DMW_PLV3 3
`define CSR_DMW_PLV0 0

`define ECODE_ADE 6'h8
`define ECODE_ALE 6'h9   // Addr not aligned
`define ECODE_PIL 6'h1   // Load page fault
`define ECODE_PIS 6'h2   // Store page fault
`define ECODE_PIF 6'h3   // Inst page fault
`define ECODE_PME 6'h4   // Page modified
`define ECODE_PPL 6'h7   // Page previlege fault
`define ECODE_TLBR 6'h3f // TLB refill


`define ESUBCODE_ADEF 9'h0

`define TLBNUM 16
`define LOG2TLBNUM 4
`define LOG2TLBNUM24 20 // 24-`LOG2TLBNUM

module csr (
    input  wire        clk,
    input  wire        rst,

    // Ports for inst access
    input  wire [ 8:0] csr_waddr,  // write address
    input  wire [ 8:0] csr_raddr,  // read  address
    input  wire        csr_we,     // write enable
    input  wire [31:0] csr_wmask,  // write mask
    input  wire [31:0] csr_wvalue, // value to be written
    output wire [31:0] csr_rvalue, // value read

    // Ports for interacting with CPU hardware
    input  wire [31:0] cpuid,      // CPU ID, just 0 is OK
    input  wire [ 7:0] hw_int_in,  // hardware interrupt
    input  wire        ipi_int_in, // IPI interrupt
    output wire [31:0] ex_entry,   // exception entry
    output wire        has_int,    // has interrupt
    output wire [31:0] era_value,  // exception return address
    input  wire        ertn_flush, // ERTN inst
    input  wire        WB_ex,      // exc from WB
    input  wire [31:0] WB_pc,
    input  wire [31:0] WB_vaddr,   // bad vaddr
    input  wire [ 5:0] WB_ecode,   // exception code
    input  wire [ 8:0] WB_esubcode,// exception subcode
    output wire [31:0] csr_tid,
    output wire [ 5:0] estate_ecode,
    output wire [ 9:0] asid_asid,  // ASID for TLBSRCH
    output wire [18:0] tlbehi_vppn,// VPPN for TLBSRCH, TLBWR, TLBFILL
    output wire [31:0] tlbidx,
    output wire [31:0] tlbelo0,
    output wire [31:0] tlbelo1,
    output wire [31:0] dmw0, dmw1,
    output wire        crmd_da, crmd_pg,
    output wire [ 1:0] crmd_plv,
    output wire [ 1:0] crmd_datf, crmd_datm,
    input  wire        tlbe_we,    // write enable of TLBEHI::VPPN, TLBELO0~1, for TLBRD
    input  wire        asid_asid_we,
    input  wire [18:0] w_tlbehi_vppn,
    input  wire        tlbidx_ne_we,
    input  wire        tlbidx_ps_we,
    input  wire        tlbidx_index_we,
    input  wire [31:0] w_tlbidx,
    input  wire [31:0] w_tlbelo0,
    input  wire [31:0] w_tlbelo1,
    input  wire [ 9:0] w_asid_asid
    /* --------------------------
        waiting for more wires
     -------------------------- */
);

    // 0x00: CRMD
    reg [1:0] csr_crmd_plv;
    reg csr_crmd_ie;
    reg csr_crmd_da, csr_crmd_pg;
    reg [1:0] csr_crmd_datf, csr_crmd_datm;

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
    reg [31:0] csr_save0_data, csr_save1_data, csr_save2_data, csr_save3_data;

    // 0x40: TID
    reg [31:0] csr_tid_tid;

    // 0x41: TCFG
    reg csr_tcfg_en, csr_tcfg_periodic;
    reg [29:0] csr_tcfg_initval;
    wire [31:0] csr_tcfg_next_value;

    // 0x42: TVAL
    wire [31:0] csr_tval_timeval;

    // 0x10: TLBIDX
    reg csr_tlbidx_ne;
    reg [5:0] csr_tlbidx_ps;
    reg [`LOG2TLBNUM - 1:0] csr_tlbidx_index;

    // 0x11: TLBEHI
    reg [18:0] csr_tlbehi_vppn;

    // 0x12~0x13: TLBELO0~1
    reg [19:0] csr_tlbelo0_ppn, csr_tlbelo1_ppn;
    reg csr_tlbelo0_g, csr_tlbelo1_g;
    reg [1:0] csr_tlbelo0_mat, csr_tlbelo1_mat;
    reg [1:0] csr_tlbelo0_plv, csr_tlbelo1_plv;
    reg csr_tlbelo0_d, csr_tlbelo1_d;
    reg csr_tlbelo0_v, csr_tlbelo1_v;

    // 0x18: ASID
    wire [7:0] csr_asid_asidbits;
    reg [9:0] csr_asid_asid;

    // 0x88: TLBRENTRY
    reg [25:0] csr_tlbrentry_pa;

    // 0x180~0x181: DMW0~1
    reg [2:0] csr_dmw0_vseg, csr_dmw1_vseg;
    reg [2:0] csr_dmw0_pseg, csr_dmw1_pseg;
    reg [1:0] csr_dmw0_mat, csr_dmw1_mat;
    reg csr_dmw0_plv3, csr_dmw1_plv3;
    reg csr_dmw0_plv0, csr_dmw1_plv0;

    // Timer counter
    reg [31:0] timer_ymr;

    /*
     * Flag of exception caused by error address
     * from WB stage.
     */
    wire flag_WB_ex_addr_err;

    // Flag MMU exception
    wire flag_ex_MMU;

    // Value read from CSR.
    wire [31:0] rval_crmd, rval_prmd, rval_ecfg, rval_estat, rval_era,
                rval_badv, rval_eentry, rval_save0, rval_save1, rval_save2,
                rval_save3, rval_tid, rval_tcfg, rval_tval, rval_ticlr,
                rval_tlbidx, rval_tlbehi, rval_tlbelo0, rval_tlbelo1,
                rval_asid, rval_tlbrentry, rval_dmw0, rval_dmw1;

    // CRMD::PLV, IE, DA, PG, DATF, DATM
    always @(posedge clk) begin
        if (rst) begin
            csr_crmd_plv <= 2'd0;
            csr_crmd_ie  <= 1'd0;
            csr_crmd_da  <= 1'd1;
            csr_crmd_pg  <= 1'd0;
            csr_crmd_datf <= 2'd0;
            csr_crmd_datm <= 2'd0;
        end
        else if (WB_ex) begin
            csr_crmd_plv <= 2'd0;
            csr_crmd_ie  <= 1'd0;
            if (WB_ecode == 6'h3f) begin
                /*
                 * Enter TLB refill exception:
                 * enable direct address mode.
                 */
                csr_crmd_da <= 1'd1;
                csr_crmd_pg <= 1'd0;
            end
        end
        else if (ertn_flush) begin
            csr_crmd_plv <= csr_prmd_pplv;
            csr_crmd_ie  <= csr_prmd_pie;
            if (csr_estat_ecode == 6'h3f) begin
                /*
                 * Return from TLB refill exception:
                 * enable page mapping mode.
                 */
                csr_crmd_da <= 1'd0;
                csr_crmd_pg <= 1'd1;
            end
        end
        else if (csr_we && csr_waddr == `CSR_CRMD) begin
            csr_crmd_plv <= csr_wmask[`CSR_CRMD_PLV] & csr_wvalue[`CSR_CRMD_PLV]
                         | ~csr_wmask[`CSR_CRMD_PLV] & csr_crmd_plv;
            csr_crmd_ie  <= csr_wmask[`CSR_CRMD_IE] & csr_wvalue[`CSR_CRMD_IE]
                         | ~csr_wmask[`CSR_CRMD_IE] & csr_crmd_ie;
            csr_crmd_da  <= csr_wmask[`CSR_CRMD_DA] & csr_wvalue[`CSR_CRMD_DA]
                         | ~csr_wmask[`CSR_CRMD_DA] & csr_crmd_da;
            csr_crmd_pg  <= csr_wmask[`CSR_CRMD_PG] & csr_wvalue[`CSR_CRMD_PG]
                         | ~csr_wmask[`CSR_CRMD_PG] & csr_crmd_pg;
            csr_crmd_datf <= csr_wmask[`CSR_CRMD_DATF] & csr_wvalue[`CSR_CRMD_DATF]
                         | ~csr_wmask[`CSR_CRMD_DATF] & csr_crmd_datf;
            csr_crmd_datm <= csr_wmask[`CSR_CRMD_DATM] & csr_wvalue[`CSR_CRMD_DATM]
                         | ~csr_wmask[`CSR_CRMD_DATM] & csr_crmd_datm;
        end
    end

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

        if (rst) begin
        /*
         * Value of ESTAT::Ecode is not defined when reset.
         * However, as TLBRD instruction will read it,
         * reset it to 0 to avoid undefined value.
         */
            csr_estat_ecode    <= 6'd0;
            csr_estat_esubcode <= 9'd0;
        end
        else if (WB_ex) begin
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

    assign flag_ex_MMU = WB_ex && (
        WB_ecode == `ECODE_PIL || WB_ecode == `ECODE_PIS ||
        WB_ecode == `ECODE_PIF || WB_ecode == `ECODE_PME ||
        WB_ecode == `ECODE_PPL || WB_ecode == `ECODE_TLBR
    );

    // BADV::VAddr
    assign flag_WB_ex_addr_err = WB_ex && (
        WB_ecode == `ECODE_ADE || WB_ecode == `ECODE_ALE || flag_ex_MMU
    );
    always @(posedge clk) begin
        if (flag_WB_ex_addr_err) begin
            csr_badv_vaddr <= //(WB_ecode == `ECODE_ADE && WB_esubcode == `ESUBCODE_ADEF)
                            //? WB_pc
                            //: WB_vaddr;
                            WB_vaddr; // It has been decided in WB stage
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

    // TLBIDX::NE, PS, Index
    always @(posedge clk) begin
        if (tlbidx_ne_we)
            csr_tlbidx_ne <= w_tlbidx[`CSR_TLBIDX_NE];
        else if (csr_we && csr_waddr == `CSR_TLBIDX)
            csr_tlbidx_ne <= csr_wmask[`CSR_TLBIDX_NE] & csr_wvalue[`CSR_TLBIDX_NE]
                          | ~csr_wmask[`CSR_TLBIDX_NE] & csr_tlbidx_ne;
        
        if (tlbidx_ps_we)
            csr_tlbidx_ps <= w_tlbidx[`CSR_TLBIDX_PS];
        else if (csr_we && csr_waddr == `CSR_TLBIDX)
            csr_tlbidx_ps <= csr_wmask[`CSR_TLBIDX_PS] & csr_wvalue[`CSR_TLBIDX_PS]
                          | ~csr_wmask[`CSR_TLBIDX_PS] & csr_tlbidx_ps;

        if (tlbidx_index_we)
            csr_tlbidx_index <= w_tlbidx[`CSR_TLBIDX_INDEX];
        else if (csr_we && csr_waddr == `CSR_TLBIDX)
            csr_tlbidx_index <= csr_wmask[`CSR_TLBIDX_INDEX] & csr_wvalue[`CSR_TLBIDX_INDEX]
                             | ~csr_wmask[`CSR_TLBIDX_INDEX] & csr_tlbidx_index;
    end

    // TLBEHI::VPPN
    always @(posedge clk) begin
        if (flag_ex_MMU)
            csr_tlbehi_vppn <= WB_vaddr[31:13];
        else if (tlbe_we)
            csr_tlbehi_vppn <= w_tlbehi_vppn;
        else if (csr_we && csr_waddr == `CSR_TLBEHI)
            csr_tlbehi_vppn <= csr_wmask[`CSR_TLBEHI_VPPN] & csr_wvalue[`CSR_TLBEHI_VPPN]
                            | ~csr_wmask[`CSR_TLBEHI_VPPN] & csr_tlbehi_vppn;
    end

    // TLBELO0~1::PPN, G, MAT, PLV, D, V
    always @(posedge clk) begin
        if (tlbe_we) begin
            csr_tlbelo0_ppn <= w_tlbelo0[`CSR_TLBELO_PPN];
            csr_tlbelo0_g   <= w_tlbelo0[`CSR_TLBELO_G];
            csr_tlbelo0_mat <= w_tlbelo0[`CSR_TLBELO_MAT];
            csr_tlbelo0_plv <= w_tlbelo0[`CSR_TLBELO_PLV];
            csr_tlbelo0_d   <= w_tlbelo0[`CSR_TLBELO_D];
            csr_tlbelo0_v   <= w_tlbelo0[`CSR_TLBELO_V];
            csr_tlbelo1_ppn <= w_tlbelo1[`CSR_TLBELO_PPN];
            csr_tlbelo1_g   <= w_tlbelo1[`CSR_TLBELO_G];
            csr_tlbelo1_mat <= w_tlbelo1[`CSR_TLBELO_MAT];
            csr_tlbelo1_plv <= w_tlbelo1[`CSR_TLBELO_PLV];
            csr_tlbelo1_d   <= w_tlbelo1[`CSR_TLBELO_D];
            csr_tlbelo1_v   <= w_tlbelo1[`CSR_TLBELO_V];
        end
        else if (csr_we && csr_waddr == `CSR_TLBELO0) begin
            csr_tlbelo0_ppn <= csr_wmask[`CSR_TLBELO_PPN] & csr_wvalue[`CSR_TLBELO_PPN]
                            | ~csr_wmask[`CSR_TLBELO_PPN] & csr_tlbelo0_ppn;
            csr_tlbelo0_g   <= csr_wmask[`CSR_TLBELO_G] & csr_wvalue[`CSR_TLBELO_G]
                            | ~csr_wmask[`CSR_TLBELO_G] & csr_tlbelo0_g;
            csr_tlbelo0_mat <= csr_wmask[`CSR_TLBELO_MAT] & csr_wvalue[`CSR_TLBELO_MAT]
                            | ~csr_wmask[`CSR_TLBELO_MAT] & csr_tlbelo0_mat;
            csr_tlbelo0_plv <= csr_wmask[`CSR_TLBELO_PLV] & csr_wvalue[`CSR_TLBELO_PLV]
                            | ~csr_wmask[`CSR_TLBELO_PLV] & csr_tlbelo0_plv;
            csr_tlbelo0_d   <= csr_wmask[`CSR_TLBELO_D] & csr_wvalue[`CSR_TLBELO_D]
                            | ~csr_wmask[`CSR_TLBELO_D] & csr_tlbelo0_d;
            csr_tlbelo0_v <= csr_wmask[`CSR_TLBELO_V] & csr_wvalue[`CSR_TLBELO_V]
                            | ~csr_wmask[`CSR_TLBELO_V] & csr_tlbelo0_v;
        end
        else if (csr_we && csr_waddr == `CSR_TLBELO1) begin
            csr_tlbelo1_ppn <= csr_wmask[`CSR_TLBELO_PPN] & csr_wvalue[`CSR_TLBELO_PPN]
                            | ~csr_wmask[`CSR_TLBELO_PPN] & csr_tlbelo1_ppn;
            csr_tlbelo1_g   <= csr_wmask[`CSR_TLBELO_G] & csr_wvalue[`CSR_TLBELO_G]
                            | ~csr_wmask[`CSR_TLBELO_G] & csr_tlbelo1_g;
            csr_tlbelo1_mat <= csr_wmask[`CSR_TLBELO_MAT] & csr_wvalue[`CSR_TLBELO_MAT]
                            | ~csr_wmask[`CSR_TLBELO_MAT] & csr_tlbelo1_mat;
            csr_tlbelo1_plv <= csr_wmask[`CSR_TLBELO_PLV] & csr_wvalue[`CSR_TLBELO_PLV]
                            | ~csr_wmask[`CSR_TLBELO_PLV] & csr_tlbelo1_plv;
            csr_tlbelo1_d   <= csr_wmask[`CSR_TLBELO_D] & csr_wvalue[`CSR_TLBELO_D]
                            | ~csr_wmask[`CSR_TLBELO_D] & csr_tlbelo1_d;
            csr_tlbelo1_v <= csr_wmask[`CSR_TLBELO_V] & csr_wvalue[`CSR_TLBELO_V]
                            | ~csr_wmask[`CSR_TLBELO_V] & csr_tlbelo1_v;
        end
    end

    // ASID::ASIDBITS
    assign csr_asid_asidbits = 8'd10;
    // ASID::ASID
    always @(posedge clk) begin
        if (asid_asid_we)
            csr_asid_asid <= w_asid_asid;
        else if (csr_we && csr_waddr == `CSR_ASID)
            csr_asid_asid <= csr_wmask[`CSR_ASID_ASID] & csr_wvalue[`CSR_ASID_ASID]
                          | ~csr_wmask[`CSR_ASID_ASID] & csr_asid_asid;
    end

    // TLBRENTRY::PA
    always @(posedge clk) begin
        if (csr_we && csr_waddr == `CSR_TLBRENTRY)
            csr_tlbrentry_pa <= csr_wmask[`CSR_TLBRENTRY_PA] & csr_wvalue[`CSR_TLBRENTRY_PA]
                             | ~csr_wmask[`CSR_TLBRENTRY_PA] & csr_tlbrentry_pa;
    end

    // DMW0~1:: VSEG, PSEG, MAT, PLV3, PLV0
    always @(posedge clk) begin
        if (rst) begin
            csr_dmw0_plv0 <= 1'b0;
            csr_dmw1_plv0 <= 1'b0;
            csr_dmw0_plv3 <= 1'b0;
            csr_dmw1_plv3 <= 1'b0;
        end
        else if (csr_we && csr_waddr == `CSR_DMW0) begin
            csr_dmw0_vseg <= csr_wmask[`CSR_DMW_VSEG] & csr_wvalue[`CSR_DMW_VSEG]
                          | ~csr_wmask[`CSR_DMW_VSEG] & csr_dmw0_vseg;
            csr_dmw0_pseg <= csr_wmask[`CSR_DMW_PSEG] & csr_wvalue[`CSR_DMW_PSEG]
                          | ~csr_wmask[`CSR_DMW_PSEG] & csr_dmw0_pseg;
            csr_dmw0_mat  <= csr_wmask[`CSR_DMW_MAT] & csr_wvalue[`CSR_DMW_MAT]
                          | ~csr_wmask[`CSR_DMW_MAT] & csr_dmw0_mat;
            csr_dmw0_plv3 <= csr_wmask[`CSR_DMW_PLV3] & csr_wvalue[`CSR_DMW_PLV3]
                          | ~csr_wmask[`CSR_DMW_PLV3] & csr_dmw0_plv3;
            csr_dmw0_plv0 <= csr_wmask[`CSR_DMW_PLV0] & csr_wvalue[`CSR_DMW_PLV0]
                          | ~csr_wmask[`CSR_DMW_PLV0] & csr_dmw0_plv0;
        end
        else if (csr_we && csr_waddr == `CSR_DMW1) begin
            csr_dmw1_vseg <= csr_wmask[`CSR_DMW_VSEG] & csr_wvalue[`CSR_DMW_VSEG]
                          | ~csr_wmask[`CSR_DMW_VSEG] & csr_dmw1_vseg;
            csr_dmw1_pseg <= csr_wmask[`CSR_DMW_PSEG] & csr_wvalue[`CSR_DMW_PSEG]
                          | ~csr_wmask[`CSR_DMW_PSEG] & csr_dmw1_pseg;
            csr_dmw1_mat  <= csr_wmask[`CSR_DMW_MAT] & csr_wvalue[`CSR_DMW_MAT]
                          | ~csr_wmask[`CSR_DMW_MAT] & csr_dmw1_mat;
            csr_dmw1_plv3 <= csr_wmask[`CSR_DMW_PLV3] & csr_wvalue[`CSR_DMW_PLV3]
                          | ~csr_wmask[`CSR_DMW_PLV3] & csr_dmw1_plv3;
            csr_dmw1_plv0 <= csr_wmask[`CSR_DMW_PLV0] & csr_wvalue[`CSR_DMW_PLV0]
                          | ~csr_wmask[`CSR_DMW_PLV0] & csr_dmw1_plv0;
        end
    end

    assign rval_crmd      = {23'd0, csr_crmd_datm, csr_crmd_datf, csr_crmd_pg, csr_crmd_da, csr_crmd_ie, csr_crmd_plv};
    assign rval_prmd      = {29'd0, csr_prmd_pie, csr_prmd_pplv};
    assign rval_ecfg      = {19'd0, csr_ecfg_lie};
    assign rval_estat     = {1'd0, csr_estat_esubcode, csr_estat_ecode, 3'd0, csr_estat_is};
    assign rval_era       =  csr_era_pc;
    assign rval_badv      =  csr_badv_vaddr;
    assign rval_eentry    = {csr_eentry_va, 6'd0};
    assign rval_save0     =  csr_save0_data;
    assign rval_save1     =  csr_save1_data;
    assign rval_save2     =  csr_save2_data;
    assign rval_save3     =  csr_save3_data;
    assign rval_tid       =  csr_tid_tid;
    assign rval_tcfg      = {csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};
    assign rval_tval      =  csr_tval_timeval;
    assign rval_ticlr     =  32'd0;
    assign rval_tlbidx    = {csr_tlbidx_ne, 1'd0, csr_tlbidx_ps, `LOG2TLBNUM24'd0, csr_tlbidx_index};
    assign rval_tlbehi    = {csr_tlbehi_vppn, 13'd0};
    assign rval_tlbelo0   = {4'd0, csr_tlbelo0_ppn, 1'd0, csr_tlbelo0_g, csr_tlbelo0_mat, csr_tlbelo0_plv, csr_tlbelo0_d, csr_tlbelo0_v};
    assign rval_tlbelo1   = {4'd0, csr_tlbelo1_ppn, 1'd0, csr_tlbelo1_g, csr_tlbelo1_mat, csr_tlbelo1_plv, csr_tlbelo1_d, csr_tlbelo1_v};
    assign rval_asid      = {8'd0, csr_asid_asidbits, 6'd0, csr_asid_asid};
    assign rval_tlbrentry = {csr_tlbrentry_pa, 6'd0};
    assign rval_dmw0      = {csr_dmw0_vseg, 1'd0, csr_dmw0_pseg, 19'd0, csr_dmw0_mat, csr_dmw0_plv3, 2'd0, csr_dmw0_plv0};
    assign rval_dmw1      = {csr_dmw1_vseg, 1'd0, csr_dmw1_pseg, 19'd0, csr_dmw1_mat, csr_dmw1_plv3, 2'd0, csr_dmw1_plv0};

    assign csr_rvalue = {32{csr_raddr == `CSR_CRMD   }} & rval_crmd
                      | {32{csr_raddr == `CSR_PRMD   }} & rval_prmd
                      | {32{csr_raddr == `CSR_ECFG   }} & rval_ecfg
                      | {32{csr_raddr == `CSR_ESTAT  }} & rval_estat
                      | {32{csr_raddr == `CSR_ERA    }} & rval_era
                      | {32{csr_raddr == `CSR_BADV   }} & rval_badv
                      | {32{csr_raddr == `CSR_EENTRY }} & rval_eentry
                      | {32{csr_raddr == `CSR_SAVE0  }} & rval_save0
                      | {32{csr_raddr == `CSR_SAVE1  }} & rval_save1
                      | {32{csr_raddr == `CSR_SAVE2  }} & rval_save2
                      | {32{csr_raddr == `CSR_SAVE3  }} & rval_save3
                      | {32{csr_raddr == `CSR_TID    }} & rval_tid
                      | {32{csr_raddr == `CSR_TCFG   }} & rval_tcfg
                      | {32{csr_raddr == `CSR_TVAL   }} & rval_tval
                      | {32{csr_raddr == `CSR_TICLR  }} & rval_ticlr
                      | {32{csr_raddr == `CSR_TLBIDX }} & rval_tlbidx
                      | {32{csr_raddr == `CSR_TLBEHI }} & rval_tlbehi
                      | {32{csr_raddr ==`CSR_TLBELO0 }} & rval_tlbelo0
                      | {32{csr_raddr ==`CSR_TLBELO1 }} & rval_tlbelo1
                      | {32{csr_raddr == `CSR_ASID   }} & rval_asid
                      | {32{csr_raddr==`CSR_TLBRENTRY}} & rval_tlbrentry
                      | {32{csr_raddr == `CSR_DMW0   }} & rval_dmw0
                      | {32{csr_raddr == `CSR_DMW1   }} & rval_dmw1
    ;

    assign ex_entry = {WB_ex && WB_ecode == `ECODE_TLBR ? csr_tlbrentry_pa : csr_eentry_va, 6'd0};
    assign has_int = ((csr_estat_is[12:0] & csr_ecfg_lie[12:0]) != 13'd0) && (csr_crmd_ie == 1'd1);
    assign estate_ecode = csr_estat_ecode;
    assign era_value = rval_era;
    assign asid_asid = csr_asid_asid;
    assign tlbehi_vppn = csr_tlbehi_vppn;
    assign tlbidx = rval_tlbidx;
    assign tlbelo0 = rval_tlbelo0;
    assign tlbelo1 = rval_tlbelo1;
    assign dmw0 = rval_dmw0;
    assign dmw1 = rval_dmw1;
    assign crmd_da = csr_crmd_da;
    assign crmd_pg = csr_crmd_pg;
    assign crmd_plv = csr_crmd_plv;
    assign crmd_datf = csr_crmd_datf;
    assign crmd_datm = csr_crmd_datm;
endmodule
