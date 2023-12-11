/* module: cache
   arch:   looongarch-32
   author: der
 */

`define WIDTH           32
`define BYTE_NUM        4
`define TAG_WIDTH       20
`define INDEX_WIDTH     8
`define OFFSET_WIDTH    4
`define LINE_WIDTH      128
`define WAY_NUM         2
`define BANK_NUM        4
`define TYPE_CACHE_LINE 3'b110
`define VALID           1'b1
`define DIRTY           1'b1
`define BLANK_LINE      256'b0
`define FULL_STRB       4'hf

module cache(
    input           clk,
    input           resetn,

    // CACHE <=--=> CPU
    input           valid,        // REQ valid
    input           op,           // 1 - write;  0 - read
    input   [ 7:0]  index,        // ADDR[11:4]
    input   [19:0]  tag,          // gen(paddr) 
    input   [ 3:0]  offset,       // ADDR[3:0]
    input   [ 3:0]  wstrb,        // write strobe(en)
    input   [31:0]  wdata,        // write DATA      
    output          addr_ok,      // ADDR trans ok - read: ADDR recved; write: ADDR recved & data recved 
    output          data_ok,      // DATA trans ok - read: DATA returned; write: DATA written
    output  [31:0]  rdata,        // read DATA

    // CACHE <=--=> AXI
    output          rd_req,       // read REQ valid
    output  [ 2:0]  rd_type,      // 3'b000-b; 3'b001-h; 3'b010-w; 3'b100-cache line
    output  [31:0]  rd_addr,      // read begin ADDR
    input           rd_rdy,       // can recv read REQ (for handshake)
    input           ret_valid,    // read ret valid
    input           ret_last,     // read ret last
    input   [31:0]  ret_data,     // read ret DATA

    output          wr_req,       // write REQ valid
    output  [ 2:0]  wr_type,      // 3'b000-b; 3'b001-h; 3'b010-w; 3'b100-cache line
    output  [31:0]  wr_addr,      // write begin ADDR
    output  [ 3:0]  wr_wstrb,     // write strobe(en)
    output  [127:0] wr_data,      // write DATA
    input           wr_rdy        // can recv write REQ (for handshake); wr_rdy MUST be ahead of wr_req
);

/*************************************
 *********** Decalaration ************
 *************************************/

// main SM
parameter   IDLE 		= 5'b00001;
parameter   LOOKUP 	    = 5'b00010;
parameter   MISS 		= 5'b00100;
parameter   REPLACE 	= 5'b01000;
parameter   REFILL 	    = 5'b10000;
reg [4:0]   current_state;
reg [4:0]   next_state;

// write buf SM
parameter   WRBUF_IDLE  = 2'b01;
parameter   WRBUF_WRITE = 2'b10;
reg [1:0]   current_state_wrbuf;
reg [1:0]   next_state_wrbuf;

// CACHE_TOP <=--=> RAM - TAGV
wire                    tagv_we     [`WAY_NUM-1:0];
wire [`INDEX_WIDTH-1:0] tagv_addr   [`WAY_NUM-1:0];
wire [    `TAG_WIDTH:0] tagv_rdata  [`WAY_NUM-1:0];     // NO '-1', for V
wire [    `TAG_WIDTH:0] tagv_wdata  [`WAY_NUM-1:0];

// CACHE_TOP <=--=> RAM - DATA
wire                    data_we     [`WAY_NUM-1:0][`BANK_NUM-1:0];
wire [`INDEX_WIDTH-1:0] data_addr   [`WAY_NUM-1:0][`BANK_NUM-1:0];
wire [      `WIDTH-1:0] data_rdata  [`WAY_NUM-1:0][`BANK_NUM-1:0];
wire [      `WIDTH-1:0] data_wdata  [`WAY_NUM-1:0][`BANK_NUM-1:0];
wire [      `WIDTH-1:0] data_wdata_strbed;

// Dirty 
reg  [ `LINE_WIDTH-1:0] dirty       [`WAY_NUM-1:0];
wire [`INDEX_WIDTH-1:0] dirty_index;

// cache READ data
wire                    read_valid  [`WAY_NUM-1:0];
wire [  `TAG_WIDTH-1:0] read_tag    [`WAY_NUM-1:0];
wire [ `LINE_WIDTH-1:0] read_rdata  [`WAY_NUM-1:0];

// REQ reg
reg                     req_op_r;
reg  [`INDEX_WIDTH-1:0] req_index_r;
reg  [  `TAG_WIDTH-1:0] req_tag_r;
reg [`OFFSET_WIDTH-1:0] req_offset_r;
reg  [      `WIDTH-1:0] req_wdata_r;
reg  [   `BYTE_NUM-1:0] req_wstrb_r;

// WR reg
reg                     wr_way_r;
reg  [             1:0] wr_bank_r;
reg  [`INDEX_WIDTH-1:0] wr_index_r;
reg  [  `TAG_WIDTH-1:0] wr_tag_r;
reg [`OFFSET_WIDTH-1:0] wr_offset_r;
reg  [        `WIDTH:0] wr_wdata_r;
reg  [     `BYTE_NUM:0] wr_wstrb_r;
wire                    wr_writing;

// Tag cmp
wire [             1:0] bank;
wire [             1:0] req_bank_r;
wire                    cache_hit;
wire                    hit_way;
wire [    `WAY_NUM-1:0] hit;
wire [    `WAY_NUM-1:0] wr_hit;
wire                    hit_write;
wire                    hit_write_hazard;
wire                    index_offset_remain;

// LOAD data
wire [      `WIDTH-1:0] load_word   [`WAY_NUM-1:0];
wire [      `WIDTH-1:0] load_res;
wire                    ret_end;

// miss(replace)
wire                    replace_way; 
reg  [             1:0] ret_cnt;
wire                    need_replace;

// for interface
reg                     wr_req_r;
wire                    rst;

// LFSR
reg  [             3:0] lfsr;

// genvar
genvar i, j;

// rst
assign rst = ~resetn;


/**************************************
 ************** Main FSM **************
 **************************************/

// Block 1 - init
always @ (posedge clk) begin
    if (rst) begin
        current_state <= IDLE;
    end else begin
        current_state <= next_state;
    end
end

// Block 2 - state transition logic
always @ (*) begin
    case(current_state)
        IDLE:
            if(valid & ~hit_write_hazard) begin
                next_state = LOOKUP; 
            end       
            else begin
                next_state = IDLE;
            end
        LOOKUP:
            if((~valid | hit_write_hazard) & cache_hit) begin
                next_state = IDLE;
            end
            else if (valid & cache_hit) begin
                next_state = LOOKUP;
            end
            // It seems to be essential (this trans does not exist in book)
            else if (need_replace) begin
                next_state = REPLACE;
            end
            else begin   // i.e. ~cache_hit
                next_state = MISS;
            end
        MISS:
            if(wr_rdy) begin
                next_state = REPLACE;
            end
            else begin
                next_state = MISS;
            end
        REPLACE:
            if(rd_rdy) begin
                next_state = REFILL;
            end
            else  begin
                next_state = REPLACE;
            end
        REFILL:
            if(ret_end) begin
                next_state = IDLE;
            end else begin
                next_state = REFILL;
            end
        default:
            next_state = IDLE;
    endcase
end


/**************************************
 ************* WRBUF FSM **************
 **************************************/

// Block 1 - init
always @ (posedge clk) begin
    if (rst) begin
        current_state_wrbuf <= WRBUF_IDLE;
    end else begin
        current_state_wrbuf <= next_state_wrbuf;
    end
end

// Block 2 - state transition logic
always @ (*) begin
    case(current_state_wrbuf)
        WRBUF_IDLE:
            // main FSM in LOOKUP and Store hit 
            if(current_state == LOOKUP & hit_write) begin
                next_state_wrbuf = WRBUF_WRITE;
            end else begin
                next_state_wrbuf = WRBUF_IDLE;
            end
        WRBUF_WRITE:
            if(hit_write) begin
                next_state_wrbuf = WRBUF_WRITE;
            end else begin
                next_state_wrbuf = WRBUF_IDLE;
            end
        default:
            next_state_wrbuf = WRBUF_IDLE;
    endcase
end


/**************************************
 ************ TAGV CONNECT ************
 **************************************/

assign tagv_we[0]    = ret_end & ~replace_way;
assign tagv_we[1]    = ret_end & replace_way;
assign tagv_wdata[0] = {req_tag_r, `VALID};
assign tagv_wdata[1] = {req_tag_r, `VALID};
assign tagv_addr[0]  = (current_state == IDLE || current_state == LOOKUP) ? index 
                     : req_index_r
;
assign tagv_addr[1]  = (current_state == IDLE || current_state == LOOKUP) ? index 
                     : req_index_r
;
generate 
    for(i = 0; i < `WAY_NUM; i = i + 1) begin : TAGV_CONNET_WAY
        TAGV_RAM tagv_ram_inst(
            .clka   (clk            ),
            .wea    (tagv_we[i]     ),
            .addra  (tagv_addr[i]   ),
            .dina   (tagv_wdata[i]  ),
            .douta  (tagv_rdata[i]  )
        );
    end
endgenerate
generate 
    for(i = 0; i < `WAY_NUM; i = i + 1) begin
        assign read_valid[i] = tagv_rdata[i][0];
        assign read_tag[i]   = tagv_rdata[i][`TAG_WIDTH:1];
    end
endgenerate


/**************************************
 ************ DATA CONNECT ************
 **************************************/

 generate
    for(j = 0; j < `WAY_NUM; j = j + 1) begin
        for(i = 0; i < `BANK_NUM; i = i + 1) begin
            assign data_wdata[j][i] = (wr_writing) ? wr_wdata_r :
                                      ((req_bank_r != i) | ~req_op_r) ? ret_data :
                                      data_wdata_strbed
            ;
            assign data_addr[j][i]  = (current_state == IDLE) | (current_state == LOOKUP) ? index : 
                                      req_index_r
            ;
        end
    end 
endgenerate
generate
    for(i = 0; i < `BANK_NUM; i = i + 1) begin
        assign data_we[0][i] = {4{(wr_writing) & (wr_bank_r == i) & ~wr_way_r}} & wr_wstrb_r
                             | {4{ret_valid & ret_cnt == i & ~replace_way}}     & 4'hf
        ;
        assign data_we[1][i] = {4{(wr_writing) & (wr_bank_r == i) & wr_way_r}}  & wr_wstrb_r
                             | {4{ret_valid & ret_cnt == i & replace_way}}      & 4'hf
        ;
    end
endgenerate
generate
    for(j = 0; j < `WAY_NUM; j = j + 1) begin : DATA_CONNET_WAY
        for(i = 0; i < `BANK_NUM; i = i + 1) begin : DATA_CONNECT_BANK
            DATA_RAM data_ram_inst(
                .clka   (clk                ),
                .wea    (data_we[j][i]      ),
                .addra  (data_addr[j][i]    ),
                .dina   (data_wdata[j][i]   ),
                .douta  (data_rdata[j][i]   )
            );
        end
    end
endgenerate
generate
    for(i = 0; i < `WAY_NUM; i = i + 1) begin
        assign read_rdata[i] = {data_rdata[i][3], data_rdata[i][2], 
                                data_rdata[i][1], data_rdata[i][0]}
        ;
    end
endgenerate
assign data_wdata_strbed = {wr_wstrb_r[3] ? wr_wdata_r[31:24] : ret_data[31:24],
                            wr_wstrb_r[2] ? wr_wdata_r[23:16] : ret_data[23:16],
                            wr_wstrb_r[1] ? wr_wdata_r[15: 8] : ret_data[15: 8],
                            wr_wstrb_r[0] ? wr_wdata_r[ 7: 0] : ret_data[ 7: 0]}
;
assign load_res= data_rdata[hit_way][req_bank_r];


/**************************************
 *************** DIRTY ****************
 **************************************/

always @ (posedge clk) begin
    if(rst) begin
        dirty[0] <= `BLANK_LINE;
        dirty[1] <= `BLANK_LINE;
    end else if (wr_writing) begin
        dirty[wr_way_r][wr_index_r] <= `DIRTY;
    end else if (ret_end) begin 
        dirty[replace_way][req_index_r] <= req_op_r;
    end
end


/**************************************
 **************** LSFR ****************
 **************************************/

always @ (posedge clk) begin
    if(rst) begin
        lfsr <= 4'b1111;
    end
    else if(ret_end) begin
        lfsr <= {lfsr[2:0], lfsr[0] ^ lfsr[1]};
    end
end


/**************************************
 *********** TAG CMP LOGIC ************
 **************************************/

assign hit[0]               = read_valid[0] && (read_tag[0] == req_tag_r);
assign hit[1]               = read_valid[1] && (read_tag[1] == req_tag_r);
assign cache_hit            = hit[0] || hit[1];
assign hit_way              = hit[0] ? 0 : 1;
assign hit_write            = (current_state == LOOKUP) && cache_hit && req_op_r;
assign hit_write_hazard     = valid & ~op & 
                            (((current_state == LOOKUP) & hit_write & index_offset_remain)
                            | ((wr_writing) & (bank == req_bank_r)))
;
assign index_offset_remain  = {index, offset} == {req_index_r, req_offset_r};


/**************************************
 ************** REQ BUF ***************
 **************************************/

always @ (posedge clk)
begin
    if(rst) begin
        req_index_r  <= 0;
        req_offset_r <= 0;
        req_op_r     <= 0;
        req_tag_r    <= 0;
        req_wdata_r  <= 0;
        req_wstrb_r  <= 0;
    end
    else if(next_state == LOOKUP) begin
        req_index_r   <= index;
        req_offset_r  <= offset;
        req_op_r      <= op;
        req_tag_r     <= tag;
        req_wdata_r   <= wdata;
        req_wstrb_r   <= wstrb;
    end
end
assign bank       = offset[3:2];
assign req_bank_r = req_offset_r[3:2];


/**************************************
 ************** WR BUF ****************
 **************************************/

always @ (posedge clk)
begin
    if(rst) begin
        wr_way_r    <= 0;
        wr_bank_r   <= 0;
        wr_index_r  <= 0;
        wr_tag_r    <= 0;
        wr_wdata_r  <= 0;
        wr_wstrb_r  <= 0;
        wr_offset_r <= 0;
    end
    else if(hit_write) begin
        wr_tag_r    <= req_tag_r;
        wr_way_r    <= hit_way;
        wr_bank_r   <= req_bank_r;
        wr_index_r  <= req_index_r;
        wr_wstrb_r  <= req_wstrb_r;
        wr_wdata_r  <= req_wdata_r;
        wr_offset_r <= req_offset_r;
    end
end

always @ (posedge clk) begin
    if (rst) begin
        wr_req_r <= 1'b0;
    end 
    else if(current_state == MISS && next_state == REPLACE) begin
        wr_req_r <=1'b1;
    end
    else if(wr_rdy) begin
        wr_req_r <= 1'b0;
    end
end

assign wr_writing = current_state_wrbuf == WRBUF_WRITE;


/**************************************
 ********** MISS(REPLACE) *************
 **************************************/

always @ (posedge clk) begin
    if(rst) begin
        ret_cnt <= 0;
    end else if (ret_valid & ~ret_last) begin
        ret_cnt <= ret_cnt + 1;
    end else if (ret_end) begin
        ret_cnt <= 0;
    end
end
assign replace_way  = lfsr[0];
assign need_replace = ~dirty[replace_way][req_index_r] | ~read_valid[replace_way];


/**************************************
 ************** INTERFACE *************
 **************************************/

// READ
assign rd_type  = `TYPE_CACHE_LINE;
assign rd_addr  = {req_tag_r, req_index_r, req_offset_r};
assign rd_req   = (current_state == REPLACE);
assign rdata    = ret_valid ? ret_data : load_res;

// WRITE
assign wr_type  = `TYPE_CACHE_LINE;
assign wr_addr  = {read_tag[replace_way], req_index_r, req_offset_r};
assign wr_req   = wr_req_r;
assign wr_wstrb = `FULL_STRB;
assign wr_data  = {data_rdata[replace_way][3],
                   data_rdata[replace_way][2],
                   data_rdata[replace_way][1],
                   data_rdata[replace_way][0]}
;

// OK
assign addr_ok = (current_state == IDLE) 
               | (current_state == LOOKUP & valid & cache_hit & 
                  (op | (~op & ~hit_write_hazard)))
;
assign data_ok = (current_state == LOOKUP & (cache_hit | req_op_r)) 
               | (current_state == REFILL & ~req_op_r & ret_valid & (ret_cnt == req_bank_r))
;

// Interface combination
assign ret_end = ret_valid & ret_last;


/**************************************
 ************** RESERVED **************
 **************************************/

endmodule