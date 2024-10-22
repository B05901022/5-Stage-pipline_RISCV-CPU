module cache_read_only(
    clk,
    proc_reset,
    proc_read,
    //proc_write,
    proc_addr,
    proc_rdata,
    //proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready,
    proc_pcadd
    //hit_rate
);

//==== parameters definition ==============================
    // for FSM state...
    localparam  START       = 2'b00;
    localparam  ALLOCATE    = 2'b01;
    localparam  BUFFER      = 2'b10;

    // for loop
    integer i;


//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read; //, proc_write;
    input   [29:0] proc_addr;
    //input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output  reg    mem_read, mem_write;
    output  reg [27:0] mem_addr;
    output [127:0] mem_wdata;
    output         proc_pcadd;
    assign proc_pcadd = 1'b1;

    //output hit_rate; //for analysis

//==== wire/reg definition ================================
    //for storage
    reg         valid_w[0:7];
    reg         valid_r[0:7];
    //reg         dirty_w[0:7];
    //reg         dirty_r[0:7];
    reg  [24:0] tag_w  [0:7];
    reg  [24:0] tag_r  [0:7];
    reg  [31:0] word_w[0:31];
    reg  [31:0] word_r[0:31];

    // for FSM
    reg  [1:0]  state_nxt;
    reg  [1:0]  state;

    // hit or miss
    wire     hit_or_miss; // 1 for hit, 0 for miss
    //reg     dirty;

    // for circuit output
    reg [31:0]  rdata;
    reg         stall;
    reg [127:0] wdata;

    // for buffer state
    reg  [127:0]  wdata_buf_w;
    reg  [127:0]  wdata_buf_r;
    //reg  [27:0]   mem_addr_buf_w, mem_addr_buf_r;


//==== Finite State Machine ===============================
always@( posedge clk) begin
    if( proc_reset ) begin
        state <= START;
    end
    else begin
        state <= state_nxt;
    end
end
//==== next state logic =====================================
always@(*) begin
    case ( state )
        START:
            begin 
                if( hit_or_miss  ) begin
                    // hit!!
                    state_nxt = START;
                end else begin
                    state_nxt = ALLOCATE;
                end
            end
        ALLOCATE:
            begin
                if( mem_ready ) begin
                    state_nxt = BUFFER;
                end
                else begin
                    state_nxt = ALLOCATE;
                end
            end
        BUFFER:
            begin
                state_nxt = START;
            end
        default:
            begin
                state_nxt = state;
            end
    endcase
end
//==== combinational circuit ==============================

assign proc_rdata = word_r[proc_addr[4:0]];
assign proc_stall = stall;
assign mem_wdata  = 0;
assign hit_or_miss = ({valid_r[proc_addr[4:2]],tag_r[proc_addr[4:2]]} == {1'b1,proc_addr[29:5]});
//assign hit_rate = hit_or_miss;

always@(*) begin
    //==== Default value ==================================
    //dirty =         1'b0;
    //rdata =         32'b0;
    stall =         1'b0;
    wdata =         128'b0;
    mem_read =      0;
    mem_write =     0;
    mem_addr   = proc_addr[29:2];
    wdata_buf_w = mem_rdata;
    //mem_addr_buf_w = mem_addr_buf_r;
    for (i=0;i<8;i=i+1) begin
        valid_w[i] = valid_r[i]; 
        //dirty_w[i] = dirty_r[i]; 
        tag_w[i]   = tag_r[i]; 
    end
    for (i=0;i<32;i=i+1) begin
        word_w[i]  = word_r[i]; 
    end
    //===========================
    case ( state ) 
        START:
            begin
                if( hit_or_miss) begin
                    // hit
                    stall = 1'b0;
                        
                end else begin
                    // miss
                    stall = 1'b1;
                    mem_read = 1'b1;                 
                end
            end
        ALLOCATE:
            begin
                stall = 1'b1;
                case ( proc_addr[4:2] ) 
                        3'd0: tag_w[0] = proc_addr[29:5];
                        3'd1: tag_w[1] = proc_addr[29:5];
                        3'd2: tag_w[2] = proc_addr[29:5];
                        3'd3: tag_w[3] = proc_addr[29:5];
                        3'd4: tag_w[4] = proc_addr[29:5];
                        3'd5: tag_w[5] = proc_addr[29:5];
                        3'd6: tag_w[6] = proc_addr[29:5];
                        3'd7: tag_w[7] = proc_addr[29:5];
                endcase
                valid_w[proc_addr[4:2]] = 1'b1;
                mem_read =  1'b1;
                mem_write = 1'b0;
            end
        BUFFER:
            begin
                stall = 1'b1;
                {{word_w[{proc_addr[4:2], 2'b11}]}, {word_w[{proc_addr[4:2], 2'b10}]},
                {word_w[{proc_addr[4:2], 2'b01}]}, {word_w[{proc_addr[4:2], 2'b00}]}} = wdata_buf_r;
            end
        default:
            begin
                stall = 1'b0;
                mem_read = 1'b0;
                mem_write = 1'b0;
            end
    endcase
end
//==== sequential circuit =================================
always@( posedge clk or posedge proc_reset) begin
    if( proc_reset ) begin
        for (i=0;i<8;i=i+1) begin
            valid_r[i] <= 1'b0; // reset valid bit
            //dirty_r[i] <= 1'b0; // reset dirty bit
            tag_r[i]   <= 25'b0; // reset tag
        end
        for (i=0;i<32;i=i+1) begin
            word_r[i]  <= 32'b0; // reset words
        end
        wdata_buf_r <= 0;
        //mem_addr_buf_r <=0;
    end
    else begin
        for (i=0;i<8;i=i+1) begin
            valid_r[i] <= valid_w[i]; // reset valid bit
            //dirty_r[i] <= dirty_w[i]; // reset dirty bit
            tag_r[i]   <= tag_w[i]; // reset tag
        end
        for (i=0;i<32;i=i+1) begin
            word_r[i]  <= word_w[i]; // reset words
        end
        wdata_buf_r <= wdata_buf_w;
       // mem_addr_buf_r <= mem_addr_buf_w;
    end
end

endmodule