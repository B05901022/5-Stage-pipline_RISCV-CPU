//`include "DECOMPRESSIONUNIT.v"
module cache_read_only(
    clk,
    proc_reset,
    proc_read,
    //proc_write,
    proc_addr,
    proc_rdata,
    //proc_wdata,
    proc_stall,
    proc_pcadd,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);

//==== parameters definition ==============================
    // for FSM state...
    localparam  START       = 2'b00;
    localparam  ALLOCATE    = 2'b01;
    localparam  BUFFER      = 2'b11;

    // for loop
    integer i;

//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read; //, proc_write;
    input   [30:0] proc_addr;
    //input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    output         proc_pcadd;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output  reg    mem_read;
    output         mem_write;
    output  reg [27:0] mem_addr;
    output [127:0] mem_wdata;

//==== wire/reg definition ================================
    //for storage
    reg         valid_w[0:7];
    reg         valid_r[0:7];
    reg  [25:0] tag_w  [0:7];
    reg  [25:0] tag_r  [0:7];
    reg  [15:0] word_w[0:63];
    reg  [15:0] word_r[0:63];
    reg         set_flag_w[0:3]; // when set_flag = 0, first block in the set will be replaced by new data
    reg         set_flag_r[0:3];

    // for FSM
    reg  [1:0]  state_nxt;
    reg  [1:0]  state;

    // for circuit output
    reg         stall;

//==== combinational circuit ==============================

assign proc_stall = stall;
assign mem_write  = 1'b0;
assign mem_wdata  = 128'b0;

//next addr
wire [30:0] next_addr;
assign next_addr = proc_addr + 31'd1;

wire [1:0]  curr_set_idx;
wire [2:0]  curr_idx;
wire [1:0]  next_set_idx;
wire [2:0]  next_idx;

assign curr_set_idx = proc_addr[4:3];
assign next_set_idx = next_addr[4:3];

// for hit or miss of first half word
wire        hit_or_miss_fhw; // 1 for hit, 0 for miss
wire        hit_0_fhw; // hit the first block in the set
wire        hit_1_fhw; // hit the second block in the set
assign hit_0_fhw = ( (proc_addr[30:5] == tag_r[{curr_set_idx, 1'b0}]) && valid_r[{curr_set_idx, 1'b0}] );
assign hit_1_fhw = ( (proc_addr[30:5] == tag_r[{curr_set_idx, 1'b1}]) && valid_r[{curr_set_idx, 1'b1}] );
assign hit_or_miss_fhw   = (hit_0_fhw|hit_1_fhw);

//examine if current instruction is 32-bit or 16-bit
wire curr_32_16; //1 for 32-bit, 0 for 16-bit
assign curr_32_16 = (hit_0_fhw) ? ( &word_r[{curr_set_idx, 1'b0, proc_addr[2:0]}][9:8] ):
                                  ( &word_r[{curr_set_idx, 1'b1, proc_addr[2:0]}][9:8] );

//for hit or miss of second half word
wire        hit_or_miss_shw; // 1 for hit, 0 for miss
wire        hit_0_shw; // hit the first block in the set
wire        hit_1_shw; // hit the second block in the set
assign hit_0_shw = ( (proc_addr[30:5] == tag_r[{next_set_idx, 1'b0}]) && valid_r[{next_set_idx, 1'b0}] );
assign hit_1_shw = ( (proc_addr[30:5] == tag_r[{next_set_idx, 1'b1}]) && valid_r[{next_set_idx, 1'b1}] );
assign hit_or_miss_shw   = (hit_0_shw|hit_1_shw);

//total hit or miss
wire        hit_or_miss;
assign hit_or_miss = (curr_32_16) ? (hit_or_miss_fhw && hit_or_miss_shw) :
                                     hit_or_miss_fhw;

//for reading data
wire [15:0] first_half_word;
wire [15:0] secnd_half_word;
assign first_half_word = (hit_0_fhw) ? word_r[{curr_set_idx, 1'b0, proc_addr[2:0]}] : 
                                       word_r[{curr_set_idx, 1'b1, proc_addr[2:0]}] ;
assign secnd_half_word = (hit_0_shw) ? word_r[{next_set_idx, 1'b0, next_addr[2:0]}] :
                                       word_r[{next_set_idx, 1'b1, next_addr[2:0]}] ; 

//for decompression
wire [15:0] orig_instr;
wire [31:0] decomp_instr;
wire [31:0] little_endian_decomp_instr;                                       
assign orig_instr = {first_half_word[7:0], first_half_word[15:8]};
DecompressionUnit DU1(.orig_instr  (orig_instr), .decomp_instr(decomp_instr));
assign little_endian_decomp_instr = { decomp_instr[ 7: 0], decomp_instr[15: 8],
                                      decomp_instr[23:16], decomp_instr[31:24] };

//for rewriting
assign curr_idx  = (set_flag_r[curr_set_idx]) ? {curr_set_idx,1'b1} : {curr_set_idx,1'b0};
assign next_idx  = (set_flag_r[next_set_idx]) ? {next_set_idx,1'b1} : {next_set_idx,1'b0};

//PC addition
assign proc_pcadd = curr_32_16;

//for fetching instrucion
reg [31:0] rdata;
assign proc_rdata = rdata;

//cross tag error
reg cross_tag_error_r, cross_tag_error_w;

//==== Finite State Machine ===============================
always@( posedge clk or posedge proc_reset) begin
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
                end else begin
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

always@(*) begin
    //==== Default value ==================================
    stall    = 1'b0;
    mem_read = 1'b0;
    rdata    = (curr_32_16) ? {first_half_word, secnd_half_word} :
                               little_endian_decomp_instr;
    mem_addr = (cross_tag_error_r) ? next_addr[30:3] : proc_addr[30:3];
    cross_tag_error_w = cross_tag_error_r;
    for (i=0;i<8;i=i+1) begin
        valid_w[i] = valid_r[i]; 
        tag_w[i]   = tag_r[i]; 
    end
    for (i=0;i<4;i=i+1) begin
        set_flag_w[i] = set_flag_r[i];
    end
    for (i=0;i<64;i=i+1) begin
        word_w[i]  = word_r[i]; 
    end
    //===========================
    case ( state ) 
        START:
            begin
                if (hit_or_miss) begin
                    // hit !!
                    stall = 1'b0;
                    set_flag_w[curr_set_idx] = (hit_0_fhw); //LRU 
                end else begin
                    // miss
                    stall = 1'b1;
                    mem_read  = 1'b1;
                    cross_tag_error_w = ( (curr_32_16 && hit_or_miss_fhw) && (~hit_or_miss_shw) );
                end
            end
        ALLOCATE:
            begin
                stall = 1'b1;
                mem_read =  1'b1;
                if (cross_tag_error_r) begin
                    valid_w[next_idx] = 1'b1;
                    tag_w[next_idx]   = proc_addr[30:5];
                end else begin
                    valid_w[curr_idx] = 1'b1;
                    tag_w[curr_idx]   = proc_addr[30:5]; 
                end
            end
        BUFFER:
            begin
                stall = 1'b1;
                if (cross_tag_error_r) begin
                    {{word_w[{next_idx, 3'b110}]}, {word_w[{next_idx, 3'b111}]},
                     {word_w[{next_idx, 3'b100}]}, {word_w[{next_idx, 3'b101}]},
                     {word_w[{next_idx, 3'b010}]}, {word_w[{next_idx, 3'b011}]},
                     {word_w[{next_idx, 3'b000}]}, {word_w[{next_idx, 3'b001}]}} = mem_rdata;
                     cross_tag_error_w = 1'b0;
                end else begin
                    {{word_w[{curr_idx, 3'b110}]}, {word_w[{curr_idx, 3'b111}]},
                     {word_w[{curr_idx, 3'b100}]}, {word_w[{curr_idx, 3'b101}]},
                     {word_w[{curr_idx, 3'b010}]}, {word_w[{curr_idx, 3'b011}]},
                     {word_w[{curr_idx, 3'b000}]}, {word_w[{curr_idx, 3'b001}]}} = mem_rdata;
                end
            end
    endcase
end
//==== sequential circuit =================================
always@( posedge clk or posedge proc_reset) begin
    if( proc_reset ) begin
        for (i=0;i<8;i=i+1) begin
            valid_r[i] <= 1'b0; // reset valid bit
            tag_r[i]   <= 26'b0; // reset tag
        end
        for (i=0;i<64;i=i+1) begin
            word_r[i]  <= 16'b0; // reset words
        end
        for (i=0;i<4;i=i+1) begin
            set_flag_r[i] = 1'b0;
        end
        cross_tag_error_r <= 1'b0;
    end
    else begin
        for (i=0;i<8;i=i+1) begin
            valid_r[i] <= valid_w[i]; // reset valid bit
            tag_r[i]   <= tag_w[i]; // reset tag
        end
        for (i=0;i<4;i=i+1) begin
            set_flag_r[i] = set_flag_w[i];
        end
        for (i=0;i<64;i=i+1) begin
            word_r[i]  <= word_w[i]; // reset words
        end
        cross_tag_error_r <= cross_tag_error_w;
    end
end

endmodule
