`include "DECOMPRESSIONUNIT.v"
//read-only cache
module cache_ro(
    clk,
    proc_reset,
    //proc_read, //always 1'b1
    //proc_write, //always 1'b0
    proc_addr,
    proc_rdata,
    //proc_wdata, //No need to write data
    proc_stall,
    mem_read,
    //mem_write, //No need to write back
    mem_addr,
    mem_rdata,
    //mem_wdata, //No need to write back
    mem_ready
	);

	//==== parameters definition ==============================
    // for FSM state...
    /*
    localparam  START       = 2'b00;
    localparam  ALLOCATE    = 2'b01;
    localparam  WRITE_BACK  = 2'b10;
    */
    localparam START    = 1'b0;
    localparam ALLOCATE = 1'b1;

    // for loop
    integer i;

	//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input   [29:0] proc_addr;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output  reg    mem_read; 
    output  reg [27:0] mem_addr;

    //Abandoned due to read-only
    // --- processor interface ---
    //input          proc_read, proc_write;
    //input   [31:0] proc_wdata;
    // --- memory interface ---
    //output reg     mem_write;
    //output [127:0] mem_wdata;

	//==== wire/reg definition ================================
    //for storage
    reg         valid_w[0:7];
    reg         valid_r[0:7];
    reg  [24:0] tag_w  [0:7];
    reg  [24:0] tag_r  [0:7];
    reg  [31:0] word_w[0:31];
    reg  [31:0] word_r[0:31];
    //reg [15:0] word_w[0:63]; 
    //reg [15:0] word_r[0:63]; 

    // for FSM
    reg  state_nxt; 
    reg  state; 

    // hit or miss
    reg     hit_or_miss; // 1 for hit, 0 for miss

    // for circuit output
    reg [31:0]  rdata;
    reg         stall;

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
	                if( hit_or_miss ) begin
	                    // hit!!
	                    state_nxt = START;
	                end
	                else begin
	                    state_nxt = ALLOCATE;
	                end
	            end
	        ALLOCATE:
	            begin
	                if( mem_ready ) begin
	                    state_nxt = START;
	                end
	                else begin
	                    state_nxt = ALLOCATE;
	                end
	            end
	    endcase
	end

	//==== combinational circuit ==============================

	assign proc_rdata = rdata;
	assign proc_stall = stall;

	always@(*) begin
	    //==== Default value ==================================
	    rdata          = 32'b0;
	    stall          = 1'b0;
	    mem_read       = 0;
	    mem_addr       = proc_addr[29:2];
	    hit_or_miss    = 1'b0;
	    for (i=0;i<8;i=i+1) begin
	        valid_w[i] = valid_r[i]; 
	        tag_w[i]   = tag_r[i]; 
	    end
	    for (i=0;i<32;i=i+1) begin
	        word_w[i]  = word_r[i]; 
	    end
	    //===========================
	    case ( state ) 
	        START:
	            begin
	                if( valid_r[proc_addr[4:2]] && ( tag_r[proc_addr[4:2]] == proc_addr[29:5] ) ) begin
	                    // hit
	                    stall = 1'b0;
	                    hit_or_miss = 1'b1;
	                    rdata = word_r[proc_addr[4:0]]; //always read
	                end
	                else begin
	                    // miss
	                    stall = 1'b1;
	                end
	            end
	        ALLOCATE:
	            begin
	                stall = 1'b1;
	                if ( mem_ready ) begin
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
	                    {{word_w[{proc_addr[4:2], 2'b11}]}, {word_w[{proc_addr[4:2], 2'b10}]},
	                     {word_w[{proc_addr[4:2], 2'b01}]}, {word_w[{proc_addr[4:2], 2'b00}]}} = mem_rdata;
	                end
	                else begin
	                    mem_addr = proc_addr[29:2];
	                    mem_read =  1'b1;
	                end
	            end
	    endcase
	end
	//==== sequential circuit =================================
	always@( posedge clk ) begin
	    if( proc_reset ) begin
	        for (i=0;i<8;i=i+1) begin
	            valid_r[i] <= 1'b0; // reset valid bit
	            tag_r[i]   <= 25'b0; // reset tag
	        end
	        for (i=0;i<32;i=i+1) begin
	            word_r[i]  <= 32'b0; // reset words
	        end
	    end
	    else begin
	        for (i=0;i<8;i=i+1) begin
	            valid_r[i] <= valid_w[i]; // reset valid bit
	            tag_r[i]   <= tag_w[i]; // reset tag
	        end
	        for (i=0;i<32;i=i+1) begin
	            word_r[i]  <= word_w[i]; // reset words
	        end
	    end
	end
endmodule

//read-only cache for compression instructions
module cache_comp(
    clk,
    proc_reset,
    proc_addr,
    proc_rdata,
    proc_stall,
    mem_read,
    mem_addr,
    mem_rdata,
    mem_ready,
    pc_add
	);

	//==== parameters definition ==============================
    // for FSM state...
    localparam START    = 1'b0;
    localparam ALLOCATE = 1'b1;

    // for loop
    integer i;

	//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input   [30:0] proc_addr;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output  reg    mem_read; 
    output  reg [27:0] mem_addr;

    // for 16-bit instruction
    output  reg pc_add; //1 means add 4; 0 means add 2;

	//==== wire/reg definition ================================
    //for storage
    reg         valid_w[0:7];
    reg         valid_r[0:7];
    reg  [24:0] tag_w  [0:7];
    reg  [24:0] tag_r  [0:7];
    reg  [15:0] word_w[0:63]; 
    reg  [15:0] word_r[0:63];

    //for next half block addr
    wire [5:0] next_half_block;

    //for cross block error
    reg cross_block_error_r, cross_block_error_w;
    wire [27:0] next_block_addr;

    //for decompression
    wire [31:0] decomp_instr;
    reg  [15:0] orig_instr;

    // for FSM
    reg  state_nxt, state;  

    // hit or miss
    reg     hit_or_miss; // 1 for hit, 0 for miss

    // for circuit output
    reg [31:0]  rdata;
    reg         stall;

    // Decompression Unit
    DecompressionUnit DU1(.orig_instr  (orig_instr), .decomp_instr(decomp_instr));

	//==== Finite State Machine ===============================
	always@( posedge clk) begin
	    if( proc_reset ) begin
	        state <= START;
	    end else begin
	        state <= state_nxt;
	    end
	end

	//==== next state logic =====================================
	always@(*) begin
	    case ( state )
	        START:
	            begin 
	                if( hit_or_miss ) state_nxt = START; // hit!!
	                else state_nxt = ALLOCATE;
	            end
	        ALLOCATE:
	            begin
	                if( mem_ready ) state_nxt = START;
	                else state_nxt = ALLOCATE;
	            end
	    endcase
	end

	//==== combinational circuit ==============================

	assign proc_rdata = rdata;
	assign proc_stall = stall;
	assign next_half_block = proc_addr[5:0] + 6'd1;
	assign next_block_addr = proc_addr[30:3] + 28'd1;

	always@(*) begin
	    //==== Default value ==================================
	    rdata               = 32'b0;
	    stall               = 1'b0;
	    mem_read            = 0;
	    mem_addr            = proc_addr[29:2];
	    hit_or_miss         = 1'b0;
	    pc_add              = 1'b1;
	    cross_block_error_w = 1'b0;
	    for (i=0;i<8;i=i+1) begin
	        valid_w[i] = valid_r[i]; 
	        tag_w[i]   = tag_r[i]; 
	    end
	    for (i=0;i<64;i=i+1) begin
	        word_w[i]  = word_r[i]; 
	    end
	    //=====================================================
	    case ( state ) 
	        START:
	            begin
	                if( valid_r[proc_addr[5:3]] && ( tag_r[proc_addr[5:3]] == proc_addr[30:6] ) ) begin
	                    // hit
	                    if ( &word_r[proc_addr[5:0]][9:8] ) begin
	                    	//32-bit instruction
	                    	if ( (&proc_addr[2:0]) && ( tag_r[next_half_block[5:3]] == proc_addr[30:6]) ) begin
	                    		//next instruction crossed block and tag of next block is WRONG
	                    		stall = 1'b1;
	                    		cross_block_error_w = 1'b1;
	                    	end else begin
	                    		//next instruction crossed block and tag of next block is right
	                    		//or next instruction didn't crossed block
	                    		stall = 1'b0;
	                    		hit_or_miss = 1'b1;
	                    		rdata = {word_r[proc_addr[5:0]][ 7: 0], word_r[proc_addr[5:0]][15: 8],
	                    				 word_r[proc_addr[5:0]][23:16], word_r[proc_addr[5:0]][31:24]}; //de-endian
	                    	end
	                    end else begin
	                    	//16-bit instruction
	                    	stall = 1'b0;
	                    	hit_or_miss = 1'b1;
	                    	orig_instr = {word_r[proc_addr[5:0]][7:0], word_r[proc_addr[5:0]][15:8]}; //de-endian
	                    	rdata = decomp_instr;
	                    	pc_add = 1'b0;
	                    end
	                    //stall = 1'b0;
	                    //hit_or_miss = 1'b1;
	                    //rdata = word_r[proc_addr[4:0]]; //always read
	                end
	                else begin
	                    // miss
	                    stall = 1'b1;
	                end
	            end
	        ALLOCATE:
	            begin
	                stall = 1'b1;
	                if ( mem_ready ) begin
	                    case ( proc_addr[4:2] ) 
	                        3'd0: tag_w[0] = proc_addr[30:6];
	                        3'd1: tag_w[1] = proc_addr[30:6];
	                        3'd2: tag_w[2] = proc_addr[30:6];
	                        3'd3: tag_w[3] = proc_addr[30:6];
	                        3'd4: tag_w[4] = proc_addr[30:6];
	                        3'd5: tag_w[5] = proc_addr[30:6];
	                        3'd6: tag_w[6] = proc_addr[30:6];
	                        3'd7: tag_w[7] = proc_addr[30:6];
	                    endcase
	                    valid_w[proc_addr[5:3]] = 1'b1;
	                    {{word_w[{proc_addr[5:3], 3'b110}]}, {word_w[{proc_addr[5:3], 3'b111}]},
	                     {word_w[{proc_addr[5:3], 3'b100}]}, {word_w[{proc_addr[5:3], 3'b101}]},
	                     {word_w[{proc_addr[5:3], 3'b010}]}, {word_w[{proc_addr[5:3], 3'b011}]},
	                     {word_w[{proc_addr[5:3], 3'b000}]}, {word_w[{proc_addr[5:3], 3'b001}]}} = mem_rdata;
	                     //blocks that contains 16/32-bit information should be first
	                end
	                else begin
	                    mem_addr = (cross_block_error_r) ? next_block_addr : proc_addr[30:3];
	                    mem_read =  1'b1;
	                end
	            end
	    endcase
	end
	//==== sequential circuit =================================
	always@( posedge clk ) begin
	    if( proc_reset ) begin
	        for (i=0;i<8;i=i+1) begin
	            valid_r[i] <= 1'b0; // reset valid bit
	            tag_r[i]   <= 25'b0; // reset tag
	        end
	        for (i=0;i<64;i=i+1) begin
	            word_r[i]  <= 16'b0; // reset words
	        end
	        cross_block_error_r <= 1'b0; //cross block and tag wrong
	    end
	    else begin
	        for (i=0;i<8;i=i+1) begin
	            valid_r[i] <= valid_w[i]; // reset valid bit
	            tag_r[i]   <= tag_w[i]; // reset tag
	        end
	        for (i=0;i<64;i=i+1) begin
	            word_r[i]  <= word_w[i]; // reset words
	        end
	        cross_block_error_r <= cross_block_error_w; //cross block and tag wrong
	    end
	end
endmodule