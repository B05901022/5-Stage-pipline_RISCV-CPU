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