//============ Breanch Predict Unit ============
module BPU(
    clk,
    rst_n,
    B,
    stall,
    PreWrong,
    BrPre
);
//==== I/O Definition ===========================
    input   clk;
    input   rst_n;
    input   B;        //which will be set to 1 if the inst. in IF stage is a branch
    input   stall;    //stall signal from cache or hazard detection unit
    input   PreWrong; //miss prediction signal sent from the comparator
    output  BrPre;   // 1/0 stands for a positive/negative prediction on branching  

//==== FSM ======================================
    parameter   WEAK_TAKE       = 2'b00;
    parameter   STRONG_TAKE     = 2'b01;    
    parameter   WEAK_NOT_TAKE   = 2'b10;  
    parameter   STRONG_NOT_TAKE = 2'b11;

    reg [1:0]   state, nxt_state;
    reg         prdict, BrPre;

    always@(*) begin
        case(state)
            WEAK_TAKE:  begin
                prdict = 1'b1;
                if(PreWrong)    nxt_state = WEAK_NOT_TAKE;
                else            nxt_state = STRONG_TAKE; 
            end
            STRONG_TAKE: begin
                prdict = 1'b1;
                if(PreWrong)    nxt_state = WEAK_TAKE;
                else            nxt_state = STRONG_TAKE; 
            end
            WEAK_NOT_TAKE: begin
                prdict = 1'b0;
                if(PreWrong)    nxt_state = WEAK_TAKE;
                else            nxt_state = STRONG_NOT_TAKE; 
            end
            STRONG_NOT_TAKE: begin
                prdict = 1'b0;
                if(PreWrong)    nxt_state = WEAK_NOT_TAKE;
                else            nxt_state = STRONG_NOT_TAKE; 
            end
        endcase
    end

    always@(*) begin
        if(B) BrPre = prdict;
        else  BrPre = 1'b0;
    end


    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            state <= WEAK_TAKE;
        end
        else begin
            if(stall) state <= state;
            else      state <= nxt_state;
        end
    end
endmodule