`timescale 1 ns/10 ps
`define TestPort    30'h10
`define BeginSymbol 32'h00000168
`define EndSymbol   32'hFFFFFD5D
`define CheckNum    5'd19

module	TestBed(
	clk,
	rst,
	addr,
	data,
	wen,
	error_num,
	duration,
	finish,
	hit_count
);
	input			clk, rst;
	input	[29:0]	addr;
	input	[31:0]	data;
	input			wen;
	input           hit_count;

	output	[7:0]	error_num;
	output	[15:0]	duration;
	output			finish;
	reg		[7:0]	error_num;
	reg		[15:0]	duration;
	reg				finish;
	
	reg		[31:0]	answer;

	reg		[1:0]	curstate;
	reg		[1:0]	nxtstate;
	reg		[4:0]	curaddr;
	reg		[4:0]	nxtaddr;
	reg		[7:0]	nxt_error_num;
	reg		[15:0]	nxtduration;
	
	reg				state,state_next;

	wire    [31:0]  data_modify;

	reg [31:0] counter;
	reg [31:0] hit_counter;

	always@( posedge clk or negedge rst ) begin
		if (~rst) begin
			counter = 0;
			hit_counter = 0;
		end else begin
			counter = counter + 1;
			hit_counter = (hit_count) ? (hit_counter+1) : hit_counter;
		end
	end
		
	parameter	state_idle 	= 2'b00;
	parameter	state_check = 2'b01;
	parameter	state_report= 2'b10;	

	assign data_modify = {data[7:0],data[15:8],data[23:16],data[31:24]}; // convert little-endian format to readable format
		
	always@( posedge clk or negedge rst )						// State-DFF
	begin
		if( ~rst )
		begin
			curstate <= state_idle;
			curaddr  <= 0;
			duration <= 0;
			error_num <= 8'd255;
			
			state <= 0;
		end
		else
		begin
			curstate <= nxtstate;
			curaddr  <= nxtaddr;
			duration <= nxtduration;
			error_num <= nxt_error_num;
			
			state <= state_next;
		end
	end
			
	always@(*)	// FSM for test
	begin
		finish = 1'b0;
		case( curstate )
		state_idle: 	begin
							nxtaddr = 0;
							nxtduration = 0;
							nxt_error_num = 255;	
							if( addr==`TestPort && data_modify==`BeginSymbol && wen )
							begin
								nxt_error_num = 0;
								nxtstate = state_check;
							end	 	
							else nxtstate = state_idle;
						end
		state_check:	begin
							nxtduration = duration + 1;
							nxtaddr = curaddr;						
							nxt_error_num = error_num;	
							if( addr==`TestPort && wen && state==0 )
							begin
								nxtaddr = curaddr + 1;
								if( data_modify != answer )
									nxt_error_num = error_num + 8'd1;
							end
							nxtstate = curstate;
							if( curaddr==`CheckNum )	
								nxtstate = state_report;
						end
		state_report:	begin
							finish = 1'b1;
							nxtaddr = curaddr;
							nxtstate = curstate;		
							nxtduration = duration;
							nxt_error_num = error_num;	
						end				
		endcase	
	end

	always@( negedge clk )						
	begin
		if(curstate == state_report) begin
			$display("--------------------------- Simulation FINISH !!---------------------------");
			if (error_num) begin 
				$display("============================================================================");
				$display("\n (T_T) FAIL!! The simulation result is FAIL!!! there were %d errors at all.\n", error_num);
				$display("============================================================================");
			end
			 else begin 
				$display("============================================================================");
				$display("\n \\(^o^)/ CONGRATULATIONS!!  The simulation result is PASS!!!\n");
				$display("============================================================================");
				$display("Total Cycle: %d", counter);
				$display("Hit Cycle: %d", hit_counter);
			end
		end
	end
	
	always@(*)begin//sub-FSM (avoid the Dcache stall condition)
		case(state)
			1'b0:begin
				if(wen)
					state_next=1;
				else
					state_next=state;				
			end
			1'b1:begin
				if(!wen)
					state_next=0;
				else
					state_next=state;	
			end
		endcase
	end
	
	
	always@(*)	// ROM for correct result
	begin
		answer = 0;
		case( curaddr )
		5'd0 :	answer = 32'h00001234;
		5'd1 :	answer = 32'h0000ABCD;
		5'd2 :	answer = 32'h091A0000;
		5'd3 :	answer = 32'h048D0000;
		5'd4 :	answer = 32'h0B608000;
		5'd5 :	answer = 32'h0ECA4000;
		5'd6 :	answer = 32'h07652000;
		5'd7 :	answer = 32'h03B29000;
		5'd8 :	answer = 32'h0AF34800;
		5'd9 :	answer = 32'h0E93A400;
		5'd10:	answer = 32'h1063D200;
		5'd11:	answer = 32'h114BE900;
		5'd12:	answer = 32'h08A5F480;
		5'd13:	answer = 32'h0D6CFA40;
		5'd14:	answer = 32'h06B67D20;
		5'd15:	answer = 32'h0C753E90;
		5'd16:	answer = 32'h063A9F48;
		5'd17:	answer = 32'h0C374FA4;
		5'd18:	answer = `EndSymbol;
		endcase			
	end

endmodule