`include "DECOMPRESSIONUNIT.v"

//First method: NOP

module FIRSTMETHOD(
	clk,
	rst_n,
	PC_1,
	INSTR,
	PC_add,
	INSTR_o
	);
	
	// === I/O ===
	input         PC_1; //PC[1] for checking if now using half block
	input  [31:0] INSTR; //the whole block fetched from cache/memory
	output [31:0] PC_add; //addition to PC for next stage
	output [31:0] INSTR_o; //Instruction send to ID-stage

	// === Wires ===
	reg  [31:0] INSTR_o;
	wire [31:0] INSTR_d;
	reg  [15:0] INSTR_c;
	reg  [15:0] INSTR_save_r, INSTR_save_w;
	//reg  [31:0] PC_r, PC_w;
	reg  [31:0] PC_add;
	reg         stall_r, stall_w;

	/*
	INSTR: original instruction(in little endian)
	INSTR_o: output instruction(in common order)
	INSTR_c: compressed instruction(in common order)
	INSTR_d: decompressed instruction(in common order)
	INSTR_save_r, INSTR_save_w: saved instruction for the next 16-bit for a complete 32-bit instruction(in common order)
	*/

	/* ------ Current Scheme --------

                   ---------
                   |CURRENT|
	               |  P C  |
	               ---------
	--------   --------|--------
			| | 32-bit |        |
	--------   --------|--------
	
	 32-bit instruction
	<------------------|

	Above condition will need an NOP
	(Because not enough information
	for the whole 32-bit instruction)
	Might be optimized in some way?

	-------------------------------*/

	DecompressionUnit DU1(.orig_instr(INSTR_c), .decomp_instr(INSTR_d));

	always@(*) begin

		PC_add       = 0;
		INSTR_o      = 32'b0;
		INSTR_c      = 16'b0;
		INSTR_save_w = 16'b0;
		stall_w      = 1'b0;

		if ( stall_r ) begin
			PC_add = 0; //PC_w = PC_r;
			INSTR_o = { INSTR[23:16], INSTR[31:24], INSTR_save_r };
			stall_w = 1'b0;
		end else begin
			if ( PC_1 ) begin
				//Half block
				if ( &INSTR[25:24] ) begin
					//32-bit instruction
					PC_add = 4; //PC_w = PC_r + 4;
					INSTR_o = { INSTR[7:0], INSTR[15:8], INSTR[23:16], INSTR[31:24] };
				end else begin
					//16-bit instruction
					PC_add = 2; //PC_w = PC_r + 2;
					INSTR_c = { INSTR[23:16], INSTR[31:24] }; //16-bit
					INSTR_o = INSTR_d;
				end
			end else begin
				//Complete block
				if ( &INSTR[9:8] ) begin
					//32-bit instruction
					PC_add = 4; //PC_w = PC_r + 4;
					INSTR_o = 32'h00_00_00_13; //NOP
					stall_w = 1'b1;
					INSTR_save_w = { INSTR[7:0], INSTR[15:8] }; //Save pre-switched operations					
				end else begin
					//16-bit instruction
					PC_add = 2; //PC_w = PC_r + 2;
					INSTR_c = { INSTR[7:0], INSTR[15:8] }; //16-bit
					INSTR_o = INSTR_d;
				end
			end
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			INSTR_save_r <= 16'b0;
			stall_r      <= 1'b0;
		end else begin
			INSTR_save_r <= INSTR_save_w;
			stall_r      <= stall_w;
		end
	end

endmodule

//Second Method: Pre-feed PC to obtain new information

module SECONDMETHOD(
	clk,
	rst_n,
	PCN_1,
	INSTR,
	PCN_add,
	PC_add,
	INSTR_o
	);
	
	// === I/O ===
	input         PCN_1; //PC[1] for checking if now using half block
	input  [31:0] INSTR; //the whole block fetched from cache/memory
	output [31:0] PCN_add; //addition to PC for next stage
	output [31:0] PC_add;  //addition to PC for next cycle
	output [31:0] INSTR_o; //Instruction send to ID-stage

	// === Wires ===
	reg  [31:0] INSTR_o;
	wire [31:0] INSTR_d;
	reg  [15:0] INSTR_c;
	reg  [15:0] INSTR_save_r, INSTR_save_w;
	reg  [31:0] PCN_add, PC_add;
	reg         flag_r, flag_w; //to examine if current instruction is a 32-bit starting from a half block

	/*
	INSTR: original instruction(in little endian)
	INSTR_o: output instruction(in common order)
	INSTR_c: compressed instruction(in common order)
	INSTR_d: decompressed instruction(in common order)
	INSTR_save_r, INSTR_save_w: saved instruction for the next 16-bit for a complete 32-bit instruction(in common order)
	*/

	/* ---------- Current Scheme ----------

	Still remain problems with Branch, Jump

	-------------------------------------*/

	DecompressionUnit DU1(.orig_instr(INSTR_c), .decomp_instr(INSTR_d));

	always@(*) begin

		PCN_add      = 0;
		PC_add       = 0;
		INSTR_o      = 32'b0;
		INSTR_c      = 16'b0;
		INSTR_save_w = 16'b0;
		flag_w       = flag_r;

		if ( PCN_1 ) begin
			//Complete block
			if ( &INSTR[25:24] ) begin
				//32-bit instruction
				PCN_add = 4;
				PC_add  = 4;
				INSTR_o = { INSTR[7:0], INSTR[15:8], INSTR[23:16], INSTR[31:24] };
				flag_w  = 1'b0;
			end else begin
				//16-bit instruction, need to check if next instruction is half block
				PCN_add = 2; //original PC
				INSTR_c = { INSTR[23:16], INSTR[31:24] };
				if ( &INSTR[9:8] ) begin
					//The next instruction is 32-bit, need to pre-feed PC to avoid NOP
					PC_add  = 4; //pre-call PC for new informations
					INSTR_save_w = { INSTR[7:0], INSTR[15:8] };
					flag_w = 1'b1;
				end else begin
					//The next instruction is 16-bit, commonly finish the task
					PC_add  = 2;
					flag_w = 1'b0;
				end
			end
		end else begin
			//Half block
			if ( flag_r ) begin
				//32-bit instruction
				PCN_add = 4;
				INSTR_o = { INSTR[23:16], INSTR[31:24], INSTR_save_r };
				if ( INSTR[9:8] ) begin
					//The next instruction is still 32-bit, continue pre-feed
					PC_add  = 4;
					INSTR_save_w = { INSTR[7:0], INSTR[15:8] };
					flag_w  = 1'b1;
				end else begin
					//The next instruction is 16-bit, back to original
					PC_add  = 2; //back to original PC-feeding
					flag_w = 1'b0;
				end
			end else begin
				//16-bit instruction
				PCN_add = 2;
				PC_add  = 2;
				INSTR_c = { INSTR[7:0], INSTR[15:8] };
				INSTR_o = INSTR_d;
				flag_w  = 1'b0;
			end
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			INSTR_save_r <= 16'b0;
			flag_r       <= 1'b0;
		end else begin
			INSTR_save_r <= INSTR_save_w;
			flag_r       <= flag_w;
		end
	end

endmodule

//Third Method: Half-block cache
