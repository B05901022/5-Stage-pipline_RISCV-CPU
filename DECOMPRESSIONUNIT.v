module DecompressionUnit(
	input  [15:0] orig_instr,
	output [31:0] decomp_instr
);

	// Instruction in normal order, not little endian

	/*
	compressed instructions:
		C.ADD
		C.MV
		C.ADDI
		C.NOP
		C.ANDI
		C.SLLI
		C.SRLI
		C.SRAI
		C.LW
		C.SW
		C.BEQZ
		C.BNEZ
		C.J
		C.JAL
		C.JR
		C.JALR
	*/
	reg [31:0] decomp_instr;
	//wire or_gate1;

	//assign or_gate = (|orig_instr[14:13]);

	always@(*) begin
		decomp_instr[31]  = 1'b0; //all starts with 0
		decomp_instr[1:0] = 2'b11; //decompressed instruction should have opcode[1:0] be 2'b11;
		//left: [30:2]
		case (orig_instr[1:0])
			2'b00: begin
				//C.SW, C.LW
				decomp_instr[30:27] = 4'b0; //Immediate zero-extention
				decomp_instr[26]    = orig_instr[5]; //Offset[6]
				decomp_instr[25]    = orig_instr[12]; //Offset[5]
				decomp_instr[19:18] = 2'b00; //only x8~x15
				decomp_instr[17:15] = orig_instr[9:7]; //x8~x15
				decomp_instr[14:12] = 3'b010;
				decomp_instr[6]     = 1'b0;
				decomp_instr[5]     = orig_instr[15];
				decomp_instr[4:2]   = 3'b000;
				//left: [24:20],[11:7]
				if (orig_instr[15]) begin
					//C.SW
					decomp_instr[24:23] = 2'b01; //only x8~x15
					decomp_instr[22:20] = orig_instr[4:2]; //x8~x15 
					decomp_instr[11:10] = orig_instr[11:10]; //Offset[4:3]
					decomp_instr[9]     = orig_instr[6]; //Offset[2]
					decomp_instr[8:7]   = 2'b00; //Immediate scaled by four, Offset[1:0]
				end else begin
					//C.LW
					decomp_instr[24:23] = orig_instr[11:10]; //Offset[4:3]
					decomp_instr[22]    = orig_instr[6]; //Offset[2]
					decomp_instr[21:20] = 2'b00; //Immediate scaled by four
					decomp_instr[11:10] = 2'b01; //only x8~x15
					decomp_instr[9:7]   = orig_instr[4:2]; //x8~x15
				end
			end
			2'b01: begin
				//C.NOP, C.ADDI, C.SRLI, C.SRAI, C.ANDI, C.JAL, C.J, C.BEQZ, C.BNEZ
				case (orig_instr[15:13])
					3'b000: begin
						//C.NOP, C.ADDI
						decomp_instr[30:26] = 5'b00000; //Immediate zero-extension
						decomp_instr[25]    = orig_instr[12]; //Immediate[5]
						decomp_instr[24:20] = orig_instr[6:2]; //Immediate[4:0]
						decomp_instr[19:15] = orig_instr[11:7]; //rd, rd=0 if NOP
						decomp_instr[14:12] = 3'b000;
						decomp_instr[11:7]  = orig_instr[6:2]; //rd, rd=0 if NOP
						decomp_instr[6:2]   = 5'b00100;						
					end
					3'b100: begin
						//C.SRLI, C.SRAI, C.ANDI
						decomp_instr[30]    = orig_instr[10]; //1 for C.SRAI; 0 for C.SRLI, C.ANDI
						decomp_instr[29:26] = 4'b0000; //Immediate zero-extension
						decomp_instr[25]    = orig_instr[12]; //shamt[5] for C.SRAI, C.SRLI; imm[5] for C.ANDI
						decomp_instr[24:20] = orig_instr[6:2]; //shamt[4:0] for C.SRAI, C.SRLI; imm[4:0] for C.ANDI
						decomp_instr[19:15] = orig_instr[11:7]; //rd, rd=0 if NOP
						decomp_instr[14]    = 1'b1; 
						decomp_instr[13]    = orig_instr[11]; //1 for C.ANDI; 0 for C.SRAI, C.SRLI
						decomp_instr[12]    = 1'b1; 
						decomp_instr[11:7]  = orig_instr[6:2]; //rd, rd=0 if NOP
						decomp_instr[6:2]   = 5'b00100;
					end
					3'b001, 3'b101: begin
						//C.JAL, C.J
						decomp_instr[30]    = orig_instr[8]; //imm[10]
						decomp_instr[29:28] = orig_instr[10:9]; //imm[9:8]
						decomp_instr[27]    = orig_instr[6]; //imm[7]
						decomp_instr[26]    = orig_instr[7]; //imm[6]
						decomp_instr[25]    = orig_instr[2]; //imm[5]
						decomp_instr[24]    = orig_instr[11]; //imm[4]
						decomp_instr[23:21] = orig_instr[5:3]; //imm[3:1]
						decomp_instr[20]    = orig_instr[12]; //imm[11]
						decomp_instr[19:12] = 8'b0; //Immediate zero-extension
						decomp_instr[11:8]  = 4'b0000; //x0 or x1
						decomp_instr[7]     = orig_instr[15]; //x0 for C.J, x1 for C.JAL
						decomp_instr[6:2]   = 5'b11011;
					end
					3'b110, 3'b111: begin
						//C.BEQZ, C.BNEZ
						decomp_instr[30:29] = 2'b00; //Immediate zero-extension
						decomp_instr[28]    = orig_instr[12]; //imm[8]
						decomp_instr[27:26] = orig_instr[6:5]; //imm[7:6]
						decomp_instr[25]    = orig_instr[2]; //imm[5]	
						decomp_instr[24:20] = 5'b00000; //rs2 = x0
						decomp_instr[19:18] = 2'b01; //x8~x15
						decomp_instr[17:15] = orig_instr[9:7]; //rs1
						decomp_instr[14:13] = 2'b00;
						decomp_instr[12]    = orig_instr[13]; //1 for C.BNEZ; 0 for C.BEQZ
						decomp_instr[11:10] = orig_instr[11:10]; //imm[4:3]
						decomp_instr[9:8]   = orig_instr[4:3]; //imm[2:1]
						decomp_instr[7]     = 1'b0; //Immediate zero-extension
						decomp_instr[6:2]   = 5'b11000;
					end
					default: begin
						decomp_instr[30:2]  = 29'b0; 
					end
				endcase // orig_instr[15:13]
			end
			2'b10: begin
				//C.SLLI, C.JR, C.JALR, C.MV, C.ADD
				decomp_instr[30:25] = 6'b000000; //Immediate zero-extension
				decomp_instr[14:13] = 2'b00;
				decomp_instr[3]     = 1'b0;
				//left:[24:20],[19:15],[12],[11:7],[6:4],[2]
				if (orig_instr[15]) begin
					//C.JR, C.JALR, C.MV, C.ADD
					decomp_instr[12] = 1'b0; //0 for C.JR, C.JALR, C.MV, C.ADD; 1 for C.SLLI
					decomp_instr[5]  = 1'b1;
					if (|orig_instr[6:2]) begin
						//C.MV, C.ADD
						decomp_instr[24:20] = orig_instr[6:2]; //rs2
						decomp_instr[11:7]  = orig_instr[11:7]; //rd
						decomp_instr[6]     = 1'b0;
						decomp_instr[4]     = 1'b1;
						decomp_instr[2]     = 1'b0;
						if (orig_instr[12]) begin
							//C.ADD
							decomp_instr[19:15] = orig_instr[11:7]; //rs1=rd
						end else begin
							//C.MV
							decomp_instr[19:15] = 5'b00000; //rs1=x0
						end
					end else begin
						//C.JR, C.JALR
						decomp_instr[24:20] = 5'b00000; //imm = 0
						decomp_instr[19:15] = orig_instr[11:7]; //rs1
						decomp_instr[11:8]  = 4'b0; //rd=x0 or x1
						decomp_instr[7]     = orig_instr[12]; //rd=x0 for JR; rd=x1 for JALR
						decomp_instr[6]     = 1'b1;
						decomp_instr[4]     = 1'b0;
						decomp_instr[2]     = 1'b1;
					end
				end else begin
					//C.SLLI
					decomp_instr[24:20] = orig_instr[6:2]; //shamt[4:0]
					decomp_instr[19:15] = orig_instr[11:7]; //rd
					decomp_instr[12]    = 1'b1; //0 for C.JR, C.JALR, C.MV, C.ADD; 1 for C.SLLI
					decomp_instr[11:7]  = orig_instr[11:7]; //rs1=rd
					decomp_instr[6:4]   = 3'b001;
					decomp_instr[2]     = 1'b0;
				end
			end
			2'b11: begin
				decomp_instr[30:2] = 29'b0;
			end
		endcase
	end

endmodule

module InstructionAnalyzer(
	PC_1,
	INSTR,
	PC_add,
	INSTR_o
	);
	
	// === I/O ===
	input         PC_1; //PC[1] for checking if now using half block
	input  [31:0] INSTR; //the whole block fetched from cache/memory
	output [31:0] PC_add; //addition for PC
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
		INSTR_save_w = 16'b0;

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