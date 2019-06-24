module DecompressionUnit(
	input  [15:0] orig_instr,
	output reg [31:0] decomp_instr
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

	always@(*) begin
		decomp_instr[1:0] = 2'b11; //decompressed instruction should have opcode[1:0] be 2'b11;
		//left: [30:2]
		case (orig_instr[1:0])
			2'b00: begin
				//C.SW, C.LW
				decomp_instr[31:27] = 5'b0;
				decomp_instr[26]    = orig_instr[5];  //Offset[6]
				decomp_instr[25]    = orig_instr[12]; //Offset[5]
				decomp_instr[19:18] = 2'b01; //only x8~x15
				decomp_instr[17:15] = orig_instr[9:7]; //x8~x15
				decomp_instr[14:12] = 3'b010;
				decomp_instr[6]     = 1'b0;
				decomp_instr[5]     = orig_instr[15];
				decomp_instr[4:2]   = 3'b000;
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
						decomp_instr[31:25] = {7{orig_instr[12]}}; //Immediate sign-extension, Immediate[5]
						decomp_instr[24:20] = orig_instr[6:2]; //Immediate[4:0]
						decomp_instr[19:15] = orig_instr[11:7]; //rd, rd=0 if NOP
						decomp_instr[14:12] = 3'b000;
						decomp_instr[11:7]  = orig_instr[11:7]; //rd, rd=0 if NOP
						decomp_instr[6:2]   = 5'b00100;						
					end
					3'b100: begin
						//C.SRLI, C.SRAI, C.ANDI
						decomp_instr[25:20] = {orig_instr[12], orig_instr[6:2]};
						decomp_instr[19:15] = {2'b01, orig_instr[9:7]};
						decomp_instr[14:12] = {1'b1, orig_instr[11], 1'b1};
						decomp_instr[11: 7] = {2'b01, orig_instr[9:7]};
						decomp_instr[ 6: 2] = 5'b00100; 
						if (orig_instr[11]) begin
							//C.ANDI
							decomp_instr[31:26] = {6{orig_instr[12]}};
						end else begin
							//C.SRLI, C.SRAI
							decomp_instr[31:26] = {1'b0, orig_instr[10], 4'b0};
						end
					end
					3'b001, 3'b101: begin
						//C.JAL, C.J
						decomp_instr[31:12] = {orig_instr[12],
											   orig_instr[8],
											   orig_instr[10: 9],
											   orig_instr[6],
											   orig_instr[7],
											   orig_instr[2],
											   orig_instr[11],
											   orig_instr[5:3],
											   orig_instr[12],
											   {8{orig_instr[12]}}
											   };
						decomp_instr[11:8]  = 4'b0000; //x0 or x1
						decomp_instr[7]     = ~orig_instr[15]; //x0 for C.J, x1 for C.JAL
						decomp_instr[6:2]   = 5'b11011;
					end
					3'b110, 3'b111: begin
						//C.BEQZ, C.BNEZ
						/*
						decomp_instr[30:28] = 3'b0;
						decomp_instr[27:25] = {orig_instr[12], orig_instr[6:5]};
						decomp_instr[24:20] = 5'b0;
						decomp_instr[19:15] = {2'b01, orig_instr[9:7]};
						decomp_instr[14:12] = {2'b00, orig_instr[13]};
						decomp_instr[11: 7] = {orig_instr[2],
											   orig_instr[11:10],
											   orig_instr[4:3],
											   1'b0}; 
						decomp_instr[ 6: 2] = 5'b11000;
						*/
						decomp_instr[31:29] = {3{orig_instr[12]}}; //Immediate sign-extension
						decomp_instr[28:25] = {orig_instr[12],  //imm[8]
											   orig_instr[6:5], //imm[7:6]
											   orig_instr[2]};  //imm[5] 	
						decomp_instr[24:20] = 5'b00000; //rs2 = x0
						decomp_instr[19:15] = {2'b01, orig_instr[9:7]}; //rs1 x8~x15
						decomp_instr[14:12] = {2'b00, orig_instr[13]}; //1 for C.BNEZ; 0 for C.BEQZ
						decomp_instr[11: 7] = {orig_instr[11:10], //imm[4:3]
											   orig_instr[4:3],   //imm[2:1]
											   orig_instr[12]};   //Immediate sign-extension
						decomp_instr[6:2]   = 5'b11000;
					end
					default: begin
						decomp_instr[31:2]  = 30'b0; 
					end
				endcase // orig_instr[15:13]
			end
			2'b10: begin
				//C.SLLI, C.JR, C.JALR, C.MV, C.ADD
				decomp_instr[14:13] = 2'b00;
				decomp_instr[3]     = 1'b0;
				//left:[31:20],[19:15],[12],[11:7],[6:4],[2]
				if (orig_instr[15]) begin
					//C.JR, C.JALR, C.MV, C.ADD
					decomp_instr[12] = 1'b0; //0 for C.JR, C.JALR, C.MV, C.ADD; 1 for C.SLLI
					decomp_instr[5]  = 1'b1;
					if (|orig_instr[6:2]) begin
						//C.MV, C.ADD
						decomp_instr[31:25] = 7'b0;
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
						decomp_instr[31:20] = 12'b0; //imm = 0
						decomp_instr[19:15] = orig_instr[11:7]; //rs1
						decomp_instr[11:8]  = 4'b0; //rd=x0 or x1
						decomp_instr[7]     = orig_instr[12]; //rd=x0 for JR; rd=x1 for JALR
						decomp_instr[6]     = 1'b1;
						decomp_instr[4]     = 1'b0;
						decomp_instr[2]     = 1'b1;
					end
				end else begin
					//C.SLLI
					decomp_instr[31:26] = 6'b0;
					decomp_instr[25:20] = {orig_instr[12], orig_instr[6:2]}; //shamt[5:0]
					decomp_instr[19:15] = orig_instr[11:7]; //rd
					decomp_instr[12]    = 1'b1; //0 for C.JR, C.JALR, C.MV, C.ADD; 1 for C.SLLI
					decomp_instr[11:7]  = orig_instr[11:7]; //rs1=rd
					decomp_instr[6:4]   = 3'b001;
					decomp_instr[2]     = 1'b0;
				end
			end
			2'b11: begin
				decomp_instr[31:2] = 30'b0;
			end
		endcase
	end

endmodule

