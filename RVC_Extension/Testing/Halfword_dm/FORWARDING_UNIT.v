module FORWARDING_UNIT(
	input  [4:0] EXMEM_RD,
	input  [4:0] IDEX_RS1,
	input  [4:0] IDEX_RS2,
	input  [4:0] IFID_RS1,
	input  [4:0] IFID_RS2,	
	input  [4:0] MEMWB_RD,
	input        EXMEM_RegWrite,
	input        MEMWB_RegWrite,
	output reg [1:0] FORWARD_A_ex,
	output reg [1:0] FORWARD_B_ex,
	output reg [1:0] FORWARD_A_id,
	output reg [1:0] FORWARD_B_id
	);
	
	wire FLAG1, FLAG2, FLAG3, FLAG4;
	wire FLAG5, FLAG6, FLAG7, FLAG8;
	wire FLAG_RS1_ZERO, FLAG_RS2_ZERO;


	assign FLAG1 = ~|(EXMEM_RD ^ IDEX_RS1);
	assign FLAG2 = ~|(MEMWB_RD ^ IDEX_RS1);
	assign FLAG3 = ~|(EXMEM_RD ^ IDEX_RS2);
	assign FLAG4 = ~|(MEMWB_RD ^ IDEX_RS2);

	assign FLAG5 = ~|(EXMEM_RD ^ IFID_RS1);
	assign FLAG6 = ~|(MEMWB_RD ^ IFID_RS1);
	assign FLAG7 = ~|(EXMEM_RD ^ IFID_RS2);
	assign FLAG8 = ~|(MEMWB_RD ^ IFID_RS2);

	assign FLAG_RS1_ZERO = (|(IFID_RS1))? 1'b1: 1'b0;
	assign FLAG_RS2_ZERO = (|(IFID_RS2))? 1'b1: 1'b0;

	assign FLAG_RS1_ZERO_EX = (|(IDEX_RS1))? 1'b1: 1'b0;
	assign FLAG_RS2_ZERO_EX = (|(IDEX_RS2))? 1'b1: 1'b0;

	always@(*) begin
		FORWARD_A_ex[1] = (EXMEM_RegWrite && FLAG1 && (FLAG_RS1_ZERO_EX));
		FORWARD_A_ex[0] = (MEMWB_RegWrite && FLAG2 && (FLAG_RS1_ZERO_EX));

		FORWARD_B_ex[1] = (EXMEM_RegWrite && FLAG3 && (FLAG_RS2_ZERO_EX));
		FORWARD_B_ex[0] = (MEMWB_RegWrite && FLAG4 && (FLAG_RS2_ZERO_EX));
		
		
		FORWARD_A_id[1] = (EXMEM_RegWrite && FLAG5 && (FLAG_RS1_ZERO));
		FORWARD_A_id[0] = (MEMWB_RegWrite && FLAG6 && (FLAG_RS1_ZERO));
		

		FORWARD_B_id[1] = (EXMEM_RegWrite && FLAG7 && (FLAG_RS2_ZERO));
		FORWARD_B_id[0] = (MEMWB_RegWrite && FLAG8 && (FLAG_RS2_ZERO));
	end

endmodule