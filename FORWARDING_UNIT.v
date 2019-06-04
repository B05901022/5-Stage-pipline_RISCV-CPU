module FORWARDING_UNIT(
	input  [4:0] EXMEM_RD,
	input  [4:0] IDEX_RS,
	input  [4:0] IDEX_RT,
	input  [4:0] MEMWB_RD,
	input        EXMEM_RegWrite,
	input        MEMWB_RegWrite,
	output reg [1:0] FORWARD_A,
	output reg [1:0] FORWARD_B
	);
	
	wire FLAG1, FLAG2, FLAG3, FLAG4;



	assign FLAG1 = (EXMEM_RD == IDEX_RS);
	assign FLAG2 = (MEMWB_RD == IDEX_RS);
	assign FLAG3 = (EXMEM_RD == IDEX_RT);
	assign FLAG4 = (MEMWB_RD == IDEX_RT);

	always@(*) begin
		FORWARD_A[1] = EXMEM_RegWrite && FLAG1;
		FORWARD_A[0] = MEMWB_RegWrite && FLAG2;

		FORWARD_B[1] = EXMEM_RegWrite && FLAG3;
		FORWARD_B[0] = MEMWB_RegWrite && FLAG4;
	end

endmodule