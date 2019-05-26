module HAZARD_DETECTION_UNIT(
	input [4:0] IDEX_RT,
	input [4:0] IFID_RS,
	input [4:0] IFID_RT,
	input       MemRead,
	output      stall
	);
	
	/*
	if (stall) begin
		PCWrite = 1'b0;
		IF_ID_Write = 1'b0;
		Ctrl_Sel = 1'b0;
	end else begin
		PCWrite = 1'b1;
		IF_ID_Write = 1'b1;
		Ctrl_Sel = 1'b1;
	end
	*/
	always@(*) begin
		if (MemRead && ( (IDEX_RT == IFID_RS) | (IDEX_RT == IFID_RT) ) ) begin
			stall = 1'b1;
		end else begin
			stall = 1'b0;
		end
	end

endmodule