// Five Stage pilpeline RISCV CPU

module RISCV_Pipeline(
    // control interface
    clk,
    rst_n,
//----------I cache interface-------
    ICACHE_ren      ,
	ICACHE_wen      ,
	ICACHE_addr     ,
	ICACHE_wdata    ,
	ICACHE_stall    ,
	ICACHE_rdata    ,
//----------D cache interface-------
	DCACHE_ren      ,
	DCACHE_wen      ,
	DCACHE_addr     ,
	DCACHE_wdata    ,
	DCACHE_stall    ,
	DCACHE_rdata   
);

//==== I/O Definition ====================
    // control interface ---------------
    input   clk;
    input   rst_n;
    //----------I cache interface-------
    output          ICACHE_ren;
    output	        ICACHE_wen;
	output  [29:0]  ICACHE_addr;
	output  [31:0]  ICACHE_wdata;
	output          ICACHE_stall;
	input   [31:0]  ICACHE_rdata;
    //----------D cache interface-------
    output          DCACHE_ren;
    output	        DCACHE_wen;
	output  [29:0]  DCACHE_addr;
	output  [31:0]  DCACHE_wdata;
	output          DCACHE_stall;
	input   [31:0]  DCACHE_rdata;     

endmodule

//===================================================
//=                 Register File                   =
//===================================================

module register_file(
    clk  ,
    rst_n,
    WEN  ,
    RW   ,
    busW ,
    RX   ,
    RY   ,
    busX ,
    busY
);
input        clk, WEN, rst_n;
input  [4:0] RW, RX, RY; // 5bit read from instruction code
input  [31:0] busW;
output [31:0] busX, busY;
    
// write your design here, you can delcare your own wires and regs. 
// The code below is just an eaxmple template
reg [31:0] r_w [0:31];
reg [31:0] r_r [0:31];
reg [31:0] busX, busY;
integer i;
    
always@(*) begin
    for(i=0; i<32; i = i+1) begin
        r_w[i] = r_r[i];
    end
        
    if (WEN) begin
        case (RW)
            5'd0 : r_w [0] = 32'b0;
            5'd1 : r_w [1] = busW;
            5'd2 : r_w [2] = busW;
            5'd3 : r_w [3] = busW;
            5'd4 : r_w [4] = busW;
            5'd5 : r_w [5] = busW;
            5'd6 : r_w [6] = busW;
            5'd7 : r_w [7] = busW;
            5'd8 : r_w [8] = busW;
            5'd9 : r_w [9] = busW;
            5'd10 : r_w [10] = busW;
            5'd11 : r_w [11] = busW;
            5'd12 : r_w [12] = busW;
            5'd13 : r_w [13] = busW;
            5'd14 : r_w [14] = busW;
            5'd15 : r_w [15] = busW;
            5'd16 : r_w [16] = busW;
            5'd17 : r_w [17] = busW;
            5'd18 : r_w [18] = busW;
            5'd19 : r_w [19] = busW;
            5'd20 : r_w [20] = busW;
            5'd21 : r_w [21] = busW;
            5'd22 : r_w [22] = busW;
            5'd23 : r_w [23] = busW;
            5'd24 : r_w [24] = busW;
            5'd25 : r_w [25] = busW;
            5'd26 : r_w [26] = busW;
            5'd27 : r_w [27] = busW;
            5'd28 : r_w [28] = busW;
            5'd29 : r_w [29] = busW;
            5'd30 : r_w [30] = busW;
            5'd31 : r_w [31] = busW;
        endcase
    end
    else begin
        for(i=0; i<32; i = i+1) begin
            r_w[i] = r_r[i];
        end
    end
    case (RX)
        5'd0 : busX = 32'b0;
        5'd1 : busX = r_r[1];
        5'd2 : busX = r_r[2];
        5'd3 : busX = r_r[3];
        5'd4 : busX = r_r[4];
        5'd5 : busX = r_r[5];
        5'd6 : busX = r_r[6];
        5'd7 : busX = r_r[7];
        5'd8 : busX = r_r[8];
        5'd9 : busX = r_r[9];
        5'd10 : busX = r_r[10];
        5'd11 : busX = r_r[11];
        5'd12 : busX = r_r[12];
        5'd13 : busX = r_r[13];
        5'd14 : busX = r_r[14];
        5'd15 : busX = r_r[15];
        5'd16 : busX = r_r[16];
        5'd17 : busX = r_r[17];
        5'd18 : busX = r_r[18];
        5'd19 : busX = r_r[19];
        5'd20 : busX = r_r[20];
        5'd21 : busX = r_r[21];
        5'd22 : busX = r_r[22];
        5'd23 : busX = r_r[23];
        5'd24 : busX = r_r[24];
        5'd25 : busX = r_r[25];
        5'd26 : busX = r_r[26];
        5'd27 : busX = r_r[27];
        5'd28 : busX = r_r[28];
        5'd29 : busX = r_r[29];
        5'd30 : busX = r_r[30];
        5'd31 : busX = r_r[31];
    endcase

    case (RY)
        5'd0 : busY = 32'b0;
        5'd1 : busY = r_r[1];
        5'd2 : busY = r_r[2];
        5'd3 : busY = r_r[3];
        5'd4 : busY = r_r[4];
        5'd5 : busY = r_r[5];
        5'd6 : busY = r_r[6];
        5'd7 : busY = r_r[7];
        5'd8 : busY = r_r[8];
        5'd9 : busY = r_r[9];
        5'd10 : busY = r_r[10];
        5'd11 : busY = r_r[11];
        5'd12 : busY = r_r[12];
        5'd13 : busY = r_r[13];
        5'd14 : busY = r_r[14];
        5'd15 : busY = r_r[15];
        5'd16 : busY = r_r[16];
        5'd17 : busY = r_r[17];
        5'd18 : busY = r_r[18];
        5'd19 : busY = r_r[19];
        5'd20 : busY = r_r[20];
        5'd21 : busY = r_r[21];
        5'd22 : busY = r_r[22];
        5'd23 : busY = r_r[23];
        5'd24 : busY = r_r[24];
        5'd25 : busY = r_r[25];
        5'd26 : busY = r_r[26];
        5'd27 : busY = r_r[27];
        5'd28 : busY = r_r[28];
        5'd29 : busY = r_r[29];
        5'd30 : busY = r_r[30];
        5'd31 : busY = r_r[31];
    endcase
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i=0; i<32; i=i+1) begin
            r_r[i] <= 32'b0;
        end
    end
    else begin
        r_r [0] <= 32'b0;
        for(i=1;i<32;i=i+1) begin
            r_r [i] <= r_w[i];
        end
    end
end	

endmodule

//==================================
//=             ALU                =
//==================================
module ALU (
    ctrl,
    x,
    y,
    out,
    zero
);
    input   [3:0]  ctrl;    // the contral signal which decide the operation of alu
    input   [31:0] x, y;    // 32 bit input data from register file or instruction[15:0]
    output  [31:0] out;     // 32 bit output
    output         zero;    // input for branch control              

    reg [31:0] out;
    reg         zero;
    wire [32:0] sub_result;
    wire [4:0] shamt; // shift amount unsigned 5-bit

    wire beq, bne;
    assign sub_result = x + ~y + 1;
    assign beq = ~(|sub_result[31:0]); // $$$ check if error 
    assign bne = beq;
    assign shamt = y[4:0];

    always@(*) begin
    case ( ctrl )
        4'b0000: out = x + y;                                   //ADD
        4'b1000: out = sub_result;                              //SUB
        4'b0010: out = out = (sub_result[32]) ? 32'd1 : 32'b0;  //SLT
        4'b0100: out = x ^ y;                                   //XOR
        4'b0110: out = x | y;                                   //OR
        4'b0111: out = x & y;                                   //AND
        4'b0001: out = x << shamt;                              //SLL
        4'b0101: out = x >> shamt;                              //SRL
        4'b1101: out = x >>> shamt;                             //SRA
        4'b1011: zero = beq;                                    //BEQ
        4'b0011: zero = bne;                                    //BNE
        default: begin
            out = 32'b0;
            zero = 0;
        end 
    endcase

    end

endmodule

//===============================================
//=             Immediate Generator             =
//===============================================
module ImmGen(
    IR,
    Imm
);
    input   [31:0]  IR;     // instruction from instruction memory
    output  [31:0]  Imm;    
    reg     [31:0]  Imm;

    always@(*) begin
        case (IR[6:2])
            5'b00000:   Imm = {{21{IR[31]}},IR[30:25],IR[24:21],IR[20]};                     // lw I-type
            5'b01000:   Imm = {{21{IR[31]}},IR[30:25],IR[11:8],IR[7]};                       // sw S-type
            5'b11000:   Imm = {{20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};              // beq B-type
            5'b11011:   Imm = {{12{IR[31]}}, IR[19:12], IR[20], IR[30:25], IR[24:21], 1'b0}; // jal J-type
            5'b11001:   Imm = {{21{IR[31]}},IR[30:25],IR[24:21],IR[20]};                     // jalr I-type
            5'b00100:   Imm = { {19{IR[31]}},IR[31:20]};                                     // I-type
            default:    Imm = 32'b0;
        endcase
    end
endmodule

//=======================================
//              ALU Control             =
//=======================================
module ALU_control(
    Func7,
    Func3,
    OP,
    ALUCtrl
);
    input           Func7;
    input   [2:0]   Func3;
    input   [1:0]   OP;
    output  [3:0]   ALUCtrl;

    reg [3:0]       ALUCtrl;
    wire            isSRA;

    assign isSRA = ({Func7, Func3[2:0]} == 4'b1101) ? 1'b1 : 1'b0;

    always@(*) begin
        case(OP)
            2'b00: begin
                //ALUop = 00, JALR, LW, SW
                ALUCtrl = 4'b0000;
            end
            2'b01: begin
                //ALUop = 01, BEQ, BNE
                if(Func3[0]) begin
                    //BNE
                    ALUCtrl = 4'b0011;
                end
                else begin
                    //BEQ
                    ALUCtrl = 4'b1011;
                end
            end
            2'b10: begin
                ALUCtrl = {Func7, Func3[2:0]};
            end
            2'b11: begin
                if(isSRA) begin
                    ALUCtrl = {Func7, Func3[2:0]};
                end
                else begin
                    ALUCtrl = {1'b0, Func3[2:0]};
                end
            end
        endcase
    end
    
endmodule

module Control_unit(
    Opcode,
    Jal,
    Jalr,
    Branch,
    MemRead,
    MemToReg,
    MemWrite,
    ALUSrc,
    RegWrite
);
    input   [4:0]   Opcode;
    output  reg     Jal, Jalr, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite;

    always@(*) begin
        case ( Opcode )
            5'b01100:   begin
                        // R-type
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b1;
                        end 
            5'b00000:   begin
                        // lw
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b1;
                            MemToReg    = 1'b1;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b1;
                            RegWrite    = 1'b1;
                        end 
            5'b01000:   begin
                        // sw
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b1;
                            ALUSrc      = 1'b1;
                            RegWrite    = 1'b0;
                        end 
            5'b11000:   begin
                        // beq
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b1;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b0;
                        end 
            5'b11011:   begin
                        // jal
                            Jal         = 1'b1;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b1;
                        end 
            5'b11001:   begin
                        // jalr
                            Jal         = 1'b0;
                            Jalr        = 1'b1;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b1;
                        end
            5'b00100:   begin
                        // I-type
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b1;
                            RegWrite    = 1'b1;
                        end              
            default:    begin
                            Jal         = 1'b0;
                            Jalr        = 1'b0;
                            Branch      = 1'b0;
                            MemRead     = 1'b0;
                            MemToReg    = 1'b0;
                            MemWrite    = 1'b0;
                            ALUSrc      = 1'b0;
                            RegWrite    = 1'b0;
                        end 
        endcase
    end        

endmodule