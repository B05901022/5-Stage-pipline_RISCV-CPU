/// Single Cycle RISCV
//=========================================================
// Input/Output Signals:
// positive-edge triggered         clk
// active low synchronous reset   rst_n
// instruction memory interface    IR_addr, IR
// output for testing purposes     RF_writedata  
//=========================================================
// Wire/Reg Specifications:
// control signals             MemToReg, MemRead, MemWrite, 
//                             RegWrite, Branch,
//                             Jalr, Jal, ALUSrc, ALUOp
// ALU control signals         ALUctrl
// ALU output signals          ALUresult, ALUzero
// instruction specifications  r, jal, jalr, lw, sw, beq
// registers input signals     Reg_R1, Reg_R2, Reg_W, WriteData 
// immediate generated signals ImmGenOut
// registers output signals    ReadData1, ReadData2
// data memory contral signals CEN, OEN, WEN
// data memory output signals  ReadDataMem
// Memory address              A
//=========================================================

module SingleCycle_CHIP( 
    clk,
    rst_n,
    IR_addr,
    IR,
    RF_writedata,
    ReadDataMem,
    CEN,
    WEN,
    A,
    ReadData2,
    OEN
);

//==== in/out declaration =================================
    //-------- processor ----------------------------------
    input         clk, rst_n;
    input  [31:0] IR;
    output [31:0] IR_addr, RF_writedata;
    //-------- data memory --------------------------------
    input  [31:0] ReadDataMem;  // read_data from memory
    output        CEN;  // chip_enable, 0 when you read/write data from/to memory
    output        WEN;  // write_enable, 0 when you write data into SRAM & 1 when you read data from SRAM
    output  [6:0] A;  // address
    output [31:0] ReadData2;  // write_data to memory
    output        OEN;  // output_enable, 0

//==== reg/wire declaration ===============================
    // the instrcution rebuild
    wire [31:0] IR_in;

    // the program counter part
    reg [31:0]  pc_r;
    wire [31:0] pc_w;
    wire [31:0] pc_add_4;
    wire [31:0] branch_or_jal_addr;
    wire        branch_or_jal_ctrl;
    wire [31:0] jalr_addr;
    
    // the control unit part
    wire        Jal, Jalr, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite; // the control signal
    Control_unit ctrl_unit(
        .Opcode     ( IR_in[6:0] ),
        .Jal        ( Jal ),
        .Jalr         ( Jalr ),
        .Branch     ( Branch ),
        .MemRead    ( MemRead ),
        .MemToReg   ( MemToReg ),
        .MemWrite   ( MemWrite ),
        .ALUSrc     ( ALUSrc ),
        .RegWrite   ( RegWrite )
    );

    // the register file part
    wire [31:0] busX, busY, busW; // TODO busW...
    register_file reg_file(
        .clk    ( clk ),
        .rst_n  ( rst_n ),
        .WEN    ( RegWrite ), // the control signal from control unit
        .RW     ( IR_in[11:7] ),
        .busW   ( busW ),
        .RX     ( IR_in[19:15] ),
        .RY     ( IR_in[24:20] ),
        .busX   ( busX ),
        .busY   ( busY )
    );

    // the ImmGen part
    wire [31:0] imm;
    ImmGen  ig(
        .IR     ( IR_in[31:0] ),
        .Imm    ( imm )
    );

    // the alu part
    wire [3:0] alu_ctrl;
    wire [31:0] alu_y, alu_result;
    wire alu_zero;
    ALU alu(
        .ctrl   ( alu_ctrl ),
        .x      ( busX ), // the ouput of register file busX
        .y      ( alu_y ), // the mux ouput of imm and busY control by ALUSrc
        .out    ( alu_result),
        .zero   ( alu_zero )
    );
    ALU_control alu_ctrl_unit(
        .Func7      ( IR_in[30] ),
        .Func3      ( IR_in[14:12]),
        .OP         ( IR_in[4:3] ), // control signal from control unit
        .ALUCtrl    ( alu_ctrl )
    );

//==== combinational part =================================
    // the instruction rebuild
    assign IR_in = {IR[7:0], IR[15:8], IR[23:16], IR[31:24] };
    

    // the PC part
    assign IR_addr = pc_r;
    assign pc_add_4 = pc_r + 4;
    assign branch_or_jal_addr = pc_r + imm;
    assign branch_or_jal_ctrl = ((Branch & alu_zero) | Jal );
    assign jalr_addr = imm + busX;
    assign pc_w = ( branch_or_jal_ctrl ) ? branch_or_jal_addr :
                  ( Jalr )               ? jalr_addr          : 
                  pc_add_4; 

    // the register file part...
    assign busW = ( Jalr|Jal ) ? pc_add_4 :
                  ( MemToReg ) ? {ReadDataMem[7:0], ReadDataMem[15:8], ReadDataMem[23:16], ReadDataMem[31:24]} :
                  alu_result;
    assign RF_writedata = busW;

    // the alu part...
    assign alu_y = ( ALUSrc ) ? imm : busY;

    // the data memory part
    assign CEN = ~(MemRead|MemWrite);
    assign WEN = MemRead;
    assign A = alu_result[8:2];
    assign ReadData2 = {busY[7:0], busY[15:8], busY[23:16], busY[31:24]};

    // OEN
    assign OEN = 0; // OEN is always zero in this case


//==== sequential part ====================================
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            pc_r <= 32'b0;
        end
        else begin
            pc_r <= pc_w;
        end
    end

//=========================================================
endmodule

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
    
always@(*) begin
        r_w [0] = r_r[0];
        r_w [1] = r_r[1];
        r_w [2] = r_r[2];
        r_w [3] = r_r[3];
        r_w [4] = r_r[4];
        r_w [5] = r_r[5];
        r_w [6] = r_r[6];
        r_w [7] = r_r[7];
        r_w [8] = r_r[8];
        r_w [9] = r_r[9];
        r_w [10] = r_r[10];
        r_w [11] = r_r[11];
        r_w [12] = r_r[12];
        r_w [13] = r_r[13];
        r_w [14] = r_r[14];
        r_w [15] = r_r[15];
        r_w [16] = r_r[16];
        r_w [17] = r_r[17];
        r_w [18] = r_r[18];
        r_w [19] = r_r[19];
        r_w [20] = r_r[20];
        r_w [21] = r_r[21];
        r_w [22] = r_r[22];
        r_w [23] = r_r[23];
        r_w [24] = r_r[24];
        r_w [25] = r_r[25];
        r_w [26] = r_r[26];
        r_w [27] = r_r[27];
        r_w [28] = r_r[28];
        r_w [29] = r_r[29];
        r_w [30] = r_r[30];
        r_w [31] = r_r[31];
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
        r_w [0] = r_r[0];
        r_w [1] = r_r[1];
        r_w [2] = r_r[2];
        r_w [3] = r_r[3];
        r_w [4] = r_r[4];
        r_w [5] = r_r[5];
        r_w [6] = r_r[6];
        r_w [7] = r_r[7];
        r_w [8] = r_r[8];
        r_w [9] = r_r[9];
        r_w [10] = r_r[10];
        r_w [11] = r_r[11];
        r_w [12] = r_r[12];
        r_w [13] = r_r[13];
        r_w [14] = r_r[14];
        r_w [15] = r_r[15];
        r_w [16] = r_r[16];
        r_w [17] = r_r[17];
        r_w [18] = r_r[18];
        r_w [19] = r_r[19];
        r_w [20] = r_r[20];
        r_w [21] = r_r[21];
        r_w [22] = r_r[22];
        r_w [23] = r_r[23];
        r_w [24] = r_r[24];
        r_w [25] = r_r[25];
        r_w [26] = r_r[26];
        r_w [27] = r_r[27];
        r_w [28] = r_r[28];
        r_w [29] = r_r[29];
        r_w [30] = r_r[30];
        r_w [31] = r_r[31];
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
        r_r [0] <= 32'b0;
        r_r [1] <= 32'b0;
        r_r [2] <= 32'b0;
        r_r [3] <= 32'b0;
        r_r [4] <= 32'b0;
        r_r [5] <= 32'b0;
        r_r [6] <= 32'b0;
        r_r [7] <= 32'b0;
        r_r [8] <= 32'b0;
        r_r [9] <= 32'b0;
        r_r [10] <= 32'b0;
        r_r [11] <= 32'b0;
        r_r [12] <= 32'b0;
        r_r [13] <= 32'b0;
        r_r [14] <= 32'b0;
        r_r [15] <= 32'b0;
        r_r [16] <= 32'b0;
        r_r [17] <= 32'b0;
        r_r [18] <= 32'b0;
        r_r [19] <= 32'b0;
        r_r [20] <= 32'b0;
        r_r [21] <= 32'b0;
        r_r [22] <= 32'b0;
        r_r [23] <= 32'b0;
        r_r [24] <= 32'b0;
        r_r [25] <= 32'b0;
        r_r [26] <= 32'b0;
        r_r [27] <= 32'b0;
        r_r [28] <= 32'b0;
        r_r [29] <= 32'b0;
        r_r [30] <= 32'b0;
        r_r [31] <= 32'b0;
    end
    else begin
        r_r [0] <= 32'b0;
        r_r [1] <= r_w[1];
        r_r [2] <= r_w[2];
        r_r [3] <= r_w[3];
        r_r [4] <= r_w[4];
        r_r [5] <= r_w[5];
        r_r [6] <= r_w[6];
        r_r [7] <= r_w[7];
        r_r [8] <= r_w[8];
        r_r [9] <= r_w[9];
        r_r [10] <= r_w[10];
        r_r [11] <= r_w[11];
        r_r [12] <= r_w[12];
        r_r [13] <= r_w[13];
        r_r [14] <= r_w[14];
        r_r [15] <= r_w[15];
        r_r [16] <= r_w[16];
        r_r [17] <= r_w[17];
        r_r [18] <= r_w[18];
        r_r [19] <= r_w[19];
        r_r [20] <= r_w[20];
        r_r [21] <= r_w[21];
        r_r [22] <= r_w[22];
        r_r [23] <= r_w[23];
        r_r [24] <= r_w[24];
        r_r [25] <= r_w[25];
        r_r [26] <= r_w[26];
        r_r [27] <= r_w[27];
        r_r [28] <= r_w[28];
        r_r [29] <= r_w[29];
        r_r [30] <= r_w[30];
        r_r [31] <= r_w[31];
    end
end	

endmodule

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
    output         zero;    // check the overflow                

    reg [31:0] out;
    wire [32:0] sub_result;

    assign sub_result = x + ~y + 1;
    assign zero = ~(|sub_result[31:0]); // $$$ check if error 

    always@(*) begin
    case ( ctrl )
        4'b0000:    out = x & y;                            // AND
        4'b0001:    out = x | y;                            // OR
        4'b0010:    out = x + y;                            // ADD
        4'b0110:    out = sub_result;                       // SUB
        4'b1000:    out = (sub_result[32]) ? 32'd1 : 32'b0;   // set on less than
        default:    out = 32'b0;
    endcase

    end

endmodule

module ImmGen(
    IR,
    Imm
);
    input   [31:0]  IR;     // instruction from instruction memory
    output  [31:0]  Imm;    
    reg     [31:0]  Imm;

    always@(*) begin
        case (IR[6:2])
            5'b00000:   Imm = {{21{IR[31]}},IR[30:25],IR[24:21],IR[20]};        // lw I-type
            5'b01000:   Imm = {{21{IR[31]}},IR[30:25],IR[11:8],IR[7]};          // sw S-type
            5'b11000:   Imm = {{20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};   // beq B-type
            5'b11011:   Imm = {{12{IR[31]}}, IR[19:12], IR[20], IR[30:25], IR[24:21], 1'b0}; // jal J-type
            5'b11001:   Imm = {{21{IR[31]}},IR[30:25],IR[24:21],IR[20]};        // jalr I-type
            default:    Imm = 32'b0;
        endcase
    end
endmodule

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

    assign ALUCtrl[0] = (OP[1] & Func3[2] & Func3[1] & (~Func3[0]));
    assign ALUCtrl[1] = ~(OP[1]&(~OP[0])&Func3[1]);
    assign ALUCtrl[2] = ( ((~OP[1])&(~Func3[1])) | (Func7 & OP[1]) );
    assign ALUCtrl[3] = ( OP[1] & (~Func3[2]) & Func3[1] );

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
    input   [6:0]   Opcode;
    output  reg     Jal, Jalr, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite;

    always@(*) begin
        case ( Opcode )
            7'b0110011: begin
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
            7'b0000011: begin
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
            7'b0100011: begin
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
            7'b1100011: begin
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
            7'b1101111: begin
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
            7'b1100111: begin
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