//always block tb
`timescale 1ns/10ps
`define CYCLE	10
`define HCYCLE	5

module decomp_tb;
    reg  [15:0] orig_instr;
    wire [31:0] decomp_instr;
    
    DecompressionUnit DU1(
        .orig_instr  (orig_instr),
        .decomp_instr(decomp_instr)
        );

   initial begin
       $fsdbDumpfile("Decomp.fsdb");
       $fsdbDumpvars;
   end

    initial begin
        orig_instr = 16'h00_01;
        
        #(`CYCLE);
        orig_instr = 16'h43_48;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_47_25_03 ) $display( "PASS --- C.LW, caution: May have Imm issue!!!(x4 or not)" );
        else begin
            $display( "FAIL --- C.LW" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'hc3_20;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h04_87_20_23 ) $display( "PASS --- C.SW, caution: May have Imm issue!!!(x4 or not)" );
        else begin
            $display( "FAIL --- C.SW" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h00_01;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_00_00_13 ) $display( "PASS --- C.NOP" );
        else begin
            $display( "FAIL --- C.NOP" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h17_fd;
        
        #(`HCYCLE);
        if( decomp_instr == 32'hff_f7_87_93 ) $display( "PASS --- C.ADDI" );
        else begin
            $display( "FAIL --- C.ADDI" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h82_85;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_16_d6_93 ) $display( "PASS --- C.SRLI" );
        else begin
            $display( "FAIL --- C.SRLI" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h84_2d;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h40_b4_54_13 ) $display( "PASS --- C.SRAI" );
        else begin
            $display( "FAIL --- C.SRAI" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h89_85;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_15_f5_93 ) $display( "PASS --- C.ANDI" );
        else begin
            $display( "FAIL --- C.ANDI" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h37_79;
        
        #(`HCYCLE);
        if( decomp_instr == 32'hf5_df_f0_ef ) $display( "PASS --- C.JAL" );
        else begin
            $display( "FAIL --- C.JAL" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'ha8_1d;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h05_c0_00_6f ) $display( "PASS --- C.J" );
        else begin
            $display( "FAIL --- C.J" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'hc1_99;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_05_84_63 ) $display( "PASS --- C.BEQZ" );
        else begin
            $display( "FAIL --- C.BEQZ" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'hf7_e5;
        
        #(`HCYCLE);
        if( decomp_instr == 32'hfc_07_9a_e3 ) $display( "PASS --- C.BNEZ" );
        else begin
            $display( "FAIL --- C.BNEZ" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h04_be;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_f4_94_93 ) $display( "PASS --- C.SLLI" );
        else begin
            $display( "FAIL --- C.SLLI" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h80_82;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_00_80_67 ) $display( "PASS --- C.JR" );
        else begin
            $display( "FAIL --- C.JR" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h98_02;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_08_00_e7 ) $display( "PASS --- C.JALR" );
        else begin
            $display( "FAIL --- C.JALR" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h86_ba;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_07_06_93 ) $display( "PASS --- C.MV" );
        else begin
            $display( "FAIL --- C.MV" );
            $displayh( decomp_instr );
        end

        //==========================================================

        #(`CYCLE);
        orig_instr = 16'h96_b2;
        
        #(`HCYCLE);
        if( decomp_instr == 32'h00_c6_86_b3 ) $display( "PASS --- C.ADD" );
        else begin
            $display( "FAIL --- C.ADD" );
            $displayh( decomp_instr );
        end

        //==========================================================

        // finish tb
        #(`CYCLE) $finish;
    end
endmodule