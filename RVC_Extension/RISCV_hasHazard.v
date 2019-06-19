// Five Stage pilpeline RISCV CPU

module RISCV_Pipeline(
    // control interface
    clk,
    rst_n,
//----------I cache interface-------
    ICACHE_ren      ,
	ICACHE_addr     ,
	ICACHE_stall    ,
	ICACHE_rdata    ,
    ICACHE_pcadd    ,
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
    assign  ICACHE_ren = 1'b1;
	output  [30:0]  ICACHE_addr;
	input           ICACHE_stall;
	input   [31:0]  ICACHE_rdata;
    input           ICACHE_pcadd;
    //----------D cache interface-------
    output          DCACHE_ren;
    output	        DCACHE_wen;
	output  [29:0]  DCACHE_addr;
	output  [31:0]  DCACHE_wdata;
	input           DCACHE_stall;
	input   [31:0]  DCACHE_rdata;     

//==== Pipeline Register Definition =====
    //---- IF/ID ------------------------
    wire [29:0]  IFID_pc_addr_w;
    wire [31:0]  IFID_inst_w; 
    reg  [29:0]  IFID_pc_addr_r;
    reg  [31:0]  IFID_inst_r; 
    reg          IFID_pcadd_r;
    wire         IFID_pcadd_w;
    //---- ID/EX ------------------------
    //WB regwrite, memToreg
    wire regwrite;
    wire memtoreg;
    wire IDEX_RegWrite_w;

    wire IDEX_MemToReg_w;
    reg  IDEX_RegWrite_r;
    reg  IDEX_MemToReg_r;
    //M  jal, jalr, branch, memread, memwrite
    wire jal;
    wire jalr;
    wire branch;
    wire memread;
    wire memwrite;

    wire IDEX_jal_w;
    wire IDEX_jalr_w; 
    wire IDEX_branch_w;
    wire IDEX_MemRead_w;
    wire IDEX_MemWrite_w;
    reg  IDEX_jal_r;
    reg  IDEX_jalr_r; 
    reg  IDEX_branch_r;
    reg  IDEX_MemRead_r;
    reg  IDEX_MemWrite_r;
    //EX aluop, alusrc
    wire [1:0]   aluop;
    wire         alusrc;
    wire [1:0]   IDEX_aluop_w;
    wire         IDEX_alusrc_w;
    reg  [4:0]   IDEX_rs1_r;
    reg  [4:0]   IDEX_rs2_r;
    wire [4:0]   IDEX_rs1_w;
    wire [4:0]   IDEX_rs2_w;
    wire [29:0]  IDEX_pc_addr_w;
    wire [31:0]  IDEX_rdata1_w;
    wire [31:0]  IDEX_rdata2_w;
    reg  [1:0]   IDEX_aluop_r;
    reg          IDEX_alusrc_r;
    reg  [29:0]  IDEX_pc_addr_r;
    reg  [31:0]  IDEX_rdata1_r;
    reg  [31:0]  IDEX_rdata2_r;
    // other data
    wire [31:0]  IDEX_imm_w;
    wire         IDEX_func7_w;
    wire [2:0]   IDEX_func3_w;
    wire [4:0]   IDEX_rd_addr_w;
    reg  [31:0]  IDEX_imm_r;
    reg          IDEX_func7_r;
    reg  [2:0]   IDEX_func3_r;
    reg  [4:0]   IDEX_rd_addr_r;
    reg          IDEX_pcadd_r;
    wire         IDEX_pcadd_w;  

    //---- EX/MEM -----------------------
    //WB regwrite, memToreg
    wire EXMEM_RegWrite_w;
    wire EXMEM_MemToReg_w;
    reg  EXMEM_RegWrite_r;
    reg  EXMEM_MemToReg_r;
    //M  jal, jalr, branch, memread, memwrite
    wire EXMEM_jal_w;
    wire EXMEM_jalr_w; 
    wire EXMEM_branch_w;
    wire EXMEM_MemRead_w;
    wire EXMEM_MemWrite_w;
    reg  EXMEM_jal_r;
    reg  EXMEM_jalr_r; 
    reg  EXMEM_branch_r;
    reg  EXMEM_MemRead_r;
    reg  EXMEM_MemWrite_r;
    // pc addr
    wire [29:0]  EXMEM_jalr_addr_w;
    wire [29:0]  EXMEM_branch_or_jal_addr_w;
    // reg  [29:0]  EXMEM_jalr_addr_r;
    // reg  [29:0]  EXMEM_branch_or_jal_addr_r;
    // alu result
    wire [31:0]  EXMEM_alu_out_w;
    // wire         EXMEM_alu_zero_w;
    wire [31:0]  EXMEM_wdata_w;
    reg  [31:0]  EXMEM_alu_out_r;
    // reg          EXMEM_alu_zero_r;
    reg  [31:0]  EXMEM_wdata_r;
    // rd_addr
    wire [4:0]   EXMEM_rd_addr_w;
    reg  [4:0]   EXMEM_rd_addr_r;
    // for jal, jalr
    wire [31:0]  EXMEM_pc_add4_w;
    reg  [31:0]  EXMEM_pc_add4_r;
 
    wire [3:0]   alu_ctrl;                         

    //---- MEM/WB -----------------------
    //WB regwrite, memToreg
    wire MEMWB_RegWrite_w;
    wire MEMWB_MemToReg_w;
    reg  MEMWB_RegWrite_r;
    reg  MEMWB_MemToReg_r;
    // alu result
    wire [31:0] MEMWB_alu_out_w;
    wire [31:0] MEMWB_rdata_w;
    reg  [31:0] MEMWB_alu_out_r;
    reg  [31:0] MEMWB_rdata_r;
    // rd_addr
    wire [4:0]   MEMWB_rd_addr_w;
    reg  [4:0]   MEMWB_rd_addr_r;
    // for jal, jalr
    wire [31:0]  MEMWB_pc_add4_w;
    reg  [31:0]  MEMWB_pc_add4_r;
    wire         MEMWB_jal_w;
    reg          MEMWB_jal_r;
    wire         MEMWB_jalr_w;
    reg          MEMWB_jalr_r;

    //---- Hazard detection ----------
    wire         hazard_stall;
    wire [1:0]   forward_a_ex; //ex stage
    wire [1:0]   forward_b_ex;
    wire [1:0]   forward_a_id; //id stage
    wire [1:0]   forward_b_id;

    // choose data from forwarding unit 
    wire  [31:0]    forwarding_x;
    wire  [31:0]    forwarding_y; 


//==== PC =============================
    wire [31:0] pc_w_no_hazard;
    wire [31:0] pc_w;
    reg  [31:0] pc_r;


//==== Connect the Circuit ============
    // PC
    wire    branch_flush; //judge if branch, if so, flush the following instrction
    wire    j_flush;
    assign j_flush = ( IDEX_jal_r | IDEX_jalr_r );

    // decide next pc
    wire [31:0] IF_stage_add_4_or_2;
    assign IF_stage_add_4_or_2 = (ICACHE_pcadd) ? 4 : 2;
    assign pc_w_no_hazard = ( IDEX_jal_r | branch_flush ) ? EXMEM_branch_or_jal_addr_w:
            ( IDEX_jalr_r )        ? EXMEM_jalr_addr_w:
            pc_r + IF_stage_add_4_or_2;

    assign pc_w = (hazard_stall)? pc_r: pc_w_no_hazard;
    // ID stage
    wire [31:0] wdata;
    assign wdata =  (MEMWB_jal_r|MEMWB_jalr_r) ? MEMWB_pc_add4_r :
                    (MEMWB_MemToReg_r) ? MEMWB_rdata_r:
                    MEMWB_alu_out_r;

    wire [31:0] busX;
    wire [31:0] busY;
    register_file reg_file(
        .clk        ( clk )                 ,
        .rst_n      ( rst_n )               ,
        .WEN        ( MEMWB_RegWrite_r )    ,
        .RW         ( MEMWB_rd_addr_r )     ,
        .busW       ( wdata )               ,
        .RX         ( IFID_inst_r[19:15] )  ,
        .RY         ( IFID_inst_r[24:20] )  ,
        .busX       ( busX )       ,
        .busY       ( busY )
    );

    // forwarding ( only forward WB stage )
    assign IDEX_rdata1_w = (forward_a_id[0])? wdata          :
                                           busX           ; 
    assign IDEX_rdata2_w = (forward_b_id[0])? wdata          :
                                           busY           ; 
    // check branch at EX stage
    EX_branch br(
        .branch(IDEX_branch_r),
        .func3_0(IDEX_func3_r[0]),
        .x(forwarding_x),
        .y(forwarding_y),
        .jump(branch_flush)
    );
    ImmGen ig(
        .IR( IFID_inst_r[31:0] )            ,
        .Imm( IDEX_imm_w )
    );
    Control_unit ctrl_unit(
        .Opcode     ( IFID_inst_r[6:2] )    ,
        .Jal        ( jal )          ,
        .Jalr       ( jalr )         ,
        .Branch     ( branch )       ,
        .MemRead    ( memread )      ,
        .MemToReg   ( memtoreg )     ,
        .MemWrite   ( memwrite )     ,
        .ALUSrc     ( alusrc )       ,
        .RegWrite   ( regwrite )     ,
        .ALUop      ( aluop )
    );

    // control unit dealing hazard 
    // 0 when flush or bubble 
    assign IDEX_jal_w = ( j_flush | branch_flush | hazard_stall )?  1'b0: jal;
    assign IDEX_jalr_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: jalr;
    assign IDEX_branch_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: branch;
    assign IDEX_MemRead_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: memread;
    assign IDEX_MemToReg_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: memtoreg;
    assign IDEX_MemWrite_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: memwrite;
    assign IDEX_alusrc_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: alusrc;
    assign IDEX_RegWrite_w = ( j_flush | branch_flush | hazard_stall )? 1'b0: regwrite;
    assign IDEX_aluop_w = ( j_flush | branch_flush | hazard_stall )? 2'b0: aluop;


    assign IDEX_pc_addr_w = IFID_pc_addr_r;
    assign IDEX_func3_w   = IFID_inst_r[14:12];
    assign IDEX_func7_w   = IFID_inst_r[30];
    assign IDEX_rd_addr_w = IFID_inst_r[11:7];
    assign IDEX_rs1_w = IFID_inst_r[19:15];
    assign IDEX_rs2_w = IFID_inst_r[24:20];
    assign IDEX_pcadd_w = IFID_pcadd_r;

    // EX stage

    // if jump, flush ex stage
    assign EXMEM_RegWrite_w =  IDEX_RegWrite_r;
    assign EXMEM_MemToReg_w =  IDEX_MemToReg_r;
    assign EXMEM_jal_w      =  IDEX_jal_r;
    assign EXMEM_jalr_w     =  IDEX_jalr_r;
    assign EXMEM_branch_w   =  IDEX_branch_r;
    assign EXMEM_MemRead_w  =  IDEX_MemRead_r;
    assign EXMEM_MemWrite_w =  IDEX_MemWrite_r;

    // assign EXMEM_jalr_addr_w = IDEX_imm_r + forwarding_x; //alu_in_x is forwarding data1, which is the address jalr will go
    assign EXMEM_branch_or_jal_addr_w = IDEX_imm_r + IDEX_pc_addr_r;

    assign EXMEM_jalr_addr_w = (forward_a_ex[1])? IDEX_imm_r + EXMEM_alu_out_r:
                               (forward_a_ex[0])? IDEX_imm_r + wdata          :
                                                  IDEX_imm_r + IDEX_rdata1_r  ;

    wire [31:0] EX_stage_add_4_or_2;
    assign EX_stage_add_4_or_2 = (IDEX_pcadd_r) ? 4 : 2;
    assign EXMEM_wdata_w    = {forwarding_y[7:0], forwarding_y[15:8], forwarding_y[23:16], forwarding_y[31:24] };
    assign EXMEM_rd_addr_w  = IDEX_rd_addr_r;
    assign EXMEM_pc_add4_w  = IDEX_pc_addr_r + EX_stage_add_4_or_2;


    wire  [31:0]    alu_in_y;
    assign forwarding_x = (forward_a_ex[1])  ? EXMEM_alu_out_r:
                          (forward_a_ex[0])  ? wdata          :
                                               IDEX_rdata1_r;
    assign alu_in_y = (IDEX_alusrc_r) ? IDEX_imm_r: forwarding_y;
    assign forwarding_y = (forward_b_ex[1])  ? EXMEM_alu_out_r:
                          (forward_b_ex[0])  ? wdata          :
                                               IDEX_rdata2_r;
                                               
    ALU alu(
        .ctrl   ( alu_ctrl )            ,
        .x      ( forwarding_x )        , 
        .y      ( alu_in_y )            , 
        .out    ( EXMEM_alu_out_w )     
        // .zero   ( EXMEM_alu_zero_w )
    );
    ALU_control alu_control(
        .Func7  ( IDEX_func7_r )        ,
        .Func3  ( IDEX_func3_r )        ,
        .OP     ( IDEX_aluop_r )        ,
        .ALUCtrl( alu_ctrl )
    );

    // MEM stage
    
    assign  MEMWB_RegWrite_w = EXMEM_RegWrite_r;
    assign  MEMWB_MemToReg_w = EXMEM_MemToReg_r;
    assign  MEMWB_alu_out_w  = EXMEM_alu_out_r;
    
    assign  DCACHE_ren  = EXMEM_MemRead_r;
    assign  DCACHE_wen  = EXMEM_MemWrite_r;
    assign  DCACHE_addr = EXMEM_alu_out_r[31:2];
    assign  DCACHE_wdata = EXMEM_wdata_r;
    assign  MEMWB_rdata_w = { DCACHE_rdata[7:0], DCACHE_rdata[15:8], DCACHE_rdata[23:16], DCACHE_rdata[31:24] };
    assign  MEMWB_rd_addr_w = EXMEM_rd_addr_r;
    assign  MEMWB_pc_add4_w = EXMEM_pc_add4_r;
    assign  MEMWB_jal_w = EXMEM_jal_r;
    assign  MEMWB_jalr_w = EXMEM_jalr_r;

    assign  ICACHE_addr = pc_r[31:1];

    assign  IFID_inst_w = (hazard_stall)?         IFID_inst_r  :
                          (branch_flush|j_flush)? 32'h0000007f : // flush control unit (bubble)
    { ICACHE_rdata[7:0], ICACHE_rdata[15:8], ICACHE_rdata[23:16], ICACHE_rdata[31:24] };
  
    assign  IFID_pc_addr_w = pc_r;
    assign  IFID_pcadd_w = ICACHE_pcadd;

    // proc_stall
    wire    proc_stall;
    assign  proc_stall = (DCACHE_stall|ICACHE_stall);

    // dealing hazard problem
    HAZARD_DETECTION_UNIT h(
        .IDEX_RT(IDEX_rd_addr_r),
        .IFID_RS(IFID_inst_r[19:15]),
        .IFID_RT(IFID_inst_r[24:20]),
        .MemRead(IDEX_MemRead_r),
        .stall(hazard_stall)
    );

    // forward data to ID / EX stage
    FORWARDING_UNIT ex(
        .EXMEM_RD(EXMEM_rd_addr_r),
        .IDEX_RS1(IDEX_rs1_r),
        .IDEX_RS2(IDEX_rs2_r),
        .IFID_RS1(IFID_inst_r[19:15]),
        .IFID_RS2(IFID_inst_r[24:20]),
        .MEMWB_RD(MEMWB_rd_addr_r),
        .EXMEM_RegWrite(EXMEM_RegWrite_r),
        .MEMWB_RegWrite(MEMWB_RegWrite_r),
        .FORWARD_A_ex(forward_a_ex),
        .FORWARD_B_ex(forward_b_ex),
        .FORWARD_A_id(forward_a_id),
        .FORWARD_B_id(forward_b_id)
    );

    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            // reset the circuit
			pc_r 						<= 0;
            IFID_pc_addr_r              <= 0;
            IFID_inst_r                 <= 32'h00000013; //nop;
            IDEX_RegWrite_r             <= 0;
            IDEX_MemToReg_r             <= 0;
            IDEX_jal_r                  <= 0;
            IDEX_jalr_r                 <= 0;
            IDEX_branch_r               <= 0;
            IDEX_MemRead_r              <= 0;
            IDEX_MemWrite_r             <= 0;
            IDEX_aluop_r                <= 0;
            IDEX_alusrc_r               <= 0;
            IDEX_pc_addr_r              <= 0;
            IDEX_rs1_r                  <= 0;
            IDEX_rs2_r                  <= 0;
            IDEX_rdata1_r               <= 0;
            IDEX_rdata2_r               <= 0;
            IDEX_imm_r                  <= 0;
            IDEX_func7_r                <= 0;
            IDEX_func3_r                <= 0;
            IDEX_rd_addr_r              <= 0;
            EXMEM_RegWrite_r            <= 0;
            EXMEM_MemToReg_r            <= 0;
            EXMEM_jal_r                 <= 0;
            EXMEM_jalr_r                <= 0;
            EXMEM_branch_r              <= 0;
            EXMEM_MemRead_r             <= 0;
            EXMEM_MemWrite_r            <= 0;
            // EXMEM_jalr_addr_r           <= 0;
            // EXMEM_branch_or_jal_addr_r  <= 0;
            EXMEM_alu_out_r             <= 0;
            // EXMEM_alu_zero_r            <= 0;
            EXMEM_wdata_r               <= 0;
            EXMEM_rd_addr_r             <= 0;
            EXMEM_pc_add4_r             <= 0;
            MEMWB_RegWrite_r            <= 0;
            MEMWB_MemToReg_r            <= 0;
            MEMWB_alu_out_r             <= 0;
            MEMWB_rdata_r               <= 0;
            MEMWB_rd_addr_r             <= 0;
            MEMWB_pc_add4_r             <= 0;
            MEMWB_jal_r                 <= 0;
            MEMWB_jalr_r                <= 0;
        end
        else begin
            if(proc_stall) begin
				pc_r					<= pc_r;
                IFID_pc_addr_r          <= IFID_pc_addr_r;
                IFID_inst_r             <= IFID_inst_r;
                IFID_pcadd_r            <= IFID_pcadd_r;
                IDEX_RegWrite_r         <= IDEX_RegWrite_r;
                IDEX_MemToReg_r         <= IDEX_MemToReg_r;
                IDEX_jal_r              <= IDEX_jal_r;
                IDEX_jalr_r             <= IDEX_jalr_r;
                IDEX_branch_r           <= IDEX_branch_r;
                IDEX_MemRead_r          <= IDEX_MemRead_r;
                IDEX_MemWrite_r         <= IDEX_MemWrite_r;
                IDEX_aluop_r            <= IDEX_aluop_r;
                IDEX_alusrc_r           <= IDEX_alusrc_r;
                IDEX_pc_addr_r          <= IDEX_pc_addr_r;
                IDEX_rs1_r              <= IDEX_rs1_r;
                IDEX_rs2_r              <= IDEX_rs2_r;
                IDEX_rdata1_r           <= IDEX_rdata1_r;
                IDEX_rdata2_r           <= IDEX_rdata2_r;
                IDEX_imm_r              <= IDEX_imm_r;
                IDEX_func7_r            <= IDEX_func7_r;
                IDEX_func3_r            <= IDEX_func3_r;
                IDEX_rd_addr_r          <= IDEX_rd_addr_r;
                IDEX_pcadd_r            <= IDEX_pcadd_r;
                EXMEM_RegWrite_r        <= EXMEM_RegWrite_r;
                EXMEM_MemToReg_r        <= EXMEM_MemToReg_r;
                EXMEM_jal_r             <= EXMEM_jal_r;
                EXMEM_jalr_r            <= EXMEM_jalr_r;
                EXMEM_branch_r          <= EXMEM_branch_r;
                EXMEM_MemRead_r         <= EXMEM_MemRead_r;
                EXMEM_MemWrite_r        <= EXMEM_MemWrite_r;
                // EXMEM_jalr_addr_r       <= EXMEM_jalr_addr_r;
                // EXMEM_branch_or_jal_addr_r <= EXMEM_branch_or_jal_addr_r;
                EXMEM_alu_out_r         <= EXMEM_alu_out_r;
                // EXMEM_alu_zero_r        <= EXMEM_alu_zero_r;
                EXMEM_wdata_r           <= EXMEM_wdata_r;
                EXMEM_rd_addr_r         <= EXMEM_rd_addr_r;
                EXMEM_pc_add4_r         <= EXMEM_pc_add4_r;
                MEMWB_RegWrite_r        <= MEMWB_RegWrite_r;
                MEMWB_MemToReg_r        <= MEMWB_MemToReg_r;
                MEMWB_alu_out_r         <= MEMWB_alu_out_r;
                MEMWB_rdata_r           <= MEMWB_rdata_r;
                MEMWB_rd_addr_r         <= MEMWB_rd_addr_r;
                MEMWB_pc_add4_r         <= MEMWB_pc_add4_r;
                MEMWB_jal_r             <= MEMWB_jal_r;
                MEMWB_jalr_r            <= MEMWB_jalr_r;
            end
            else begin
				pc_r					<= pc_w;
                IFID_pc_addr_r          <= IFID_pc_addr_w;
                IFID_inst_r             <= IFID_inst_w;
                IFID_pcadd_r            <= IFID_pcadd_w;
                IDEX_RegWrite_r         <= IDEX_RegWrite_w;
                IDEX_MemToReg_r         <= IDEX_MemToReg_w;
                IDEX_jal_r              <= IDEX_jal_w;
                IDEX_jalr_r             <= IDEX_jalr_w;
                IDEX_branch_r           <= IDEX_branch_w;
                IDEX_MemRead_r          <= IDEX_MemRead_w;
                IDEX_MemWrite_r         <= IDEX_MemWrite_w;
                IDEX_aluop_r            <= IDEX_aluop_w;
                IDEX_alusrc_r           <= IDEX_alusrc_w;
                IDEX_pc_addr_r          <= IDEX_pc_addr_w;
                IDEX_rs2_r              <= IDEX_rs2_w;
                IDEX_rs1_r              <= IDEX_rs1_w;
                IDEX_rdata1_r           <= IDEX_rdata1_w;
                IDEX_rdata2_r           <= IDEX_rdata2_w;
                IDEX_imm_r              <= IDEX_imm_w;
                IDEX_func7_r            <= IDEX_func7_w;
                IDEX_func3_r            <= IDEX_func3_w;
                IDEX_rd_addr_r          <= IDEX_rd_addr_w;
                IDEX_pcadd_r            <= IDEX_pcadd_w;
                EXMEM_RegWrite_r        <= EXMEM_RegWrite_w;
                EXMEM_MemToReg_r        <= EXMEM_MemToReg_w;
                EXMEM_jal_r             <= EXMEM_jal_w;
                EXMEM_jalr_r            <= EXMEM_jalr_w;
                EXMEM_branch_r          <= EXMEM_branch_w;
                EXMEM_MemRead_r         <= EXMEM_MemRead_w;
                EXMEM_MemWrite_r        <= EXMEM_MemWrite_w;
                // EXMEM_jalr_addr_r       <= EXMEM_jalr_addr_w;
                // EXMEM_branch_or_jal_addr_r  <= EXMEM_branch_or_jal_addr_w;
                EXMEM_alu_out_r             <= EXMEM_alu_out_w;
                // EXMEM_alu_zero_r            <= EXMEM_alu_zero_w;
                EXMEM_wdata_r               <= EXMEM_wdata_w;
                EXMEM_rd_addr_r             <= EXMEM_rd_addr_w;
                EXMEM_pc_add4_r             <= EXMEM_pc_add4_w;
                MEMWB_RegWrite_r            <= MEMWB_RegWrite_w;
                MEMWB_MemToReg_r            <= MEMWB_MemToReg_w;
                MEMWB_alu_out_r             <= MEMWB_alu_out_w;
                MEMWB_rdata_r               <= MEMWB_rdata_w;
                MEMWB_rd_addr_r             <= MEMWB_rd_addr_w;
                MEMWB_pc_add4_r             <= MEMWB_pc_add4_w;
                MEMWB_jal_r                 <= MEMWB_jal_w;
                MEMWB_jalr_r                <= MEMWB_jalr_w;
            end
        end
    end
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
    out
    // zero
);
    input   [3:0]  ctrl;    // the contral signal which decide the operation of alu
    input   [31:0] x, y;    // 32 bit input data from register file or instruction[15:0]
    output  [31:0] out;     // 32 bit output
    // output         zero;    // input for branch control              

    reg [31:0] out;
    wire [4:0] shamt; // shift amount unsigned 5-bit

    assign shamt = y[4:0];

    always@(*) begin
    case ( ctrl )
        4'b0000: begin out = x + y; end                         //ADD
        4'b1000: begin out = {x[31], x} + ~{y[31],y} + 1; end   //SUB
        4'b0010: begin
            out = ({x[31], x} + ~{y[31],y} + 1) >>31;           //SLT
        end                                                          
        4'b0100: begin out = x ^ y; end                         //XOR
        4'b0110: begin out = x | y; end                         //OR
        4'b0111: begin out = x & y; end                         //AND
        4'b0001: begin out = x << shamt; end                    //SLL
        4'b0101: begin out = x >> shamt; end                    //SRL
        4'b1101: begin out = $signed(x) >>> shamt; end          //SRA
        default: begin
            out = 32'b0;
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
            5'b00100:   Imm = { {20{IR[31]}},IR[31:20]};                                     // I-type
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
                // if(Func3[0]) begin
                //     //BNE
                //     ALUCtrl = 4'b0011;
                // end
                // else begin
                //     //BEQ
                //     ALUCtrl = 4'b1011;
                // end
                ALUCtrl = 4'b0011;
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

//========================
//=    Control Unit      =
//========================
module Control_unit(
    Opcode,
    Jal,
    Jalr,
    Branch,
    MemRead,
    MemToReg,
    MemWrite,
    ALUSrc,
    RegWrite,
    ALUop
);
    input   [4:0]   Opcode;
    output  reg     Jal, Jalr, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite;
    output  reg [1:0] ALUop;
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
                            ALUop       = 2'b10;
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
                            ALUop       = 2'b00;
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
                            ALUop       = 2'b00;
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
                            ALUop       = 2'b01;
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
                            ALUop       = 2'b00;
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
                            ALUop       = 2'b00;
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
                            ALUop       = 2'b11;
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
                            ALUop       = 2'b00;
                        end 
        endcase
    end        

endmodule

module EX_branch(
    branch,
    func3_0,
    x,
    y,
    jump
);

input branch;
input func3_0;
input [31:0] x, y;
output jump; //if the condition is satisfy

wire beq, bne;

assign bne = (|(x^y));
assign beq = ~bne;
assign jump = (func3_0)? bne && branch: beq && branch;

endmodule