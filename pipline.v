
`default_nettype none
`timescale 1ns/100ps

`define WORD 31:0
`define ADD  6'h00
`define ADDI 6'h08
`define BNE  6'h05
`define LW   6'h23
`define SW   6'h2b

/************************************************/
module top();
    reg CLK, RST_X;

    initial begin
        CLK = 0;
        forever #50 CLK = ~CLK;
    end

    initial begin
        RST_X = 0;
        #300 RST_X = 1;
    end
    
    initial begin /* initialize the instruction memory */
       /*
       p.imem.mem[0] = 32'b 10001100100000010000000000000100; //lw $s1, 4($s4)
       p.imem.mem[1] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[1] = 32'b 00000000001000100000100000100000; //add $s1, $s1, $s2
       p.imem.mem[2] = 32'b 00100000010000100000000000000001; //addi $s2, $s2, 1
       p.imem.mem[3] = 32'b 00010100010000111111111111111101; //bne $s2, $s3, -3
       p.imem.mem[4] = 32'b 10101100100000010000000000000100; //sw 4($4), $s1
       p.imem.mem[5] = 32'b 10001100100001010000000000000100; //lw $s5, 4($s4)
       */
       /*
       p.imem.mem[0] = 32'b 10001100100000010000000000000000; //lw $s1, 0($s4)
       p.imem.mem[1] = 32'b 00000000001001010010100000100000; //add $s5, $s1, $s5
       p.imem.mem[2] = 32'b 00100000100001000000000000000100; //addi $s4, $s4, 4
       p.imem.mem[3] = 32'b 00100000010000100000000000000001; //addi $s2, $s2, 1
       p.imem.mem[4] = 32'b 00010100010000111111111111111011; //bne $s2, $s3, -5
       */
       p.imem.mem[0] = 32'b 10101100100001010000000000000100; //sw 4($4), $s5
       p.imem.mem[1] = 32'b 10001100100000010000000000000100; //lw $s1, 4($s4)
       p.imem.mem[2] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[3] = 32'b 00000000001000100000100000100000; //add $s1 $s1 $s2
       p.imem.mem[4] = 32'b 00100000010000100000000000000001; //addi $s2 $s2 1
       p.imem.mem[5] = 32'b 00010100001000101111111111111110; //bne $s1 $s2 -2
       p.imem.mem[6] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[7] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[8] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[9] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[10] = 32'b 00000000000000000000000000000000; //nop
       /*
       p.imem.mem[0] = 32'b 00000000001000100000100000100000; //add $s1 $s1 $s2
       p.imem.mem[1] = 32'b 00100000010000100000000000000001; //addi $s2 $s2 1
       p.imem.mem[2] = 32'b 00010100001000101111111111111110; //bne $s1 $s2 -2
       p.imem.mem[3] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[4] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[5] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[6] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[7] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[8] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[9] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[10] = 32'b 00000000000000000000000000000000; //nop
       p.imem.mem[11] = 32'b 00000000000000000000000000000000; //nop
	*/
       
    end

   initial begin /* initialize the instruction memory */
      p.dmem.mem[0] = 5;
      p.dmem.mem[1] = 3;
      p.dmem.mem[2] = 20;
    end

    initial begin /* initialize the register file */
       /*
       p.regfile.r[1] = 0; //$s1
       p.regfile.r[2] = 0; //$s2
       p.regfile.r[3] = 3; //number of loop
       p.regfile.r[4] = 0; //base addr
       p.regfile.r[5] = 0;
	*/
       p.regfile.r[0] = 0;
       p.regfile.r[1] = 0;
       p.regfile.r[2] = 3;
       p.regfile.r[3] = 0;
       p.regfile.r[4] = 0;
       p.regfile.r[5] = 5;
       
    end
        
    initial begin
        #6000 $finish();
    end

    always @(posedge CLK) begin
        $write("%x %x: %d %d %d %d %d %d %d\n", p.pc, p.Ifir, p.regfile.r[1], p.regfile.r[2],
	       p.Wbres, p.alu_result, p.reg_write, p.Wbop, p.binp2);
    end

   initial begin
      $dumpfile("wave.vcd");
      $dumpvars(0, p);
   end

    PROCESSOR_01 p(CLK, RST_X);
endmodule

/****************/
module PROCESSOR_01(CLK, RST_X);
   input CLK, RST_X;   
   reg [31:0] pc;

   //----------IF Stage----------//
   wire [31:0] 	Ifir;
   wire [31:0] 	tpc;
   wire [31:0] 	npc;
   
   MEM imem(CLK, pc, 0, 0, Ifir); /* instruction memory */
   always @(posedge CLK) begin
      if(!RST_X) pc <= 0;
      else pc <= (pc_src) ? tpc : npc;
   end
   reg [31:0] 	IfId_ir;
   reg [31:0] 	IfId_npc; 	
   always @(posedge CLK) begin
      IfId_ir <= (!RST_X) ? 0 : Ifir;
      IfId_npc <= (!RST_X) ? 0 : pc + 4;
   end
      
   //----------ID Stage----------//
   wire [31:0]  Idnpc = IfId_npc;
   wire [31:0] 	Idir = IfId_ir;
   wire [4:0] 	Idrs = IfId_ir[25:21];
   wire [4:0] 	Idrt = IfId_ir[20:16];
   wire [4:0] 	Idrd = IfId_ir[15:11];
   wire [5:0] 	Idop = IfId_ir[31:26];
   wire signed [15:0] Idimm = Idir[15:0];
   wire signed [31:0] Ideximm = {{16{Idimm[15]}}, Idimm};
   wire signed [31:0] Idshiftimm = (Ideximm << 2);
   wire 	reg_write = ((Wbop == `ADD) || (Wbop == `ADDI) || (Wbop == `LW));
   
   wire [31:0] 	 Idrrs, Idrrt;
   GPR regfile(CLK, Idrs, Idrt, Wbdst, Wbres, reg_write, Idrrs, Idrrt);

   reg [31:0] 	 IdEx_rrs, IdEx_rrt, IdEx_ir;
   reg [4:0] 	 IdEx_rd;
   
   always @(posedge CLK) begin
      IdEx_rrs <= (!RST_X) ? 0 : Idrrs;
      IdEx_rrt <= (!RST_X) ? 0 : Idrrt;
      IdEx_rd <= (!RST_X) ? 0 : ((Idop == `BNE) ||(Idop == `SW)) ? 
		 0 : (Idop == `ADD) ? Idrd : Idrt; //hold dst or not
      IdEx_ir <= (!RST_X) ? 0 : Idir;
   end
   
   //decide branch or not
   //data forwarding
   wire signed [31:0] binp1 = (Idrs == Exrd) ? alu_result : (Idrs == Mard) ? 
	Maalures : (Idrs == Wbdst) ? Wbres : Idrrs;
   wire signed [31:0] binp2 =  (Idrt == Exrd) ? alu_result : (Idrt == Mard) ? 
	Maalures : (Idrt == Wbdst) ? Wbres : Idrrt;
   wire 	      pc_src = (Idop == `BNE) && (binp1 != binp2);   
   //set npc and tpc
   assign npc = pc + 4;
   assign tpc = pc + Idshiftimm;
   
   //----------Ex Stage----------//
   //set alu inputs
   wire [31:0] Exir = IdEx_ir;
   wire [5:0] Exop = IdEx_ir[31:26];
   wire signed [31:0] Eximm = {{16{Exir[15]}}, Exir[15:0]};
   wire [4:0] 	      Exrs = IdEx_ir[25:21];
   wire [4:0] 	      Exrt = IdEx_ir[20:16];
   //data forwarding
   wire [31:0] 	      Exrrs = (Exrs == Mard) ? Maalures : (Exrs == Wbdst) ? 
		      Wbres : IdEx_rrs;
   wire [31:0] 	      Exrrt = (Exrt == Mard) ? Maalures : (Exrt == Wbdst) ?
		      Wbres : IdEx_rrt;
   wire [4:0] 	      Exrd = IdEx_rd;
   wire 	      alu_src;
   wire [31:0] 	      alu_inp2;
   assign alu_src = ((Exop == `ADDI) || (Exop == `LW) || (Exop == `SW));
   assign alu_inp2 = (alu_src) ? Eximm : Exrrt;
   wire 	 alu_zero;
   wire [31:0] 	 alu_result;
   wire 	 alu_ctl = 0;
   ALU alu(CLK, alu_ctl, Exrrs, alu_inp2, alu_zero, alu_result);
   
   reg [31:0] 	 ExMa_alures, ExMa_rrt, ExMa_ir;
   reg [4:0] 	 ExMa_rd;
   always @(posedge CLK) begin
      ExMa_ir <= (!RST_X) ? 0 : Exir;
      ExMa_alures <= (!RST_X) ? 0 : alu_result;
      ExMa_rrt <= (!RST_X) ? 0 : Exrrt;
      ExMa_rd <= (!RST_X) ? 0 : Exrd;
   end
   
   //----------Ma Stage----------//
   wire [31:0]  Mair = ExMa_ir;
   wire [31:0]  Maalures = ExMa_alures;
   wire [4:0] 	Mard = ExMa_rd;
   wire [31:0] 	writedata = ExMa_rrt;
   wire [5:0] 	Maop = ExMa_ir[31:26];
   wire 	mem_write = (Maop == `SW);
   wire [31:0] 	 mem_result;
   MEM dmem(CLK, Maalures, writedata, mem_write, mem_result);

   reg [31:0] 	 MaWb_memres, MaWb_alures, MaWb_dst, MaWb_ir;
   
   always @(posedge CLK) begin
      MaWb_ir <= (!RST_X) ? 0 : Mair;
      MaWb_memres <= (!RST_X) ? 0 : mem_result;
      MaWb_alures <= (!RST_X) ? 0 : Maalures;
      MaWb_dst <= (!RST_X) ? 0 : Mard;
   end
	     
   //----------Wb Stage----------//
   //set mem_to_reg
   wire [5:0] 	  Wbop = MaWb_ir[31:26];
   wire mem_to_reg = (Wbop == `LW); //if op is LW, get memres
   wire [31:0] 	  Wbres = (mem_to_reg) ? MaWb_memres : MaWb_alures;
   wire [5:0] 	  Wbdst = MaWb_dst;
      
endmodule

/****************************/
module MEM(CLK, ADDR, D_IN, D_WE, D_OUT);
   //read address
    input         CLK;
    input  [31:0] ADDR, D_IN;
    input   [3:0] D_WE;
    output [31:0] D_OUT;
      
    reg [31:0] mem[1024*8-1:0]; // 8K word memory

    assign D_OUT  = mem[ADDR[14:2]];
    always @(posedge CLK) if(D_WE) mem[ADDR[14:2]] <= D_IN;
endmodule

/************
/* 32bitx32 2R/1W General Purpose Registers (Register File)                 
/***********/
module GPR(CLK, REGNUM0, REGNUM1, REGNUM2, DIN0, WE0, DOUT0, DOUT1);
    input             CLK;
    input       [4:0] REGNUM0, REGNUM1, REGNUM2;
    input      [31:0] DIN0;
    input             WE0;
    output     [31:0] DOUT0, DOUT1;

    reg [31:0] r[0:31];

    assign DOUT0 = (REGNUM0==0) ? 0 : r[REGNUM0];
    assign DOUT1 = (REGNUM1==0) ? 0 : r[REGNUM1];

    always @(posedge CLK) if(WE0) r[REGNUM2] <= DIN0;
endmodule
/*************/

module ALU(CLK, ALU_CTL, NUM1, NUM2, ZERO, ALU_RES);
   input CLK;
   input ALU_CTL;
   input [31:0] NUM1, NUM2;
   output 	ZERO;
   output [31:0] 	ALU_RES;

   assign ZERO = (ALU_RES == 0);
   assign ALU_RES = NUM1 + NUM2;
endmodule // ALU
