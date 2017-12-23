
`default_nettype none
`timescale 1ns/100ps

`define WORD 31:0
`define ADD  6'h00
`define ADDI 6'h08
`define BNE  6'h05
`define LW   6'h23
`define SW   6'h2b
`define HALT 6'h2e
`define NOP  6'h2f
  
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
       //p.imem.mem[0] = {`ADD, 5'd1, 5'd2, 5'd1, 5'd0, 6'h20};
       
       p.imem.mem[0] = {`ADDI, 5'd0, 5'd1, 16'hffff};//L0
       p.imem.mem[1] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[2] = {`BNE, 5'd2, 5'd4, 16'd1};
       p.imem.mem[3] = {`BNE, 5'd0, 5'd12, 16'd17};
       p.imem.mem[4] = {`ADDI, 5'd11, 5'd11, 16'd4};//L1
       p.imem.mem[5] = {`ADDI, 5'd11, 5'd10, 16'hfffc};
       p.imem.mem[6] = {`ADDI, 5'd1, 5'd1, 16'd1};//L2
       p.imem.mem[7] = {`BNE, 5'd1, 5'd5, 16'd1};
       p.imem.mem[8] = {`BNE, 5'd0, 5'd12, 16'hfff7};
       p.imem.mem[9] = {`ADDI, 5'd10, 5'd10, 16'd4};//L3
       p.imem.mem[10] = {`ADD, 5'd1, 5'd2, 5'd7, 5'd0, 6'h20};
       p.imem.mem[11] = {`LW, 5'd10, 5'd8, 16'd0};
       p.imem.mem[12] = {`LW, 5'd11, 5'd9, 16'd0};
       p.imem.mem[13] = {`BNE, 5'd8, 5'd0, 16'd2};
       p.imem.mem[14] = {`ADDI, 5'd3, 5'd3, 16'd1};
       p.imem.mem[15] = {`BNE, 5'd12, 5'd0, 16'hfff6};
       p.imem.mem[16] = {`BNE, 5'd9, 5'd12, 16'd2};//L4
       p.imem.mem[17] = {`ADDI, 5'd3, 5'd3, 16'd2};
       p.imem.mem[18] = {`BNE, 5'd12, 5'd0, 16'hfff3};
       p.imem.mem[19] = {`ADDI, 5'd3, 5'd3, 16'd3};//L5
       p.imem.mem[20] = {`BNE, 5'd12, 5'd0, 16'hfff1};
       p.imem.mem[21] = {`HALT, 26'd0};
	
       /*
       p.imem.mem[0] = {`ADD, 5'd0, 5'd0, 5'd1, 5'd0, 6'h20};
       p.imem.mem[1] = {`ADDI, 5'd1, 5'd1, 16'd1};
       p.imem.mem[2] = {`BNE, 5'd1, 5'd1, 16'hfffe};
       p.imem.mem[3] = {`NOP, 26'd0};
       p.imem.mem[4] = {`BNE, 5'd1, 5'd4, 16'hfffc};
       p.imem.mem[5] = {`HALT, 26'd0};
	*/
       
              
    end

   //initial begin
   //     #2000 $finish();
   // end
   
   integer i;
    initial begin /* initialize the register file */
       p.regfile.r[0] = 0;
       p.regfile.r[1] = -1; //i
       p.regfile.r[2] = -1; //j
       p.regfile.r[3] = 0; //sum
       p.regfile.r[4] = 1001; //1001
       p.regfile.r[5] = 4; //4
       p.regfile.r[6] = 4096; //4096
       p.regfile.r[7] = 0;
       p.regfile.r[8] = 0;
       p.regfile.r[9] = 0;
       p.regfile.r[10] = -4;
       p.regfile.r[11] = -4;
       p.regfile.r[12] = 1;
       for(i=0;i<4096;i++) begin
	     p.dmem.mem[i] = 0;
       end	     
    end
   
    always @(posedge CLK) begin
        $write("%x %x %x %x %x %x: %d %d %d %d %d \n", p.pc, p.Ifop, p.Idop, p.Exop, p.Maop, p.Wbop, p.regfile.r[1], p.regfile.r[2], p.regfile.r[3], p.bne_num, p.miss_num);
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
   reg [31:0] clk_num;
   reg [31:0] bne_num;
   reg [31:0] miss_num;
   
   //----------IF Stage----------//
   wire [31:0] 	Ifir;
   wire [31:0] 	expc;
   wire 	miss;
   wire 	Ifb;
   wire [5:0] 	Ifop = Ifir[31:26]; //for debug

   MEM imem(CLK, pc, 0, 1'b0, Ifir); /* instruction memory */

   GSHARE gshare(CLK, RST_X, pc, Ifir, miss, Ifb, expc);
   
   always @(posedge CLK) begin
      if(!RST_X) pc <= 0;
      else pc <= expc;
   end
   reg [31:0] 	IfId_ir;
   reg 		IfId_b;
   always @(posedge CLK) begin
      IfId_ir <= (!RST_X || miss) ? {`NOP, 26'd0} : Ifir;
      IfId_b <= (!RST_X) ? 0 : Ifb;
      clk_num <= (!RST_X) ? 0 : clk_num+1;
   end
      
   //----------ID Stage----------//
   wire [31:0] 	Idir = IfId_ir;
   wire [4:0] 	Idrs = IfId_ir[25:21];
   wire [4:0] 	Idrt = IfId_ir[20:16];
   wire [4:0] 	Idrd = IfId_ir[15:11];
   wire [5:0] 	Idop = IfId_ir[31:26];
   wire 	Idb = IfId_b;
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
		 31 : (Idop == `ADD) ? Idrd : Idrt; //hold dst or not
      IdEx_ir <= (!RST_X) ? 0 : Idir; //if expect miss then nop
   end
   
   //decide branch or not
   //data forwarding
   wire signed [31:0] binp1 = (Idrs == Exrd) ? alu_result : 
	((Idrs == Mard) && (Maop == `LW)) ? mem_result :
	((Idrs == Mard) && (Maop != `LW)) ? Maalures : 
	(Idrs == Wbdst) ? Wbres : Idrrs;
   wire signed [31:0] binp2 =  (Idrt == Exrd) ? alu_result : 
	((Idrt == Mard) && (Maop == `LW)) ? mem_result :
	((Idrt == Mard) && (Maop != `LW)) ? Maalures : 
	(Idrt == Wbdst) ? Wbres : Idrrt;
   wire 	      pc_src = (Idop == `BNE) && (binp1 != binp2); 
   assign miss = (Idop == `BNE) && (pc_src != Idb); //correct expextion or not
   
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
   wire 	mem_read = (Maop == `LW);	
   wire [31:0] 	 mem_result;
   MEM dmem(CLK, Maalures, writedata, mem_write, mem_result);
   reg [31:0] 	 MaWb_memres, MaWb_alures, MaWb_ir;  
   reg [4:0] 	 MaWb_dst;
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
   wire [4:0] 	  Wbdst = MaWb_dst;
   
   always @(posedge CLK) begin
      if(Wbop == `HALT) $finish();
      miss_num <= (!RST_X) ? 0 : (miss) ? miss_num+1 : miss_num;
      bne_num <= (!RST_X) ? 0 : (Wbop == `BNE) ? bne_num+1 : bne_num;
   end
      
endmodule

/****************************/
module MEM(CLK, ADDR, D_IN, D_WE, D_OUT);
   //read address
   input         CLK;
   input [31:0]   ADDR, D_IN;
   input 	  D_WE;
   output [31:0]  D_OUT;
   
   reg [31:0] 	  mem[1024*8-1:0]; // 8K word memory
   
   assign D_OUT  = mem[ADDR[14:2]];
   always @(posedge CLK) if(D_WE) mem[ADDR[14:2]] <= D_IN;
endmodule

//Gshare
module GSHARE(CLK, RST_X, pc, gir, miss, b, expc);
   input CLK, RST_X;
   input [31:0] pc, gir;
   input 	miss;
   output 	b; //branch or not
   output [31:0] expc;

   reg [1:0] 	 counter[16*1024-1:0]; //4KB
   reg [13:0] 	 history; //14bit
   reg [1:0] 	 pastcounter; //past counter	 
   reg [31:0] 	 pastaddr; //past step addr
   reg 		 pastb; //past step branch or not
   reg [31:0] 	 pastpc; //when miss, use this pc
 		 
   wire [13:0] 	 addr = (miss) ? (pc[13:0]^history)^14'b1 : (pc[13:0]^history);
   wire signed [15:0] gimm = gir[15:0];
   wire signed [31:0] geximm = {{16{gimm[15]}}, gimm};
   wire signed [31:0] gshiftimm = (geximm << 2);
   
   assign b = (gir[31:26] == `BNE) && ((counter[addr]==2'h2) || (counter[addr]==2'h3));
   assign expc = (miss) ? pastpc : (b) ? pc+4+gshiftimm : pc + 4 ;

   integer i;
   always @(posedge CLK) begin
      if(!RST_X) begin
	 for(i = 0; i < 16*1024; i=i+1)begin
	    counter[i] <= 1;
	 end
	 history <= 0;
	 pastaddr <= 0;
	 pastb <= 0;
      end
      if (RST_X && miss) begin
	 counter[pastaddr] = (!pastb) ? {(pastcounter != 2'h0), (pastcounter != 2'h1)}
			     :{(pastcounter == 2'h3), (pastcounter == 2'h2)};
	 history = {history[13:1], !history[0]};
      end
      if(RST_X && (gir[31:26] == `BNE)) begin
	 history =  {history[12:0], (expc != pc+4)};
	 pastcounter = counter[addr];
 	 pastaddr = addr;
	 pastb = b;
	 pastpc = (b) ? pc+4 : pc+4+gshiftimm ;
	 counter[addr] = (b) ? {(counter[addr] != 2'h0), (counter[addr] != 2'h1)}
			 :{(counter[addr] == 2'h3), (counter[addr] == 2'h2)};
      end			  
   end
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

    reg signed [31:0] r[0:31];

    assign DOUT0 = (REGNUM0==0) ? 0 : (REGNUM0==REGNUM2) ? DIN0 : r[REGNUM0];
    assign DOUT1 = (REGNUM1==0) ? 0 : (REGNUM1==REGNUM2) ? DIN0 : r[REGNUM1];

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
